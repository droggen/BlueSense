/*
	File: init
	
	Initialisation and deinitialisation functions.
	
*/
#include "cpu.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>

#if BOOTLOADER==0
//#include "adc.h"
#include "lcd.h"
#include "fb.h"
#include "serial.h"
#include "main.h"
#include "spi.h"
#include "init.h"
#include "system.h"
#include "i2c.h"
#include "wait.h"
#include "ltc2942.h"
//#include "i2c.h"
#include "ds3232.h"
#include "rn41.h"
#include "ufat.h"
#include "interface.h"
#include "mpu.h"
#include "spi-usart0.h"
#include "uiconfig.h"
#endif

unsigned char init_ddra;
unsigned char init_porta;
unsigned char init_ddrb;
unsigned char init_portb;


// Disable I2C if the I2C interface has a hardware issue preventing booting.
#define DISABLE_I2C		0

/******************************************************************************
   function: init_basic
*******************************************************************************
	Low-level initialisation
	
	
	Parameters:
		
******************************************************************************/
void init_basic(void)
{
	//_delay_ms(2000);

	// INIT MODULE
	init_module();
	
	
	//system_led_on(1);
	//_delay_ms(2000);
	
	// LED Test
	system_led_test();	
	
	
	system_led_set(0b000);
	
	//_delay_ms(2000);
		
	// I2C INITIALISATION
	#if !DISABLE_I2C
		i2c_init();	
	#endif
	
	// Open file_usb
	#if HWVER==1
		// Initialize the streams
		//file_usb = fdevopen(uart0_fputchar_int,uart0_fgetchar_int);		
		//file_dbg=fdevopen(dbg_fputchar,0);
		#error: HWVER1 is unsupported
	#endif
	#if (HWVER==4) || (HWVER==5) || (HWVER==6) || (HWVER==7)
		file_usb = serial_open(10,1);
	#endif
	
	// Set IO to non-blocking
	serial_setblocking(file_usb,0);
	
	/////////////////////////// PRINT WORKS FROM HERE on USB ///////////////////////////
	
	// Open Bluetooth communication
	file_bt=serial_open(1,1);
	// Set to non-blocking
	serial_setblocking(file_bt,0);
	
	
	interface_init();
	interface_changedetectenable(1);
	
	interface_signalchange(system_isbtconnected(),system_isusbconnected());
	//interface_test();
	
	fprintf_P(file_usb,PSTR("BlueSense2\n"));
	
	//init_powerreduce();
}
void init_extended(void)
{
	// Get number of boots
	unsigned long numboot = eeprom_read_dword((uint32_t*)STATUS_ADDR_NUMBOOT0);
	fprintf_P(file_pri,PSTR("Boot %lu\n"),numboot);
	eeprom_write_dword((uint32_t*)STATUS_ADDR_NUMBOOT0,numboot+1);
	


	// Fuel gauge initialisation
	// Get the charge
	#if !DISABLE_I2C
		// Read current data
		_poweruse_off.oncharge = ltc2942_getcharge();
		_poweruse_off.onvoltage = ltc2942_getvoltage();
		ds3232_readdatetime_conv_int(0,&_poweruse_off.onh,&_poweruse_off.onm,&_poweruse_off.ons,&_poweruse_off.onday,&_poweruse_off.onmonth,&_poweruse_off.onyear);

		// Read data stored during last soft-off
		_poweruse_off.valid = eeprom_read_byte((uint8_t*)STATUS_ADDR_OFFCURRENT_VALID);
		_poweruse_off.offcharge = eeprom_read_dword((uint32_t*)STATUS_ADDR_OFFCURRENT_CHARGE0);
		_poweruse_off.offvoltage = eeprom_read_word((uint16_t*)STATUS_ADDR_OFFCURRENT_VOLTAGE0);
		_poweruse_off.offh=eeprom_read_byte((uint8_t*)STATUS_ADDR_OFFCURRENT_H);
		_poweruse_off.offm=eeprom_read_byte((uint8_t*)STATUS_ADDR_OFFCURRENT_M);
		_poweruse_off.offs=eeprom_read_byte((uint8_t*)STATUS_ADDR_OFFCURRENT_S);
		_poweruse_off.offday=eeprom_read_byte((uint8_t*)STATUS_ADDR_OFFCURRENT_DAY);
		_poweruse_off.offmonth=eeprom_read_byte((uint8_t*)STATUS_ADDR_OFFCURRENT_MONTH);
		_poweruse_off.offyear=eeprom_read_byte((uint8_t*)STATUS_ADDR_OFFCURRENT_YEAR);
		
		// Mark the data as now invalid
		eeprom_write_byte((uint8_t*)STATUS_ADDR_OFFCURRENT_VALID,0);
		
		// Render info
		for(unsigned char i=0;i<5;i++)
			_poweruse_off.str[0][0]=0;
		if(_poweruse_off.valid)
		{
			sprintf(_poweruse_off.str[0],"Soft off at: %02d.%02d.%02d %02d:%02d:%02d Q: %lu V: %u\n",_poweruse_off.offday,_poweruse_off.offmonth,_poweruse_off.offyear,_poweruse_off.offh,_poweruse_off.offm,_poweruse_off.offs,_poweruse_off.offcharge,_poweruse_off.offvoltage);
			sprintf(_poweruse_off.str[1],"Power on at: %02d.%02d.%02d %02d:%02d:%02d Q: %lu V: %u\n",_poweruse_off.onday,_poweruse_off.onmonth,_poweruse_off.onyear,_poweruse_off.onh,_poweruse_off.onm,_poweruse_off.ons,_poweruse_off.oncharge,_poweruse_off.onvoltage);
			
			// Compute delta-T in seconds; does not work if the measurement is across one month
			unsigned long toff = _poweruse_off.offday*86400l + _poweruse_off.offh * 3600l + _poweruse_off.offm * 60l + _poweruse_off.offs;
			unsigned long ton = _poweruse_off.onday*86400l + _poweruse_off.onh * 3600l + _poweruse_off.onm * 60l + _poweruse_off.ons;
			
			sprintf(_poweruse_off.str[2],"Delta T: %lus\n",ton-toff);
			
			// Compute power
			signed short pwr1 = ltc2942_getavgpower(_poweruse_off.offcharge,_poweruse_off.oncharge,_poweruse_off.onvoltage,(ton-toff)*1000l);
			signed long pwr2 = ltc2942_getavgpower2(_poweruse_off.offcharge,_poweruse_off.oncharge,_poweruse_off.offvoltage,_poweruse_off.onvoltage,ton-toff);
			sprintf(_poweruse_off.str[3],"pwr1: %d mW pwr2: %ld uW\n",pwr1,pwr2);
		}
		else
			sprintf(_poweruse_off.str[1],"No off-power data\n");
		// Display info		
		for(unsigned char i=0;i<5;i++)
			fputs(_poweruse_off.str[i],file_pri);
		
		ltc2942_init();
		
	#endif

	
	
	
	
	
	
	//_delay_ms(1000);
	//fprintf_P(file_usb,PSTR("sizeof long long, long, __int24, int, short, char: %d %d %d %d %d\n"),sizeof(long long),sizeof(long),sizeof(__int24),sizeof(int),sizeof(short),sizeof(char));
	//fprintf_P(file_bt,PSTR("sizeof long long, long, __int24, int, short, char: %d %d %d %d %d\n"),sizeof(long long),sizeof(long),sizeof(__int24),sizeof(int),sizeof(short),sizeof(char));
	
	//unsigned short endiantest = 0x1234;
	
	//fprintf_P(file_usb,PSTR("Endian test. Value: %04x. Value at address 0: %02x; address 1: %02x\n"),endiantest,*((unsigned char *)(&endiantest)+0),*((unsigned char *)(&endiantest)+1));
	



		
	
	// Hook bootloader detector
	cli(); 
	dbg_rx_callback = bootloaderhook_dbg; 					// Hook the bootloader detector
	//uart1_rx_callback = bootloaderhook_bluetooth; 		// Hook the bootloader detector
	sei();															
	
	
	//boottest();
	
	
	
	
	/*while(1)
	{
		unsigned char tt=0;
		if(system_isbtconnected())
			tt+=2;
		if(system_isusbconnected())
			tt+=1;
		system_led_set(tt);
	}*/
	/*//while(1);
	while(!system_isbtconnected());
	system_led_set(0b010);
	while(1)
	{
		unsigned long cc = ltc2942_getcharge();
		fprintf(file_bt,"Hello %ld. tot tx: %ld tot rx: %ld\n",cc,dbg_tot_tx,dbg_tot_rx);
		fprintf(file_usb,"Hello %ld\n",cc);
		_delay_ms(100);
	}*/
	
	//stk500();
	
	//system_status_ok(8);
	
	// BLUETOOTH INITIALISATION
	//cli(); uart1_rx_callback = 0; sei();															// Deactivate callback
	//cli(); uart1_rx_callback = echo_uart1_rx_callback; sei();					// Activate callback
	rn41_Setup(file_usb,file_bt,system_devname);
	bluetoothrts = (PIND&0x10)?1:0;
	//cli(); uart1_rx_callback = echo_uart1_rx_callback; sei();					// Activate callback
	cli(); uart1_rx_callback = 0; sei();															// Deactivate callback
	
	
	#if HWVER==4
		// Here detect connected interfaces, requires to sample the battery in HW4
		for(unsigned char i=0;i<5;i++)
		{
			//system_status_ok2(1);
			printf("Before initial bat ADCSRA %02x\n",ADCSRA);
			unsigned short badc=ADCRead(7,1);
			system_battery = Battery_ADC2mv(badc);
			printf("After initial bat ADCSRA %02x\n",ADCSRA);
			printf("System battery: %d. adc read: %d\n",system_battery,badc);
		}
		//system_status_ok2(2);
		_delay_ms(100);	// Wait for text to be sent before interface update
	#endif
	
	
	

	//while(!system_isbtconnected());
	
	//i2c_forcestop();

	//system_status_ok(6);
	
	// LCD
	/*fprintf_P(file_pri,PSTR("Initialise LCD...\n"));
	//fprintf_P(file_usb,PSTR("Initialise LCD...\n"));
	//fprintf_P(file_bt,PSTR("Initialise LCD...\n"));
	system_enable_lcd = ConfigLoadEnableLCD();
	init_lcd();
	//fprintf_P(file_usb,PSTR("done\n"));
	fprintf_P(file_pri,PSTR("done\n"));*/
	
	/*_delay_ms(250);	
	system_status_ok(2);
	_delay_ms(250);*/
	
	//system_blink(15,50,0b01);
	
	//_delay_ms(500);
	
	/*_delay_ms(250);	
	system_status_ok(2);
	_delay_ms(250);*/
	
	// RTC INITIALISATION
	#if !DISABLE_I2C
		ds3232_init();	
		ds3232_printreg(file_pri);
	#endif
	
	// Initialise current time from RTC
	#if !DISABLE_I2C
		system_settimefromrtc();
	#else
		timer_init(0);
	#endif
	
	
	//_delay_ms(500);
	
	//system_status_ok(5);
	
	//system_status_ok(3);
	
	/*I2C_TRANSACTION tx;
	i2c_transaction_setup(&tx,DBG_ADDRESS,I2C_WRITE,1,0);
	while(1)
	{
		fprintf(file_usb,"usb\n");
		fprintf(file_bt,"bt. is ubs conn: %d\n",system_isusbconnected());
        fprintf(file_bt,"usb level tx: %d rx: %d\n",buffer_level(&_dbg_rx_state),buffer_level(&_dbg_tx_state));
		ds3232_printreg(file_bt);
		fprintf(file_bt,"time: %lu\n",timer_ms_get());
		_delay_ms(500);
		
		tx.dodata = 4;
		strcpy((char*)tx.data,"USB\n");
		unsigned char r = i2c_transaction_queue(1,1,&tx);
		if(r)
		{
			fprintf(file_bt,"could not queue\n");
		}	
		else
		{
			fprintf(file_bt,"tx status: %d i2cerror: %d\n",tx.status,tx.i2cerror);
		}
		
		
	}
	*/
	
	//b0func();
	
	

	
	// CONFIGURATION
	

	//system_status_ok(4);
	
	/*while(1)
	{
		_delay_ms(127);
		fprintf(file_fb,"Time: %ld\n",timer_ms_get());
		lcd_sendframe();
	}
	
	while(1);*/
	
	/*unsigned char r;
	r = ds3232_writetime(12,12,12);
	printf("Write time: %02Xh\n",r);
	r = ds3232_writedate(4,5,6,7);
	printf("Write date: %02Xh\n",r);*/
	
	
	
	
	
	// MPU INITIALISATION
	fprintf_P(file_pri,PSTR("Initialise MPU\n"));
	
	// No interrupt handling routine setup
	//isr_motionint = 0;	
	// Init the MPU
	mpu_init();
	
	
	//system_status_ok(3);
	
	// Estimate baseline performance (optional)
	//fprintf_P(file_pri,PSTR("Estimating baseline performance... "));		
	//system_perf = main_perfbench();
	//fprintf_P(file_pri,PSTR("%ld\n"),system_perf);


	//system_status_ok(2);

	// Register the battery sample callback, and initiate an immediate read of the battery 
	#if !DISABLE_I2C
		timer_register_slowcallback(ltc2942_backgroundgetstate,9);		// Every 10 seconds
		ltc2942_backgroundgetstate(0);
		
		ds3232_readtemp((signed short*)&system_temperature);

		
		
		//timer_register_slowcallback(system_sampletemperature,6);
		
		
	#endif
	//timer_register_callback(system_batterystat,99);		// Battery status at 10Hz
	timer_register_50hzcallback(system_batterystat,4);		// Battery status at 10Hz
	timer_register_slowcallback(system_lifesign,0);		// Low speed blinking
	//timer_register_callback(system_lifesign2,99);		// High speed blinking
	

	system_status_ok(1);
	
	
	ufat_init();
	
	
/*	lcd_writechar('A',0,0,1,0b0000000000000000,0b1111111111111111);
	lcd_writechar('B',1,7,1,0b0000000000000000,0b1111111111111111);
	lcd_writechar('C',2,14,1,0b0000000000000000,0b1111111111111111);
		
	lcd_writechar('D',10,10,2,0b0000000000000000,0b1111111111111111);
	
	lcd_writechar('E',20,20,3,0b0000000000000000,0b1111111111111111);
	
	lcd_writechar('F',50,50,5,0b1111100000000000,0b0000011111100000);
	
	lcd_writechar('G',90,90,4,0b0000000000000000,0b1111111111111111);
	
	lcd_writestring("Hello",0,100,2,0b0000000000000000,0b1111111111111111);*/

	//demo_1();
	//demo_clock();	
	//demo_acc();
	//demo_gyr();
	
	
	
	
	
	//_delay_ms(100);
	//system_blink(20,20,0b01);
	//_delay_ms(100);
	
	
	
	/*while(1)
	{
		//fprintf_P(file_usb,PSTR("Bat: %d time: %ld\n"),system_getbattery(),system_getbatterytime());
		
		fprintf_P(file_usb,PSTR("pwren#: %d chrg: %d pb: %d\n"),(PIND>>6)&1,(PINC>>2)&1,(PINC>>4)&1);
		fprintf_P(file_bt,PSTR("pwren#: %d chrg: %d pb: %d\n"),(PIND>>6)&1,(PINC>>2)&1,(PINC>>4)&1);
		
		_delay_ms(200);
	}*/
	
	
	/*for(unsigned i=0;i<200;i++)
	{
		printf("rtc: %lu cpu: %lu\n",rtc_time_sec,cpu_time);
		_delay_ms(20);
	}*/
	
	
	//mode_sd();

	//printf_P(PSTR("HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH"));
	//while(1)
	/*{
		
		printf("DDRA: %02X\n",DDRA);
		printf("PORTA: %02X\n",PORTA);
		printf("PINA: %02X\n",PINA);
		//_delay_ms(500);
	}*/
	
	
	/*while(1)
	{
		timer_printcallbacks(file_usb);
		_delay_ms(500);
	}*/
	
	//system_led_set(0b01);
	
	//CommandParserMPUTest_Quaternion("",0);
	
	
	
	// Load and set the boot script
	char buf[CONFIG_ADDR_SCRIPTLEN];
	ConfigLoadScript(buf);
	CommandSet(buf,strlen(buf));
	
	
	
}

#if HWVER==1
void init_ports(void)
{
	
	//DDxn, PORTxn, and PINx
	// DDxn =1 : output
	
	DDRA  = 0b00111000;
	PORTA = 0b01100111;
	
	
	DDRB  = 0b10110011;
	PORTB = 0b00011111;
	
	
	
	DDRC  = 0b10001000;
	PORTC = 0b00010000;	        
	
		
	DDRD  = 0b01101010;
	PORTD = 0b00000001;	// What it should be
	//PORTD = 0b01000001;		// Arm factory reset
	
	// Interrupt on change on PA6 (RTC int)
	// PA6 is PCINT6
	PCMSK0 = 0b01000000;			// Mask to select interrupt PA6
		
	// Interrupt on change on PC5 (Motion int)
	// PC5 is PCINT21
	PCMSK2 = 0b00100000;			// Mask to select interrupt PC5
	
	// Enable PCIE0 and PCIE2
	PCICR = 0b00000101;				// Enable interrupt on port A and C
}
#endif

#if (HWVER==4) || (HWVER==5) || (HWVER==6) || (HWVER==7)
void init_ports(void)
{
	
	//DDxn, PORTxn, and PINx
	//DDxn =1 : output
	
	
	
	/*PORTC=0b10000000;		// Power stays on (ddrc still input but with pull up)
	//_delay_us(10);
	DDRC  = 0b11001000;		// put DDRC as output
	
	//while(1);
	
	_delay_us(10);*/
	
	
	/*
		On HW8.28 PC7 drops between DDRB=init_ddrb and PORTB=init_portb
		Attempt to identify which pin triggers the drop: PB0
	*/
	
	init_ddrb = 0b10110011;
	init_portb = 0b10111111;
	
	//DDRB  = init_ddrb;		// Initial
	/*
		DDRB = 0b10110011;
		__builtin_avr_nops(10);
		PORTB = ...
		Duration: 1200ns
		
		2x DDRB: 1250ns
		3x DDRB: 1350ns
		4x DDRB: 1425ns
		5x DDRB: 1525ns
		
		~100nS per DDRB
	*/
	/*DDRB  = 0b10110011;
	DDRB  = 0b10110011;
	DDRB  = 0b10110011;
	DDRB  = 0b10110011;
	DDRB  = 0b10110011;*/
	
	/* 
		DDRB=....
		DDRB=.... bit by bit
		__builtin_avr_nops(10);
		PORTB = ...
		Drop lasts: 1800ns
	*/
	/*DDRB  = 0b00000001;
	DDRB  = 0b00000011;
	DDRB  = 0b00010011;
	DDRB  = 0b00110011;
	DDRB  = 0b10110011;*/
	
	/*
		With this:
		DDRB  = 0b10000000;
		DDRB  = 0b10100000;
		DDRB  = 0b10110000;
		DDRB  = 0b10110010;
		DDRB  = 0b10110011;
		duration: 1200ns
	*/
	
	//DDRB  = 0b10000000;			// 1 bit ok - does not trigger.
	//DDRB  = 0b00100000;			// 1 bit ok - does not trigger
	//DDRB  = 0b10100000;				// 2 bits ok - does not trigger
	//DDRB  = 0b10110000;				// 3 bits ok - does not trigger
	//DDRB  = 0b10110010;				// 4 bits ok - does not trigger
	//DDRB  = 0b10110011;				// Triggers
	//DDRB  = 0b00000001;					// Triggers - Issue is PB0 aka SCK2 (MPU clock)
	/*while(1);
	DDRB  = 0b10100000;
	DDRB  = 0b10110000;
	DDRB  = 0b10110010;
	DDRB  = 0b10110011;*/
	
	
	
	// Potential fix: first set pull-up, then set as output
	PORTB = init_portb;
	DDRB  = init_ddrb;		// Initial
	
	//__builtin_avr_nops(10);	//
	
	//PORTB = init_portb;
	
	
	//PORTB&=0b11111101;		// turn on a led
	
	//while(1);
	
	#if (HWVER==7)
	// V7
	init_ddra = 0b00110000;
	init_porta = 0b11111111;
	#else
	// V1-V6
	init_ddra = 0b00110000;
	init_porta = 0b01111111;;
	#endif	
	DDRA  = init_ddra;
	PORTA = init_porta;
	
	
	//_delay_ms(1000);
	//while(1);
	
	//_delay_ms(150);
	
	// Code initially set DDRC then PORTC; however this could lead to pulses....

	/*
		CRITICAL:
		In version HW8, PC7 must be set to drive 1 before being configured as output.
		Therefore set PORTC before setting DDRC
	*/
	#if (HWVER==4) 
	PORTC = 0b11010000;		// Default for V4
	#endif
	#if (HWVER==5)
	PORTC = 0b01010000;		// Default for V5
	#endif
	#if (HWVER==6) || (HWVER==7)
	PORTC = 0b11011000;		// Default for V6, V7
	#endif
	
	DDRC  = 0b11001000;		// Default
	
	

	
	
	/////////////////////////////////
	//DDRC  = 0b11001011;	// Test i2c, set as output
	//PORTC = 0b11010011;	// test i2c
	//PORTA &= 0b11011111;	// BT hard reset
	/////////////////////////////////
	
	#if (HWVER==4) 	
	DDRD  = 0b01101010;	// V4
	PORTD = 0b00000011;	// V4
	#endif
	#if (HWVER==5)
	DDRD  = 0b00101010;	// V5
	PORTD = 0b01000011;	// V5 pullup on pwren
	#endif
	#if (HWVER==6) || (HWVER==7) 
	DDRD  = 0b00101010;	// V6
	PORTD = 0b00000011;	// V6, V7
	#endif
	
	
	
}
void init_portchangeint(void)
{

	// Interrupt on change on PA6 (RTC int)
	// PA6 is PCINT6
	PCMSK0 = 0b01000000;			// Mask to select interrupt PA6
		
	// Interrupt on change on PC5 (Motion int)
	// PC5 is PCINT21
	PCMSK2 = 0b00100000;			// Mask to select motion_int (PC5)
	
	#if HWVER==4 	
	// Interrupt on change on PD7 (Bluetooth connect) and PD4 (Bluetooth RTS)
	// PD7 is PCINT31
	PCMSK3 = 0b10010000;
	#endif
	#if (HWVER==5) || (HWVER==6) || (HWVER==7)
	// Interrupt on change on PD7 (Bluetooth connect), PD4 (Bluetooth RTS), and PD6 (USB connect)
	// PD7 is PCINT31
	PCMSK3 = 0b11010000;
	#endif
	
	
	// Enable PCIE0 and PCIE2
	PCICR = 0b00001101;				// Enable interrupt on port A and C
}
void deinit_portchangeint(void)
{
	// Deinitialise the port change interrupt	
	PCICR=0;
	PCMSK0=0;
	PCMSK2=0;
	PCMSK3=0;
}

#endif

void init_timers(void)
{
	// Timer 1: CPU
	/*
	TCCR1A = 0x00;									// Clear timer on compare
	TCCR1B = 0x08|0x01;								// Clear timer on compare, prescaler 1
	TIMSK1 = (1<<OCIE1A);							// Output compare match A interrupt enable
	#if HWVER==1
	OCR1A = 7199;									// Top value: divides by OCR1A+1; 7199 leads to divide by 7200
	#endif
	#if (HWVER==4) || (HWVER==5) || (HWVER==6) || (HWVER==7)
	OCR1A = 10799;									// Top value: divides by OCR1A+1; 10799 leads to divide by 10800
	#endif
	*/
	// Use timer 3 for internal clocking, no prescaler
	TCCR3A = 0x00;									// Clear timer on compare
	TCCR3B = 0x08|0x01;								// Clear timer on compare, prescaler 1
	TIMSK3 = (1<<OCIE3A);							// Output compare match A interrupt enable
	#if HWVER==1
	OCR3A = 7199;									// Top value: divides by OCR1A+1; 7199 leads to divide by 7200
	#endif
	#if (HWVER==4) || (HWVER==5) || (HWVER==6) || (HWVER==7)
	OCR3A = 10799;									// Top value: divides by OCR1A+1; 10799 leads to divide by 10800
	#endif
	
	// Use timer 2 for internal clocking at 20mS intervals
	TCCR2A = 0b00000010;							// Clear timer on compare
	TCCR2B = 0b00000111;							// Prescaler 1024
	TIMSK2 = 0b00000010;							// Output compare match A interrupt enable
	#if HWVER==1
	OCR2A = 143;									// Divides by 243+1 -> 20ms period.
	#endif
	#if (HWVER==4) || (HWVER==5) || (HWVER==6) || (HWVER==7)
	OCR2A = 215;									// Divides by 215+1 -> 20ms period.
	#endif

}
void deinit_timers(void)
{
	
	//TIMSK1 = 0;		// Deinitialise timer 1
	TIMSK3 = 0;		// Deinitialise timer 3
}

void init_module(void)
{
	init_ports();
	init_portchangeint();
	init_timers();
	
#if BOOTLOADER==0
	spi_init(SPI_DIV_2);
#endif
	
#if BOOTLOADER==0
	
	sei();				// enable all interrupts
	
	
	
	
	
	#if HWVER==1
		uart1_init(3,0);	
	#endif
	#if (HWVER==4) || (HWVER==5) || (HWVER==6) || (HWVER==7)
		uart1_init(5,0);	// 115200bps  @ 11.06 Mhz
		//uart1_init(2,0);	// 230400bps  @ 11.06 Mhz
	#endif
	
	#if HWVER==1
		uart0_init(1,0);	
	#endif
	
	#if (HWVER==4) || (HWVER==5) || (HWVER==6) || (HWVER==7)
		#if BOOTLOADER==0
			spiusart0_init();
		#endif
	#endif
	
#endif
}

#if BOOTLOADER==0
void init_lcd(void)
{
	lcd_spi_init();
	file_fb = fb_initfb();
	// Reset display (PB2)
	PORTB &= 0b11111011;
	_delay_ms(100);
	PORTB |= 0b00000100;
	_delay_ms(100);	
	lcd_led(1);	
	lcd_init();
	lcd_clear565(0);
	lcd_writestring("BlueSense2",4,55,3,0x0000,0xffff);
}
void deinit_lcd(void)
{
	lcd_spi_deinit();
	file_fb = 0;
}


void init_powerreduce(void)
{
	// Power reduction
	/*
	In idle mode, calling this during init:
		-18 -18 -18 -18 -19 -18 -18 -18 -18 -19: 18.2mW

	In idle mode, not calling this during init:
		-19 -19 -21 -18 -21 -19 -21 -19 -19:  19.5mW
		
	~1mW savings.
	*/
	
	// ADC should be turned off (default): ADEN=bit7=0
	ADCSRA&=0b01111111;
	
	// Analog Comparator must be disabled and set not to use the Internal Voltage Reference, otherwise the internal voltage reference stays enabled
	// By default the analog comparator is enabled.
	// Disable Analog Comparator: ACD=bit7=1; Disable Fixed Bandgap ACBG=bit6=0
	ACSR=0b10000000;
	
	// Brown out detector should be disabled. This is controlled by the fuses (not in software control)
	
	// Internal Voltage Reference should be disabled. The Internal voltage reference is used by the brown out detector (must be off), the analog comparator (must be off), and the ADC (must be off)
	
	// Watchdog timer must be disabled. If the fuses enable the watchdog nothing can be done. Otherwise, by default the watchdog timer runs even if the watchdog is not activated
	wdt_disable();
	
	// Disable the digital input buffer on AIN1/0 
	DIDR1=0b00000011;
	
	// Disable the digital input buffer on ADC7, ADC3-0
	DIDR0=0b10001111;
	
	// On-chip debug system (OCDEN) and JTAG (JTAGEN) fuses must be unset

	
	// Power reduction register 0: PRTWI PRTIM2 PRTIM0 PRUSART1 PRTIM1 PRSPI PRUSART0 PRADC
	// Disable: Timer2, Timer0, ADC
	PRR0 = 0b01100001;
	// // Power reduction register 1: PRTIM3
	// Disable: Timer3
	PRR1 = 0b00000001;
	
}
ISR(WDT_vect)
{
    // Do something with a pin
	//static unsigned char status=0;
	
	//wdt_reset();
	WDTCSR |= (1<<WDIE);			// WDT goes into system reset mode automatically; we must re-enable interrupt mode
	system_led_toggle(1);	
	//system_led_toggle(100);	
}

void init_wdt(void)
{
	cli();												// Disable all interrupts
	wdt_reset();										// Needed?		
	WDTCSR = (1<<WDCE)|(1<<WDE);						// WDT change sequence
	WDTCSR = (1<<WDIE)|(1<<WDE)|(1<<WDP3);				// WDT interrupt mode, 8 seconds
	//WDTCSR = (1<<WDIE)|(1<<WDE)|(1<<WDP2)|(1<<WDP1);	// WDT interrupt mode, 1 second
	sei();												// Enable all interrupts.
}

#endif