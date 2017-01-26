/*
	file: Main file
	
	todo
	- ADC sample multiple channels in A mode
	- ADC sample/return multiple channels
	- Increase the size of the command buffer
	- Reactivate the bootloader
	- Label in the motion mode
	- Add test in wait function that would allow to detect non-monoticity/wraparound of uS counter
	- Log becoming an system-wide functionality that redirects all serial comm to file?
	
	
	1) Auto-detect SD card on boot?
	H 2) Motion stream mode: two parameters: mode and logging 
	3) Power measurements in background
	4) PC interface
	5) Default mode (e.g. to automatically stream or enter demo or clock mode)
	6) Background blinking
	7) Speedup boot
	8) Download logs?
	9) Minimise flash usage
	10) Fix warnings in codebase
	V 11) Malloc pulled in from fdevopen ? 
	12) Script to launch at start
	12.1: CommandGet: must be split in one command reading from file_pri and filling internal buffer, and one reading internal buffer until it is empty
	
	
	Not linking unused functions: http://www.avrfreaks.net/forum/not-linking-unused-functions
	
	Tests on HW6:
	- SD card
	- Interface connect/disconnect and interface swap
	- ADC
	
	
	
	

	When "idle": must automatically detect which channel is used to send commands
	Must print information to both channels (if connected).

	What should be the features of the firmware?
	
	- Upon connect, auto stream data
	- set time
	- configure: sample rate, format, etc
	- start recording 
	- stop recording
	- list files
	- dump files
	



	
*/



#include "cpu.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <stdio.h>
#include <string.h>

#include "main.h"
#include "adc.h"
#include "serial.h"
#include "serial1.h"
#include "i2c.h"
#include "ds3232.h"
#include "rn41.h"
#include "mpu.h"
#include "mpu_test.h"
#include "pkt.h"
#include "wait.h"
#include "init.h"
//#include "lcd.h"
//#include "fb.h"
#include "uiconfig.h"
#include "helper.h"
#include "ufat.h"
//#include "i2c_poll.h"
#include "i2c_internal.h"
//#include "streaming.h"
#include "dbg.h"
#include "mpu-usart0.h"
//#include "mpu-common.h"
#include "system.h"
#include "boot.h"
#include "interface.h"
//#include "command.h"
#include "commandset.h"
#include "mode_sample_adc.h"
#include "mode_bench.h"
#include "mode_idle.h"
#include "mode_bt.h"
#include "mode_sd.h"
#include "mode_coulomb.h"
#include "mode_sample_motion.h"
#include "mode_motionrecog.h"
#include "mode_teststream.h"
#include "mode_testsync.h"
#include "mode_mputest.h"
#include "mode_demo.h"
#include "mode_demo_clock.h"
#include "ltc2942.h"
#include "motionconfig.h"
#include "wait.h"
#include "mode.h"

FILE *file_bt;			// Bluetooth
FILE *file_usb;			// USB
FILE *file_fb;			// Screen
FILE *file_dbg;			// Debug (assigned to file_bt or file_usb)
FILE *file_pri;			// Primary (assigned to file_bt or file_usb)

unsigned char sharedbuffer[520];

volatile unsigned char bluetoothrts=0;



// Device name
unsigned char system_devname[5];

// Configuration parameters
unsigned char config_enable_id,config_enable_timestamp,config_enable_battery,config_enable_temperature,config_enable_acceleration,config_enable_gyroscope,config_enable_checksum,config_data_format;
unsigned char config_sensorsr;
unsigned char config_streaming_format;

// Other
unsigned char system_mode=APP_MODE_IDLE;				// idle
//unsigned char system_mode=3;			// clock
//unsigned char system_mode=4;			// demo
//unsigned char system_mode=APP_MODE_ADC;			// adc
//unsigned char system_mode=APP_MODE_BT;



// System status
unsigned long int system_perf=0;
volatile signed short system_temperature=0;
volatile unsigned long int system_temperature_time=0;
signed short system_offdeltacharge;

/*void uiprintf(const char *fmt,...)
{
	va_list args;	
	va_start(args,fmt);
	vfprintf(file_bt,fmt,args);
//	vfprintf(file_dbg,fmt,args);
}*/




// Strings


void banner(FILE *f)
{
	fprintf(f,"----------------------- welcome to sensor\n");
}



/******************************************************************************
	function: system_lifesign
*******************************************************************************
	Blinks an LED to indicate aliveness. 
	
	Called from a timer callback.
	
	The parameter and return value are not used by the callback system.	
******************************************************************************/
unsigned char system_lifesign(unsigned char unused)
{
	system_led_toggle(0b10);
	return 0;
}





/******************************************************************************
	Timer interrupt vectors
******************************************************************************/
// CPU 1024Hz
ISR(TIMER1_COMPA_vect)
{
	// check whether higher priority interrupts are there.
	/*if(UCSR1A&(1<<RXC1))
	{
		USART1_RX_vect_core();
	}*/
	//wdt_reset();
	_timer_tick_1024hz();
}
// RTC 1024Hz
#if HWVER==1
ISR(TIMER2_COMPA_vect)
{
	//_timer_tick_1024hz();
}
#endif
// Pin change RTC
ISR(PCINT0_vect)
{
	//dbg_fputchar_nonblock('1',0);
	// Check pin state; second counter incremented on falling edge
	if(!(PINA&0b01000000))
	{
		_timer_tick_hz();
	}
}
// Pin change motion
ISR(PCINT2_vect)
{
	//dbg_fputchar_nonblock('m',0);
	//motion_int_ctr2++;
	// React on motion_int pin
	if(PINC&0x20)
	{
		mpu_isr();
	}
}
// Pin change: BT connect (PD7) USB connect (PD6) and RTS (PD4)
ISR(PCINT3_vect)
{
	//char b[16];
	
	
	//static unsigned char lastpind=0;
	unsigned char pind;
	
	pind = PIND;
	
	/*b[0]='P';
	b[1]=hex2chr((pind>>4)&0xf);
	b[2]=hex2chr(pind&0xf);
	b[3]='\n';
	b[4]=0;
	dbg_putbuf(b,4);*/
	

	// Check RTS change
	unsigned char rts = (pind&0x10)?1:0;
	if(rts != bluetoothrts)
	{
		Serial1RTSToggle(rts);
		//char buf[16]="rts 0\n";
		//buf[4]='0'+rts;
		//fputbuf(file_usb,buf,5);
		PORTC&=0b10111111;
		PORTC|=(rts<<6);
	}
	bluetoothrts = rts;		

	// Check connection change
	signed char cur_bt_connected,cur_usb_connected=-1;
	cur_bt_connected = system_isbtconnected();		
	#if (HWVER==5) || (HWVER==6) || (HWVER==7)
	cur_usb_connected = system_isusbconnected();	
	#endif	
	interface_signalchange(cur_bt_connected,cur_usb_connected);
}





/******************************************************************************
	function: main_perfbench
*******************************************************************************	
	Benchmarks the processor, here using simple counting. Use to assess
	CPU load during sampling.
	
	Return value:	performance result
******************************************************************************/
unsigned long main_perfbench(void)
{
	unsigned long int t_last,t_cur;
	unsigned long int ctr,cps;
	const unsigned long int mintime=2000;
		
	ctr=0;
	t_last=timer_ms_get();
	while((t_cur=timer_ms_get())-t_last<mintime)
	{
		ctr++;
	}
	cps = ctr*1000/(t_cur-t_last);
	return cps;
}


/*void test_timer1(void)
{
	unsigned char hour,min,sec;
	// Sync to RTC
	ds3232_readtime_sync_conv(&hour,&min,&sec);
	cli();
	rtc_time_sec=0;
	rtc_time=0;
	cpu_time=0;
	timer_init(0);
	TCNT1=0;
	TCNT2=0;
	sei();
	
	while(1)
	{
		_delay_ms(5);
		unsigned long t = timer_ms_get();
		printf("%05lu %05lu\n",rtc_time_sec,t);
	}
}


void test_timer(void)
{
	unsigned long t1,t2;
	while(1)
	{
		t1 = rtc_time;
		t2 = cpu_time;
		fprintf(file_usb,"%lu %lu %ld\n",t1,t2,t1-t2);
		_delay_ms(10);
	}
}
*/



void rtc_status(FILE *f)
{
	printf("RTC registers:\n");
	ds3232_printreg(f);
}



/*void test_rtc2(void)
{
	unsigned char r;
	unsigned char val[256];
	while(1)
	{
		//ds3232_readtime();
		
		printf("readreg: ");
		r = i2c_readregs(address,0,7,val);
		printf("%d\n",r);
		for(int i=0;i<7;i++)
			printf("%02X ",val[i]);
		printf("\n");
		
		_delay_ms(100);
	}
	r=ds3232_readtime_sync(val);
	unsigned long lt1,lt2,t1,t2,i;
	unsigned long ft1,ft2;
	signed long m1,m2;
	i=0;
	ft1 = lt1 = rtc_time;
	ft2 = lt2 = cpu_time;
	
	while(1)
	{
		r=ds3232_readtime_sync(val);
		t1=rtc_time;
		t2=cpu_time;
		i++;
		m1 = t1-ft1;
		m1 = m1-i*1024;
		m2 = t2-ft2;
		m2 = m2-i*1024;
		printf("%04lu. RTC: dt: %04lu. dtavg: %04lu. dtavgmod: %03ld.   CPU: dt: %04lu. dtavg: %04lu. dtavgmod: %03ld.   Sec: %04lu\n",i,t1-lt1,(t1-ft1)/i,m1,t2-lt2,(t2-ft2)/i,m2,rtc_time_sec);			
		lt1 = t1;
		lt2 = t2;
		_delay_ms(900);
	}
		
		//t = t*1000;
		//t = t/1024;
		//t = t*999;
		//t = t/1023;
		
		//for(int i=0;i<7;i++)
			//printf("%02X ",val[i]);
		//printf("\n");

}*/
void test_rtc3(void)
{
//	unsigned address = 104;
	//unsigned char r;
//	unsigned char val[256];
	/*while(1)
	{
		//ds3232_readtime();
		
		printf("readreg: ");
		r = i2c_readregs(address,0,7,val);
		printf("%d\n",r);
		for(int i=0;i<7;i++)
			printf("%02X ",val[i]);
		printf("\n");
		
		_delay_ms(100);
	}*/
	//
	/*unsigned long ct;
	ct = rtc_time_sec; while(rtc_time_sec == ct);
	
	unsigned long lt1,lt2,t1,t2,i;
	unsigned long ft1,ft2;
	signed long m1,m2;
	i=0;
	ft1 = lt1 = rtc_time;
	ft2 = lt2 = cpu_time;
	
	while(1)
	{
		ct = rtc_time_sec; while(rtc_time_sec == ct);
		t1=rtc_time;
		t2=cpu_time;
		i++;
		m1 = t1-ft1;
		m1 = m1-i*1024;
		m2 = t2-ft2;
		m2 = m2-i*1024;
		printf("%04lu. RTC: dt: %04lu. dtavg: %04lu. dtavgmod: %03ld.   CPU: dt: %04lu. dtavg: %04lu. dtavgmod: %03ld.   Sec: %04lu\n",i,t1-lt1,(t1-ft1)/i,m1,t2-lt2,(t2-ft2)/i,m2,rtc_time_sec);			
		lt1 = t1;
		lt2 = t2;
		_delay_ms(900);
	}
		
		//t = t*1000;
		//t = t/1024;
		//t = t*999;
		//t = t/1023;
		
		//for(int i=0;i<7;i++)
			//printf("%02X ",val[i]);
		//printf("\n");

		*/
		
		

}

/*void test_rtc4(void)
{
	unsigned char h,m,s;
	while(1)
	{
		ds3232_printreg(file_usb);
		//ds3232_readtime_sync_conv(&h,&m,&s);
		ds3232_readtime_conv(&h,&m,&s);
		printf("%02d:%02d:%02d\n",h,m,s);
		
		_delay_ms(900);
	}
	
	
}*/

unsigned char echo_uart1_rx_callback(unsigned char c)
{
	dbg_fputchar(c,0);				// Echo on USB
	//fputc(c, file_usb);
	
	
	return 1;										// Keep byte
	//return 0;										// Discard byte
}







/*void cbsamplecbx3(unsigned char status,unsigned char error,signed short x,signed short y,signed short z)
{
	unsigned long int time = timer_ms_get();

	motion_int_samplecompleted++;
	if(status!=0)
		motion_int_sampleerror++;
		

		
	if(do_packet==0)
	{
		char str[256];
		
		//printf("Before ht\n");
		
		//memset(str,'A',256);
		//str[255]=0;
		
		u32toa(time,str);
				
		str[strlen(str)+1]=0;
		str[strlen(str)]=' ';		
		u16toa(status,str+strlen(str));
		
		str[strlen(str)+1]=0;
		str[strlen(str)]=' ';		
		u16toa(error,str+strlen(str));
		
		str[strlen(str)+1]=0;
		str[strlen(str)]=' ';		
		s16toa(ax,str+strlen(str));
		
		str[strlen(str)+1]=0;
		str[strlen(str)]=' ';		
		s16toa(ay,str+strlen(str));
		
		str[strlen(str)+1]=0;
		str[strlen(str)]=' ';		
		s16toa(az,str+strlen(str));
		
		str[strlen(str)+1]=0;
		str[strlen(str)]=' ';		
		s16toa(gx,str+strlen(str));
		
		str[strlen(str)+1]=0;
		str[strlen(str)]=' ';		
		s16toa(gy,str+strlen(str));
		
		str[strlen(str)+1]=0;
		str[strlen(str)]=' ';		
		s16toa(gz,str+strlen(str));
		
		str[strlen(str)+1]=0;
		str[strlen(str)]=' ';		
		s16toa(temp,str+strlen(str));
		
		str[strlen(str)+1]=0;
		str[strlen(str)]='\n';
		

		//printf("cbsample3: %s\n",str);	
		int s = strlen(str);
		//if(uart0_fputbuf_int(str,s))
//			motion_int_samplepushfailed++;
		uart1_fputbuf_int(str,s);
	}
	else
	{
		PACKET p;
		//packet_init(&p);
		//packet_addchar(&p,'D');
		//packet_addchar(&p,'X');
		//packet_addchar(&p,'X');
		//packet_addshort_little(&p,time&0xffff);
		//packet_addshort_little(&p,(time>>16)&0xffff);
		//packet_addchar(&p,status);
		//packet_addchar(&p,error);
		//packet_addshort_little(&p,ax);
		//packet_addshort_little(&p,ay);
		//packet_addshort_little(&p,az);
		//packet_addshort_little(&p,gx);
		//packet_addshort_little(&p,gy);
		//packet_addshort_little(&p,gz);
		//packet_addshort_little(&p,temp);
		//packet_addchecksum_fletcher16_little(&p);
		int s = packet_size(&p);
	//if(uart0_fputbuf_int(p.data,s))
	//		motion_int_samplepushfailed++;
		uart1_fputbuf_int(p.data,s);
	}
}*/
/*
void cbsamplecb4(unsigned char status,unsigned char error,signed short ax,signed short ay,signed short az,signed short gx,signed short gy,signed short gz,signed short temp)
{
	unsigned long int time = timer_ms_get();

	motion_int_samplecompleted++;
	if(status!=0)
		motion_int_sampleerror++;
		

		
	if(do_packet==0)
	{
		char str[256];

		// We know how many bytes are resulting from the transformation
		// 4399999999 -32768 -32768 -32768 -32768 -32768 -32768 -32768 -32768 -32768 
		// 0         10     17     24     31     38     45     52     59     66     73
		//            11     18     25     32     39     46     53     60     67
		u32toa(time,str);
		str[10]=' ';
		s16toa(status,str+11);
		str[17]=' ';
		s16toa(error,str+18);
		str[24]=' ';
		s16toa(ax,str+25);
		str[31]=' ';
		s16toa(ay,str+32);		
		str[38]=' ';
		s16toa(az,str+39);
		str[45]=' ';
		s16toa(gx,str+46);		
		str[52]=' ';
		s16toa(gy,str+53);
		str[59]=' ';
		s16toa(gz,str+60);
		str[66]=' ';
		s16toa(temp,str+67);
		str[73]='\n';
		str[74]=0;
		
		

		//printf("cbsample4: %s\n",str);	
		int s = strlen(str);
		//if(uart0_fputbuf_int(str,s))
//			motion_int_samplepushfailed++;
		uart1_fputbuf_int(str,s);
	}
	else
	{
		return;
		PACKET p;
		packet_init(&p);
		packet_add8(&p,'D');
		packet_add8(&p,'X');
		packet_add8(&p,'X');
		packet_add16_little(&p,time&0xffff);
		packet_add16_little(&p,(time>>16)&0xffff);
		packet_add8(&p,status);
		packet_add8(&p,error);
		packet_add16_little(&p,ax);
		packet_add16_little(&p,ay);
		packet_add16_little(&p,az);
		packet_add16_little(&p,gx);
		packet_add16_little(&p,gy);
		packet_add16_little(&p,gz);
		packet_add16_little(&p,temp);
		//packet_end(&p);
		packet_addchecksum_fletcher16_little(&p);
		int s = packet_size(&p);
	//if(uart0_fputbuf_int(p.data,s))
	//		motion_int_samplepushfailed++;
		uart1_fputbuf_int(p.data,s);
	}
}
*/
/*
unsigned char cbsample(unsigned char a)
{
	//uart0_putchar('t');
	if(do_sample)
	{
		if(mpu_get_agt_int_cb(cbsamplecb4)==0)
		//if(!mpu_get_agt_int_cb(packet_send))
			motion_int_sampleissued++;
		else
			motion_int_samplefailed++;
			
	}
	return 0;
}*/







/*char ui_shouldexit(FILE *file)
{
	int c;
	c = fgetc(file);
	if(c!=EOF)
	{
		if(c=='x' || c=='X')
			return 1;
	}
	return 0;
}*/



/*void main_log(void)
{
	lcd_clear565(0);
	lcd_writestring("Logging",26,0,2,0x0000,0xffff);	
	
	fprintf_P(file_usb,PSTR("Logging mode. Press <X> to exit\n"));
	
	unsigned long t1;
	unsigned long tctr;
	int ctr=0;
	int c;	
	
	//serial_setblocking(file_usb,0);
	
	// The only sleep mode that can be used is IDLE. ADC_NR shuts down the IO clock, which is used by the UART
	set_sleep_mode(SLEEP_MODE_IDLE); 
	//set_sleep_mode(SLEEP_MODE_EXT_STANDBY);	
	
	mpu_config_motionmode(1,0);
	
	mpu_set_interruptpin(0x00);
	//mpu_set_interrutenable(0,0,0,1);
	mpu_set_interrutenable(0,0,0,0);
	
	
	
	
	
	
	while(1)
	{
		printf("Logging (sim): %d\n",ctr);
		
		if(ui_shouldexit(file_usb))
			return;


		t1 = timer_ms_get();
		tctr=0;
		while(timer_ms_get()-t1<1000)
		{
			tctr++;
			
			
			// Go to sleep. We can be waken up by many things: ADC, UART, timer. We check if there's something to do, then sleep more.
			sleep_enable();	
			sleep_cpu();
			sleep_disable();
			//_delay_ms(1);
			
		}
		printf("ctr: %ld\n",tctr);
		ctr++;
	}
	
	
	
}*/



/******************************************************************************
	system_gettemperature
*******************************************************************************	
	Returns the temperature from the DS332 in hundredth of degree.
	The temperature is sampled in the background through a timer callback.
	
******************************************************************************/
signed short system_gettemperature(void)
{
	signed short t;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		t = system_temperature;	
	}
	return t;
}
/******************************************************************************
	system_gettemperaturetime
*******************************************************************************	
	Returns the temperature from the DS332 in hundredth of degree.
	The temperature is sampled in the background through a timer callback.	
******************************************************************************/
unsigned long system_gettemperaturetime(void)
{
	unsigned long t;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		t = system_temperature_time;	
	}
	return t;
}

unsigned char system_sampletemperature(unsigned char p)
{
	ds3232_readtemp_int_cb(system_sampletemperature_cb);
	return 0;
}

void system_sampletemperature_cb(unsigned char s,signed short t)
{
	system_temperature = t;
	system_temperature_time = timer_ms_get();
}







extern void stk500(void);

/*void boottest(void)
{
	fprintf(file_bt,"Boot test\n");
	fprintf(file_bt,"Setting passing register\n");
	fprintf(file_bt,"Calling in 1\n");
	_delay_ms(1000);
	//boot_dbg();
	boot_bluetooth();
	//void boot_bluetooth(void);
	fprintf(file_bt,"Done\n");
	while(1);
}*/

/******************************************************************************
	function: bootloaderhook_dbg
*******************************************************************************	
	Hook checking the data received on the usb interface for the byte indicating
	that an stk500v2 bootloader message is received. 
	In which case, enter the bootloader	
	
	Parameters:
		c			-		Character received on the interface
	Returns:
		1			-		To place the character in the receive buffer
		otherwise	-		The character is discarded
******************************************************************************/
unsigned char bootloaderhook_dbg(unsigned char c)
{
	// Enters the bootloader on reception of 0x1B
	if(c==0x1B)
	//if(c=='a')		// to help debugging
	{
		for(char i=0;i<5;i++)
		{
			system_led_set(0b000);
			_delay_ms(50);
			system_led_set(0b111);
			_delay_ms(50);
		}
		boot_dbg();
	}
	return 1;
}
unsigned char bootloaderhook_bluetooth(unsigned char c)
{
	// Enters the bootloader on reception of 0x1B
	if(c==0x1B)
		boot_bluetooth();
	return 1;
}

/*
 7-8 ms for 1000 calls -> 8us/call
*/

void b0func(void)
{
	unsigned long t1,t2,tt;
	
b0start:

	for(unsigned i=0;i<5;i++)
	{
		unsigned long tt1,tt2,tt3;
		tt1=timer_us_get();
		tt2=timer_us_get_c();
		printf("%lu %lu\n",tt1,tt2);
		_delay_ms(100);		
	}

	tt=0;
	t1=timer_us_get();
	for(unsigned i=0;i<1000;i++)
	{
		timer_us_get();
	}
	t2=timer_us_get();
	printf("time with asm: %lu (%lu)\n",t2-t1,tt);
	
	tt=0;
	t1=timer_us_get();
	for(unsigned i=0;i<1000;i++)
	{
		timer_us_get_c();
	}
	t2=timer_us_get();
	printf("time with c: %lu (%lu)\n",t2-t1,tt);
	
	
	goto b0start;
	
}


/******************************************************************************
Main program loop
******************************************************************************/
int main(void)
{

	// INIT MODULE
	init_module();
	
	// LED Test
	system_led_test();	
	system_led_set(0b000);
		
	// I2C INITIALISATION	
	i2c_init();	
	
	// Fuel gauge initialisation
	// Get the charge - this assumes 
	unsigned long curcharge = ltc2942_getcharge();
	unsigned long oldcharge;
	oldcharge = eeprom_read_byte((uint8_t*)STATUS_ADDR_OFFCURRENT_CHARGE3);
	oldcharge<<=8;
	oldcharge |= eeprom_read_byte((uint8_t*)STATUS_ADDR_OFFCURRENT_CHARGE2);
	oldcharge<<=8;
	oldcharge |= eeprom_read_byte((uint8_t*)STATUS_ADDR_OFFCURRENT_CHARGE1);
	oldcharge<<=8;
	oldcharge |= eeprom_read_byte((uint8_t*)STATUS_ADDR_OFFCURRENT_CHARGE0);
	system_offdeltacharge=curcharge-oldcharge;
	ltc2942_init();																// Get 
	
		
	// DEBUG
	dbg_init();
	
	// First open stdio peripheral, then others
	#if HWVER==1
		// Initialize the streams
		file_usb = fdevopen(uart0_fputchar_int,uart0_fgetchar_int);		
		file_dbg=fdevopen(dbg_fputchar,0);
	#endif
	#if (HWVER==4) || (HWVER==5) || (HWVER==6) || (HWVER==7)
		file_usb = serial_open(10,1);
	#endif	
	// Bluetooth communication
	file_bt=serial_open(1,1);
	// Set all non blocking
	serial_setblocking(file_usb,0);
	serial_setblocking(file_bt,0);
	
	
	
	
	timer_register_callback(dbg_callback,1);
	//timer_register_callback(dbg_callback,4);
	/////////////////////////// PRINT WORKS FROM HERE ///////////////////////////
	
	//_delay_ms(500);
	//system_blink(20,50,0);
	
	//fprintf(file_pri,PSTR("Delta charge while off: %d\n"),system_offdeltacharge);
	fprintf(file_usb,PSTR("Delta charge while off: %d\n"),system_offdeltacharge);
	
	
	
	
	
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
	fprintf_P(file_usb,PSTR("Bluetooth setup\n"));
	rn41_Setup(file_usb,file_bt,system_devname);
	bluetoothrts = (PIND&0x10)?1:0;
	//cli(); uart1_rx_callback = echo_uart1_rx_callback; sei();					// Activate callback
	cli(); uart1_rx_callback = 0; sei();															// Deactivate callback


	//system_status_ok(7);
	system_status_ok(4);
	
	
	
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
	
	//system_status_ok2(5);
	
	interface_init();
	interface_changedetectenable(1);
	
	//system_status_ok2(4);
	
	interface_signalchange(system_isbtconnected(),system_isusbconnected());
	
	//system_status_ok2(3);
	
	interface_test();
	
	//system_status_ok2(2);

	

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
	
	banner(file_pri);
	//_delay_ms(500);
	
	/*_delay_ms(250);	
	system_status_ok(2);
	_delay_ms(250);*/
	
	// RTC INITIALISATION
	//fprintf(file_fb,"Before RTC init\n");
	//fprintf(file_bt,"Before RTC init\n");
	//fprintf(file_pri,"Before RTC init\n");
	//_delay_ms(500);
	rtc_init();	
	//_delay_ms(500);
	//fprintf(file_fb,"After RTC init\n");
	//fprintf(file_bt,"After RTC init\n");
	//fprintf(file_pri,"After RTC init\n");
	ds3232_printreg(file_pri);
	//ds3232_printreg(file_fb);
	
	//_delay_ms(500);
	
	//system_status_ok(5);
	
	system_status_ok(3);
	
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
	//fprintf_P(file_usb,PSTR("Initialise MPU subsystem\n"));
	fprintf_P(file_pri,PSTR("Initialise MPU subsystem\n"));
	
	// No interrupt handling routine setup
	//isr_motionint = 0;	
	// Init the MPU
	mpu_init();
	
	
	system_status_ok(3);
	
	// Estimate baseline performance
	printf_P(PSTR("Estimating baseline performance... "));		
	//fprintf_P(file_bt,PSTR("Estimating baseline performance... "));		
	//system_perf = main_perfbench();
	printf_P(PSTR("%ld\n"),system_perf);
	//fprintf_P(file_bt,PSTR("%ld\n"),system_perf);


	system_status_ok(2);

	// Register battery sample callback
	//timer_register_slowcallback(system_samplebattery,9);
	ds3232_readtemp((signed short*)&system_temperature);

	//timer_register_slowcallback(system_samplebattery_start,2);	
	//timer_register_slowcallback(system_sampletemperature,6);
	timer_register_slowcallback(system_lifesign,0);
	

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
	
	// Load the configuration script and store it into the command buffer
	/*char script[CONFIG_ADDR_SCRIPTLEN];	
	ConfigLoadScript(script);
	for(unsigned char i=0;i<CONFIG_ADDR_SCRIPTLEN;i++)
	{
		printf("%02X ",script[i]);
	}
	printf("\n");
	script[0]++;
	for(unsigned char i=1;i<CONFIG_ADDR_SCRIPTLEN;i++)
		script[i]=script[0]+i;
	ConfigSaveScript(script,CONFIG_ADDR_SCRIPTLEN);*/
	
	// copy to the command buffer
	//script[CONFIG_ADDR_SCRIPTLEN-1]='\n';
	//CommandSet(script,CONFIG_ADDR_SCRIPTLEN);
	//CommandSet("b\r\n",3);
	char buf[CONFIG_ADDR_SCRIPTLEN];
	ConfigLoadScript(buf);
	for(unsigned i=0;i<CONFIG_ADDR_SCRIPTLEN;i++)
		printf("%02X ",buf[i]);
	fprintf_P(file_pri,PSTR("Boot script: '%s' (len: %d)\n"),buf,strlen(buf));
	
	// Massage the script: replace newlines by semicolons
	for(unsigned char i=0;i<CONFIG_ADDR_SCRIPTLEN;i++)
		if(buf[i]==';') 
			buf[i]='\n';
	
	// set the boot script
	CommandSet(buf,strlen(buf));
	
		
	// Massage the script: replace newlines by semicolons
	/*for(unsigned char i=0;i<CONFIG_ADDR_SCRIPTLEN;i++)
		if(buf[i]==';') 
			buf[i]='\n';
	fprintf_P(file_pri,PSTR("Boot script: '%s'\n"),buf);*/
	
	//system_status_ok(5);
	
	//_delay_ms(500);
	


	
	
	
	
	mode_main();			// This never returns.
	
	while(1);
	return 0;
}







   





