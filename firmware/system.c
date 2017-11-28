#include "cpu.h"
#include <avr/io.h>
#include <util/atomic.h>



#if BOOTLOADER==0
#include <util/delay.h>
#include "wait.h"
#include "main.h"
#include "adc.h"
#include "interface.h"
#include "megalol/i2c_internal.h"
#include "ltc2942.h"
#include "ds3232.h"
#else
//extern unsigned long int timer_ms_get(void);
#include <util/delay.h>
#endif

#include "system.h"
#include "init.h"


#if BOOTLOADER==0
//volatile unsigned short system_battery_voltage=0;
//volatile unsigned long int system_battery_updatetime=0;
unsigned char system_enable_lcd=0;
#endif


void system_delay_ms(unsigned short t)
{
	//#if BOOTLOADER==1
	//	unsigned long int tstart=timer_ms_get();
	//	while(timer_ms_get()-tstart<(unsigned long int)t);
	//#else
		_delay_ms(t);
	//#endif
}

#if HWVER==1
void system_led_set(unsigned char led)
{
	led=(~led)&0b11;
	PORTB = (PORTB&0b11111100)|(led&0b11);
}
void system_led_toggle(unsigned char led)
{
	PINB = led&0b11;
}
#endif
#if (HWVER==4) || (HWVER==5) 
void system_led_set(unsigned char led)
{
	led=(~led)&0b11;
	PORTB = (PORTB&0b11111101)|(led&0b10);
	PORTC = (PORTC&0b10111111)|((led&0b1)<<6);
}
void system_led_on(unsigned char ledn)
{
	if(ledn==0)
		PORTC&=0b10111111;
	if(ledn==1)
		PORTB&=0b11111101;
}
void system_led_off(unsigned char ledn)
{
	if(ledn==0)
		PORTC|=0b01000000;
	if(ledn==1)
		PORTB|=0b00000010;
}
void system_led_toggle(unsigned char led)
{
	PINB = led&0b10;
	PINC = (led&0b1)<<6;
}
#endif
#if (HWVER==6) || (HWVER==7)
void system_led_set(unsigned char led)
{
	led=(~led)&0b111;
	PORTB = (PORTB&0b11111101)|(led&0b10);
	PORTC = (PORTC&0b10110111)|((led&0b1)<<6)|((led&0b100)<<1);
}
void system_led_on(unsigned char ledn)
{
	if(ledn==0)
		PORTC&=0b10111111;
	if(ledn==1)
		PORTB&=0b11111101;
	if(ledn==2)
		PORTC&=0b11110111;
}
void system_led_off(unsigned char ledn)
{
	if(ledn==0)
		PORTC|=0b01000000;
	if(ledn==1)
		PORTB|=0b00000010;
	if(ledn==2)
		PORTC|=0b00001000;
}
void system_led_toggle(unsigned char led)
{
	PINB = led&0b10;
	PINC = ((led&0b1)<<6) | ((led&0b100)<<1);
}
void system_led_test(void)
{
	//for(unsigned char i=0;i<4;i++)
	{
		unsigned char v=0b100;
		for(unsigned char j=0;j<3;j++)
		{
			system_led_set(v);
			v>>=1;
			_delay_ms(50);
		}
		_delay_ms(50);
		v=1;
		for(unsigned char j=0;j<3;j++)
		{
			
			system_led_set(v);			
			v<<=1;
			_delay_ms(50);
		}
		_delay_ms(50);
	}
}
#else
void system_led_test(void)
{
	for(unsigned char i=0;i<4;i++)
	{
		unsigned char v=0b10;
		for(unsigned char j=0;j<2;j++)
		{
			system_led_set(v);
			v>>=1;
			_delay_ms(50);
		}
		_delay_ms(50);
		v=1;
		for(unsigned char j=0;j<2;j++)
		{
			
			system_led_set(v);			
			v<<=1;
			_delay_ms(50);
		}
		_delay_ms(50);
	}
}
#endif
void system_blink(unsigned char n,unsigned char delay,unsigned char init)
{
	system_led_set(init);
	for(unsigned char i=0;i<n;i++)
	{
		system_delay_ms(delay);
		system_led_toggle(0b11);
	}
}
void system_blink_led(unsigned char n,unsigned char timeon,unsigned char timeoff,unsigned char led)
{
	system_led_off(led);
	system_delay_ms(timeoff*4);
	for(unsigned char i=0;i<n;i++)
	{
		system_led_on(led);
		system_delay_ms(timeon);
		system_led_off(led);
		system_delay_ms(timeoff);
	}	
	system_delay_ms(timeoff*3);
}
void system_status_ok(unsigned char status)
{
	system_blink_led(status,150,150,1);
}
void system_status_ok2(unsigned char status)
{
	system_blink_led(status,150,150,1);
}
/******************************************************************************
	function: system_lifesign
*******************************************************************************
	Blinks an LED to indicate aliveness. 
	
	Called from a timer callback.
	
	Parameters:
		sec		-	Number of seconds elapsed since the epoch

	Returns:
		0 (unused)
******************************************************************************/
unsigned char system_lifesign(unsigned char sec)
{
	// 
	if((sec&0b11)==00)
		system_led_on(1);
	else
		system_led_off(1);	
	return 0;
}
unsigned char system_lifesign2(unsigned char sec)
{
	system_led_toggle(0b10);
	return 0;
}
/******************************************************************************
	function: system_batterystat
*******************************************************************************
	Uses the red LED (led0) to indicate battery level and whether the power-button 
	is pressed.
	
	If the power-button is pressed, the RED led turns on for the first 4 seconds, then 
	starts blinking until power is cut.
	
	Also handles storing battery state during power off.
	If a long-press on the power button occurs:
	- at 4 seconds, issues a background read of battery state.
	- at 4.5 seconds, save battery state.
	
	Must be called from a timer callback at 10Hz.
	
	If the power button is pressed, turn on the red LED. 
	If the power button is not pressed, then every 4 seconds:
	- either does not blink (battery on the full side)
	- or blinks 1, 2 or 3 times to indicate how low the battery is.
	
	Requires the ltc2942_last_mV function to return the battery voltage.
	
	
	Parameters:
		unused	-	Unused parameter passed by the timer callback

	Returns:
		0 (unused)
******************************************************************************/
unsigned char system_batterystat(unsigned char unused)
{
	// Counter counts from 0 to 39 hundredth of seconds and wraps around (10 second period).
	static unsigned char counter=0;
	static unsigned char nblinks;
#if HWVER==7
	static unsigned char lastpc=0;
	unsigned char newpc;
	static unsigned char pressduration=0;
#endif
	
	
#if HWVER==7
	newpc = PINC;
	// Check if pin has changed state and update LED accordingly
	if( (newpc^lastpc)&0b00010000)
	{
		// pin has changed
		pressduration=0;			// Reset the press duration
		// If push button is pressed, turn on the LED, otherwise off.
		if((newpc&0b00010000)==0)
		{
			system_led_on(0);
		}
		else
		{
			system_led_off(0);
		}
		
		lastpc=newpc;
		counter=9;	// Reset the counter to somewhere after the blinks
	}
	// If the push button is pressed, do not do the battery display, but increase the press duration counter
	if((newpc&0b00010000)==0)
	{
		pressduration++;
		if(pressduration==40)
		{
			// Issue a background read of the battery state
			ltc2942_backgroundgetstate(0);
		}
		if(pressduration==45)
		{
			// Store the battery info
			system_storepoweroffdata2();
			
		}
		if(pressduration>40 && pressduration<45)
		{
			system_led_toggle(0b001);
		}
		return 0;
	}
#endif
	
	if(counter==0)
	{
		// If counter is zero, we initialise the blink logic.
		// nblinks indicate the number of blinks to issue.
		nblinks=0;
		unsigned short mv = ltc2942_last_mV();
		
		//mv = BATTERY_VERYVERYLOW-1;
		
		nblinks=0;
		if(mv<BATTERY_LOW)
			nblinks++;
		if(mv<BATTERY_VERYLOW)
			nblinks++;
		if(mv<BATTERY_VERYVERYLOW)
			nblinks++;
			
		// Must blink nblinks time.		
	}
	// When nblinks is nonzero, the led is turned on when the lsb is 0, and turned off when the lsb is 1.
	if(nblinks)
	{
		if( (counter&1) == 0)
		{
			system_led_on(0);
		}
		else
		{
			system_led_off(0);
			nblinks--;
		}
	}
	
	counter++;
	if(counter>39)
		counter=0;
	return 0;
}
void system_status_error(unsigned char error,unsigned char forever)
{
	do
	{
		system_led_on(1);
		system_blink_led(error,150,150,0);
		system_led_off(1);
		if(forever)
			system_delay_ms(1000);
	}
	while(forever);
}

void system_chargeled_set(unsigned char led)
{
// Test blink PC2 (charge led)
	/*DDRC |= 0b100;		// Put PC 2 as output
	for(unsigned char i=0;i<5;i++)
	{
		_delay_ms(150);
		PORTC^=0b100;			// Blink PC2		
	}*/
}


/*
	Converts ADC readings to millivolts
	
	XXXX Assumption: XXXXXXXXXXXXXXXXXXXX voltage regulator gives 3039mV, ADC senses vbat/2
	Assumption: internal vref=2.56V, ADC senses vbat/2
*/
unsigned short Battery_ADC2mv(unsigned short adc)
{
	// 32-bit required to avoid overflow.
	//unsigned long vcc=3039;
	unsigned long vcc=2560;
		
	unsigned long t = vcc*adc;
	t=t*2;	
	t = t/1023;
	
	return (unsigned short)t;
}


unsigned char system_isbtconnected(void)
{
	return (PIND&0x80)?1:0;
}
// system_isusbconnected requires that a battery readout has been performed prior to calling this function.
#if HWVER==4
unsigned char system_isusbconnected(void)
{
	if(system_battery>BATTERY_THRESHOLD_CONNECTED)
		return 1;
	else
		return 0;
}
#endif
#if HWVER==5
unsigned char system_isusbconnected(void)
{
	return (PIND&0b01000000)?0:1;
}
#endif
#if (HWVER==6) || (HWVER==7)
unsigned char system_isusbconnected(void)
{
	return (PIND&0b01000000)?1:0;
}
#endif


#if BOOTLOADER==0



/******************************************************************************
	system_getbattery
*******************************************************************************	
	Returns the battery voltage in mV. 
	The battery voltage is sampled in the background through a timer callback.
******************************************************************************/
unsigned short system_getbattery(void)
{
	
	return ltc2942_last_mV();
}
/******************************************************************************
	system_getbatterytime
*******************************************************************************	
	Returns the battery voltage in mV. 
	The battery voltage is sampled in the background through a timer callback.
	
******************************************************************************/
/*unsigned long system_getbatterytime(void)
{
	unsigned long t;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		t = system_battery_updatetime;	
	}
	return t;
}
*/
#if HWVER==1
void system_off(void)
{
}
#endif
#if HWVER==4
void system_off(void)
{
	// Clear PC7
	PORTC&=0b01111111;
}
#endif
#if HWVER==5
void system_off(void)
{
	// Set PC7
	PORTC|=0b10000000;
}
#endif
#if (HWVER==6) || (HWVER==7)
void system_off(void)
{
	// Clear PC7
	PORTC&=0b01111111;
}
#endif


void system_adcpu_off(void)
{
	DDRA &= 0xf0;										// 4 extension ADC as input
	PORTA = init_porta&0xf0;					// disable pull-up on 4 ADC input
}
void system_adcpu_on(void)
{
	//DDRA &= 0xf0;										// 4 extension ADC as input
	PORTA = init_porta;								// enable pull-up on 4 ADC input
}

unsigned char *system_getdevicename(void)
{
	return system_devname;
}

#endif

/******************************************************************************
	function: system_getrtcint
*******************************************************************************	
	Returns the status of the DS3232 INT#/SQW.
	
	This pin is active low. The DS3232 seconds counter is incremented at the
	same time as high to low transitions.
	
	This pin is connected to PA6 in all hardware versions.
	
	Parameters:

	Returns:
		Status of the INT#/SQW pin (1 or 0)
******************************************************************************/
unsigned char system_getrtcint(void)
{
	return (PINA>>6)&1;
}
/******************************************************************************
	function: system_settimefromrtc
*******************************************************************************	
	Sets the system time returned by timer_ms_get and timer_us_get from the RTC.
	
	
	The epoch for millisecond time is midnight of the first day of the month.
	The epoch for the microsecond time is the current hour.
	
	
	Parameters:

	Returns:

******************************************************************************/
void system_settimefromrtc(void)
{
	unsigned char h,m,s;		// hour, minutes, seconds
	unsigned char day,month,year;
	unsigned long t1;
	
	fprintf_P(file_pri,PSTR("Setting time from RTC... "));
	
	ds3232_readdatetime_conv_int(1,&h,&m,&s,&day,&month,&year);
	unsigned long epoch_s = ((day-1)*24l+h)*3600l+m*60l+s;
	unsigned long epoch_us = (m*60l+s)*1000l*1000l;
	timer_init(epoch_s,epoch_us);
	t1=timer_ms_get();
	fprintf_P(file_pri,PSTR("done: %02d.%02d.%02d %02d:%02d:%02d = %lu ms\n"),day,month,year,h,m,s,t1);

}
/******************************************************************************
	function: system_storepoweroffdata
*******************************************************************************	
	Store the current power/voltate/date/time to EEPROM. This is used before 
	powering off the system to compute the power used in off mode.
	
	Interrupts:
		Not suitable for use in interrupts.
	
	
	Parameters:

	Returns:

******************************************************************************/
void system_storepoweroffdata(void)
{
	// Read time synchronously
	unsigned char h,m,s,day,month,year;
	ds3232_readdatetime_conv_int(1,&h,&m,&s,&day,&month,&year);
	// Set the charge counter at mid-point
	ltc2942_setchargectr(32768);	// Set charge counter at mid-point
	// Read the voltage and charge	
	unsigned long charge = ltc2942_getcharge();	
	unsigned long chargectr = ltc2942_getchargectr();
	unsigned short voltage = ltc2942_getvoltage();
	
	printf("charge: %lu chargectr: %lu voltage: %u\n",charge,chargectr,voltage);
	
	// Store charge, voltage and time
	eeprom_write_byte((uint8_t*)STATUS_ADDR_OFFCURRENT_VALID,1);
	eeprom_write_dword((uint32_t*)STATUS_ADDR_OFFCURRENT_CHARGE0,charge);
	eeprom_write_word((uint16_t*)STATUS_ADDR_OFFCURRENT_VOLTAGE0,voltage);
	eeprom_write_byte((uint8_t*)STATUS_ADDR_OFFCURRENT_H,h);
	eeprom_write_byte((uint8_t*)STATUS_ADDR_OFFCURRENT_M,m);
	eeprom_write_byte((uint8_t*)STATUS_ADDR_OFFCURRENT_S,s);
	eeprom_write_byte((uint8_t*)STATUS_ADDR_OFFCURRENT_DAY,day);
	eeprom_write_byte((uint8_t*)STATUS_ADDR_OFFCURRENT_MONTH,month);
	eeprom_write_byte((uint8_t*)STATUS_ADDR_OFFCURRENT_YEAR,year);	
}
void system_storepoweroffdata2(void)
{
	// Get voltage and charge
	unsigned long charge = ltc2942_last_charge();	
	unsigned short voltage = ltc2942_last_mV();
	unsigned long time = ltc2942_last_updatetime();
	
	// Store charge, voltage and time
	eeprom_write_byte((uint8_t*)STATUS_ADDR_OFFCURRENT_VALID,1);
	eeprom_write_dword((uint32_t*)STATUS_ADDR_OFFCURRENT_CHARGE0,charge);
	eeprom_write_word((uint16_t*)STATUS_ADDR_OFFCURRENT_VOLTAGE0,voltage);
	eeprom_write_dword((uint32_t*)STATUS_ADDR_OFFCURRENT_TIME,time);
	
}
void system_loadpoweroffdata2(_POWERUSE_OFF &pu)
{
	unsigned long charge;
	unsigned short voltage;
	unsigned long time;
	
	// Load charge, voltage and time
	pu.valid = eeprom_read_byte((uint8_t*)STATUS_ADDR_OFFCURRENT_VALID);
	pu.offcharge = eeprom_read_dword((uint32_t*)STATUS_ADDR_OFFCURRENT_CHARGE0);
	pu.offvoltage = eeprom_read_word((uint16_t*)STATUS_ADDR_OFFCURRENT_VOLTAGE0);
	pu.offtime = eeprom_read_dword((uint32_t*)STATUS_ADDR_OFFCURRENT_TIME);


	
	
}
