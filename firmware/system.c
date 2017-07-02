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
	for(unsigned char i=0;i<4;i++)
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
	system_blink_led(status,150,150,0);
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
	
	The parameter and return value are not used by the callback system.	
******************************************************************************/
unsigned char system_lifesign(unsigned char unused)
{
	system_led_toggle(0b10);
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

void system_power_low(void)
{
	// PWR_STBY=1 (PC3)
	#if (HWVER!=6) && (HWVER!=7)
	PORTC|=0b00001000;	
	#endif
}
void system_power_normal(void)
{
	// PWR_STBY=0 (PC3)
	#if (HWVER!=6) && (HWVER!=7)
	PORTC&=0b11110111;	
	#endif
}

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



