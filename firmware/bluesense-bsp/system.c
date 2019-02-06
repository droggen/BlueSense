// System: system functions suitable for bootloader and application code

#include "cpu.h"
#include <avr/io.h>
#include <util/atomic.h>


#include <util/delay.h>
#include "system.h"
#include "init.h"




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
#endif
#if (HWVER==9)
void system_led_set(unsigned char led)
{
	led=(~led)&0b111;
	PORTC = (PORTC&0b10010111)|((led&0b1)<<6)|((led&0b10)<<4)|((led&0b100)<<1);
}
void system_led_on(unsigned char ledn)
{
	if(ledn==0)
		PORTC&=0b10111111;
	if(ledn==1)
		PORTC&=0b11011111;
	if(ledn==2)
		PORTC&=0b11110111;
}
void system_led_off(unsigned char ledn)
{
	if(ledn==0)
		PORTC|=0b01000000;
	if(ledn==1)
		PORTC|=0b00100000;
	if(ledn==2)
		PORTC|=0b00001000;
}
void system_led_toggle(unsigned char led)
{
	PINC = ((led&0b1)<<6) | ((led&0b10)<<4) | ((led&0b100)<<1);
}
#endif
#if (HWVER==6) || (HWVER==7) || (HWVER==9)
// Three LED version
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
// Two LED version
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

#if (HWVER==9)
unsigned char system_isusbconnected(void)
{
	return (PINC&0b00000100)?1:0;
}
#endif
