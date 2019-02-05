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

#include "isr.h"
#include "wait.h"
#include "mpu.h"
#include "serial.h"
#include "serial1.h"
#include "system.h"
#include "interface.h"

volatile unsigned char bluetoothrts=0;

/******************************************************************************
	Timer interrupt vectors
******************************************************************************/
// CPU 1024Hz
ISR(TIMER3_COMPA_vect)
{
	// check whether higher priority interrupts are there.
	/*if(UCSR1A&(1<<RXC1))
	{
		USART1_RX_vect_core();
	}*/
	//wdt_reset();
	_timer_tick_1024hz();
}
// CPU some lower frequency stuff 
ISR(TIMER2_COMPA_vect)
{
	_timer_tick_50hz();
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

	//PORTC=(PORTC&0b11110111)|(((~PINA)&0b01000000)>>3);

	//system_led_toggle(0b100);

	
	//dbg_fputchar_nonblock('1',0);
	// Check pin state; second counter incremented on falling edge
	if(!(PINA&0b01000000))
	{
		_timer_tick_hz();
	}
}
#if HWVER==9
// HW9+ can detect MPU interrupt by timer comparison, used when downsampling 
ISR(TIMER1_COMPA_vect)
{
	/*static int c=0;
	c++;
	//dbg_fputchar_nonblock('m',0);
	if( (c&0b111111) ==0x00)
		system_led_toggle(0b001);*/
	
	// No need to check signal edge, directly call interrupt vector
	mpu_isr();
	// Clear the compare match interrut, if it was set again prior to mpu_isr returning
	TIFR1=0b00000010;
}
// HW9+ can detect MPU interrupt by timer input capture pin, used when not downsampling 
ISR(TIMER1_CAPT_vect)
{
	/*static int c=0;
	c++;
	//dbg_fputchar_nonblock('n',0);
	if( (c&0b111111)==0x00)
		system_led_toggle(0b100);*/
	
	// No need to check signal edge, directly call interrupt vector
	mpu_isr();
	// Clear the input capture interrut, if it was set again prior to mpu_isr returning
	TIFR1=0b00100000;
}
#endif


#if HWVER!=9
// HW9 supports timer-based edge detect and downsampling. Therefore, this is disabled by default on HW9.
// MPU interrupt detected by pin change interrupt
#if (HWVER==9)
ISR(PCINT1_vect)
#else
ISR(PCINT2_vect)
#endif
{
	/* React on motion_int pin
		The naive but technically correct approach is: trigger mpu_isr on the rising edge of motion_int.
		An issue arises if other interrupts delay this ISR by more than 50uS where
		the MPU interrupt pulse would be missed.
		
		Therefore we trigger on the falling edge as the low level lasts longer.
		
		This does not entirely prevent:
		- missing interrupts: if between the moment the ISR is called and the pin readout the pin toggles to 1, which could happen if the ISR is delayed by other interrupts.
		- or spurious mpu_isr calls: the ISR is called on the rising edge after some delay, the pin is sampled at 0 and a mpu_isr is triggered; now the pin is at 0 but the falling edge scheduled a new interrupt, which will lead to a spurious mpu_isr call.
		
		Spurious interrupts can be handled by clearing the PCIFR bit 2 (writing to 1) after the mpu_isr. This can lead to missing motion interrupts.
		Alternatively, the MPU interrupt flag can be checked in the mpu_isr routine, at the cost of CPU overhead. In this case, PCIFR should not be cleared. This however can still lead to missing motion interrupts.
		
		Solution decided: clear PCIFR bit 2 to avoid superfluous calls to mpu_isr and check in mpu_isr for spurious motion interrupts.
	*/
	
	
	
	//unsigned char i = PCIFR;
	//unsigned char p = PINC;
	//char b[16];
	//b[0]=hex2chr((i>>4)&0xf);
	//b[0]=hex2chr((i>>0)&0xf);
	//b[2]=32;
	//b[3]=hex2chr((p>>4)&0xf);
	//b[4]=hex2chr((p>>0)&0xf);
	//b[1]=' ';
	
	//for(i=0;i<3;i++)
		//dbg_fputchar_nonblock(b[i],0);
	
	//dbg_fputchar_nonblock(b[i],0);


	#if (HWVER==9)
	if((PINB&0x02)==0)			// MPU ISR on falling edge; hack to avoid missing interrupts
		mpu_isr();				
	//if(PINB&0x02)			// MPU ISR on rising edge; technically correct but misses interrupts.
	//	mpu_isr();
	PCIFR=0b0010;		// Clear pending interrupts
	#else
	if((PINC&0x20)==0)			// MPU ISR on falling edge; hack to avoid missing interrupts
		mpu_isr();				
	//if(PINC&0x20)			// MPU ISR on rising edge; technically correct but misses interrupts.
	//	mpu_isr();
	PCIFR=0b0100;		// Clear pending interrupts
	#endif
	

}
#endif

#if (HWVER==9)
// Pin change: USB connect (PC2)
ISR(PCINT2_vect)
{
	system_led_toggle(0b100);
	signed char cur_bt_connected,cur_usb_connected=-1;
	cur_bt_connected = system_isbtconnected();
	cur_usb_connected = system_isusbconnected();	
	interface_signalchange(cur_bt_connected,cur_usb_connected);
}
#endif
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
		//if(rts)
			//system_led_on(0);
		//PORTC&=0b10111111;
		//PORTC|=(rts<<6);
	}
	bluetoothrts = rts;		

	
	// Check connection change
	signed char cur_bt_connected,cur_usb_connected=-1;
	cur_bt_connected = system_isbtconnected();		
	
	#if (HWVER!=9)
	// HW9 does not have USB in this ISR.
	
	//PORTC&=0b11110111;
	//PORTC|=cur_bt_connected<<3;
	
	#if (HWVER==5) || (HWVER==6) || (HWVER==7)
	cur_usb_connected = system_isusbconnected();	
	#endif
	
	#endif	// HWVER!=9
	
	// Update interface
	interface_signalchange(cur_bt_connected,cur_usb_connected);
	
}
