/*
	file: main_bl-minimal
	
	Minimalistic test bootloader for BlueSense2.
	
	Setups the board interfaces; signal aliveness with the LEDs and sending a message to the USB interface; and jumps to the user code.
	
	Compile with Makefile_bl-minimal.
	
	Flags must be: 
		Extended: FF
		High: D0
		Low: FF


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

#include "init.h"
#include "system.h"
#include "i2c.h"
#include "dbg.h"
#include "serial.h"

FILE *file_usb;			// Debug interface
//FILE *file_bt;		// Bluetooth interface

void (*app_start)(void) = 0x0000;

/******************************************************************************
	Timer interrupt vectors
******************************************************************************/
// CPU 1024Hz
ISR(TIMER3_COMPA_vect)
{
	// Blink at ~2Hz
	/*static unsigned int ctr=0;
	ctr=(ctr+1)&0xff;
	if(ctr==0)
		system_led_toggle(0b001);*/
		
	//wdt_reset();
	//time_ms++;
	dbg_callback(0);
}


void move_interrupts()
{
	// Move interrupt vectors to the boot area
	unsigned char temp = MCUCR;
	MCUCR = temp|(1<<IVCE);
	MCUCR = temp|(1<<IVSEL);
}

/******************************************************************************
Main program loop
******************************************************************************/
int main(void)
{
	unsigned char mcusr,ubrr0;
	/*
		We have a hard time rebooting to the application once all the registers are initialised. 
		Therefore we set a watchdog reset, and here we detect whether we were reseted by the watchdog.
		If yes, then jump directly to application (all registers are still clear).
	*/
	mcusr=MCUSR;
	ubrr0=UBRR0;
	MCUSR=0;
	if(mcusr&0x8)
	{
		// Rebooted due to a watchdog reset: go to application code.
		wdt_disable();
		app_start();
	}

	// Init ports
	init_ports();
	
	// LED lifesign
	system_blink_inbootloader();
	
	
	// Initialise timers
	init_timers();
	// Initialise I2C
	i2c_init();
	
	// Initialise debug interface
	dbg_init();
	dbg_setnumtxbeforerx(10);
	// Initialise I/O
	file_usb = serial_open(10,1);
	serial_setblocking(file_usb,0);
	
	
	
	// Move interrupts to the boot area
	move_interrupts();
	// Enable interrupts
	sei();
	
	// Print text
	fputs("BS2 BL\n",file_usb);
	fprintf(file_usb,"MCUSR %X\n",mcusr);
	fprintf(file_usb,"UBRR0 %X\n",ubrr0);
	
	// Wait a bit and reboot
	_delay_ms(1000);
	fputs("BS2 BL Reboot\n",file_usb);
	
	wdt_enable(WDTO_250MS);
	while(1); // WDT will do reset
	
}





