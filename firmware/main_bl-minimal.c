/*
	file: main_bl-minimal
	
	Minimalistic test bootloader.
	
	Setups the board interfaces; signal aliveness with the LEDs and sending a message to the USB interface; and jumps to the user code.
	
	Compile with Makefile_bl-minimal.

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

FILE *file_usb,*file_bt;
void (*app_start)(void) = 0x0000;

/******************************************************************************
	Timer interrupt vectors
******************************************************************************/
// CPU 1024Hz
ISR(TIMER1_COMPA_vect)
{
	//wdt_reset();
	//time_ms++;
	dbg_callback(0);
}



/******************************************************************************
Main program loop
******************************************************************************/
int main(void)
{
	unsigned char mcusr;
	/*
		We have a hard time rebooting to the application once all the registers are initialised. 
		Therefore we set a watchdog reset, and here we detect whether we were reseted by the watchdog.
		If yes, then jump directly to application (all registers are still clear).
	*/
	mcusr=MCUSR;
	MCUSR=0;
	if(mcusr&0x8)
	{
		// Rebooted due to a watchdog reset: go to application code.
		wdt_disable();
		app_start();
	}

	// INIT MODULE
	init_ports();
	init_timers();
	//uart1_init(5,0);
	i2c_init();
	dbg_init();
	file_usb = serial_open(10,1);
	//file_bt=serial_open(1,1);
	serial_setblocking(file_usb,0);
	//serial_setblocking(file_bt,0);
	dbg_setnumtxbeforerx(10);
	
	// Move interrupt vector and enable interrupts
	unsigned char temp = MCUCR;
	MCUCR = temp|(1<<IVCE);
	MCUCR = temp|(1<<IVSEL);
	sei();
	
	//cli();
	//dbg_rx_callback=echo_dbg2bt;
	//sei();
	
		
	fputs("BS2 BL\n",file_usb);
	//fprintf(file_usb,"%s Bootloader\n","BlueSense");
	//fputs("BlueSense2 BTBootloader\n",file_bt);
	
	
	//fprintf(0,"a");			// 1560
	//fprintf(0,"a %d",4);			// 2886, 2528, 4428
	//fputs("a",0);					// 1560
	
	//char *b;
	//b = malloc(128);
	
	//scanf("%d",&n);
	
	// Bootloader life sign
	system_led_set(0b111);
	_delay_ms(100);
	system_led_set(0b101);
	
	// Wait for something to happen
	_delay_ms(1000);
	
	
	// Turn off bootloader life sign
	system_led_set(0b000);
	
	// Reboot
	
	
	/*while(1)
	{
		system_blink(10,50,0b01);
		system_led_set(0b00);
		_delay_ms(500);
	}*/
	
	
	fputs("BS2 BL Reboot\n",file_usb);
	
	// Waits for I/O transfers to complete
	_delay_ms(100);
	// Move interrupt vector and enable interrupts
	//cli();
	//temp = MCUCR;
	//MCUCR = temp|(1<<IVCE);
	//MCUCR = temp&(~(1<<IVSEL));
	
	wdt_enable(WDTO_250MS);
	while(1); // WDT will do reset
	
	
	
	
	


}





