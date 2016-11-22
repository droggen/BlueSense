/*
	file: main_bl
	
	STK500v2 compatible bootloader for BlueSense2.
	
	Three case exist for entering the bootloader:
	- The device has been turned on and during the bootloader wait period a program command is received
	  In this case: UBRR0=0
	- The device is running the application which intercepted a program command on the USB interface and jumped to the bootloader
	  In this case: UBRR0=250
	- The device is running the application which intercepted a program command on the Bluetooth interface and jumped to the bootloader
	  In this case: UBRR0=251
	
	The booloader must be called as follows from avrdude:
		avrdude -p atmega1284p -c stk500v2 -P com11 [-V] -U flash:w:main.hex -D 
	The option -D is required as the chip erase function is not implemented.
	The option -V prevents verification.
	
	The code uses a bootloader interface and a debug interface.
	file_usb and file_bt are respectively the USB and Bluetooth interface. 
	Depending on which is the interface on which the bootloader is detected, the bootloader interface
	becomes file_bl and the other one file_dbg.
	
	
	The stk500 booloader implements only the following functions:
	- CMD_SIGN_ON: to enter the ISP
	- CMD_SPI_MULTI: to query signature and fuse bits
	- CMD_GET_PARAMETER: to provide AVRISP HW and SW version
	- CMD_LOAD_ADDRESS: 
	- CMD_PROGRAM_FLASH_ISP
	- CMD_PROGRAM_EEPROM_ISP
	- CMD_READ_FLASH_ISP
	- CMD_READ_EEPROM_ISP
	
	ISSUES:
	- If jumping to bootloader from the application all the application interrupt are enabled, in particular port change interrupts. 
	  As there is no interrupt handler defined for these, if they occur they will lead to a reset.
	
	Compile with Makefile_bl
	
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
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "init.h"
#include "wait.h"
#include "system.h"
#include "i2c.h"
#include "dbg.h"
#include "serial.h"
#include "stk500.h"

//#define ENABLEBOOTLOADERONBLUETOOTH

FILE *file_usb;
FILE *file_bt;
volatile unsigned long time_ms=0;

#define TIMEOUTMS 3000
//#define TIMEOUTMS 10000



void (*app_start)(void) = 0x0000;


unsigned long int timer_ms_get(void)
{
	unsigned long t;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		t=time_ms;
	}
	return t;
}

// This detects a bootloader on 
FILE *detectbl(void)
{
	unsigned long tstart,tblink;
	tblink=tstart=timer_ms_get();
	while(timer_ms_get()-tstart<(unsigned long)TIMEOUTMS)
	{
		if(fgetrxbuflevel(file_usb))
		{
			return file_usb;
		}
		#ifdef ENABLEBOOTLOADERONBLUETOOTH
			if(fgetrxbuflevel(file_bt))
			{
				return file_bt;
			}
		#endif
		if(timer_ms_get()-tblink>500)
		{
			system_led_toggle(0b010);
			tblink = timer_ms_get();
		}
	}			
	return 0;
}

void sendchar(char c)
{
	fputc(c,file_bl);
}

int getchar_timeout(void)
{
	unsigned long tstart,tblink;
	
	system_led_toggle(0b1);
	tblink=tstart=timer_ms_get();
	while(timer_ms_get()-tstart<(unsigned long)TIMEOUTMS)
	{
		int c = fgetc(file_bl);
		if(c!=-1)
		{
			system_led_set(0b00);
			return c;
		}
		//_delay_ms(100);
		if(timer_ms_get()-tblink>100)
		{
			system_led_toggle(0b1);
			tblink = timer_ms_get();
		}
	}			
	return -1;
}


/******************************************************************************
	Timer interrupt vectors
******************************************************************************/
// CPU 1024Hz
ISR(TIMER1_COMPA_vect)
{
	//wdt_reset();
	time_ms++;
	dbg_callback(0);
}

unsigned char hex2chr(unsigned char v)
{
	if(v>9)
		return 'A'+v-10;
	return '0'+v;
}

extern volatile unsigned char dbg_rxlevel;

unsigned char echo_dbg2bt(unsigned char c)
{
	//#ifdef ENABLEBOOTLOADERONBLUETOOTH
		fputc(c,file_bt);
	//#endif
	return 1;
}

// Minimalistic sprintf
// format: 
//	-: 8-bit (char or short/int)
//	+: 16-bit (short/int)
//  *: 32-bit (long)
/*void msprintf(char *buffer,char *format,...)
{
	unsigned di=0,si=0;
	va_list args;	
	va_start(args,format);
	while(1)
	{
		if(format[si]==0)
		{
			buffer[di]=0;
			return;
		}
		if(format[si]=='-' || format[si]=='+')
		{
			unsigned short d = va_arg(args,unsigned int);
			if(format[si]=='+')
			{
				buffer[di] = hex2chr((d>>12)&0xf);
				di++;
				buffer[di] = hex2chr((d>>8)&0xf);
				di++;			
			}			
			buffer[di] = hex2chr((d>>4)&0xf);
			di++;
			buffer[di] = hex2chr(d&0xf);
			di++;			
		}
		else if(format[si]=='*')
		{
		}
		else
		{
			buffer[di] = format[si];
			di++;
			
		}
		si++;		
	}
}*/

void mfprintf(FILE *f,char *format,...)
{
	char buffer[256];
	unsigned di=0,si=0;
	va_list args;	
	va_start(args,format);
	while(1)
	{
		if(format[si]==0)
		{
			buffer[di]=0;
			break;
		}
		if(format[si]=='-' || format[si]=='+')
		{
			unsigned short d = va_arg(args,unsigned int);
			if(format[si]=='+')
			{
				buffer[di] = hex2chr((d>>12)&0xf);
				di++;
				buffer[di] = hex2chr((d>>8)&0xf);
				di++;			
			}			
			buffer[di] = hex2chr((d>>4)&0xf);
			di++;
			buffer[di] = hex2chr(d&0xf);
			di++;			
		}
		/*else if(format[si]=='*')
		{
		}*/
		else
		{
			buffer[di] = format[si];
			di++;
			
		}
		si++;		
	}
	fputs(buffer,f);
}

/*void echo(void)
{
	cli();
	dbg_rx_callback=echo_dbg2bt;
	sei();
	// BLINK
	unsigned char c=0;
	while(1)
	{
		unsigned short n = fgetrxbuflevel(file_usb);
		//unsigned short m = fgetrxbuffree(&_dbg_rx_buffer);
		putchar('h');
		putchar('0'+c%10);
		dbg_fputchar(hex2chr((n>>4)&0xf),0);
		dbg_fputchar(hex2chr(n&0xf),0);
		dbg_fputchar(hex2chr((dbg_rxlevel>>4)&0xf),0);
		dbg_fputchar(hex2chr(dbg_rxlevel&0xf),0);
		unsigned short c = getchar();
		dbg_fputchar(hex2chr((c>>4)&0xf),0);
		dbg_fputchar(hex2chr(c&0xf),0);
		dbg_fputchar('\r',0);
		dbg_fputchar('\n',0);
		
		//printf("Current buffer: %d\n",n);
		c++;
		system_blink(10,50,0b01);
		system_led_set(0b00);
		_delay_ms(100);
	}
}*/

/******************************************************************************
Main program loop
******************************************************************************/
int main(void)
{
	unsigned char mcusr;
	char *blstr="BS2 BL\n";
	char *leavingstr="BS2 BL: end\n";
	
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
		wdt_disable();			// The watchdog is still activated and must be deactivated now
		app_start();			// Jump to address 0.
	}
	
	
	

	// INIT MODULE
	init_ports();
	init_timers();
	uart1_init(5,0);
	i2c_init();
	dbg_init();
	file_usb = serial_open(10,1);
	serial_setblocking(file_usb,0);
	file_bt=serial_open(1,1);
	serial_setblocking(file_bt,0);
	dbg_setnumtxbeforerx(10);
	
	// Move interrupt vector and enable interrupts
	unsigned char temp = MCUCR;
	MCUCR = temp|(1<<IVCE);
	MCUCR = temp|(1<<IVSEL);
	sei();
	
	// Activate this to echo the data received from the USB interface on the BT interface
	cli();
	dbg_rx_callback=echo_dbg2bt;
	sei();
	
		
start:
	// Bootloader life sign
	system_led_set(0b111);
	_delay_ms(100);
	system_led_set(0b101);
	_delay_ms(100);

	//mfprintf(file_usb,"PCMSK0: - UBRR0: - \n",PCMSK0,UBRR0);
	
	/*
		One of three cases can occur:
		- The processor was reset/powered up: we wait for a lifesign on one of the interface
		- The bootloader was called from a hook in the application on interface USB
		- The bootloader was called from a hook in the application on interface BT
	*/
	
	// Find how we entered the bootloader
	switch(UBRR0)
	{
		case 0:
		{
			// The bootloader is entered due to processor reset or power up.
			// Display welcome message
			fputs(blstr,file_usb);
			fputs(blstr,file_bt);
			
			// Detect on which interface there is activity
			file_bl = detectbl();
			
			
			// Nothing received on any interface: jump to application code
			if(file_bl==0)
			{
				// Got to reboot
				goto reboot;
			}
			// Something received - check which interface is used. 
			// detectbl is aware of ENABLEBOOTLOADERONBLUETOOTH and will not return file_bt if ENABLEBOOTLOADERONBLUETOOTH is not enabled
			// Set the bootloader and debug interface according to where a life sign was detected
			if(file_bl==file_usb)
				file_dbg=file_bt;
			else
				file_dbg=file_usb;
			// Got to the boot loader
			goto enterboot;
		}
		case 250:
		{
			// Called from application with boot over USB interface
			file_bl = file_usb;
			file_dbg = file_bt;
			break;
		}
		#if ENABLEBOOTLOADERONBLUETOOTH
		// Only check if ENABLEBOOTLOADERONBLUETOOTH is enabled
		case 251:
		{
			// Called from application with boot over Bluetooth interface
			file_bl = file_bt;
			file_dbg = file_usb;
			break;
		}
		#endif
		default:
		{
			// Unknown code: reboot
			goto reboot;			
		}
	}
	
	// Code arrives here if the bootloader was called from a hook in the application
	// It is likely that the interface would loose a few incoming bytes due to the transition to the bootloader and reinitialisation. 
	// Therefore we send a message 'fail' to invite the programmer to re-send the command.

	// If we received a character indicative of bootloader, return immediately with an error, which will lead avrdude to resend a signon command.
	// TODO: if the code is fast enough the bootloader may be entered the character that has been received 
	//fputs("SE\n",file_bt);
	char b[64];
	b[0] = CMD_SIGN_ON;
	b[1] = STATUS_CMD_FAILED;
	sendmessage(b,2,1);

enterboot:

	mfprintf(file_usb,"file_usb: + file_bt: + file_bl: + file_dbg: +\n",file_usb,file_bt,file_bl,file_dbg);
			

	stk500();
	
	/*system_led_set(0b010);
	
	
	system_led_set(0b001);
	fputs("fake bootloader stuff 1\n",file_usb);
	_delay_ms(1000);
	system_led_set(0b010);
	fputs("fake bootloader stuff 1\n",file_usb);
	_delay_ms(1000);
	system_led_set(0b100);
	fputs("fake bootloader stuff 1\n",file_usb);
	_delay_ms(1000);*/


reboot:
	
	// Print bye message
	fputs(leavingstr,file_usb);
	fputs(leavingstr,file_bt);
	
	//system_led_set(0b101);
		
	
	
	
	


//	goto start;

	// Blink, which also waits for I/O transfers to complete
	//system_blink(20,50,0b00);
	_delay_ms(200);
	// Move interrupt vector and enable interrupts
	/*cli();
	temp = MCUCR;
	MCUCR = temp|(1<<IVCE);
	MCUCR = temp&(~(1<<IVSEL));*/
	
	system_led_set(0b110);
	
	wdt_enable(WDTO_250MS);
	
	system_led_set(0b111);
	
	while(1); // WDT will do reset
	
}





