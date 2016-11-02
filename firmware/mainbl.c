/*
	BlueSense2 Bootloader
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

FILE *file_usb,*file_bt;
volatile unsigned long time_ms=0;

#define TIMEOUTMS 10000

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

FILE *detectbl(FILE *f1,FILE *f2)
{
	unsigned long tstart,tblink;
	tblink=tstart=timer_ms_get();
	while(timer_ms_get()-tstart<(unsigned long)TIMEOUTMS)
	{
		if(fgetrxbuflevel(f1))
		{
			return f1;
		}
		if(fgetrxbuflevel(f2))
		{
			return f2;
		}		
		if(timer_ms_get()-tblink>500)
		{
			system_led_toggle(0b1);
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
	wdt_reset();
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
	fputc(c,file_bt);
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
	
	/*
		We have a hard time rebooting to the application once all the registers are initialised. 
		Therefore we set a watchdog reset, and here we detect whether we were reseted by the watchdog.
		If yes, then jump directly to application (all registers are still clear).
	*/
	mcusr=MCUSR;
	MCUSR=0;
	if(mcusr&0x8)
	{
		wdt_disable();
		app_start();
	}
	
	
	// INIT MODULE
	init_ports();
	init_timers();
	uart1_init(5,0);
	i2c_init();
	dbg_init();
	file_usb = serial_open(10,1);
	file_bt=serial_open(1,1);
	serial_setblocking(file_usb,0);
	serial_setblocking(file_bt,0);
	dbg_setnumtxbeforerx(10);
	
	// Move interrupt vector and enable interrupts
	unsigned char temp = MCUCR;
	MCUCR = temp|(1<<IVCE);
	MCUCR = temp|(1<<IVSEL);
	sei();
	
	cli();
	dbg_rx_callback=echo_dbg2bt;
	sei();
	
		
	fputs("BlueSense2 Bootloader\n",file_usb);
	fputs("BlueSense2 BTBootloader\n",file_bt);
		
	system_blink(20,100,0b00);
	
//	mfprintf(file_usb,"MCUSR: -\n",mcusr);
	
//if(mcusr&0x8)
//		goto reboot;
	
	
	unsigned char b[64];
	//mfprintf(file_bt,"UBRRO: -\n",UBRR0);
	
	// Find how we have been called: reset or from the main application
	if(UBRR0 == 0)
	{
		// Were reseted - must autodetect from which port bootloader may come
		//fputs("AD\n",file_bt);
		file_bl = detectbl(file_usb,file_bt);
		
		//msprintf(b,"Detect: +\n",file_bl);
		//fputs(b,file_bt);
		if(file_bl==0)
		{
			// Must jump to application code
			goto reboot;
		}
		if(file_bl==file_usb)
			file_dbg=file_bt;
		else
			file_dbg=file_usb;
		
	}
	if(UBRR0 == 250)
	{
		// Called from application, boot over dbg
		file_bl = file_usb;
		file_dbg = file_bt;
	}
	if(UBRR0 == 251)
	{
		// Called from application, boot over bluetooth
		file_bl = file_bt;
		file_dbg = file_usb;
	}
	
		
	if(UBRR0==250 || UBRR0==251)
	{
		//fputs("SE\n",file_bt);
		b[0] = CMD_SIGN_ON;
		b[1] = STATUS_CMD_FAILED;
		sendmessage(b,2,1);
	}
	
	fputs("Entering ISP\n",file_dbg);
	
	stk500();
	
		
	fputs("Reboot\n",file_dbg);
	
reboot:
	// Blink, which also waits for I/O transfers to complete
	system_blink(20,50,0b00);
	// Move interrupt vector and enable interrupts
	cli();
	temp = MCUCR;
	MCUCR = temp|(1<<IVCE);
	MCUCR = temp&(~(1<<IVSEL));
	
	wdt_enable(WDTO_250MS);
	while(1); // WDT will do reset
	
}





