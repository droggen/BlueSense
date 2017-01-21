#if ENABLEGFXDEMO==1

#include "cpu.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "main.h"
#include "adc.h"
#include "serial.h"
#include "i2c.h"
#include "ds3232.h"
#include "rn41.h"
#include "mpu.h"
#include "mpu_test.h"
#include "pkt.h"
#include "wait.h"
#include "init.h"
#include "lcd.h"
#include "fb.h"
#include "uiconfig.h"
#include "helper.h"
#include "i2c_internal.h"
#include "system.h"
#include "mode_demo.h"
#include "mode_demo_clock.h"

#include "commandset.h"


/*
const char help_t[] PROGMEM ="T<hh><mm><ss>";
const char help_d[] PROGMEM ="D<dd><mm><yy>";
const char help_quit[] PROGMEM ="Exit current mode";
const char help_h[] PROGMEM ="Help";
*/

#define CommandParsersClockNum 4
const COMMANDPARSER CommandParsersClock[CommandParsersClockNum] =
{ 
	{'T', CommandParserTime,help_t},
	{'D', CommandParserDate,help_d},
	{'!', CommandParserQuit,help_quit},
	{'H', CommandParserHelp,help_h}
};




void mode_clock(void)
{
	demo_clock(0);
}
char demo_clock(unsigned long int dt)
{
	char str[32];
	unsigned char ctr=0;
	unsigned char topline = 20;
	unsigned long int t1;
	unsigned long wu=0;
	unsigned char hour,min,sec=0,lastsec=0;
	unsigned short temp;
	
	lcd_clear565(0);
	
	set_sleep_mode(SLEEP_MODE_IDLE); 
	
	t1 = timer_ms_get();
	
	while( (dt==0) || (dt!=0 && (timer_ms_get()-t1<dt) ) )
	{
		while(CommandProcess(CommandParsersClock,CommandParsersClockNum));
		
		//_delay_ms(2000);
		
		if(CommandShouldQuit())
			return 1;
		
		
		// Read DS3232
		ds3232_readtime_conv(&hour,&min,&sec);		
		// Prepare string
		sprintf(str,"%02d:%02d:%02d",hour,min,sec);		
		// Write
		lcd_writestring(str,0,topline,4,0x0000,0xffff);	
		// Write
		if(sec!=lastsec)
		{
			fprintf_P(file_pri,PSTR("%s\n"),str);
			fprintf_P(file_dbg,PSTR("%s\n"),str);
		}
		// Write the milliseconds
		unsigned long int ms = timer_ms_get();
		ms = ms%1000;
		sprintf(str,"%03d",(int)ms);
		lcd_writestring(str,104,topline+24,2,0x0000,0xffff);	
		//_delay_ms(1);
		
		//printf_P(PSTR("...\n"));
		//fprintf_P(file_bt,PSTR("...\n"));
		
		
		sleep_enable();	
		sleep_cpu();
		sleep_disable();
		wu++;
		
		if(lastsec!=sec)
		{
			sprintf(str,"%04ld",wu);		
			lcd_writestring(str,112,122,1,0x0000,0x7f7f7f);	
		
			unsigned long temp = system_gettemperature();
			sprintf(str,"%02d.%02d  C",(signed short)(temp/100),(signed short)(temp%100));
			lcd_writestring(str,32,topline+88,2,0x0000,0xffff);	
			fprintf_P(file_pri,PSTR("%s\n"),str);
			fprintf_P(file_dbg,PSTR("%s\n"),str);
			
			signed short b = system_getbattery();
			//printf("b: %d\n",b);
		
			sprintf(str,"%04d mV",b);
			lcd_writestring(str,0,122,1,0x0000,0x7f7f7f);
			fprintf_P(file_pri,PSTR("%s\n"),str);
			fprintf_P(file_dbg,PSTR("%s\n"),str);


			unsigned char d,m,y;
			ds3232_readdate_conv(&d,&m,&y);
			sprintf(str,"%02d.%02d.%02d",d,m,y);
			lcd_writestring(str,16,topline+60,3,0x0000,0xffff);	
			fprintf_P(file_pri,PSTR("%s\n"),str);
			fprintf_P(file_dbg,PSTR("%s\n"),str);
		}
		
		ctr++;
		lastsec=sec;		
	}
	return 0;
}



#endif
