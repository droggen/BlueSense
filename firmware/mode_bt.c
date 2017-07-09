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
#include "pkt.h"

#include "mode.h"
#include "mode_bench.h"

#include "commandset.h"




#define CommandParsersBTNum 2
const COMMANDPARSER CommandParsersBT[CommandParsersBTNum] =
{ 
	{'H', CommandParserHelp,help_h},
	{'!', CommandParserQuit,help_quit}
};




unsigned char CommandParserBT(char *buffer,unsigned char size)
{	
	
	CommandChangeMode(APP_MODE_BT);
	
	return 0;
}
void help(void)
{
	fprintf_P(file_usb,PSTR("The following commands are available:\n"));
	fprintf_P(file_usb,PSTR("\t111: Set speed to 115.2kbps\n"));
	fprintf_P(file_usb,PSTR("\t222: Set speed to 230.4kbps\n"));
	fprintf_P(file_usb,PSTR("\t444: Set speed to 460.8kbps\n"));
	fprintf_P(file_usb,PSTR("\t555: Set speed to 57.6kbps\n"));
	fprintf_P(file_usb,PSTR("\tRRR: BT hardware reset\n"));
	fprintf_P(file_usb,PSTR("\tDDD: Report data overrun\n"));
	fprintf_P(file_usb,PSTR("\t???: Command help\n"));
	fprintf_P(file_usb,PSTR("\tXXX: Exit\n"));
}

void mode_bt(void)
{
	unsigned char cmdchar=0;
	unsigned char cmdn=0;
	unsigned char shouldrun=1;
	int c;
	
	fprintf_P(file_usb,PSTR("bt mode\n"));
	//fprintf_P(file_pri,PSTR("bt mode\n"));
	help();	
	
	while(shouldrun)
	{
		//while(CommandProcess(CommandParsersBench,CommandParsersBenchNum));		
		//if(CommandShouldQuit())
			//break;
			
		if((c=fgetc(file_usb))!=-1)
		//if((c=fgetc(file_pri))!=-1)
		{
			cmddecodestart:
			if( (c=='1' || c=='2'|| c=='4' || c=='5' || c=='R' || c=='r' || c=='D' || c=='d' || c=='X' || c=='x' || c=='?') || cmdn)
			{
				
				if(cmdn==0)
				{
					//fprintf_P(file_usb,PSTR("<first char>\n"));
					cmdchar=c;
					cmdn++;
				}
				else
				{
					if(c==cmdchar)
					{
						cmdn++;
						//fprintf_P(file_usb,PSTR("<%d char in sequence>\n"),cmdn);
						if(cmdn==3)
						{
							// command recognised
							switch(cmdchar)
							{
								case '1':
									fprintf_P(file_usb,PSTR("<Setting serial to 115.2kbps>\n"));
									//fprintf_P(file_pri,PSTR("<Setting serial to 115.2kbps>\n"));
									uart1_init(5,0);	// 115200bps  @ 11.06 Mhz		
									break;
								case '2':
									fprintf_P(file_usb,PSTR("<Setting serial to 230.4kbps>\n"));
									//fprintf_P(file_pri,PSTR("<Setting serial to 230.4kbps>\n"));
									uart1_init(2,0);	// 230400bps  @ 11.06 Mhz
									break;
								case '4':
									fprintf_P(file_usb,PSTR("<Setting serial to 460.8kbps>\n"));
									//fprintf_P(file_pri,PSTR("<Setting serial to 460.8kbps>\n"));
									uart1_init(2,1);	// 460800bps  @ 11.06 Mhz
									break;
								case '5':
									fprintf_P(file_usb,PSTR("<Setting serial to 57.6kbps>\n"));
									//fprintf_P(file_pri,PSTR("<Setting serial to 57.6kbps>\n"));
									uart1_init(11,0);	// 57600bps @ 11.06 Mhz		
									break;
								case 'R':
								case 'r':
									fprintf_P(file_usb,PSTR("<Resetting BT>\n"));
									rn41_Reset(file_usb);
									break;
								case 'X':
								case 'x':
									fprintf_P(file_usb,PSTR("<Exit>\n"));
									//fprintf_P(file_pri,PSTR("<Exit>\n"));
									shouldrun=0;
									break;
								case 'd':
								case 'D':
									fprintf_P(file_usb,PSTR("<DOR: %lu>\n"),Serial1DOR);
									//fprintf_P(file_pri,PSTR("<DOR: %lu>\n"),Serial1DOR);
									break;
								case '?':
								default:
									help();
									break;
							}
							cmdn=0;
						}						
					}
					else
					{
						//fprintf_P(file_usb,PSTR("<%d char in sequence (%02x), current not similar (%02x)>\n"),cmdn,cmdchar,c);
						// Not same as previous characters, output previous and current
						for(unsigned char i=0;i<cmdn;i++)
							fputc(cmdchar,file_bt);
						cmdn=0;
						goto cmddecodestart;					
					}
				}
			}
			else
			{
				//fprintf(file_usb,"send %02x\n",c);
				fputc(c,file_bt);
				_delay_ms(10);
			}
		}
		//else
			//fputc('.',file_usb);
		while((c=fgetc(file_bt))!=-1)
		{
			fputc(c,file_usb);
		}
		//fprintf(file_usb,"%d\n",buffer_level(&SerialData1Rx));
		
		//_delay_ms(5);
		//_delay_ms(1);

	}
	fprintf_P(file_pri,PSTR("bt end\n"));

}

