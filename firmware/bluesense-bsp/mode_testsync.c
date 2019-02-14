#include "cpu.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/eeprom.h>
#include <util/atomic.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "global.h"
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
#include "sd.h"
#include "ufat.h"

#include "mode_sample_adc.h"

#include "commandset.h"
#include "mode_global.h"


#define CommandParsersTestSyncNum 4
const COMMANDPARSER CommandParsersTestSync[CommandParsersTestSyncNum] =
{ 
	{'Z',CommandParserSync,help_z},
	{'I', CommandParserIO,help_i},
	{'!', CommandParserQuit,help_quit},
	{'H', CommandParserHelp,help_h}
};



void mode_testsync(void)
{
	//char buffer[64];
	//WAITPERIOD p=0;
	//unsigned short v[8];
	//unsigned long time;
	//unsigned long stat_totsample=0;
	//unsigned long stat_samplesendfailed=0;
	//unsigned long stat_timemsstart=0;
	//unsigned long stat_timemsend=0;
	//unsigned long time_laststatus=0;

	
	
	
	fprintf_P(file_pri,PSTR("Test sync mode\n"));
	
	//unsigned short data[8];
	//data[0]=0;	
	
	//stat_timemsstart = timer_ms_get();
	
	unsigned long oldtime = timer_ms_get()/1000;
	unsigned long newtime,newtimeus;
		
	while(1)
	{
		while(CommandProcess(CommandParsersTestSync,CommandParsersTestSyncNum));		
		if(CommandShouldQuit())
			break;
			
		
		// Busy loop until new second
		newtime = timer_ms_get();
		if(newtime/1000!=oldtime)
		{
			newtimeus=timer_us_get();
			fprintf_P(file_pri,PSTR("%lu %lu\n"),newtime,newtimeus);	
			oldtime=newtime/1000;
		}
		
	}


	
	
}


