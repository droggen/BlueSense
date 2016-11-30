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
#include "sd.h"
#include "ufat.h"
#include "mode.h"
#include "mode_teststream.h"

#include "commandset.h"
#include "mode_global.h"

FILE *file_log;				// Log file
FILE *file_stream;		// File where to stream


const char help_tslog[] PROGMEM="L[,<lognum>]: Stops streaming and start logging to 'lognum' (0 by default); if logging is already in progress stops logging and resumes streaming";

#define CommandParsersTSNum 6
const COMMANDPARSER CommandParsersTS[CommandParsersTSNum] =
{ 
	{'H', CommandParserHelp,help_h},
	{'S', CommandParserTeststream,help_s},
	{'F', CommandParserStreamFormat,help_f},
	{'W', CommandParserSwap,help_w},
	{'L', CommandParserTSLog,help_tslog},
	{'!', CommandParserQuit,help_quit}
};

unsigned long mode_ts_period=1000000;

/*
	Parsers must return:
		0:	Message execution ok (message valid)
		1:	Message execution error (message valid)
		2:	Message invalid 		
*/


unsigned char CommandParserTSLog(char *buffer,unsigned char size)
{
	unsigned int lognum;
	if(size==0)
	{
		lognum=0;
	}
	else
	{
		unsigned char rv;
		char *p1;
		rv = ParseComma(buffer,1,&p1);
		if(rv)
			return 2;
		
		// Get lognum
		if(sscanf(p1,"%u",&lognum)!=1)
		{
			printf("failed lognum\n");
			return 2;
		}	
	}
	
	if(file_log==0)
	{
		printf("Starting log on %u\n",lognum);
		// Initialise the SD card, the filesystem and open the log
		unsigned char rv = ufat_init();
		if(rv!=0)
		{
			printf_P(PSTR("Init SD failure (%d)\r"),rv);
			return 1;
		}
		file_log = ufat_log_open(lognum);
		if(!file_log)
		{
			printf("Error opening log\n");
			return 1;
		}
		log_printstatus();
		file_stream=file_log;
	}
	else
	{
		printf("Terminating logging\n");
		ufat_log_close();
		file_log=0;
		file_stream=file_pri;
	}
	
	return 0;
}


unsigned char CommandParserTeststream(char *buffer,unsigned char size)
{
	unsigned char rv;
	char *p1;
	rv = ParseComma(buffer,1,&p1);
	if(rv)
		return 2;
		
	// Get period
	unsigned long period;
	if(sscanf(p1,"%lu",&period)!=1)
	{
		printf("failed period\n");
		return 2;
	}
	
	mode_ts_period = period;	
	fprintf_P(file_pri,PSTR("period: %lu\n"),period);
		
	ConfigSaveTSPeriod(mode_ts_period);
	
	CommandChangeMode(APP_MODE_TS);
		
	return 0;
}






void mode_teststream(void)
{
	char buffer[64];
	PACKET packet;
	WAITPERIOD p=0;
	//unsigned short v[8];
	unsigned long time;
	unsigned long stat_totsample=0;
	unsigned long stat_samplesendfailed=0;
	unsigned long stat_timemsstart=0;
	unsigned long stat_timemsend=0;
	//unsigned long time_laststatus=0;


	// Load mode configuration
	mode_ts_period=ConfigLoadTSPeriod();
	mode_stream_format_bin=ConfigLoadStreamBinary();
	mode_stream_format_ts=ConfigLoadStreamTimestamp();
	mode_stream_format_bat=ConfigLoadStreamBattery();
	
	fprintf_P(file_pri,PSTR("Teststream mode start: period: %lu binary: %d timestamp: %d battery: %d\n"),mode_ts_period,mode_stream_format_bin,mode_stream_format_ts,mode_stream_format_bat);	
	
	
	
	set_sleep_mode(SLEEP_MODE_IDLE); 
	
	// Packet init
	packet_init(&packet,"DXX",3);
	
	file_stream=file_pri;
	
	unsigned short data[8];
	data[0]=0;	
	
	stat_timemsstart = timer_ms_get();
	while(1)
	{
	//	fprintf_P(file_pri,PSTR("Some ADC stuff: %d. period: %d. mask: %02X. pri: %p dbg: %p\n"),ctr,mode_ts_period,mode_adc_mask,file_pri,file_dbg);

		while(CommandProcess(CommandParsersTS,CommandParsersTSNum));		
		if(CommandShouldQuit())
			break;
			

		
		// Period
		time = timer_waitperiod_us(mode_ts_period,&p);
		
		
		
		// Display debug status
		/*if(time-time_laststatus>10000000)
		{
			fprintf_P(file_dbg,PSTR("ADC mode. file_pri: %p. file_dbg: %p. Samples: %lu in %lu ms. Samples not transmitted: %lu\n"),file_pri,file_dbg,stat_totsample,timer_ms_get()-stat_timemsstart,stat_samplesendfailed);
			time_laststatus = time;
		}*/
		
		// Get time from rtc
		unsigned long trtc;
		cli();
		trtc=_timer_time_1_in_ms;
		sei();
		
		// Generate some data
		data[0]++;
		
		
		// Process buffer 1-adc_data_isample
		unsigned char n = 0;
		
		if(!mode_stream_format_bin)
		{
			char *bufferptr=buffer;
			if(mode_stream_format_ts)
			{
				u32toa(time,bufferptr);
				bufferptr+=10;
				*bufferptr=' ';
				bufferptr++;
			}
			// time rtc
			if(mode_stream_format_ts)
			{
				u32toa(trtc,bufferptr);
				bufferptr+=10;
				*bufferptr=' ';
				bufferptr++;
			}
			if(mode_stream_format_bat)
			{
				u16toa(system_getbattery(),bufferptr);
				bufferptr+=5;
				*bufferptr=' ';
				bufferptr++;
			}
			for(unsigned i=0;i<n;i++)
			{
				u16toa(data[i],bufferptr);
				bufferptr+=5;
				*bufferptr=' ';
				bufferptr++;
			}
			*bufferptr='\n';
			bufferptr++;
			if(fputbuf(file_stream,buffer,bufferptr-buffer))
				stat_samplesendfailed++;
		}		
		else
		{
			// Packet mode			
			packet_reset(&packet);
			if(mode_stream_format_ts)
			{
				packet_add16_little(&packet,time&0xffff);
				packet_add16_little(&packet,(time>>16)&0xffff);
			}
			if(mode_stream_format_bat)
			{
				packet_add16_little(&packet,system_getbattery());
			}
			for(unsigned i=0;i<n;i++)
			{
				packet_add16_little(&packet,data[i]);
			}
			packet_end(&packet);
			packet_addchecksum_fletcher16_little(&packet);
			int s = packet_size(&packet);
			if(fputbuf(file_stream,(char*)packet.data,s))
				stat_samplesendfailed++;
		}
		stat_totsample++;	
		
		//fprintf(file_dbg,"bat: %d connected: %d\n",system_getbattery(),system_isusbconnected());
		
	}
	stat_timemsend = timer_ms_get();
	
	if(file_log)
	{
		file_log=0;
		ufat_log_close();
	}
	
	system_adcpu_on();
	
	// Print statistics
	unsigned long sps = stat_totsample*1000/(stat_timemsend-stat_timemsstart);
	fprintf_P(file_pri,PSTR("ADC mode end. Samples: %lu in %lu ms (%lu samples/sec). Samples not transmitted: %lu (%lu %%)\n"),stat_totsample,stat_timemsend-stat_timemsstart,sps,stat_samplesendfailed,stat_samplesendfailed*100/stat_totsample);
	fprintf_P(file_dbg,PSTR("ADC mode end. Samples: %lu in %lu ms (%lu samples/sec). Samples not transmitted: %lu (%lu %%)\n"),stat_totsample,stat_timemsend-stat_timemsstart,sps,stat_samplesendfailed,stat_samplesendfailed*100/stat_totsample);
	
	_delay_ms(1000);
	fprintf_P(file_pri,PSTR("ADCSRA: %02X\n"),ADCSRA);
	fprintf_P(file_dbg,PSTR("ADCSRA: %02X\n"),ADCSRA);
	
	
	return;
}

