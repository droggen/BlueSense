/*
	file: mode_adc
	
	ADC sampling and streaming/logging mode.
	
	This function contains the 'A' mode of the sensor which allows to acquire multiple ADC channels and stream or log them. 
	
	*TODO*
	
	* Statistics when logging could display log-only information (samples acquired, samples lost, samples per second)
*/

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
#include "pkt.h"
#include "wait.h"
#include "uiconfig.h"
#include "helper.h"
#include "system.h"
#include "pkt.h"
#include "ufat.h"
#include "mode.h"
#include "mode_global.h"
#include "mode_sample.h"
#include "mode_sample_adc.h"
#include "commandset.h"


unsigned long mode_adc_period;
unsigned char mode_adc_mask;


const char help_adcpullup[] PROGMEM="P,<on>: if on=1, activates pullups on user ADC inputs (channels 0-3), otherwise deactivate. In principle pullups should be deactivated.";

const COMMANDPARSER CommandParsersADC[] =
{ 
	{'H', CommandParserHelp,help_h},
//	{'A', CommandParserADC,help_a},
	{'N', CommandParserAnnotation,help_annotation},
	{'F', CommandParserStreamFormat,help_f},
//	{'W', CommandParserSwap,help_w},
	{'L', CommandParserSampleLog,help_samplelog},
	{'P', CommandParserADCPullup,help_adcpullup},
	{'Z',CommandParserSync,help_z},
	{'i',CommandParserInfo,help_info},
	{'!', CommandParserQuit,help_quit}
};
const unsigned char CommandParsersADCNum=sizeof(CommandParsersADC)/sizeof(COMMANDPARSER); 



/*
	Parsers must return:
		0:	Message execution ok (message valid)
		1:	Message execution error (message valid)
		2:	Message invalid 		
*/

/******************************************************************************
	function: CommandParserADCPullup
*******************************************************************************	
	Parses a user command to enable/disable the pull-ups on the ADC inputs.

	Command format: P,<on>
	
	Activates or desactivates the pull-up.
	
	Parameters:
		buffer			-			Buffer containing the command
		size			-			Length of the buffer containing the command
			
******************************************************************************/
unsigned char CommandParserADCPullup(char *buffer,unsigned char size)
{
	unsigned char rv;
	unsigned on;
	rv = ParseCommaGetInt((char*)buffer,1,&on);
	if(rv)
		return 2;
	if(!(on==0 || on==1))
		return 2;
	if(on==0)
		system_adcpu_off();
	else
		system_adcpu_on();
	
	return 0;
}



/******************************************************************************
	function: CommandParserADC
*******************************************************************************	
	Parses a user command to enter the ADC mode.
	
	Command format: A,<mask>,<period>
	
	Stores the mask and period in eeprom.
	
	Parameters:
		buffer			-			Buffer containing the command
		size			-			Length of the buffer containing the command
			
******************************************************************************/
unsigned char CommandParserADC(char *buffer,unsigned char size)
{
	unsigned char rv;
	char *p1,*p2;
	rv = ParseComma((char*)buffer,2,&p1,&p2);
	if(rv)
		return 2;
		
	// Get mask
	unsigned int mask;
	if(sscanf(p1,"%u",&mask)!=1)
	{
		return 2;
	}	
	if(mask>0xff || mask==0)
	{
		return 2;
	}
	// Get period
	unsigned long period;
	if(sscanf(p2,"%lu",&period)!=1)
	{
		return 2;
	}
	
	fprintf_P(file_pri,PSTR("mask: %02x. period: %lu\n"),mask,period);
		
	//ConfigSaveADCMask(mask);
	//ConfigSaveADCPeriod(period);
	mode_adc_period=period;
	mode_adc_mask=mask;
	
	CommandChangeMode(APP_MODE_ADC);

		
	return 0;
}




/******************************************************************************
	function: mode_adc
*******************************************************************************	
	ADC mode loop.
	
	This function initialises the board for ADC acquisition and enters a continuous 
	sample/stream loop.
	
******************************************************************************/
void mode_adc(void)
{
	char buffer[128];			// Should be at least 128 
	PACKET packet;
	WAITPERIOD p=0;
	unsigned long time;
	unsigned long stat_totsample=0;
	unsigned long stat_samplesendfailed=0;
	unsigned long stat_timemsstart=0;
	unsigned long stat_timemsend=0;
	unsigned long time_laststatus=0;
	unsigned char numchannels;
	unsigned char enableinfo;
	unsigned char putbufrv;
	unsigned short pktctr=0;
	
	
	mode_sample_file_log=0;

	// Load mode configuration
	mode_stream_format_bin=ConfigLoadStreamBinary();
	mode_stream_format_ts=ConfigLoadStreamTimestamp();
	mode_stream_format_bat=ConfigLoadStreamBattery();
	mode_stream_format_pktctr=ConfigLoadStreamPktCtr();
	mode_stream_format_label = ConfigLoadStreamLabel();
	enableinfo = ConfigLoadEnableInfo();
	
	// Some info
	fprintf_P(file_pri,PSTR("ADC mode: period: %lu mask: %02X binary: %d timestamp: %d battery: %d label: %d\n"),mode_adc_period,mode_adc_mask,mode_stream_format_bin,mode_stream_format_ts,mode_stream_format_bat,mode_stream_format_label);	
	
	
	
	// Compute the number of channels from the mask
	numchannels = __builtin_popcount(mode_adc_mask);
	
	
	// Update the current annotation
	CurrentAnnotation=0;
	
	// Set the ADC prescaler; for this hardware: 11.0592MHz/64=172.8KHz, which is in the 50KHz-200KHz range recommended by the AVR datasheet
	ADCSetPrescaler(ADCCONV_PRESCALER_64);	
	//ADCSetPrescaler(ADCCONV_PRESCALER_128);	
	

	/*printf("DDRA: %02X\n",DDRA);
	printf("PORTA: %02X\n",PORTA);
	printf("PINA: %02X\n",PINA);
	printf("deact\n");*/
	
	// Deactivate the pull-ups on all the ADC inputs
	system_adcpu_off();
	
	/*printf("DDRA: %02X\n",DDRA);
	printf("PORTA: %02X\n",PORTA);
	printf("PINA: %02X\n",PINA);*/
	
	
	set_sleep_mode(SLEEP_MODE_IDLE); 
	
	
	// Packet init
	packet_init(&packet,"DXX",3);
	
	// Get the current time in us and ms
	stat_timemsstart = timer_ms_get();	
	time_laststatus = timer_us_get();
	while(1)
	{
		// Get user commands
		while(CommandProcess(CommandParsersADC,CommandParsersADCNum))
		{	
			// Reset the time period, in case the command was the "time sync" command
			p=0;
		}
		if(CommandShouldQuit())
			break;

		// TOFIX
		// Check that we do not overrun the maximum duration allowed by 32-bit in uS (4'000'000=1h6mn).
		if(p>4000000000)
		{
			fprintf_P(file_pri,PSTR("Maximum timing duration reached; interrupting\n"));
			break;
		}
			
					
		// Periodic wait - timer_waitperiod_us returns the current time in us
		time = timer_waitperiod_us(mode_adc_period,&p);
		
		
		// Display info if enabled
		if(enableinfo)
		{
			// TOFIX
			// Time in us wraps around every 1.19 hours
			if(time-time_laststatus>10000000)
			{
				char str[128];
				sprintf_P(str,PSTR("ADC mode. Samples: %lu in %lu ms. Sample error: %lu. Log size: %lu\n"),stat_totsample,timer_ms_get()-stat_timemsstart,stat_samplesendfailed,ufat_log_getsize());
				fputbuf(file_pri,str,strlen(str));
				fputbuf(file_dbg,str,strlen(str));
				time_laststatus = time;
			}
		}
		
		// Initiate ADC conversion using double buffering
		unsigned short *data = ADCReadDoubleBuffered(mode_adc_mask);
	
	
		// Send data to primary stream or to log if available
		FILE *file_stream;
		if(mode_sample_file_log)
			file_stream=mode_sample_file_log;
		else
			file_stream=file_pri;
		

		// Encode the samples
		if(!mode_stream_format_bin)
		{
			// Plain text encoding
			char *bufferptr=buffer;
			if(mode_stream_format_pktctr)
			{
				bufferptr=format1u16(bufferptr,pktctr);
			}
			if(mode_stream_format_ts)
			{
				bufferptr = format1u32(bufferptr,time);
			}
			if(mode_stream_format_bat)
			{
				bufferptr=format1u16(bufferptr,system_getbattery());
			}
			if(mode_stream_format_label)
			{
				bufferptr=format1u16(bufferptr,CurrentAnnotation);
			}
			for(unsigned i=0;i<numchannels;i++)
			{
				bufferptr=format1u16(bufferptr,data[i]);
			}
			*bufferptr='\n';
			bufferptr++;
			putbufrv = fputbuf(file_stream,buffer,bufferptr-buffer);		
		}		
		else
		{
			// Packet encoding
			packet_reset(&packet);
			if(mode_stream_format_pktctr)
			{
				packet_add16_little(&packet,pktctr);
			}
			if(mode_stream_format_ts)
			{
				packet_add16_little(&packet,time&0xffff);
				packet_add16_little(&packet,(time>>16)&0xffff);
			}
			if(mode_stream_format_bat)
			{
				packet_add16_little(&packet,system_getbattery());
			}
			if(mode_stream_format_label)
			{
				packet_add16_little(&packet,CurrentAnnotation);
			}
			for(unsigned i=0;i<numchannels;i++)
			{
				packet_add16_little(&packet,data[i]);
			}
			packet_end(&packet);
			packet_addchecksum_fletcher16_little(&packet);
			int s = packet_size(&packet);
			putbufrv = fputbuf(file_stream,(char*)packet.data,s);
		}
		
		// Update the statistics in case of errors
		if(putbufrv)
		{
			// There was an error in fputbuf: increment the number of samples failed to send.			
			stat_samplesendfailed++;
			// Check whether the fputbuf was done on a log file; in which case close the log file.
			if(file_stream==mode_sample_file_log)
			{
				mode_sample_logend();
				fprintf_P(file_pri,PSTR("ADC mode: log file full\n"));
			}
		}
		// Update the overall statistics
		stat_totsample++;

		// Increment packet counter
		pktctr++;
	}
	stat_timemsend = timer_ms_get();
	
	// End the logging, if logging was ongoing
	mode_sample_logend();
	
	// Enable the pull-ups on the ADC
	system_adcpu_on();
	
	// Print statistics
	unsigned long sps = stat_totsample*1000/(stat_timemsend-stat_timemsstart);
	char str[128];
	sprintf_P(str,PSTR("ADC mode end. Samples: %lu in %lu ms (%lu samples/sec). Samples not transmitted: %lu (%lu %%)\n"),stat_totsample,stat_timemsend-stat_timemsstart,sps,stat_samplesendfailed,stat_samplesendfailed*100/stat_totsample);
	fputbuf(file_pri,str,strlen(str));
	fputbuf(file_dbg,str,strlen(str));
}






