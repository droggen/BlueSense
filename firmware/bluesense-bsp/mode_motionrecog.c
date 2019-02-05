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

#include "main.h"
#include "mpu.h"
#include "pkt.h"
#include "wait.h"
#include "lcd.h"
#include "fb.h"
#include "system.h"
#include "helper.h"
#include "serial.h"
#include "dbg.h"
#include "mode_motionrecog.h"
#include "mode_global.h"
#include "motionconfig.h"
#include "commandset.h"
#include "uiconfig.h"
#include "ufat.h"
#include "sd.h"


FILE *file_stream;		// File where to stream


#define CommandParsersMotionRecogNum 3
const COMMANDPARSER CommandParsersMotionRecog[CommandParsersMotionRecogNum] =
{ 
	{'H', CommandParserHelp,help_h},
	{'W', CommandParserSwap,help_w},
	{'!', CommandParserQuit,help_quit}
};



void print_sample(FILE *f)
{
	// Print all the channels, even if some channels are not aquired
	unsigned char str[128];
	unsigned char s;
	unsigned char *strptr = str;
	
	if(1)		// timestamp
	{
		u32toa(mpu_data_time[mpu_data_rdptr],strptr);
		strptr+=10;
		*strptr=' ';
		strptr++;
	}
	if(1)		// battery
	{
		u16toa(system_getbattery(),strptr);
		strptr+=5;
		*strptr=' ';
		strptr++;
	}
	s16toa(mpu_data_ax[mpu_data_rdptr],strptr);
	strptr+=6;
	*strptr=' ';
	strptr++;
	s16toa(mpu_data_ay[mpu_data_rdptr],strptr);
	strptr+=6;
	*strptr=' ';
	strptr++;
	s16toa(mpu_data_az[mpu_data_rdptr],strptr);
	strptr+=6;
	*strptr=' ';
	strptr++;
	s16toa(mpu_data_gx[mpu_data_rdptr],strptr);		
	strptr+=6;
	*strptr=' ';
	strptr++;
	s16toa(mpu_data_gy[mpu_data_rdptr],strptr);
	strptr+=6;
	*strptr=' ';
	strptr++;
	s16toa(mpu_data_gz[mpu_data_rdptr],strptr);		
	strptr+=6;
	*strptr='\n';		
	strptr++;

	fputbuf(f,str,strptr-str);
}



void recog_sample_start(void)
{
	// Load the configuration
	unsigned mode = ConfigLoadMotionMode();
	
	mpu_config_motionmode(mode,1);	
}
void recog_sample_stop(void)
{
	mpu_config_motionmode(MPU_MODE_OFF,0);	
}


unsigned char CommandParserMotionRecog(unsigned char *buffer,unsigned char size)
{
	printf("Parse g\n");
	
	// Check if command is ?
	if(size==1 && buffer[0]=='?')
	{
		// print help
		fprintf_P(file_pri,PSTR("Available sample modes:\n"));		
		for(unsigned char i=0;i<MOTIONCONFIG_NUM+1;i++)
		{
			char buf[64];
			strcpy_P(buf,(PGM_P)pgm_read_word(mc_options+i));
			fprintf_P(file_pri,PSTR("[%d]\t"),i);
			fputs(buf,file_pri);
			fputc('\n',file_pri);
		}		
		return 0;
	}
	// If no param, enter mode
	if(size==0)
	{
		system_mode = APP_MODE_MOTIONRECOG;
		CommandQuit=1;		// ??
		return 0;
	}
	// Mode specified
	unsigned char rv,*p1;
	rv = ParseComma(buffer,1,&p1);
	if(rv)
		return 2;
		
	// Get mode
	unsigned mode;
	if(sscanf((char*)p1,"%u",&mode)!=1)
	{
		printf("failed mode\n");
		return 2;
	}	
	if(mode>MOTIONCONFIG_NUM)
		return 2;
		
	// If mode valid and nonzero then store config
	if(mode>0)	
		ConfigSaveMotionMode(mode-1);
	
	system_mode = APP_MODE_MOTIONRECOG;
	CommandQuit=1;		// ??
	return 0;
}

/******************************************************************************
Main recognition loop

******************************************************************************/
void mode_motionrecog(void)
{
	unsigned long int t_last,t_cur,t_status,t_start,t_end;
	unsigned long totsamples=0;
	
	fprintf(file_usb,"Recognition mode\n");
	file_stream = file_pri;
	
	system_led_set(0b01);	
	// Print something on LCD
	lcd_clear565(0);
	lcd_writestring("Recognition",28,0,2,0x0000,0xffff);	

	

	recog_sample_start();		// Start sensor data acquisition
	
	t_start = timer_ms_get();
	t_status = timer_ms_get();
	while(1)
	{
		// Check if the user enters something on keyboard to quit (e.g. !)
		while(CommandProcess(CommandParsersMotionRecog,CommandParsersMotionRecogNum));		
		if(CommandShouldQuit())
			break;
		//_delay_ms(1);
		
		/*if(mpu_data_newsample)
		{
			printf("ns wk %ld. lvl: %d wrptr %d\n",wakeup,stream_buffer_level(),mpu_data_wrptr);
			mpu_data_newsample=0;
		}*/
		
		// Get how many samples are in the buffer
		unsigned char l = mpu_data_level();
		totsamples+=l;
		
		// Process each sample; remember to call stream_buffer_rdptrnext to get the next sample.
		for(unsigned char i=0;i<l;i++)
		{
			print_sample(file_stream);
			mpu_data_rdnext();	
			
		}
		
		// Print some status information at regular intervals. 
		// Remove this if it takes too much CPU time
		if((t_cur=timer_ms_get())-t_status>10000)
		{		
			fprintf_P(file_pri,PSTR("Streaming since %lums\n"),timer_ms_get()-t_start);
			fprintf_P(file_pri,PSTR(" Tot samples %lu.\n"),totsamples);
			fprintf_P(file_fb,PSTR("Streaming since %lums.\n"),timer_ms_get()-t_start);
			fprintf_P(file_fb,PSTR(" Tot samples %lu.\n"),totsamples);

			t_status = timer_ms_get();
		}
	}
	
	recog_sample_stop();		// Stop sensor data acquisition
	t_end=timer_ms_get();
	

	printf_P(PSTR("Streaming stopped. Total streaming time: %lu ms\n"),t_end-t_start);
	mpu_printstat(file_fb);
	mpu_printstat(file_usb);
		
}