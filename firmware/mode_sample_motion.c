/*
	file: mode_motionstream
	
	Sampling and streaming/logging of the motion sensor.
	
	This function contains the 'M' mode of the sensor which allows to acquire the data from the motion sensor.
	
	This mode contains conditional codepath depending on the #defines FIXEDPOINTQUATERNION, FIXEDPOINTQUATERNIONSHIFT and ENABLEQUATERNION.
	
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
#include "mode_sample.h"
#include "mode_sample_motion.h"
#include "mode_global.h"
#include "motionconfig.h"
#include "commandset.h"
#include "uiconfig.h"
#include "ufat.h"
#include "sd.h"
#include "mode.h"
#include "MadgwickAHRS.h"


// If quaternions are enabled: conversion factors
#if ENABLEQUATERNION==1
#if FIXEDPOINTQUATERNION==1
FIXEDPOINTTYPE atog=1.0k/16384.0k;
#else
float atog=1.0/16384.0;
#endif
#endif

unsigned char enableinfo;

unsigned long stat_samplesendfailed;

const COMMANDPARSER CommandParsersMotionStream[] =
{ 
	{'H', CommandParserHelp,help_h},
	//{'W', CommandParserSwap,help_w},
	{'F', CommandParserStreamFormat,help_f},
	{'L', CommandParserSampleLog,help_samplelog},
	{'Z',CommandParserSync,help_z},
	{'i',CommandParserInfo,help_info},
	{'N', CommandParserAnnotation,help_annotation},
	{'!', CommandParserQuit,help_quit}
};
const unsigned char CommandParsersMotionStreamNum=sizeof(CommandParsersMotionStream)/sizeof(COMMANDPARSER); 



// Builds the text string
unsigned char stream_sample_text(FILE *f)
{
	char motionstream[128];		// Buffer to build the string of motion data
	char *strptr = motionstream;
	
	// Format packet counter
	if(mode_stream_format_pktctr)
	{
		strptr=format1u16(strptr,mpu_data_packetctr[mpu_data_rdptr]);
	}
	// Format timestamp
	if(mode_stream_format_ts)
	{
		strptr = format1u32(strptr,mpu_data_time[mpu_data_rdptr]);
	}
	// Format battery
	if(mode_stream_format_bat)
	{
		strptr = format1u16(strptr,system_getbattery());
	}
	// Format label
	if(mode_stream_format_label)
	{
		strptr=format1u16(strptr,CurrentAnnotation);
	}
	// Formats acceleration if selected
	if(sample_mode & MPU_MODE_BM_A)
		strptr = format3s16(strptr,mpu_data_ax[mpu_data_rdptr],mpu_data_ay[mpu_data_rdptr],mpu_data_az[mpu_data_rdptr]);
	// Formats gyro if selected
	if(sample_mode & MPU_MODE_BM_G)
		strptr = format3s16(strptr,mpu_data_gx[mpu_data_rdptr],mpu_data_gy[mpu_data_rdptr],mpu_data_gz[mpu_data_rdptr]);
	// Formats magnetic if selected
	if(sample_mode & MPU_MODE_BM_M)
		strptr = format3s16(strptr,mpu_data_mx[mpu_data_rdptr],mpu_data_my[mpu_data_rdptr],mpu_data_mz[mpu_data_rdptr]);
	// Formats quaternions if selected
	
	if(sample_mode & MPU_MODE_BM_Q)
	{
		#if ENABLEQUATERNION==1
			strptr = format4f16(strptr,q0,q1,q2,q3);
		#else
			*strptr='0';
			strptr++;
			*strptr=' ';
			strptr++;
			*strptr='0';
			strptr++;
			*strptr=' ';
			strptr++;
			*strptr='0';
			strptr++;
			*strptr=' ';
			strptr++;
			*strptr='0';
			strptr++;
			*strptr=' ';
			strptr++;
		#endif
	}	
	*strptr='\n';		
	strptr++;

	if(fputbuf(f,motionstream,strptr-motionstream))
		return 1;
	return 0;	
}
unsigned char stream_sample_bin(FILE *f)
{
	PACKET p;
	packet_init(&p,"DXX",3);
	
	
	// Format packet counter
	if(mode_stream_format_pktctr)
	{
		packet_add16_little(&p,mpu_data_packetctr[mpu_data_rdptr]);
	}
	// Format timestamp
	if(mode_stream_format_ts)
	{
		packet_add16_little(&p,mpu_data_time[mpu_data_rdptr]&0xffff);
		packet_add16_little(&p,(mpu_data_time[mpu_data_rdptr]>>16)&0xffff);
	}
	// Format battery
	if(mode_stream_format_bat)
		packet_add16_little(&p,system_getbattery());
	if(mode_stream_format_label)
		packet_add16_little(&p,CurrentAnnotation);
		
	// Formats acceleration if selected
	if(sample_mode & MPU_MODE_BM_A)
	{
		packet_add16_little(&p,mpu_data_ax[mpu_data_rdptr]);
		packet_add16_little(&p,mpu_data_ay[mpu_data_rdptr]);
		packet_add16_little(&p,mpu_data_az[mpu_data_rdptr]);
	}
	// Formats gyro if selected
	if(sample_mode & MPU_MODE_BM_G)
	{
		packet_add16_little(&p,mpu_data_gx[mpu_data_rdptr]);
		packet_add16_little(&p,mpu_data_gy[mpu_data_rdptr]);
		packet_add16_little(&p,mpu_data_gz[mpu_data_rdptr]);
	}
	// Formats magnetic if selected
	if(sample_mode & MPU_MODE_BM_M)
	{
		packet_add16_little(&p,mpu_data_mx[mpu_data_rdptr]);
		packet_add16_little(&p,mpu_data_my[mpu_data_rdptr]);
		packet_add16_little(&p,mpu_data_mz[mpu_data_rdptr]);
	}
	// Formats quaternions if selected
	if(sample_mode & MPU_MODE_BM_Q)
	{	
		#if ENABLEQUATERNION==1
		_Accum k;
		signed short v;
		k = q0*10000k; v = k;
		packet_add16_little(&p,v);
		k = q1*10000k; v = k;
		packet_add16_little(&p,v);
		k = q2*10000k; v = k;
		packet_add16_little(&p,v);
		k = q3*10000k; v = k;
		packet_add16_little(&p,v);	
		#else
		packet_add16_little(&p,1);
		packet_add16_little(&p,0);
		packet_add16_little(&p,0);
		packet_add16_little(&p,0);
		#endif
	}
	packet_end(&p);
	packet_addchecksum_fletcher16_little(&p);
	int s = packet_size(&p);
	if(fputbuf(f,(char*)p.data,s))
		return 1;
	return 0;
}



unsigned char stream_sample(FILE *f)
{
	if(mode_stream_format_bin==0)
		return stream_sample_text(f);
	else
		return stream_sample_bin(f);
	return 0;
}




/*void stream_status(FILE *file)
{
	fprintf_P(file,PSTR("Battery: %ld mV\n"),system_getbattery());	
}*/

void stream_start(void)
{
	// Load the configuration
	unsigned mode = ConfigLoadMotionMode();
	mode_stream_format_bin=ConfigLoadStreamBinary();
	mode_stream_format_ts=ConfigLoadStreamTimestamp();
	mode_stream_format_bat=ConfigLoadStreamBattery();
	mode_stream_format_pktctr=ConfigLoadStreamPktCtr();
	mode_stream_format_label = ConfigLoadStreamLabel();
	enableinfo = ConfigLoadEnableInfo();
	
	
	mpu_config_motionmode(mode,1);	
	
	//mpu_setgyroscale(MPU_GYR_SCALE_250);
	//mpu_setgyroscale(MPU_GYR_SCALE_1000);
	mpu_setgyroscale(MPU_GYR_SCALE_2000);
	
}
void stream_stop(void)
{
	mpu_config_motionmode(MPU_MODE_OFF,0);
}


unsigned char CommandParserMotion(unsigned char *buffer,unsigned char size)
{
	unsigned char rv;
	int mode;
	
	rv = ParseCommaGetInt((char*)buffer,1,&mode);
	if(rv)
	{
		// print help
		mpu_printmotionmode(file_pri);
		return 2;
	}
	if(mode<0 || mode>MOTIONCONFIG_NUM)
		return 2;
		
	// If mode valid then store config
	ConfigSaveMotionMode(mode);
	
	CommandChangeMode(APP_MODE_MOTIONSTREAM);
	return 0;
}

/******************************************************************************
	function: mode_motionstream
*******************************************************************************	
	Streaming mode loop	
******************************************************************************/
void mode_motionstream(void)
{
	unsigned long wakeup=0;
	unsigned long int t_cur,time_laststatus,stat_timemsstart,t_end;
	unsigned char putbufrv;
	unsigned long stat_totsample=0;
	
	
	system_led_set(0b01);
	
	//lcd_clear565(0);
	//lcd_writestring("Streaming",28,0,2,0x0000,0xffff);	

	set_sleep_mode(SLEEP_MODE_IDLE); 

	mode_sample_file_log=0;


	stream_start();
	
	stat_timemsstart = timer_ms_get();
	
	stat_samplesendfailed=0;
	
	
	
	time_laststatus = stat_timemsstart = timer_ms_get();
	while(1)
	{
		while(CommandProcess(CommandParsersMotionStream,CommandParsersMotionStreamNum));		
		if(CommandShouldQuit())
			break;
		_delay_ms(1);
		
		
		// Display info if enabled
		if(enableinfo)
		{
			// TOFIX
			// Time in us wraps around every 1.19 hours
			if(timer_ms_get()-time_laststatus>10000)
			{
				char str[128];
				sprintf_P(str,PSTR("Motion mode. Samples: %lu in %lu ms. Sample error: %lu. Log size: %lu\n"),stat_totsample,timer_ms_get()-stat_timemsstart,stat_samplesendfailed,ufat_log_getsize());
				fputbuf(file_pri,str,strlen(str));
				fputbuf(file_dbg,str,strlen(str));
				time_laststatus+=10000;
			}
		}
		
		
		
		// Stream
		unsigned char l = mpu_data_level();
		
	
		for(unsigned char i=0;i<l;i++)
		{
			// Compute the quaternions if in a quaternion mode
			#if ENABLEQUATERNION==1
			if(sample_mode==MPU_MODE_ACCGYRMAGQ || sample_mode==MPU_MODE_Q)
			{
				#if FIXEDPOINTQUATERNION==1
				FIXEDPOINTTYPE ax,ay,az,gx,gy,gz;
				ax = mpu_data_ax[mpu_data_rdptr]*atog;
				ay = mpu_data_ay[mpu_data_rdptr]*atog;
				az = mpu_data_az[mpu_data_rdptr]*atog;
				gx = mpu_data_gx[mpu_data_rdptr]*mpu_gtor;
				gy = mpu_data_gy[mpu_data_rdptr]*mpu_gtor;
				gz = mpu_data_gz[mpu_data_rdptr]*mpu_gtor;				
				// Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of the magnetometer;
				// the magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
				MadgwickAHRSupdate(gx,gy,gz,ax,ay,az,	-mpu_data_my[mpu_data_rdptr],
														-mpu_data_mx[mpu_data_rdptr],
														mpu_data_mz[mpu_data_rdptr]);
				#else
				float ax,ay,az,gx,gy,gz;
				ax = mpu_data_ax[mpu_data_rdptr]*atog;
				ay = mpu_data_ay[mpu_data_rdptr]*atog;
				az = mpu_data_az[mpu_data_rdptr]*atog;
				gx = mpu_data_gx[mpu_data_rdptr]*mpu_gtor;
				gy = mpu_data_gy[mpu_data_rdptr]*mpu_gtor;
				gz = mpu_data_gz[mpu_data_rdptr]*mpu_gtor;				
				// Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of the magnetometer;
				// the magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
				MadgwickAHRSupdate(gx,gy,gz,ax,ay,az,	-mpu_data_my[mpu_data_rdptr],
														-mpu_data_mx[mpu_data_rdptr],
														mpu_data_mz[mpu_data_rdptr]);
				#endif
			}
			#endif
			
			// Send data to primary stream or to log if available
			FILE *file_stream;
			if(mode_sample_file_log)
				file_stream=mode_sample_file_log;
			else
				file_stream=file_pri;

			// Send the samples and check for error
			putbufrv = stream_sample(file_stream);
			
			// Update the statistics in case of errors
			if(putbufrv)
			{
				// There was an error in fputbuf: increment the number of samples failed to send.			
				stat_samplesendfailed++;
				// Check whether the fputbuf was done on a log file; in which case close the log file.
				if(file_stream==mode_sample_file_log)
				{
					mode_sample_logend();
					fprintf_P(file_pri,PSTR("Motion mode: log file full\n"));
				}
			}
			stat_totsample++;
			
			
			mpu_data_rdnext();
			//_delay_ms(5);
		}
		
		
		
		
		
		wakeup++;
		
		
		//ctr++;
		
		
		if((t_cur=timer_ms_get())-time_laststatus>1000)
		{		
			//fprintf_P(file_pri,PSTR("Streaming since %lums\n"),timer_ms_get()-stat_timemsstart);
			
			system_led_toggle(0b100);
			
			//fprintf_P(file_fb,PSTR("wakeups: %u\n"),ctr);
			
			// Display status info
			
			//stream_losses_short(file_fb);
			//stream_status(file_fb);
		
		
			//printf("Bat: %d\n",system_getbattery());
		
		
			// Do CPU benchmark		
			/*perf = main_perfbench();
			sprintf(s,"CPU free: %lu\n",perf*100l/system_perf);
			fputs(s,file_fb);		*/
			
			time_laststatus = timer_ms_get();
			
			//ctr=0;
		}
	}
	
	stream_stop();
	
	t_end=timer_ms_get();
	
	// End the logging, if logging was ongoing
	mode_sample_logend();
	

	printf_P(PSTR("Streaming stopped. Total streaming time: %lu ms\n"),t_end-stat_timemsstart);
	printf_P(PSTR("Put buffer errors: %lu\n"),stat_samplesendfailed);
	
	printf_P(PSTR("wakeups: %u\n"),wakeup);
	mpu_printstat(file_pri);
	

	
}



