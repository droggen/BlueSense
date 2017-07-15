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
#include "ltc2942.h"


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
unsigned long stat_totsample;
unsigned long stat_timemsstart,stat_t_cur,stat_wakeup,stat_time_laststatus;


typedef struct {
	unsigned long t;
	signed short mW,mA,mV;
} BATSTAT;
#define MAXBATSTATPTR 150
BATSTAT batstat[MAXBATSTATPTR];
unsigned short batstatptr;

MPUMOTIONDATA mpumotiondata;


const char help_samplestatus[] PROGMEM="Battery and logging status";
const char help_batbench[] PROGMEM="Battery benchmark";

const COMMANDPARSER CommandParsersMotionStream[] =
{ 
	{'H', CommandParserHelp,help_h},
	//{'W', CommandParserSwap,help_w},
	{'F', CommandParserStreamFormat,help_f},
	{'L', CommandParserSampleLog,help_samplelog},
	{'Z',CommandParserSync,help_z},
	{'i',CommandParserInfo,help_info},
	{'N', CommandParserAnnotation,help_annotation},
	{'q', CommandParserBatteryInfo,help_battery},
	{'s', CommandParserSampleStatus,help_samplestatus},
	{'x', CommandParserBatBench,help_batbench},
	{'!', CommandParserQuit,help_quit}
};
const unsigned char CommandParsersMotionStreamNum=sizeof(CommandParsersMotionStream)/sizeof(COMMANDPARSER); 

unsigned char CommandParserSampleStatus(char *buffer,unsigned char size)
{
	stream_status(file_pri);
	return 0;
}
void printbatstat(FILE *f)
{
	for(unsigned short i=0;i<batstatptr;i++)
	fprintf_P(f,PSTR("%d %lu %d %d %d\n"),i,batstat[i].t,batstat[i].mV,batstat[i].mA,batstat[i].mW);
}

unsigned char CommandParserBatBench(char *buffer,unsigned char size)
{
	printbatstat(file_pri);
	return 0;
}

// Builds the text string
unsigned char stream_sample_text(FILE *f)
{
	char motionstream[192];		// Buffer to build the string of motion data
	char *strptr = motionstream;
	
	// Format packet counter
	if(mode_stream_format_pktctr)
	{
		strptr=format1u16(strptr,mpumotiondata.packetctr);
	}
	// Format timestamp
	if(mode_stream_format_ts)
	{
		strptr = format1u32(strptr,mpumotiondata.time);
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
		strptr = format3s16(strptr,mpumotiondata.ax,mpumotiondata.ay,mpumotiondata.az);
	// Formats gyro if selected
	if(sample_mode & MPU_MODE_BM_G)
		strptr = format3s16(strptr,mpumotiondata.gx,mpumotiondata.gy,mpumotiondata.gz);
	// Formats magnetic if selected
	if(sample_mode & MPU_MODE_BM_M)
		strptr = format3s16(strptr,mpumotiondata.mx,mpumotiondata.my,mpumotiondata.mz);
	// Formats quaternions if selected
	
	if(sample_mode & MPU_MODE_BM_Q)
	{
		#if ENABLEQUATERNION==1
			#if FIXEDPOINTQUATERNION==1
				strptr = format4fract16(strptr,q0,q1,q2,q3);
			#else
				strptr = format4float(strptr,q0,q1,q2,q3);
			#endif
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
		packet_add16_little(&p,mpumotiondata.packetctr);
	}
	// Format timestamp
	if(mode_stream_format_ts)
	{
		packet_add16_little(&p,mpumotiondata.time&0xffff);
		packet_add16_little(&p,(mpumotiondata.time>>16)&0xffff);
	}
	// Format battery
	if(mode_stream_format_bat)
		packet_add16_little(&p,system_getbattery());
	if(mode_stream_format_label)
		packet_add16_little(&p,CurrentAnnotation);
		
	// Formats acceleration if selected
	if(sample_mode & MPU_MODE_BM_A)
	{
		packet_add16_little(&p,mpumotiondata.ax);
		packet_add16_little(&p,mpumotiondata.ay);
		packet_add16_little(&p,mpumotiondata.az);
	}
	// Formats gyro if selected
	if(sample_mode & MPU_MODE_BM_G)
	{
		packet_add16_little(&p,mpumotiondata.gx);
		packet_add16_little(&p,mpumotiondata.gy);
		packet_add16_little(&p,mpumotiondata.gz);
	}
	// Formats magnetic if selected
	if(sample_mode & MPU_MODE_BM_M)
	{
		packet_add16_little(&p,mpumotiondata.mx);
		packet_add16_little(&p,mpumotiondata.my);
		packet_add16_little(&p,mpumotiondata.mz);
	}
	// Formats quaternions if selected
	if(sample_mode & MPU_MODE_BM_Q)
	{	
		#if ENABLEQUATERNION==1
			#if FIXEDPOINTQUATERNION==1
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
				float k;
				signed short v;
				k = q0*10000.0; v = k;
				packet_add16_little(&p,v);
				k = q1*10000.0; v = k;
				packet_add16_little(&p,v);
				k = q2*10000.0; v = k;
				packet_add16_little(&p,v);
				k = q3*10000.0; v = k;
				packet_add16_little(&p,v);	
			#endif
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

/******************************************************************************
	function: stream_status
*******************************************************************************	
	Sends to a specified file information about the current streaming/logging state,
	including battery informationa and log status.
	
	In binary streaming mode an information packet with header DII is created.
	The packet definition string is: DII;is-s-siiiiic;f

	In text streaming mode an easy to parse string is sent prefixed by '#'.
	
	
	
	Paramters:
		f		-	File to which to send the status

	Returns:
		Nothing
*******************************************************************************/
void stream_status(FILE *f)
{
	unsigned long wps = stat_wakeup*1000l/(stat_t_cur-stat_time_laststatus);
	if(mode_stream_format_bin==0)
	{
		// Information text
		char str[128];
		sprintf_P(str,PSTR("#t=%lu ms; %s"),stat_t_cur-stat_timemsstart,ltc2942_last_strstatus());
		fputbuf(f,str,strlen(str));
		sprintf_P(str,PSTR("; wps=%lu; spl=%lu spl; err=%lu spl; log=%lu KB; logmax=%lu KB; logfull=%lu %%\n"),wps,stat_totsample,stat_samplesendfailed,ufat_log_getsize()>>10,ufat_log_getmaxsize()>>10,ufat_log_getsize()/(ufat_log_getmaxsize()/100l));
		fputbuf(f,str,strlen(str));
	}
	else
	{
		// Information packet
		PACKET p;
		packet_init(&p,"DII",3);
		packet_add32_little(&p,stat_t_cur-stat_timemsstart);		
		packet_add16_little(&p,ltc2942_last_mV());
		packet_add16_little(&p,ltc2942_last_mA());
		packet_add16_little(&p,ltc2942_last_mW());
		packet_add32_little(&p,wps);
		packet_add32_little(&p,stat_totsample);
		packet_add32_little(&p,stat_samplesendfailed);
		packet_add32_little(&p,ufat_log_getsize()>>10);
		packet_add32_little(&p,ufat_log_getmaxsize()>>10);
		packet_add8(&p,ufat_log_getsize()/(ufat_log_getmaxsize()/100l));
		packet_end(&p);
		packet_addchecksum_fletcher16_little(&p);
		int s = packet_size(&p);
		fputbuf(f,(char*)p.data,s);
	}
}





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
	
	// Load the persistent data
	unsigned char scale;
	scale = ConfigLoadMotionAccScale();	
	fprintf_P(file_pri,PSTR("Acc scale: %d\n"),scale);
	mpu_setaccscale(scale);
	scale = ConfigLoadMotionGyroScale();
	fprintf_P(file_pri,PSTR("Gyro scale: %d\n"),scale);
	mpu_setgyroscale(scale);
}
void stream_stop(void)
{
	mpu_config_motionmode(MPU_MODE_OFF,0);
}


unsigned char CommandParserMotion(char *buffer,unsigned char size)
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
	
	unsigned long int time_lastblink,t_end;
	unsigned long int time_lastmemlog;
	unsigned char putbufrv;
	
	
	
	system_led_set(0b01);
	
	//lcd_clear565(0);
	//lcd_writestring("Streaming",28,0,2,0x0000,0xffff);	

	set_sleep_mode(SLEEP_MODE_IDLE); 
	sleep_enable();

	mode_sample_file_log=0;


	stream_start();
	
	stat_totsample=0;
	stat_samplesendfailed=0;	
	stat_wakeup=0;	
	stat_t_cur = time_lastmemlog = time_lastblink = stat_time_laststatus = stat_timemsstart = timer_ms_get();
	batstatptr=0;
	
	// Store the data
	batstat[batstatptr].t = stat_t_cur-stat_timemsstart;
	batstat[batstatptr].mV = ltc2942_last_mV();
	batstat[batstatptr].mA = ltc2942_last_mA();
	batstat[batstatptr].mW = ltc2942_last_mW();
	batstatptr++;
	
	while(1)
	{
		sleep_cpu();
		stat_wakeup++;
		
		// Process user commands
		while(CommandProcess(CommandParsersMotionStream,CommandParsersMotionStreamNum));		
		if(CommandShouldQuit())
			break;
		// Busy loop to 
		//_delay_ms(1);
		
		
		
		stat_t_cur=timer_ms_get();
		// Blink
		if(stat_t_cur-time_lastblink>1000)
		{		
			system_led_toggle(0b100);
			time_lastblink=stat_t_cur;
		}		
		// Display info if enabled
		if(enableinfo)
		{
			if(stat_t_cur-stat_time_laststatus>10000)
			//if(stat_t_cur-stat_time_laststatus>2000)
			{
				stream_status(file_pri);
				stat_time_laststatus=stat_t_cur;
				stat_wakeup=0;
			}
		}
		
		// Memory logs for battery benchmarks
		if(stat_t_cur-time_lastmemlog>150000l)
		//if(stat_t_cur-time_lastmemlog>30000l)
		{		
			// Store the data
			printf("Storing bat log at %d\n",batstatptr);
			if(batstatptr<MAXBATSTATPTR)
			{
				batstat[batstatptr].t = stat_t_cur-stat_timemsstart;
				batstat[batstatptr].mV = ltc2942_last_mV();
				batstat[batstatptr].mA = ltc2942_last_mA();
				batstat[batstatptr].mW = ltc2942_last_mW();
				batstatptr++;
			}
			time_lastmemlog=stat_t_cur;
		}
		
		// Stream
		unsigned char l = mpu_data_level();
		for(unsigned char i=0;i<l;i++)
		{
			// Get the data from the auto read buffer; if no data available break
			if(mpu_data_getnext_raw(mpumotiondata))
				break;
			
		
			// Compute the quaternions if in a quaternion mode
			#if ENABLEQUATERNION==1
			if(sample_mode==MPU_MODE_ACCGYRMAGQ || sample_mode==MPU_MODE_Q)
			{
				#if FIXEDPOINTQUATERNION==1
				
				/*float tax,tay,taz;
				tax = mpu_data_ax[mpu_data_rdptr]*1.0/16384.0;
				tay = mpu_data_ay[mpu_data_rdptr]*1.0/16384.0;
				taz = mpu_data_az[mpu_data_rdptr]*1.0/16384.0;
				
				// change range to avoid possible overflow
				tax /= 16;
				tay /= 16;
				taz /= 16;
				*/
				FIXEDPOINTTYPE ax,ay,az,gx,gy,gz;
				
				//ax=tax;
				//ay=tay;
				//az=taz;
				
				ax = mpumotiondata.ax*atog;
				ay = mpumotiondata.ay*atog;
				az = mpumotiondata.az*atog;
				gx = mpumotiondata.gx*mpu_gtorps;
				gy = mpumotiondata.gy*mpu_gtorps;
				gz = mpumotiondata.gz*mpu_gtorps;				
				
				// Killing g does not fix bug
				// Killing a seems to fix bug
				// Killing m does not seem to fix bug.
				//gx=0;
				//gy=0;
				//gz=0;
				
				// bug still present with g=0 and m=0 and a amplitude reduced to avoid overflow. seems related to normalisation
				// bug not related to normalisation of a
				// bug seems in recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
				
				
				
				// Bug might be normalisation related
				
				
				// Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of the magnetometer;
				// the magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
				//unsigned long t1,t2;
				//t1=timer_us_get();
				MadgwickAHRSupdate_fixed(gx,gy,gz,ax,ay,az,	-mpumotiondata.my,
														-mpumotiondata.mx,
														mpumotiondata.mz);
				//t2=timer_us_get();
				//printf("%lu\n",t2-t1);
				//MadgwickAHRSupdate_fixed(gx,gy,gz,ax,ay,az,	0,
				//										0,
				//										0);
				#else
				float ax,ay,az,gx,gy,gz;
				ax = mpumotiondata.ax*atog;
				ay = mpumotiondata.ay*atog;
				az = mpumotiondata.az*atog;
				gx = mpumotiondata.gx*mpu_gtorps;
				gy = mpumotiondata.gy*mpu_gtorps;
				gz = mpumotiondata.gz*mpu_gtorps;				
				// Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of the magnetometer;
				// the magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
				//unsigned long t1,t2;
				//t1=timer_us_get();
				MadgwickAHRSupdate_float(gx,gy,gz,ax,ay,az,	-mpumotiondata.my,
														-mpumotiondata.mx,
														mpumotiondata.mz);
				//t2=timer_us_get();
				//printf("%lu\n",t2-t1);
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
					break;
				}
			}
			stat_totsample++;
		}
		
		
		
		
		// Stop if batter too low
		if(ltc2942_last_mV()<3350)
		//if(ltc2942_last_mV()<4160)
		{
			mode_sample_logend();
			fprintf_P(file_pri,PSTR("Low battery, interrupting\n"));
			break;
		}
	}
	
	stream_stop();	
	t_end=timer_ms_get();
	
	// End the logging, if logging was ongoing
	mode_sample_logend();
	
	// Store batstat in a logfile
	mode_sample_file_log = ufat_log_open(ufat_log_getnumlogs()-1);
	if(!mode_sample_file_log)
	{
		fprintf_P(file_pri,PSTR("Error opening batstatlog\n"));
	}
	else
	{
		log_printstatus();
		fprintf(mode_sample_file_log,"Battery log:\n");
		printbatstat(mode_sample_file_log);
		fprintf(mode_sample_file_log,"Battery log end\n");
		mode_sample_file_log=0;
		ufat_log_close();
	}

	

	printf_P(PSTR("Streaming stopped. Total streaming time: %lu ms\n"),t_end-stat_timemsstart);
	printf_P(PSTR("Put buffer errors: %lu\n"),stat_samplesendfailed);
	
	printf_P(PSTR("stat_wakeups: %u\n"),stat_wakeup);
	mpu_printstat(file_pri);
	

	
}





