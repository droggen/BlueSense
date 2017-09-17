/*
	file: mode_sample
	
	Contains common functionalities between mode_sample_adc and mode_sample_motion.
	
	Offers:
	
	* mode_sample_logend: 			to terminate logs
	* help_samplelog				help string
	* mode_sample_file_log			FILE* for logging
	
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
#include "mode_sample.h"
#include "mode_global.h"
#include "commandset.h"


// Log file used by the modes mode_adc and mode_motionstream
FILE *mode_sample_file_log;				

const char help_samplelog[] PROGMEM="L[,<lognum>]: without parameter logging is stopped, otherwise logging starts on lognum";


/******************************************************************************
	function: CommandParserSampleLog
*******************************************************************************	
	Parses a user command to enable logging to a specified logfile. 
	
	This is used by mode_sample_adc and mode_sample_motion. It relies on the global variable
	mode_file_log.
	
	
	Command format: L[,<lognum>]
	
	Parameters:
		buffer			-			Buffer containing the command
		size			-			Length of the buffer containing the command	

	Returns:
		0		-		Success
		1		-		Message execution error (message valid)
		2		-		Message invalid 
******************************************************************************/
unsigned char CommandParserSampleLog(char *buffer,unsigned char size)
{
	unsigned int lognum;
	
	if(size==0)
	{
		// L-only was passed: stop logging if logging ongoing
		if(mode_sample_file_log==0)
		{
			fprintf_P(file_pri,PSTR("No ongoing logging\n"));
			return 1;
		}
		mode_sample_logend();
		return 0;		
	}
	
	// L,lognum was passed: parse
	unsigned char rv;
	rv = ParseCommaGetInt((char*)buffer,1,&lognum);
	if(rv)
		return 2;		// Message invalid
	
	rv = mode_sample_startlog(lognum);
	return rv;			// Returns 0 (ok) or 1 (execution error but message valid)
}

/******************************************************************************
	function: mode_sample_startlog
*******************************************************************************	
	Starts a log on the specified logfile. 
	If the filesystem is unavailable or a log is ongoing returns an error.
	
	If lognum is negative, does nothing.
	
	Parameters:
		lognum		-			Log to open. If negative, do nothing and return success.

	Returns:
		0			-			Success
		1			-			Error

******************************************************************************/
unsigned char mode_sample_startlog(int lognum)
{
	// Log negative means no log to start
	if(lognum<0)
	{
		return 0;
	}
		
	// Non-negative: start logging
	
	// Check if the filesystem has been initialised successfully
	if(!ufat_available())
	{
		fprintf_P(file_pri,PSTR("No filesystem available\n"));
		// Return an error
		return 1;
	}
	// Check if logging ongoing
	if(mode_sample_file_log!=0)
	{
		fprintf_P(file_pri,PSTR("Already logging; stop before restarting\n"));
		// Return an error
		return 1;
	}
	
	//printf("not logging therefore start logging\n");
	
	// Not logging therefore start logging
	fprintf_P(file_pri,PSTR("Logging on %u\n"),lognum);

	mode_sample_file_log = ufat_log_open(lognum);
	if(!mode_sample_file_log)
	{
		fprintf_P(file_pri,PSTR("Error opening log\n"));
		return 1;
	}
	return 0;
}


/******************************************************************************
	function: mode_sample_logend
*******************************************************************************	
	Checks whether logging was in progress, and if yes closes the log

******************************************************************************/
void mode_sample_logend(void)
{
	if(mode_sample_file_log)
	{
		fprintf_P(file_pri,PSTR("Terminating logging\n"));
		mode_sample_file_log=0;
		ufat_log_close();
	}
}
