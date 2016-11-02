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
#include "test_mmc.h"
#include "pkt.h"
#include "wait.h"
#include "init.h"
#include "ui.h"
#include "lcd.h"
#include "fb.h"
#include "uiconfig.h"
#include "helper.h"
#include "i2c_internal.h"
#include "system.h"
#include "pkt.h"

#include "demo.h"
#include "mode_bench.h"

#include "commandset.h"




#define CommandParsersBenchNum 3
const COMMANDPARSER CommandParsersBench[CommandParsersBenchNum] =
{ 
	{'H', CommandParserHelp,help_h},
	{'B', CommandParserBench,help_b},
	{'!', CommandParserQuit,help_quit}
};

unsigned char mode_bench_io=0;


unsigned char CommandParserBench(unsigned char *buffer,unsigned char size)
{	
	if(size!=2 || buffer[0]!=',')
		return 2;
	
	buffer++;	// Skip comma
	
	unsigned io;
	if(sscanf((char*)buffer,"%u",&io)!=1)
		return 2;
	if(!(io==0 || io==1))
		return 2;
	
	system_mode = APP_MODE_BENCHIO;
	CommandQuit=1;
	
	mode_bench_io=io;
	
	
	return 0;
}


void mode_bench(void)
{
	fprintf_P(file_pri,PSTR("bench mode\n"));
	char s[256];
	int c;
	
	for(unsigned i=0;i<255;i++)
		s[i] = '0'+i%10;
	s[255] = '\n';
	
	
	
	while(1)
	{
		while(CommandProcess(CommandParsersBench,CommandParsersBenchNum));		
		if(CommandShouldQuit())
			break;
			
		FILE *f;
		if(mode_bench_io==0)
			f=file_usb;
		else
			f=file_bt;
			
		// Transfer 128KB
		unsigned long size = 128*1024l;
		unsigned it = size/256;
		unsigned long t1,t2;
		printf("size: %ld\n",size);
		printf("it: %d\n",it);
		t1 = timer_ms_get();
		for(unsigned i=0;i<it;i++)
		{
			fwrite(s,256,1,f);
			if((i&0xf)==0) 
				fputc('.',file_usb);
				
			// Get feedback if any
			while((c=fgetc(file_bt))!=-1)
				fputc(c,file_usb);
		}	
		t2 = timer_ms_get();
		unsigned long bps = size*1000/(t2-t1);
		fprintf_P(file_pri,PSTR("Transfer of %lu bytes in %lu ms. Bandwidth: %lu byte/s\n"),size,t2-t1,bps);	
		fprintf_P(file_dbg,PSTR("Transfer of %lu bytes in %lu ms. Bandwidth: %lu byte/s\n"),size,t2-t1,bps);	
		
		_delay_ms(5000);
				

	}
	fprintf_P(file_pri,PSTR("bench end\n"));

	return 0;
}

