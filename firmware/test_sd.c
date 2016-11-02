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

#include "mode_sd.h"
#include "commandset.h"
#include "mode_global.h"
#include "spi.h"
#include "sd.h"
#include "ufat.h"





void test_sd_benchmarkwriteblock(unsigned long capacity_sectors)
{
	unsigned long tott,maxt,tt1,tt2;
	unsigned char rv;
	unsigned long i;

	unsigned long numblock=2048;		// max 8192 due to speed computation
	unsigned long startb[6]={0,4096,8192,131072,2097152,0};
	
	unsigned long binsize=200;	// Bin size in microsecond
	unsigned long numbin=32;		// Number of bins
	unsigned long bins[numbin+1];		

	startb[5] = capacity_sectors-numblock;
	


	printf_P(PSTR("Benchmarking sequential block write\r"));

	// Set something in the buffer
	memset(ufatblock,0,512);

	for(int bi=0;bi<6;bi++)			// Iterate the regions of the card to test
	{
		printf_P(PSTR("Benchmarking blocks %lu->%lu\r"),startb[bi],startb[bi]+numblock);
		tott=0;
		maxt=0;
		for(unsigned i=0;i<numbin+1;i++) bins[i]=0;
		for(i=startb[bi];i<startb[bi]+numblock;i++)
		{
			ufatblock[0]=i;
						
			tt1=timer_us_get();
			rv = sd_block_write(i,ufatblock);
			tt2=timer_us_get();
			
			if(rv!=0)
			{
				printf_P(PSTR("Write block %lu failed\n"),i);
				break;
			}
			
			unsigned long dt;
			dt = tt2-tt1;
			tott+=dt;
			if(dt>maxt)
				maxt=dt;			
			dt/=binsize;
			if(dt>=numbin)
				dt=numbin;
			bins[dt]++;
		}	
		// numblock*512/tott [bytes/us]=[MioB/s]
		// numblock*512/1024[KB] / tott/1000000 [s] [KB/s] -> numblock/2 [KB] / tott/1000000 [s] -> numblock/2 * 1000000/tott -> numblock*500000/tott [KB/s]
		unsigned long spd = numblock*500000/tott;
		printf_P(PSTR("Total time: %lu us Maximum time per block: %lu us\n"),tott,maxt);
		printf_P(PSTR("Average write speed: %lu KB/s\n"),spd);
		printf_P(PSTR("Histogram of times:\r"));
		for(i=0;i<numbin;i++)
		{
			printf_P(PSTR("[%lu-%lu] us: %lu\n"),i*binsize,i*binsize+binsize-1,bins[i]);
		}
		printf_P(PSTR("[%lu-inf) %lu\r"),i*binsize,bins[i]);
	}
}

void test_sd_benchmarkwriteblockmulti(unsigned long capacity_sectors)
{
	unsigned long tott,maxt,tt1,tt2;
	unsigned char rv;
	unsigned long i;

	unsigned long numblock=2048;		// max 8192 due to speed computation
	unsigned long startb[6]={0,4096,8192,131072,2097152,0};
	
	unsigned long binsize=200;	// Bin size in microsecond
	unsigned long numbin=32;		// Number of bins
	unsigned long bins[numbin+1];		

	startb[5] = capacity_sectors-numblock;
	


	printf_P(PSTR("Benchmarking sequential multiblock write\r"));

	// Set something in the buffer
	memset(ufatblock,0,512);

	for(int bi=0;bi<6;bi++)			// Iterate the regions of the card to test
	{
		printf_P(PSTR("Benchmarking blocks %lu->%lu\r"),startb[bi],startb[bi]+numblock);
		tott=0;
		maxt=0;
		for(unsigned i=0;i<numbin+1;i++) bins[i]=0;
		sd_stream_open(startb[bi]);
		for(i=0;i<numblock;i++)
		{
			ufatblock[0]=i;
						
			tt1=timer_us_get();
			//rv = sd_write_stream_write_block(ufatblock,0);
			//rv = sd_write_stream_write(ufatblock,512,0);
			rv = sd_streamcache_write(ufatblock,512,0);
			//rv = sd_write_stream_write_block2(ufatblock,0);
			tt2=timer_us_get();
			
			if(rv!=0)
			{
				printf_P(PSTR("Multiblock write block %lu failed\n"),i);
				break;
			}
			
			unsigned long dt;
			dt = tt2-tt1;
			tott+=dt;
			if(dt>maxt)
				maxt=dt;			
			dt/=binsize;
			if(dt>=numbin)
				dt=numbin;
			bins[dt]++;
		}	
		//rv = sd_write_streammulti_close(0);
		rv = sd_streamcache_close(0);
		if(rv!=0)
		{
			printf_P(PSTR("Multiblock close failed\n"));
			break;
		}
		// numblock*512/tott [bytes/us]=[MioB/s]
		// numblock*512/1024[KB] / tott/1000000 [s] [KB/s] -> numblock/2 [KB] / tott/1000000 [s] -> numblock/2 * 1000000/tott -> numblock*500000/tott [KB/s]
		unsigned long spd = numblock*500000/tott;
		printf_P(PSTR("Total time: %lu us Maximum time per block: %lu us\n"),tott,maxt);
		printf_P(PSTR("Average write speed: %lu KB/s\n"),spd);
		printf_P(PSTR("Histogram of times:\r"));
		for(i=0;i<numbin;i++)
		{
			printf_P(PSTR("[%lu-%lu] us: %lu\n"),i*binsize,i*binsize+binsize-1,bins[i]);
		}
		printf_P(PSTR("[%lu-inf) %lu\r"),i*binsize,bins[i]);
	}
}
void test_sdmulti(void)
{
	unsigned char rv;
	unsigned long currentaddr;
	sd_stream_open(0);
	for(unsigned j=0;j<512;j++)
			ufatblock[j]=j;
	//for(unsigned i=0;i<16;i++)
	for(unsigned i=0;i<8;i++)
	{
		ufatblock[0]=i;
		//memset(ufatblock,i,512);		
		
		//rv = sd_write_stream_write_block(ufatblock,&currentaddr);
		//rv = sd_write_stream_write_block2(ufatblock,&currentaddr);
		rv = sd_streamcache_write(ufatblock,512,&currentaddr);
		printf("w %02X at %lu\n",rv,currentaddr);
	}
	
	//rv = sd_write_stream_write2(ufatblock,512,&currentaddr);
	//printf("w %d at %lu\n",rv,currentaddr);
	
	//rv = sd_write_streammulti_close(&currentaddr);
	memset(ufatblock,'x',200);
	rv = sd_streamcache_write(ufatblock,200,&currentaddr);
	printf("w %02X at %lu\n",rv,currentaddr);
	memset(ufatblock,'y',200);
	rv = sd_streamcache_write(ufatblock,200,&currentaddr);
	printf("w %02X at %lu\n",rv,currentaddr);
	memset(ufatblock,'z',200);
	rv = sd_streamcache_write(ufatblock,200,&currentaddr);
	printf("w %02X at %lu\n",rv,currentaddr);
	
	rv = sd_streamcache_close(&currentaddr);
	printf("close %d at %lu\n",rv,currentaddr);
}

