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
		sd_stream_open(startb[bi],0);
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
	sd_stream_open(0,0);
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

/*
	Similar to ufat_log_test2 but using streamcache write to absolute sectors
	
*/
/*void sd_bench_streamcache_write(unsigned long startsect,unsigned long size,unsigned long preerase)
{
	char buf[512];
	unsigned long t1;
	unsigned long cursize;
	unsigned long pkt;
	unsigned long numfail;
	unsigned long lastsize;
	unsigned char rv;
	unsigned short towrite=512;
	
	
	memset(buf,'0',towrite);
	buf[towrite-1]=0;
	buf[towrite-2]='\n';
	
	
	
	//_sd_acmd23(9765);
	//_sd_acmd23(20000);
	//sd_select_n(0);
	//_sd_acmd23b(20000);
	//sd_select_n(1);
	
	printf_P(PSTR("Benchmarking streamcache block write from %lu writing 512 bytes up to %lu with %lu pre-erased sectors\n"),startsect,size,preerase);
	sd_stream_open(startsect,preerase);
	
	
	cursize=0;
	pkt=0;
	numfail=0;
	lastsize=0;
	unsigned long tlast=timer_ms_get();
	
	while(cursize<size)
	{
		// Do something
		//_delay_ms(1);
		//_delay_us(500);
		
		if(cursize>=lastsize+512)
		{
			lastsize=cursize;
			unsigned long t=timer_ms_get();
			fprintf_P(file_pri,PSTR("%lu %lu "),cursize,t-tlast);
			tlast=t;
			
		}
		
		
		t1=timer_ms_get();
		char *strptr = buf;
		strptr=format1u32(strptr,t1);		
		strptr=format1u32(strptr,pkt);
		strptr=format1u32(strptr,numfail);
		strptr=format1u32(strptr,cursize);
		strptr=format1u32(strptr,cursize);
				
		rv = sd_streamcache_write(buf,towrite,0);
		if(rv)
		{
			numfail++;
			fprintf_P(file_pri,PSTR("f\n"));
		}	
		else
		{
			cursize+=towrite;
			fprintf_P(file_pri,PSTR("\n"));
		}
		
		pkt++;		
		
	}
	
	rv = sd_streamcache_close(0);
	if(rv!=0)
	{
		printf_P(PSTR("Failed sd_write_stream_close\n"));
	}
	
}
*/
/*void sd_bench_write(unsigned long startsect,unsigned long size)
{
	char buf[512];
	unsigned long t1;
	unsigned long cursize;
	unsigned long pkt;
	unsigned long numfail;
	unsigned long lastsize;
	unsigned char rv;
	unsigned short towrite=512;
	
	
	memset(buf,'0',towrite);
	buf[towrite-1]=0;
	buf[towrite-2]='\n';
	
	
	
	//_sd_acmd23(9765);
	
	printf_P(PSTR("Benchmarking block write from %lu writing 512 bytes up to %lu\n"),startsect,size);

	
	cursize=0;
	pkt=0;
	numfail=0;
	lastsize=0;
	unsigned long tlast=timer_ms_get();
	
	while(cursize<size)
	{
		if(cursize>=lastsize+512)
		{
			lastsize=cursize;
			unsigned long t=timer_ms_get();
			fprintf_P(file_pri,PSTR("%lu %lu\n"),cursize,t-tlast);
			tlast=t;
			
		}
		
		
		t1=timer_ms_get();
		char *strptr = buf;		
		strptr=format1u32(strptr,t1);		
		strptr=format1u32(strptr,pkt);
		strptr=format1u32(strptr,numfail);
		strptr=format1u32(strptr,cursize);
		strptr=format1u32(strptr,cursize);
				
		rv=sd_block_write(startsect,buf);
		if(rv)
		{
			numfail++;
		}	
		else
		{
			cursize+=towrite;
		}
		
		pkt++;		
		startsect++;
		
	}	
}*/

void printhist(unsigned long *hist1,unsigned long *hist10,unsigned long *hist100)
{
	for(unsigned short i=0;i<10;i++)
		printf("%05d ",i);
	printf("| ");
	for(unsigned short i=1;i<10;i++)
		printf("%05d ",i*10);
	printf("| ");
	for(unsigned short i=1;i<11;i++)
		printf("%05d ",i*100);
	printf("\n");
	for(unsigned short i=0;i<10;i++)
		printf("%05lu ",hist1[i]);
	printf("| ");
	for(unsigned short i=1;i<10;i++)
		printf("%05lu ",hist10[i]);
	printf("| ");
	for(unsigned short i=1;i<11;i++)
		printf("%05lu ",hist100[i]);
	printf("\n");
}

void sd_bench_write2(unsigned long startsect,unsigned long size)
{
	char buf[512];
	unsigned long t1,t2;
	unsigned long cursize;
	unsigned long pkt;
	unsigned long numfail;
	unsigned long lastsize;
	unsigned char rv;
	unsigned short towrite=512;
	
	unsigned long hist1[11];	// 1ms bins from 0ms to 10-infms
	unsigned long hist10[11];	// 10ms bins from 0ms to 100-infms
	unsigned long hist100[11];	// 100ms bins from 0ms to 1-infs
	
	unsigned long slist[20];	// Worst times
	
	memset(slist,0,20*sizeof(unsigned long));
	
	
	hist_init(hist1,11);
	hist_init(hist10,11);
	hist_init(hist100,11);

	memset(buf,'0',towrite);
	buf[towrite-1]=0;
	buf[towrite-2]='\n';
	
	printf_P(PSTR("Benchmarking block write from %lu writing 512 bytes up to %lu\n"),startsect,size);

	
	cursize=0;
	pkt=0;
	numfail=0;
	lastsize=0;
	
	
	while(cursize<size)
	{
		if(cursize>=lastsize+1024*256l)
		{
			lastsize=cursize;
			fprintf_P(file_pri,PSTR("%luB\n"),cursize);
			printhist(hist1,hist10,hist100);
		}
		
		
		t1=timer_ms_get();
		char *strptr = buf;		
		strptr=format1u32(strptr,t1);		
		strptr=format1u32(strptr,pkt);
		strptr=format1u32(strptr,numfail);
		strptr=format1u32(strptr,cursize);
		strptr=format1u32(strptr,cursize);
				
		t1 = timer_ms_get();
		rv=sd_block_write(startsect,buf);
		t2 = timer_ms_get();
		
		if(rv)
		{
			numfail++;
			printf("F %lu\n",numfail);
			break;
		}	
		else
		{
			cursize+=towrite;
			
			//printf("%ld\n",t2-t1);
			
			// Add to histogram
			hist_insert(hist1,11,1,t2-t1);
			hist_insert(hist10,11,10,t2-t1);
			hist_insert(hist100,11,100,t2-t1);
			
			slist_add(slist,20,t2-t1);
			
			// Print histogram
			//printhist(hist1,hist10,hist100);
		}
		
		pkt++;		
		startsect++;
	}
	printf("Done. Num fail: %lu\n",numfail);
	printhist(hist1,hist10,hist100);
	printf("Worst times:\n");
	for(int i=0;i<20;i++)
		printf("%lu ",slist[i]);
	printf("\n");
	
}
void sd_bench_stream_write2(unsigned long startsect,unsigned long size,unsigned long preerase)
{
	char buf[512];
	unsigned long t1,t2;
	unsigned long cursize;
	unsigned long pkt;
	unsigned long numfail;
	unsigned long lastsize;
	unsigned char rv;
	unsigned short towrite=512;
	
	unsigned long hist1[11];	// 1ms bins from 0ms to 10-infms
	unsigned long hist10[11];	// 10ms bins from 0ms to 100-infms
	unsigned long hist100[11];	// 100ms bins from 0ms to 1-infs
	
	unsigned long slist[20];	// Worst times
	
	memset(slist,0,20*sizeof(unsigned long));
	
	
	hist_init(hist1,11);
	hist_init(hist10,11);
	hist_init(hist100,11);

	memset(buf,'0',towrite);
	buf[towrite-1]=0;
	buf[towrite-2]='\n';
	
	printf_P(PSTR("Benchmarking stream block write from %lu writing 512 bytes up to %lu with %lu pre-erased sectors\n"),startsect,size,preerase);
	sd_stream_open(startsect,preerase);
	
	cursize=0;
	pkt=0;
	numfail=0;
	lastsize=0;
	
	
	while(cursize<size)
	{
		if(cursize>=lastsize+1024*256l)
		{
			lastsize=cursize;
			fprintf_P(file_pri,PSTR("%luB\n"),cursize);
			printhist(hist1,hist10,hist100);
		}
		
		
		t1=timer_ms_get();
		char *strptr = buf;		
		strptr=format1u32(strptr,t1);		
		strptr=format1u32(strptr,pkt);
		strptr=format1u32(strptr,numfail);
		strptr=format1u32(strptr,cursize);
		strptr=format1u32(strptr,cursize);
				
		t1 = timer_ms_get();
		rv = sd_stream_write(buf,512,0);
		t2 = timer_ms_get();
		
		if(rv)
		{
			numfail++;
			printf("F %lu\n",numfail);
			break;
		}	
		else
		{
			cursize+=towrite;
			
			//printf("%ld\n",t2-t1);
			
			// Add to histogram
			hist_insert(hist1,11,1,t2-t1);
			hist_insert(hist10,11,10,t2-t1);
			hist_insert(hist100,11,100,t2-t1);
			
			slist_add(slist,20,t2-t1);
			
			// Print histogram
			//printhist(hist1,hist10,hist100);
		}
		
		pkt++;		
		startsect++;
	}
	printf("Done. Num fail: %lu\n",numfail);
	printhist(hist1,hist10,hist100);
	printf("Worst times:\n");
	for(int i=0;i<20;i++)
		printf("%lu ",slist[i]);
	printf("\n");
	rv = sd_stream_close(0);
	if(rv!=0)
	{
		printf_P(PSTR("Failed sd_stream_close\n"));
	}
	
}
void sd_bench_streamcache_write2(unsigned long startsect,unsigned long size,unsigned long preerase)
{
	char buf[512];
	unsigned long t1,t2;
	unsigned long cursize;
	unsigned long pkt;
	unsigned long numfail;
	unsigned long lastsize;
	unsigned char rv;
	unsigned short towrite=512;
	
	unsigned long hist1[11];	// 1ms bins from 0ms to 10-infms
	unsigned long hist10[11];	// 10ms bins from 0ms to 100-infms
	unsigned long hist100[11];	// 100ms bins from 0ms to 1-infs
	
	unsigned long slist[20];	// Worst times
	
	memset(slist,0,20*sizeof(unsigned long));
	
	
	hist_init(hist1,11);
	hist_init(hist10,11);
	hist_init(hist100,11);

	memset(buf,'0',towrite);
	buf[towrite-1]=0;
	buf[towrite-2]='\n';
	
	printf_P(PSTR("Benchmarking stream block write from %lu writing 512 bytes up to %lu with %lu pre-erased sectors\n"),startsect,size,preerase);
	sd_stream_open(startsect,preerase);
	
	cursize=0;
	pkt=0;
	numfail=0;
	lastsize=0;
	
	
	while(cursize<size)
	{
		if(cursize>=lastsize+1024*256l)
		{
			lastsize=cursize;
			fprintf_P(file_pri,PSTR("%luB\n"),cursize);
			printhist(hist1,hist10,hist100);
		}
		
		
		t1=timer_ms_get();
		char *strptr = buf;		
		strptr=format1u32(strptr,t1);		
		strptr=format1u32(strptr,pkt);
		strptr=format1u32(strptr,numfail);
		strptr=format1u32(strptr,cursize);
		strptr=format1u32(strptr,cursize);
				
		t1 = timer_ms_get();
		rv = sd_streamcache_write(buf,towrite,0);
		t2 = timer_ms_get();
		
		if(rv)
		{
			numfail++;
			printf("F %lu\n",numfail);
			break;
		}	
		else
		{
			cursize+=towrite;
			
			//printf("%ld\n",t2-t1);
			
			// Add to histogram
			hist_insert(hist1,11,1,t2-t1);
			hist_insert(hist10,11,10,t2-t1);
			hist_insert(hist100,11,100,t2-t1);
			
			slist_add(slist,20,t2-t1);
			
			// Print histogram
			//printhist(hist1,hist10,hist100);
		}
		
		pkt++;		
		startsect++;
	}
	printf("Done. Num fail: %lu. Wrote: %lu\n",numfail,cursize);
	printhist(hist1,hist10,hist100);
	printf("Worst times:\n");
	for(int i=0;i<20;i++)
		printf("%lu ",slist[i]);
	printf("\n");
	rv = sd_streamcache_close(0);
	if(rv!=0)
	{
		printf_P(PSTR("Failed sd_stream_close\n"));
	}
	
}
/*void sd_bench_stream_write(unsigned long startsect,unsigned long size,unsigned long preerase)
{
	// preerase in sector count.
	char buf[512];
	unsigned long t1;
	unsigned long cursize;
	unsigned long pkt;
	unsigned long numfail;
	unsigned long lastsize;
	unsigned char rv;
	unsigned short towrite=512;
	
	
	memset(buf,'0',towrite);
	buf[towrite-1]=0;
	buf[towrite-2]='\n';
	
	
	
	//_sd_acmd23(9765);
	
	printf_P(PSTR("Benchmarking stream write from %lu writing 512 bytes up to %lu\n"),startsect,size);
	sd_stream_open(startsect,preerase);
	
	
	cursize=0;
	pkt=0;
	numfail=0;
	lastsize=0;
	unsigned long tlast=timer_ms_get();
	
	while(cursize<size)
	{
		// Do something
		//_delay_ms(1);
		//_delay_us(500);
		
		if(cursize>=lastsize+512)
		{
			lastsize=cursize;
			unsigned long t=timer_ms_get();
			fprintf_P(file_pri,PSTR("%lu %lu\n"),cursize,t-tlast);
			tlast=t;
			
		}
		
		
		t1=timer_ms_get();
		char *strptr = buf;		
		strptr=format1u32(strptr,t1);		
		strptr=format1u32(strptr,pkt);
		strptr=format1u32(strptr,numfail);
		strptr=format1u32(strptr,cursize);
		strptr=format1u32(strptr,cursize);
				
		rv = sd_stream_write(buf,towrite,0);
		if(rv)
		{
			numfail++;
		}	
		else
		{
			cursize+=towrite;
		}
		
		pkt++;		
		
	}
	
	rv = sd_stream_close(0);
	if(rv!=0)
	{
		printf_P(PSTR("Failed sd_stream_close\n"));
	}
	
}*/
