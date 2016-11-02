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
#include "mode.h"
#include "mode_sd.h"
#include "commandset.h"
#include "mode_global.h"
#include "spi.h"
#include "sd.h"
#include "ufat.h"
#include "test_sd.h"

const char help_sdinit[] PROGMEM="Low-level SD card initialisation";
const char help_write[] PROGMEM="W,<sector>,<value>: Fills a sector with the given value (all data in decimal)";
const char help_stream[] PROGMEM="S,<sector>,<value>,<size>,<bsize>: Writes size bytes data in streaming mode with caching; sends data by blocks of size bsize";
const char help_read[] PROGMEM="R,<sector>: Reads a sector (sector number in decimal)";
const char help_volume[] PROGMEM="Initialise volume";
const char help_format[] PROGMEM="F,<numlogfiles>: Format the card for numlogfiles and initialise the volume";
const char help_logtest[] PROGMEM="L,<lognum>,<sizebytes>,<char>,<bsiz>: Writes to lognum sizebytes character char in bsiz blocks";
const char help_sdbench[] PROGMEM="B,<benchtype>";

#define CommandParsersSDNum 10
const COMMANDPARSER CommandParsersSD[CommandParsersSDNum] =
{ 
	{'I', CommandParserSDInit,help_sdinit},
	{'W', CommandParserSDWrite,help_write},
	{'S', CommandParserSDStream,help_stream},
	{'R', CommandParserSDRead,help_read},
	{'V', CommandParserSDVolume,help_volume},
	{'F', CommandParserSDFormat,help_format},
	{'L', CommandParserSDLogTest,help_logtest},
	{'B', CommandParserSDBench,help_sdbench},
	{'H', CommandParserHelp,help_h},	
	{'!', CommandParserQuit,help_quit}	
};

unsigned char CommandParserSDBench(unsigned char *buffer,unsigned char size)
{
	unsigned char rv;
	char *p1;
	rv = ParseComma((char*)buffer,1,&p1);
	if(rv)
		return 2;
		
	// Get sector
	unsigned int benchtype;
	if(sscanf(p1,"%u",&benchtype)!=1)
	{
		printf("failed benchtype\n");
		return 2;
	}	
	switch(benchtype)
	{
		case 0:
		default:
			printf("card capacity sector: %lu\n",_fsinfo.card_capacity_sector);
			test_sd_benchmarkwriteblock(_fsinfo.card_capacity_sector);
			break;
		case 1:
			test_sd_benchmarkwriteblockmulti(_fsinfo.card_capacity_sector);
			break;
		case 3:
			test_sd_benchmarkwriteblockmulti(_fsinfo.card_capacity_sector);
			break;
		case 2:
			test_sdmulti();
			break;
	}
	return 0;
}
unsigned char CommandParserSDFormat(unsigned char *buffer,unsigned char size)
{
	unsigned int numlog;
	if(ParseCommaGetInt((char*)buffer,1,&numlog))
		return 2;
		
	printf("Formatting with %u log files\n",numlog);
	ufat_format(numlog);
	return 0;
}
unsigned char CommandParserSDLogTest(unsigned char *buffer,unsigned char size)
{
	unsigned char rv;
	unsigned int lognum,ch,sz,bsiz;
	rv = ParseCommaGetInt((char*)buffer,4,&lognum,&sz,&ch,&bsiz);
	if(rv)
		return 2;
		
	//printf_P(PSTR("Logging to %d for %u bytes with char %02x in block size of %u\n"),lognum,sz,ch,bsiz);
	//unsigned long t1,t2;
	//t1 = timer_ms_get();
	ufat_log_test(lognum,sz,ch,bsiz);
	//t2 = timer_ms_get();
	//printf_P(PSTR("Time: %lu ms. %lu bytes/s\n"),t2-t1,(sz*1000)/(t2-t1));
	return 0;
}
unsigned char CommandParserSDWrite(unsigned char *buffer,unsigned char size)
{
	unsigned char rv;
	char *p1,*p2;
	rv = ParseComma((char*)buffer,2,&p1,&p2);
	if(rv)
		return 2;
		
	// Get sector
	unsigned long sector;
	if(sscanf(p1,"%lu",&sector)!=1)
	{
		printf("failed sector\n");
		return 2;
	}	
	// Get data
	unsigned int data;
	if(sscanf(p2,"%u",&data)!=1)
	{
		printf("failed data\n");
		return 2;
	}
	
	printf("Writing %02hX at sector %lu\n",data,sector);
	unsigned char block[512];
	for(unsigned short i=0;i<512;i++)
		block[i]=data;
	rv = sd_block_write(sector,block);
	printf("Write result: %d\n",rv);
	
	return 0;
	
}
unsigned char CommandParserSDRead(unsigned char *buffer,unsigned char size)
{
	unsigned char rv;
	char *p1;
	rv = ParseComma((char*)buffer,1,&p1);
	if(rv)
		return 2;
		
	// Get sector
	unsigned long sector;
	if(sscanf(p1,"%lu",&sector)!=1)
	{
		printf("failed sector\n");
		return 2;
	}	
		
	printf("Read sector %lu\n",sector);
	unsigned char block[512];
	rv = sd_block_read(sector,block);
	printf("Read result: %d\n",rv);
	for(unsigned i=0;i<32;i++)
	{
		for(unsigned j=0;j<16;j++)
		{
			printf("%02X ",block[i*16+j]);
		}
		for(unsigned j=0;j<16;j++)
		{
			unsigned char c = block[i*16+j];
			if(c>32 && c<127)
				printf("%c",c);
			else
				printf(".");
		}
		printf("\n");
	}
	
	return 0;
	
}
unsigned char CommandParserSDStream(unsigned char *buffer,unsigned char size)
{
	unsigned char rv;
	char *p1,*p2,*p3,*p4;
	rv = ParseComma((char*)buffer,4,&p1,&p2,&p3,&p4);
	if(rv)
		return 2;
		
	// Get sector
	unsigned long sector;
	if(sscanf(p1,"%lu",&sector)!=1)
	{
		printf("sect err\n");
		return 2;
	}	
	// Get data
	unsigned int data;
	if(sscanf(p2,"%u",&data)!=1)
	{
		return 2;
	}
	// Get size
	unsigned int len;
	if(sscanf(p3,"%u",&len)!=1)
	{
		return 2;
	}
	// Get size
	unsigned int bsize;
	if(sscanf(p4,"%u",&bsize)!=1)
	{
		return 2;
	}
	if(bsize>512)
		return 2;
	
	printf("Streaming %u bytes %02hX at sector %lu in blocks of %u bytes\n",len,data,sector,bsize);
	char block[512];
	unsigned long curraddr;
	for(unsigned short i=0;i<512;i++)
		block[i]=data;
	sd_stream_open(sector);
	while(len)
	{
		int effw=bsize;
		if(len<effw)
			effw=len;
		
		rv = sd_streamcache_write((unsigned char*)block,effw,&curraddr);
		printf("Write %d. Return: %d. Current address: %lu\n",effw,rv,curraddr);
		len-=effw;
	}
	rv=sd_streamcache_close(&curraddr);
	printf("Close. Return: %d. Current address: %lu\n",rv,curraddr);
	
	return 0;
	
}
unsigned char CommandParserSDVolume(unsigned char *buffer,unsigned char size)
{	
	ufat_init();
	
	return 0;
	
}
unsigned char CommandParserSDInit(unsigned char *buffer,unsigned char size)
{
	unsigned char rv;
	CID cid;
	CSD csd;
	unsigned long capacity_sector;	
	
	printf_P(PSTR("Init SD\r"));
		
	rv = sd_init(&cid,&csd,&capacity_sector);
	if(rv!=0)
	{
		printf_P(PSTR("Init SD failure (%d)\r"),rv);
		return 1;
	}
	_fsinfo.card_capacity_sector=capacity_sector;
	printf_P(PSTR("Init SD success (%d)\r"),rv);
	sd_print_cid(file_pri,&cid);
	sd_print_csd(file_pri,&csd);	
	printf_P(PSTR("Card capacity: %ld sectors\n"),capacity_sector);
	
	// Check that we deal with an SDHC card with block length of 512 bytes
	if(csd.CSD!=1 || csd.READ_BL_LEN!=9 || csd.WRITE_BL_LEN!=9)
	{
		printf("SD card unsuitable\n");
		return 1;
	}
	
	
	return 0;
}

unsigned char CommandParserSD(unsigned char *buffer,unsigned char size)
{
	CommandChangeMode(APP_MODE_SD);
	return 0;
}


void mode_sd(void)
{
	//unsigned char crc;
	
	/*crc=0;
	crc = crc7(crc,0|0x40);
	//crc = crc7(crc,0);
	printf("crc: %02X crcend: %02X\n",crc,crc7end(crc));
	crc = crc7(crc,0);
	printf("crc: %02X crcend: %02X\n",crc,crc7end(crc));
	crc = crc7(crc,0);
	printf("crc: %02X crcend: %02X\n",crc,crc7end(crc));
	crc = crc7(crc,0);
	printf("crc: %02X crcend: %02X\n",crc,crc7end(crc));
	crc = crc7(crc,0);
	printf("crc: %02X crcend: %02X\n",crc,crc7end(crc));
	crc = crc7(crc,0);
	printf("crc: %02X crcend: %02X\n",crc,crc7end(crc));
	
	crc=0;
	crc = crc7(crc,0x11|0x40);
	printf("crc: %02X crcend: %02X\n",crc,crc7end(crc));
	crc = crc7(crc,0);
	printf("crc: %02X crcend: %02X\n",crc,crc7end(crc));
	crc = crc7(crc,0);
	printf("crc: %02X crcend: %02X\n",crc,crc7end(crc));
	crc = crc7(crc,0);
	printf("crc: %02X crcend: %02X\n",crc,crc7end(crc));
	crc = crc7(crc,0);
	printf("crc: %02X crcend: %02X\n",crc,crc7end(crc));
	
	*/
	
	
	
	while(1)
	{
	//	fprintf_P(file_pri,PSTR("Some ADC stuff: %d. period: %d. mask: %02X. pri: %p dbg: %p\n"),ctr,mode_adc_period,mode_adc_mask,file_pri,file_dbg);

		while(CommandProcess(CommandParsersSD,CommandParsersSDNum));
		if(CommandShouldQuit())
			break;
			
	}
}


