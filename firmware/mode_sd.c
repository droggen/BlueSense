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
const char help_erase[] PROGMEM="E,<sectorstart>,<sectorend>: erase all sectors or from [start;end]. If start and end are zero the entire flash is erased.";
const char help_write[] PROGMEM="W,<sector>,<value>: Fills a sector with the given value (all data in decimal)";
const char help_stream[] PROGMEM="S,<sector>,<value>,<size>,<bsize>: Writes size bytes data in streaming mode with caching; sends data by blocks of size bsize";
const char help_read[] PROGMEM="R,<sector>: Reads a sector (sector number in decimal)";
const char help_volume[] PROGMEM="Initialise volume";
const char help_format[] PROGMEM="F,<numlogfiles>: Format the card for numlogfiles and initialise the volume (maximum numlogfiles=14)";
const char help_logtest[] PROGMEM="l,<lognum>,<sizekb>: QA test. Logs test data to <lognum> up to <sizekb> KB. Use to validate speed/consistency of SD card writes.";
//const char help_logtest2[] PROGMEM="L,<lognum>,<sizebytes>,<char>,<bsiz>: Writes to lognum sizebytes character char in bsiz blocks";
const char help_sdbench[] PROGMEM="B,<benchtype>";
const char help_sdbench2[] PROGMEM="b,<startsect>,<sizekb> stream cache write from startsect up to sizekb";
const char help_sdbench3[] PROGMEM="1,<startsect>,<sizekb>,<preerasekb> stream cache write from startsect up to sizekb, optional preerase kb";

#define CommandParsersSDNum 15
const COMMANDPARSER CommandParsersSD[CommandParsersSDNum] =
{ 
	{'I', CommandParserSDInit,help_sdinit},
	{'E', CommandParserSDErase,help_erase},
	//{'e', CommandParserSDErase2,help_erase},
	//{'X', CommandParserSDErase3,help_erase},
	{'W', CommandParserSDWrite,help_write},
	{'S', CommandParserSDStream,help_stream},
	{'R', CommandParserSDRead,help_read},
	{'V', CommandParserSDVolume,help_volume},
	{'F', CommandParserSDFormat,help_format},
	{'L', CommandParserSDLogTest,help_logtest},
	//{'l', CommandParserSDLogTest2,help_logtest2},
	{'B', CommandParserSDBench,help_sdbench},
	{'b', CommandParserSDBench2,help_sdbench2},
	{'1', CommandParserSDBench_t1,help_sdbench3},
	{'2', CommandParserSDBench_t2,help_sdbench2},
	{'H', CommandParserHelp,help_h},	
	{'!', CommandParserQuit,help_quit}	
};

unsigned char CommandParserSDErase(char *buffer,unsigned char size)
{
	// Parse arguments
	unsigned long addr1,addr2;
	unsigned char rv = ParseCommaGetLong(buffer,2,&addr1,&addr2);
	if(rv)
		return 2;
	
	if(addr1==0 && addr2==0)
	{
		addr2 = _fsinfo.card_capacity_sector-1;
	}
	printf_P(PSTR("Erase %lu-%lu\n"),addr1,addr2);
	
	if(sd_erase(addr1,addr2))
	{
		printf_P(PSTR("Erase error.\n"));
		return 1;
	}
	
	
	return 0;
}
/*unsigned char CommandParserSDErase2(char *buffer,unsigned char size)
{
	// Parse arguments
	unsigned long addr;
	unsigned char rv = ParseCommaGetLong(buffer,1,&addr);
	if(rv)
		return 2;
	
	printf("Erase end: %lu\n",addr);
	
	_sd_cmd33(addr);
	return 0;
}
unsigned char CommandParserSDErase3(char *buffer,unsigned char size)
{
	printf("try erase\n");
	//_sd_cmd38();
	_sd_cmd_38_wait();
	return 0;
}*/


unsigned char CommandParserSDBench(char *buffer,unsigned char size)
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
unsigned char CommandParserSDBench2(char *buffer,unsigned char size)
{
	// Parse arguments
	unsigned long startaddr,sizekb,preerasekb;
	unsigned char rv = ParseCommaGetLong(buffer,3,&startaddr,&sizekb,&preerasekb);
	if(rv)
		return 2;
	
	printf("Stream cache write from %lu up to %luKB\n",startaddr,sizekb);
	
	sd_bench_streamcache_write2(startaddr,sizekb*1024l,preerasekb*2l);
	return 0;
}
unsigned char CommandParserSDBench_t1(char *buffer,unsigned char size)
{
	// Parse arguments
	unsigned long startaddr,sizekb,preerasekb;
	unsigned char rv = ParseCommaGetLong(buffer,3,&startaddr,&sizekb,&preerasekb);
	if(rv)
		return 2;
	
	printf("Stream write from %lu up to %luKB\n",startaddr,sizekb);
	
	// preerase in sectors=preerase*1024/512=preerase*2l
	sd_bench_stream_write2(startaddr,sizekb*1024l,preerasekb*2l);
	return 0;
}
unsigned char CommandParserSDBench_t2(char *buffer,unsigned char size)
{
	// Parse arguments
	unsigned long startaddr,sizekb;
	unsigned char rv = ParseCommaGetLong(buffer,2,&startaddr,&sizekb);
	if(rv)
		return 2;
	
	printf("Write from %lu up to %luKB\n",startaddr,sizekb);
	
	//sd_bench_write(startaddr,sizekb*1024l);
	sd_bench_write2(startaddr,sizekb*1024l);
	return 0;
}

unsigned char CommandParserSDFormat(char *buffer,unsigned char size)
{
	unsigned int numlog;
	if(ParseCommaGetInt((char*)buffer,1,&numlog))
		return 2;
		
	fprintf_P(file_pri,PSTR("Formatting with %u log files\n"),numlog);
	ufat_format(numlog);
	return 0;
}
/*unsigned char CommandParserSDLogTest2(char *buffer,unsigned char size)
{
	unsigned char rv;
	unsigned int lognum,ch,sz,bsiz;
	rv = ParseCommaGetInt((char*)buffer,4,&lognum,&sz,&ch,&bsiz);
	if(rv)
		return 2;
		
	ufat_log_test2(lognum,sz,ch,bsiz);
	return 0;
}*/
unsigned char CommandParserSDLogTest(char *buffer,unsigned char size)
{
	unsigned char rv;
	unsigned int lognum,sz;
	rv = ParseCommaGetInt((char*)buffer,2,&lognum,&sz);
	if(rv)
		return 2;
	
	printf("lognum: %u\n",lognum);
	printf("sz: %u KB\n",sz);
	ufat_log_test(lognum,(unsigned long)sz*1024l,65536);
	return 0;
}
unsigned char CommandParserSDWrite(char *buffer,unsigned char size)
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
	char block[512];
	for(unsigned short i=0;i<512;i++)
		block[i]=data;
	rv = sd_block_write(sector,block);
	printf("Write result: %d\n",rv);
	
	return 0;
	
}
unsigned char CommandParserSDRead(char *buffer,unsigned char size)
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
	char block[512];
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
unsigned char CommandParserSDStream(char *buffer,unsigned char size)
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
	sd_stream_open(sector,0);
	while(len)
	{
		unsigned int effw=bsize;
		if(len<effw)
			effw=len;
		
		rv = sd_streamcache_write(block,effw,&curraddr);
		printf("Write %d. Return: %d. Current address: %lu\n",effw,rv,curraddr);
		len-=effw;
	}
	rv=sd_streamcache_close(&curraddr);
	printf("Close. Return: %d. Current address: %lu\n",rv,curraddr);
	
	return 0;
	
}
unsigned char CommandParserSDVolume(char *buffer,unsigned char size)
{	
	ufat_init();
	
	return 0;
	
}
unsigned char CommandParserSDInit(char *buffer,unsigned char size)
{
	unsigned char rv;
	CID cid;
	CSD csd;
	SDSTAT sdstat;
	unsigned long capacity_sector;	
	
	printf_P(PSTR("Init SD\r"));
		
	rv = sd_init(&cid,&csd,&sdstat,&capacity_sector);
	if(rv!=0)
	{
		printf_P(PSTR("Init SD failure (%d)\r"),rv);
		return 1;
	}
	_fsinfo.card_capacity_sector=capacity_sector;
	printf_P(PSTR("Init SD success (%d)\r"),rv);
	sd_print_cid(file_pri,&cid);
	sd_print_csd(file_pri,&csd);	
	sd_print_sdstat(file_pri,&sdstat);	
	printf_P(PSTR("Card capacity: %ld sectors\n"),capacity_sector);
	
	// Check that we deal with an SDHC card with block length of 512 bytes
	if(csd.CSD!=1 || csd.READ_BL_LEN!=9 || csd.WRITE_BL_LEN!=9)
	{
		printf("SD card unsuitable\n");
		return 1;
	}
	
	
	return 0;
}

unsigned char CommandParserSD(char *buffer,unsigned char size)
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
	
	fprintf_P(file_pri,PSTR("SD>\n"));
	
	while(1)
	{
	//	fprintf_P(file_pri,PSTR("Some ADC stuff: %d. period: %d. mask: %02X. pri: %p dbg: %p\n"),ctr,mode_adc_period,mode_adc_mask,file_pri,file_dbg);

		CommandProcess(CommandParsersSD,CommandParsersSDNum);
		if(CommandShouldQuit())
			break;
			
	}
	fprintf_P(file_pri,PSTR("<SD\n"));
}


