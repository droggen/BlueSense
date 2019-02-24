#ifndef __SD_H
#define __SD_H

#include "sd_bit.h"
#include "sd_int.h"

// MMC/SD commands
#define MMC_GO_IDLE_STATE						0
#define MMC_SEND_OP_COND						1
#define SD_SEND_IF_COND							8
#define MMC_SEND_CSD							9
#define MMC_SEND_CID							10
#define MMC_SEND_STATUS							13
#define MMC_SET_BLOCKLEN						16
#define MMC_READ_SINGLE_BLOCK					17
#define MMC_WRITE_BLOCK							24
#define MMC_WRITE_MULTIPLE_BLOCK				25
#define MMC_PROGRAM_CSD							27
#define MMC_SET_WRITE_PROT						28
#define MMC_CLR_WRITE_PROT						29
#define MMC_SEND_WRITE_PROT						30
#define MMC_TAG_SECTOR_START					32
#define MMC_TAG_SECTOR_END						33
#define MMC_UNTAG_SECTOR						34
#define MMC_TAG_ERASE_GROUP_START				35
#define MMC_TAG_ERARE_GROUP_END					36
#define MMC_UNTAG_ERASE_GROUP					37
#define MMC_ERASE								38
#define MMC_CMD55 								55
#define MMC_READ_OCR							58
#define MMC_CRC_ON_OFF							59

#define MMC_ACMD13 								13
#define MMC_ACMD23 								23
#define MMC_ACMD41 								41


#define MMC_STARTBLOCK             				0xFE
#define MMC_STARTMULTIBLOCK             		0xFC
#define MMC_STOPMULTIBLOCK             			0xFD

//#define MMC_TIMEOUT_ICOMMAND					50				// Timeout to receive an answer to simple commands (ms)
#define MMC_TIMEOUT_ICOMMAND					500				// Timeout to receive an answer to simple commands (ms)
#define MMC_TIMEOUT_READWRITE					1500			// Timeout to receive an answer to read/write commands (ms)
#define SD_ERASE_TIMEOUT						20000			// Timeout for erase command (ms)
#define SD_TIMEOUT_ACMD41 						800
#define MMC_RETRY								3				// Number of times commands are retried
#define SD_DELAYBETWEENCMD	1

#define SD_CHECK_BIT							0x80			// MSB set to 0 indicates R1 answer

#define SD_CACHE_SIZE 512


#define SD_CRC_CMD55							0x65

//#define MMCDBG
// Debug streaming write functions
//#define SD_DBG_STREAM 1

#define MMCPRECLOCK
#define MMCPOSTCLOCK
//#define MMCCLOCKMORE

extern unsigned short _sdbuffer_n;


void sd_select_n(char ss);



// Initialization functions
unsigned char sd_init(CID *cid,CSD *csd,SDSTAT *sdstat,unsigned long *capacity);

// Block read/write functions
unsigned char sd_block_write(unsigned long addr,char *buffer);
unsigned char sd_block_read(unsigned long addr,char *buffer);



// Multiblock streaming
extern unsigned char _sd_write_stream_open;
extern unsigned long _sd_write_stream_address;

void sd_stream_open(unsigned long addr,unsigned long preerase);
unsigned char sd_stream_close(unsigned long *currentaddr);
unsigned char sd_stream_write(char *buffer,unsigned short size,unsigned long *currentsect);
unsigned char sd_streamcache_write(char *buffer,unsigned short size,unsigned long *currentaddr);
//unsigned char sd_write_stream_write_block(unsigned char *buffer,unsigned long *currentaddr);
//unsigned char sd_write_stream_write_block2(unsigned char *buffer,unsigned long *currentaddr);
unsigned char sd_streamcache_close(unsigned long *currentaddr);

unsigned char sd_erase(unsigned long addr1,unsigned long addr2);

// Print functions
void sd_print_csd(FILE *f,CSD *csd);
void sd_print_cid(FILE *f,CID *cid);
void sd_print_sdstat(FILE *f,SDSTAT *sdstat);
void mmc_printcxd(CID *cid,CSD *csd,unsigned long capacity_byte);
void sd_print_ocr(FILE *f,OCR *ocr);
void sd_print_sdstat(FILE *f,SDSTAT *sdstat);


#endif