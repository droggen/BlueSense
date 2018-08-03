

//************************************************************************
//*	these are used to test issues
//*	http://code.google.com/p/arduino/issues/detail?id=505
//*	Reported by mark.stubbs, Mar 14, 2011
//*	The STK500V2 bootloader is comparing the seqNum to 1 or the current sequence 
//*	(IE: Requiring the sequence to be 1 or match seqNum before continuing).  
//*	The correct behavior is for the STK500V2 to accept the PC's sequence number, and echo it back for the reply message.
#define	_FIX_ISSUE_505_
//************************************************************************
//*	Issue 181: added watch dog timmer support
#define	_FIX_ISSUE_181_

#include "cpu.h"

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/boot.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <avr/eeprom.h>
#include <avr/common.h>
#include <stdlib.h>
#include <stdio.h>
#include "serial.h"
#include "stk500-command.h"
#include "system.h"
#include "main.h"
#include "adc.h"
#include "stk500.h"


FILE *file_bl;			// Bootloader interface
FILE *file_dbg;			// Debugging interface


// Define to read actual battery voltage
//#define ENABLE_TRUEVTARGET
//#define BOOTDBG
//#define DISABLE_EEPROM

// Settings
#define MAXERR 5




// HW and SW version, reported to AVRISP, must match version of AVRStudio
#define CONFIG_PARAM_BUILD_NUMBER_LOW	0
#define CONFIG_PARAM_BUILD_NUMBER_HIGH	0
#define CONFIG_PARAM_HW_VER				0x0F
#define CONFIG_PARAM_SW_MAJOR			2
#define CONFIG_PARAM_SW_MINOR			0x0A


// Calculate the address where the bootloader starts from FLASHEND and BOOTSIZE
// (adjust BOOTSIZE below and BOOTLOADER_ADDRESS in Makefile if you want to change the size of the bootloader)
// BOOTSIZE in bytes. Largest is 4096 words
// FLASHEND is in bytes
#define BOOTSIZE 8192
#define APP_END  (FLASHEND -BOOTSIZE + 1)


// Signature bytes are not available in avr-gcc io_xxx.h
#if defined (__AVR_ATmega8__)
	#define SIGNATURE_BYTES 0x1E9307
#elif defined (__AVR_ATmega16__)
	#define SIGNATURE_BYTES 0x1E9403
#elif defined (__AVR_ATmega32__)
	#define SIGNATURE_BYTES 0x1E9502
#elif defined (__AVR_ATmega8515__)
	#define SIGNATURE_BYTES 0x1E9306
#elif defined (__AVR_ATmega8535__)
	#define SIGNATURE_BYTES 0x1E9308
#elif defined (__AVR_ATmega162__)
	#define SIGNATURE_BYTES 0x1E9404
#elif defined (__AVR_ATmega128__)
	#define SIGNATURE_BYTES 0x1E9702
#elif defined (__AVR_ATmega1280__)
	#define SIGNATURE_BYTES 0x1E9703
#elif defined (__AVR_ATmega2560__)
	#define SIGNATURE_BYTES 0x1E9801
#elif defined (__AVR_ATmega2561__)
	#define SIGNATURE_BYTES 0x1e9802
#elif defined (__AVR_ATmega1284P__)
	#define SIGNATURE_BYTES 0x1e9705
#elif defined (__AVR_ATmega640__)
	#define SIGNATURE_BYTES  0x1e9608
#elif defined (__AVR_ATmega64__)
	#define SIGNATURE_BYTES  0x1E9602
#elif defined (__AVR_ATmega169__)
	#define SIGNATURE_BYTES  0x1e9405
#elif defined (__AVR_AT90USB1287__)
	#define SIGNATURE_BYTES  0x1e9782
#else
	#error "no signature definition for MCU available"
#endif







// 0 success, 1 timeout, 2 not message start, 3 not token, 4 wrong checksum
unsigned char getmessage(unsigned char *buffer,unsigned short *size,unsigned char *seq)
{
	unsigned char	checksum = 0;
	int c;
	

	// Get start byte
	c=getchar_timeout();
	if(c==-1)
		return 1;
	if(c!=MESSAGE_START)
		return 2;
	checksum = MESSAGE_START^0;
	
	// Get sequence number
	c=getchar_timeout();
	if(c==-1)
		return 1;
	*seq = c;
	checksum ^= c;

	// Get size
	c=getchar_timeout();
	if(c==-1)
		return 1;
	*size = c<<8;
	checksum ^= c;
	c=getchar_timeout();
	if(c==-1)
		return 1;
	*size = *size|c;
	checksum ^= c;
	
	// Get token
	c=getchar_timeout();
	if(c==-1)
		return 1;
	if(c!=TOKEN)
		return 3;
	checksum ^= c;
		
	for(unsigned i=0;i<*size;i++)
	{
		c=getchar_timeout();
		if(c==-1)
			return 1;
		buffer[i]=c;
		checksum ^= c;
	}
	
	c=getchar_timeout();
	if(c==-1)
		return 1;
	if(c!=checksum)
		return 4;
	
	return 0;
}
// Always succeed; returns the next sequence number (seq+1)
unsigned char sendmessage(unsigned char *buffer, unsigned short size, unsigned char seq)
{
	unsigned char checksum;
	unsigned char c;
	
	sendchar(MESSAGE_START);
	checksum = MESSAGE_START^0;
	
	sendchar(seq);
	checksum ^= seq;
	
	c = ((size>>8)&0xFF);
	sendchar(c);
	checksum ^= c;
	
	c = size&0x00FF;
	sendchar(c);
	checksum ^= c;
	
	sendchar(TOKEN);
	checksum ^= TOKEN;
	
	for(unsigned short i=0;i<size;i++)
	{
		c=buffer[i];
		sendchar(c);
		checksum ^=c;
	}
	sendchar(checksum);
	return seq++;
}

void stk500(void)
{
	unsigned char buffer[285];
	unsigned short size;
	unsigned char seq;
	unsigned char leave=0;
	address_t address = 0;
	char s[128];
	unsigned char err=0;
	
	//fputs("stk500\n",file_dbg);
	//delay_ms(100);

	while(!leave)
	{
		// Get message
		unsigned char rv = getmessage(buffer,&size,&seq);
		system_led_toggle(0b10);
		#ifdef BOOTDBG
		fprintf(file_dbg,"get message returned: %d. size: %d. seq: %d. command. %02x\n",rv,size,seq,buffer[0]);
		#endif
		if(rv!=0)
		{
			s[0]='!';
			s[1]='\n';
			s[2]=0;
			fputs(s,file_dbg);
			err++;
			if(err>MAXERR)
			{
				leave=1;
			}			
			continue;
		}
		
		
		//#ifdef BOOTDBG
		//msprintf(s,"M-\n",buffer[0]);
		mfprintf(file_dbg,"M-\n",buffer[0]);
			/*s[0]='M';
			s[1]=hex2chr((buffer[0]>>4)&0xf);
			s[2]=hex2chr((buffer[0]>>0)&0xf);
			s[3]='\n';
			s[4]=0;*/
			//fputs(s,file_dbg);
		//#endif
		if(rv!=0)
		{
			#ifdef BOOTDBG
			fprintf(file_dbg,"msg rcv failed: %d\n",rv);
			#endif
			return;
		}
					
		switch(buffer[0])
		{
			case CMD_SIGN_ON:
				#ifdef BOOTDBG
					fprintf(file_dbg,"CMD_SIGN_ON\n");
				#endif
				// Reset the address
				address=0;
				
				size = 11;
				buffer[1] 	=	STATUS_CMD_OK;
				buffer[2] 	=	8;
				buffer[3] 	=	'A';
				buffer[4] 	=	'V';
				buffer[5] 	=	'R';
				buffer[6] 	=	'I';
				buffer[7] 	=	'S';
				buffer[8] 	=	'P';
				buffer[9] 	=	'_';
				buffer[10]	=	'2';
				break;		
			case CMD_SPI_MULTI:
			{
				unsigned char answerByte;
								
				#ifdef BOOTDBG
				fprintf(file_dbg,"CMD_SPI_MULTI. numtx: %d. numrx: %d. rxstart: %d. txdata: ",buffer[1],buffer[2],buffer[3]);
				for(unsigned short i=0;i<size-4;i++)
					fprintf(file_dbg,"%02x ",buffer[4+i]);
				fprintf(file_dbg,"\n");
				#endif
	
				if(buffer[4]==0x30)
				{
					unsigned char signatureIndex	=	buffer[6];	
					if(signatureIndex == 0)
					{
						answerByte	=	(SIGNATURE_BYTES >> 16) & 0x000000FF;
					}
					else if ( signatureIndex == 1 )
					{
						answerByte	=	(SIGNATURE_BYTES >> 8) & 0x000000FF;
					}
					else
					{
						answerByte	=	SIGNATURE_BYTES & 0x000000FF;
					}
				}
				else if(buffer[4]==0x50 && buffer[5]==0x00)
				{
					answerByte	=	boot_lock_fuse_bits_get(GET_LOW_FUSE_BITS);
				}
				else if(buffer[4]==0x58 && buffer[5]==0x08)
				{
					answerByte	=	boot_lock_fuse_bits_get(GET_HIGH_FUSE_BITS);
				}
				else if(buffer[4]==0x50 && buffer[5]==0x08)
				{
					answerByte	=	boot_lock_fuse_bits_get(GET_EXTENDED_FUSE_BITS);
				}
				else
				{
					// undefined command
					#ifdef BOOTDBG
						//msprintf(s,"CMD_SPI_MULTI u: - - - : - - - -\n",buffer[1],buffer[2],buffer[3],buffer[4],buffer[5],buffer[6],buffer[7]);
						//fputs(s,file_dbg);
						mfprintf(file_dbg,"CMD_SPI_MULTI u: - - - : - - - -\n",buffer[1],buffer[2],buffer[3],buffer[4],buffer[5],buffer[6],buffer[7]);
						
					//#else
						//fputc('u',file_dbg);
					#endif
					answerByte = 0;
					//answerByte = 0xff;
				}
				
				size		=	7;
				buffer[1]	=	STATUS_CMD_OK;
				buffer[2]	=	0;
				buffer[3]	=	buffer[4];
				buffer[4]	=	0;
				buffer[5]	=	answerByte;
				buffer[6]	=	STATUS_CMD_OK;
				break;
			}		
			case CMD_GET_PARAMETER:
			{
				unsigned char value;
	
				#ifdef BOOTDBG
				fprintf(file_dbg,"CMD_GET_PARAMETER. Param: %d\n",buffer[1]);
				#endif
				switch(buffer[1])
				{
					case PARAM_BUILD_NUMBER_LOW:
						value	=	CONFIG_PARAM_BUILD_NUMBER_LOW;
						break;
					case PARAM_BUILD_NUMBER_HIGH:
						value	=	CONFIG_PARAM_BUILD_NUMBER_HIGH;
						break;
					case PARAM_HW_VER:
						value	=	CONFIG_PARAM_HW_VER;
						break;
					case PARAM_SW_MAJOR:
						value	=	CONFIG_PARAM_SW_MAJOR;
						break;
					case PARAM_SW_MINOR:
						value	=	CONFIG_PARAM_SW_MINOR;
						break;
					case PARAM_VTARGET:
					{
						#ifdef ENABLE_TRUEVTARGET
							unsigned short bv = ADC_read(7,1);
							unsigned short bv2= Battery_ADC2mv(bv);
							value = bv2/100;
						#else
							value=245;
						#endif
						break;
					}
					default:
						fputc('U',file_dbg);
						value	=	0;
						break;
				}
				size = 3;
				buffer[1] = STATUS_CMD_OK;
				buffer[2] = value;
				break;
			}
			case CMD_LOAD_ADDRESS:
				// Address is received in words, and converted (and stored internally) in bytes
				#if defined(RAMPZ)
					address	=	( ((address_t)(buffer[1])<<24)|((address_t)(buffer[2])<<16)|((address_t)(buffer[3])<<8)|(buffer[4]) )<<1;
				#else
					address	=	( ((buffer[3])<<8)|(buffer[4]) )<<1;		//convert word to byte address
				#endif
				#ifdef BOOTDBG
					s[0]='A';
					s[1]=hex2chr(buffer[1]>>4);
					s[2]=hex2chr(buffer[1]&0xf);
					s[3]=hex2chr(buffer[2]>>4);
					s[4]=hex2chr(buffer[2]&0xf);
					s[5]=hex2chr(buffer[3]>>4);
					s[6]=hex2chr(buffer[3]&0xf);
					s[7]=hex2chr(buffer[4]>>4);
					s[8]=hex2chr(buffer[4]&0xf);
					s[9]='\n';
					s[10]=0;
					fputs(s,file_dbg);
				#endif
				size		=	2;
				buffer[1]	=	STATUS_CMD_OK;
				break;
			case CMD_PROGRAM_FLASH_ISP:
			case CMD_PROGRAM_EEPROM_ISP:
			{
				unsigned int	size	=	((buffer[1])<<8) | buffer[2];
				unsigned char	*p	=	buffer+10;
				unsigned int	data;
				unsigned char	highByte, lowByte;
	
				#ifdef BOOTDBG
					s[0]='W';
					s[1]=hex2chr((size>>12)&0xf);
					s[2]=hex2chr((size>>8)&0xf);
					s[3]=hex2chr((size>>4)&0xf);
					s[4]=hex2chr((size>>0)&0xf);
					s[5]='\n';
					s[6]=0;
					fputs(s,file_dbg);
				#endif
				
				
				if(buffer[0] == CMD_PROGRAM_FLASH_ISP && address<APP_END)
				{
					// We prevent bootloader from modifying itself.
					/*s[0]='E';
					//s[1]=hex2chr((eraseAddress>>28)&0xf);
					//s[2]=hex2chr((eraseAddress>>24)&0xf);
					//s[3]=hex2chr((eraseAddress>>20)&0xf);
					//s[4]=hex2chr((eraseAddress>>16)&0xf);
					//s[5]=hex2chr((eraseAddress>>12)&0xf);
					//s[6]=hex2chr((eraseAddress>>8)&0xf);
					//s[7]=hex2chr((eraseAddress>>4)&0xf);
					//s[8]=hex2chr((eraseAddress>>0)&0xf);
					s[1]=hex2chr((address>>28)&0xf);
					s[2]=hex2chr((address>>24)&0xf);
					s[3]=hex2chr((address>>20)&0xf);
					s[4]=hex2chr((address>>16)&0xf);
					s[5]=hex2chr((address>>12)&0xf);
					s[6]=hex2chr((address>>8)&0xf);
					s[7]=hex2chr((address>>4)&0xf);
					s[8]=hex2chr((address>>0)&0xf);
					
					s[9]='\n';
					s[10]=0;
					fputs(s,file_dbg);*/
						
					//boot_page_erase(eraseAddress);			// Perform page erase, byte address
					ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
					{
						boot_page_erase(address);				// Perform page erase, byte address
						boot_spm_busy_wait();								// Wait until the memory is erased.
						boot_rww_enable();
					}
					//eraseAddress += SPM_PAGESIZE;				// point to next page to be erase
	
					#ifdef BOOTDBG
						s[0]='w';
						s[1]=hex2chr((address>>28)&0xf);
						s[2]=hex2chr((address>>24)&0xf);
						s[3]=hex2chr((address>>20)&0xf);
						s[4]=hex2chr((address>>16)&0xf);
						s[5]=hex2chr((address>>12)&0xf);
						s[6]=hex2chr((address>>8)&0xf);
						s[7]=hex2chr((address>>4)&0xf);
						s[8]=hex2chr((address>>0)&0xf);
						s[9]='\n';
						s[10]=0;
						fputs(s,file_dbg);
					#endif
					// Write FLASH 
					// Not disabling interrupts leads to problem, with some address not being filled (0xffff).
					// There does not seem to be a need to disable interrupts in the write and busywait phases.
					ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
					{
						for(unsigned short i=0;i<size;i+=2)
						{
							lowByte		=	*p++;
							highByte 	=	*p++;
		
							data		=	(highByte << 8) | lowByte;
							
							/*if(address&0x100)
								data=0x5555;
							else
								data=0xaaaa;*/
							
							boot_page_fill(address+i,data);			// Byte address
						}
					
						boot_page_write(address);				
						boot_spm_busy_wait();
						boot_rww_enable();				// Re-enable the RWW section
						// Next address
						address+=size;
					}
				}
				else
				{
					#ifndef DISABLE_EEPROM
						// EEPROM
						//	issue 543, this should work, It has not been tested.
						uint16_t ii = address >> 1;
						// write EEPROM 
						while (size) {
							eeprom_write_byte((uint8_t*)ii, *p++);
							address+=2;						// Select next EEPROM byte
							ii++;
							size--;
						}
					#endif
				}
				size = 2;
				buffer[1]	=	STATUS_CMD_OK;
				break;
			}
			case CMD_READ_FLASH_ISP:
			case CMD_READ_EEPROM_ISP:
			{
				unsigned short readsize = ((buffer[1])<<8) | buffer[2];
				unsigned char *p = buffer+1;
				size = readsize+3;
	
				#ifdef BOOTDBG
					s[0]='R';
					s[1]=hex2chr((readsize>>12)&0xf);
					s[2]=hex2chr((readsize>>8)&0xf);
					s[3]=hex2chr((readsize>>4)&0xf);
					s[4]=hex2chr((readsize>>0)&0xf);
					s[5]='\n';
					s[6]=0;
					fputs(s,file_dbg);
				#endif
	
				*p++	=	STATUS_CMD_OK;
				if(buffer[0] == CMD_READ_FLASH_ISP)
				{
					unsigned short data;
	
					// Read FLASH
					do {
						data	=	pgm_read_word_far(address);
						*p++	=	(unsigned char)data;		//LSB
						*p++	=	(unsigned char)(data >> 8);	//MSB
						address	+=	2;							// Select next word in memory
						readsize	-=	2;
					}while(readsize);
				}
				else
				{
					#ifndef DISABLE_EEPROM
						// Read EEPROM 
						do {
							EEARL	=	address;			// Setup EEPROM address
							EEARH	=	((address >> 8));
							address++;					// Select next EEPROM byte
							EECR	|=	(1<<EERE);			// Read EEPROM
							*p++	=	EEDR;				// Send EEPROM data
							readsize--;
						} while(readsize);
					#endif
				}
				*p++	=	STATUS_CMD_OK;
				break;
			}
			case CMD_CHIP_ERASE_ISP:
				size = 2;
				//buffer[1] = STATUS_CMD_OK;
				buffer[1] = STATUS_CMD_FAILED;	//*	isue 543, return FAILED instead of OK
				break;
			case CMD_READ_SIGNATURE_ISP:
			case CMD_READ_LOCK_ISP:
			case CMD_READ_FUSE_ISP:
				#ifdef BOOTDBG
					fputs("CMD:NOTIMPL\n",file_dbg);
				#endif
				size = 2;
				buffer[1] = STATUS_CMD_FAILED;
				break;
			case CMD_SET_PARAMETER:
			case CMD_ENTER_PROGMODE_ISP:
			case CMD_LEAVE_PROGMODE_ISP:
			default:
				size = 2;
				buffer[1]	=	STATUS_CMD_OK;
				if(buffer[0]==CMD_LEAVE_PROGMODE_ISP)
					leave=1;
				break;
		}
		#ifdef BOOTDBG
		fputs("Sending answer\n",file_dbg);
		#endif
		seq = sendmessage(buffer,size,seq);
	}
	fputs("Leaving ISP\n",file_dbg);

}
/*
unsigned char echodbg2bt(unsigned char c)
{
	//fputc(c,file_dbg);
	//fputc('.',file_dbg);
	fputc(c,file_dbg);
	return 1;
}
*/



