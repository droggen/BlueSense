#include "cpu.h"
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include "wait.h"
#include "spi.h"
#include "main.h"
#include "sd.h"

/*
	File: sd
	
	SD card functions.
	
	This provides low-level communication with SDHC cards (to check: compatibility with SDXC). 
	
	This library requires cards compatible with the Physical Spec version 2.00. This rules out
	some standard capacity SD cards and MMC cards.
	
	As this library uses the Physical Spec Version 2.0 all card read/write operations must be done on entire sectors (512-bytes) and the address of read and write operations are given in sectors.
	
	*General functions*
	
	* sd_init:							Initialise the SD card and returns card CID, CSD and capacity.
	* sd_print_csd:						Prints the CSD fields to a stream.
	* sd_print_cid:						Prints the CID fields to a stream.
	* sd_print_ocr						Prints the OCR fields to a stream.

	*Individual block read/write*
	
	SD card sectors (512-bytes blocks) can be read and written with:
	
	* sd_block_write: 					Writes a sector of data at sector addr.
	* sd_block_read:					Reads a sector of data from sector addr.
	
	The address is the absolute sector number on the SD card.
	
	
	*Streaming writes*

	Stream write functions allow to initiate (open) a "streaming write" operation to the memory card, 
	and write one or more data blocks. Internally, SD-card "multiblock writes" are used to speed-up writes (i.e. it avoids group erases after single sector writes).
	
	To perform a streaming write:
	
		1. Open the block to write into with sd_stream_open
		2. Provide the block data with any number of calls to sd_stream_write
		3. Close the block with sd_stream_close

	sd_stream_write returns whenever a complete data block is written to the card, so that the application can decide e.g. to update a FAT 
	(sd_stream_close must be called first to terminate the ongoing multiple block write if a e.g. a FAT were to be updated). It is the application's responsibility
	to call sd_stream_write multiple times if needed until all the data has been written.
	
	The following functions are availablefor streaming writes:
	
	* sd_stream_open:				Start a stream write at the specified address (used both for caching and non-caching streaming writes).
	* sd_stream_write:				Writes data in streaming multiblock write
	* sd_stream_close:				Terminates a streaming write.

	*Streaming writes with caching*
	
	Streaming write with caching allows to reach higher performance than streaming write without. 

	To perform a streaming write with caching:
	
		1. Open the block to write into with sd_stream_open
		2. Provide the block data with any number of calls to sd_streamcache_write
		3. Close the block with sd_streamcache_close

	sd_streamcache_write returns only if the entire data has been written to the card or to the cache, or if there is an error.
	Note that this behavior is different from sd_stream_write which returns when a block is completed or all the data is written.
	
	
	* sd_stream_open:				Start a stream write at the specified address (used both for caching and non-caching streaming writes).
	* sd_streamcache_write:			Writes data in streaming multiblock write with caching.
	* sd_streamcache_close			Finishes a multiblock write with caching.
	
	*Dependencies*
	
	* spi
	* Card on SPI interface: this library assumes the card is interfaced on the SPI interface. The SPI interface must be 
	initialised before using this library.
	
	
	
	
	*Usage in interrupts*
	Not suitable for use in interrupts.
	
	
	*Possible improvements*
	Reduce or remove delay prior to command
*/



/******************************************************************************
	function: sd_init
*******************************************************************************	
	Initialise the SD card and returns card CID, CSD and capacity.
	
	This function supports SDHC cards (to check: SDXC). It does not
	support version 1 SD cards nor MMC cards.
	
	This function requires the SPI interface to have been initialised.
	
	The initialisation procedure is as follows:
	- Issue clock pulses
	- Issue CMD0 with cs
	- Issue CMD8 (SEND_IF_COND) (required to be compliant with Physical Spec Version 2.00) and check echo back and voltage range
	- [Issue CMD58 (Read OCR) and check voltage range] This is unnecessary as CMD58 is issued after ACMD41 again for extended info.
	- Issue ACMD41
	- Issue CMD58 (Read OCR) and check voltage range
	- Read CSD
	- Read CID	
	
	Parameters:
		cid			-	Pointer to receive card CID
		csd			-	Pointer to receive card CSD
		capacity	-	Pointer to receive card capacity in sectors
		
	Returns:
		0			-	Success
		1			-	Error
******************************************************************************/
unsigned char sd_init(CID *cid,CSD *csd,unsigned long *capacity)
{
	unsigned char c;
	int i;
	unsigned char response[32];
	unsigned char r1;
	OCR ocr;
	unsigned long t1;

	

	// 0. Native initialization
	// This was used for SD/MMC. Although it does not appear necessary with SDHC, 
	// it speeds up some of the initialisation, likely bringing the card in a known
	// state.
	sd_select_n(0);
	_delay_ms(SD_DELAYBETWEENCMD);
	for(i=0; i < 16; i++) spi_rw_noselect(0xFF); // send 10*8=80 clock pulses
	_delay_ms(SD_DELAYBETWEENCMD);
	sd_select_n(1);		// Make sure we are in SDC mode
	
	// 1. CMD0: GO_IDLE_STATE, Response R1	
	// Expected answer: 1 (mask ff)
	#ifdef MMCDBG
		printf_P(PSTR("MMC_GO_IDLE_STATE\n"));
	#endif
	c=_sd_command_r1_retry(MMC_GO_IDLE_STATE,0,0,0,0,0xff,0x01,&r1);					// Answer mask: FF, OK answer: 01
	#ifdef MMCDBG
		printf_P(PSTR("MMC_GO_IDLE_STATE: %02X. %02X\r\n"),c,r1);
	#endif
	if(c)
	{
		printf_P(PSTR("SD: reset error\n"));
		return 1;																// Error
	}
	

	
	// 2. CMD8: SEND_IF_COND, Response R7
	// p1: 00000000 p2: 00000000 p3: 0000vvvv p4: kkkkkkkk. v: supported voltage, k: echo back pattern
	// Response R7: response length of 5 bytes (40 bits)
	// Expected answer: 1 (mask ff)
	#ifdef MMCDBG
	printf_P(PSTR("SD_SEND_IF_COND\n"));
	#endif
	unsigned char echo='x';
	unsigned char voltage=0b1;		// 2.7-3.6V
	c = _sd_command_rn_retry(SD_SEND_IF_COND,0,0,voltage,echo,response,5,0xff,0x01);		// 5 bytes answer, answer mask: FF, OK answer: 01
	#ifdef MMCDBG
	printf_P(PSTR("SD_SEND_IF_COND: %02X\n"),c);
	for(i=0;i<5;i++)
		printf_P(PSTR("%02X "),response[i]);
	printf_P(PSTR("\n"));
	#endif
	if(c || response[4]!=echo || (response[3]&0b1111)!=voltage)
	{
		printf_P(PSTR("SD: card not supported (may be version 1 or otherwise incompatible)\n"));
		return 1;
	}
	
	
	// At this stage we are guaranteed to have a card with Physical Spec Version 2.00 
	
	
	
	/*
	// x. Enable CRC
	// This was used for SD/MMC, but removed as appears unnecessary with SDHC.
	//#ifdef MMCDBG
	printf_P(PSTR("MMC_CRC_ON_OFF\n"));
	//#endif
	c=_sd_command_retry(MMC_CRC_ON_OFF,0,0,0,0,0xff,0x01,&r1);
	//c=_sd_command_retry(MMC_CRC_ON_OFF,0,0,0,1,0xff,0x00,&r1);
	//#ifdef MMCDBG
	printf_P(PSTR("MMC_CRC_ON_OFF: %02X. %02X\r\n"),c,r1);
	//#endif
	if(c)
		return 1;*/


	
	
	// 3. CMD58 read OCR to check voltage range
	// This is unnecessary as CMD58 is issued after ACMD41 again for extended info.
	/*
	if(_sd_cmd58(&ocr))
	{
		printf_P(PSTR("SD: CMD58 error\n"));
		return 1;
	}
	sd_print_ocr(file_pri,&ocr);
	
	if(!ocr.v3031 && !ocr.v2930)
	{
		printf_P(PSTR("SD: Unsupported voltage\n"));
		return 1;
	}
	*/
	
	// 4. ACMD41 (SD_SEND_OP_COND) to start initialisation and check for initialisation completed
	// ACMD are application-specific commands, and must be prefixed by a CMD55.
	t1 = timer_ms_get();
	do
	{
		//printf_P(PSTR("CMD55/ACMD41\n"));
		_delay_ms(SD_DELAYBETWEENCMD);
		// 4a. CMD55
		// CMD55 must precede ACMD; indicates next command is application
		// p1-p4 are 0
		// Reponse R1. 
		// Assume that this always succeeds.		
		#ifdef MMCDBG
		printf_P(PSTR("CMD55\n"));
		#endif
		c = _sd_command_r1_retry(MMC_CMD55,0,0,0,0,0xfe,0x00,&r1);				// p1-p4=0, Mask to check idle bit, idle must be 0
		#ifdef MMCDBG
		printf_P(PSTR("CMD55: %02X. %02X\r\n"),c,r1);
		#endif
	
	
		// 4b. ACMD41 (SD_SEND_OP_COND) 
		// p[31]=0, p[30]=hcs, p[29:0]=0
		// Response R1
		#ifdef MMCDBG
		printf_P(PSTR("ACMD41\n"));
		#endif
		unsigned char hcs=1;		// We supports SDHC/SDXC
		c = _sd_command_r1_retry(MMC_ACMD41,(hcs&1)<<6,0,0,0,0xfe,0x00,&r1);		// Response idle bit must be 0
		//c = _sd_command_retry(MMC_ACMD41,0xff,0,0,0,0xff,0x00,&r1);
		#ifdef MMCDBG
		printf_P(PSTR("ACMD41: %02X. %02X\r\n"),c,r1);
		#endif
	}
	while(r1!=0 && (timer_ms_get()-t1<SD_TIMEOUT_ACMD41));
	//printf("time: %lu\n",timer_ms_get()-t1);

	if(r1!=0)
	{
		printf_P(PSTR("SD: ACMD41 fail\n"));
		return 1;
	}
	
	// 5. CMD58 read OCR to check voltage range
	if(_sd_cmd58(&ocr))
	{
		printf_P(PSTR("SD: CMD58 error\n"));
		return 1;
	}
	//sd_print_ocr(file_pri,&ocr);
	
	if(!ocr.v3031 && !ocr.v2930)
	{
		printf_P(PSTR("SD: Unsupported voltage\n"));
		return 1;
	}	
		
		
	

	// Set block length to 512 bytes
	// Only for SD Physical Spec 1.0
	/*
	printf_P(PSTR("MMC_SET_BLOCKLEN\n"));
	c=MMC_Command(MMC_SET_BLOCKLEN,0,0,0x02,0x00,0x95);	
	printf_P(PSTR("Result of MMC_SET_BLOCKLEN: %X\r\n"),c);
	if(c!=0)
		return 4;
	*/

	// 6. MMC_SEND_CSD
	_sd_cmd9(csd,capacity);
	// 7. Read CID
	_sd_cmd10(cid);
	
	return 0;
}





void sd_select_n(char ss)
{
	ss=ss?1:0;
	PORTB=(PORTB&0xEF)|(ss<<4);

}




/************************************************************************************************************************************************************
*************************************************************************************************************************************************************
BLOCK READ/WRITE   BLOCK READ/WRITE   BLOCK READ/WRITE   BLOCK READ/WRITE   BLOCK READ/WRITE   BLOCK READ/WRITE   BLOCK READ/WRITE   BLOCK READ/WRITE   
*************************************************************************************************************************************************************
************************************************************************************************************************************************************/

/******************************************************************************
	function: sd_block_read
*******************************************************************************
	Reads a sector of data from sector addr.
	
	The block size is fixed at 512 bytes, i.e. one sector.
	Uses internally the single block write function.
	
	Parameters:
		addr		-	Address in sector (0=first sector, 1=second sector, ...)
		buffer		-	Buffer of 512 bytes which receives the data
	
	Returns:
		0			- 	Success
		nonzero		- 	Error
******************************************************************************/
unsigned char sd_block_read(unsigned long addr,unsigned char *buffer)
{
	unsigned char r1;
	unsigned short checksum;
	unsigned char response = _sd_command_r1_readblock(MMC_READ_SINGLE_BLOCK,addr>>24,addr>>16,addr>>8,addr,&r1,buffer,512,&checksum);
	return response;
}

/******************************************************************************
	function: sd_block_write
*******************************************************************************
	Writes a sector of data at sector addr.
	
	The block size is fixed at 512 bytes, i.e. one sector.
	Uses internally the single block write function.
	
	Parameters:
		addr		-	Address in sector (0=first sector, 1=second sector, ...)
		buffer		-	Buffer of 512 bytes of data to write
	
	Returns:
		0			- 	Success
		nonzero		- 	Error
******************************************************************************/
unsigned char sd_block_write(unsigned long addr,unsigned char *buffer)
{
	unsigned char response;

	response=_sd_block_open(addr);

	#ifdef MMCDBG
		printf_P(PSTR("_sd_block_open, open returned: %X\n"),response);
	#endif

	if(response!=0)
	{
		#ifdef MMCDBG
		printf_P(PSTR("sd_block_write failed\n"));
		#endif
		return response;
	}
	

	// This operation never fails: it's SPI transmit only.
	_sd_writebuffer(buffer,512);
	

	response = _sd_block_close();

	#ifdef MMCDBG
		printf_P(PSTR("_sd_block_close, returned %X\n"),response);
	#endif

	return response;
}




/************************************************************************************************************************************************************
*************************************************************************************************************************************************************
STREAM WRITE W/O CACHING   STREAM WRITE W/O CACHING   STREAM WRITE W/O CACHING   STREAM WRITE W/O CACHING   STREAM WRITE W/O CACHING   STREAM WRITE W/O CACHING   
*************************************************************************************************************************************************************
************************************************************************************************************************************************************/

unsigned char _sd_write_stream_open;					// Block open for writing
unsigned long _sd_write_stream_address;					// Write address in sectors
unsigned long _sd_write_stream_mustwait;				// Whether wait for end of block requied
unsigned short _sd_write_stream_numwritten;				// Number of bytes written within a block by a stream function
unsigned char _sd_write_stream_block_started;			// Block is started (i.e. data token has been transmitted)
unsigned long _sd_write_stream_t1;						// Time of conclusion of block write
unsigned long _sd_write_stream_error;

unsigned char _sdbuffer[SD_CACHE_SIZE];					// Cache memory for streaming writes with caching, also used for padding
unsigned short _sdbuffer_n;								// Amount of memory used in cache

/******************************************************************************
	function: sd_stream_open
*******************************************************************************
	Start a stream write at the specified address (used both for caching and non-caching streaming writes).

	This function initialises the state machine internally used for streaming writes.
	Internally, multiblock transfers to the SD card are used.
	
	Parameters:
		addr	-		Write start address in sectors
		
			
******************************************************************************/
void sd_stream_open(unsigned long addr)
{
	_sd_write_stream_open=0;						// Command write multiblock not sent yet
	_sd_write_stream_address=addr;					// Write address
	_sd_write_stream_mustwait=0;					// Must wait for answer from card
	_sd_write_stream_block_started = 0;				// Start data token not sent yet
	_sd_write_stream_numwritten = 0;				// No bytes written yet
	_sdbuffer_n=0;									// Number of data into buffer	
	_sd_write_stream_error=0;						// Number of errors
	
}


/******************************************************************************
	function:	sd_stream_write
*******************************************************************************
	Writes data in streaming multiblock write

	This function does not use any cache to hide the card write time and thus speedup writes.
	
	This function returns whenever a complete data block is written to the card, so that the application can decide e.g. to update a FAT 
	(sd_stream_close must be called first to terminate the ongoing multiple block write if a e.g. a FAT were to be updated). 
	It is the application's responsibility 	to call sd_stream_write multiple times if needed until all the data has been written.
	
	In the current implementation it is not possible to distinguish from the return value whether the function returns because a 512-bytes block is completed 
	or because all the user-specified bytes are written. The variable currentaddr may be used for this or an auxiliary user counter must be used for this.
	
	In case of error in sd_stream_write the following occurs:
		- the multiple block write is terminated (application needs not call sd_stream_close - this is internally done)
		- the stream is in closed state - attempt to open it will be done the next time sd_stream_write is called.
		- Either the block where the write error is skipped (no valid data within that block) and writes continues from the next one, or the same block is reopened if the error occurred during opening the block

	Parameters:
		buffer			-	Buffer of data to write	
		size			-	Number of bytes to write
		currentsect		-	Optionally a pointer to a variable holding the address (in sectors) of the block currently being used to store the data. 
							If 0, the address will not be provided

	Returns:
		0				-	Success
		other			-	Failure
******************************************************************************/
unsigned char sd_stream_write(unsigned char *buffer,unsigned short size,unsigned long *currentsect)
{
	// Find how many bytes can effectively be written until the sector is full.
	unsigned long effw;
	unsigned char rv;

	#ifdef MMCDBG
		//printf_P(PSTR("sd_stream_write. size: %d. strmopen: %d strmstrt: %d numwritten: %ld addr: %lX\r"),_sd_write_stream_open,_sd_write_stream_started,_sd_write_stream_numwritten,_sd_write_stream_address);
	#endif

	// Within this call: number of bytes sent to the card (errors not accounted for) and whether a block is completed
	if(currentsect)
		*currentsect = _sd_write_stream_address;
		
	// if nothing to write, success
	if(size==0)
		return 0;


	//	Write command not yet send.
	if(!_sd_write_stream_open)
	{
		rv = _sd_multiblock_open(_sd_write_stream_address);
		
		if(rv)
		{
			#ifdef MMCDBG
				printf_P(PSTR("sd_stream_write. size: %ld. strmopen: %d strmstrt: %d numwritten: %ld addr: %lX\r"),size,_sd_write_stream_open,_sd_write_stream_started,_sd_write_stream_numwritten,_sd_write_stream_address);
				printf_P(PSTR("sd_stream_write. _sd_multiblock_open failed\r"));
			#endif
			return 1;								// Abort if the open failed
		}	

		_sd_write_stream_open = 1;					// Block write is now open,...
	}

	// Block start not yet sent
	if(!_sd_write_stream_block_started)
	{
		spi_rw_noselect(MMC_STARTMULTIBLOCK);		// Send Data Token
		_sd_write_stream_block_started=1;			// Block has started
	}
	
	// Write the data until: either all data is written, or a block is completed (whichever comes first)
	// Find the suitable write size: the smallest smallest of size or 512-_sd_write_stream_numwritten
	if(size<512-_sd_write_stream_numwritten)
		effw=size;
	else
		effw=512-_sd_write_stream_numwritten;
		
	// Send data
	_sd_writebuffer(buffer,effw);
	
	// Update the counters
	_sd_write_stream_numwritten+=effw;
	
	// If a block is full, terminates it
	if(_sd_write_stream_numwritten>=512)
	{
		// Reset the internal state for the next block
		_sd_write_stream_block_started=0;								// Block hasn't started
		_sd_write_stream_numwritten=0;
	
		// Stop block
		rv = _sd_block_stop();

		// Increment the write address to the next sector for the next write call.
		_sd_write_stream_address+=1;

		// If the call failed we terminate the multiblock write 
		if(rv!=0)
		{
			_sd_multiblock_close();
			_sd_write_stream_open=0;							// Multiblock write not open
			return 2;
		}
	}

	

	// Success

	return 0;
}



/******************************************************************************
	sd_stream_close
*******************************************************************************	
	Terminates a streaming write.

	First pads the last block with padchar and writes it. 
	Then terminates the multi block write and releases the card.

	Parameters:
		currentsect		-	Optionally a pointer to a variable holding the address (in sectors) of the last block to hold data of the streaming write.
							If 0, the address will not be provided


	Return value:
		0:			ok
		other:	error
******************************************************************************/
unsigned char sd_stream_close(unsigned long *currentsect)
{
	unsigned char response;

	#ifdef MMCDBG
		printf_P(PSTR("sd_stream_close. strmopen: %d strmstrt: %d numwritten: %ld addr: %lX\r"),_sd_write_stream_open,_sd_write_stream_started,_sd_write_stream_numwritten,_sd_write_stream_address);
	#endif
	
	
	if(currentsect)
		*currentsect = _sd_write_stream_address;
	
	if(_sd_write_stream_block_started)
	{
		unsigned short topad = 512-_sd_write_stream_numwritten;

		// Write
		sd_stream_write(_sdbuffer,topad,0);

		// Flag as closed, even if the stop operation may fail
		_sd_write_stream_block_started=0;

		// Stop the block
		response = _sd_block_stop();
	}
	
	// Terminates the multiblock write
	response = _sd_multiblock_close();
	
	// Flag as closed
	_sd_write_stream_open=0;

	return response;
}

/************************************************************************************************************************************************************
*************************************************************************************************************************************************************
STREAM WRITE W/ CACHING   STREAM WRITE W/ CACHING   STREAM WRITE W/ CACHING   STREAM WRITE W/ CACHING   STREAM WRITE W/ CACHING   STREAM WRITE W/ CACHING   
*************************************************************************************************************************************************************
************************************************************************************************************************************************************/


/******************************************************************************
	function:	sd_streamcache_write
*******************************************************************************
	Writes data in streaming multiblock write with caching.

	Rationale for caching: after completing a block the card needs some time to be ready for a new block. This time is
	generally short but occasionnally may be longer when block reordering occurs.
	If sd_streamcache_write is called while the card is still busy from a prior write the data is moved into the cache and the function returns immediately.
	If the cache has no space for the data then the function will block until the cards is ready and will open a new block and write directly the data to the card.
	Upon subsequent writes, or calling sd_streamcache_close, the cache content is written to the card.
		
	This function uses the cache buffer _sdbuffer and ensures it does not grow above SD_CACHE_SIZE.

	Parameters:
		buffer			-	Buffer of data to write	
		size			-	Number of bytes to write. Use size=0 to flush the 
		currentsect		-	Optionally a pointer to a variable holding the address (in sectors) of the block currently being used to store the data. 
							If 0, the address will not be provided

	Returns:
		0				-	Success
		other			-	Failure
******************************************************************************/
unsigned char sd_streamcache_write(unsigned char *buffer,unsigned short size,unsigned long *currentsect)
{
	// Find how many bytes can effectively be written until the sector is full.
	unsigned long effw;
	unsigned char rv,error;

	// Error indicates the number of errors that occurred during this function. Normally it should remain 0.
	error=0;			
	#ifdef MMCDBG
		//printf_P(PSTR("sd_write_stream_write_partial. size: %d. strmopen: %d strmstrt: %d numwritten: %ld addr: %lX\r"),_sd_write_stream_open,_sd_write_stream_started,_sd_write_stream_numwritten,_sd_write_stream_address);
	#endif

	// This loop will be iterated to write size bytes to the card every time a block is completed
sd_streamcache_write_loop:
	//fputc('W',file_pri);
	//fprintf(file_pri,"write2. size: %u. _sdbuffer_n: %u\n",size,_sdbuffer_n);

	// Update current sector written to
	if(currentsect)
		*currentsect = _sd_write_stream_address;
		
	// Wait for card to become ready from previous block write (_sd_write_stream_mustwait)
	
	//while(_sd_write_stream_mustwait || size==0)		// DAN 17.10.2016: why || size==0? This seems a bug
	while(_sd_write_stream_mustwait)					// DAN 17.10.2016: to test, a-priori no need to enter this loop if size=0
	{
//		fputc('w',file_pri);
		// Get card status
		rv = spi_rw_noselect(0xFF);
		if(rv==0xff)
		{
			//fputc('r',file_pri);
			// Card responds that it is ready: indicate wait not needed and exit from loop
			_sd_write_stream_mustwait=0;
			break;
		}
		// Check the time elapsed from _sd_block_stop_nowait for a timeout.
		if(timer_ms_get()-_sd_write_stream_t1>=MMC_TIMEOUT_READWRITE)
		{
			// Timeout waiting for card. Close the block
			//fputc('t',file_pri);			
			_sd_write_stream_error++;
			error++;
			_sd_multiblock_close();
			_sd_write_stream_open=0;							// Multiblock write not open
			break;
		}
		// If size is nonzero (don't force writing buffer) and user data fits in cache buffer then store user data in cache and return immediately with success
		// If the user data does not fit in the cache buffer then this while loop will block until the card is ready or a timeout occurs
		// The size test ensures the cache buffer never holds more than SD_CACHE_SIZE bytes.
		if(size && size<=SD_CACHE_SIZE-_sdbuffer_n)
		{
			//fputc('b',file_pri);
			memcpy(_sdbuffer+_sdbuffer_n,buffer,size);
			_sdbuffer_n+=size;
			//return error|0x80;
			return error;
		}
	}
	// Here: card is ready (transaction/block may or may not be open/closed)
		
	// Nothing to write in the user-provided buffer nor in the cache buffer therefore returns successfully.
	if(size==0 && _sdbuffer_n==0)
		//return error|0x40;
		return error;
		
	
	//	Write command not yet send.
	if(!_sd_write_stream_open)
	{
		//fputc('o',file_pri);
		rv = _sd_multiblock_open(_sd_write_stream_address);
		
		if(rv)
		{
			#ifdef MMCDBG
				printf_P(PSTR("sd_write_stream_write_partial. size: %ld. strmopen: %d strmstrt: %d numwritten: %ld addr: %lX\r"),size,_sd_write_stream_open,_sd_write_stream_started,_sd_write_stream_numwritten,_sd_write_stream_address);
				printf_P(PSTR("sd_write_stream_write_partial. _sd_write_stream_address failed\r"));
			#endif
			_sd_write_stream_error++;
			error++;
			//return error|0x20;			// Abort if the open failed
			return error;
		}	
		_sd_write_stream_open = 1;						// Block write is now open,...	
		_sd_write_stream_block_started=0;			// Block has not started
	}

	// Block start not yet sent
	if(!_sd_write_stream_block_started)
	{
		//fputc('b',file_pri);
		spi_rw_noselect(MMC_STARTMULTIBLOCK);	// Send Data Token
		_sd_write_stream_block_started=1;			// Block has started
		_sd_write_stream_numwritten=0;				// So far wrote 0
	}
		
	// Write the data until: either all data is written, or a block is completed (whichever comes first)
	// First write data in cache buffer, if any. It is always possible to write the entire cache buffer.
	_sd_writebuffer(_sdbuffer,_sdbuffer_n);
	_sd_write_stream_numwritten+=_sdbuffer_n;
	_sdbuffer_n=0;
	
	//printf("num: %u\n",_sd_write_stream_numwritten);
	
	// Write the user-provided buffer or a subset of it until card the block is full	
	if(size<=512-_sd_write_stream_numwritten)
		effw=size;
	else
		effw=512-_sd_write_stream_numwritten;
		
	// Send data and update counters
	_sd_writebuffer(buffer,effw);
	_sd_write_stream_numwritten+=effw;
	// Update the buffer pointer and counter
	buffer+=effw;
	size-=effw;
	
	//printf("num: %u\n",_sd_write_stream_numwritten);
	
	
	// If a block is full, terminates it
	if(_sd_write_stream_numwritten>=512)
	{
		//fputc('s',file_pri);
		// Reset the internal state for the next block
		_sd_write_stream_block_started=0;								// Block hasn't started
	
		// Increment the write address to the next sector for the next write call.
		_sd_write_stream_address+=1;
	
		// Stop block
		//rv = _sd_block_stop();
		rv = _sd_block_stop_nowait();
		_sd_write_stream_t1=timer_ms_get();
		_sd_write_stream_mustwait=1;

		// If the call failed we terminate the multiblock write 
		if(rv!=0)
		{
			//fputc('e',file_pri);
			// Wait for the block to complete
			_sd_write_stream_error++;
			error++;
			_sd_block_stop_dowait();
			_sd_multiblock_close();
			_sd_write_stream_open=0;							// Multiblock write not open
			_sd_write_stream_mustwait=0;
		}
	}
	
	// If size is non null, loop to write the remaining data
	if(size!=0)
	{
		//printf("loop\n");
		goto sd_streamcache_write_loop;
	}

//	fputc('\n',file_pri);
	// Success
	//return error|0x10;
	return error;
}

/******************************************************************************
	sd_streamcache_close
*******************************************************************************	
	Terminates a streaming write with caching.

	Parameters:
		currentsect		-	Optionally a pointer to a variable holding the address (in sectors) of the last block to hold data of the streaming write.
							If 0, the address will not be provided
							
	Return value:
		?
******************************************************************************/
unsigned char sd_streamcache_close(unsigned long *currentsect)
{
	unsigned char response;

	#ifdef MMCDBG
		printf_P(PSTR("sd_stream_close. strmopen: %d strmstrt: %d numwritten: %ld addr: %lX\r"),_sd_write_stream_open,_sd_write_stream_started,_sd_write_stream_numwritten,_sd_write_stream_address);
	#endif
	
	//printf("close2, blockstarted: %d\n",_sd_write_stream_block_started);
	
	// Force emptying buffer
	sd_streamcache_write(0,0,0);
		
	//printf("close2, after empty: %d\n",_sd_write_stream_block_started);
	
	// Get the current write address, which is the last sector that will hold data from the streaming write.
	// Two cases occur: if the block is not started _sd_write_stream_address points to one sector past the last sector used;
	// if the block is started, _sd_write_stream_address points to the last sector used after padding.
	if(currentsect)
	{
		if(_sd_write_stream_block_started)
			*currentsect = _sd_write_stream_address;
		else
			*currentsect = _sd_write_stream_address-1;
	}
	
	
	if(_sd_write_stream_block_started)
	{
		
		unsigned short topad = 512-_sd_write_stream_numwritten;
		//printf("_sd_write_stream_numwritten: %u topad: %u\n",_sd_write_stream_numwritten,topad);

		// Write
		memset(_sdbuffer,0x55,topad);
		sd_stream_write(_sdbuffer,topad,0);			// Why use sd_stream_write instead of sd_streamcache_write?

		// Flag as closed, even if the stop operation may fail
		_sd_write_stream_block_started=0;

		// Stop the block
		response = _sd_block_stop();
	}

	
	
	// -----------
	// With sd_write_stream_write_block2 must wait for card to become ready from previous block write	
	/*if(_sd_write_stream_mustwait)
	{
		response = _sd_block_stop_dowait();
		_sd_write_stream_mustwait=0;
		// Possibility of error. We cannot return as we have to attempt writing the new block
		if(response!=0)
		{
			_sd_multiblock_close();
			_sd_write_stream_open=0;							// Multiblock write not open
			return response;
		}
	}*/
	// -----------
	
	
	

	// Terminates the multiblock write
	response = _sd_multiblock_close();
	
	// Flag as closed
	_sd_write_stream_open=0;

	return response;
}

/************************************************************************************************************************************************************
*************************************************************************************************************************************************************
PRINT FUNCTIONS   PRINT FUNCTIONS   PRINT FUNCTIONS   PRINT FUNCTIONS   PRINT FUNCTIONS   PRINT FUNCTIONS   PRINT FUNCTIONS   PRINT FUNCTIONS   PRINT FUNCTIONS  
*************************************************************************************************************************************************************
************************************************************************************************************************************************************/



/******************************************************************************
	function: sd_print_csd
*******************************************************************************	
	Prints the CSD fields to a stream.
	
	Parameters:
		f			-	Stream on which to print
		csd			-	Pointer to CSD		
******************************************************************************/
void sd_print_csd(FILE *f,CSD *csd)
{
	fprintf_P(f,PSTR("CSD:\n"));
	fprintf_P(f,PSTR("\tCSD: %X\n"),csd->CSD);
	fprintf_P(f,PSTR("\tTAAC: %X\n"),csd->TAAC);
	fprintf_P(f,PSTR("\tNSAC: %X\n"),csd->NSAC);
	fprintf_P(f,PSTR("\tTRAN_SPEED: %X\n"),csd->TRAN_SPEED);
	fprintf_P(f,PSTR("\tCCC: %X\n"),csd->CCC);
	fprintf_P(f,PSTR("\tREAD_BL_LEN: %X\n"),csd->READ_BL_LEN);
	fprintf_P(f,PSTR("\tREAD_BL_PARTIAL: %X\n"),csd->READ_BL_PARTIAL);
	fprintf_P(f,PSTR("\tWRITE_BLK_MISALIGN: %X\n"),csd->WRITE_BLK_MISALIGN);
	fprintf_P(f,PSTR("\tREAD_BLK_MISALIGN: %X\n"),csd->READ_BLK_MISALIGN);
	fprintf_P(f,PSTR("\tDSR_IMP: %X\n"),csd->DSR_IMP);
	if(csd->CSD==0)
	{
		fprintf_P(f,PSTR("\tC_SIZE: %04X\n"),csd->C_SIZE);
		fprintf_P(f,PSTR("\tVDD_R_CURR_MIN: %X\n"),csd->VDD_R_CURR_MIN);
		fprintf_P(f,PSTR("\tVDD_R_CURR_MAX: %X\n"),csd->VDD_R_CURR_MAX);
		fprintf_P(f,PSTR("\tVDD_W_CURR_MIN: %X\n"),csd->VDD_W_CURR_MIN);
		fprintf_P(f,PSTR("\tVDD_W_CURR_MAX: %X\n"),csd->VDD_W_CURR_MAX);
		fprintf_P(f,PSTR("\tC_SIZE_MULT: %X\n"),csd->C_SIZE_MULT);
	}
	else
	{
		fprintf_P(f,PSTR("\tC_SIZE: %08lX (%ld KB)\n"),csd->C_SIZE,(csd->C_SIZE+1)*512);
	}
	fprintf_P(f,PSTR("\tERASE_BLK_EN: %X\n"),csd->ERASE_BLK_EN);
	fprintf_P(f,PSTR("\tSECTOR_SIZE: %X\n"),csd->SECTOR_SIZE);	
	fprintf_P(f,PSTR("\tWP_GRP_SIZE	: %X\n"),csd->WP_GRP_SIZE);
	fprintf_P(f,PSTR("\tWP_GRP_ENABLE: %X\n"),csd->WP_GRP_ENABLE);	
	fprintf_P(f,PSTR("\tR2W_FACTOR: %X\n"),csd->R2W_FACTOR);
	fprintf_P(f,PSTR("\tWRITE_BL_LEN: %X\n"),csd->WRITE_BL_LEN);
	fprintf_P(f,PSTR("\tWRITE_BL_PARTIAL: %X\n"),csd->WRITE_BL_PARTIAL);
	fprintf_P(f,PSTR("\tFILE_FORMAT_GRP: %X\n"),csd->FILE_FORMAT_GRP);
	fprintf_P(f,PSTR("\tCOPY: %X\n"),csd->COPY);
	fprintf_P(f,PSTR("\tPERM_WRITE_PROTECT: %X\n"),csd->PERM_WRITE_PROTECT);
	fprintf_P(f,PSTR("\tTMP_WRITE_PROTECT: %X\n"),csd->TMP_WRITE_PROTECT);
	fprintf_P(f,PSTR("\tFILE_FORMAT: %X\n"),csd->FILE_FORMAT);
	fprintf_P(f,PSTR("\tCRC: %X\n"),csd->CRC);	
}
/******************************************************************************
	function: sd_print_cid
*******************************************************************************	
	Prints the CID fields to a stream.
	
	Parameters:
		f			-	Stream on which to print
		cid			-	Pointer to CID
******************************************************************************/
void sd_print_cid(FILE *f,CID *cid)
{
	fprintf_P(f,PSTR("CID:\n"));
	fprintf_P(f,PSTR("\tMID: %02X\n"),cid->MID);
	fprintf_P(f,PSTR("\tOID: %04X\n"),cid->OID);
	fprintf_P(f,PSTR("\tProduct name: %s\n"),cid->PNM);
	fprintf_P(f,PSTR("\tProduct revision: %02X\n"),cid->PRV);
	fprintf_P(f,PSTR("\tOID: %08lX\n"),cid->PSN);
	//fprintf_P(f,PSTR("\tMDT: %d/%d\n"),(cid->MDT>>4),1997+(cid->MDT&0xF));
	fprintf_P(f,PSTR("\tMDT: %d/%d\n"),cid->MDT&0b1111,2000+(cid->MDT>>4));
}
/******************************************************************************
	function: sd_print_ocr
*******************************************************************************	
	Prints the OCR fields to a stream.
	
	Parameters:
		f			-	Stream on which to print
		cid			-	Pointer to OCR
******************************************************************************/
void sd_print_ocr(FILE *f,OCR *ocr)
{	
	fprintf_P(f,PSTR("OCR:\n"));
	fprintf_P(f,PSTR("\tstatus: %X\n"),ocr->busy);
	fprintf_P(f,PSTR("\tCard Capacity Status: %X\n"),ocr->CCS);
	fprintf_P(f,PSTR("\tUHS-II Card Status: %X\n"),ocr->UHSII);
	fprintf_P(f,PSTR("\tSwitching to 1.8V accepted: %X\n"),ocr->S18A);
	fprintf_P(f,PSTR("\t3.5-3.6V: %X\n"),ocr->v3536);
	fprintf_P(f,PSTR("\t3.4-3.5V: %X\n"),ocr->v3435);
	fprintf_P(f,PSTR("\t3.3-3.4V: %X\n"),ocr->v3334);
	fprintf_P(f,PSTR("\t3.2-3.3V: %X\n"),ocr->v3233);
	fprintf_P(f,PSTR("\t3.1-3.2V: %X\n"),ocr->v3132);
	fprintf_P(f,PSTR("\t3.0-3.1V: %X\n"),ocr->v3031);
	fprintf_P(f,PSTR("\t2.9-3.0V: %X\n"),ocr->v2930);
	fprintf_P(f,PSTR("\t2.8-2.9V: %X\n"),ocr->v2829);
	fprintf_P(f,PSTR("\t2.7-2.98: %X\n"),ocr->v2728);
}


