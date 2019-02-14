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
#include "global.h"
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
/*
	TODO
		
		- When opening a log, issue a erase for the log file?
		
		
		
		
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
unsigned char sd_init(CID *cid,CSD *csd,SDSTAT *sdstat,unsigned long *capacity)
{
	unsigned char c;
	int i;
	char response[32];
	unsigned char r1;
	OCR ocr;
	unsigned long t1;

	

	// 0. Native initialization
	// This was used for SD/MMC. Although it does not appear necessary with SDHC, 
	// it speeds up some of the initialisation, likely bringing the card in a known
	// state.
	sd_select_n(0);
	_delay_ms(SD_DELAYBETWEENCMD);
	for(i=0; i < 128; i++) spi_rw_noselect(0xFF); // send at least 80 clock pulses
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
	sd_print_csd(file_pri,csd);
	// 7. Read CID
	_sd_cmd10(cid);
	
	// 8. Optional. Read SD Status (ACMD13)
	if(sdstat)
	{
		_sd_acmd13(sdstat);
		sd_print_sdstat(file_pri,sdstat);
	}
	
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
unsigned char sd_block_read(unsigned long addr,char *buffer)
{
	unsigned short checksum;
	unsigned char response = _sd_command_r1_datablock(MMC_READ_SINGLE_BLOCK,addr>>24,addr>>16,addr>>8,addr,0,buffer,512,&checksum);
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
unsigned char sd_block_write(unsigned long addr,char *buffer)
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
unsigned char _sd_write_stream_mustpreerase;			// Indicates whether the pre-erase command must be issued
unsigned long _sd_write_stream_preerase;				// Indicates how many sectors must be pre-erased


char _sdbuffer[SD_CACHE_SIZE];							// Cache memory for streaming writes with caching, also used for padding
unsigned short _sdbuffer_n;								// Amount of memory used in cache

/******************************************************************************
	function: sd_stream_open
*******************************************************************************
	Start a stream write at the specified address. This must be called prior to caching and non-caching streaming writes.

	This function initialises the state machine internally used for streaming writes.
	Internally, multiblock transfers to the SD card are used.
	
	Parameters:
		addr		-		Write start address in sectors
		preerase	-		Number of sectors to pre-erase, or 0 not to pre-erase.
			
******************************************************************************/
void sd_stream_open(unsigned long addr,unsigned long preerase)
{
	_sd_write_stream_open=0;						// Command write multiblock not sent yet
	_sd_write_stream_address=addr;					// Write address
	_sd_write_stream_mustwait=0;					// Must wait for answer from card
	_sd_write_stream_block_started = 0;				// Start data token not sent yet
	_sd_write_stream_numwritten = 0;				// No bytes written yet
	_sdbuffer_n=0;									// Number of data into buffer	
	_sd_write_stream_error=0;						// Number of errors
	if(preerase)
		_sd_write_stream_mustpreerase=1;			// The pre-erase command must be issued prior to multiblock write
	else
		_sd_write_stream_mustpreerase=0;			// The pre-erase command must not be issued.
	_sd_write_stream_preerase=preerase;
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
unsigned char sd_stream_write(char *buffer,unsigned short size,unsigned long *currentsect)
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
		// Issue the multiblock write, with an optional preerase if this is the first time the multiblock write is started.
		// As multiple starts could occur if an error occured during transfer, the preerase should be decremented by the number of written sector. Currently this logic is not implemented.
		if(_sd_write_stream_mustpreerase)
		{
			rv = _sd_multiblock_open(_sd_write_stream_address,_sd_write_stream_preerase);
			_sd_write_stream_mustpreerase=0;		// No more pre-erase now
		}
		else
		{
			rv = _sd_multiblock_open(_sd_write_stream_address,0);		// No preerase
		}
		
		
		if(rv)
		{
			#ifdef MMCDBG
				//printf_P(PSTR("sd_stream_write. size: %ld. strmopen: %d strmstrt: %d numwritten: %ld addr: %lX\r"),size,_sd_write_stream_open,_sd_write_stream_started,_sd_write_stream_numwritten,_sd_write_stream_address);
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
	
		// Stop block and wait for readiness
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
		//printf_P(PSTR("sd_stream_close. strmopen: %d strmstrt: %d numwritten: %ld addr: %lX\r"),_sd_write_stream_open,_sd_write_stream_started,_sd_write_stream_numwritten,_sd_write_stream_address);
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
		size			-	Number of bytes to write. Use size=0 to flush the buffer.
		currentsect		-	Optionally a pointer to a variable holding the address (in sectors) of the block currently being used to store the data. 
							If 0, the address will not be provided

	Returns:
		0				-	Success
		other			-	Failure
******************************************************************************/
unsigned char sd_streamcache_write(char *buffer,unsigned short size,unsigned long *currentsect)
{
	// Find how many bytes can effectively be written until the sector is full.
	unsigned long effw;
	unsigned char rv,error;

	// Error indicates the number of errors that occurred during this function. Normally it should remain 0.
	error=0;			
	#ifdef MMCDBG
		printf_P(PSTR("sd_streamcache_write: size: %u. incache: %u: strmopen: %d blkstr: %d wrinblk: %u addr: %lX\r"),size,_sdbuffer_n,_sd_write_stream_open,_sd_write_stream_block_started,_sd_write_stream_numwritten,_sd_write_stream_address);
	#endif

	// This loop will be iterated to write size bytes to the card every time a block is completed
sd_streamcache_write_loop:
	//fputc('W',file_pri);
	#ifdef MMCDBG
		printf_P(PSTR("sd_streamcache_write_loop: size: %u\n"),size);
	#endif
	//fprintf(file_pri,"write2. size: %u. _sdbuffer_n: %u\n",size,_sdbuffer_n);

	// Update current sector written to
	if(currentsect)
		*currentsect = _sd_write_stream_address;
		
	// Wait for card to become ready from previous block write (_sd_write_stream_mustwait)	
	while(_sd_write_stream_mustwait)					// Enter wait loop if neeeded regardless of the amount of data to write
	{
		//fputc('w',file_pri);
		// Get card status
		rv = spi_rw_noselect(0xFF);
		// To prevent timeout if streaming slowly. E.g. when sending one packet to the card every second only one FF is sent to the card per second, which timeouts MMC_TIMEOUT_READWRITE.
		// Sending more FF is harmless at the cost of slightly slower maximum write speed (less than 0.5%).
		rv = spi_rw_noselect(0xFF);
		rv = spi_rw_noselect(0xFF);
		rv = spi_rw_noselect(0xFF);
		
		#ifdef MMCDBG
			printf_P(PSTR("sd_streamcache: wait <- %02X\n"),rv);
		#endif
			
		// DAN TOCHECK: add more clocks here
		
		if(rv==0xff)
		{
			// Card responds that it is ready: indicate wait not needed and exit from loop
			_sd_write_stream_mustwait=0;
			break;
		}
		// Check the time elapsed from _sd_block_stop_nowait for a timeout.
		// ISSUE: with certain cards and slow streaming speed, the following timeouts. 
		// This happens because multiple FF's must be sent above, and only one FF is sent per "putbuf" call. Options: send multiple FF's above, or increase the timeout. 
		// Choice: issue multiple FF above
		if(timer_ms_get()-_sd_write_stream_t1>=MMC_TIMEOUT_READWRITE)
		{
			//printf("Dan timeout\n");
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
			#ifdef MMCDBG
				printf_P(PSTR("sd_streamcache_write: cached, return ok\n"));
			#endif
			return error;		// Error is 0 (success)
		}

	}
	// Here: card is ready (transaction/block may or may not be open/closed)
		
	// Nothing to write in the user-provided buffer nor in the cache buffer therefore returns successfully.
	if(size==0 && _sdbuffer_n==0)
	{
		#ifdef MMCDBG
			printf_P(PSTR("sd_streamcache_write: return ok\n"));
		#endif
		return error;			// Error is 0 (success)
	}
		
	
	//	Write command not yet send.
	if(!_sd_write_stream_open)
	{
		#ifdef MMCDBG
			printf_P(PSTR("open stream\n"));
		#endif
		//fputc('o',file_pri);
		// Issue the multiblock write, with an optional preerase if this is the first time the multiblock write is started.
		// As multiple starts could occur if an error occured during transfer, the preerase should be decremented by the number of written sector. Currently this logic is not implemented.
		if(_sd_write_stream_mustpreerase)
		{
			rv = _sd_multiblock_open(_sd_write_stream_address,_sd_write_stream_preerase);
			_sd_write_stream_mustpreerase=0;		// No more pre-erase now
		}
		else
		{
			rv = _sd_multiblock_open(_sd_write_stream_address,0);		// No preerase
		}
		
		if(rv)
		{
			#ifdef MMCDBG
				//printf_P(PSTR("sd_streamcache_write. size: %ld. strmopen: %d strmstrt: %d numwritten: %ld addr: %lX\r"),size,_sd_write_stream_open,_sd_write_stream_started,_sd_write_stream_numwritten,_sd_write_stream_address);
				printf_P(PSTR("sd_streamcache_write. _sd_write_stream_address failed\r"));
			#endif
			_sd_write_stream_error++;
			error++;
			//return error|0x20;					// Abort if the open failed
			return error;
		}	
		_sd_write_stream_open = 1;					// Block write is now open,...	
		_sd_write_stream_block_started=0;			// Block has not started
	}

	// Block start not yet sent
	if(!_sd_write_stream_block_started)
	{
		#ifdef MMCDBG
			printf_P(PSTR("start block\n"));
		#endif
		//fputc('b',file_pri);
		spi_rw_noselect(MMC_STARTMULTIBLOCK);	// Send Data Token
		_sd_write_stream_block_started=1;			// Block has started
		_sd_write_stream_numwritten=0;				// So far wrote 0
	}
		
	// Write the data until: either all data is written, or a block is completed (whichever comes first)
	// First write data in cache buffer, if any. 
	// It is always possible to write the entire cache buffer because the only time the buffer can 
	// fill itself and become full is during the waiting for card ready; and the first time the card is ready 
	// the buffer is emptied and stays empty.	
	// _sdbuffer_n must be <=512, which means maximum buffer size is 512 bytes.
	#ifdef MMCDBG
		printf_P(PSTR("sd_streamcache_write: write %u cache\n"),_sdbuffer_n);
	#endif
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
	#ifdef MMCDBG
		printf_P(PSTR("sd_streamcache_write: write %u data\n"),effw);
	#endif
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
		#ifdef MMCDBG
			printf_P(PSTR("close block\n"));
		#endif
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

	#ifdef MMCDBG
		printf_P(PSTR("sd_streamcache_write: return ok\n"));
	#endif
	// Success
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
		0				-	Ok
		Nonzero			-	Error
******************************************************************************/
unsigned char sd_streamcache_close(unsigned long *currentsect)
{
	unsigned char response;

	#ifdef MMCDBG
		printf_P(PSTR("sd_streamcache_close: strmopen: %d blkstr: %d wrinblk: %u addr: %lX\r"),_sd_write_stream_open,_sd_write_stream_block_started,_sd_write_stream_numwritten,_sd_write_stream_address);
	#endif
	
	// 1. Flush the cache. At this stage the block should be closed, but without waiting for ready
	response=sd_streamcache_write(0,0,0);
	if(response)
	{
		printf_P(PSTR("sd_streamcache_close: error flushing\n"));
		_sd_multiblock_close();
		_sd_write_stream_open=0;
		return 1;
	}
	
	// 2. Check if padding to terminate a block is needed
	if(_sd_write_stream_block_started)
	{
		unsigned short topad = 512-_sd_write_stream_numwritten;
		#ifdef MMCDBG
			printf("padding: %u topad: %u\n",_sd_write_stream_numwritten,topad);
		#endif

		// Write
		memset(_sdbuffer,0x55,topad);
		response = sd_streamcache_write(_sdbuffer,topad,0);
		if(response)
		{
			printf_P(PSTR("sd_streamcache_close: error padding\n"));
			_sd_multiblock_close();
			_sd_write_stream_open=0;
			return 1;
		}
		// 3. Flush cache again for the padding, if data was cached
		response=sd_streamcache_write(0,0,0);
		if(response)
		{
			printf_P(PSTR("sd_streamcache_close: error flushing\n"));
			_sd_multiblock_close();
			_sd_write_stream_open=0;
			return 1;
		}	
	}
	
	#ifdef MMCDBG
		printf_P(PSTR("sd_streamcache_close after pad+flush: strmopen: %d blkstr: %d wrinblk: %u addr: %lX\r"),_sd_write_stream_open,_sd_write_stream_block_started,_sd_write_stream_numwritten,_sd_write_stream_address);
	#endif

	// 4. This "Flush cache" is used to wait for ready after a block close; if there was no need to wait it returns transparently.
	response=sd_streamcache_write(0,0,0);
	if(response)
	{
		printf_P(PSTR("sd_streamcache_close: error flushing\n"));
		_sd_multiblock_close();
		_sd_write_stream_open=0;
		return 1;
	}	
	
	// Get the address of the last written block.
	if(currentsect)
	{
		*currentsect = _sd_write_stream_address-1;
	}
	
	// 5. Terminates the multiblock write
	response = _sd_multiblock_close();
	
	#ifdef MMCDBG
		printf_P(PSTR("_sd_multiblock_close: %02X\n"),response);
	#endif
	
	// 5. Flag as closed
	_sd_write_stream_open=0;
	
	if(response)
	{
		printf_P(PSTR("sd_streamcache_close: error multiblock close\n"));
		return 1;
	}

	return 0;
}

unsigned char sd_erase(unsigned long addr1,unsigned long addr2)
{
	
	if(_sd_cmd32(addr1))
		return 1;
	if(_sd_cmd33(addr2))
		return 1;
	if(_sd_cmd38())
		return 1;
	return 0;
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
/******************************************************************************
	function: sd_print_sdstat
*******************************************************************************	
	Prints the SDSTAT fields to a stream.
	
	Parameters:
		f			-	Stream on which to print
		sdstat		-	Pointer to SDSTAT
******************************************************************************/
void sd_print_sdstat(FILE *f,SDSTAT *sdstat)
{
	fprintf_P(f,PSTR("SDSTAT:\n"));
	fprintf_P(f,PSTR("\tDAT_BUS_WIDTH: %02X\n"),sdstat->DAT_BUS_WIDTH);
	fprintf_P(f,PSTR("\tSECURED_MODE: %02X\n"),sdstat->SECURED_MODE);
	fprintf_P(f,PSTR("\tSD_CARD_TYPE: %04X\n"),sdstat->SECURED_MODE);
	fprintf_P(f,PSTR("\tSIZE_OF_PROTECTED_AREA: %08lX\n"),sdstat->SIZE_OF_PROTECTED_AREA);
	fprintf_P(f,PSTR("\tSPEED_CLASS: %02X (class "),sdstat->SPEED_CLASS);
	if(sdstat->SPEED_CLASS<=3)
		fprintf_P(f,PSTR("%d)\n"),sdstat->SPEED_CLASS*2);
	else
	{
		if(sdstat->SPEED_CLASS==4)
			fprintf_P(f,PSTR("10)\n"));
		else
			fprintf_P(f,PSTR("reserved)\n"));
	}	
	fprintf_P(f,PSTR("\tPERFORMANCE_MOVE: %02X\n"),sdstat->PERFORMANCE_MOVE);
	fprintf_P(f,PSTR("\tAU_SIZE: %02X ("),sdstat->AU_SIZE);
	if(sdstat->AU_SIZE>=1 && sdstat->AU_SIZE<=0x0a)
	{
		fprintf_P(f,PSTR("%dKB)\n"),(1<<sdstat->AU_SIZE)*8);
	}
	else
	{
		switch(sdstat->AU_SIZE)
		{
			case 0:
				fprintf_P(f,PSTR("not defined)\n"));
				break;
			case 0x0b:
				fprintf_P(f,PSTR("12MB)\n"));
				break;
			case 0x0c:
				fprintf_P(f,PSTR("16MB)\n"));
				break;
			case 0x0d:
				fprintf_P(f,PSTR("24MB)\n"));
				break;
			case 0x0e:
				fprintf_P(f,PSTR("32MB)\n"));
				break;
			case 0x0f:
				fprintf_P(f,PSTR("64MB)\n"));
				break;
		}
	}
	fprintf_P(f,PSTR("\tERASE_SIZE: %04X\n"),sdstat->ERASE_SIZE);
	fprintf_P(f,PSTR("\tERASE_TIMEOUT: %02X\n"),sdstat->ERASE_TIMEOUT);
	fprintf_P(f,PSTR("\tERASE_OFFSET: %02X\n"),sdstat->ERASE_OFFSET);
	fprintf_P(f,PSTR("\tUHS_SPEED_GRADE: %02X\n"),sdstat->UHS_SPEED_GRADE);
	
	fprintf_P(f,PSTR("\tVIDEO_SPEED_CLASS: %02X\n"),sdstat->VIDEO_SPEED_CLASS);
	fprintf_P(f,PSTR("\tVSC_AU_SIZE: %02X\n"),sdstat->VSC_AU_SIZE);
	fprintf_P(f,PSTR("\tSUS_ADDR: %02X\n"),sdstat->SUS_ADDR);
	fprintf_P(f,PSTR("\tAPP_PERF_CLASS: %02X\n"),sdstat->APP_PERF_CLASS);
	fprintf_P(f,PSTR("\tPERFORMANCE_ENHANCE: %02X\n"),sdstat->PERFORMANCE_ENHANCE);
	fprintf_P(f,PSTR("\tDISCARD_SUPPORT: %02X\n"),sdstat->DISCARD_SUPPORT);
	fprintf_P(f,PSTR("\tFULE_SUPPORT: %02X\n"),sdstat->FULE_SUPPORT);
}




