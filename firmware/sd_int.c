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

#include "sd.h"

/*
	File: sd_int
	
	SD card internal functions.
	
	This provides internal functions used to communicate with the SD card.
	
	This library requires cards compatible with the Physical Spec version 2.00. This rules out some standard capacity SD cards and MMC cards.
	
	As this library uses the Physical Spec Version 2.0 all card read/write operations must be done on entire sectors (512-bytes) and the address of read and write operations are given in sectors.
	

	*Internal low-level functions*
	
	* _sd_command_rn: 					DoSel, issue command, read n bytes answer, Desel
	* _sd_command_rn_ns:				NoSel, issue command, read n bytes answer
	
	* _sd_command_rn_retry:				Retries _sd_command_rn until desired answer or timeout and returns RN answer; computes crc
	* _sd_command_rn_retry_crc:			Retries _sd_command_rn until desired answer or timeout and returns RN answer; use provided crc
	
	* _sd_command_r1_retry:				Calls _sd_command_rn_retry with R1 answers
	* _sd_command_r1_retry_crc:			Calls _sd_command_rn_retry_crc with R1 answers
	
	* _sd_readbock_ns:					Loops until a start block tocken is encountered then reads n block bytes and checksum.
	* _sd_command_r1_readblock:			Sends a command giving an R1 answer followed by a block answer (CSD, CID, read data)
	


	
	*Internal block read/write*
	
	* _sd_block_open:					Selects the card and start a block write command at the specified address.
	* _sd_block_stop:					Call after 512 bytes are written to complete the block with the checksum and wait for readyness.
	* _sd_block_stop_nowait:			Completes a block write by sending the CRC and returns; does not wait for readyness.
	* _sd_block_stop_dowait:			Waits for readyness after a block write.
	* _sd_block_close:					Terminates the block write transaction and deselects the card.
	* _sd_writebuffer:					Writes size bytes from buffer to card.
	* _sd_writeconst:					Writes size times byte b to card.

	

	*Internal multiblock read/write*

	* _sd_multiblock_open:			Selects the card and starts a multiblock write by sending the MMC_WRITE_MULTIPLE_BLOCK command.
	* _sd_multiblock_close: 		Terminates the multiblock write by sending MMC_STOPBLOCK and deselecting the card.
	
	Note that there is no internal "_sd_multiblock write" in this library; use _sd_writebuffer internally.

	
	

	The following are the state variables:
		_sd_write_stream_open;									// Block open for writing (.ie. multiblock command send)
		_sd_write_stream_started								// Block is started (i.e. data token has been transmitted)
		_sd_write_stream_address;								// Address to write to


	
	
	*Dependencies*
	
	* spi
	* Card on SPI interface: this library assumes the card is interfaced on the SPI interface. The SPI interface must be 
	initialised before using this library.
	

	
	
	
	*Internal functions*
	
	* _sd_cmd9: 						Issue CMD9 to read CSD data.
	* _sd_cmd10: 						Issue CMD10 to read CID data.
	* _sd_cmd58							Issue CMD58 (read OCR) aka "Extension Register Read Command"
	
	
	
	
	
	*Usage in interrupts*
	Not suitable for use in interrupts.
	
	
	*Possible improvements*
	Reduce or remove delay prior to command
	
	
	
*/

/************************************************************************************************************************************************************
*************************************************************************************************************************************************************
LOW-LEVEL   LOW-LEVEL   LOW-LEVEL   LOW-LEVEL   LOW-LEVEL   LOW-LEVEL   LOW-LEVEL   LOW-LEVEL   LOW-LEVEL   LOW-LEVEL   LOW-LEVEL   LOW-LEVEL   LOW-LEVEL   
*************************************************************************************************************************************************************
************************************************************************************************************************************************************/



/******************************************************************************
	_sd_command_rn and _sd_command_rn_ns
*******************************************************************************	
	Issue a command and reads n bytes of answer.
	_sd_command_rn selects the card.
	_sd_command_rn_ns doesn't select the card.
	
	If n=0 it behaves as if n=1.
	
	The first byte of the card response is returned by the function and stored in
	response[0] if response is non null. Subsequent response bytes are stored at
	response[1], response[2], ...
	
		
	cmd:				command to send to card
	p1-p4: 			parameters
	crc: 				crc
	response:		If non null points to where the response must be stored
	n:					Number of response bytes
				
	Return value: r1
******************************************************************************/
unsigned char _sd_command_rn(unsigned char cmd,unsigned char p1,unsigned char p2,unsigned char p3,unsigned char p4,unsigned char crc,char *response, unsigned short n)
{
	unsigned char r;
	sd_select_n(0);
	r = _sd_command_rn_ns(cmd,p1,p2,p3,p4,crc,response,n);
	sd_select_n(1);
	return r;
}
unsigned char _sd_command_rn_ns(unsigned char cmd,unsigned char p1,unsigned char p2,unsigned char p3,unsigned char p4,unsigned char crc,char *response, unsigned short n)
{
	unsigned char r1;
	unsigned long int t1;

	// Send the command
	spi_rw_noselect(cmd|0x40);
	spi_rw_noselect(p1);
	spi_rw_noselect(p2);
	spi_rw_noselect(p3);
	spi_rw_noselect(p4);			
	spi_rw_noselect(crc);

	// Wait until an answer is received, or a timeout occurs
	#ifdef MMCDBG
		printf_P(PSTR("_sd_command_rn_ns\n"));
	#endif
	t1 = timer_ms_get();
	do
	{
		r1 = spi_rw_noselect(0xFF);
		#ifdef MMCDBG
			printf_P(PSTR("  cmdresp %X\n"),r1);
		#endif
	}
	//while(r1 == 0xFF && (timer_ms_get()-t1<MMC_TIMEOUT_ICOMMAND));
	while( (r1&0x80) && (timer_ms_get()-t1<MMC_TIMEOUT_ICOMMAND));

	if(response)
		response[0]=r1;
	
	if(r1&0x80)			// Timeout ocurred: return 
	{
		#ifdef MMCDBG
			printf_P(PSTR("_sd_command_rn_ns: timeout\n"));
		#endif	
		return r1;			// Returns nonzero indicating a failure.
	}

	// Read the remainder response
	
	for(unsigned int i=1;i<n;i++)
	{
		response[i] = spi_rw_noselect(0xFF);
	}

	// Trailing byte requirement
	// DAN 13.02.2016: not needed?
	//unsigned char r2 = spi_rw_noselect(0xFF);
	//printf_P(PSTR("   cmdresp trailing %02X\n"),r2);
	
	#ifdef MMCDBG
		printf_P(PSTR("_sd_command_response_ns: returning ok: %02X\n"),r1);
	#endif
	return r1; 		// Success
}

/******************************************************************************
	function: _sd_command_rn_retry
*******************************************************************************	
	Selects the SD card and sends a command, retrying if failure.
	Suitable for card answers different than R1.
	
	This function computs the CRC which is required to issue a command internally.
	
	Parameters:
		cmd			- 		Command to send to card
		p1-p4		-		Parameters p1 to p4
		crc			- 		CRC of the command
		response	-		Pointer to buffer receiving the card response
		n			-		Number of response bytes
		answermask	-		The card answer is masked with answermask before being compared with okanswer.
							When an exact answer is desired use answermask=0xff
		okanswer	-		Expected answer in case of success after masking with answermask.
				
	Returns:
		0			-		Success
		1			-		Error
******************************************************************************/
unsigned char _sd_command_rn_retry(unsigned char cmd,unsigned char p1,unsigned char p2,unsigned char p3,unsigned char p4,char *response,unsigned short n, unsigned char answermask,unsigned char okanswer)
{
	// Compute the crc
	unsigned char crc = _sd_crc7command(cmd,p1,p2,p3,p4);
	return _sd_command_rn_retry_crc(cmd,p1,p2,p3,p4,crc,response,n,answermask,okanswer);
}
/******************************************************************************
	function: _sd_command_rn_retry_crc
*******************************************************************************	
	Selects the SD card and sends a command, retrying if failure.
	Suitable for card answers different than R1.
	
	Use this function if the crc of the command is known.
	
	Parameters:
		cmd			- 		Command to send to card
		p1-p4		-		Parameters p1 to p4
		crc			- 		CRC of the command
		response	-		Pointer to buffer receiving the card response
		n			-		Number of response bytes
		answermask	-		The card answer is masked with answermask before being compared with okanswer.
							When an exact answer is desired use answermask=0xff
		okanswer	-		Expected answer in case of success after masking with answermask.
				
	Returns:
		0			-		Success
		1			-		Error
******************************************************************************/
unsigned char _sd_command_rn_retry_crc(unsigned char cmd,unsigned char p1,unsigned char p2,unsigned char p3,unsigned char p4, unsigned char crc,char *response,unsigned short n, unsigned char answermask,unsigned char okanswer)
{
	unsigned char c,retry = 0;
	do
	{
		_delay_ms(SD_DELAYBETWEENCMD);
		c = _sd_command_rn(cmd,p1,p2,p3,p4,crc,response,n);
		#ifdef MMCDBG
		printf_P(PSTR("cmd %02X: %02X\n"),cmd,c);
		#endif
	} while( ((c&answermask)!=okanswer) && (retry++<MMC_RETRY) );
	if( ((c&answermask)!=okanswer) )
		return 1;
	return 0;
}

/******************************************************************************
	function: _sd_command_r1_retry
*******************************************************************************	
	Selects the SD card and sends a command, retrying if failure.
	Suitable for card answers R1.
	
	This function computs the CRC which is required to issue a command internally.
	
	
	Parameters:
		cmd			- 		Command to send to card
		p1-p4		-		Parameters p1 to p4
		answermask	-		The card answer is masked with answermask before being compared with okanswer.
							When an exact answer is desired use answermask=0xff
		okanswer	-		Expected answer in case of success after masking with answermask.
	
					
	Returns:
		0			-		Success
		1			-		Error
******************************************************************************/
unsigned char _sd_command_r1_retry(unsigned char cmd,unsigned char p1,unsigned char p2,unsigned char p3,unsigned char p4, unsigned char answermask,unsigned char okanswer,unsigned char *r1)
{
	// Compute the crc
	unsigned char crc = _sd_crc7command(cmd,p1,p2,p3,p4);
	return _sd_command_r1_retry_crc(cmd,p1,p2,p3,p4,crc,answermask,okanswer,r1);
}
/******************************************************************************
	function: _sd_command_r1_retry_crc
*******************************************************************************	
	Selects the SD card and sends a command, retrying if failure.
	Suitable for card answers R1.
	
	Use this function if the crc of the command is known.
	
	Parameters:
		cmd			- 		Command to send to card
		p1-p4		-		Parameters p1 to p4
		crc			- 		CRC of the command
		answermask	-		The card answer is masked with answermask before being compared with okanswer.
							When an exact answer is desired use answermask=0xff
		okanswer	-		Expected answer in case of success after masking with answermask.
	
	Returns:
		0			-		Success
		1			-		Error
******************************************************************************/
unsigned char _sd_command_r1_retry_crc(unsigned char cmd,unsigned char p1,unsigned char p2,unsigned char p3,unsigned char p4, unsigned char crc, unsigned char answermask,unsigned char okanswer,unsigned char *r1)
{
	unsigned char c,retry = 0;
	do
	{
		_delay_ms(SD_DELAYBETWEENCMD);
		c = _sd_command_rn(cmd,p1,p2,p3,p4,crc,0,0);
		#ifdef MMCDBG
		printf_P(PSTR("cmd %02X: %02X\n"),cmd,c);
		#endif
	} while( ((c&answermask)!=okanswer) && (retry++<MMC_RETRY) );
	if(r1)
		*r1 = c;
	if( ((c&answermask)!=okanswer) )
		return 1;
	return 0;
}

/******************************************************************************
	function: _sd_readbock_ns
*******************************************************************************	
	Loops reading until the start token is encountered, then read n bytes of data.
	
	n:		Number of data bytes to read (excluding the 2 bytes checksum, which are automatically read)
	
	Does not select the card.
				
	Return value:
		0:				Success
		1:				Error
******************************************************************************/
unsigned char _sd_readbock_ns(char *buffer,unsigned short n,unsigned short *checksum)
{
	unsigned char r1;
	unsigned short i;
	unsigned long int t1;
	

	t1 = timer_ms_get();
	do
	{
		r1 = spi_rw_noselect(0xFF);
		//printf_P(PSTR("MMC_CommanRead wait start: %X\n"),r1);
	}
	while(r1!= MMC_STARTBLOCK && (timer_ms_get()-t1<MMC_TIMEOUT_READWRITE));
	if(r1 != MMC_STARTBLOCK)
	{
		//printf_P(PSTR("Failre before start block\n"));
		//sd_select_n(1);			// Deselect the card
		return 1;					// Return, indicating a timeout waiting for MMC_STARTBLOCK
	}
	
	// read in data
	for(i=0; i<n;i++)
	{
		*buffer++ = spi_rw_noselect(0xFF);
	}
	
	
	// Read 16-bit CRC
	*checksum = spi_rw_noselect(0xFF);
	*checksum<<=8;
	*checksum |= spi_rw_noselect(0xFF);

	// Trailing stuff
	// Dan 13.02.2016: not needed?
	//spi_rw_noselect(0xFF);	
	
	return 0;	
}
/******************************************************************************
	_sd_command_r1_readblock
*******************************************************************************	
	Issues a command returning an R1 answer followed by a block answer (typically: CSD, CID, read data)
	
	Parameters:
		cmd			-	Command to issue
		p1-p4		-	Command parameters
		r1			-	R1 answer
		block		-	Buffer containing the block answer
		n			-	Number of block answer bytes (excluding checksum)
		checksum	-	Received checksum	
				
	Returns: 
		0			-	Success
		1			-	Error
******************************************************************************/
unsigned char _sd_command_r1_readblock(unsigned char cmd,unsigned char p1,unsigned char p2,unsigned char p3,unsigned char p4,char *r1,char *block,unsigned short n,unsigned short *checksum)
{
	unsigned char c;
	sd_select_n(0);
	// Issue command which gives R1, dummy crc
	c = _sd_command_rn_ns(cmd,p1,p2,p3,p4,0x55,r1,1);
	if(c!=0)
	{
		sd_select_n(1);
		return 1;
	}
	// Read block
	c = _sd_readbock_ns(block,n,checksum);
	sd_select_n(1);
	if(c!=0)
		return 1;
	
	// Optionally compute own checksum
	/*unsigned short x;
	x=0;
	for(unsigned short i=0;i<n;i++)	
		x=sd_crc16(x,block[i]);
	x = sd_crc16end(x);
	printf_P(PSTR("Checksum: %04X. Own checksum: %04X\n"),*checksum,x);
	if(*checksum!=x)
		return 1;*/
	
		
	
	return 0;
}


/******************************************************************************
	function: _sd_cmd9
*******************************************************************************	
	Issue CMD9 to read CSD data.
	This function also parses the CSD data to return the card capacity in sectors.
			
	Parameters:
		csd				-	Pointer to receive the CSD data
		capacity_sector	-	Pointer to receive the card capacity in sectors
		
	Returns:
		0				-	Success
		1				-	Error
******************************************************************************/
unsigned char _sd_cmd9(CSD *csd,unsigned long *capacity_sector)
{
	char response[32];
	unsigned char c;
	char r1;
	unsigned short checksum;
	unsigned char n=16;
	
	#ifdef MMCDBG
	printf_P(PSTR("MMC_SEND_CSD\n"));	
	#endif
	c = _sd_command_r1_readblock(MMC_SEND_CSD,0,0,0,0,&r1,response,n,&checksum);
	#ifdef MMCDBG
	printf_P(PSTR("MMC_SEND_CSD: %02X. r1: %02x. checksum: %04X\r\n"),c,r1,checksum);
	for(i=0;i<n;i++)
		printf_P(PSTR("%02d: %02X "),i,response[i]);
	printf_P(PSTR("\n"));
	#endif
	
	if(c!=0)
		return 1;
	
	if(csd)
	{
		csd->CSD							= _sd_bit2val(response,0,2);					// [127:126]
		csd->TAAC							= _sd_bit2val(response,8,8);					// [119:112]
		csd->NSAC							= _sd_bit2val(response,16,8);					// [111:104]
		csd->TRAN_SPEED						= _sd_bit2val(response,24,8);					// [103:96]
		csd->CCC							= _sd_bit2val(response,32,12);					// [95:84]
		csd->READ_BL_LEN					= _sd_bit2val(response,44,4);					// [83:80]
		csd->READ_BL_PARTIAL				= _sd_bit2val(response,48,1);					// [79:79]
		csd->WRITE_BLK_MISALIGN				= _sd_bit2val(response,49,1);					// [78:78]
		csd->READ_BLK_MISALIGN				= _sd_bit2val(response,50,1);					// [77:77]
		csd->DSR_IMP						= _sd_bit2val(response,51,1);					// [76:76]
		if(csd->CSD==0)
		{
			csd->C_SIZE						= _sd_bit2val(response,54,12);					// [73:62]
			csd->VDD_R_CURR_MIN				= _sd_bit2val(response,66,3);					// [61:59]
			csd->VDD_R_CURR_MAX				= _sd_bit2val(response,69,3);					// [58:56]
			csd->VDD_W_CURR_MIN				= _sd_bit2val(response,72,3);					// [55:53]
			csd->VDD_W_CURR_MAX				= _sd_bit2val(response,75,3);					// [52:50]
			csd->C_SIZE_MULT				= _sd_bit2val(response,78,3);					// [49:47]
		}
		if(csd->CSD==1)
		{
			csd->C_SIZE						= _sd_bit2val(response,58,22);					// [69:48]
			csd->VDD_R_CURR_MIN				= 0;
			csd->VDD_R_CURR_MAX				= 0;
			csd->VDD_W_CURR_MIN				= 0;
			csd->VDD_W_CURR_MAX				= 0;
			csd->C_SIZE_MULT				= 0;
		}		
		csd->ERASE_BLK_EN					= _sd_bit2val(response,81,1);					// [46:46]
		csd->SECTOR_SIZE					= _sd_bit2val(response,82,7);					// [45:39]
		csd->WP_GRP_SIZE					= _sd_bit2val(response,89,7);					// [38:32]
		csd->WP_GRP_ENABLE					= _sd_bit2val(response,96,1);					// [31:31]
		csd->R2W_FACTOR						= _sd_bit2val(response,99,3);					// [28:26]
		csd->WRITE_BL_LEN					= _sd_bit2val(response,102,4);					// [25:22]
		csd->WRITE_BL_PARTIAL				= _sd_bit2val(response,106,1);					// [21:21]
		csd->FILE_FORMAT_GRP				= _sd_bit2val(response,112,1);					// [15:15]
		csd->COPY							= _sd_bit2val(response,113,1);					// [14:14]
		csd->PERM_WRITE_PROTECT				= _sd_bit2val(response,114,1);					// [13:13]
		csd->TMP_WRITE_PROTECT				= _sd_bit2val(response,115,1);					// [12:12]
		csd->FILE_FORMAT					= _sd_bit2val(response,116,2);					// [11:10]
		csd->CRC							= _sd_bit2val(response,120,7);					// [7:1]
	}
	if(capacity_sector)
	{
		*capacity_sector = (csd->C_SIZE+1)*1024;
	}
	return 0;
}

/******************************************************************************
	function: _sd_cmd10
*******************************************************************************	
	Issue CMD10 to read CID data.
			
	Parameters:
		cid				-	Pointer to receive the CID data
		
	Returns:
		0				-	Success
		1				-	Error
******************************************************************************/
unsigned char _sd_cmd10(CID *cid)
{
	char response[32];
	unsigned char c;
	char r1;
	unsigned short checksum;
	
	
	//MMC_SEND_CID
	#ifdef MMCDBG
	printf_P(PSTR("MMC_SEND_CID\n"));
	#endif
	c = _sd_command_r1_readblock(MMC_SEND_CID,0,0,0,0,&r1,response,16,&checksum);
	#ifdef MMCDBG
	printf_P(PSTR("MMC_SEND_CID: %02X. r1: %02x. checksum: %04X\r\n"),c,r1,checksum);
	for(i=0;i<n;i++)
		printf_P(PSTR("%02d: %02X "),i,response[i]);
	printf_P(PSTR("\n"));
	#endif
	if(c!=0)
		return 1;

	if(cid)
	{
		cid->MID=_sd_bit2val(response,0,8);
		cid->OID=_sd_bit2val(response,8,16);
		for(unsigned char i=0;i<5;i++)
			cid->PNM[i]=response[3+i];
		cid->PNM[5]=0;
		cid->PRV=_sd_bit2val(response,64,8);
		cid->PSN=_sd_bit2val(response,72,32);
		cid->MDT=_sd_bit2val(response,108,12);
	}

	return 0;
}

/******************************************************************************
	_sd_cmd58
*******************************************************************************	
	Issue CMD58 (read OCR, get CCS) aka "Extension Register Read Command"
	
	p1-p4: 0
	Reponse type R3 aka 5 bytes response
	
	CCS bit is in OCR[30] but only after issuing CMD58 after CMD41.
		
	Parameters:
		ocr		-	Pointer to receive OCR
		
	Returns:
		0		-	Success
		1		-	Error
******************************************************************************/
unsigned char _sd_cmd58(OCR *ocr)
{
	unsigned char c;
	char response[10];
	#ifdef MMCDBG
	printf_P(PSTR("SD CMD58\n"));
	#endif
	c = _sd_command_rn_retry_crc(MMC_READ_OCR,0,0,0,0,0x55,response,5,0xfe,0x00);			// 5 bytes response, Mask FE means checking only idle bit, idle must be 0
	#ifdef MMCDBG
	printf_P(PSTR("SD CMD58: %02X\n"),c);
	for(i=0;i<5;i++)
		printf_P(PSTR("%02X "),response[i]);
	printf_P(PSTR("\n"));
	#endif
	if(c)
		return 1;
	ocr->busy=_sd_bit2val(response+1,0,1);
	ocr->CCS=_sd_bit2val(response+1,1,1);
	ocr->UHSII=_sd_bit2val(response+1,2,1);
	ocr->S18A=_sd_bit2val(response+1,7,1);
	ocr->v3536=_sd_bit2val(response+1,8,1);
	ocr->v3435=_sd_bit2val(response+1,9,1);
	ocr->v3334=_sd_bit2val(response+1,10,1);
	ocr->v3233=_sd_bit2val(response+1,11,1);
	ocr->v3132=_sd_bit2val(response+1,12,1);
	ocr->v3031=_sd_bit2val(response+1,13,1);
	ocr->v2930=_sd_bit2val(response+1,14,1);
	ocr->v2829=_sd_bit2val(response+1,15,1);
	ocr->v2728=_sd_bit2val(response+1,16,1);
	return 0;
}




/************************************************************************************************************************************************************
*************************************************************************************************************************************************************
SINGLE BLOCK   SINGLE BLOCK   SINGLE BLOCK   SINGLE BLOCK   SINGLE BLOCK   SINGLE BLOCK   SINGLE BLOCK   SINGLE BLOCK   SINGLE BLOCK   SINGLE BLOCK   SINGLE 
*************************************************************************************************************************************************************
************************************************************************************************************************************************************/



/******************************************************************************
	function: _sd_block_open
*******************************************************************************
	Selects the card and start a block write command at the specified address.
	This function is used for single block writes.
	
	Does:
	- Select card
	- Issue a MMC_WRITE_BLOCK command
	- Issue a MMC_STARTBLOCK token

	Returns:
		0				-	Start of write ok
		other			-	Error
******************************************************************************/
unsigned char _sd_block_open(unsigned long addr)
{
	char response;

	sd_select_n(0);				//	Select card
	response=_sd_command_rn_ns(MMC_WRITE_BLOCK,addr>>24,addr>>16,addr>>8,addr,0x55,&response,1);
	
	if (response!=0)				// Command failed
	{
		sd_select_n(1);			// Deselect card

		#ifdef MMCDBG
			printf_P(PSTR("_mmc_write_block_open fail %d\n"),response);
		#endif
		return response;				
	}
	#ifdef MMCDBG
		printf_P(PSTR("_mmc_write_block_open ok %d\n"),response);
	#endif

	spi_rw_noselect(MMC_STARTBLOCK);			// Send Data Token

	return 0;
}



/******************************************************************************
	function: _sd_block_close
*******************************************************************************
	Terminates the block write transaction and deselects the card.
	This function is used for single/multi block writes.

	It is the responsability of the application to call this function after exactly one block of data has been provided.
	
	Does:
	- Completes the block with CRC and wait for card ready
	- Deselects card

	Returns:
		0				-	End of write ok
		other			-	Error
******************************************************************************/
unsigned char _sd_block_close(void)
{
	unsigned char rv;

	#ifdef MMCDBG
		printf_P(PSTR("_sd_block_close\n"));
	#endif

	rv = _sd_block_stop();		// Stop the block: issue CRC, wait for card ready

	// Deselect the card
	sd_select_n(1);

	return rv;					
}


/******************************************************************************
	function: _sd_block_stop
*******************************************************************************
	Call after 512 bytes are written to complete the block with the checksum and wait for readyness.
	Reads back the error codes.
	This function is used for single/multi block writes.
	
	Internally it calls:
	- _sd_block_stop_nowait: Send CRC and check data was received
	- _sd_block_stop_dowait: Wait for card ready	
			
	Returns:
		0			- 	Ok
		other		-	Error
******************************************************************************/
unsigned char _sd_block_stop(void)
{
	unsigned char rv;
	// Send CRC and check data was received
	rv = _sd_block_stop_nowait();
	if(rv)
		return rv;
	// 
	return _sd_block_stop_dowait();
}
/******************************************************************************
	function: _sd_block_stop_nowait
*******************************************************************************
	Completes a block write by sending the CRC and returns; does not wait for readyness.
	This allows interleaving data processing while sdcard completes writes
			
	Returns:
		0			- 	Ok
		other		-	Error
******************************************************************************/
unsigned char _sd_block_stop_nowait(void)
{
	unsigned char dataresponse;
	unsigned char response1 __attribute__((unused));
	unsigned char response2 __attribute__((unused));
	unsigned char response3 __attribute__((unused));
	// Send CRC
	response1 = spi_rw_noselect(0xFF);
	response2 = spi_rw_noselect(0xFF);

	
	// Upon sending the last byte (2nd CRC byte), the card:
	// 	- Immediately reads as 'data response' and indicates if the data is accepted: xxx00101
	// 	- Then reads as 0, aka not-idle
	// 	- Once write is completed reads as 1, aka idle
	// 	- Then reads as FF, aka transaction completed
	
	dataresponse = spi_rw_noselect(0xFF);				// This should be the data response
	response3 = spi_rw_noselect(0xFF);					// This should be 'non idle', not needed as we loop until idle

	//#ifdef MMCDBG
		//printf_P(PSTR("_sd_block_stop: r1,r2,dr,d3: %X %X %X %X\n"),response1,response2,dataresponse,response3);
	//#endif

	// Data response must be XXX00101 for data accepted
	if((dataresponse&0x1F)!=0x05)						// Failure
	{
		return 1;										// Return error
	}
	
	return 0;
}
/******************************************************************************
	function: _sd_block_stop_dowait
*******************************************************************************
	Waits for readyness after a block write.
			
	Returns:
		0			- 	Ok
		other		-	Error
******************************************************************************/
unsigned char _sd_block_stop_dowait(void)
{
	unsigned response;
	unsigned long int t1;
	// Wait until ready for next block
	t1 = timer_ms_get();
	do
	{
		response = spi_rw_noselect(0xFF);
		//printf_P(PSTR(" %X\n"),response);
	}
	while(response!=0xFF && (timer_ms_get()-t1<MMC_TIMEOUT_READWRITE));

	#ifdef MMCDBG
		printf_P(PSTR("_sd_block_stop_dowait: success: %d\n"),response3==0xff?1:0);
	#endif

	if(response!=0xFF)									// Failure
	{
		return 2;											// Return error
	}
	return 0;
}


/******************************************************************************
	function: _sd_writebuffer
*******************************************************************************
	Writes size bytes from buffer to card.
	This function is used for single or multiblock block writes.

	It is the responsability of the caller to ensure that the number of bytes written 
	is equal to a complete block before calling _sd_block_close. 
	
	Parameters:
		buffer		- 	Buffer containing the data to write to the card
		size		-	Number of bytes to write to the card
******************************************************************************/
void _sd_writebuffer(char *buffer,unsigned short size)
{
	// Send data
	spi_wn_noselect(buffer,size);
}

/******************************************************************************
	_sd_writeconst
*******************************************************************************
	Writes size times byte b to card.
	This function is used for single or multiblock writes.

	It is the responsability of the caller to ensure that the number of bytes written 
	is equal to a complete block before calling _sd_block_close. 
	
	Parameters:
		b			- 	Value to write
		size		-	Number of times b is written to the card
******************************************************************************/
void _sd_writeconst(unsigned char b,unsigned short size)
{
	// Send data
	for(unsigned short k=0;k<size;k++)						
	{															
		spi_rw_noselect(b);
	}
}




/************************************************************************************************************************************************************
*************************************************************************************************************************************************************
MULTIBLOCK WRITES   MULTIBLOCK WRITES   MULTIBLOCK WRITES   MULTIBLOCK WRITES   MULTIBLOCK WRITES   MULTIBLOCK WRITES   MULTIBLOCK WRITES   MULTIBLOCK WRITES   
*************************************************************************************************************************************************************
************************************************************************************************************************************************************/


/******************************************************************************
	_sd_multiblock_open
*******************************************************************************
	Selects the card and starts a multiblock write by sending the MMC_WRITE_MULTIPLE_BLOCK command.
	Used by streaming write commands.
		
	Does:
	- Select the card
	- Issue an MMC_WRITE_MULTIPLE_BLOCK at the desired start address
	
	Parameters:
		addr		-	Write start address in sectors
			
	Returns:
		0			-	Ok
		other		-	Error
******************************************************************************/
unsigned char _sd_multiblock_open(unsigned long addr)
{
	unsigned char rv;
	char r1;

	#ifdef MMCDBG
		printf_P(PSTR("_sd_multiblock_open\r"));
	#endif	

	sd_select_n(0);				//	Select card
	rv=_sd_command_rn_ns(MMC_WRITE_MULTIPLE_BLOCK,addr>>24,addr>>16,addr>>8,addr,0x95,&r1,1);
	if (rv!=0)				// Command failed
	{
		sd_select_n(1);			// Deselect card
		return rv;				
	}
	return 0;
}
/******************************************************************************
	_sd_multiblock_close
*******************************************************************************
	Terminates the multiblock write by sending MMC_STOPBLOCK and deselecting the card.
	Used by streaming write commands.	
	
	Must only be called after a complete block transfer.

	Does:
	- Send MMC_STOPMULTIBLOCK
	- Wait for a response
	- Deselect the card

	Returns:
		0			-	Ok
		other		-	Error
******************************************************************************/
unsigned char _sd_multiblock_close(void)
{
	unsigned char response;
	//unsigned long int t1;

	#ifdef MMCDBG
		printf_P(PSTR("_sd_multiblock_close\r"));
	#endif
	// Send Stop Tran token
	response = spi_rw_noselect(MMC_STOPMULTIBLOCK);
	
	//printf_P(PSTR("response to stop multiblock %X\r"),response);
	/*t1 = timer_ms_get();
	do
	{
		response = spi_rw_noselect(0xFF);
		//printf_P(PSTR(" %X\r"),response);
	}
	while(response!=0xFF && (timer_ms_get()-t1<MMC_TIMEOUT_READWRITE));*/
	response=_sd_block_stop_dowait();


	//testfewmore();
	
	sd_select_n(1);										// Deselect card
	//printf_P(PSTR("writestream close dataresponse: %X %X %X %X. time. %lu\n"),response1,response2,dataresponse,response3,t2-t1);

	return response;
}


