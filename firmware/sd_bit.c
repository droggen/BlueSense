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
	File: sd_bit
	
	Internal SD bit-manipulation functions: CRC, bit extraction, etc.
	
	*Internal bit manipulation functions*
	
	* _sd_bit2val:						Returns the value made up by n bits from a specified startbit position from a bitstring.			
	
	*Internal CRC functions*
	
	* _sd_crc7command:					Part of the CRC7 computation.
	* _sd_crc7byte:						Part of the CRC7 computation.
	* _sd_crc7end:						Part of the CRC7 computation.
	
	
	
	
	
*/

/******************************************************************************
	function: _sd_bit2val
*******************************************************************************	
	Returns the value made up by n bits from a specified startbit position 
	from a bitstring.
	
		
	Parameters:
		data 		-	Pointer to a bitstring
		startbit	-	Starting bit. Bit 0 is the MSB of data[0], bit 7 is the LSB of data[0], bit 8 is the MSB of data[1], etc.
						The startbit becomes the MSB of the return value.
		n			-	Number of bits to read
		
	Returns:
		Value
******************************************************************************/
unsigned long _sd_bit2val(unsigned char *data,unsigned int startbit,unsigned int n)
{
	unsigned long v,bit;
	unsigned int byteptr;
	unsigned int bitptr;

	v=0;

	byteptr = startbit>>3;
	bitptr = 7-(startbit&0x07);

	for(unsigned int i=0;i<n;i++)
	{
		bit = (data[byteptr]&(1<<bitptr))?1:0;    // read the bit from the input stream
		v = (v<<1)+bit;
		bitptr = (bitptr-1)&0x07;
		if(bitptr==0x07)
			byteptr++;
	}

	return v;

}

/************************************************************************************************************************************************************
*************************************************************************************************************************************************************
CRC   CRC   CRC   CRC   CRC   CRC   CRC   CRC   CRC   CRC   CRC   CRC   CRC   CRC   CRC   CRC   CRC   CRC   CRC   CRC   CRC   CRC   CRC   CRC   CRC   CRC   
*************************************************************************************************************************************************************
************************************************************************************************************************************************************/


/******************************************************************************
	function: _sd_crc7command
*******************************************************************************	
	Part of the CRC7 computation.
	Commands (at least cmd0 and cmd8) must be terminated by a crc.
	CRC is computed on the command (including bit 6 set) and 4 bytes parameters and 
	additional 7 zero bits (augmentation).
	
	The functions _sd_crc7command, _sd_crc7byte and _sd_crc7end are used to compute this checksum.
	
	_sd_crc7command can directly be called with the same parameters as _sd_command???
	
	
	Parameters:
		cmd			-	command to send to card
		p1-p4 		-	parameters
******************************************************************************/
unsigned char _sd_crc7command(unsigned char cmd,unsigned char p1,unsigned char p2,unsigned char p3,unsigned char p4)
{
	unsigned char crc=0;
	crc = _sd_crc7byte(crc,0x40|cmd);
	crc = _sd_crc7byte(crc,p1);
	crc = _sd_crc7byte(crc,p2);
	crc = _sd_crc7byte(crc,p3);
	crc = _sd_crc7byte(crc,p4);
	crc = _sd_crc7end(crc);
	return crc;
}
/******************************************************************************
	function: _sd_crc7byte
*******************************************************************************	
	Part of the CRC7 computation.
	Commands (at least cmd0 and cmd8) must be terminated by a crc.
	CRC is computed on the command (including bit 6 set) and 4 bytes parameters and 
	additional 7 zero bits (augmentation).
	
	The functions _sd_crc7command, _sd_crc7byte and _sd_crc7end are used to compute this checksum.
	
	Call _sd_crc7byte to update the crc7 with each byte of the payload.
	
	Parameters:
		crc7		-	Current CRC
		d	 		-	New byte of the payload
	
	Returns
		crc7		-	New CRC
******************************************************************************/
unsigned char _sd_crc7byte(unsigned char crc7,unsigned char d)
{
	// Using Maxim application note 3969
	for(unsigned char i=0;i<8;i++)
	{
		// Shift crc left and shift in msb of data
		crc7<<=1;
		if(d&0x80)
			crc7|=1;
		d<<=1;
		// If 8th bit (falling of crc7) is one, xor with polynomial
		if(crc7&0x80)
			crc7^=0b00001001;
	}
	// Mask 8th bit as crc7
	return crc7&0x7f;
}
/******************************************************************************
	function: _sd_crc7end
*******************************************************************************	
	Part of the CRC7 computation.
	Commands (at least cmd0 and cmd8) must be terminated by a crc.
	CRC is computed on the command (including bit 6 set) and 4 bytes parameters and 
	additional 7 zero bits (augmentation).
	
	The functions _sd_crc7command, _sd_crc7byte and _sd_crc7end are used to compute this checksum.
	
	Call sd_crc7end at the end to finalise the CRC with the augmentation.
	
	This function shifts the CRC left by 1 and adds 1, so that the return 
	value can be directly sent to the card.
	
	Parameters:
		cmd			-	command to send to card
		p1-p4 		-	parameters
******************************************************************************/
unsigned char _sd_crc7end(unsigned char crc7)
{
	// Augmentation: shift 7 zeros
	for(unsigned char i=0;i<7;i++)
	{
		// Shift crc left and shift in msb of data
		crc7<<=1;
		// If 8th bit (falling of crc7) is one, xor with polynomial
		if(crc7&0x80)
			crc7^=0b00001001;
	}
	// Shift left by 1 and add 1, to ensure the returned byte can be directly sent to the card (the 7 bits are aligned left with lsb=1)
	crc7<<=1;
	crc7|=1;
	return crc7;
}


// sd_crc16 could be used to verify the readblock checksum, currently unused
/*unsigned short sd_crc16(unsigned short crc16,unsigned char d)
{
	// Using Maxim application note 3969
	for(unsigned char i=0;i<8;i++)
	{
		// Check msb
		unsigned short msb = crc16&0x8000;
		// Shift crc left and shift in msb of data
		crc16<<=1;
		if(d&0x80)
			crc16|=1;
		d<<=1;
		// If msb bit (falling of crc16) is one, xor with polynomial
		if(msb)
			crc16^=0x1021;
	}
	return crc16;
}*/
// sd_crc16 could be used to verify the readblock checksum, currently unused
/*
unsigned short sd_crc16end(unsigned short crc16)
{
	// Using Maxim application note 3969
	for(unsigned char i=0;i<16;i++)
	{
		// Check msb
		unsigned short msb = crc16&0x8000;
		// Shift crc left and shift in msb of data
		crc16<<=1;
		// If msb bit (falling of crc16) is one, xor with polynomial
		if(msb)
			crc16^=0x1021;
	}
	return crc16;
}
*/
