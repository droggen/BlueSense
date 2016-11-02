#include "cpu.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <stdio.h>
#include <string.h>

#include "spi.h"


/*
	File: spi
	
	SPI functions.
	
	This library interfaces with the AVR's SPI interface.
	
	In order to select the spi peripheral the macro SPI_SELECT and SPI_DESELECT must be defined in spi.h. 
	The functions postfixed by _noselect will not select/deselect the peripheral, while the others will do.
	
	All the transfer functions wait for the transfer to be completed before returning. 
	
	*Dependencies*
	
	* None
	
	
	*Key functions*
	
	The key functions are:
	
	* spi_init:						Initialise SPI as Master in mode 0.
	* spi_rw: 						Exchange one byte with an SPI slave.
	* spi_rw_noselect: 				Exchange one byte with an SPI slave.
	* spi_rwn:						Exchanges n bytes with the SPI slave.
	* spi_rwn_noselect: 			Exchanges n bytes with the SPI slave.
	* spi_wn_noselect:				Writes n bytes to an SPI slave without storing the value returned by the slave.
	
	
	
	*Usage in interrupts*
	
	These functions can be used in interrupts. 
	However, as the transfer functions are blocking until the transfer is completed, they may be inadequate 
	if time-sensitive interrupts must be serviced.	
	
	*Possible improvements*
	Communication with some peripherals is sometimes one way only from the master to the slave.
	In this case, waiting for completion of the transfer at the end of a transfer function may be 
	sub-optimal as no data needs to be read back. In this case, waiting for completion of transfer
	at the beginning of the transfer function followed by initiation of a new transfer and an 
	immediate return may be more efficient, as the transfer would occur while returning to the caller.
	
	
*/


//volatile unsigned char _spi_ongoing=0;


/******************************************************************************
	function: spi_init
*******************************************************************************	
	Initialise SPI as Master in mode 0.
	
	Remember to set MOSI and SCK output, MISO as input.
	
	Parameters:
		spidiv	-	SPI divider: use one of SPI_DIV_2, SPI_DIV_4, ... SPI_DIV_128
	

*******************************************************************************/
void spi_init(unsigned char spidiv)
{
	spidiv=spidiv&0b111;
	SPCR = 0b01010000|(spidiv&0b11);					// SPE, MSTR, SPR
	SPSR = (spidiv>>2)&1;								// SPI2X
	
	//_spi_ongoing=0;
}

/******************************************************************************
	function: spi_rw
*******************************************************************************	
	Exchange one byte with an SPI slave: sends one byte to the slave and 
	receive one byte from it.
	
	This function selects the slave prior to the exchange, and deselects it 
	afterwards.
		
	Parameters:
		data	-	byte to send
		
	Returns:
		byte	-	byte read from the slave		
******************************************************************************/
unsigned char spi_rw(unsigned char data)
{
	//while(_spi_ongoing);	// Wait for end of interrupt transfer if any

	SPI_SELECT;
	
	
	SPDR = data;
	while(!(SPSR & (1<<SPIF))); 				//the SPIF Bit ist set when transfer complete...

	SPI_DESELECT;

	return SPDR;
}


/******************************************************************************
	function: spi_rw_noselect
*******************************************************************************	
	Exchange one byte with an SPI slave: sends one byte to the slave and 
	receive one byte from it.
	
	This function does not select/deselect the slave; this must be done by user
	code.
		
	Parameters:
		data	-	byte to send
		
	Returns:
		byte	-	byte read from the slave		
******************************************************************************/
unsigned char spi_rw_noselect(unsigned char data)
{
	//while(_spi_ongoing);	// Wait for end of interrupt transfer if any
	
	
	SPDR = data;
	while(!(SPSR & (1<<SPIF))); 				//the SPIF Bit ist set when transfer complete...

	
	return SPDR;
}

/******************************************************************************
	function: spi_rwn
*******************************************************************************	
	Exchanges n bytes with the SPI slave. Sends a n-byte buffer to the slave and 
	receive n bytes from it.
	
	This function selects the slave prior to the exchange, and deselects it 
	afterwards.
		
	Parameters:
		ptr		-	buffer comprising the data to send to the slave and 
					where the data read from it will be stored
		n		-	number of bytes to exchange with the slave
		
	Returns:
		-
******************************************************************************/
void spi_rwn(unsigned char *ptr,unsigned char n)
{
	//while(_spi_ongoing);	// Wait for end of interrupt transfer if any

	SPI_SELECT;
	
	while(n!=0)
	{
		// Put data into buffer, sends the data
		SPDR = *ptr;
		
		while(!(SPSR & (1<<SPIF))); 				//the SPIF Bit ist set when transfer complete...
		
		*ptr=SPDR;
		ptr++;
		n--;
	}
	
	SPI_DESELECT;
}

/******************************************************************************
	function: spi_rwn_noselect
*******************************************************************************	
	Exchanges n bytes with the SPI slave. Sends a n-byte buffer to the slave and 
	receive n bytes from it.
	
	This function does not select/deselect the slave; this must be done by user
	code.
		
	Parameters:
		ptr		-	buffer comprising the data to send to the slave and 
					where the data read from it will be stored
		n		-	number of bytes to exchange with the slave
		
	Returns:
		-
******************************************************************************/
void spi_rwn_noselect(unsigned char *ptr,unsigned short n)
{
	//while(_spi_ongoing);	// Wait for end of interrupt transfer if any

		
	while(n!=0)
	{
		// Put data into buffer, sends the data
		SPDR = *ptr;
		
		while(!(SPSR & (1<<SPIF))); 				//the SPIF Bit ist set when transfer complete...
		
		*ptr=SPDR;
		ptr++;
		n--;
	}
	
}
/******************************************************************************
	function: spi_wn_noselect
*******************************************************************************	
	Writes n bytes to an SPI slave without storing the value returned by the slave.
	
	This function is faster than spi_rwn and spi_rwn_noselect and can be 
	used if the goal is to send data to a slave without expecting data in return.
	
	This function does not select/deselect the slave; this must be done by user
	code.
		
	Parameters:
		ptr		-	buffer comprising the data to send to the slave and 
					where the data read from it will be stored
		n		-	number of bytes to exchange with the slave
		
	Returns:
		-
******************************************************************************/
void spi_wn_noselect(unsigned char *ptr,unsigned short n)
{
	//while(_spi_ongoing);	// Wait for end of interrupt transfer if any
		
	while(n!=0)
	{
		// Put data into buffer, sends the data
		SPDR = *ptr;
		
		while(!(SPSR & (1<<SPIF))); 				//the SPIF Bit ist set when transfer complete...
		
		ptr++;
		n--;
	}
}

