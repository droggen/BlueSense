
/*
	file: circbuf
	
	Implements a circular buffer, typically used for buffering in communication pipes.
	
	The key data structure is CIRCULARBUFFER which holds a pointer to a buffer, read and write pointers, and additional management information.
	
	
	*Usage in interrupts*
	
	
*/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>
#include <util/atomic.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "circbuf.h"

/******************************************************************************
*******************************************************************************
*******************************************************************************
BUFFER MANAGEMENT   BUFFER MANAGEMENT   BUFFER MANAGEMENT   BUFFER MANAGEMENT  
*******************************************************************************
*******************************************************************************
******************************************************************************/

/******************************************************************************
   function: buffer_put
*******************************************************************************
	Adds a character to the circular buffer. 
	
	This must only be called if the buffer is not full, as no check is done on the buffer capacity.
	
	Parameters:
		io	-		CIRCULARBUFFER buffer
		c	-		Character to add to the buffer
******************************************************************************/
void buffer_put(volatile CIRCULARBUFFER *io, unsigned char c)
{
	//ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
	io->buffer[io->wrptr]=c;
	io->wrptr=(io->wrptr+1)&(io->mask);
	}
}
/******************************************************************************
   function: buffer_get
*******************************************************************************
	Returns a character from the circular buffer. 
	
	This must only be called if the buffer is not empty, as no check is done on the buffer capacity.
	
	Returns:
		Character from the buffer
******************************************************************************/
unsigned char buffer_get(volatile CIRCULARBUFFER *io)
{
	unsigned char c;
	//ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
	c = io->buffer[io->rdptr];
	io->rdptr=(io->rdptr+1)&(io->mask);	
	}
	return c;
}

/******************************************************************************
   function: buffer_unget
*******************************************************************************
	Ungets a character into the circular buffer.
	
	This function keeps byte ordering by ungetting the byte at 'rdptr-1'.
	
	This must only be called if the buffer is not full, as no check is done on the buffer capacity.
	
	Parameters:
		io	-		CIRCULARBUFFER buffer
		c	-		Character to unget to the buffer
		
	Returns:
		c
******************************************************************************/
unsigned char buffer_unget(volatile CIRCULARBUFFER *io,unsigned char c)
{
	io->rdptr=(io->rdptr-1)&(io->mask);
	io->buffer[io->rdptr]=c;	
	return c;
}


/******************************************************************************
   function: buffer_isempty
*******************************************************************************
	Checks if the circular buffer is empty.
	
	Parameters:
		io	-		CIRCULARBUFFER buffer
		
	Returns:
		0	-		Not empty
		1	-		Empty
******************************************************************************/
unsigned char buffer_isempty(volatile CIRCULARBUFFER *io)
{
	unsigned char v;
	//ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
	if(io->rdptr==io->wrptr)
		v=1;
	else
		v=0;
	}
	return v;
}

/******************************************************************************
   function: buffer_isempty
*******************************************************************************
	Clears the content of the buffer
	
	Parameters:
		io	-		CIRCULARBUFFER buffer
******************************************************************************/
void buffer_clear(volatile CIRCULARBUFFER *io)
{
	io->wrptr=io->rdptr=0;
}

/******************************************************************************
   function: buffer_isfull
*******************************************************************************
	Checks if the circular buffer is full.
	
	Parameters:
		io	-		CIRCULARBUFFER buffer
		
	Returns:
		0	-		Not full
		1	-		Full
******************************************************************************/
unsigned char buffer_isfull(volatile CIRCULARBUFFER *io)
{
	unsigned char v;
	//ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
	// We loose 1 character in the buffer because rd=wr means empty buffer, and 
	// wr+1=rd means buffer full (whereas it would actually mean that one more byte can be stored).
	if( ((io->wrptr+1)&io->mask) == io->rdptr )
		v=1;
	else
		v=0;
	}
	return v;
}
/******************************************************************************
   function: buffer_level
*******************************************************************************
	Indicates how many bytes are in the buffer
	
	Parameters:
		io	-		CIRCULARBUFFER buffer
		
	Returns:
		Number of bytes in the buffer
******************************************************************************/
unsigned short buffer_level(volatile CIRCULARBUFFER *io)
{
	unsigned short l;
	//ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
	l = ((io->wrptr-io->rdptr)&io->mask);
	}
	return l;
}
/******************************************************************************
   function: buffer_freespace
*******************************************************************************
	Indicates how much free space is in the buffer.
	
	Parameters:
		io	-		CIRCULARBUFFER buffer
		
	Returns:
		Number of free bytes in the buffer
******************************************************************************/
unsigned short buffer_freespace(volatile CIRCULARBUFFER *io)
{
	return io->size-buffer_level(io)-1;
}
