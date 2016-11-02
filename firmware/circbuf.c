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

void buffer_put(volatile BUFFEREDIO *io, unsigned char c)
{
	//ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
	io->buffer[io->wrptr]=c;
	io->wrptr=(io->wrptr+1)&(io->mask);
	}
}
unsigned char buffer_get(volatile BUFFEREDIO *io)
{
	unsigned char c;
	//ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
	c = io->buffer[io->rdptr];
	io->rdptr=(io->rdptr+1)&(io->mask);	
	}
	return c;
}
/*
Ungets a character.
In order to keep correct byte ordering, byte is unget at 'rdptr-1'.
(It is not possible to use 'buffer_put' for this purpose, as byte ordering would end up mixed).
*/
unsigned char buffer_unget(volatile BUFFEREDIO *io,unsigned char c)
{
	io->rdptr=(io->rdptr-1)&(io->mask);
	io->buffer[io->rdptr]=c;	
	return c;
}
unsigned char buffer_isempty(volatile BUFFEREDIO *io)
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
void buffer_clear(volatile BUFFEREDIO *io)
{
	io->wrptr=io->rdptr=0;
}
// We loose 1 character in the buffer because rd=wr means empty buffer, and 
// wr+1=rd means buffer full (whereas it would actually mean that one more byte can be stored).
unsigned char buffer_isfull(volatile BUFFEREDIO *io)
{
	unsigned char v;
	//ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
	if( ((io->wrptr+1)&io->mask) == io->rdptr )
		v=1;
	else
		v=0;
	}
	return v;
}
// Indicates how many bytes are in the buffer
unsigned short buffer_level(volatile BUFFEREDIO *io)
{
	unsigned short l;
	//ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
	l = ((io->wrptr-io->rdptr)&io->mask);
	}
	return l;
}
// Indicates how many free space is in the buffer - we still loose one byte.
unsigned short buffer_freespace(volatile BUFFEREDIO *io)
{
	return io->size-buffer_level(io)-1;
}
