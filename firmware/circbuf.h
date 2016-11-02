#ifndef __CIRCBUF_H
#define __CIRCBUF_H

/******************************************************************************
*******************************************************************************
BUFFER MANAGEMENT   BUFFER MANAGEMENT   BUFFER MANAGEMENT   BUFFER MANAGEMENT  
*******************************************************************************
******************************************************************************/

typedef struct 
{
	unsigned char volatile *buffer;
	unsigned short volatile wrptr,rdptr;
	unsigned short size,mask; 
} BUFFEREDIO;

void buffer_put(volatile BUFFEREDIO *io, unsigned char c);
unsigned char buffer_get(volatile BUFFEREDIO *io);
unsigned char buffer_unget(volatile BUFFEREDIO *io,unsigned char c);
unsigned char buffer_isempty(volatile BUFFEREDIO *io);
void buffer_clear(volatile BUFFEREDIO *io);
unsigned char buffer_isfull(volatile BUFFEREDIO *io);
unsigned short buffer_level(volatile BUFFEREDIO *io);
unsigned short buffer_freespace(volatile BUFFEREDIO *io);

#endif