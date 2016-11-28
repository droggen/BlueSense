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
} CIRCULARBUFFER;

void buffer_put(volatile CIRCULARBUFFER *io, unsigned char c);
unsigned char buffer_get(volatile CIRCULARBUFFER *io);
unsigned char buffer_unget(volatile CIRCULARBUFFER *io,unsigned char c);
unsigned char buffer_isempty(volatile CIRCULARBUFFER *io);
void buffer_clear(volatile CIRCULARBUFFER *io);
unsigned char buffer_isfull(volatile CIRCULARBUFFER *io);
unsigned short buffer_level(volatile CIRCULARBUFFER *io);
unsigned short buffer_freespace(volatile CIRCULARBUFFER *io);

#endif