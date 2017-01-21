/*
   Copyright (C) 2009-2016:
         Daniel Roggen, droggen@gmail.com

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

/*
	Generic serial functions - to phase out
*/
#ifndef __SERIAL_H
#define __SERIAL_H


#include <stdio.h>
#include "circbuf.h"
#include "serial0.h"
#include "serial1.h"
#include "dbg.h"


// Common for uart, dbg(i2c) and others


typedef struct 
{
	unsigned char blocking;
	CIRCULARBUFFER *txbuf;
	CIRCULARBUFFER *rxbuf;	
	unsigned char (*putbuf)(char *data,unsigned char n);
} SERIALPARAM;

/*
Issue: when wanting to do a buffer_free(stream) -> fcde
*/
FILE *serial_open(unsigned char periph,unsigned char interrupt);

/******************************************************************************
*******************************************************************************
UART MANAGEMENT   UART MANAGEMENT   UART MANAGEMENT   UART MANAGEMENT   
******************************************************************************* 
******************************************************************************/

//void uart_getbuffersize(unsigned short *rx0,unsigned short *tx0,unsigned short *rx1,unsigned short *tx1);

void serial_setblocking(FILE *file,unsigned char blocking);
unsigned char serial_isblocking(FILE *file);

unsigned char fputbuf(FILE *stream,char *data,unsigned char n);
unsigned short fgettxbuflevel(FILE *stream);
unsigned short fgetrxbuflevel(FILE *stream);
unsigned short fgettxbuffree(FILE *stream);
unsigned short fgetrxbuffree(FILE *stream);

//void flush(FILE *f);



#endif
