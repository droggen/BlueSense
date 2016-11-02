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
	Serial functions
*/
#ifndef __SERIAL0_H
#define __SERIAL0_H


#ifdef ENABLE_SERIAL0

#include <stdio.h>
#include "serial.h"
#include "circbuf.h"


// Maximum size of the rx/tx buffers. Must be a power of 2.
#define SERIAL0_RX_BUFFERSIZE_MAX 256
#define SERIAL0_TX_BUFFERSIZE_MAX 512
// Serial buffers
extern volatile BUFFEREDIO SerialData0Rx;
extern volatile BUFFEREDIO SerialData0Tx;


// Callbacks for hooking into the interrupt routines.
// The callback is called first with the received character.
// The callback must return 1 to place the character in the receive queue, 0 if the character was processed otherwise.
extern unsigned char  (*uart0_rx_callback)(unsigned char);

// Initialisation
int uart0_init(unsigned int ubrr, unsigned char u2x);
void uart0_clearbuffers(void);

/*
	Direct serial access
*/
int uart0_fputchar(char ch,FILE* stream);
int uart0_fgetchar(FILE *stream);
int uart0_putchar(char ch);
int uart0_getchar(void);


/*
	Interrupt-driven serial access
*/
int uart0_fputchar_int(char c, FILE*stream);
unsigned char uart0_fputbuf_int(unsigned char *data,unsigned char n);
int uart0_fgetchar_nonblock_int(FILE *stream);
int uart0_fgetchar_int(FILE *stream);


unsigned char uart0_ischar_int(void);
unsigned short uart0_txbufferfree(void);
int uart0_peek_int(void);
void uart0_ungetch_int(unsigned char c);
BUFFEREDIO *uart0_get_rxbuf(void);
BUFFEREDIO *uart0_get_txbuf(void);


#endif

#endif
