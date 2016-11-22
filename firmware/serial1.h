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
#ifndef __SERIAL1_H
#define __SERIAL1_H

#ifdef ENABLE_SERIAL1

#include <stdio.h>
#include "serial.h"
#include "circbuf.h"

#define ENABLE_BLUETOOTH_RTS

// Maximum size of the rx/tx buffers. Must be a power of 2.
#define SERIAL1_RX_BUFFERSIZE_MAX 512
#define SERIAL1_TX_BUFFERSIZE_MAX 512
//#define SERIAL1_TX_BUFFERSIZE_MAX 2048
// Serial buffers
extern volatile BUFFEREDIO SerialData1Rx;
extern volatile BUFFEREDIO SerialData1Tx;

void Serial1RTSToggle(unsigned char rts);
void USART1_RX_vect_core(void);

extern volatile unsigned long Serial1DOR;

// Callbacks for hooking into the interrupt routines.
extern unsigned char  (*uart1_rx_callback)(unsigned char);

// Initialisation
int uart1_init(unsigned int ubrr, unsigned char u2x);
void uart1_deinit(void);
void uart1_clearbuffers(void);


/*
	Direct serial access
*/
int uart1_fputchar(char ch,FILE* stream);
int uart1_fgetchar(FILE *stream);
int uart1_putchar(char ch);
int uart1_getchar(void);


/*
	Interrupt-driven serial access
*/
int uart1_fputchar_int(char c, FILE*stream);
unsigned char uart1_fputbuf_int(unsigned char *data,unsigned char n);
int uart1_fgetchar_nonblock_int(FILE *stream);
int uart1_fgetchar_int(FILE *stream);


unsigned char uart1_ischar_int(void);
unsigned short uart1_txbufferfree(void);
int uart1_peek_int(void);
void uart1_ungetch_int(unsigned char c);
BUFFEREDIO *uart1_get_rxbuf(void);
BUFFEREDIO *uart1_get_txbuf(void);

#endif

#endif
