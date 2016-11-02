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

#ifdef ENABLE_SERIAL0

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "circbuf.h"
#include "serial.h"
#include "serial0.h"

volatile BUFFEREDIO SerialData0Rx;
volatile BUFFEREDIO SerialData0Tx;

// Memory for the serial buffers
unsigned char _serial0_rx_buffer[SERIAL0_RX_BUFFERSIZE_MAX];
unsigned char _serial0_tx_buffer[SERIAL0_TX_BUFFERSIZE_MAX];

/******************************************************************************
	uart0_rx_callback
*******************************************************************************	
	Pointer to a callback function called everytime a character is received.
	This callback must return 1 to place the character in the receive queue, 
	0 if the character must be discarded (e.g. because processed otherwise).
		
	If the pointer is null, no callback is called.
******************************************************************************/
unsigned char  (*uart0_rx_callback)(unsigned char)=0;

/******************************************************************************
	uart0_init
*******************************************************************************	
	Initialises UART1 including data structures for interrupt-driven access and 
	UART port settings.
		
	ubrr:			Baud rate register
	u2x:			Double speed (1) or normal speed (0)
	
	Baud rate is: 	fosc/(16*(ubrr+1)), with u2x=0
									fosc/(9*(ubrr+1)), with u2x=1
		
	Return value:
		0:				Success
******************************************************************************/
int uart0_init(unsigned int ubrr, unsigned char u2x)
{
	SerialData0Rx.buffer=_serial0_rx_buffer;
	SerialData0Rx.size=SERIAL1_RX_BUFFERSIZE_MAX;
	SerialData0Rx.mask=SERIAL1_RX_BUFFERSIZE_MAX-1;
	
	SerialData0Tx.buffer=_serial0_tx_buffer;
	SerialData0Tx.size=SERIAL1_TX_BUFFERSIZE_MAX;
	SerialData0Tx.mask=SERIAL1_TX_BUFFERSIZE_MAX-1;
	
	uart0_clearbuffers();

	// Setup the serial port
	UCSR0B = 0x00; 				  //disable while setting baud rate
	UCSR0C = 0x06; 				  // Asyn,NoParity,1StopBit,8Bit,

	if(u2x)
		UCSR0A = 0x02;				  // U2X = 1
	else
		UCSR0A = 0x00;
		
	UBRR0 = ubrr;
	//UBRR1H = (unsigned char)((ubrr>>8)&0x000F);
	//UBRR1L = (unsigned char)(ubrr&0x00FF);			

	UCSR0B = (1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0);			// RX/TX Enable, RX/TX Interrupt Enable

	return 0;
}

/******************************************************************************
	uart0_clearbuffers
*******************************************************************************	
	Clear all the data in the RX and TX buffers.
******************************************************************************/
void uart0_clearbuffers(void)
{
	buffer_clear(&SerialData0Rx);
	buffer_clear(&SerialData0Tx);
}

/******************************************************************************
	uart0_putchar
*******************************************************************************	
	Alias to uart0_fputchar
******************************************************************************/
int uart0_putchar(char ch)		
{
	return uart0_fputchar(ch, 0);
}
/******************************************************************************
	uart0_getchar
*******************************************************************************	
	Alias to uart0_fgetchar
******************************************************************************/
int uart0_getchar()				// Placeholder
{
	return uart0_fgetchar(0);
}
/******************************************************************************
	uart0_fputchar
*******************************************************************************	
	Non-interrupt driven, blocking write.
	
	ch:			character to write
	stream:	unused, can be 0
******************************************************************************/
int uart0_fputchar(char ch,FILE* stream)
{
	while(!(UCSR0A & 0x20)); // wait for empty transmit buffer
	UDR0 = ch;     	 		 // write char
	return 0;
}

/******************************************************************************
	uart0_fgetchar
*******************************************************************************	
	Non-interrupt driven, blocking read.
	
	stream:	unused, can be 0
	
	Return value:		character read
******************************************************************************/
int uart0_fgetchar(FILE *stream)
{
	char c;
	while(!(UCSR0A & 0x80)); // wait for receive complete
	c=UDR0;
	return c;
}



/******************************************************************************
	uart0_fputchar_int
*******************************************************************************	
	Interrupt driven, blocking write.
	
	c:			character to write
	stream:	unused, can be 0
	
	Return value:	0
******************************************************************************/
int uart0_fputchar_int(char c, FILE*stream)
{
	// Wait until send buffer is free. 
	while( buffer_isfull(&SerialData0Tx) );
	// Store the character in the buffer
	buffer_put(&SerialData0Tx,c);
	
	// Trigger an interrupt when UDR is empty
	UCSR0B|=(1<<UDRIE0);		
	return 0;
}

/******************************************************************************
	uart0_fgetchar_nonblock_int
*******************************************************************************	
	Interrupt driven, non-blocking read.
	
	stream:	unused, can be 0
	
	Return value:	
		EOF (-1):	No data available
		other:		Character read
******************************************************************************/
int uart0_fgetchar_nonblock_int(FILE *stream)
{
	char c;
	if(buffer_isempty(&SerialData0Rx))
		return EOF;
	c = buffer_get(&SerialData0Rx);
	return ((int)c)&0xff;
}

/******************************************************************************
	uart0_fgetchar_int
*******************************************************************************	
	Interrupt driven, possibly blocking read. 
	Whether the access is blocking or non-blocking is defined by the previous 
	call to uart_setblocking.
	
	stream:	unused, can be 0.
	
	Return value:	
		EOF (-1):	No data available (only happens if non-blocking is enabled)
		other:		Character read
******************************************************************************/
int uart0_fgetchar_int(FILE *stream)
{
	int c;
	do{c=uart0_fgetchar_nonblock_int(stream);}
	while(c==EOF && serial_isblocking(stream));
	
	return c;
}

/******************************************************************************
	uart0_fputbuf_int
*******************************************************************************	
	Atomically writes a buffer to a stream, or fails if the buffer is full.
			
	Return value:	
		0:				success
		nonzero:	error
******************************************************************************/
unsigned char uart0_fputbuf_int(unsigned char *data,unsigned char n)
{
	if(uart0_txbufferfree()>=n)
	{
		for(unsigned short i=0;i<n;i++)
			buffer_put(&SerialData0Tx,data[i]);
		// Trigger an interrupt when UDR is empty
		UCSR0B|=(1<<UDRIE0);		
		return 0;
	}
	return 1;
}

/*
	Interrupt: transmit buffer empty
*/
ISR(USART0_UDRE_vect)
{
	if(buffer_isempty(&SerialData0Tx))					// No data to transmit
	{
		UCSR0B&=~(1<<UDRIE0);		// Deactivate interrupt otherwise we reenter continuously the loop
		return;
	}
	// Write data
	UDR0 = buffer_get(&SerialData0Tx);
}
/*
	Interrupt: data received
*/
ISR(USART0_RX_vect)
{
	unsigned char c=UDR0;
	if(uart0_rx_callback==0 || (*uart0_rx_callback)(c)==1)
	{
		// If rx buffer full: choose to lose rx data
		if(!buffer_isfull(&SerialData0Rx))
			buffer_put(&SerialData0Rx,c);
	}
}


/******************************************************************************
	Buffer manipulation
******************************************************************************/
unsigned char uart0_ischar_int(void)
{
	if(buffer_isempty(&SerialData0Rx))
		return 0;
	return 1;
}
unsigned short uart0_txbufferfree(void)
{
	return buffer_freespace(&SerialData0Tx);
}
int uart0_peek_int(void)
{
	return SerialData0Rx.buffer[SerialData0Rx.rdptr];
}
void uart0_ungetch_int(unsigned char c)
{
	buffer_unget(&SerialData0Rx,c);
}
BUFFEREDIO *uart0_get_rxbuf(void)
{
	return &SerialData0Rx;
}
BUFFEREDIO *uart0_get_txbuf(void)
{
	return &SerialData0Tx;
}

#endif