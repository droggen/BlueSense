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

#include "cpu.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>
#include <stdlib.h>
#include <string.h>
#include <util/delay.h>
#include <stdio.h>
#include "circbuf.h"
#include "serial.h"
#include "serial1.h"

#ifdef ENABLE_SERIAL1

volatile BUFFEREDIO SerialData1Rx;
volatile BUFFEREDIO SerialData1Tx;

volatile unsigned long Serial1DOR=0;

// Memory for the serial buffers
unsigned char _serial1_rx_buffer[SERIAL1_RX_BUFFERSIZE_MAX];
unsigned char _serial1_tx_buffer[SERIAL1_TX_BUFFERSIZE_MAX];



/******************************************************************************
	uart1_rx_callback
*******************************************************************************	
	Pointer to a callback function called everytime a character is received.
	This callback must return 1 to place the character in the receive queue, 
	0 if the character must be discarded (e.g. because processed otherwise).
		
	If the pointer is null, no callback is called.
******************************************************************************/
unsigned char  (*uart1_rx_callback)(unsigned char)=0;

/******************************************************************************
	uart1_init
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
int uart1_init(unsigned int ubrr, unsigned char u2x)
{
	SerialData1Rx.buffer=_serial1_rx_buffer;
	SerialData1Rx.size=SERIAL1_RX_BUFFERSIZE_MAX;
	SerialData1Rx.mask=SERIAL1_RX_BUFFERSIZE_MAX-1;
	
	SerialData1Tx.buffer=_serial1_tx_buffer;
	SerialData1Tx.size=SERIAL1_TX_BUFFERSIZE_MAX;
	SerialData1Tx.mask=SERIAL1_TX_BUFFERSIZE_MAX-1;
	
	uart1_clearbuffers();

	// Setup the serial port
	UCSR1B = 0x00; 				  //disable while setting baud rate
	UCSR1C = 0x06; 				  // Asyn,NoParity,1StopBit,8Bit,

	if(u2x)
		UCSR1A = 0x02;				  // U2X = 1
	else
		UCSR1A = 0x00;
		
	UBRR1 = ubrr;
	//UBRR1H = (unsigned char)((ubrr>>8)&0x000F);
	//UBRR1L = (unsigned char)(ubrr&0x00FF);			

//	UCSR1B = (1<<RXCIE1)|(1<<RXEN1)|(1<<TXEN1);			// RX/TX Enable, RX/TX Interrupt Enable
	UCSR1B = (1<<RXCIE1)|(1<<RXEN1)|(1<<TXEN1);			// RX/TX Enable, RX/TX Interrupt Enable
	// XXX
	//UCSR1B = (1<<TXCIE1)|(1<<RXCIE1)|(1<<RXEN1)|(1<<TXEN1);			// RX/TX Enable, RX/TX Interrupt Enable
	//UCSR1B = (1<<RXEN1)|(1<<TXEN1);			// RX/TX Enable, RX/TX Interrupt Enable

	// If RTS-enabled communication, enable interrupt on pin change
/*#if ENABLE_BLUETOOTH_RTS
	PCMSK3 |= 1<<PCINT28;	// PCINT28 (PortD.4) pin change interrupt enable
	PCICR |= 1<<PCIE3;		// Enable pin change interrupt for PCINT31..24
#endif*/

	return 0;
}

/******************************************************************************
	uart1_deinit
*******************************************************************************	
	Deinitialises UART1.
******************************************************************************/
void uart1_deinit(void)
{
	UCSR1B=0;
}

/******************************************************************************
	uart1_clearbuffers
*******************************************************************************	
	Clear all the data in the RX and TX buffers.
******************************************************************************/
void uart1_clearbuffers(void)
{
	buffer_clear(&SerialData1Rx);
	buffer_clear(&SerialData1Tx);
}

#if BOOTLOADER==0
/******************************************************************************
	uart1_putchar
*******************************************************************************	
	Alias to uart1_fputchar
******************************************************************************/
int uart1_putchar(char ch)		
{
	return uart1_fputchar(ch, 0);
}
/******************************************************************************
	uart1_getchar
*******************************************************************************	
	Alias to uart1_fgetchar
******************************************************************************/
int uart1_getchar()				// Placeholder
{
	return uart1_fgetchar(0);
}
/******************************************************************************
	uart1_fputchar
*******************************************************************************	
	Non-interrupt driven, blocking write.
	
	ch:			character to write
	stream:	unused, can be 0
******************************************************************************/
int uart1_fputchar(char ch,FILE* stream)
{
	while(!(UCSR1A & 0x20)); // wait for empty transmit buffer
	UDR1 = ch;     	 		 // write char
	return 0;
}

/******************************************************************************
	uart1_fgetchar
*******************************************************************************	
	Non-interrupt driven, blocking read.
	
	stream:	unused, can be 0
	
	Return value:		character read
******************************************************************************/
int uart1_fgetchar(FILE *stream)
{
	char c;
	while(!(UCSR1A & 0x80)); // wait for receive complete
	c=UDR1;
	return c;
}
#endif


/******************************************************************************
	uart1_fputchar_int
*******************************************************************************	
	Interrupt driven, blocking write.
	
	c:			character to write
	stream:	unused, can be 0
	
	Return value:	0
******************************************************************************/
int uart1_fputchar_int(char c, FILE*stream)
{
	// Wait until send buffer is free. 
	while( buffer_isfull(&SerialData1Tx) );
	// Store the character in the buffer
	buffer_put(&SerialData1Tx,c);
	
	// Trigger an interrupt when UDR is empty
	UCSR1B|=(1<<UDRIE1);		
	// XXX
	/*if(UCSR1A&(1<<UDRE1))
		UDR1 = buffer_get(&SerialData1Rx);*/
	return 0;
}

/******************************************************************************
	uart1_fgetchar_nonblock_int
*******************************************************************************	
	Interrupt driven, non-blocking read.
	
	stream:	unused, can be 0
	
	Return value:	
		EOF (-1):	No data available
		other:		Character read
******************************************************************************/
int uart1_fgetchar_nonblock_int(FILE *stream)
{
	char c;
	if(buffer_isempty(&SerialData1Rx))
		return EOF;
	c = buffer_get(&SerialData1Rx);
	return ((int)c)&0xff;
}

/******************************************************************************
	uart1_fgetchar_int
*******************************************************************************	
	Interrupt driven, possibly blocking read. 
	Whether the access is blocking or non-blocking is defined by the previous 
	call to uart_setblocking.
	
	stream:	unused, can be 0.
	
	Return value:	
		EOF (-1):	No data available (only happens if non-blocking is enabled)
		other:		Character read
******************************************************************************/
int uart1_fgetchar_int(FILE *stream)
{
	int c;
	do{c=uart1_fgetchar_nonblock_int(stream);}
	while(c==EOF && serial_isblocking(stream));
	
	return c;
}

#if BOOTLOADER==0
/******************************************************************************
	uart1_fputbuf_int
*******************************************************************************	
	Atomically writes a buffer to a stream, or fails if the buffer is full.
			
	Return value:	
		0:				success
		nonzero:	error
******************************************************************************/
unsigned char uart1_fputbuf_int(unsigned char *data,unsigned char n)
{
	if(uart1_txbufferfree()>=n)
	{
		for(unsigned short i=0;i<n;i++)
			buffer_put(&SerialData1Tx,data[i]);
		// Trigger an interrupt when UDR is empty
		UCSR1B|=(1<<UDRIE1);		
		// XXX
		/*if(UCSR1A&(1<<UDRE1))
			UDR1 = buffer_get(&SerialData1Rx);*/
		return 0;
	}
	return 1;
}
#endif

/*
	Interrupt: transmit buffer empty
*/
ISR(USART1_UDRE_vect)
{
	#ifdef ENABLE_BLUETOOTH_RTS
	// If RTS is enabled, and RTS is set, clear the interrupt flag and return (i.e. do nothing because the receiver is busy)
	//_delay_us(10);
	if(PIND&0x10)
	{
		UCSR1B&=~(1<<UDRIE1);								// Deactivate interrupt otherwise we reenter continuously the loop
		return;
	}
#endif
	
	if(buffer_isempty(&SerialData1Tx))					// No data to transmit
	{
		UCSR1B&=~(1<<UDRIE1);								// Deactivate interrupt otherwise we reenter continuously the loop
		return;
	}
	// Write data
	UDR1 = buffer_get(&SerialData1Tx);
}
/*ISR(USART1_TX_vect)
{
#ifdef ENABLE_BLUETOOTH_RTS
	// If RTS is enabled, and RTS is set, clear the interrupt flag and return (i.e. do nothing because the receiver is busy)
	if(PIND&0x10)
	{
		UCSR1B&=~(1<<UDRIE1);								// Deactivate interrupt otherwise we reenter continuously the loop
		return;
	}
#endif
	
	if(buffer_isempty(&SerialData1Tx))					// No data to transmit
	{
		UCSR1B&=~(1<<UDRIE1);								// Deactivate interrupt otherwise we reenter continuously the loop
		return;
	}
	// Write data
	UDR1 = buffer_get(&SerialData1Tx);
}*/
/*
	Interrupt: data received
*/
void USART1_RX_vect_core(void)
{
	if(UCSR1A&(1<<DOR1))
		Serial1DOR++;
	unsigned char c=UDR1;
	if(uart1_rx_callback==0 || (*uart1_rx_callback)(c)==1)
	{
		// If rx buffer full: choose to lose rx data
		if(!buffer_isfull(&SerialData1Rx))
			buffer_put(&SerialData1Rx,c);
	}
} 
ISR(USART1_RX_vect)
{
	USART1_RX_vect_core();
	/*unsigned char c=UDR1;
	if(uart1_rx_callback==0 || (*uart1_rx_callback)(c)==1)
	{
		// If rx buffer full: choose to lose rx data
		if(!buffer_isfull(&SerialData1Rx))
			buffer_put(&SerialData1Rx,c);
	}*/
}


/*
	RTS-enabled communication with the bluetooth chip
*/
#ifdef ENABLE_BLUETOOTH_RTS
void Serial1RTSToggle(unsigned char rts)
{
	if(rts)			// Trigger a bluetooth interrupt on high to low transitions. If we are high, we return.
		return;
	UCSR1B|=(1<<UDRIE1);		// will generate interrupt when UDR is empty
	// XXX
	// if no ongoing transfer generate TXC interrupt
	//if(UCSR1A&(1<<UDRE1))
		//UCSR1A|=(1<<TXC1);		// will generate TXC interrupt
	//	if(UCSR1A&(1<<UDRE1))
		//	UDR1 = buffer_get(&SerialData1Rx);
}
#else
void Serial1RTSToggle(unsigned char rts)
{
}
#endif


/******************************************************************************
	Buffer manipulation
******************************************************************************/
#if BOOTLOADER==0
unsigned char uart1_ischar_int(void)
{
	if(buffer_isempty(&SerialData1Rx))
		return 0;
	return 1;
}
unsigned short uart1_txbufferfree(void)
{
	return buffer_freespace(&SerialData1Tx);
}
int uart1_peek_int(void)
{
	return SerialData1Rx.buffer[SerialData1Rx.rdptr];
}
void uart1_ungetch_int(unsigned char c)
{
	buffer_unget(&SerialData1Rx,c);
}
#endif

BUFFEREDIO *uart1_get_rxbuf(void)
{
	return &SerialData1Rx;
}
BUFFEREDIO *uart1_get_txbuf(void)
{
	return &SerialData1Tx;
}



#endif