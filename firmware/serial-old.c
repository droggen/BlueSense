/*
   Copyright (C) 2009:
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

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "serial.h"
#include "main.h"
#include "circbuf.h"

volatile struct SerialData SerialData0;
volatile struct SerialData SerialData1;

// Memory for the serial buffers
unsigned char _serial0_rx_buffer[SERIAL0_RX_BUFFERSIZE_MAX];
unsigned char _serial0_tx_buffer[SERIAL0_TX_BUFFERSIZE_MAX];
unsigned char _serial1_rx_buffer[SERIAL1_RX_BUFFERSIZE_MAX];
unsigned char _serial1_tx_buffer[SERIAL1_TX_BUFFERSIZE_MAX];

unsigned short serial1_tx_buffersize_modes[3]={SERIAL1_TX_BUFFERSIZE_L0,SERIAL1_TX_BUFFERSIZE_L1,SERIAL1_TX_BUFFERSIZE_L2};

// Init the buffer structures
// Must be called before any use of the uart functions
void uart_init(unsigned char mode)
{
	unsigned short serial1_tx_buffersize;
	if(mode>2)
		mode=2;
	serial1_tx_buffersize = serial1_tx_buffersize_modes[mode];
	
	SerialData0.rx.buffer=_serial0_rx_buffer;
	SerialData0.rx.size=SERIAL0_RX_BUFFERSIZE_MAX;
	SerialData0.rx.mask=SERIAL0_RX_BUFFERSIZE_MAX-1;
	
	SerialData0.tx.buffer=_serial0_tx_buffer;
	SerialData0.tx.size=SERIAL0_TX_BUFFERSIZE_MAX;
	SerialData0.tx.mask=SERIAL0_TX_BUFFERSIZE_MAX-1;
	
	SerialData1.rx.buffer=_serial1_rx_buffer;
	SerialData1.rx.size=SERIAL1_RX_BUFFERSIZE_MAX;
	SerialData1.rx.mask=SERIAL1_RX_BUFFERSIZE_MAX-1;
	
	SerialData1.tx.buffer=_serial1_tx_buffer;
	SerialData1.tx.size=serial1_tx_buffersize;
	SerialData1.tx.mask=serial1_tx_buffersize-1;
	
	buffer_clear(&SerialData0.rx);
	buffer_clear(&SerialData0.tx);
	buffer_clear(&SerialData1.rx);
	buffer_clear(&SerialData1.tx);
}

/*
	Returns the size of the uart buffers.
*/
void uart_getbuffersize(unsigned short *rx0,unsigned short *tx0,unsigned short *rx1,unsigned short *tx1)
{
	*rx0 = SerialData0.rx.size;
	*tx0 = SerialData0.tx.size;
	*rx1 = SerialData1.rx.size;
	*tx1 = SerialData1.tx.size;
}







/******************************************************************************
*******************************************************************************
*******************************************************************************
UART MANAGEMENT   UART MANAGEMENT   UART MANAGEMENT   UART MANAGEMENT   
*******************************************************************************
******************************************************************************* 
******************************************************************************/
unsigned char  (*uart0_rx_callback)(unsigned char)=0;
unsigned char  (*uart1_rx_callback)(unsigned char)=0;

/******************************************************************************
*******************************************************************************
INITIALIZATION   INITIALIZATION   INITIALIZATION   INITIALIZATION   INITIALIZATION   
*******************************************************************************
******************************************************************************/
int uart0_init(unsigned int ubrr, unsigned char u2x)
{
	uart0_clearbuffers();

	// Setup the serial port
	UCSR0B = 0x00; 				  //disable while setting baud rate
	UCSR0C = 0x06; 				  // Asyn,NoParity,1StopBit,8Bit,

	if(u2x)
		UCSR0A = 0x02;				  // U2X = 1
	else
		UCSR0A = 0x00;
		
	UBRR0H = (unsigned char)((ubrr>>8)&0x000F);
	UBRR0L = (unsigned char)(ubrr&0x00FF);	

	UCSR0B = (1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0);			// RX/TX Enable, RX/TX Interrupt Enable

	return 1;
}

int uart1_init(unsigned int ubrr, unsigned char u2x)
{
	uart1_clearbuffers();

	// Setup the serial port
	UCSR1B = 0x00; 				  //disable while setting baud rate
	UCSR1C = 0x06; 				  // Asyn,NoParity,1StopBit,8Bit,

	if(u2x)
		UCSR1A = 0x02;				  // U2X = 1
	else
		UCSR1A = 0x00;
		
	UBRR1H = (unsigned char)((ubrr>>8)&0x000F);
	UBRR1L = (unsigned char)(ubrr&0x00FF);			

	UCSR1B = (1<<RXCIE1)|(1<<RXEN1)|(1<<TXEN1);			// RX/TX Enable, RX/TX Interrupt Enable

	// If RTS-enabled communication, enable interrupt on pin change
/*#if ENABLE_BLUETOOTH_RTS
	PCMSK3 |= 1<<PCINT28;	// PCINT28 (PortD.4) pin change interrupt enable
	PCICR |= 1<<PCIE3;		// Enable pin change interrupt for PCINT31..24
#endif*/

	return 1;
}

/*
	Sets whether the read functions must be blocking or non blocking.
	In non-blocking mode, when no characters are available, read functions return EOF.
*/
void uart_setblocking(FILE *file,unsigned char blocking)
{
	fdev_set_udata(file,(void*)(blocking?0:1));
}
unsigned char uart_isblocking(FILE *file)
{
	return fdev_get_udata(file)?0:1;
}

/*
	uartx_clearbuffers: clears all the data in the transmit and receive buffers.
*/
void uart0_clearbuffers(void)
{
	buffer_clear(&SerialData0.rx);
	buffer_clear(&SerialData0.tx);	
}
void uart1_clearbuffers(void)
{
	buffer_clear(&SerialData1.rx);
	buffer_clear(&SerialData1.tx);
}

/******************************************************************************
*******************************************************************************
DIRECT ACCESS   DIRECT ACCESS   DIRECT ACCESS   DIRECT ACCESS   DIRECT ACCESS  
******************************************************************************* 
******************************************************************************/
int uart0_putchar(char ch)		// Placeholder
{
	return uart0_fputchar(ch,0);
}

int uart0_getchar()				// Placeholder
{
	return uart0_fgetchar(0);
}

int uart0_fputchar(char ch,FILE* stream)
{
	 while(!(UCSR0A & 0x20)); // wait for empty transmit buffer
	 UDR0 = ch;     	 		 // write char
	return 0;
}

int uart0_fgetchar(FILE *stream)
{
	char c;
	while(!(UCSR0A & 0x80)); // wait for receive complete
	c=UDR0;
	return c;
}

int uart1_putchar(char ch)		// Placeholder
{
	return uart1_fputchar(ch, 0);
}

int uart1_getchar()				// Placeholder
{
	return uart1_fgetchar(0);
}

int uart1_fputchar(char ch,FILE* stream)
{
	while(!(UCSR1A & 0x20)); // wait for empty transmit buffer
	UDR1 = ch;     	 		 // write char
	return 0;
}

int uart1_fgetchar(FILE *stream)
{
	char c;
	while(!(UCSR1A & 0x80)); // wait for receive complete
	c=UDR1;
	return c;
}

/******************************************************************************
*******************************************************************************
INTERRUPT-DRIVEN ACCESS   INTERRUPT-DRIVEN ACCESS   INTERRUPT-DRIVEN ACCESS   
*******************************************************************************
******************************************************************************/





int uart0_fputchar_int(char c, FILE*stream)
{
	// Wait until send buffer is free. 
	while( buffer_isfull(&SerialData0.tx) );
	// Store the character in the buffer
	buffer_put(&SerialData0.tx,c);
	
	// Trigger an interrupt
	UCSR0B|=(1<<UDRIE0);		// will generate interrupt when UDR is empty
	return 0;
}

/******************************************************************************
	uart0_fputbuf_int
*******************************************************************************	
	Atomically writes a buffer to a stream, or fails if the buffer is full.
			
	Return value:	
		zero:			success
		nonzero:	error

******************************************************************************/
unsigned char uart0_fputbuf_int(unsigned char *data,unsigned char n)
{
	if(uart0_txbufferfree()>=n)
	{
		for(unsigned short i=0;i<n;i++)
			buffer_put(&SerialData0.tx,data[i]);
		// Trigger an interrupt
		UCSR0B|=(1<<UDRIE0);		// will generate interrupt when UDR is empty
		return 0;
	}
	return 1;
}

int uart1_fputchar_int(char c, FILE*stream)
{
	// Wait until send buffer is free. 
	while( buffer_isfull(&SerialData1.tx) );
	// Store the character in the buffer
	buffer_put(&SerialData1.tx,c);
	
	// Trigger an interrupt
	UCSR1B|=(1<<UDRIE1);		// will generate interrupt when UDR is empty
	return 0;
}
/******************************************************************************
	uart1_fputbuf_int
*******************************************************************************	
	Atomically writes a buffer to a stream, or fails if the buffer is full.
			
	Return value:	
		zero:			success
		nonzero:	error

******************************************************************************/
unsigned char uart1_fputbuf_int(unsigned char *data,unsigned char n)
{
	if(uart1_txbufferfree()>=n)
	{
		for(unsigned short i=0;i<n;i++)
			buffer_put(&SerialData1.tx,data[i]);
		// Trigger an interrupt
		UCSR1B|=(1<<UDRIE1);		// will generate interrupt when UDR is empty
		return 0;
	}
	return 1;
}


int uart0_fgetchar_nonblock_int(FILE *stream)
{
	char c;
	if(buffer_isempty(&SerialData0.rx))
		return EOF;
	c = buffer_get(&SerialData0.rx);
	return ((int)c)&0xff;
}

int uart1_fgetchar_nonblock_int(FILE *stream)
{
	char c;
	if(buffer_isempty(&SerialData1.rx))
		return EOF;
	c = buffer_get(&SerialData1.rx);
	return ((int)c)&0xff;
}


int uart0_fgetchar_int(FILE *stream)
{
	int c;
	do{ c=uart0_fgetchar_nonblock_int(stream);}
	while(c==EOF && fdev_get_udata(stream)==0);
	return c;
}
int uart1_fgetchar_int(FILE *stream)
{
	int c;
	do{c=uart1_fgetchar_nonblock_int(stream);}
	while(c==EOF && fdev_get_udata(stream)==0);
	return c;
}


/******************************************************************************
Interrupt vectors
******************************************************************************/
/*ISR(USART0_UDRE_vect)
{
	if(buffer_isempty(&SerialData0.tx))					// No data to transmit
	{
		UCSR0B&=~(1<<UDRIE0);		// Deactivate interrupt otherwise we reenter continuously the loop
		return;
	}
	// Write data
	UDR0 = buffer_get(&SerialData0.tx);
}*/
ISR(USART1_UDRE_vect)
{
	// If RTS is enabled, and RTS is set, clear the interrupt flag and return (i.e. do nothing because the receiver is busy)
/*#if ENABLE_BLUETOOTH_RTS
	if(PIND&0x10)
	{
		UCSR1B&=~(1<<UDRIE1);								// Deactivate interrupt otherwise we reenter continuously the loop
		return;
	}
#endif*/
	
	if(buffer_isempty(&SerialData1.tx))					// No data to transmit
	{
		UCSR1B&=~(1<<UDRIE1);								// Deactivate interrupt otherwise we reenter continuously the loop
		return;
	}
	// Write data
	UDR1 = buffer_get(&SerialData1.tx);
}

/*
	Interrupt: data received
*/
/*ISR(USART0_RX_vect)
{
	unsigned char c=UDR0;
	if(uart0_rx_callback==0 || (*uart0_rx_callback)(c)==1)
		buffer_put(&SerialData0.rx,c);
	
}*/
ISR(USART1_RX_vect)
{
	unsigned char c=UDR1;
	if(uart1_rx_callback==0 || (*uart1_rx_callback)(c)==1)
		buffer_put(&SerialData1.rx,c);
}


/*
	RTS-enabled communication with the bluetooth chip
*/
#if ENABLE_BLUETOOTH_RTS
	/*
		Interrupt vector for RTS pin change on UART 1 (Bluetooth)
		We catch all the PCINT3, which corresponds to pins 31..24, but only PCINT28 is activated in this code.
	*/
	/*ISR(PCINT3_vect)				// PortD.4 interrupt on change
	{
		if(PIND&0x10)			// Trigger a bluetooth interrupt on high to low transitions. If we are high, we return.
			return;
		// todo check transition....
		UCSR1B|=(1<<UDRIE1);		// will generate interrupt when UDR is empty
	}*/
#endif


/******************************************************************************
	Stream manipulation
******************************************************************************/
unsigned char uart0_ischar_int(void)
{
	if(buffer_isempty(&SerialData0.rx))
		return 0;
	return 1;
}
unsigned char uart1_ischar_int(void)
{
	if(buffer_isempty(&SerialData1.rx))
		return 0;
	return 1;
}
unsigned short uart0_txbufferfree(void)
{
	return buffer_freespace(&SerialData0.tx);
}
unsigned short uart1_txbufferfree(void)
{
	return buffer_freespace(&SerialData1.tx);
}

int uart0_peek_int(void)
{
	return SerialData0.rx.buffer[SerialData0.rx.rdptr];
}

int uart1_peek_int(void)
{
	return SerialData1.rx.buffer[SerialData1.rx.rdptr];
}

void uart0_ungetch_int(unsigned char c)
{
	buffer_unget(&SerialData0.rx,c);
}
void uart1_ungetch_int(unsigned char c)
{
	buffer_unget(&SerialData1.rx,c);
}
/*
	Flushes f by reading until empty
*/
void flush(FILE *file)
{
	int c;
	// Non-blocking mode
	uart_setblocking(file,0);
	while((c=fgetc(file))!=EOF);
	// Return to blocking mode
	uart_setblocking(file,1);
}


