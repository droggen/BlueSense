#include "cpu.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <stdio.h>
#include <string.h>

#include "main.h"
#include "adc.h"
#include "serial.h"
#include "i2c.h"
#include "ds3232.h"
#include "rn41.h"
#include "mpu.h"
#include "mpu_test.h"
#include "pkt.h"
#include "wait.h"
#include "init.h"
#include "lcd.h"
#include "fb.h"
#include "uiconfig.h"
#include "helper.h"
#include "i2c_internal.h"
#include "mode_demo.h"
//#include "streaming.h"
#include "dbg.h"
#include "mpu-usart0.h"
#include "spi-usart0.h"

/*
	File: spi-usart0
	
	Provides functions to handle SPI communication using USART0. 
	
	
	The key functions are:		
	
	* spiusart0_init: 		setups usart0 for SPI communication	
	* spiusart0_rw:			selects the peripheral and exchanges 1 byte. Waits for peripheral available before start, waits for completion. Do not use in interrupts.
	* spiusart0_rwn:			selects the peripheral and exchanges n byte. Waits for peripheral available before start, waits for completion. Do not use in interrupts.
	* spiusart0_rwn_int:		selects the peripheral and exchanges n byte using interrupt-driven transfer.  Waits for peripheral available before start, waits for completion. Do not use in interrupts.
	* spiusart0_rwn_int_cb:	selects the peripheral and exchanges n byte using interrupt-driven transfer. Returns if peripheral not available, calls a callback on transfer completion. Interrupt safe.
	
	The only function safe to call from an interrupt routing is spiusart0_rwn_int_cb. The other functions can cause deadlocks and must not be used in interrupts.
	
	All functions will wait for the peripheral to be available, except spiusart0_rwn_int_cb which will return immediately with an error if the peripheral is used. This
	allows calls to all functions to be mixed from main code or interrupts (within interrupts only spiusart0_rwn_int_cb is allowed). 
	
	The peripheral is selected with macros SPIUSART0_SELECT and SPIUSART0_DESELECT. When the assembly ISR is used, the file spi-usart0-isr.S must be modified to handle the select/deselect.
	
*/



/*
	Note on interrupts:
	UDR empty interrupt:			executes as long as UDR is empty; must write to UDR or clear UDRIE
	RX complete interrupt:		executes as long as RX complete flag is set; must read UDR to clear interrupt
	TX complete interrupt:		executes once when TX complete flag set; flag automatically cleared
	
	In SPI mode, a transmission and reception occur simultaneously. 
	RXC or TXC could equally be used. Here we use TXC.
	
*/

volatile unsigned char *_spiusart0_bufferptr;
volatile unsigned short _spiusart0_n;
volatile unsigned char _spiusart0_ongoing=0;
void (*_spiusart0_callback)(void);
volatile unsigned int _spiusart0_txint=0;


extern void spiisr(void);


//ISR(USART0_TX_vect)
void vector22x(void)
{
	//dbg_fputchar_nonblock('X',0);
	//spiisr();	
	//dbg_fputchar_nonblock('x',0);
	//_delay_us(250);
	//return;
	
	//_spiusart0_txint++;
	//system_led_toggle(0b10);
	
	
	//dbg_fputchar_nonblock('X',0);
	//unsigned char s = UCSR0A;
	/*dbg_fputchar_nonblock(hex2chr(s>>4),0);
	dbg_fputchar_nonblock(hex2chr(s&0xf),0);
	dbg_fputchar_nonblock(' ',0);
	dbg_fputchar_nonblock(hex2chr(_spiusart0_txint>>12),0);
	dbg_fputchar_nonblock(hex2chr((_spiusart0_txint>>8)&0xf),0);
	dbg_fputchar_nonblock(hex2chr((_spiusart0_txint>>4)&0xf),0);
	dbg_fputchar_nonblock(hex2chr(_spiusart0_txint&0xf),0);
	dbg_fputchar_nonblock(' ',0);
	dbg_fputchar_nonblock(hex2chr(_spiusart0_n>>12),0);
	dbg_fputchar_nonblock(hex2chr((_spiusart0_n>>8)&0xf),0);
	dbg_fputchar_nonblock(hex2chr((_spiusart0_n>>4)&0xf),0);
	dbg_fputchar_nonblock(hex2chr(_spiusart0_n&0xf),0);
	dbg_fputchar_nonblock(' ',0);
	dbg_fputchar_nonblock(hex2chr( ((int)_spiusart0_bufferptr) >>12),0);
	dbg_fputchar_nonblock(hex2chr(( ((int)_spiusart0_bufferptr)>>8)&0xf),0);
	dbg_fputchar_nonblock(hex2chr(( ((int)_spiusart0_bufferptr)>>4)&0xf),0);
	dbg_fputchar_nonblock(hex2chr( ((int)_spiusart0_bufferptr)&0xf),0);
	dbg_fputchar_nonblock('\n',0);
	*/
	
	
	// Store received data in buffer
	*_spiusart0_bufferptr = UDR0;
	
	_spiusart0_n--;	
	
	// No more data
	if(_spiusart0_n==0)
	{
		// Deselect
		SPIUSART0_DESELECT;
		
		// Deactivate int
		UCSR0B &= 0b10111111;
		
		// Indicate done
		_spiusart0_ongoing=0;
		
		/*dbg_fputchar('I',0);
		dbg_fputchar('D',0);
		dbg_fputchar('A'+_spiusart0_bufferptr,0);
		dbg_fputchar(' ',0);
		dbg_fputchar('A'+_spiusart0_n,0);
		dbg_fputchar('\n',0);*/
		//printf_P("i d %d\n",_spiusart0_txint);
		
		// Call the callback;
		if(_spiusart0_callback)
			_spiusart0_callback();
		
		
		
		return;
	}
	
	_spiusart0_bufferptr++;
	
	
	// Write next data
	UDR0 = *_spiusart0_bufferptr;
	//UDR0 = 0x80;
}
/******************************************************************************
	function: spiusart0_init
*******************************************************************************	
	Initialise USART0 in SPI Master mode.		
******************************************************************************/

void spiusart0_init(void)
{
	// UBRR before activation
	UBRR0=0;
	
	// Set frame format
	// SPI mode: MSB first, data latched on rising edge, data must be transitionned on falling edge, sck high when idle -> mode 3
	UCSR0C = 0b11000011;			// Master, MSB first, UCPHA = 1, UCPOL = 1	
	
	UCSR0B = 0b00011000;			// Enable RX&TX
	//UCSR0B = 0b11111000;			// Enable RXCIE TXCIE UDRIE interrupts and RX TX 
	//UCSR0B = 0b11011000;			// Enable RXCIE TXCIE interrupts and RX TX 
	//UCSR0B = 0b01011000;			// Enable TXCIE interrupts and RX TX 
	
	// Set baud rate
	//UBRR0=1;				//	2764 KHz @ 11'059'200 Hz
	//UBRR0=3;				//	1382 KHz @ 11'059'200 Hz
	UBRR0=5;				//	921.6 KHz @ 11'059'200 Hz		// Works (default)
	//UBRR0=54;				//	100.5 KHz @ 11'059'200 Hz
	//UBRR0=255;			//	21.6 KHz @ 11'059'200 Hz
	
	
	_spiusart0_ongoing=0;
	
	printf_P(PSTR("SPI USART: A: %02X B: %02X C: %02X\n"),UCSR0A,UCSR0B,UCSR0C);
}
void spiusart0_deinit(void)
{
	// Deinitialise the SPI interface
	UCSR0B &= 0b10111111;	
}

/******************************************************************************
	function: _spiusart0_waitavailandreserve
*******************************************************************************	
	Waits until no SPI transaction occurs (_spiusart0_ongoing=0) and 
	immediately indicates a transaction is ongoing (_spiusart0_ongoing=1).
	Interrupts are managed to ensure no interrupt-initiated transaction can sneak
	in.	
******************************************************************************/
void _spiusart0_waitavailandreserve(void)
{
	// Wait for end of interrupt transfer if any transfer is ongoing
	spiusart0_waitavailandreserve_wait:
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		if(!_spiusart0_ongoing)
		{
			_spiusart0_ongoing=1;
			goto spiusart0_waitavailandreserve_start;
		}
	}
	// Ensures opportunities to manage interrupts
	_delay_us(1);				// 1 byte at 1MHz is 1uS. 
	goto spiusart0_waitavailandreserve_wait;
	
	// Start of transmission
	spiusart0_waitavailandreserve_start:
	return;
	//while(_spiusart0_ongoing);
}

/******************************************************************************
	function: spiusart0_rw
*******************************************************************************	
	Select peripheral, read/write 1 byte, deselect peripheral.
	
	Waits until the SPI interface is available before initiating the transfer,
	and waits until completion of the transfer before returning.	
	
	Do not call in an interrupt routine as this can lead to a deadlock.
	
	Parameters:
		data	-	Data to send to peripheral
	
	Returns:
		Data read from peripheral
******************************************************************************/
unsigned char spiusart0_rw(unsigned char data)
{
	// Wait for end of interrupt transfer if any transfer is ongoing
	_spiusart0_waitavailandreserve();

	SPIUSART0_SELECT;

	// Put data into buffer, sends the data
	UDR0 = data;
	
	// Wait for data to be received
	while ( !(UCSR0A & 0b10000000) );
	
	SPIUSART0_DESELECT;

	// Get the data before indicating the peripheral is available
	data = UDR0;
	_spiusart0_ongoing=0;
	return data;			
}

/******************************************************************************
	function: spiusart0_rwn
*******************************************************************************	
	Read/write n bytes in the provided buffer. 
	
	Waits until the SPI interface is available before initiating the transfer,
	and waits until completion of the transfer before returning.			
	
	Do not call in an interrupt routine as this can lead to a deadlock.
******************************************************************************/
void spiusart0_rwn(unsigned char *ptr,unsigned char n)
{
	// Wait for end of interrupt transfer if any transfer is ongoing
	_spiusart0_waitavailandreserve();
	
	SPIUSART0_SELECT;
	
	while(n!=0)
	{
		// Put data into buffer, sends the data
		UDR0 = *ptr;
		
		// Wait for data to be received
		while ( !(UCSR0A & 0b10000000) );
		
		*ptr=UDR0;
		ptr++;
		n--;
	}
	
	SPIUSART0_DESELECT;
	
	_spiusart0_ongoing=0;

	return;				// To avoid compiler complaining
}


/******************************************************************************
	function: spiusart0_rwn_int
*******************************************************************************	
	Read/write n bytes in the provided buffer using interrupts. 
	
	Waits until the SPI interface is available before initiating the transfer,
	and waits until completion of the transfer before returning.					
	
	Do not call in an interrupt routine as this can lead to a deadlock.
******************************************************************************/
void spiusart0_rwn_int(unsigned char *ptr,unsigned char n)
{
	// Wait for end of interrupt transfer if any transfer is ongoing
	_spiusart0_waitavailandreserve();
		
	_spiusart0_txint=0;
	_spiusart0_bufferptr = ptr;
	_spiusart0_n=n;
	_spiusart0_ongoing=1;
	_spiusart0_callback=0;
	
	//printf_P(PSTR("before int _spiusart0_txint %d\n"),_spiusart0_txint);
	//printf_P(PSTR("SPI USART: A: %02X B: %02X C: %02X\n"),UCSR0A,UCSR0B,UCSR0C);
	
	// manually clear	
	UCSR0A = 0b01000000;	// Clear TXCn before enabling interrupt, otherwise it fires immediately
	
	//printf_P(PSTR("SPI USART after clear	: A: %02X B: %02X C: %02X\n"),UCSR0A,UCSR0B,UCSR0C);
	//printf_P(PSTR("buffer ptr: %p\n"),_spiusart0_bufferptr);
	
	
	// Activate interrupts
	UCSR0B |= 0b01000000;
	
	
	
	//printf_P(PSTR("after int 1 _spiusart0_txint %d\n"),_spiusart0_txint);
	//_delay_ms(500);
	//printf_P(PSTR("after int 2 _spiusart0_txint %d\n"),_spiusart0_txint);
	//_delay_ms(500);
	
	// Start the transfer
	SPIUSART0_SELECT;
	//printf_P(PSTR("SPI USART after enable int: A: %02X B: %02X C: %02X PORTA: %02X\n"),UCSR0A,UCSR0B,UCSR0C,PORTA);
		
	UDR0 = *ptr;
	
	// Wait completion
	while(_spiusart0_ongoing);
	//_delay_ms(10);
	//printf("done\n");
	
	//UCSR0B &= 0b10111111; // not needed, interrupt does this
	//SPIUSART0_DESELECT;// not needed, interrupt does this
	
	//printf_P(PSTR("SPI USART after end of trans: A: %02X B: %02X C: %02X. PORTA: %02X\n"),UCSR0A,UCSR0B,UCSR0C,PORTA);
	
	//printf_P(PSTR("_spiusart0_txint %d\n"),_spiusart0_txint);
	//_delay_ms(100);
	//printf_P(PSTR("_spiusart0_txint %d\n"),_spiusart0_txint);
	//_delay_ms(100);
	//printf_P(PSTR("_spiusart0_txint %d\n"),_spiusart0_txint);

}
/******************************************************************************
	spiusart0_rwn_int_cb
*******************************************************************************	
	Read/write n bytes in the provided buffer using interrupts. 
	
	Does return immediately if the SPI interface is busy. A callback is called 
	upon completion of the transfer.
	
	This function can be called in an interrupt safely.
	
	Return:
			0:		Transaction initiated
			1:		Error initiating transaction
******************************************************************************/
unsigned char spiusart0_rwn_int_cb(unsigned char *ptr,unsigned char n,void (*cb)(void))
{
	//printf_P(PSTR("before int _spiusart0_txint %d\n"),_spiusart0_txint);
	
	// Wait for end of interrupt transfer if any
	//printf("spiusart0_rwn_int_cb: ongoing: %d\n",_spiusart0_ongoing);
	if(_spiusart0_ongoing)
		return 1;

	_spiusart0_txint=0;
	_spiusart0_bufferptr = ptr;
	_spiusart0_n=n;
	_spiusart0_ongoing=1;
	_spiusart0_callback=cb;
	//_spiusart0_callback=dummycb;
	//_spiusart0_callback=cb;
	
	//printf_P(PSTR("before int _spiusart0_txint %d\n"),_spiusart0_txint);
	//printf_P(PSTR("SPI USART: A: %02X B: %02X C: %02X\n"),UCSR0A,UCSR0B,UCSR0C);
	
	// manually clear	
	UCSR0A = 0b01000000;	// Clear TXCn before enabling interrupt, otherwise it fires immediately
	
	//printf_P(PSTR("SPI USART after clear	: A: %02X B: %02X C: %02X\n"),UCSR0A,UCSR0B,UCSR0C);
	//printf_P(PSTR("buffer ptr: %p\n"),_spiusart0_bufferptr);
	
	
	// Activate interrupts
	UCSR0B |= 0b01000000;
	
	
	
	//printf_P(PSTR("after int 1 _spiusart0_txint %d\n"),_spiusart0_txint);
	//_delay_ms(500);
	//printf_P(PSTR("after int 2 _spiusart0_txint %d\n"),_spiusart0_txint);
	//_delay_ms(500);
	
	// Start the transfer
	SPIUSART0_SELECT;
	//printf_P(PSTR("SPI USART after enable int: A: %02X B: %02X C: %02X PORTA: %02X\n"),UCSR0A,UCSR0B,UCSR0C,PORTA);
		
	UDR0 = *ptr;
	
	// Wait completion
	//while(_spiusart0_ongoing);
	//_delay_ms(10);
	//printf("done\n");
	
	//UCSR0B &= 0b10111111; // not needed, interrupt does this
	//SPIUSART0_DESELECT;// not needed, interrupt does this
	
	//printf_P(PSTR("SPI USART after end of trans: A: %02X B: %02X C: %02X. PORTA: %02X\n"),UCSR0A,UCSR0B,UCSR0C,PORTA);
	
	//printf_P(PSTR("_spiusart0_txint %d\n"),_spiusart0_txint);
	//_delay_ms(100);
	//printf_P(PSTR("_spiusart0_txint %d\n"),_spiusart0_txint);
	//_delay_ms(100);
	//printf_P(PSTR("_spiusart0_txint %d\n"),_spiusart0_txint);
	return 0;
}

/******************************************************************************
	spiusart0_isongoing
*******************************************************************************	
	Indicates if an interrupt-driven transaction is ongoing		
******************************************************************************/
/*unsigned char spiusart0_isongoing(void)
{
	return _spiusart0_ongoing;
}*/


