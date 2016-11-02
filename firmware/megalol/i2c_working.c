/*
   MEGALOL - ATmega LOw level Library
	I2C Module
   Copyright (C) 2010:
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
#include <stdarg.h>
#include "serial.h"
//#include "global.h"
//#include "helper.h"
#include "main.h"
//#include "test.h"
//#include "wait.h"
#include "i2c.h"
#include "adc.h"


// Internal stuff
I2C_TRANSACTION *i2c_transaction_buffer[I2C_TRANSACTION_MAX];										// Not volatile, only modified in usermode
unsigned char _i2c_transaction_buffer_wr, _i2c_transaction_buffer_rd;						// rd modified by int, wr modified by int if queue called from int, but user function using these deactivate interrupts
volatile unsigned char _i2c_transaction_idle=1;																	// Indicates whether a transaction is in progress or idle. Modified by interrupt; read in queue to detect end of transaction
I2C_TRANSACTION *_i2c_current_transaction;																			// Current transaction for the interrupt routine.
unsigned char _i2c_current_transaction_n;																				// Modified by interrupt, but only used in interrupt
unsigned char _i2c_current_transaction_state;																		// Modified by interrupt, but only used in interrupt




// An I2C bus can also be reset with the magical sequence explained in multiple places : start, clock 9 bits, start, stop.

/******************************************************************************
*******************************************************************************
DIRECT ACCESS   DIRECT ACCESS   DIRECT ACCESS   DIRECT ACCESS   DIRECT ACCESS   
*******************************************************************************
******************************************************************************/


void i2c_init(void)
{
	i2c_transaction_init();

	TWCR = 0;					// Deactivate
	//TWBR = 52; TWSR = 0;	// 100KHz @ 12 MHz
	//TWBR = 52; TWSR = 2;	// ~7KHz @ 12 MHz
	
	//TWBR = 7; TWSR = 0;	// 400KHz @ 12 MHz
	TWBR = 1; TWSR = 0;	// 409.6KHz @ 7'372'800 Hz
	//TWBR = 0; TWSR = 0;		// 460.8KHz @ 7'372'800 Hz
	//TWBR = 1; TWSR = 3;	// 409.6KHz/64=6KHz @ 7'372'800 Hz
	//TWBR = 0; TWSR = 0;	// 460.8KHz @ 7'372'800 Hz



//	TWCR = (1<<TWEN);			// Enable TWI

}

/*
	Starts a write by:
		- Sending a start condition
		- Sending the slave address
		- Returning the state of the acknowledge
	
	Upon success, the applicaton shall write data using i2c_writedata

	Return values:
		0x08:		Start transmitted
		0x10:		Repeat start transmitted
		0x18:		SLA+W transmitted, ack received
		0x20:		SLA+W transmitted, ack not received
		0x38:		data  transmitted, ack received
		0x30:		data  transmitted, ack not received
		0x38:		arbitration lost in SLA+W or data bytes
		0x00:		successful completion (start sent, SLA+W sent and ack received)
*/
unsigned char i2c_writestart(unsigned addr7,unsigned char debug)
{
	unsigned char twcr,twsr,twdr;

	// Transmit a start condition
	if(debug)
		printf_P(PSTR("        I2C START\r"));
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);						
	do
	{
		twcr = TWCR;
		twsr = TWSR;
		twdr = TWDR;
		if(debug)
			printf_P(PSTR("                TWCR: %X TWSR: %X TWDR: %X\r"),twcr,twsr,twdr);
	}
	while (!(twcr & (1<<TWINT)));										// Wait until TWINT is set
	twsr &= 0xF8;
	if((twsr!=0x08) && (twsr!=0x10))
	{
		if(debug)
			printf_P(PSTR("        I2C START error (%X)\r"),twsr);
		return twsr;
	}
	if(debug)
		printf_P(PSTR("        I2C START ok (%X)\r"),twsr);

	// Transmit the address
	if(debug)
		printf_P(PSTR("        I2C SLA+W\r"));
	TWDR = addr7<<1;														// I2C 7-bit address + write
	TWCR = (1<<TWINT) | (1<<TWEN);								// Send
	do
	{
		twcr = TWCR;
		twsr = TWSR;
		twdr = TWDR;
		if(debug)
			printf_P(PSTR("                TWCR: %X TWSR: %X TWDR: %X\r"),twcr,twsr,twdr);
	}
	while (!(twcr & (1<<TWINT)));										// Wait until TWINT is set
	twsr &= 0xF8;

	if(twsr != 0x18)
	{
		if(debug)
			printf_P(PSTR("        I2C address send error (%X)\r"),twsr);
		return twsr;
	}
	if(debug)
		printf_P(PSTR("        I2C address send ok (%X)\r"),twsr);

	return 0;
}
/*
	i2c_writedata
	Assumes a i2c_startwrite was successfully called, i.e. a slave has acknowledged being addressed and is waiting for data.

	Return values:
		See i2c_writestart

*/
unsigned char i2c_writedata(unsigned char data,unsigned char debug)
{
	unsigned char twcr,twsr,twdr;	

	if(debug)
		printf_P(PSTR("        I2C DATA Write\r"));
	
	TWDR = data;
	TWCR = (1<<TWINT) | (1<<TWEN);
	do
	{
		twcr = TWCR;
		twsr = TWSR;
		twdr = TWDR;
		if(debug)
			printf_P(PSTR("                TWCR: %X TWSR: %X TWDR: %X\r"),twcr,twsr,twdr);
	}
	while (!(twcr & (1<<TWINT)));										// Wait until TWINT is set
	twsr &= 0xF8;

	if(twsr != 0x28)
	{
		if(debug)
			printf_P(PSTR("        I2C DATA Write error (%X)\r"),twsr);
		return twsr;
	}
	if(debug)
		printf_P(PSTR("        I2C DATA Write ok (%X)\r"),twsr);
	return 0;
}
/*

	Return values:
		0x08:		Start transmitted
		0x10:		Repeat start transmitted
		0x38:		Arbitration lost in SLA+R or not acknowledge
		0x40:		SLA+R transmitted, ack received
		0x48:		SLA+R transmitted, not ack received
		0x50:		data byte received, returned ack
		0x58: 	data byte received, returned not ack
*/
unsigned char i2c_readstart(unsigned addr7,unsigned char debug)
{
	unsigned char twcr,twsr,twdr;

	// Transmit a start condition
	if(debug)
		printf_P(PSTR("        I2C SLA+R START\r"));
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);						
	do
	{
		twcr = TWCR;
		twsr = TWSR;
		twdr = TWDR;
		if(debug)
			printf_P(PSTR("                TWCR: %X TWSR: %X TWDR: %X\r"),twcr,twsr,twdr);
	}
	while (!(twcr & (1<<TWINT)));										// Wait until TWINT is set
	twsr &= 0xF8;
	if((twsr!=0x08) && (twsr!=0x10))		// Start and repeat start are okay
	{
		if(debug)
			printf_P(PSTR("        I2C SLA+R START error (%X)\r"),twsr);
		return twsr;
	}
	if(debug)
		printf_P(PSTR("        I2C SLA+R START ok (%X)\r"),twsr);

	// Transmit the address
	if(debug)
		printf_P(PSTR("        I2C SLA+R\r"));
	TWDR = (addr7<<1)+1;														// I2C 7-bit address + read
	TWCR = (1<<TWINT) | (1<<TWEN);						// Send
	do
	{
		twcr = TWCR;
		twsr = TWSR;
		twdr = TWDR;
		if(debug)
			printf_P(PSTR("                TWCR: %X TWSR: %X TWDR: %X\r"),twcr,twsr,twdr);
	}
	while (!(twcr & (1<<TWINT)));										// Wait until TWINT is set
	twsr &= 0xF8;

	if(twsr != 0x40)
	{
		if(debug)
			printf_P(PSTR("        I2C SLA+R error (%X)\r"),twsr);
		return twsr;
	}
	if(debug)
		printf_P(PSTR("        I2C SLA+R ok (%X)\r"),twsr);

	return 0;
}
/*
	i2c_readdata
	Assumes a i2c_startread was successfully called, i.e. a slave has acknowledged being addressed and is ready to send data.

	Return values:
		See i2c_readstart

*/
unsigned char i2c_readdata(unsigned char *data,unsigned char acknowledge,unsigned char debug)
{
	unsigned char twcr,twsr,twdr;	

	if(debug)
		printf_P(PSTR("        I2C DATA Read\r"));
	
	if(acknowledge)
		TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	else
		TWCR = (1<<TWINT) | (1<<TWEN);
	do
	{
		twcr = TWCR;
		twsr = TWSR;
		twdr = TWDR;
		if(debug)
			printf_P(PSTR("                TWCR: %X TWSR: %X TWDR: %X\r"),twcr,twsr,twdr);
	}
	while (!(twcr & (1<<TWINT)));										// Wait until TWINT is set
	twsr &= 0xF8;

	if((acknowledge && twsr != 0x50) | (acknowledge==0 && twsr != 0x58))
	{
		if(debug)
			printf_P(PSTR("        I2C DATA Read error (%X)\r"),twsr);
		return twsr;
	}
	if(debug)
		printf_P(PSTR("        I2C DATA Read ok (%X)\r"),twsr);
	*data = twdr;
	return 0;
}

void i2c_stop(unsigned char debug)
{
	unsigned char twcr,twsr,twdr;	
	twcr = TWCR;
	twsr = TWSR;
	twdr = TWDR;
	if(debug)
		printf_P(PSTR("        I2C STOP:                TWCR: %X TWSR: %X TWDR: %X\r"),twcr,twsr,twdr);
	TWCR = (1<<TWINT)|(1<<TWEN)| (1<<TWSTO);		// Transmit stop
	// Unclear from datasheed if we have to wait for stop or if it is effective immediately
	// Some online resources indicate that we can wait until TWSTO is cleared. 
	// Experience shows in some cases TWSTO is not cleared.
	
	/*
	do
	{
		twcr = TWCR;
		twsr = TWSR;
		twdr = TWDR;
		if(debug)
			printf_P(PSTR("        I2C STOP:                TWCR: %X TWSR: %X TWDR: %X\r"),twcr,twsr,twdr);
	}
	while(twcr&(1<<TWSTO));							// Wait until the stop condition is transmitted (bit is cleared upon transmission)
	*/
}

/*
	Scan the entire bus and returns a 128 bit bitmap in h,l indicating if a peripheral exists (1) or not (0), with the MSB of h the peripheral 127 and the LSB of l the peripheral 0
*/
void i2c_scan(unsigned long long *h,unsigned long long *l)
{
	*h=0;
	*l=0;
	for(signed char i=127;i>=0;i--)
	{
		TWCR = 0;				// We purposefully disconnect the I2C, as scanning sometimes leads to bus hang. This assumes the IO pins are set as output driving high.
		*h<<=1;
		*h|=(*l>>63);
		*l<<=1;
		int rv = i2c_readstart(i,0);
		if(rv==0)
			*l|=1;
		//i2c_stop(1);
	}
}

/*
	General purpose read I2C register, for most I2C devices.
	Operation: writes the register of interest, and performs a single byte read
	
	Return value:	zero for success
								nonzero for error (see i2c_readstart and AVR datasheet for info)
*/
unsigned char i2c_readreg(unsigned char addr7,unsigned char reg,unsigned char *val)
{
	unsigned char r;
	r=i2c_writestart(addr7,0);			// Slave write
	if(r!=0x00)
			return r;
	r=i2c_writedata(reg,0);				// Register of interest
	if(r!=0x00)
			return r;
	r=i2c_readstart(addr7,0);		// Slave read
	if(r!=0x00)
			return r;
	r=i2c_readdata(val,0,0);				// Read with NAK
	if(r!=0x00)
			return r;
	i2c_stop(0);										// Stop
	return 0x00;
}
/*
	General purpose read several I2C registers, for most I2C devices.
	Operation: writes the register of interest, and performs a multiple byte read
	
	Return value:	zero for success
								nonzero for error (see i2c_readstart and AVR datasheet for info)
*/
unsigned char i2c_readregs(unsigned char addr7,unsigned char reg,unsigned char n,unsigned char *val)
{
	unsigned char r;
	r=i2c_writestart(addr7,0);			// Slave write
	if(r!=0x00)
			return r;
	r=i2c_writedata(reg,0);				// Register of interest
	if(r!=0x00)
			return r;
	r=i2c_readstart(addr7,0);		// Slave read
	if(r!=0x00)
			return r;
	for(unsigned char i=0;i<n;i++)
	{
		r=i2c_readdata(&val[i],i==n-1 ? 0:1,0);				// Acknowledge with ACK except last read with NACK
		if(r!=0x00)
				return r;
	}
	i2c_stop(0);										// Stop
	return 0x00;
}

/*
	General purpose write an I2C register, for most I2C devices.
	Operation: writes the register of interest, writes the register address, writes the value
	
	Return value:	zero for success
								nonzero for error (see i2c_readstart and AVR datasheet for info)
*/
unsigned char i2c_writereg(unsigned char addr7,unsigned char reg,unsigned char val)
{
	unsigned char r;
	r=i2c_writestart(addr7,0);
	if(r!=0x00)
			return r;
	i2c_writedata(reg,0);
	if(r!=0x00)
			return r;
	i2c_writedata(val,0);
	if(r!=0x00)
			return r;
	i2c_stop(0);
	return 0;
}

void i2c_check(void)
{
	unsigned long long i2cbmh,i2cbml;
	printf("Scanning I2C bus. Peripherals at address: ");
	i2c_scan(&i2cbmh,&i2cbml);
	for(int i=0;i<128;i++)
	{
		if(i2cbml&1)
			printf("%d ",i);
		i2cbml>>=1;
		i2cbml|=(i2cbmh&1)<<63;
		i2cbmh>>=1;
	}
	printf("\n");
}



/******************************************************************************
*******************************************************************************
INTERRUPT-DRIVEN ACCESS   INTERRUPT-DRIVEN ACCESS   INTERRUPT-DRIVEN ACCESS   
*******************************************************************************
******************************************************************************/

/*unsigned char hex2chr(unsigned char v)
{
	if(v>9)
		return 'A'+v-10;
	return '0'+v;
}*/


/******************************************************************************
Interrupt vector
******************************************************************************/

//#define I2CDBG
void TWI_vect_int(void);
void TWI_vect_int2(void);
//ISR(TWI_vect)
//{
	
	//_delay_ms(100);	// Force failure
	/*unsigned char sreg = SREG;
	uart0_putchar('s');	
	uart0_putchar(hex2chr(sreg>>4));
	uart0_putchar(hex2chr(sreg&0xf));*/
	
	
	
//	TWI_vect_int();
	//TWI_vect_int2();
	
	/*sreg = SREG;
	uart0_putchar('S');	
	uart0_putchar(hex2chr(sreg>>4));
	uart0_putchar(hex2chr(sreg&0xf));*/
	
	//_delay_ms(100);	// Force failure
//}
// Test interrupt

void TWI_vect_int2(void)
{
	static unsigned char ___ctr=0;	
	uart0_putchar('i');
	uart0_putchar(hex2chr(___ctr));
	uart0_putchar(' ');
	unsigned char twsr = TWSR&0xf8;
	uart0_putchar(hex2chr(twsr>>4));
	uart0_putchar(hex2chr(twsr&0xf));
	
	if(___ctr==0)
	{
		TWDR = ((105+0x10*0)<<1)+1;		// +1: read, +0: write
		TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWIE);
	}
	if(___ctr==1)
	{
		TWDR = 0x00;
		TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWIE);
	}
	if(___ctr==2)
	{
		TWDR = 0x00;
		TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWIE);
	}
	if(___ctr==3)
	{
		// Stop 
		//TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN)|(1<<TWIE);
		// Start
		TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN)|(1<<TWIE);
	}
	___ctr++;
	uart0_putchar('I');
	uart0_putchar(hex2chr(___ctr));
	if(___ctr==4)
		___ctr=0;
	uart0_putchar(hex2chr(___ctr));
	
}
ISR(TWI_vect)
{
	TWI_vect_intx();
}
void TWI_vect_intx(void)
{
	TWI_vect_start:
	
	#ifdef I2CDBG
	uart0_putchar('i');
	uart0_putchar(hex2chr(((unsigned)_i2c_current_transaction>>12)&0x0f));
	uart0_putchar(hex2chr(((unsigned)_i2c_current_transaction>>8)&0x0f));
	uart0_putchar(hex2chr(((unsigned)_i2c_current_transaction>>4)&0x0f));
	uart0_putchar(hex2chr(((unsigned)_i2c_current_transaction>>0)&0x0f));
	uart0_putchar(' ');
	uart0_putchar(hex2chr(_i2c_transaction_idle));
	uart0_putchar('\r');
	#endif
	if(_i2c_transaction_idle==1)
	{
		// Idle: check if next transaction to queue
		_i2c_current_transaction = _i2c_transaction_getnext();	// Current transaction
		#ifdef I2CDBG
		uart0_putchar('D');
		uart0_putchar(hex2chr(((unsigned)_i2c_current_transaction>>12)&0x0f));
		uart0_putchar(hex2chr(((unsigned)_i2c_current_transaction>>8)&0x0f));
		uart0_putchar(hex2chr(((unsigned)_i2c_current_transaction>>4)&0x0f));
		uart0_putchar(hex2chr(((unsigned)_i2c_current_transaction>>0)&0x0f));
		uart0_putchar('\r');
		#endif
		if(_i2c_current_transaction==0)
			return;																								// No next transaction
		// Initialize internal stuff
		_i2c_current_transaction_n = 0;												// Number of data bytes transferred so far
		_i2c_current_transaction_state = 0;										// State of the transaction
		_i2c_transaction_idle=0;																// Non idle	
		
	}
	
	TWI_vect_int();
	
	if(_i2c_transaction_idle==1)
		goto TWI_vect_start;
}
void TWI_vect_int(void)
{
	unsigned char twsr;
	//PORTD ^=  _BV(PORT_LED1);												// Toggle - signal some interrupt is active.


	twsr = TWSR;																// Get the I2C status
	twsr &= 0xF8;

	/*#ifdef I2CDBG
		uart0_putchar('I');
		uart0_putchar(hex2chr(_i2c_current_transaction_state));
		uart0_putchar(' ');
		uart0_putchar(hex2chr(_i2c_current_transaction_n));
		uart0_putchar(' ');
		uart0_putchar('S');
		uart0_putchar(hex2chr((TWSR>>4)));
		uart0_putchar(hex2chr((TWSR&0xf)));
		uart0_putchar(' ');
		uart0_putchar('C');
		uart0_putchar(hex2chr((TWCR>>4)));
		uart0_putchar(hex2chr((TWCR&0xf)));
		uart0_putchar(' ');
		uart0_putchar('D');
		uart0_putchar(hex2chr((TWDR>>4)));
		uart0_putchar(hex2chr((TWDR&0xf)));
		uart0_putchar('\r');
	#endif	
*/
	
	/*if(_i2c_transaction_idle)												
	{
		TWCR&=~(1<<TWIE);											// Deactivate interrupt
		return;
	}*/

	
	


	// Process the state machine.
	switch(_i2c_current_transaction_state)							
	{
		// ------------------------------------------------------
		// State 0:	Always called when executing the transaction.
		// 			Setup interrupt and TWI enable
		// 			Generate the start condition if needed. Otherwise, go to next state.
		// ------------------------------------------------------
		case 0:	
			#ifdef I2CDBG	
			uart0_putchar('X');
			uart0_putchar('0');
			uart0_putchar('\r');
			#endif

			_i2c_transaction_idle = 0;													// Mark as non-idle
			_i2c_current_transaction_state++;									// Set next state
			
			if(_i2c_current_transaction->dostart)								// Start must be generated
			{
				
				TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN)|(1<<TWIE);	// Generate the start condition.
				return;																						// Return - will be called back upon completion.
			}
			
		// Start must not be generated: flows to next state (no break)

		// ------------------------------------------------------
		// State 1:	Address state
		//				Process the return value of the start condition (if executed)
		//				Sends the address (if needed)
		// ------------------------------------------------------
		case 1:				
			#ifdef I2CDBG													
			uart0_putchar('X');
			uart0_putchar('1');
			uart0_putchar('\r');
			#endif
	
			// If we previously executed a start, check the return value
			if(_i2c_current_transaction->dostart)
			{
				if((twsr!=0x08) && (twsr!=0x10))								// Errror code isn't start and repeat start -> error
				{
					// Error. 
					_i2c_transaction_idle=1;											// Return to idle
					_i2c_current_transaction->status = 1;					// Indicate failure in start condition
					_i2c_current_transaction->i2cerror = twsr;		// Indicate I2C failure code

					TWCR&=~(1<<TWIE);															// Deactivate interrupt

					if(_i2c_current_transaction->callback)
						_i2c_current_transaction->callback(_i2c_current_transaction);
					return;
				}
			}

			_i2c_current_transaction_state++;							// Set next state

			if(_i2c_current_transaction->doaddress)					// Address must be sent
			{
				
				TWDR = (_i2c_current_transaction->address<<1)+_i2c_current_transaction->rw;	// I2C 7-bit address + r/w#
				#ifdef I2CDBG
				uart0_putchar('D');uart0_putchar('A');uart0_putchar(hex2chr(TWDR>>4));uart0_putchar(hex2chr(TWDR&0x0f));uart0_putchar('\r');
				#endif
				TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWIE);					// Send address
				return;
			}
			// Address must not be sent: flows to the next state (no break)


		// ------------------------------------------------------
		// State 2:	Address return state
		//				Process the return value of the address condition (if executed).
		// ------------------------------------------------------
		case 2:		
			#ifdef I2CDBG
			uart0_putchar('X');
			uart0_putchar('2');
			uart0_putchar('\r');					
			#endif
			if(_i2c_current_transaction->doaddress)						// Check the error code of address sent
			{
				if((_i2c_current_transaction->rw==I2C_WRITE && twsr!=0x18) 
					|| (_i2c_current_transaction->rw==I2C_READ && twsr!=0x40))	// Errror code isn't an acknowledge -> error
				{
					// Error.
					_i2c_transaction_idle=1;											// Return to idle
					_i2c_current_transaction->status = 2;					// Indicate failure in address
					_i2c_current_transaction->i2cerror = twsr;		// Indicate I2C failure code

					TWCR&=~(1<<TWIE);															// Deactivate interrupt

					if(_i2c_current_transaction->callback)
						_i2c_current_transaction->callback(_i2c_current_transaction);

					return;
				}
			}
			_i2c_current_transaction_state=4;								// Set next state - state 4: data transfer
			_i2c_current_transaction_n=0;										// Setups the number of bytes currently transferred

			// Go to the next state
			goto case4;
			break;

		// ------------------------------------------------------
		// State 3:	Check data response state. This state is entered multiple times when multiple bytes must be transferred. 
		//				Checks the return value of the data transfer
		// ------------------------------------------------------
		case 3:
			#ifdef I2CDBG
			uart0_putchar('X');
			uart0_putchar('3');
			uart0_putchar('\r');
			#endif
			
			// Check the answer to the last byte transfer	
			if( (_i2c_current_transaction->rw==I2C_WRITE && twsr!=0x28) 
					|| (_i2c_current_transaction->rw==I2C_READ && twsr!=0x50 && twsr!=0x58))
			{
				// Error.
				_i2c_transaction_idle=1;																												// Return to idle
				_i2c_current_transaction->status = 3 | ((_i2c_current_transaction_n-1)<<4);		// Indicate failure in data, indicates number of bytes sent before error
				_i2c_current_transaction->i2cerror = twsr;																			// Indicate I2C failure code

				TWCR&=~(1<<TWIE);																																// Deactivate interrupt

				if(_i2c_current_transaction->callback)
					_i2c_current_transaction->callback(_i2c_current_transaction);

				return;
			}
			// If read mode, then read the data
			if(_i2c_current_transaction->rw==I2C_READ)
			{
				_i2c_current_transaction->data[_i2c_current_transaction_n-1]=TWDR;
			}

			
			// No error, flow to next state and transmit next byte

		
		// ------------------------------------------------------
		// State 4:	Transmit data state. This state is entered multiple times when multiple bytes must be transferred. 
		//				Transmits data (if any)
		// ------------------------------------------------------
		
		case 4:
		case4:
			#ifdef I2CDBG
			uart0_putchar('X');
			uart0_putchar('4');
			uart0_putchar('\r');
			#endif
			if(_i2c_current_transaction_n<_i2c_current_transaction->dodata)			// If data to transmit data
			{
				_i2c_current_transaction_n++;																			// Indicate one data more procesed
				_i2c_current_transaction_state=3;																	// Set next state - state 3: check data transfer answer

				// Write mode
				if(_i2c_current_transaction->rw==I2C_WRITE)													
				{
					TWDR = _i2c_current_transaction->data[_i2c_current_transaction_n-1];
					TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWIE);										
					#ifdef I2CDBG
						//uart0_putchar('x');
						//uart0_putchar('w');
						//uart0_putchar('\r');
					#endif
					return;
				}
			
				// Read mode
				/*if(_i2c_current_transaction->dataack[_i2c_current_transaction_n-1])	
					TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWIE)|(1<<TWEA);							// Set acknowledge bit
				else
					TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWIE);												// Clear acknowledge bit*/
					
				if(_i2c_current_transaction_n == _i2c_current_transaction->dodata)	// Last data
					TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWIE);												// Clear acknowledge bit
				else
					TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWIE)|(1<<TWEA);							// Set acknowledge bits
					
				#ifdef I2CDBG
					//uart0_putchar('x');
					//uart0_putchar('r');
					//uart0_putchar('\r');
				#endif
				return;
			}
				
			// All bytes transferred, or no bytes to transfer... next state.
			_i2c_current_transaction_state++;														// Set next state
			// Flow to next state (no break)

		// ------------------------------------------------------
		// State 5:	Stop state
		//				Transmits stop condition
		// ------------------------------------------------------
		case 5:
		default:
			#ifdef I2CDBG
			uart0_putchar('X');
			uart0_putchar('5');
			uart0_putchar('\r');
			#endif
			if(_i2c_current_transaction->dostop)
				TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);											// Transmit stop, no interrupt

			// Do not wait for any answer... we've completed successfully the task
			_i2c_transaction_idle=1;																			// Return to idle
			_i2c_current_transaction->status = 0;													// Indicate success
			_i2c_current_transaction->i2cerror = 0;												// Indicate success

			if(_i2c_current_transaction->callback)
				_i2c_current_transaction->callback(_i2c_current_transaction);
	}
}


/******************************************************************************
	i2c_transaction_execute
*******************************************************************************	
	Execute the transaction.
	Two execution modes are possible: wait for completion/error and return, or call the callback upon completion/error
	In both case interrupts are used to execute the transaction

	Return value:
		0:			success
		1:			error
******************************************************************************/

unsigned char i2c_transaction_execute(I2C_TRANSACTION *trans,unsigned char blocking)
{
	return i2c_transaction_queue(1,blocking,trans);
}
/*unsigned char i2c_transaction_idle(void)
{
	return _i2c_transaction_idle;
}*/
void i2c_transaction_cancel(void)
{
	cli();
	TWCR&=~(1<<TWIE);											// Deactivate interrupt
	sei();
	// Resets the hardware - to take care of some challenging error situations
	TWCR=0;														// Deactivate i2c
	TWCR=(1<<TWEN);											// Activate i2c
	if(!_i2c_transaction_idle)								// Transaction wasn't idle -> call the callback in cancel mode
	{
		TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);		// Deactivate interrupt, generate a stop condition.
		_i2c_current_transaction->status=255;			// Mark transaction canceled
		if(_i2c_current_transaction->callback)			// Call callback if available
			_i2c_current_transaction->callback(_i2c_current_transaction);
		
	}
	_i2c_transaction_idle = 1;
	
}





//------------------------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------


void i2c_test_int0(void)
{
	/*printf("Enable TWI and TWI interrupt\n");
	_delay_ms(100);
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWIE);
	_delay_ms(1000);
	printf("Manually set TWINT\n");
	TWCR = TWCR|(1<<TWINT);
	_delay_ms(1000);
	*/
	printf("Enable TWI and TWI interrupt\n");
	_delay_ms(100);
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWIE);
	_delay_ms(1000);
	while(1)
	{
		printf("Generate a start condition\n");
		_delay_ms(100);
		TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN)|(1<<TWIE);
		_delay_ms(1000);
		
		printf("Generate a stop condition\n");
		_delay_ms(100);
		TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN)|(1<<TWIE);
		_delay_ms(1000);
		printf("End\n");
	}
	
}
/******************************************************************************
	_i2c_transaction_init
*******************************************************************************	
	Initialise the data structures for i2c transactions.
	Internally called by i2c_init
******************************************************************************/
void i2c_transaction_init(void)
{
	_i2c_transaction_buffer_wr=0;
	_i2c_transaction_buffer_rd=0;
	_i2c_current_transaction=0;
}
/******************************************************************************
	i2c_transaction_queue
*******************************************************************************	
	Enqueues the specified transactions for later execution. Execution is 
	scheduled immediately if no other transactions are in the queue, otherwise it 
	happens as soon as the queue is empty.
	
	n:				Number of transactions to enqueue
	blocking:	Whether to block until all enqueued transactions are completed
	...				variable number of pointers to I2C_TRANSACTION structures
	
		Return value:
		0:			success
		1:			error
******************************************************************************/
unsigned char i2c_transaction_queue(unsigned char n,unsigned char blocking,...)
{
	va_list args;	
	I2C_TRANSACTION *tlast;
	
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		// Check enough space to store n transactions
		if(_i2c_transaction_getfree()<n || n==0)
		{
			return 1;
		}
		
		va_start(args,blocking);
		
		for(unsigned char i=0;i<n;i++)
		{
			I2C_TRANSACTION *t = va_arg(args,I2C_TRANSACTION *);
			tlast=t;
			//printf("Transaction %d: %p\n",i,t);
			_i2c_transaction_put(t);
		}	
		//printf("State of transactions\n");
		//i2c_transaction_printall(file_usb);
		/*I2C_TRANSACTION *t = _i2c_transaction_findnext();
		printf("FindNext: %p\n",t);
		printf("Current: %p\n",_i2c_current_transaction);
		t=_i2c_transaction_findnext();
		printf("FindNext2: %p\n",t);
		t=_i2c_transaction_findnext();
		printf("FindNext3: %p\n",t);
		t=_i2c_transaction_getnext();
		printf("GetNext: %p\n",t);
		t=_i2c_transaction_getnext();
		printf("GetNext2: %p\n",t);
		t=_i2c_transaction_getnext();
		printf("GetNext3: %p\n",t);*/
		
		// If idle, call IV to execute next; if not idle, the IV will automatically proceed to next transaction
		if(_i2c_transaction_idle)
			TWI_vect_intx();
	}
	//printf("Wait\n");

	if(blocking)
		while(!_i2c_transaction_idle);

	return 0;
	
}

/******************************************************************************
	i2c_transaction_getqueued
*******************************************************************************	
	Returns the number of transaction queued
******************************************************************************/
unsigned char i2c_transaction_getqueued(void)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		return (_i2c_transaction_buffer_wr-_i2c_transaction_buffer_rd)&I2C_TRANSACTION_MASK;
	}
	return 0;	// To avoid compiler complaints
}


/******************************************************************************
	i2c_transaction_removenext
*******************************************************************************	
	Removes the next transaction, if any.
******************************************************************************/
void i2c_transaction_removenext(void)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		if(_i2c_transaction_buffer_rd!=_i2c_transaction_buffer_wr)
			_i2c_transaction_buffer_rd=(_i2c_transaction_buffer_rd+1)&I2C_TRANSACTION_MASK;
	}
}

/******************************************************************************
	_i2c_transaction_getfree
*******************************************************************************	
	Returns the number of available transaction slots
******************************************************************************/
unsigned char _i2c_transaction_getfree(void)
{
	return I2C_TRANSACTION_MAX-i2c_transaction_getqueued()-1;
}
/******************************************************************************
	_i2c_transaction_put
*******************************************************************************	
	Internal use. Ensure atomic use.
******************************************************************************/
void _i2c_transaction_put(I2C_TRANSACTION *trans)
{
	i2c_transaction_buffer[_i2c_transaction_buffer_wr]=trans;
	_i2c_transaction_buffer_wr=(_i2c_transaction_buffer_wr+1)&I2C_TRANSACTION_MASK;
}
/******************************************************************************
	_i2c_transaction_getnext
*******************************************************************************	
	Internal use. Ensure atomic use.
	
	Returns the next active transaction or zero; the transaction pointer is updated.
	
	Return value:
		0:				If no next transaction
		nonzero:	pointer to next transaction
******************************************************************************/
I2C_TRANSACTION *_i2c_transaction_getnext(void)
{
	/*while(_i2c_transaction_buffer_rd!=_i2c_transaction_buffer_wr)
	{
		if(i2c_transaction_buffer[_i2c_transaction_buffer_rd]->_active)
		{*/
			// Found an active transaction, increment/wrap read pointer, return transaction
			
			if(_i2c_transaction_buffer_rd!=_i2c_transaction_buffer_wr)
			{
			
				I2C_TRANSACTION *t=i2c_transaction_buffer[_i2c_transaction_buffer_rd];
				_i2c_transaction_buffer_rd=(_i2c_transaction_buffer_rd+1)&I2C_TRANSACTION_MASK;
				return t;
			}
			return 0;
		/*}
		// Transaction not active, 
		_i2c_transaction_buffer_rd=(_i2c_transaction_buffer_rd+1)&I2C_TRANSACTION_MASK;
	}
	// No transactions
	return 0;*/
}
/******************************************************************************
	_i2c_transaction_findnext
*******************************************************************************	
	Internal use. Ensure atomic use.
	
	Returns the next active transaction or zero; the transaction pointer is not updated.
		
	Return value:
		0:				If no next transaction
		nonzero:	pointer to next transaction
******************************************************************************/
/*I2C_TRANSACTION *_i2c_transaction_findnext(void)
{
	unsigned char rd = _i2c_transaction_buffer_rd;
	while(rd!=_i2c_transaction_buffer_wr)
	{
		if(i2c_transaction_buffer[rd]->_active)
		{
			// Found an active transaction, return transaction
			return i2c_transaction_buffer[rd];
		}
		// Transaction not active, 
		rd=(rd+1)&I2C_TRANSACTION_MASK;
	}
	// No transactions
	return 0;
}*/

void i2c_transaction_print(I2C_TRANSACTION *trans,FILE *file)
{
	fprintf_P(file,PSTR("I2C transaction %p\n"),trans);
	fprintf_P(file,PSTR("Address %02X rw %d\n"),trans->address,trans->rw);
	for(unsigned char i=0;i<16;i++)
		fprintf_P(file,PSTR("%02X "),trans->data[i]);
	fprintf_P(file,PSTR("\n"));
	for(unsigned char i=0;i<16;i++)
		fprintf_P(file,PSTR("%02X "),trans->dataack[i]);
	fprintf_P(file,PSTR("\n"));
	fprintf_P(file,PSTR("CB: %p User: %p\n"),trans->callback,trans->user);
	fprintf_P(file,PSTR("Status: %02X Error: %02X\n"),trans->status,trans->i2cerror);
	
}

void i2c_transaction_printall(FILE *file)
{
	fprintf_P(file,PSTR("I2C transaction buffer. rd: %02d wr: %02d\n"),_i2c_transaction_buffer_rd,_i2c_transaction_buffer_wr);
	for(unsigned char i=0;i<I2C_TRANSACTION_MAX;i++)
		fprintf_P(file,PSTR("%p "),i2c_transaction_buffer[i]);
	fprintf_P(file,PSTR("\n"));
}



/******************************************************************************
	i2c_readregs_int
*******************************************************************************	
	General purpose read several I2C registers, for most I2C devices.
	Operation: writes the register of interest, and performs a multiple byte read
	
	Interrupt-driven, blocking.
	
	Maximum transaction size limited by val; the internal buffer in I2C_TRANSACTION
	is bypassed
	
	Return value:	
		zero:			success
		nonzero:	error (see i2c_readstart and AVR datasheet for info)		

******************************************************************************/

unsigned char i2c_readregs_int(unsigned char addr7,unsigned char reg,unsigned char n,unsigned char *val)
{
	I2C_TRANSACTION t1,t2;
	// Register selection transaction
	t1.address=addr7;
	t1.rw=I2C_WRITE;												// write: 0. read: 1.
	t1.doaddress=1;													// Send the address (otherwise, only the rest is done)
	t1.dodata=1;														// Send/read the data (otherwise, only the rest is done): specifies the number of bytes, or 0.
	t1.dostart=1;														// Send I2C start 
	t1.dostop=0;														// Do not send I2C stop (repeat start on next transaction) to avoid relinquishing bus
	t1.data[0]=59;													// data to send/receive
	t1.callback=0;
	
	t2.address=addr7;
	t2.rw=I2C_READ;														// write: 0. read: 1.
	t2.doaddress=1;														// Send the address (otherwise, only the rest is done)
	t2.dostart=1;															// Send I2C start 
	t2.dostop=1;															// Send I2C stop 
	t2.dodata=n;															// Send/read the data (otherwise, only the rest is done): specifies the number of bytes, or 0.
	for(unsigned char i=0;i<n-1;i++) t2.dataack[i]=1; t2.dataack[n-1]=0; // NACK
	t2.callback=0;
	
	unsigned char r = i2c_transaction_queue(2,1,&t1,&t2);
	
	// Error
	if(r!=0)
		return 1;
	
	// Copy the data into val.
	
	
	
	return 0;	
}