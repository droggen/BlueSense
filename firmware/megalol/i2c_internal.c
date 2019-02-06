/*
   MEGALOL - ATmega LOw level Library
	I2C Module
   Copyright (C) 2010-2015:
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
#include "i2c.h"
#include "i2c_internal.h"

#include "serial1.h"
//#include "helper.h"

// Internal stuff
I2C_TRANSACTION *i2c_transaction_buffer[I2C_TRANSACTION_MAX];										// Not volatile, only modified in usermode
unsigned char _i2c_transaction_buffer_wr, _i2c_transaction_buffer_rd;						// rd modified by int, wr modified by int if queue called from int, but user function using these deactivate interrupts

// Transaction pool
I2C_TRANSACTION _i2c_transaction_pool[I2C_TRANSACTION_MAX];									
unsigned char _i2c_transaction_pool_alloc[I2C_TRANSACTION_MAX];
unsigned char _i2c_transaction_pool_wr, _i2c_transaction_pool_rd;
unsigned char _i2c_transaction_pool_reserved;

// Caching of current transaction parameters
volatile unsigned char _i2c_transaction_idle=1;																	// Indicates whether a transaction is in progress or idle. Modified by interrupt; read in queue to detect end of transaction
I2C_TRANSACTION *_i2c_current_transaction;																			// Current transaction for the interrupt routine.
unsigned char _i2c_current_transaction_n;																				// Modified by interrupt, but only used in interrupt
unsigned char _i2c_current_transaction_state;																		// Modified by interrupt, but only used in interrupt
unsigned char _i2c_current_transaction_dodata;																	// Modified by interrupt, but only used in interrupt
unsigned char _i2c_current_transaction_address;																	// Modified by interrupt, but only used in interrupt
unsigned char _i2c_current_transaction_rw;																			// Modified by interrupt, but only used in interrupt
I2C_CALLBACK  _i2c_current_transaction_callback;																// Modified by interrupt, but only used in interrupt
volatile unsigned char *_i2c_current_transaction_status,*_i2c_current_transaction_i2cerror;
unsigned char *_i2c_current_transaction_data;


volatile unsigned long int i2c_intctr=0;



/******************************************************************************
*******************************************************************************
INTERRUPT-DRIVEN ACCESS   INTERRUPT-DRIVEN ACCESS   INTERRUPT-DRIVEN ACCESS   
*******************************************************************************
******************************************************************************/

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
#if BOOTLOADER==0
unsigned char i2c_transaction_execute(I2C_TRANSACTION *trans,unsigned char blocking)
{
	return i2c_transaction_queue(1,blocking,trans);
}
#endif

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
/*
TODO
i2c_transaction_queue blocking is not working if interrupt routine pushes transactions, as idle may not be reached
(usually it will be reached, but it may wait longer than needed)
idea: "done" filed in i2c transaction, set when transaction completed.
transactio setup must clear this
wait until done by checking this field
*/
unsigned char i2c_transaction_queue(unsigned char n,unsigned char blocking,...)
{
	va_list args;	
//	I2C_TRANSACTION *tlast;
	
	// No transaction to queue: success.
	if(n==0)
		return 0;

		
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		// Check enough space to store n transactions
		if(_i2c_transaction_getfree()<n)
		{
			//printf("i2c No free\n");
			return 1;
		}
		
		va_start(args,blocking);
		
		for(unsigned char i=0;i<n;i++)
		{
			I2C_TRANSACTION *t = va_arg(args,I2C_TRANSACTION *);
			
			// Initialise link2next: links to next if not the last transaction to push
			if(i!=n-1)
				t->link2next=1;
			else
				t->link2next=0;
			
//			tlast=t;
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
	//printf("I2C wait\n");

	if(blocking)
		while(!_i2c_transaction_idle);

	return 0;
	
}


/*void i2c_transaction_cancel(void)
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
		*_i2c_current_transaction_status=255;			// Mark transaction canceled
		if(_i2c_current_transaction_callback)			// Call callback if available
			_i2c_current_transaction_callback(_i2c_current_transaction);
		
	}
	_i2c_transaction_idle = 1;
	
}
*/






/******************************************************************************
	i2c_transaction_setup
*******************************************************************************	
	Initialise a transaction structure with the specified parameters and sane defaults.
******************************************************************************/
void i2c_transaction_setup(I2C_TRANSACTION *t,unsigned char addr7,unsigned char mode,unsigned char stop,unsigned char n)
{
	t->address=addr7;
	t->rw=mode;
	t->extdata=0;
//	t->dostart=1;
	t->dostop=stop;
//	t->doaddress=1;
	t->dodata=n;
	t->callback=0;
	t->user=0;
	t->link2next=0;
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
	_i2c_transaction_pool_wr=0;
	_i2c_transaction_pool_rd=0;
	_i2c_transaction_pool_reserved=0;
	for(unsigned char i=0;i<I2C_TRANSACTION_MAX;i++)
		_i2c_transaction_pool_alloc[i]=0;
}

/*
void TWI_vect_int2(void)
{
	static unsigned char ___ctr=0;	
	//uart0_putchar('i');
	//uart0_putchar(hex2chr(___ctr));
	//uart0_putchar(' ');
	unsigned char twsr = TWSR&0xf8;
	//uart0_putchar(hex2chr(twsr>>4));
	//uart0_putchar(hex2chr(twsr&0xf));
	
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
	//uart0_putchar('I');
	//uart0_putchar(hex2chr(___ctr));
	if(___ctr==4)
		___ctr=0;
	//uart0_putchar(hex2chr(___ctr));
	
}*/
ISR(TWI_vect)
{
	i2c_intctr++;
	TWI_vect_intx();
}
void TWI_vect_intx(void)
{
	TWI_vect_start:
	
	#if I2CDBG==1
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
		#if I2CDBG==1
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
		_i2c_current_transaction_n = 0;													// Number of data bytes transferred so far
		_i2c_current_transaction_state = 0;											// State of the transaction
		_i2c_transaction_idle=0;																// Non idle	
		// Cache transaction to speedup access
		_i2c_current_transaction_dodata=_i2c_current_transaction->dodata;	
		_i2c_current_transaction_address=_i2c_current_transaction->address;
		_i2c_current_transaction_rw=_i2c_current_transaction->rw;
		_i2c_current_transaction_callback=_i2c_current_transaction->callback;
		_i2c_current_transaction_status=&_i2c_current_transaction->status;
		_i2c_current_transaction_i2cerror=&_i2c_current_transaction->i2cerror;
		if(!_i2c_current_transaction->extdata)
			_i2c_current_transaction_data=_i2c_current_transaction->data;
		else
				_i2c_current_transaction_data=_i2c_current_transaction->extdata;
		
		
	}
	
	TWI_vect_statemachine();
	
	if(_i2c_transaction_idle==1)
		goto TWI_vect_start;
}

/******************************************************************************
	TWI_vect_statemachine
*******************************************************************************	
	State machine executing one transaction including start, address, data, stop.
******************************************************************************/
inline void TWI_vect_statemachine(void)
{
	unsigned char twsr;
	//PORTD ^=  _BV(PORT_LED1);												// Toggle - signal some interrupt is active.


	twsr = TWSR;																// Get the I2C status
	twsr &= 0xF8;

	/*#if I2CDBG==1
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
		/*char b[64];
		b[0]='I';
		b[1]=hex2chr(_i2c_current_transaction_state);
		b[2]=' ';
		b[3]=hex2chr(_i2c_current_transaction_n);
		b[4]=' ';
		b[5]='S';
		b[6]=hex2chr((TWSR>>4));
		b[7]=hex2chr((TWSR&0xf));
		b[8]=' ';
		b[9]='C';
		b[10]=hex2chr((TWCR>>4));
		b[11]=hex2chr((TWCR&0xf));
		b[12]=' ';
		b[13]='D';
		b[14]=hex2chr((TWDR>>4));
		b[15]=hex2chr((TWDR&0xf));
		b[16]=' ';
		b[17]='a';
		b[18]=hex2chr(_i2c_current_transaction_address>>4);
		b[19]=hex2chr(_i2c_current_transaction_address&0xf);
		if(_i2c_current_transaction_rw)
			b[20]='r';
		else
			b[20]='w';
		b[21]=' ';
		b[22]='s';
		b[23]='0'+_i2c_current_transaction_state;
		b[24]=' ';
		b[25]='n';
		b[26]=hex2chr(_i2c_current_transaction_n>>4);
		b[27]=hex2chr(_i2c_current_transaction_n&0xf);
		b[28]=' ';
		b[29]='N';
		b[30]=hex2chr(_i2c_current_transaction_dodata>>4);
		b[31]=hex2chr(_i2c_current_transaction_dodata&0xf);
		b[32]='\r';
		
		uart1_fputbuf_int(b,33);*/
		
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
			#if I2CDBG==1	
			uart0_putchar('X');
			uart0_putchar('0');
			uart0_putchar('\r');
			#endif

			_i2c_transaction_idle = 0;													// Mark as non-idle
			_i2c_current_transaction_state++;									// Set next state
			
			TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN)|(1<<TWIE);	// Generate the start condition.
			return;																						// Return - will be called back upon completion.
			
		// Start must not be generated: flows to next state (no break)

		// ------------------------------------------------------
		// State 1:	Address state
		//				Process the return value of the start condition (if executed)
		//				Sends the address (if needed)
		// ------------------------------------------------------
		case 1:				
			#if I2CDBG==1													
			uart0_putchar('X');
			uart0_putchar('1');
			uart0_putchar('\r');
			#endif
	
			// If we previously executed a start, check the return value

			if((twsr!=0x08) && (twsr!=0x10))								// Error code isn't start and repeat start -> error
			{
				// Error. 
				_i2c_transaction_idle=1;											// Return to idle
				*_i2c_current_transaction_status = 1;					// Indicate failure in start condition
				*_i2c_current_transaction_i2cerror = twsr;		// Indicate I2C failure code
				
				//TWCR&=~(1<<TWIE);															// Deactivate interrupt
				//TWCR|=(1<<TWSTO);															// Dan: send stop
				TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);											// Transmit stop, no interrupt
				//TWCR=0;

				if(_i2c_current_transaction_callback)
					_i2c_current_transaction_callback(_i2c_current_transaction);
					
				// Remove next linked transactions
				_i2c_transaction_removelinked(_i2c_current_transaction);				
					
				return;
			}

			_i2c_current_transaction_state++;							// Set next state

			TWDR = (_i2c_current_transaction_address<<1)+_i2c_current_transaction_rw;	// I2C 7-bit address + r/w#
			#if I2CDBG==1
			uart0_putchar('D');uart0_putchar('A');uart0_putchar(hex2chr(TWDR>>4));uart0_putchar(hex2chr(TWDR&0x0f));uart0_putchar('\r');
			#endif
			TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWIE);					// Send address
			return;

			// Address must not be sent: flows to the next state (no break)


		// ------------------------------------------------------
		// State 2:	Address return state
		//				Process the return value of the address condition (if executed).
		// ------------------------------------------------------
		case 2:		
			#if I2CDBG==1
			uart0_putchar('X');
			uart0_putchar('2');
			uart0_putchar('\r');					
			#endif
			if((_i2c_current_transaction_rw==I2C_WRITE && twsr!=0x18) 
				|| (_i2c_current_transaction_rw==I2C_READ && twsr!=0x40))	// Errror code isn't an acknowledge -> error
			{
				// Error.
				_i2c_transaction_idle=1;											// Return to idle
				*_i2c_current_transaction_status = 2;					// Indicate failure in address
				*_i2c_current_transaction_i2cerror = twsr;		// Indicate I2C failure code

				//TWCR&=~(1<<TWIE);															// Deactivate interrupt
				//TWCR|=(1<<TWSTO);															// Dan: send stop
				TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);											// Transmit stop, no interrupt
				//TWCR=0;

				if(_i2c_current_transaction_callback)
					_i2c_current_transaction_callback(_i2c_current_transaction);
					
				// Remove next linked transactions
				_i2c_transaction_removelinked(_i2c_current_transaction);

				return;
			}

			_i2c_current_transaction_state=4;								// Set next state - state 4: data transfer

			// Go to the next state
			goto case4;
			break;

		// ------------------------------------------------------
		// State 3:	Check data response state. This state is entered multiple times when multiple bytes must be transferred. 
		//				Checks the return value of the data transfer
		// ------------------------------------------------------
		case 3:
			#if I2CDBG==1
			uart0_putchar('X');
			uart0_putchar('3');
			uart0_putchar('\r');
			#endif
			
			// Check the answer to the last byte transfer	
			if( (_i2c_current_transaction_rw==I2C_WRITE && twsr!=0x28) 
			//if( (_i2c_current_transaction_rw==I2C_WRITE && (twsr!=0x28 || twsr!=0x30)) 
					|| (_i2c_current_transaction_rw==I2C_READ && twsr!=0x50 && twsr!=0x58))
			{
				// Error.
				_i2c_transaction_idle=1;																												// Return to idle
				*_i2c_current_transaction_status = 3 | ((_i2c_current_transaction_n-1)<<4);		// Indicate failure in data, indicates number of bytes sent before error
				*_i2c_current_transaction_i2cerror = twsr;																			// Indicate I2C failure code

				//TWCR&=~(1<<TWIE);																																// Deactivate interrupt
				//TWCR|=(1<<TWSTO);															// Dan: send stop
				TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);											// Transmit stop, no interrupt
				//TWCR=0;

				if(_i2c_current_transaction_callback)
					_i2c_current_transaction_callback(_i2c_current_transaction);
					
				// Remove next linked transactions
				_i2c_transaction_removelinked(_i2c_current_transaction);

				return;
			}
			// If read mode, then read the data
			if(_i2c_current_transaction_rw==I2C_READ)
			{
				_i2c_current_transaction_data[_i2c_current_transaction_n-1]=TWDR;
			}

			
			// No error, flow to next state and transmit next byte

		
		// ------------------------------------------------------
		// State 4:	Transmit data state. This state is entered multiple times when multiple bytes must be transferred. 
		//				Transmits data (if any)
		// ------------------------------------------------------
		
		case 4:
		case4:
			#if I2CDBG==1
			uart0_putchar('X');
			uart0_putchar('4');
			uart0_putchar('\r');
			#endif
			if(_i2c_current_transaction_n<_i2c_current_transaction_dodata)			// If data to transmit data
			{
				_i2c_current_transaction_state=3;																	// Set next state - state 3: check data transfer answer

				// Write mode
				if(_i2c_current_transaction_rw==I2C_WRITE)													
				{
					TWDR = _i2c_current_transaction_data[_i2c_current_transaction_n];
					TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWIE);										
					#if I2CDBG==1
						//uart0_putchar('x');
						//uart0_putchar('w');
						//uart0_putchar('\r');
					#endif
				}
				else
				{
					// Read mode						
					if(_i2c_current_transaction_n == _i2c_current_transaction_dodata-1)	// Last data
						TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWIE);												// Clear acknowledge bit
					else
						TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWIE)|(1<<TWEA);							// Set acknowledge bit
						
					#if I2CDBG==1
						//uart0_putchar('x');
						//uart0_putchar('r');
						//uart0_putchar('\r');
					#endif
				}
				
				_i2c_current_transaction_n++;																			// Indicate one data more procesed
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
			#if I2CDBG==1
			uart0_putchar('X');
			uart0_putchar('5');
			uart0_putchar('\r');
			#endif
			if(_i2c_current_transaction->dostop)
				TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);											// Transmit stop, no interrupt
			//else
				//TWCR = (1<<TWINT)|(1<<TWEN);											// Transmit stop, no interrupt

			// Do not wait for any answer... we've completed successfully the task
			_i2c_transaction_idle=1;																			// Return to idle
			*_i2c_current_transaction_status = 0;													// Indicate success
			*_i2c_current_transaction_i2cerror = 0;												// Indicate success

			// Callback only if not linked to next
			if(!_i2c_current_transaction->link2next)
				if(_i2c_current_transaction_callback)
					_i2c_current_transaction_callback(_i2c_current_transaction);
					
			// Here must dealloc transacton
			i2c_transaction_pool_free1(_i2c_current_transaction);
	}
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
#if BOOTLOADER==0
void i2c_transaction_removenext(void)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		if(_i2c_transaction_buffer_rd!=_i2c_transaction_buffer_wr)
			_i2c_transaction_buffer_rd=(_i2c_transaction_buffer_rd+1)&I2C_TRANSACTION_MASK;
	}
}
#endif
/******************************************************************************
	_i2c_transaction_removelinked
*******************************************************************************	
	Internal use, ensure called atomically.
	trans: transaction from which the next linked ones must be removed.
	
	This function removes the transaction from the transaction queue.
	If a transaction has been allocated from the transaction pool (including trans)
	it is freed. 

	Removes all the subsequent linked transactions, if any.
******************************************************************************/
void _i2c_transaction_removelinked(I2C_TRANSACTION *trans)
{
	// Get the current transaction
	I2C_TRANSACTION *cur=trans;

	// Free if from pool
	i2c_transaction_pool_free1(cur);
	
	while(cur->link2next)
	{
		cur = _i2c_transaction_getnext();
		if(!cur)
			break;
		
		// Free if from pool
		i2c_transaction_pool_free1(cur);
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
			
	if(_i2c_transaction_buffer_rd!=_i2c_transaction_buffer_wr)
	{
	
		I2C_TRANSACTION *t=i2c_transaction_buffer[_i2c_transaction_buffer_rd];
		_i2c_transaction_buffer_rd=(_i2c_transaction_buffer_rd+1)&I2C_TRANSACTION_MASK;
		return t;
	}
	return 0;
}

#if BOOTLOADER==0
void i2c_transaction_print(I2C_TRANSACTION *trans,FILE *file)
{
	fprintf_P(file,PSTR("I2C transaction %p\n"),trans);
	fprintf_P(file,PSTR("Address %02X rw %d\n"),trans->address,trans->rw);
	for(unsigned char i=0;i<16;i++)
		fprintf_P(file,PSTR("%02X "),trans->data[i]);
	fprintf_P(file,PSTR("\n"));
//	for(unsigned char i=0;i<16;i++)
//		fprintf_P(file,PSTR("%02X "),trans->dataack[i]);
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
#endif


/******************************************************************************
*******************************************************************************
TRANSACTION POOL
*******************************************************************************
******************************************************************************/
/******************************************************************************
	i2c_transaction_pool_reserve
*******************************************************************************	
	Reserves (allocates) n transactions from the global transaction pool.
	Transactions must be freed with i2c_transaction_pool_free or i2c_transaction_pool_free1.

	Parameters ... must be of type I2C_TRANSACTION **, i.e. pointer to a variable
	that will hold the pointer to the transaction.


	Return value:
		0:				Success
		nonzero:	Error
******************************************************************************/
#if BOOTLOADER==0
unsigned char i2c_transaction_pool_reserve(unsigned char n,...)
{
	va_list args;
	
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		if(I2C_TRANSACTION_MAX-_i2c_transaction_pool_reserved<n)
			return 1;

		va_start(args,n);
		for(unsigned char i=0;i<n;i++)
		{
			unsigned char j;
			// Search for first free
			for(j=0;j<I2C_TRANSACTION_MAX;j++)
			{
				//printf("%d\n",_i2c_transaction_pool_alloc[j]);
				if(!_i2c_transaction_pool_alloc[j])
					break;
			}
			
			// Free transaction at j			
			//printf("Trans %d free at %p\n",j,&_i2c_transaction_pool[j]);
			_i2c_transaction_pool_alloc[j]=1;
			
			I2C_TRANSACTION **t = va_arg(args,I2C_TRANSACTION **);
			
			*t = &_i2c_transaction_pool[j];
		}
		_i2c_transaction_pool_reserved+=n;
	}
	return 0;
}
#endif 
/******************************************************************************
	i2c_transaction_pool_free
*******************************************************************************	
	Frees (deallocates) n transactions from the global transaction pool.
	Transactions should be allocated with i2c_transaction_pool_reserve.
	However, if a transaction that has not been allocated with 
	i2c_transaction_pool_reserve is passed that transaction is ignored.
	This makes it safe to call this function regardless of how the transaction
	has been allocated (e.g. global, stack, or allocated from the pool).
	
	
	Parameters ... must be of type I2C_TRANSACTION *, i.e. pointers to 
	the transaction.

	Return value:
		0:				Success
		nonzero:	Error
******************************************************************************/
void i2c_transaction_pool_free(unsigned char n,...)
{
	#if BOOTLOADER==0
	va_list args;
	
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		va_start(args,n);
		for(unsigned char i=0;i<n;i++)
		{
			// Get the transaction
			I2C_TRANSACTION *t = va_arg(args,I2C_TRANSACTION *);
			
			// Find the offset of the transaction in the pool
			unsigned short off = (unsigned short)(t-_i2c_transaction_pool);
			
			// Skip if it is not a transaction allocated from the pool
			if(off>=I2C_TRANSACTION_MAX)
				continue;
			
			//printf("remove at off: %d\n",off);
			_i2c_transaction_pool_alloc[off] = 0;
			_i2c_transaction_pool_reserved--;
		}		
	}
	#endif
}

/******************************************************************************
	i2c_transaction_pool_free1
*******************************************************************************	
	Frees (deallocates) 1 transactions from the global transaction pool.
	The transaction should be allocated with i2c_transaction_pool_reserve.
	However, if a transaction that has not been allocated with 
	i2c_transaction_pool_reserve is passed that transaction is ignored.
	This makes it safe to call this function regardless of how the transaction
	has been allocated (e.g. global, stack, or allocated from the pool).
	
	

	Return value:
		0:				Success
		nonzero:	Error
******************************************************************************/
void i2c_transaction_pool_free1(I2C_TRANSACTION *t)
{
	#if BOOTLOADER==0
	
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{		
		// Find the offset of the transaction in the pool
		unsigned short off = (unsigned short)(t-_i2c_transaction_pool);
		
		// Skip if it is not a transaction allocated from the pool
		if(off>=I2C_TRANSACTION_MAX)
			continue;
		
		//printf("remove at off: %d\n",off);
		_i2c_transaction_pool_alloc[off] = 0;
		_i2c_transaction_pool_reserved--;
	}
	#endif
}
