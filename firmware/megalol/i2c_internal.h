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


#ifndef __I2C_INTERNAL_H
#define __I2C_INTERNAL_H


#define I2C_READ			1
#define I2C_WRITE			0

#define I2CDBG 0
//#define I2CDBG 1

#define I2C_TRANSACTION_MAX		8
#define I2C_TRANSACTION_MASK	(I2C_TRANSACTION_MAX-1)

struct  _I2C_TRANSACTION;
typedef unsigned char(*I2C_CALLBACK)(struct _I2C_TRANSACTION *);

// Only entries modified by interrupts are marked as volatile
typedef struct  _I2C_TRANSACTION {
	unsigned char address;													// 7-bit address
	unsigned char rw;																// write: 0. read: 1.
	unsigned char data[16];													// data to send/receive
	unsigned char *extdata;													// If nonzero, use this buffer for send/receive, otherwise data is used
	//volatile unsigned char dataack[16];									// acknowledge to send/receive
	unsigned char dodata;													// Send/read the data (otherwise, only the rest is done): specifies the number of bytes, or 0.
	unsigned char dostop;													// Send I2C stop 
	I2C_CALLBACK callback;													// Callback to call upon completion or error
	void *user;																// User data

	unsigned char link2next;												// Linked: failure in this transaction cancels the next one; success in this transaction do not call callback, only last one
	
	// Status information
	volatile unsigned char status;											// Status:	Lower 4 bit indicate status. 
																			//				0: success. 
																			//				1: failed in start condition.
																			//				2: failed in address.
																			//				3: failed in data (high 4 bit indicated how many bytes were transmitted successfully before the error)
	volatile unsigned char i2cerror;										// Indicate the AVR I2C error code, when status is non-null
} I2C_TRANSACTION;



// Internal stuff
extern I2C_TRANSACTION *i2c_transaction_buffer[I2C_TRANSACTION_MAX];									
extern unsigned char _i2c_transaction_buffer_wr, _i2c_transaction_buffer_rd;						


// Transaction pool
extern I2C_TRANSACTION _i2c_transaction_pool[I2C_TRANSACTION_MAX];
extern unsigned char _i2c_transaction_pool_alloc[I2C_TRANSACTION_MAX];
extern unsigned char _i2c_transaction_pool_wr, _i2c_transaction_pool_rd;
extern unsigned char _i2c_transaction_pool_reserved;


// Caching of current transaction parameters
extern volatile unsigned char _i2c_transaction_idle;																	// Indicates whether a transaction is in progress or idle. Modified by interrupt; read in queue to detect end of transaction
extern I2C_TRANSACTION *_i2c_current_transaction;																				// Current transaction for the interrupt routine.
extern unsigned char _i2c_current_transaction_n;																				// Modified by interrupt, but only used in interrupt
extern unsigned char _i2c_current_transaction_state;																		// Modified by interrupt, but only used in interrupt
extern unsigned char _i2c_current_transaction_dodata;																		// Modified by interrupt, but only used in interrupt
extern unsigned char _i2c_current_transaction_address;																	// Modified by interrupt, but only used in interrupt
extern unsigned char _i2c_current_transaction_rw;																				// Modified by interrupt, but only used in interrupt
extern I2C_CALLBACK  _i2c_current_transaction_callback;																	// Modified by interrupt, but only used in interrupt
extern volatile unsigned char *_i2c_current_transaction_status,*_i2c_current_transaction_i2cerror;
extern unsigned char *_i2c_current_transaction_data;
extern I2C_TRANSACTION *i2c_transaction_buffer[I2C_TRANSACTION_MAX];
extern volatile unsigned char _i2c_transaction_idle;


extern volatile unsigned long int i2c_intctr;

/******************************************************************************
*******************************************************************************
TRANSACTION POOL
*******************************************************************************
******************************************************************************/
unsigned char i2c_transaction_pool_reserve(unsigned char n,...);
void i2c_transaction_pool_free(unsigned char n,...);
void i2c_transaction_pool_free1(I2C_TRANSACTION *t);
void i2c_transaction_pool_print(void);
void i2c_transaction_pool_test(void);
void i2c_transaction_pool_test2(void);
void i2c_transaction_pool_test3(void);



/******************************************************************************
*******************************************************************************
INTERRUPT-DRIVEN ACCESS   INTERRUPT-DRIVEN ACCESS   INTERRUPT-DRIVEN ACCESS   
*******************************************************************************
******************************************************************************/
void i2c_transaction_init(void);
void TWI_vect_intx(void);
void TWI_vect_statemachine(void);
void TWI_vect_int2(void);

void _i2c_transaction_put(I2C_TRANSACTION *trans);
I2C_TRANSACTION *_i2c_transaction_getnext(void);
void _i2c_transaction_removelinked(I2C_TRANSACTION *);
unsigned char _i2c_transaction_getfree(void);

void i2c_transaction_print(I2C_TRANSACTION *trans,FILE *file);
void i2c_transaction_printall(FILE *file);


#endif