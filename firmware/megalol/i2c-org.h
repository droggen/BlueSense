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

#ifndef __I2CSENSOR_H
#define __I2CSENSOR_H


#define I2C_READ			1
#define I2C_WRITE			0

//#define I2CDBG

#define I2C_TRANSACTION_MAX		8
#define I2C_TRANSACTION_MASK	(I2C_TRANSACTION_MAX-1)

struct _I2C_TRANSACTION;

typedef unsigned char(*I2C_CALLBACK)(struct _I2C_TRANSACTION *);


// Only entries modified by interrupts are marked as volatile
typedef struct  _I2C_TRANSACTION {
	unsigned char address;													// 7-bit address
	unsigned char rw;																// write: 0. read: 1.
	unsigned char data[16];													// data to send/receive
	unsigned char *extdata;													// If nonzero, use this buffer for send/receive, otherwise data is used
	//volatile unsigned char dataack[16];						// acknowledge to send/receive
	unsigned char doaddress;												// Send the address (otherwise, only the rest is done)
	unsigned char dodata;														// Send/read the data (otherwise, only the rest is done): specifies the number of bytes, or 0.
	unsigned char dostart;													// Send I2C start 
	unsigned char dostop;														// Send I2C stop 
	I2C_CALLBACK callback;													// Callback to call upon completion or error
	void *user;																			// User data
	
	// Status information
	volatile unsigned char status;									// Status:	Lower 4 bit indicate status. 
																									//				0: success. 
																									//				1: failed in start condition.
																									//				2: failed in address.
																									//				3: failed in data (high 4 bit indicated how many bytes were transmitted successfully before the error)
	volatile unsigned char i2cerror;								// Indicate the AVR I2C error code, when status is non-null
} I2C_TRANSACTION;


extern I2C_TRANSACTION *i2c_transaction_buffer[I2C_TRANSACTION_MAX];

extern volatile unsigned char _i2c_transaction_idle;

/******************************************************************************
*******************************************************************************
DIRECT ACCESS   DIRECT ACCESS   DIRECT ACCESS   DIRECT ACCESS   DIRECT ACCESS   
*******************************************************************************
******************************************************************************/

void i2c_init(void);
unsigned char i2c_readstart(unsigned addr7,unsigned char debug);
unsigned char i2c_readdata(unsigned char *data,unsigned char acknowledge,unsigned char debug);
unsigned char i2c_writestart(unsigned addr7,unsigned char debug);
unsigned char i2c_writedata(unsigned char data,unsigned char debug);
void i2c_stop(unsigned char debug);
void i2c_scan(unsigned long long *h,unsigned long long *l);
void i2c_check(void);

unsigned char i2c_readreg(unsigned char addr7,unsigned char reg,unsigned char *val);
unsigned char i2c_readregs(unsigned char addr7,unsigned char reg,unsigned char n,unsigned char *val);
unsigned char i2c_writereg(unsigned char addr7,unsigned char reg,unsigned char val);

/******************************************************************************
*******************************************************************************
INTERRUPT-DRIVEN ACCESS   INTERRUPT-DRIVEN ACCESS   INTERRUPT-DRIVEN ACCESS   
*******************************************************************************
******************************************************************************/
void i2c_transaction_setup(I2C_TRANSACTION *t,unsigned char addr7,unsigned char mode,unsigned char stop,unsigned char n);

extern unsigned char _i2c_transaction_buffer_wr,_i2c_transaction_buffer_rd;

void TWI_vect_intx(void);
void TWI_vect_int(void);

unsigned char i2c_transaction_execute(I2C_TRANSACTION *trans,unsigned char blocking);
void i2c_transaction_cancel(void);

// 
void i2c_transaction_init(void);
unsigned char i2c_transaction_queue(unsigned char n,unsigned char blocking,...);
unsigned char i2c_transaction_getqueued(void);
void i2c_transaction_removenext(void);

unsigned char i2c_readregs_int(unsigned char addr7,unsigned char reg,unsigned char n,unsigned char *val);

void _i2c_transaction_put(I2C_TRANSACTION *trans);
I2C_TRANSACTION *_i2c_transaction_getnext(void);
unsigned char _i2c_transaction_getfree(void);

void i2c_transaction_print(I2C_TRANSACTION *trans,FILE *file);
void i2c_transaction_printall(FILE *file);

void i2c_int_tst(void);

//--------------------
void i2c_test_int0(void);


#endif



