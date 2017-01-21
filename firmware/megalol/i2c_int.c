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
#include "serial.h"
//#include "global.h"
//#include "helper.h"
#include "main.h"
//#include "test.h"
//#include "wait.h"
#include "i2c.h"
#include "adc.h"
#include "i2c_internal.h"

/*
	Write error codes
		0x08:		Start transmitted
		0x10:		Repeat start transmitted
		0x18:		SLA+W transmitted, ack received
		0x20:		SLA+W transmitted, ack not received
		0x38:		data  transmitted, ack received
		0x30:		data  transmitted, ack not received
		0x38:		arbitration lost in SLA+W or data bytes
		0x00:		successful completion (start sent, SLA+W sent and ack received)
		
	Read error codes
		0x08:		Start transmitted
		0x10:		Repeat start transmitted
		0x38:		Arbitration lost in SLA+R or not acknowledge
		0x40:		SLA+R transmitted, ack received
		0x48:		SLA+R transmitted, not ack received
		0x50:		data byte received, returned ack
		0x58: 	data byte received, returned not ack
		
*/

// An I2C bus can also be reset with the magical sequence explained in multiple places : start, clock 9 bits, start, stop.





/******************************************************************************
*******************************************************************************
INTERRUPT-DRIVEN ACCESS   INTERRUPT-DRIVEN ACCESS   INTERRUPT-DRIVEN ACCESS   
*******************************************************************************
******************************************************************************/


/******************************************************************************
	i2c_readreg_int
*******************************************************************************	
	General purpose read I2C register, for most I2C devices.
	Operation: writes the register of interest, and performs a single byte read.
		
	Interrupt-driven, blocking.
		
	Return value:
		0:				Success
		nonzero:	Error (see i2c_readstart and AVR datasheet for info)
******************************************************************************/
unsigned char i2c_readreg_int(unsigned char addr7,unsigned char reg,unsigned char *val)
{
	I2C_TRANSACTION t1,t2;
	
	// Register selection transaction
	i2c_transaction_setup(&t1,addr7,I2C_WRITE,0,1);
	t1.data[0]=reg;
	// Register read transaction
	i2c_transaction_setup(&t2,addr7,I2C_READ,1,1);
	t2.extdata=val;
	
	
	unsigned char r;
	do 
	{
		r = i2c_transaction_queue(2,1,&t1,&t2);
	} while(r);
	
	// Check error code
	if(t1.status!=0)
		return t1.i2cerror;
	if(t2.status!=0)
		return t2.i2cerror;
	return 0;
}
/******************************************************************************
	i2c_readreg_int_cb
*******************************************************************************	
	General purpose read I2C register, for most I2C devices.
	Operation: writes the register of interest, and performs a single byte read.
	
	Returns immediately (non blocking) and a callback is called upon completion
	or transaction error.
	Relies on the transaction pool.
	Failure can occur: due to insufficient free transactions in the pool, due
	to insufficient space in transaction queue, or during the transaction itself.

	val: 	if zero result is in the transaction buffer; 
				if nonzero then result is in val

	Interrupt-driven, blocking.
		
	Return value:
		0:				Success
		nonzero:	Error:  not enough transactions in pool or no space in queue
******************************************************************************/
unsigned char i2c_readreg_int_cb(unsigned char addr7,unsigned char reg,unsigned char *val,I2C_CALLBACK cb)
{
	I2C_TRANSACTION *t1,*t2;
	unsigned char r;
	
	r=i2c_transaction_pool_reserve(2,&t1,&t2);
	if(r)
		return 1;
	
	// Register selection transaction
	i2c_transaction_setup(t1,addr7,I2C_WRITE,0,1);
	t1->data[0]=reg;
	t1->callback=cb;
	// Register read transaction
	i2c_transaction_setup(t2,addr7,I2C_READ,1,1);
	t2->extdata=val;
	t2->callback=cb;
	
	r = i2c_transaction_queue(2,0,t1,t2);
	if(r)
		return 2;
	
	return 0;
}

/******************************************************************************
	i2c_readregs_int
*******************************************************************************	
	General purpose read several I2C registers, for most I2C devices.
	Operation: writes the register of interest, and performs a multiple byte read.
	
	Interrupt-driven, blocking.
	
	val: 	if zero result is in the transaction buffer; 
				if nonzero then result is in val
	
	Return value:	
		zero:			success
		nonzero:	error (see i2c_readstart and AVR datasheet for info)		
******************************************************************************/
unsigned char i2c_readregs_int(unsigned char addr7,unsigned char reg,unsigned char n,unsigned char *val)
{
	I2C_TRANSACTION t1,t2;
	
	// Register selection transaction
	i2c_transaction_setup(&t1,addr7,I2C_WRITE,0,1);
	t1.data[0]=reg;
	// Register read transaction
	i2c_transaction_setup(&t2,addr7,I2C_READ,1,n);
	t2.extdata=val;
	
	
	unsigned char r;
	do 
	{
		r = i2c_transaction_queue(2,1,&t1,&t2);
	} while(r);
	
	// Check error code
	if(t1.status!=0)
		return t1.i2cerror;
	if(t2.status!=0)
		return t2.i2cerror;
	return 0;	
}
/******************************************************************************
	i2c_readregs_int_cb
*******************************************************************************	
	General purpose read several I2C registers, for most I2C devices.
	Operation: writes the register of interest, and performs a multiple byte read.
	
	Returns immediately (non blocking) and a callback is called upon completion
	or transaction error.
	Relies on the transaction pool.
	Failure can occur: due to insufficient free transactions in the pool, due
	to insufficient space in transaction queue, or during the transaction itself.

	val: 	if zero result is in the transaction buffer; 
				if nonzero then result is in val
	
	Interrupt-driven, blocking.
	
	val: 	if zero result is in the transaction buffer; 
				if nonzero then result is in val
	
	Return value:	
		zero:			success
		nonzero:	error (see i2c_readstart and AVR datasheet for info)		
******************************************************************************/
unsigned char i2c_readregs_int_cb(unsigned char addr7,unsigned char reg,unsigned char n,unsigned char *val,I2C_CALLBACK cb,void *user)
{
	I2C_TRANSACTION *t1,*t2;
	unsigned char r;
	
	r=i2c_transaction_pool_reserve(2,&t1,&t2);
	if(r)
		return 1;
	
	// Register selection transaction
	i2c_transaction_setup(t1,addr7,I2C_WRITE,0,1);
	t1->data[0]=reg;
	t1->callback=cb;
	t1->user=user;
	// Register read transactions
	i2c_transaction_setup(t2,addr7,I2C_READ,1,n);
	t2->extdata=val;
	t2->callback=cb;
	t2->user=user;
	
	r = i2c_transaction_queue(2,0,t1,t2);
	if(r)
		return 2;
	
	return 0;
}
/******************************************************************************
	i2c_writereg_int
*******************************************************************************	
	General purpose write an I2C register, for most I2C devices.
	Operation: writes the register of interest, writes the value
		
	Interrupt-driven, blocking.
		
	Return value:
		0:				Success
		nonzero:	Error (see i2c_readstart and AVR datasheet for info)
******************************************************************************/
unsigned char i2c_writereg_int(unsigned char addr7,unsigned char reg,unsigned char val)
{
	I2C_TRANSACTION t1;
	
		
	// Transaction: select register, write, stop
	i2c_transaction_setup(&t1,addr7,I2C_WRITE,1,2);
	t1.data[0]=reg;
	t1.data[1]=val;
	
	unsigned char r;
	do 
	{
		//printf("i2cwri queue\n");
		r = i2c_transaction_queue(1,1,&t1);
		//if(r)
			//printf("i2cwri fail %02X\n",r);
	} while(r);
	
	//printf("i2cwri done status: %02x error: %02x\n",t1.status,t1.i2cerror);
	// Check error code
	if(t1.status!=0)
		return t1.i2cerror;
	return 0;
}
/******************************************************************************
	i2c_writeregs_int
*******************************************************************************	
	General purpose write an I2C register, for most I2C devices.
	Operation: writes the register address, writes the values
		
	Maximum number of registers: size of transaction-1
		
	Interrupt-driven, blocking.
		
	Return value:
		0:				Success
		nonzero:	Error (see i2c_readstart and AVR datasheet for info)
******************************************************************************/
unsigned char i2c_writeregs_int(unsigned char addr7,unsigned char reg,unsigned char *val,unsigned char n)
{
	I2C_TRANSACTION t1;
	
	// Transaction: select register, write, stop
	i2c_transaction_setup(&t1,addr7,I2C_WRITE,1,n+1);
	t1.data[0]=reg;
	for(unsigned char i=0;i<n;i++)
		t1.data[1+i]=val[i];
	
	unsigned char r;
	do 
	{
	//	printf("queue ");
		r = i2c_transaction_queue(1,1,&t1);
		//if(r)
			//printf("fail\n");
	} while(r);
	
	// Check error code
	if(t1.status!=0)
		return t1.i2cerror;
	return 0;
}