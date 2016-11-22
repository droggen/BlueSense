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
#ifdef ENABLE_I2CINTERRUPT
#include "i2c_int.h"
#endif
#ifdef ENABLE_I2CPOLL
#include "i2c_poll.h"
#endif


/*
	File: i2c
	
	I2C functions
	
*/

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
GENERAL & WRAPPER
*******************************************************************************
******************************************************************************/

void i2c_init(void)
{
	i2c_transaction_init();

	#if HWVER==1
	TWCR = 0;					// Deactivate
	TWBR = 1; TWSR = 0;			// 409.6KHz @ 7'372'800 Hz
	//TWBR = 0; TWSR = 0;		// 460.8KHz @ 7'372'800 Hz
	//TWBR = 1; TWSR = 1;		// 307.2KHz @ 7'372'800 Hz
	//TWBR = 1; TWSR = 3;		// 409.6KHz/64=6KHz @ 7'372'800 Hz
	//TWBR = 255; TWSR = 3;		// 225Hz @ 7'372'800 Hz
	//TWBR = 255; TWSR = 3;		// 225Hz @ 7'372'800 Hz
	
	//TWBR = 4; TWSR = 3;		// 13963Hz @ 7'372'800 Hz
	//TWBR = 10; TWSR = 3;		// 5689Hz @ 7'372'800 Hz
	
	//TWBR = 0; TWSR = 0;		// 460.8KHz @ 7'372'800 Hz
	#endif
	#if (HWVER==4) || (HWVER==5) || (HWVER==6)
	TWBR = 6; TWSR = 0;			// 394.9KHz @ 11'059'200 Hz
	//TWBR = 47; TWSR = 0;		// 100.5KHz @ 11'059'200 Hz
	//TWBR = 34; TWSR = 2;		// 10KHz @ 11'059'200 Hz
	//TWBR = 172; TWSR = 3;		// 501.9Hz @ 11'059'200 Hz
	//TWBR = 5; TWSR = 0;		// 425.3KHz @ 11'059'200 Hz
	#endif






	TWCR = (1<<TWEN);			// Enable TWI
	// Create stop condition
//	TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
	
	// Reset bus
	

}
void i2c_deinit(void)
{
	TWCR=0;			// Disable TWI
}

#if BOOTLOADER==0
unsigned char i2c_readreg(unsigned char addr7,unsigned char reg,unsigned char *val)
{
	#ifdef ENABLE_I2CINTERRUPT
		//printf("readreg: int\n");
		return i2c_readreg_int(addr7,reg,val);
	#else
		#ifdef ENABLE_I2CPOLL
			return i2c_readreg_poll(addr7,reg,val);
		#else
			#error No I2c read function (neither interrupt nor polling)
		#endif
	#endif
}
unsigned char i2c_readregs(unsigned char addr7,unsigned char reg,unsigned char n,unsigned char *val)
{
	#ifdef ENABLE_I2CINTERRUPT
	//printf("readregs: int\n");
		return i2c_readregs_int(addr7,reg,n,val);
	#else
		#ifdef ENABLE_I2CPOLL
			return i2c_readregs_poll(addr7,reg,n,val);
		#else
			return 55;
		#endif
	#endif
}
unsigned char i2c_writereg(unsigned char addr7,unsigned char reg,unsigned char val)
{
	#ifdef ENABLE_I2CINTERRUPT
		//printf("writereg: int\n");
		return i2c_writereg_int(addr7,reg,val);
	#else
		#ifdef ENABLE_I2CPOLL
	  //printf("writereg: poll\n");
		return i2c_writereg_poll(addr7,reg,val);
		#else
			return 55;
		#endif
	#endif
}
unsigned char i2c_writeregs(unsigned char addr7,unsigned char reg,unsigned char *val,unsigned char n)
{
	#ifdef ENABLE_I2CINTERRUPT
		//printf("writereg: int\n");
		return i2c_writeregs_int(addr7,reg,val,n);
	#else
		#ifdef ENABLE_I2CPOLL
	//printf("writereg: poll\n");
		return i2c_writeregs_poll(addr7,reg,val);
		#else
			return 55;
		#endif
	#endif
}
#endif
