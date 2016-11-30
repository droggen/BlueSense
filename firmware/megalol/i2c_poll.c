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
#include "i2c_poll.h"
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
DIRECT ACCESS   DIRECT ACCESS   DIRECT ACCESS   DIRECT ACCESS   DIRECT ACCESS   
*******************************************************************************
******************************************************************************/


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
		int rv = i2c_readstart(i);
		if(rv==0)
			*l|=1;
		//i2c_stop(1);
	}
}



/******************************************************************************
	i2c_readreg_poll
*******************************************************************************	
	General purpose read I2C register, for most I2C devices.
	Operation: writes the register of interest, and performs a single byte read
		
	Return value:
		0:				Success
		nonzero:	Error (see i2c_readstart and AVR datasheet for info)
******************************************************************************/
unsigned char i2c_readreg_poll(unsigned char addr7,unsigned char reg,unsigned char *val)
{
	unsigned char r;
	r=i2c_writestart(addr7);			// Slave write
	if(r!=0x00)
	{
		fprintf(file_bt,"w");
			return r;
	}
	r=i2c_writedata(reg);				// Register of interest
	if(r!=0x00)
			return r;
	r=i2c_readstart(addr7);		// Slave read
	if(r!=0x00)
			return r;
	r=i2c_readdata(val,0);				// Read with NAK
	if(r!=0x00)
			return r;
	i2c_stop();										// Stop
	return 0x00;
}

/******************************************************************************
	i2c_readregs_poll
*******************************************************************************	
	General purpose read several I2C registers, for most I2C devices.
	Operation: writes the register of interest, and performs a multiple byte read
		
	Return value:
		0:				Success
		nonzero:	Error (see i2c_readstart and AVR datasheet for info)
******************************************************************************/
unsigned char i2c_readregs_poll(unsigned char addr7,unsigned char reg,unsigned char n,unsigned char *val)
{
	unsigned char r;
	r=i2c_writestart(addr7);			// Slave write
	if(r!=0x00)
			return r;
	r=i2c_writedata(reg);				// Register of interest
	if(r!=0x00)
			return r;
	r=i2c_readstart(addr7);		// Slave read
	if(r!=0x00)
			return r;
	for(unsigned char i=0;i<n;i++)
	{
		r=i2c_readdata(&val[i],i==n-1 ? 0:1);				// Acknowledge with ACK except last read with NACK
		if(r!=0x00)
				return r;
	}
	i2c_stop();										// Stop
	return 0x00;
}


/******************************************************************************
	i2c_writereg_poll
*******************************************************************************	
	General purpose write an I2C register, for most I2C devices.
	Operation: writes the register of interest, writes the register address, writes the value
		
	Return value:
		0:				Success
		nonzero:	Error (see i2c_readstart and AVR datasheet for info)
******************************************************************************/

unsigned char i2c_writereg_poll(unsigned char addr7,unsigned char reg,unsigned char val)
{
	unsigned char r;
	r=i2c_writestart(addr7);
	if(r!=0x00)
	{
		i2c_stop();
		return r;
	}
	i2c_writedata(reg);
	if(r!=0x00)
	{
		i2c_stop();
		return r;
	}
	i2c_writedata(val);
	if(r!=0x00)
	{
		i2c_stop();
		return r;
	}
	i2c_stop();
	return 0;
}
/******************************************************************************
	i2c_writeregs_poll
*******************************************************************************	
	General purpose write an I2C register, for most I2C devices.
	Operation: writes the register of interest, writes the register address, writes the value
		
	Return value:
		0:				Success
		nonzero:	Error (see i2c_readstart and AVR datasheet for info)
******************************************************************************/
unsigned char i2c_writeregs_poll(unsigned char addr7,unsigned char reg,unsigned char *val,unsigned char n)
{
	unsigned char r;
	r=i2c_writestart(addr7);
	if(r!=0x00)
	{
		i2c_stop();
		return r;			
	}
	i2c_writedata(reg);
	if(r!=0x00)
	{
		i2c_stop();
		return r;
	}
	for(unsigned i=0;i<n;i++)
	{
		i2c_writedata(val[i]);
		if(r!=0x00)
		{
			i2c_stop();
			return r;
		}
	}
	i2c_stop();
	return 0;
}

void i2c_check(void)
{
	unsigned long long i2cbmh,i2cbml;
	printf_P(PSTR("Scanning I2C bus. Peripherals at address: "));
	i2c_scan(&i2cbmh,&i2cbml);
	for(int i=0;i<128;i++)
	{
		if(i2cbml&1)
			printf("%d ",i);
		i2cbml>>=1;
		i2cbml|=(i2cbmh&1)<<63;
		i2cbmh>>=1;
	}
	printf_P(PSTR("\n"));
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
unsigned char i2c_writestart(unsigned addr7)
{
	unsigned char twcr,twsr,twdr;

	// Transmit a start condition
	#if I2CDBG==1
		printf_P(PSTR("        I2C START\r"));
	#endif
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);						
	do
	{
		twcr = TWCR;
		twsr = TWSR;
		twdr = TWDR;
		#if I2CDBG==1
			printf_P(PSTR("                TWCR: %X TWSR: %X TWDR: %X\r"),twcr,twsr,twdr);
		#endif
		fprintf_P(file_bt,PSTR("                TWCR: %X TWSR: %X TWDR: %X\r"),twcr,twsr,twdr);
	}
	while (!(twcr & (1<<TWINT)));										// Wait until TWINT is set
	twsr &= 0xF8;
	if((twsr!=0x08) && (twsr!=0x10))
	{
		#if I2CDBG==1
			printf_P(PSTR("        I2C START error (%X)\r"),twsr);
		#endif
		fprintf_P(file_bt,PSTR("        I2C START error (%X)\r"),twsr);
		return twsr;
	}
	#if I2CDBG==1
		printf_P(PSTR("        I2C START ok (%X)\r"),twsr);
	#endif

	// Transmit the address
	#if I2CDBG==1
		printf_P(PSTR("        I2C SLA+W\r"));
	#endif
	fprintf_P(file_bt,PSTR("        I2C SLA+W\r"));
	TWDR = addr7<<1;														// I2C 7-bit address + write
	TWCR = (1<<TWINT) | (1<<TWEN);								// Send
	do
	{
		twcr = TWCR;
		twsr = TWSR;
		twdr = TWDR;
		#if I2CDBG==1
			printf_P(PSTR("                TWCR: %X TWSR: %X TWDR: %X\r"),twcr,twsr,twdr);
		#endif
		fprintf_P(file_bt,PSTR("                TWCR: %X TWSR: %X TWDR: %X\r"),twcr,twsr,twdr);
	}
	while (!(twcr & (1<<TWINT)));										// Wait until TWINT is set
	twsr &= 0xF8;

	if(twsr != 0x18)
	{
		#if I2CBDG
			printf_P(PSTR("        I2C address send error (%X)\r"),twsr);
		#endif
		fprintf_P(file_bt,PSTR("        I2C address send error (%X)\r"),twsr);
		return twsr;
	}
	#if I2CDBG==1
		printf_P(PSTR("        I2C address send ok (%X)\r"),twsr);
	#endif
	fprintf_P(file_bt,PSTR("        I2C address send ok (%X)\r"),twsr);

	return 0;
}
/*
	i2c_writedata
	Assumes a i2c_startwrite was successfully called, i.e. a slave has acknowledged being addressed and is waiting for data.

	Return values:
		See i2c_writestart

*/
unsigned char i2c_writedata(unsigned char data)
{
	unsigned char twcr,twsr;
	#if I2CDBG==1
	unsigned char twdr;
	#endif
	
	#if I2CDBG==1
		printf_P(PSTR("        I2C DATA Write\r"));
	#endif
	
	TWDR = data;
	TWCR = (1<<TWINT) | (1<<TWEN);
	do
	{
		twcr = TWCR;
		twsr = TWSR;
		#if I2CDBG==1
			twdr = TWDR;
			printf_P(PSTR("                TWCR: %X TWSR: %X TWDR: %X\r"),twcr,twsr,twdr);
		#endif
	}
	while (!(twcr & (1<<TWINT)));										// Wait until TWINT is set
	twsr &= 0xF8;

	if(twsr != 0x28)
	{
		#if I2CDBG==1
			printf_P(PSTR("        I2C DATA Write error (%X)\r"),twsr);
		#endif
		return twsr;
	}
	#if I2CDBG==1
		printf_P(PSTR("        I2C DATA Write ok (%X)\r"),twsr);
	#endif
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
unsigned char i2c_readstart(unsigned addr7)
{
	unsigned char twcr,twsr;

	#if I2CDBG==1
	unsigned char twdr;
	#endif

	// Transmit a start condition
	#if I2CDBG==1
		printf_P(PSTR("        I2C SLA+R START\r"));
	#endif
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);						
	do
	{
		twcr = TWCR;
		twsr = TWSR;
		#if I2CDBG==1
			twdr = TWDR;
			printf_P(PSTR("                TWCR: %X TWSR: %X TWDR: %X\r"),twcr,twsr,twdr);
		#endif
	}
	while (!(twcr & (1<<TWINT)));										// Wait until TWINT is set
	twsr &= 0xF8;
	if((twsr!=0x08) && (twsr!=0x10))		// Start and repeat start are okay
	{
		#if I2CDBG==1
			printf_P(PSTR("        I2C SLA+R START error (%X)\r"),twsr);
		#endif
		return twsr;
	}
	#if I2CDBG==1
		printf_P(PSTR("        I2C SLA+R START ok (%X)\r"),twsr);
	#endif

	// Transmit the address
	#if I2CDBG==1
		printf_P(PSTR("        I2C SLA+R\r"));
	#endif
	TWDR = (addr7<<1)+1;														// I2C 7-bit address + read
	TWCR = (1<<TWINT) | (1<<TWEN);						// Send
	do
	{
		twcr = TWCR;
		twsr = TWSR;
		#if I2CDBG==1
			twdr = TWDR;
			printf_P(PSTR("                TWCR: %X TWSR: %X TWDR: %X\r"),twcr,twsr,twdr);
		#endif
	}
	while (!(twcr & (1<<TWINT)));										// Wait until TWINT is set
	twsr &= 0xF8;

	if(twsr != 0x40)
	{
		#if I2CDBG==1
			printf_P(PSTR("        I2C SLA+R error (%X)\r"),twsr);
		#endif
		return twsr;
	}
	#if I2C_DBG
		printf_P(PSTR("        I2C SLA+R ok (%X)\r"),twsr);
	#endif

	return 0;
}
/*
	i2c_readdata
	Assumes a i2c_startread was successfully called, i.e. a slave has acknowledged being addressed and is ready to send data.

	Return values:
		See i2c_readstart

*/
unsigned char i2c_readdata(unsigned char *data,unsigned char acknowledge)
{
	unsigned char twcr,twsr,twdr;	

	#if I2CDBG==1
		printf_P(PSTR("        I2C DATA Read\r"));
	#endif
	
	if(acknowledge)
		TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	else
		TWCR = (1<<TWINT) | (1<<TWEN);
	do
	{
		twcr = TWCR;
		twsr = TWSR;
		twdr = TWDR;
		#if I2CDBG==1
			printf_P(PSTR("                TWCR: %X TWSR: %X TWDR: %X\r"),twcr,twsr,twdr);
		#endif
	}
	while (!(twcr & (1<<TWINT)));										// Wait until TWINT is set
	twsr &= 0xF8;

	if((acknowledge && twsr != 0x50) | (acknowledge==0 && twsr != 0x58))
	{
		#if I2CDBG==1
			printf_P(PSTR("        I2C DATA Read error (%X)\r"),twsr);
		#endif
		return twsr;
	}
	#if I2CDBG==1
		printf_P(PSTR("        I2C DATA Read ok (%X)\r"),twsr);
	#endif
	*data = twdr;
	return 0;
}

void i2c_stop(void)
{
	#if I2CDBG==1
	unsigned char twdr,twsr,twcr;
	#endif
	
	
	
	
	#if I2CDBG==1
		twdr = TWDR;
		twsr = TWSR;
		twcr = TWCR;
		printf_P(PSTR("        I2C STOP:                TWCR: %X TWSR: %X TWDR: %X\r"),twcr,twsr,twdr);
	#endif
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
		#if I2CDBG==1
			printf_P(PSTR("        I2C STOP:                TWCR: %X TWSR: %X TWDR: %X\r"),twcr,twsr,twdr);
		#endif
	}
	while(twcr&(1<<TWSTO));							// Wait until the stop condition is transmitted (bit is cleared upon transmission)
	*/
}