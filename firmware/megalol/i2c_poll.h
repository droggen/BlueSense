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

#ifndef __I2CPOLL_H
#define __I2CPOLL_H

#include "i2c_internal.h"


/******************************************************************************
*******************************************************************************
DIRECT ACCESS   DIRECT ACCESS   DIRECT ACCESS   DIRECT ACCESS   DIRECT ACCESS   
*******************************************************************************
******************************************************************************/

void i2c_scan(unsigned long long *h,unsigned long long *l);
void i2c_check(void);

unsigned char i2c_readreg_poll(unsigned char addr7,unsigned char reg,unsigned char *val);
unsigned char i2c_readregs_poll(unsigned char addr7,unsigned char reg,unsigned char n,unsigned char *val);
unsigned char i2c_writereg_poll(unsigned char addr7,unsigned char reg,unsigned char val);


unsigned char i2c_readstart(unsigned addr7);
unsigned char i2c_readdata(unsigned char *data,unsigned char acknowledge);
unsigned char i2c_writestart(unsigned addr7);
unsigned char i2c_writedata(unsigned char data);
void i2c_stop(void);


#endif



