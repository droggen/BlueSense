/*
   CASIMIR - Firmware
   Copyright (C) 2009,2010:
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
#include <avr/pgmspace.h>
#include <stdio.h>


/******************************************************************************
	Helper stuff
******************************************************************************/
void data_checksum(unsigned char *buffer,unsigned short n,unsigned char *x1,unsigned char *x2)
{
	unsigned char xsum1,xsum2;
	xsum1=0;
	xsum2=0;
	for(unsigned short i=0;i<n;i++)
	{
		xsum1^=buffer[i];
		xsum2+=buffer[i];
	}
	*x1=xsum1;
	*x2=xsum2;
}
/*
  Fletcher's 16-bit checksum
*/
void packet_fletcher16x(unsigned char *data, int len,unsigned char *xsum1,unsigned char *xsum2)
{
   unsigned short sum1,sum2;
   sum1 = 0xff;
   sum2 = 0xff;

   while (len)
   {
      int tlen = len > 21 ? 21 : len;
      len -= tlen;
      do
      {
         sum1 += *data++;
         sum2 += sum1;
      }
      while (--tlen);
      sum1 = (sum1 & 0xff) + (sum1 >> 8);
      sum2 = (sum2 & 0xff) + (sum2 >> 8);
   }
   // Second reduction step to reduce sums to 8 bits
   sum1 = (sum1 & 0xff) + (sum1 >> 8);
   sum2 = (sum2 & 0xff) + (sum2 >> 8);

	*xsum1 = sum1;
	*xsum2 = sum2;
}

void prettyprintblock(unsigned char *buffer,unsigned long int start)
{
	unsigned long int tmp;
	for(unsigned int i=0;i<16;i++)
	{
		tmp = start + i*32;
		printf_P(PSTR("%04X%04X \t"),(unsigned short)(tmp>>16),(unsigned short)tmp);
		for(unsigned int j=0;j<32;j++)
			printf_P(PSTR("%02X "),buffer[i*32+j]);
		tmp = start + i*32 + 31;
		printf_P(PSTR(" %04X%04X\r"),(unsigned short)(tmp>>16),(unsigned short)tmp);

	}
}


