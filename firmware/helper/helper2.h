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



#ifndef __HELPER2_H
#define __HELPER2



void data_checksum(unsigned char *buffer,unsigned short n,unsigned char *x1,unsigned char *x2);
void packet_fletcher16x(unsigned char *data, int len,unsigned char *xsum1,unsigned char *xsum2);
void prettyprintblock(unsigned char *buffer,unsigned long int start);

#endif
