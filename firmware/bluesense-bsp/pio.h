/*
	MEGALOL - ATmega LOw level Library
	Parallel I/O Module
	Copyright (C) 2019:
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

#ifndef __PIO_H 
#define __PIO_H

enum PIOMODE {
	PIOMODE_OUTPUT, PIOMODE_INPUT, PIOMODE_INPUTPULLUP
};

void PIOPinMode(unsigned char pinnumber,PIOMODE mode);
void PIODigitalWrite(unsigned char pinnumber,unsigned char set);
unsigned char PIODigitalRead();

#endif