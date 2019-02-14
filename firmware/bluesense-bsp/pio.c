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

/*
	file: pio
	
	Parallel I/O funcions
	
	The key functions are:
	
	* PIODigitalWrite:			
	* PIODigitalRead
	* PIOPinMode:				Set the IO mode of the pin
	
	
	
	*Pin numbers*
	
	This module offers an arduino-like interface to I/O pins. Each I/O pin is numbered, and the physical mapping to the actual AVR pin is handled by this module.
	
	Hardware version 7:
	
	--- Text	
	Pin#	SchematicName			AVRPin		Function
	0 		X_ADC0					PA0			Digital IO, ADC input
	1		X_ADC1					PA1			Digital IO, ADC input
	2		X_ADC2					PA2			Digital IO, ADC input
	3		X_ADC3					PA3			Digital IO, ADC input
	4		N/A
	5		X_AIN0					PB2			Digital IO, Analog comparator input
	6		X_AIN1					PB3			Digital IO, Analog comparator input
	---
	
	
	
	*Available modes*
	
	Each pin can be set in one of these modes: PIOMODE_OUTPUT, PIOMODE_INPUT, PIOMODE_INPUTPULLUP.
	
	When a pin is used for analog input, it must be configured as PIOMODE_INPUT first.
	
	
	*Hardware compatibility*
	
	This module is only implemented for HW v.7
	
	
*/

#include "cpu.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <stdio.h>
#include <string.h>

#include "main.h"
#include "init.h"
#include "pio.h"

/******************************************************************************
   function: PIOPinMode
*******************************************************************************
	Sets the pin mode.
	
	Modes are one of: PIOMODE_OUTPUT, PIOMODE_INPUT, PIOMODE_INPUTPULLUP.
	For analog input, use PIOMODE_INPUT.
	
	Parameters:
		pinnumber	-		Number of the pin
		mode		-		Mode of the pin
******************************************************************************/
void PIOPinMode(unsigned char pinnumber,PIOMODE mode)
{
	#if HWVER==7
	// Do nothing in case of wrong pin number
	if(pinnumber>6)
		return;
		
	volatile uint8_t *ddr;
	volatile uint8_t *port;
	//unsigned char init_ddr;
	//unsigned char init_port;
	unsigned char bm=1;
	
	if(pinnumber<=3)		
	{
		// Pins on port A (port 0, 1, 2, 3)
		ddr = &DDRA;				//	DDRA
		port = &PORTA;				//	PORTA	
		//init_ddr = init_ddra;
		//init_port = init_porta;
			
		bm<<=pinnumber;			// Create a bitmask: bm bitmask takes value 1 at position of pin		
	}
	if(pinnumber==4)
	{
		// Pin on port PA7
		ddr = &DDRA;				//	DDRA
		port = &PORTA;				//	PORTA	
		
		bm = 0b10000000;
	}
	if(pinnumber>=5 && pinnumber<=6)
	{
		// Pins on port B (port 5, 6)
		ddr = &DDRB;				//	DDRB
		port = &PORTB;				//	PORTB
		//init_ddr = init_ddrb;
		//init_port = init_portb;
		
		bm<<=(pinnumber-3);		// Create a bitmask: bm bitmask takes value 1 at position of pin		
	}	
	switch(mode)
	{
		case PIOMODE_OUTPUT:
			// Set DDRA bit to 1 (output)
			//*ddr = init_ddra|bm;
			*ddr = (*ddr)|bm;
			break;
		case PIOMODE_INPUT:
			// Set DDRA bit to 0 (input)
			//*ddr = init_ddr&(~bm);
			*ddr = (*ddr)&(~bm);
			// Set PORTA bit to 0 (no pull-up)
			//*port = init_port&(~bm);
			*port = (*port)&(~bm);
			break;
		case PIOMODE_INPUTPULLUP:
			// Set DDRA bit to 0 (input)
			//*ddr = init_ddr&(~bm);
			*ddr = (*ddr)&(~bm);
			// Set PORTA bit to 1 (pull-up)
			//*port = init_port|bm;
			*port = (*port)|bm;
			break;
		default:
			break;
	}
	#endif
}


/******************************************************************************
   function: PIODigitalWrite
*******************************************************************************
	Sets the state of a pin to 1 (set) or 0 (clear).
	
	The pin must have been previously set to the mode PIOMODE_OUTPUT. Behaviour
	is undefined otherwise.
		
	Parameters:
		pinnumber	-		Number of the pin
		set			-		Nonzero to set the pin to 1, zero to set the pin to 0.
******************************************************************************/
void PIODigitalWrite(unsigned char pinnumber,unsigned char set)
{
	#if HWVER==7
	// Do nothing in case of wrong pin number
	if(pinnumber>6)
		return;
		
	volatile uint8_t *port;
	unsigned char bm=1;
	if(pinnumber<=3)		
	{
		// Pins on port A (port 0, 1, 2, 3)
		port = &PORTA;				//	PORTA
		bm<<=pinnumber;				// Create a bitmask: bm bitmask takes value 1 at position of pin		
	}
	if(pinnumber==4)
	{
		// Pin on port PA7
		port = &PORTA;				//	PORTA			
		bm = 0b10000000;
	}
	if(pinnumber>=5 && pinnumber<=6)
	{
		// Pins on port B (port 5, 6)
		port = &PORTB;				// PORTB
		bm<<=(pinnumber-3);			// Create a bitmask: bm bitmask takes value 1 at position of pin		
	}
	if(set)
		*port = (*port)|bm;			// Set bit
	else
		*port = (*port)&(~bm);		// Clear bit
	#endif
}


/******************************************************************************
   function: PIODigitalRead
*******************************************************************************
	Reads the state of a pin.
	
	The pin must have been previously set to the mode PIOMODE_INPUT or 
	PIOMODE_INPUTPULLUP. Behaviour is undefined otherwise.
		
	Parameters:
		pinnumber	-		Number of the pin
	Returns:
		1 if the pin is pulled high, 0 if it is pulled low.
******************************************************************************/
unsigned char PIODigitalRead(unsigned char pinnumber)
{
	#if HWVER==7
	// Do nothing in case of wrong pin number
	if(pinnumber>6)
		return 0;
		
	volatile uint8_t *pin;
	unsigned char bm=1;
	if(pinnumber<=3)		
	{
		// Pins on port A (port 0, 1, 2, 3)
		pin = &PINA;				//	PINA			
		bm<<=pinnumber;				// Create a bitmask: bm bitmask takes value 1 at position of pin		
	}
	if(pinnumber==4)
	{
		// Pin on port PA7
		pin = &PINA;				//	PINA
		bm = 0b10000000;
	}
	if(pinnumber>=5 && pinnumber<=6)
	{
		// Pins on port B (port 5, 6)
		pin = &PINB;				// PINB
		bm<<=(pinnumber-3);		// Create a bitmask: bm bitmask takes value 1 at position of pin		
	}
	return ((*pin) & bm) ? 1 : 0;
	
	#endif
	return 0;
}

	
	

