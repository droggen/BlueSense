/*
   MEGALOL - ATmega LOw level Library
	ADC Module
   Copyright (C) 2009-2016:
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





#ifndef __ADC_H
#define __ADC_H



#define ADCCONV_PRESCALER_128		7
#define ADCCONV_PRESCALER_64		6
#define ADCCONV_PRESCALER_32		5
#define ADCCONV_PRESCALER_16		4
#define ADCCONV_PRESCALER_8			3
#define ADCCONV_PRESCALER_4			2
#define ADCCONV_PRESCALER_2			1

#define _ADC_CONVERSION_IDLE		0
#define _ADC_CONVERSION_ONGOING		1

// Define ADCCONV_START with the desired pre-scaler, ADC Enable, ADC Start. Do NOT set the interrupt enable. Interrupts are set in ADCCONV_START_INT
#define ADCCONV_START 			(0b11000000|__adc_prescaler)				// ADEN, ADSC, prescaler
#define ADCCONV_START_INT		(ADCCONV_START|0b00001000)					// ADEN, ADSC, prescaler, ADIE

// Assuming 200kHz max frq, 13 clocks/conversion: 65uS/conversion -> wait 33uS, _delay_us(33);
// Choosing a lower value speeds up the conversion but slows down the overall system
#define _ADC_WAITCONV				33
//#define _ADC_WAITCONV				10


typedef void(*ADC_CALLBACK)(volatile unsigned short*);			// Callback type
extern ADC_CALLBACK __adc_callback;								// Callback for the last triggered conversion
extern volatile unsigned char __adc_convtype;					// 0: none, 1: ongoing

extern unsigned char __adc_prescaler;

// Public functions
void ADCDeinit(void);
void ADCSetAutoPowerOff(unsigned char autopoweroff);
void ADCSetPrescaler(unsigned char prescaler);
unsigned short ADCRead(unsigned char channel,unsigned char intvref);
unsigned short ADCReadPoll(unsigned char channel);
void ADCReadMulti(unsigned char channels,unsigned short *v);
unsigned char ADCReadMultiCallback(unsigned char channels,ADC_CALLBACK callback);
unsigned char ADCReadMultiCallbackBuffer(unsigned char channels,ADC_CALLBACK callback,volatile unsigned short *buffer);
unsigned short *ADCReadDoubleBuffered(unsigned char channels);
unsigned char ADCIsRunning(void);
void ADCWait(void);

// Internal functions
void _adc_ivect(void);




#endif


