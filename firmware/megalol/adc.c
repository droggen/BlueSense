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


/*
	file: ADC
	
	ADC conversion functions.
	
	The key functions are:
	
	* ADCSetAutoPowerOff:			Sets whether the ADC should automatically be powered off after a conversion
	* ADCSetPrescaler:				Sets the prescaler for the ADC conversions
	* ADCRead:						Blocking single-channel ADC conversion
	* ADCReadPoll:					Blocking single-channel ADC conversion using polling
	* ADCReadMulti: 				Blocking multiple channel ADC conversions	
	* ADCReadMultiCallback:			Non-blocking multiple channel ADC conversions with results provided to a user callback
	* ADCReadMultiCallbackBuffer:	Non-blocking multiple channel ADC conversions with results provided to a user callback via a user-provided buffer
	* ADCReadDoubleBuffered:		Blocking multiple channel ADC conversions with double buffering.
	* ADCWait:						Busy wait until an ongoing conversion is completed.
	* ADCIsRunning:					Indicates whether a conversion is ongoing
	
	Callback functions must be of type 'void cb(volatile unsigned short *)'. The callbacks are passed a pointer to a buffer containing the conversion results. 
	This buffer is either an internal buffer of this library when using ADCReadMultiCallback or the user supplied buffer when using ADCReadMultiCallbackBuffer.
	
	*Usage in interrupts*
	
	The non-blocking functions are suitable for use in interrupts: ADCReadMultiCallback, ADCReadMultiCallbackBuffer and ADCIsRunning.
	
	The blocking functions can also be used in the interrupts, however as they block until the conversion is done they will prevent the
	system from processing other tasks.
	
	
	*Notes*
	The behaviour is undefined when mixing ADCReadPoll and the other conversion functions. ADCReadPoll does not use a lock to 
	check whether another conversion is in progress. It is suitable if only a single thread performs conversions. 
	
	*Internals*
	
	Internally, all the converstions are building upon ADCReadMultiCallbackBuffer and are realised using interrupts, except ADCReadPoll. 
	
	The variable __adc_convtype is the state of the conversion logic:
	
	__adc_convtype=_ADC_CONVERSION_IDLE		-	No ongoing conversion (ADC is free)
	__adc_convtype=_ADC_CONVERSION_ONGOING	-	Ongoing conversion (ADC is performing a conversion)
*/


#include "cpu.h"
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <util/delay.h>
#include "adc.h"
#include "serial.h"
#include "helper.h"

#include "main.h"


unsigned char __adc_prescaler=ADCCONV_PRESCALER_128;
// Define ADCCONV_START with the desired pre-scaler, ADC Enable, ADC Start. Do NOT set the interrupt enable. Interrupts are set in ADCCONV_START_INT
#define ADCCONV_START 			(0b11000000|__adc_prescaler)				// ADEN, ADSC, prescaler
#define ADCCONV_START_INT		(ADCCONV_START|0b00001000)					// ADEN, ADSC, prescaler, ADIE

ADC_CALLBACK __adc_callback;							// Callback for chained conversion
unsigned char __adc_autopoweroff=0;						// auto power off: 0=no, 1=yes
volatile unsigned char __adc_convtype=0;				// 0: none, 1: ongoing
unsigned char __adc_channels;							// Bitmask indicating which channels to sample
volatile unsigned char __adc_channel_mask;				// Mask shifted left by 1 to check which channel to sample
volatile unsigned char __adc_channel_mux;				// ADC multiplexer channel to convert
volatile unsigned char __adc_channel_num;				// Entry number in __adc_buffer_conversion where to store results
volatile unsigned short *__adc_conversion_ptr;			// Pointer where conversion results are stored 
volatile unsigned short __adc_buffer_conversion[8];			// Internal buffer for conversion results
unsigned short __adc_dbuffer_conversion[2][8];					// double buffer
unsigned char __adc_dbuffer_primary=0;		// Sample buffer and process buffer


/************************************************************************************************************************************************************
*************************************************************************************************************************************************************
PUBLIC FUNCTIONS   PUBLIC FUNCTIONS   PUBLIC FUNCTIONS   PUBLIC FUNCTIONS   PUBLIC FUNCTIONS   PUBLIC FUNCTIONS   PUBLIC FUNCTIONS   PUBLIC FUNCTIONS   PUBLIC 
*************************************************************************************************************************************************************
************************************************************************************************************************************************************/

/******************************************************************************
   function: ADCSetAutoPowerOff
*******************************************************************************
	Sets whether the ADC should automatically be powered off after a conversion.
	
	The first conversion after a power-off takes 25 ADC cycles, whereas subsequent conversions take
	13 cycles.
	
	Parameters:
		autopoweroff		-		0 to disable automatic power-off, 1 otherwise.

******************************************************************************/
void ADCSetAutoPowerOff(unsigned char autopoweroff)
{
	__adc_autopoweroff = autopoweroff;
}
/******************************************************************************
   function: ADCSetPrescaler
*******************************************************************************
	Sets the prescaler for the ADC conversions. 
	
	This prescaler is used by all conversions. Use the macros ADCCONV_PRESCALER_xx 
	to specify the prescaler.
	
	The AVR documentation recommends to use an ADC clock between 50KHz and 200KHz to achieve the maximum 
	resolution. 
	
	Parameters:

      prescaler - The prescaler value
	  
	The following prescalers are available:
	
	* ADCCONV_PRESCALER_128
	* ADCCONV_PRESCALER_64
	* ADCCONV_PRESCALER_32
	* ADCCONV_PRESCALER_16
	* ADCCONV_PRESCALER_8
	* ADCCONV_PRESCALER_4
	* ADCCONV_PRESCALER_2	

******************************************************************************/
void ADCSetPrescaler(unsigned char prescaler)
{
	__adc_prescaler = prescaler;
}

/******************************************************************************
	function: ADCRead
*******************************************************************************	
	Blocking single-channel ADC conversion.
	
	Triggers a read of an ADC channel and returns after the conversion is completed.
	If an ongoing conversion is in progress, waits for it to complete before starting
	the new conversion.
	
	This is not suitable for use in interrupts.
	
	Parameters:
		channel		-	channel to convert: 0 to 7
		intvref		-	1: 2.56V internal voltage reference; 0: AVCC voltage reference 
						Note: intvref is currently not used
						
	Returns:
		Conversion result
******************************************************************************/
unsigned short ADCRead(unsigned char channel,unsigned char intvref)
{
	unsigned short buffer;
	// Wait until a conversion could be started
	while(ADCReadMultiCallbackBuffer(1<<channel,0,&buffer))
	{
		// As ADCReadMultiCallback disables interrupts, repeatedly calling this function slow-downs interrupt-driven operations.
		// Therefore we wait a reasonable amount of time for a single conversion. Assuming 200kHz max frq, 13 clocks/conversion: 65uS/conversion -> wait 33uS_delay_us(_ADC_WAITCONV);
		_delay_us(_ADC_WAITCONV);
	}
	
	// Here the conversion has started
	
	// Wait for the conversion to complete
	// Note between the end of a conversion and this function actually detecting the end of a conversion
	// another conversion could be started (e.g. by a timer interrupt). 
	// This means that the latency of this function could be higher than expected.
	ADCWait();	

	// Result is now in the user buffer
	return buffer;
}

/******************************************************************************
	function: ADCReadPoll
*******************************************************************************	
	Blocking single-channel ADC conversion using polling.
	
	Triggers a read of an ADC channel and returns after the conversion is completed.
	
	This function uses polling to wait for the conversion to complete and return the conversion result.
	
	This function does not check if there is an ongoing conversion and therefore it should not be mixed
	with the other conversion functions provided by this library. 
	
	This function can be used in interrupts but care must be taken as it blocks until the conversion is completed.
	
	Parameters:
		channel		-	channel to convert: 0 to 7

	Returns:
		Conversion result
******************************************************************************/
unsigned short ADCReadPoll(unsigned char channel)
{
	ADMUX = 0x40 | channel;		// Vref=VCC

	// Start the conversion 	
	ADCSRA = ADCCONV_START;
	while(ADCSRA&0x40);
	
	if(__adc_autopoweroff)
		ADCSRA = 0x00;

	return ADCW;
}

/******************************************************************************
	function ADCReadMulti
*******************************************************************************
	Blocking multiple channel ADC conversions
	
	Triggers a read of multiple ADC channels and returns after the conversion is completed.
	If an ongoing conversion is in progress, waits for it to complete before starting
	the new conversion.
	
	This is not suitable for use in interrupts.
	
	Parameters:
		channels	-		bitmask indicating which channel to convert. E.g. 0b01000011 converts ADC channel 0, 1 and 6.
		v			-		buffer large enough to hold conversion results (maximum size 8)	
******************************************************************************/
void ADCReadMulti(unsigned char channels,unsigned short *v)
{
	// Wait until a conversion could be started
	while(ADCReadMultiCallbackBuffer(channels,0,v))
	{
		// As ADCReadMultiCallback disables interrupts, repeatedly calling this function slow-downs interrupt-driven operations.
		// Therefore we wait a reasonable amount of time for a single conversion. 
		_delay_us(_ADC_WAITCONV);
	}
	
	// Here the conversion has started
	
	// Wait for the conversion to complete
	// Note between the end of a conversion and this function actually detecting the end of a conversion
	// another conversion could be started (e.g. by a timer interrupt). 
	// This means that the latency of this function could be higher than expected.
	ADCWait();	

	// Result is now in the user buffer
}
/******************************************************************************
	function: ADCReadMultiCallback
*******************************************************************************
	Non-blocking multiple channel ADC conversions with results provided to a user callback.
	
	Starts the conversion of multiple ADC channels, calls a callback once completed 
	with a pointer to a buffer holding the results. 
	The data in this buffer must be consumed by the callback, as subsequent calls
	to this library may change the buffer content.
	
	This function returns immediately with an error if there is already an ongoing conversion.
	
	This function can be used in interrupts.
	
	Parameters:
		channels	-		bitmask indicating which channel to convert. E.g. 0b01000011 converts ADC channel 0, 1 and 6.
		callback	-		function to be called at the end of the conversion. 
							If the callback is null then no callback is  called and the user code must wait for the end of the conversion with ADCWait.
		buffer		-		Buffer where the conversion results are stored
	
	Returns:
		0		-	Conversion started
		1		-	Error: ongoing conversion
		2		-	Error: no conversion requested
******************************************************************************/
unsigned char ADCReadMultiCallback(unsigned char channels,ADC_CALLBACK callback)
{
	// Call the internal conversion function with an internal buffer to hold the results
	return ADCReadMultiCallbackBuffer(channels,callback,__adc_buffer_conversion);
}

/******************************************************************************
	function: ADCReadMultiCallbackBuffer
*******************************************************************************
	Non-blocking multiple channel ADC conversions with results provided to a user callback via a user-provided buffer.
	
	Starts the conversion of multiple ADC channels, calls a callback once completed 
	with a pointer to a user-supplied buffer holding the results. 
	
	This function returns immediately with an error if there is already an ongoing conversion.
	
	This function can be used in interrupts.
	
	Parameters:
		channels	-		bitmask indicating which channel to convert. E.g. 0b01000011 converts ADC channel 0, 1 and 6.
		callback	-		function to be called at the end of the conversion. 
							If the callback is null then no callback is  called and the user code must wait for the end of the conversion with ADCWait.
		buffer		-		Buffer where the conversion results are stored; this is the buffer which will be passed to the user callback upon completion.
	
	Returns:
		0		-	Conversion started
		1		-	Error: ongoing conversion
		2		-	Error: no conversion requested
******************************************************************************/
unsigned char ADCReadMultiCallbackBuffer(unsigned char channels,ADC_CALLBACK callback,volatile unsigned short *buffer)
{
	// We block interrupts throughout this function, to avoid an interrupt to trigger a conversion while this function is called.
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		// If ongoing conversion return error
		if(__adc_convtype!=_ADC_CONVERSION_IDLE)
			return 1;
	
		// If no conversion requested, return error
		if(channels==0)
			return 2;
		
		// Initialise the variables to sequence the conversion
		__adc_channels=channels;
		__adc_channel_num=0;
		__adc_channel_mux=0;
		__adc_channel_mask=1;
		__adc_conversion_ptr=buffer;
		
		
		//printf("Channels: %d\n",__adc_channels);
		
		// Store the callback to call when the conversion is done
		__adc_callback = callback;
		
		// Call the internal logic of the interrupt vector; as __adc_convtype is still _ADC_CONVERSION_IDLE, _adc_ivect will know this is the first call in a sequence of conversions
		_adc_ivect();
	}
	return 0;
}


/******************************************************************************
	function: ADCReadDoubleBuffered
*******************************************************************************	
	Blocking multiple channel ADC conversions with double buffering.
	
	This function returns the value of the previously launched ADC conversion, starts 
	a new conversion, and returns immediately. 
	This function is suitable when regular sampling is performed. 
	Therefore the data returned by this function lags by one function call 
	(1/samplerate, if regularly called), and allows to process incoming data while new 
	data is sampled in the background.
	
	If an ongoing conversion is in progress ADC this function blocks until
	that conversion is completed.
	
	This is not suitable for use in interrupts.
	
	Parameters:
		channels	-	bitmask indicating which channel to sample
	
	Return:	
		Pointer to a 8-long buffer comprising the results 
******************************************************************************/
unsigned short *ADCReadDoubleBuffered(unsigned char channels)
{
	unsigned short *b;

	// Wait for the conversion to complete
	// Note between the end of a conversion and this function actually detecting the end of a conversion
	// another conversion could be started (e.g. by a timer interrupt). 
	// This means that the latency of this function could be higher than expected.
	ADCWait();	
	
	// switch the primary and background buffer
	b=__adc_dbuffer_conversion[__adc_dbuffer_primary];	
	__adc_dbuffer_primary=1-__adc_dbuffer_primary;
	
	
	// Wait until a conversion could be started
	while(ADCReadMultiCallbackBuffer(channels,0,__adc_dbuffer_conversion[__adc_dbuffer_primary]))
	{
		// As ADCReadMultiCallback disables interrupts, repeatedly calling this function slow-downs interrupt-driven operations.
		// Therefore we wait a reasonable amount of time for a single conversion. Assuming 200kHz max frq, 13 clocks/conversion: 65uS/conversion -> wait 33uS_delay_us(_ADC_WAITCONV);
		_delay_us(_ADC_WAITCONV);
	}
	
	// Here the conversion has started
	
	// Return past conversion buffer
	return b;		
}


/******************************************************************************
	function: ADCWait
*******************************************************************************
	Busy wait until an ongoing conversion is completed.
******************************************************************************/
void ADCWait(void)
{
	while(__adc_convtype);
}
/******************************************************************************
	Function: ADCIsRunning
*******************************************************************************
	Indicates whether a conversion is ongoing.
	
	This function can be used in interrupts.

	Returns:
		0			-	No conversion is ongoing
		nonzero		-	A conversion is ongoing
******************************************************************************/
unsigned char ADCIsRunning(void)
{
	return __adc_convtype;
}


/************************************************************************************************************************************************************
*************************************************************************************************************************************************************
INTERNAL FUNCTIONS   INTERNAL FUNCTIONS   INTERNAL FUNCTIONS   INTERNAL FUNCTIONS   INTERNAL FUNCTIONS   INTERNAL FUNCTIONS   INTERNAL FUNCTIONS   INTERNAL 
*************************************************************************************************************************************************************
************************************************************************************************************************************************************/


/******************************************************************************
	ADC_vect
*******************************************************************************
	ADC interrupt handler. 
		
	The ADC logic is in _adc_ivect and separate from the ADC interrupt handler because:
	 - _adc_ivect is used to initialise the conversion and receive conversion results; 
	 - Calling the ISR would return with a RETI which would re-enable interrupts which is undersired.
******************************************************************************/
ISR(ADC_vect)
{	
	// Call the internal logic of the interrupt vector
	_adc_ivect();	
}

/******************************************************************************
	_adc_ivect
*******************************************************************************
	Internal logic for the ADC conversion interrupt. 
	
	This function is called:
	- From the ADC ISR during the conversion process. In this case it must copy the conversion result to the result buffer and initiate the next conversion
	- From ADCReadMultiCallback to initialise internal variables and start the conversion process
		
******************************************************************************/
void _adc_ivect(void)
{
	//dbg_fputchar_nonblock('I',0);

	// Check if we are called as a result of a conversion completion (_ADC_CONVERSION_ONGOING) or the first time to initiate the first conversion (_ADC_CONVERSION_IDLE)
	if(__adc_convtype==_ADC_CONVERSION_IDLE)
	{
		// The first time a conversion is initiated this function is called with _ADC_CONVERSION_IDLE
		// This indicates no conversion result is available, it has to be initiated, and the state must change to _ADC_CONVERSION_ONGOING
		__adc_convtype=_ADC_CONVERSION_ONGOING;
		//dbg_fputchar_nonblock('i',0);
	}
	else
	{
		// Store the conversion result in the buffer at location __adc_channel_num
		__adc_conversion_ptr[__adc_channel_num] = ADCW;	
		//dbg_fputchar_nonblock('o',0);
		
		// Go to next channel: increment channel number
		__adc_channel_num++;
	}

	// Iterate until we find the next channel to convert. If __adc_channels&__adc_channels_mask is zero then we try the next channel
	while(!(__adc_channels&__adc_channel_mask))
	{
		// Mask to next channel: shift bitmask by 1
		__adc_channel_mask<<=1;
		// Increment ADC multiplexer channel
		__adc_channel_mux++;
		//dbg_fputchar_nonblock('n',0);
		
		// If the channel mask became 0 then there are no more channels to convert
		if(__adc_channel_mask==0)
		{
			// Clear __adc_convtype to allow new conversions in callback
			__adc_convtype = _ADC_CONVERSION_IDLE;
			
			// Call the user callback with the results, if it is provided
			if(__adc_callback)
				__adc_callback(__adc_conversion_ptr);	
		
			// The user callback could trigger a new conversion. 
			// We check if a conversion was triggered by looking at bits indicating a conversion has started or an interrupt will be called. Otherwise we deactivate the ADC to save power.
			if(__adc_autopoweroff)
				if( !((ADCSRA&0x40) || (ADCSRA&0x10)) )
					ADCSRA = 0x00;
			//dbg_fputchar_nonblock('r',0);
			return;
		}			
	}
	
	//dbg_fputchar_nonblock('C',0);
	// At this stage __adc_channel_num is the number of the channel to convert

	// Select the multiplexer channel and voltage reference
	ADMUX = 0x40 | __adc_channel_mux;
	
	// Start the conversion with interrupt 	
	ADCSRA = ADCCONV_START_INT;			
	
	// Mask to next channel: shift bitmask by 1, increment channel mux by 1
	__adc_channel_mask<<=1;
	__adc_channel_mux++;
}










