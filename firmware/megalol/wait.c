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
	file: wait
	
	This library provides functions related to timing, waiting, and issuing periodic callbacks.
	
	This library requires some of these functions to be called from a timer interrupt to update the internal logic:
	
	_tick_1024hz		-	This function must be called from an interrupt running at 1024Hz
	_timer_tick_hz		-	Optionally, this may be called once per second if a high-accuracy RTC with second resolution is available.
		
	The overall time is composed of the highly accurate second counter (if available), combined with the less 1024Hz timer for millisecond accuracy.
	
	This library allows to register user callbacks to be called at regular interval from the timer interrupts with a specified clock divider.
	The callbacks are divided in millisecond-accuracy callbacks (functions timer_register_callback and timer_unregister_callback) and second-accuracy
	callbacks (functions timer_register_slowcallback and timer_unregister_slowcallback).
	The slow callbacks are only available if the 1Hz tick is called (_timer_tick_hz). Slow callbacks are lighter on processor usage and should be preferred
	for task to be executed with a longer time interval which does not need millisecond accuracy.
	
	The functions timer_waitperiod_ms and timer_waitperiod_us allow to wait until a period of time or a multiple of the period has elapsed from the previous call.
	This can be used to schedule taks which must land on a periodic grid, such as sampling.
	
	
	The key functions are:
	
	* timer_init:						Initialise the timer subsystem with a given epoch.
	* timer_ms_get:						Return the time in millisecond since the epoch.
	* timer_us_get: 					Return the time in microsecond since the epoch.
	* timer_register_callback:			Register a callback that will be called at 1024Hz/(divider+1) Hz
	* timer_register_slowcallback:		Register a callback that will be called at 1Hz/(divider+1) Hz
	* timer_unregister_callback: 		Unregisters a callback
	* timer_unregister_slowcallback:	Unregisters a slow callback
	* timer_printcallbacks:				Prints the list of registered callbacks
	* timer_waitperiod_ms:				Wait until a a period of time - or a multiple of it - has elapsed from the previous call.
	* timer_waitperiod_us:				Wait untila a period of time - or a multiple of it - has elapsed from the previous call.
	
*/


#include "cpu.h"
#include "wait.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>
#include <util/delay.h>
#include "numeric.h"

/******************************************************************************
	Global variables
*******************************************************************************
	_timer_time_1_in_ms: 	second counter, stored in millisecond to speeup time read
	_timer_time_1024: 1/1024s counter, to be reset at each second if an RTC is 
										available
										This counter should not be higher than 1hr, otherwise the
										conversion to time will fail. This is not an issue if a
										1Hz timer is available. 
******************************************************************************/
volatile unsigned long _timer_time_1_in_ms=0;
volatile unsigned long _timer_time_1024=0;
volatile unsigned long _timer_lastmillisec=0;
volatile unsigned long long _timer_lastmicrosec=0;

// Timer callbacks: fixed-number of callbacks
unsigned char timer_numcallbacks=0;
TIMER_CALLBACK timer_callbacks[TIMER_NUMCALLBACKS];
unsigned char timer_numslowcallbacks=0;
TIMER_CALLBACK timer_slowcallbacks[TIMER_NUMCALLBACKS];



/******************************************************************************
	function: timer_init
*******************************************************************************
	Initialise the timer subsystem with a given epoch.
	
	The epoch is optional: set it to zero if the system does not have a battery-backed
	RTC.

	Parameters:
		epoch_sec 		-		Time in second from an arbitrary "zero time". 
								If the system has an absolute time reference (e.g. a battery backed RTC) use 
								the RTC time as the epoch, otherwise set to zero.	
******************************************************************************/
void timer_init(unsigned long epoch_sec)
{
	// Ensures an atomic change
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		// Initialise the state variables
		_timer_time_1_in_ms=epoch_sec*1000;
		_timer_time_1024=0;
		_timer_lastmillisec=0;
		_timer_lastmicrosec=0;
		
		// Clear counter and interrupt flags
		TCNT1=0;				// Clear counter, and in case the timer generated an interrupt during this initialisation process clear the interrupt flag manually
		TIFR1=0b00100111;		// Clear all interrupt flags
		
		// Reset callback counters
		for(unsigned char i=0;i<timer_numslowcallbacks;i++)
		{
			timer_slowcallbacks[i].counter=0;
		}
		for(unsigned char i=0;i<timer_numcallbacks;i++)
		{
			timer_callbacks[i].counter=0;
		}
		
		//system_led_set(0);		// Reset the lifesign led
	}
}

/*
void wait_ms(unsigned int ms)
{

	unsigned int ocr, counter=0;
	unsigned char countL, countH;

	TCCR1A = 0x00;
	//if(ms > 65500/8)
	//	ms = 65500/8;
	//ocr = 8*ms; 		// Approx delay in ms with 1024 prescaler
	if(ms>65500/12)
		ms=65500/12;
	ocr = 12*ms;

	TCNT1H = 0;
	TCNT1L = 0;

	TIMSK1 = 0x00;		// no interrupt
	TCCR1B = 0x05;		// normal, prescaler=1024, start timer

	while(counter<ocr)
	{
		countL = TCNT1L;
		countH = TCNT1H;
		counter = (((unsigned int)countH) << 8) | (unsigned int)countL;
	}
	TCCR1B = 0x00;  	// disable timer
}
*/

/*
*/
/*
void timer_start(void)
{
	TCCR1A = 0x00;

	TCNT1H = 0;
	TCNT1L = 0;

	TIMSK1 = 0x00;		// no interrupt
	TCCR1B = 0x05;		// normal, prescaler=1024, start timer	
}
*/
/*
	Return the elapsed time since timer_start was called, in ~milliseconds (exactly 1.024ms/unit)
*/
/*
unsigned int timer_elapsed(void)
{
	unsigned char countL, countH;
	unsigned int counter;
	countL = TCNT1L;
	countH = TCNT1H;
	counter = (((unsigned int)countH) << 8) | (unsigned int)countL;
	return counter/12;	
}
*/

/*
	No prescaler, counts the CPU cycles.
*/
/*
void timer_start_fast(void)
{
	TCCR1A = 0x00;

	TCNT1H = 0;
	TCNT1L = 0;

	TIMSK1 = 0x00;		// no interrupt
	TCCR1B = 0x01;		// normal, prescaler=1024, start timer	
}
*/
/*
	Returns the elapsed time in CPU cycles
*/
/*
unsigned int timer_elapsed_fast(void)
{
	unsigned char countL, countH;
	unsigned int counter;
	countL = TCNT1L;
	countH = TCNT1H;
	counter = (((unsigned int)countH) << 8) | (unsigned int)countL;
	return counter;	
}
*/
/*void timer_rtc_start(void)
{
	TCCR2A = 0x00;

	ASSR=0x20;

	TCNT2H = 0;
	TCNT2L = 0;

	TIMSK1 = 0x00;		// no interrupt
	TCCR2B = 0x05;		// normal, prescaler=1024, start timer	
}*/
/*
	Return the elapsed time since timer_start was called, in ~milliseconds (exactly 1.024ms/unit)
*/
/*unsigned int timer_rtc_elapsed(void)
{
	unsigned char countL, countH;
	unsigned int counter;
	countL = TCNT1L;
	countH = TCNT1H;
	counter = (((unsigned int)countH) << 8) | (unsigned int)countL;
	return counter/8;	
}
*/



/*
	Interrupt-driven timer functions 
*/
//volatile unsigned long int timer_millisecond;



/*
	Initializes timer 0 with approx a millisecond timer count.

	Override this function according to system clock
*/
/*void timer_ms_init(void)
{
	// Stop the timer
	TCCR0B = 0x00;		// Stop

	// Init the tick counter
	timer_millisecond = 0;

	// Set-up the timer and activate the timer interrupt		
	TCCR0A = 0x02;			// Clear timer on compare
	TCNT0 = 0;				// Clear timer
	OCR0A	=	46;			// CTC value, assuming 12MHz: 12MHz/(46+1)/256 -> 1.002ms
	TIMSK0 	= 0x02;		// Output compare A match interrupt
	TCCR0B 	= 0x04;		// Normal, prescaler = 256, start timer	
	

}*/
extern unsigned long cpu_time;
/******************************************************************************
	function: timer_ms_get_c
*******************************************************************************
	Return the time in millisecond since the epoch.
	
	The maximum time since the epoch is about 48.5 days or .
	The return value is guaranteed to be monotonic until the time counter wraps
	around after 48 days.
	
	Returns:
		Time in milliseconds since the epoch	
******************************************************************************/
unsigned long timer_ms_get_c(void)
{
	unsigned long t1,t1024;
	unsigned long t;
	
	// Copy current time atomically
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		t1 = _timer_time_1_in_ms;
		t1024 = _timer_time_1024;
	}
	
	// Convert 1/1024s into milliseconds using a fast asm implementation
	//t1024 *= 1000;
	//t1024 >>= 10;
	//t1024 = (t1024*1000)>>10;
	//t1024=shr_u32_10(t1024);
	//t1024=shr_u32_10(t1024*1000);
	t1024=u32_mul1000_shr_10(t1024);
	
	// Compose the elapsed time
	t = t1 + t1024;
	
	// Ensure monoticity of the returned time. 
	// Correct time if the elapsed time is lower than the last returned time.
	// This may (but is unlikely) happen if the 1024Hz timer is highly inaccurate.
	// In that case we return the last returned time.
	
	// Ensure monoticity
	if(t<_timer_lastmillisec)
		return _timer_lastmillisec;
	
	// Update the last returned time
	_timer_lastmillisec = t;	
	return t;
}
/******************************************************************************
	timer_us_get
*******************************************************************************
	Return the time in microsecond since the epoch.
	
	TODO: this function should return an unsigned long long to allow for 
	epochs older than 1 hour.
	
******************************************************************************/
unsigned long int timer_us_get_c(void)
{
	unsigned long t1,t1024;
	unsigned short tcnt;
	//unsigned long tcntus;
	unsigned long long t;
	//unsigned long t;
	
	// Copy current time 
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		tcnt = TCNT1;			// Copy first as counter keeps going on
		t1 = _timer_time_1_in_ms;
		t1024 = _timer_time_1024;		
	}
	
		
	
	// Convert t1024 to ms
	t1024=u32_mul1000_shr_10(t1024);
	
	// TCNT is at 11059200Hz. Convert tcnt to uS using approximate function
	// TCNT*1000000/11059200 in us. 
	// 
	tcnt=u16_mul5925_shr_16(tcnt);
	//tcntus = tcnt;
	//tcntus =tcntus*625/6912;
	
	// Compose the elapsed time
	t = t1*1000 + t1024*1000 + tcnt;
	//t = t1*1000 + t1024*1000 + tcntus;
	
	// Ensure monoticity of the returned time. 
	// Correct time if the elapsed time is lower than the last returned time.
	// This may (but is unlikely) happen if the 1024Hz timer is highly inaccurate.
	// In that case we return the last returned time.
	
	// Ensure monoticity
	if(t<_timer_lastmicrosec)
		return _timer_lastmicrosec;
	
	// Update the last returned time
	_timer_lastmicrosec = t;	
	return t;
}


/******************************************************************************
	function: timer_register_callback
*******************************************************************************
	                                            1024Hz
	Register a callback that will be called at --------- Hz.
	                                           divider+1
	       
	Parameters:
		callback		-	User callback
		divider			-	How often the callback must be called, i.e. 1024Hz/(divider+1) Hz.
		   
	Returns:
		-1				-	Can't register callback
		otherwise		-	Callback ID
	
******************************************************************************/
char timer_register_callback(unsigned char (*callback)(unsigned char),unsigned short divider)
{
	//printf("Register\n");
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		if(timer_numcallbacks>=TIMER_NUMCALLBACKS)
			return -1;
		timer_callbacks[timer_numcallbacks].callback=callback;
		timer_callbacks[timer_numcallbacks].counter=0;
		timer_callbacks[timer_numcallbacks].top=divider;
		timer_numcallbacks++;
		//printf("Now %d callbacks\n",timer_numcallbacks);
	}
		return timer_numcallbacks-1;
}
/******************************************************************************
	function: timer_register_slowcallback
*******************************************************************************
	                                             1Hz
	Register a callback that will be called at --------- Hz.
	                                           divider+1

	Slow callbacks are only available if the 1Hz tick is available (_timer_tick_hz), otherwise
	use timer_register_callback.

	Parameters:
		callback		-	User callback
		divider			-	How often the callback must be called, i.e. 1Hz/(divider+1) Hz.


	Returns:
		-1				-	Can't register callback
		otherwise		-	Callback ID
	
******************************************************************************/
char timer_register_slowcallback(unsigned char (*callback)(unsigned char),unsigned short divider)
{
	//printf("Register\n");
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		if(timer_numslowcallbacks>=TIMER_NUMCALLBACKS)
			return -1;
		timer_slowcallbacks[timer_numslowcallbacks].callback=callback;
		timer_slowcallbacks[timer_numslowcallbacks].counter=0;
		timer_slowcallbacks[timer_numslowcallbacks].top=divider;
		timer_numslowcallbacks++;
		//printf("Now %d callbacks\n",timer_numcallbacks);
	}
	return timer_numslowcallbacks-1;
}
/******************************************************************************
	function: timer_unregister_callback
*******************************************************************************
	Unregisters a callback.
	
	Parameters:
		callback		-		User callback to unregister
******************************************************************************/
void timer_unregister_callback(unsigned char (*callback)(unsigned char))
{
	//printf("Unregister cb %p\n",callback);
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		// Iterate all entries to find the callback
		for(unsigned char i=0;i<timer_numcallbacks;i++)
		{
			//printf("check %d: %p == %p?\n",i,timer_callbacks[i],callback);
			if(timer_callbacks[i].callback==callback)
			{
				//printf("found callback %d\n",i);
				// Found the callback - shift up the other callbacks
				for(unsigned char j=i;j<timer_numcallbacks-1;j++)
				{
					//printf("Move %d <- %d\n",j,j+1);
					timer_callbacks[j]=timer_callbacks[j+1];
				}
				timer_numcallbacks--;
				break;
			}
		}
	}
	//printf("Num callbacks: %d\n",timer_numcallbacks);
}
/******************************************************************************
	function: timer_unregister_slowcallback
*******************************************************************************
	Unregisters a slow callback.
	
	Parameters:
		callback		-		User callback to unregister	
******************************************************************************/
void timer_unregister_slowcallback(unsigned char (*callback)(unsigned char))
{
	//printf("Unregister cb %p\n",callback);
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		// Iterate all entries to find the callback
		for(unsigned char i=0;i<timer_numslowcallbacks;i++)
		{
			//printf("check %d: %p == %p?\n",i,timer_callbacks[i],callback);
			if(timer_slowcallbacks[i].callback==callback)
			{
				//printf("found callback %d\n",i);
				// Found the callback - shift up the other callbacks
				for(unsigned char j=i;j<timer_numslowcallbacks-1;j++)
				{
					//printf("Move %d <- %d\n",j,j+1);
					timer_slowcallbacks[j]=timer_slowcallbacks[j+1];
				}
				timer_numslowcallbacks--;
				break;
			}
		}
	}
	//printf("Num callbacks: %d\n",timer_numcallbacks);
}
/******************************************************************************
	function: timer_printcallbacks
*******************************************************************************
	Prints the list of registered callbacks.
	
	Parameters:
		f		-		FILE on which to print the information
******************************************************************************/
void timer_printcallbacks(FILE *f)
{
	fprintf_P(f,PSTR("Number of callbacks %d/%d\n"),timer_numcallbacks,TIMER_NUMCALLBACKS);
	for(unsigned char i=0;i<timer_numcallbacks;i++)
	{
		fprintf_P(f,PSTR("CB %d @%p %d/%d\n"),i,timer_callbacks[i].callback,timer_callbacks[i].counter,timer_callbacks[i].top);
	}
	fprintf_P(f,PSTR("Number of slow callbacks %d/%d\n"),timer_numslowcallbacks,TIMER_NUMCALLBACKS);
	for(unsigned char i=0;i<timer_numslowcallbacks;i++)
	{
		fprintf_P(f,PSTR("CB %d @%p %d/%d\n"),i,timer_slowcallbacks[i].callback,timer_slowcallbacks[i].counter,timer_slowcallbacks[i].top);
	}
}


/******************************************************************************
	function: _tick_hz
*******************************************************************************
	This function may be called from a RTC interrupt at 1Hz (optional).
	
	The underlying assumption is that this RTC 1Hz timer is more accurate than the 
	1024Hz timer derived from the processor clock.
	The overall time is composed of the highly accurate second counter, combined
	with the less 1024Hz timer for millisecond accuracy.
******************************************************************************/
void _timer_tick_hz(void)
{
	TCNT1=0;				// Clear counter, and in case the timer generated an interrupt during this initialisation process clear the interrupt flag manually
	TIFR1=0b00100111;		// Clear all interrupt flags



	// Increment the second counter
	_timer_time_1_in_ms+=1000;
	// Reset the 1/1024s timer.
	_timer_time_1024=0;
	
	// Process the callbacks
	for(unsigned char i=0;i<timer_numslowcallbacks;i++)
	{
		timer_slowcallbacks[i].counter++;
		if(timer_slowcallbacks[i].counter>timer_slowcallbacks[i].top)
		{
			timer_slowcallbacks[i].counter=0;
			timer_slowcallbacks[i].callback(0);
		}		
	}	
}



/******************************************************************************
	_timer_tick_1024hz
*******************************************************************************
	This function must be called from an interrupt routine every 1/1024 hz (mandatory)
	
	This may not be a very accurate clock (the 1 Hz clock aims to compensate for that).
	
	This routine also dispatches timer callbacks for downsampled versions of this clock	
	
******************************************************************************/
void _timer_tick_1024hz(void)
{
	_timer_time_1024++;
	
	// Process the callbacks
	for(unsigned char i=0;i<timer_numcallbacks;i++)
	{
		timer_callbacks[i].counter++;
		if(timer_callbacks[i].counter>timer_callbacks[i].top)
		{
			timer_callbacks[i].counter=0;
			timer_callbacks[i].callback(0);
		}		
	}	
}


/*unsigned long timer_waitperiod_ms(unsigned short p,WAITPERIOD *wp)
{
	unsigned long t;
	
	if(p==0)
		return timer_ms_get();
	
	// First call: initialise when is the next period
	if(*wp==0)
	{
		*wp = timer_ms_get();
	}
	
	// Set the wait deadline. This addition may wrap around
	*wp+=p;	
	
	// Identify closest next period
	while(timer_ms_get()>*wp)
		*wp+=p;
	
	// Wait until passed next period
	while( (t=timer_ms_get())<=*wp);	
	
	return t;	
}*/

/******************************************************************************
	function: timer_waitperiod_ms
*******************************************************************************
	Wait until a a period of time, or a multiple of it, has elapsed from the previous call.
		
	Prior to the first call to this function, wp must be set to 0.
	
	This function is time-wraparound-safe if the desired period and maximum
	time elapsed between each function call is lower than 0x80000000 ms (i.e. 24 days). 
		
	Parameters:
		p			-		Period in milliseconds
		wp			-		Pointer to WAITPERIOD, internally used to regularly
							schedule the waits.
							
	Returns:
		Current time in ms.		
******************************************************************************/
unsigned long timer_waitperiod_ms(unsigned short p,WAITPERIOD *wp)
{
	unsigned long t,wp2;
	
	// Current time
	t=timer_ms_get();
	
	// If wait delay is zero return immediately
	if(p==0)
		return t;
	
	// First call: initialise when is the next period
	if(*wp==0)
		*wp = t;
	
	
	// Set the wait deadline. This addition may wrap around
	wp2=*wp+p;
	
	//printf("wp: %lu wp2: %lu\n",*wp,wp2);
	
	// Identify closest next period. Logic is wrap-around safe assuming wait period or time between calls is lower than maxrange/2.
	// If wp2 is later than time then wp2-time is zero or a small number.
	// If wp2 is earlier than time, then wp2-time is larger some large value maxrange/2=0x80000000
	// If wp2 is equal to time, consider ok and go to wait.
	while(wp2-timer_ms_get()>=0x80000000)
		wp2+=p;
	
	// Wait until time passes the next period
	// If t is earlier than wp2 then the difference is larger than some large value maxrange/2=0x80000000
	// if t is equal or later than wp2, then the difference is zero or a small number.
	while( (t=timer_ms_get())-wp2>0x80000000 )
		_delay_us(10);
	
	// Store the next time into the user-provided pointer
	*wp=wp2;
	return t;	
}

/*unsigned long timer_waitperiod_us_old(unsigned long p,WAITPERIOD *wp)
{
	unsigned long t;
	
	return 0;
	return timer_us_get();
	
	if(p<50)											// Benchmarks show ~30uS minimum delay.
		return timer_us_get();
	
	// First call: initialise when is the next period
	if(*wp==0)
	{
		*wp = timer_us_get();
	}
	
	// Set wait deadline
	*wp+=p;	
	
	// Identify closest next period
	while(timer_us_get()>*wp)
		*wp+=p;
	
	// Wait until passed next period
	//while( (t=timer_us_get())<=*wp);	
	while( (t=timer_us_get())<=*wp)
		_delay_us(50);
	
	return t;	
}*/
/******************************************************************************
	function: timer_waitperiod_us
*******************************************************************************
	Wait until a a period of time, or a multiple of it, has elapsed from the previous call.
		
	Prior to the first call to this function, wp must be set to 0.
	
	This function is time-wraparound-safe if the desired period and maximum
	time elapsed between each function call is lower than 0x80000000 us (i.e. 35 minutes). 
	
	This function returns immediately for delays lower than 50uS.
	This function may not be sufficiently accurate for delays below about 150us.
	
	Parameters:
		p			-		Period in microseconds
		wp			-		Pointer to WAITPERIOD, internally used to regularly
							schedule the waits.
							
	Returns:
		Current time in us.
******************************************************************************/
unsigned long timer_waitperiod_us(unsigned long p,WAITPERIOD *wp)
{
	unsigned long t,wp2;
	
	// Current time
	t = timer_us_get();
	
	// Benchmarks show ~30uS minimum delay at 11MHz.
	// Return immediately for delays lower than 50uS, as the while loops to define the next period would be too slow
	if(p<50)											
		return t;
	
	// First call: initialise when is the next period
	if(*wp==0)
		*wp = t;
	
	// Set the wait deadline. This addition may wrap around
	wp2=*wp+p;
	
	// Identify closest next period. Logic is wrap-around safe assuming wait period or time between calls is lower than maxrange/2.
	// If wp2 is later than time then wp2-time is zero or a small number.
	// If wp2 is earlier than time, then wp2-time is larger some large value maxrange/2=0x80000000
	// If wp2 is equal to time, consider ok and go to wait.
	while(wp2-timer_us_get()>=0x80000000)
		wp2+=p;
	
	// Wait until time passes the next period
	// If t is earlier than wp2 then the difference is larger than some large value maxrange/2=0x80000000
	// if t is equal or later than wp2, then the difference is zero or a small number.
	while( (t=timer_us_get())-wp2>0x80000000 )
		_delay_us(10);
	
	// Store the next time into the user-provided pointer
	*wp=wp2;
	return t;	
}

/******************************************************************************
	function: timer_waitperiod_us_test
*******************************************************************************
	Tests of timer_waitperiod_us
******************************************************************************/
/*void timer_waitperiod_test(void)
{
	unsigned char n=64;
	unsigned long times[n];
	WAITPERIOD p;
	
	for(unsigned long period=1;period<=32000;period*=2)
	{
		printf("Test period %lu\n",period);
		
		p=0;
		for(unsigned char i=0;i<n;i++)
		{
			times[i] = timer_waitperiod_us(period,&p);			
		}
		//for(unsigned char i=0;i<n;i++)
			//printf("%lu ",times[i]);
		//printf("\n");
		for(unsigned char i=0;i<n;i++)
			printf("%lu ",times[i]-times[0]);
		printf("\n");
		for(unsigned char i=0;i<n-1;i++)
			printf("%lu ",times[i+1]-times[i]);
		printf("\n");
	}
	
}*/