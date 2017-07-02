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

#include "ltc2942.h"
#include "i2c.h"
#include "i2c_int.h"
#include "wait.h"

/*
	Convert last into mAh using prescaler
	Compute average power (using voltage)
	Reset counter to midrange if reaches top/bottom
*/


/*
	File: ltc2942
	
	LTC2942 functions
	
	This library interfaces with the LTC2942 gas gauge which allows to measure charge flowing through the battery.
	
	*Dependencies*
	
	* I2C library
	
	*Key functions*
	
	This library provides immediate functions which immediately carry out an action, 
	and functions which initiate background reads and return data read after these background reads.
	
	In order to use background functions, ltc2942_backgroundgetstate must be called at regular intervals, typically
	from a timer interrupt. 
		
	Prior to using any functions ltc2942_init must be called.
	
	The key immediate functions are:
	
	* ltc2942_init:					Initialise the LTC2942 to default settings.
	* ltc2942_getcharge:			Returns the charge counter in uAh.
	* ltc2942_getchargectr:			Returns the 16-bit accumulated charge counter.
	* ltc2942_setchargectr:			Sets the 16-bit accumulated charge counter.
	* ltc2942_getvoltage:			Returns the voltage on the battery lead in millivolts.
	* ltc2942_gettemperature:		Returns the LTC2942 temperature in decidegrees (1 unit=0.1°celsius).
	* ltc2942_setadc:				Setups the LTC2942 conversion of voltage and temperature and the charge prescaler. 	
	* ltc2942_setprescaler:			Setups the LTC2942 charge prescaler. 
	* ltc2942_printreg:				Prints the LTC2942 register states to a file.
	* ltc2942_getavgpower: 			Computes the average power consumption in mW

	The key background functions are:
	* ltc2942_backgroundgetstate:	Initiates a background read of the LTC2942 charge, voltage and temperature registers
									This function can be called for a timer interrupt to perform transparent background read of the battery status
	* ltc2942_last_charge:			Returns the charge counter in uAh.
	* ltc2942_last_chargectr:		Returns the 16-bit accumulated charge counter.
	* ltc2942_last_mV:				Returns the voltage on the battery lead in millivolts.
	* ltc2942_last_mA:				Returns the average current between two background reads in mA.
	* ltc2942_last_mW:				Returns the average power between two background reads in mW.
	* ltc2942_last_temperature:		Returns the LTC2942 temperature in decidegrees (1 unit=0.1°celsius).
	* ltc2942_last_strstatus		Returns a status string describing the battery mV, mA and mW.
	
	*Usage in interrupts*
	
	None of the immediate functions are suitable to be called from interrupts and will result in a crash, with exception of ltc2942_getavgpower. 
		
	The function ltc2942_backgroundgetstate can be called from an interrupt (typically a timer interrupt) to initiate a background
	read of the data. All the background functions can be called from interrupts.
	
*/

unsigned char __ltc2942_prescaler=0;					// This variable mirrors the prescaler P set with ltc2942_setprescaler
unsigned char __ltc2942_prescalerM=1;					// This variable mirrors the prescaler M set with ltc2942_setprescaler (M=2^P)
I2C_TRANSACTION __ltc2942_trans_selreg;					// I2C transaction to select register for background read
I2C_TRANSACTION __ltc2942_trans_read;					// I2C transaction to read registers for background read
I2C_TRANSACTION __ltc2942_trans_setctr;					// I2C transaction to set counter to midrange for background read
volatile unsigned long int _ltc2942_last_updatetime=0;	// Background read: time of read
volatile unsigned long _ltc2942_last_charge=0;			// Background read: charge (uAh)
volatile unsigned short _ltc2942_last_chargectr=0;		// Background read: charge counter (raw)
volatile unsigned short _ltc2942_last_mV=0;				// Background read: voltage 
volatile signed short _ltc2942_last_mW=0;				// Background read: average power in mW between two reads
volatile signed short _ltc2942_last_mA=0;				// Background read: average current in mA between two reads
volatile short _ltc2942_last_temperature;				// Background read: temperature
unsigned char _ltc2942_previousreadexists=0;			// Flag used to indicate whether a previous background read was performed; used to compute mA and mW when two reads are available.
char _ltc2924_batterytext[42];							// Holds a text description of the battery status.


/******************************************************************************
	function: ltc2942_init
*******************************************************************************	
	Initialise the LTC2942 to default settings: automatic conversion of voltage
	and temperature, highest charge resolution, charge counter set at mid-point 
	suitable to measure battery charge or discharge.
	
	With these settings the maximum charge/discharge that can be measured is 21mAh.
	
	Note: 
	no error checking on the I2C interface.
	
*******************************************************************************/
void ltc2942_init(void)
{
	ltc2942_setadc(0b11);			// Enable autoconversion of voltage and temperature
	ltc2942_setprescaler(0);		// Highest resolution P=0, M=1
	//ltc2942_setprescaler(3);		// P=3, M=8
	ltc2942_setchargectr(32768);	// Set charge counter at mid-point
	// Initialise the I2C transactions for background reads
	// Register selection transaction from register 2, which is accumulated charge MSB
	i2c_transaction_setup(&__ltc2942_trans_selreg,LTC2942_ADDRESS,I2C_WRITE,0,1);
	__ltc2942_trans_selreg.data[0]=0x02;			
	// Register read transaction for 12 registers
	i2c_transaction_setup(&__ltc2942_trans_read,LTC2942_ADDRESS,I2C_READ,1,12);	
	__ltc2942_trans_read.callback=__ltc2942_trans_read_done;
	// Transaction to set counter to midrange
	i2c_transaction_setup(&__ltc2942_trans_setctr,LTC2942_ADDRESS,I2C_WRITE,1,3);
	__ltc2942_trans_setctr.data[0] = 0x02; 	// Charge register
	__ltc2942_trans_setctr.data[1] = 0x80;
	__ltc2942_trans_setctr.data[2] = 0x00;
	_ltc2942_previousreadexists=0;
}

/******************************************************************************
	function: ltc2942_getchargectr
*******************************************************************************	
	Returns the 16-bit accumulated charge counter.
	
	Note: 
	no error checking on the I2C interface.
	
	Returns:
		Charge	-	16-bit accumulated charge 
*******************************************************************************/
unsigned short ltc2942_getchargectr(void)
{
	unsigned char v[2];
	unsigned short charge;
	
	i2c_readregs(LTC2942_ADDRESS,0x02,2,v);
	//printf("charge: %02x %02x\n",v[0],v[1]);
	charge = v[0];
	charge<<=8;
	charge|=v[1];
	return charge;
}

/******************************************************************************
	function: ltc2942_getcharge
*******************************************************************************	
	Returns the charge counter in uAh.
	This function takes into account the prescaler of the LTC2942 and converts
	the charge counter into uAh.
	
	The maximum value that this function returns is 5'570'475.
	

	
	Note: 
	no error checking on the I2C interface.
	
	Returns:
		Charge	-	Charge in uAh
*******************************************************************************/
unsigned long ltc2942_getcharge(void)
{
	unsigned short charge;	// charge counter
	unsigned long charge2;	// charge in uAh
	
	// Conversion formula:
	// qLSB=0.085mAh*M/128 with M the prescaler
	
	charge = ltc2942_getchargectr();
	//charge2 = (85*(1<<__ltc2942_prescaler)*(unsigned long)charge)/128;			// charge2: charge in uAh
	charge2 = (85*__ltc2942_prescalerM*(unsigned long)charge)/128;			// charge2: charge in uAh
	
	return charge2;
}


/******************************************************************************
	function: ltc2942_setchargectr
*******************************************************************************	
	Sets the 16-bit accumulated charge counter.
	
	Returns:
		0			-	Success
		nonzero		-	Error
*******************************************************************************/
unsigned char ltc2942_setchargectr(unsigned short charge)
{
	unsigned char rv;
	unsigned char v[2];
	
	v[0] = charge>>8;
	v[1] = (unsigned char) charge;
	
	rv = i2c_writeregs(LTC2942_ADDRESS,0x02,v,2);
		
	return rv;	
}

/******************************************************************************
	function: ltc2942_getvoltage
*******************************************************************************	
	Returns the voltage on the battery lead in millivolts.
	
	Note: 
	no error checking on the I2C interface.
	
	Returns:
		Voltage 	-	Voltage in millivolts
*******************************************************************************/
unsigned short ltc2942_getvoltage(void)
{
	unsigned char v[2];
	unsigned long voltage;
	i2c_readregs(LTC2942_ADDRESS,0x08,2,v);
	//printf("voltage: %02x %02x\n",v[0],v[1]);
	voltage = v[0];
	voltage<<=8;
	voltage|=v[1];
	voltage*=6000;
	//voltage>>=16;			// WARNING: should divide by 65535, but approximate by 65536
	voltage/=65535;
		
	return voltage;
}
/******************************************************************************
	function: ltc2942_gettemperature
*******************************************************************************	
	Returns the LTC2942 temperature in decidegrees (1 unit=0.1°celsius).

	Note: 
	no error checking on the I2C interface.
	
	Returns:
		Temperature[dd] -	Temperature in decidegrees (1 unit=0.1°celsius)
*******************************************************************************/

short ltc2942_gettemperature(void)
{
	unsigned char v[2];
	long temperature;
	i2c_readregs(LTC2942_ADDRESS,0x0C,2,v);
	//printf("temperature: %02x %02x\n",v[0],v[1]);
	temperature = v[0];
	temperature<<=8;
	temperature|=v[1];
	temperature *= 600 * 10;		// *10 to get decimal kelvin
	temperature/=65535;
	temperature-=2721;			// to decimal degree
	return temperature;
}


/******************************************************************************
	function: ltc2942_setadc
*******************************************************************************	
	Setups the LTC2942 ADC conversion of voltage and temperature between single, automatic or disabled.
	
	Parameters:
		adc			-	2-bits indicating: 
						11:temperature&voltage conversion every second;
						10: single voltage conversion;
						01: single temperature conversion;
						00: no conversion (sleep).
					
	
	
	Returns:
		0			-	Success
		nonzero		-	Error
*******************************************************************************/
unsigned char ltc2942_setadc(unsigned char adc)
{
	unsigned char rv;
	unsigned char control;
	
	rv = i2c_readreg(LTC2942_ADDRESS,0x01,&control);
	if(rv)
		return rv;
	adc&=0b11;
	control&=0b00111000;	// Shutdown=0, AL#/CC disabled, preserve prescaler
	control|=adc<<6;
	rv = i2c_writereg(LTC2942_ADDRESS,0x01,control);
	return rv;
}

/******************************************************************************
	function: ltc2942_setprescaler
*******************************************************************************	
	Setups the LTC2942 charge prescaler. 
	
	Parameters:				
		prescaler	-	3-bits indicating the prescaler value M between 1 and 128. M = 2^prescaler.
		
	
	Returns:
		0			-	Success
		nonzero		-	Error
*******************************************************************************/
unsigned char ltc2942_setprescaler(unsigned char prescaler)
{
	unsigned char rv;
	unsigned char control;
	
	rv = i2c_readreg(LTC2942_ADDRESS,0x01,&control);
	if(rv)
		return rv;
	prescaler&=0b111;
	__ltc2942_prescaler=prescaler;
	__ltc2942_prescalerM=(1<<__ltc2942_prescaler);
	control&=0b11000000;	// Shutdown=0, AL#/CC disabled, preserve ADC
	control|=prescaler<<3;
	rv = i2c_writereg(LTC2942_ADDRESS,0x01,control);
	return rv;
}
/******************************************************************************
	function: ltc2942_getprescaler
*******************************************************************************	
	Returns the 3-bit prescaler P value.
	The actual prescaler value M is betweeen 1 and 128: M=2^prescaler.
	
	This function returns a cached version of the prescaler set with 
	ltc2942_setprescaler.
	
	
	Returns:
		prescaler	-	3-bit prescaler value
*******************************************************************************/
unsigned char ltc2942_getprescaler(void)
{
	return __ltc2942_prescaler;
}

/*unsigned char ltc2942_setcontrol(unsigned char control)
{
	unsigned char rv;
	rv = i2c_writereg(LTC2942_ADDRESS,0x01,control);
	return rv;
}*/




/******************************************************************************
	function: ltc2942_printreg
*******************************************************************************	
	Prints the LTC2942 register states to a file.
	
	Parameters:				
		file	-	File on which the registers must be printed
		
	
	Returns:
		void
*******************************************************************************/
void ltc2942_printreg(FILE *file)
{
	unsigned char rv,v;
	fprintf_P(file,PSTR("LTC2942:\n"));
	for(unsigned int r=0;r<=0xf;r++)
	{
		rv = i2c_readreg(LTC2942_ADDRESS,r,&v);
		fprintf_P(file,PSTR("\t%02Xh: "),r);
		if(rv)
		{
			fprintf_P(file,PSTR("Error\n"));
		}
		fprintf_P(file,PSTR("%02Xh\n"),v);
	}
}

//unsigned short ltc2942_start


/******************************************************************************
	ltc2942_deltaQ
*******************************************************************************	
	Computes the charge delta between two readings of the accumulated charge
	counter, taking into account the prescaler.
	
	Parameters:				
		q1			-	First reading of the accumulated charge counter
		q2			-	Second reading of the accumulated charge counter
	
	Returns:
		dq			-	Delta charge in uAh
*******************************************************************************/
/*signed long ltc2942_deltaQ(unsigned short q1,unsigned short q2)
{
	signed long dq;

	dq = ((signed long)q2)-((signed long)q1);	// dq: uncalibrated charge difference
	dq = (85*(1<<__ltc2942_prescaler)*dq)/128;			// dq: charge difference in uAh
	return dq;
}*/



/******************************************************************************
	function: ltc2942_getavgpower
*******************************************************************************	
	Computes the average power consumption in mW between two readings of 
	the charge done after a time interval ms. 
	This function assumes that the battery voltage is approximately constant
	between the first and second charge measure. 
	
	Parameters:				
		c1			-	First charge reading in uAh (at a previous time)
		c2			-	Second charge reading in uAh (at current time)
		voltage		-	Voltage in millivolts
		ms			-	Time interval between first and second charge readings
	
	Returns:
		dq			-	Average power in mW
*******************************************************************************/
signed short ltc2942_getavgpower(unsigned long c1,unsigned long c2,unsigned short voltage,unsigned long ms)
{
	signed long dq = c2-c1;	// delta uAh

	signed long den;
	signed long pwr;
	
	den = dq*voltage;						// delta energy nWh
	pwr = den*36/10/((signed long)ms);		// power in mW
	
	return (signed short)pwr;
}


/******************************************************************************
	function: ltc2942_backgroundgetstate
*******************************************************************************	
	Call from user code or a timer interrupt to initiate a background read
	of the LTC2942 state registers (accumulated charge, voltage, temperature).
	
	The background read is completed when the callback __ltc2942_trans_read_done
	is called by the I2C engine.
	
	The results are available through the ltc2942_last_xxx functions.
	
	Parameters:
		None / not used
	Returns:
		Nothing.
	
******************************************************************************/
unsigned char ltc2942_backgroundgetstate(unsigned char)
{
	unsigned char r = i2c_transaction_queue(2,0,&__ltc2942_trans_selreg,&__ltc2942_trans_read);
	if(r)
	{
		// Failed to queue the transactions
	}	
	return 0;
}
/******************************************************************************
	function: ltc2942_last_updatetime
*******************************************************************************	
	Returns the time in ms when the last background read occurred.
	
	Parameters:
		None 
	Returns:
		Time in ms when a background read last occurred.
	
******************************************************************************/
unsigned long ltc2942_last_updatetime(void)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		return _ltc2942_last_updatetime;
	}
	return 0;
}
/******************************************************************************
	function: ltc2942_last_charge
*******************************************************************************	
	Returns the charge counter in uAh obtained during the last background read.
	
	This function takes into account the prescaler of the LTC2942 and converts
	the charge counter into uAh.
	
	The maximum value that this function returns is 5'570'475.

	Returns:
		Charge	-	Charge in uAh
*******************************************************************************/
unsigned long ltc2942_last_charge(void)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		return _ltc2942_last_charge;
	}
	return 0;
}
/******************************************************************************
	function: ltc2942_last_chargectr
*******************************************************************************	
	Returns the 16-bit accumulated charge counter obtained during the last 
	background read.

	Returns:
		Charge	-	16-bit accumulated charge 
*******************************************************************************/
unsigned short ltc2942_last_chargectr(void)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		return _ltc2942_last_chargectr;
	}
	return 0;
}
/******************************************************************************
	function: ltc2942_last_mV
*******************************************************************************	
	Returns the voltage on the battery lead in millivolts obtained during the last 
	background read.
	
	Returns:
		Voltage 	-	Voltage in millivolts
*******************************************************************************/
unsigned short ltc2942_last_mV(void)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		return _ltc2942_last_mV;
	}
	return 0;
}
/******************************************************************************
	function: ltc2942_last_temperature
*******************************************************************************	
	Returns the LTC2942 temperature in decidegrees (1 unit=0.1°celsius) 
	obtained during the last  background read.

	Returns:
		Temperature[dd] -	Temperature in decidegrees (1 unit=0.1°celsius)
*******************************************************************************/
short ltc2942_last_temperature(void)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		return _ltc2942_last_temperature;
	}
	return 0;
}
/******************************************************************************
	function: ltc2942_last_mW
*******************************************************************************	
	Returns the power used between two subsequent background reads in mW.

	Returns:
		Power	-	Power in mW
*******************************************************************************/
signed short ltc2942_last_mW(void)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		return _ltc2942_last_mW;
	}
	return 0;
}
/******************************************************************************
	function: ltc2942_last_mA
*******************************************************************************	
	Returns the power used between two subsequent background reads in mW.

	Returns:
		Power	-	Power in mW
*******************************************************************************/
signed short ltc2942_last_mA(void)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		return _ltc2942_last_mA;
	}
	return 0;
}

/******************************************************************************
	__ltc2942_trans_read_done
*******************************************************************************	
	Callback called by the I2C engine when an interrupt-read completes.
	Do not call from user code.
	
	This function has been benchmarked to take 375uS when division by 65535 
	is used for voltage and temperature conversion and 255uS when an 
	approximation by dividing by 65536 is used.
	
	
	Parameters:
		t		-	Current transaction (which will be __ltc2942_trans_read
	Returns:
		Battery information is stored in global variables.
	
******************************************************************************/
unsigned char __ltc2942_trans_read_done(I2C_TRANSACTION *t)
{
	// Store when the data was read
	unsigned long lasttime = _ltc2942_last_updatetime;
	_ltc2942_last_updatetime = timer_ms_get();
	
	// Mirror the data in user accessible variables
	// Voltage: requires a 32-bit temp
	unsigned long voltage=__ltc2942_trans_read.data[6];
	voltage<<=8;
	voltage|=__ltc2942_trans_read.data[7];
	voltage*=6000;
	voltage>>=16;			// WARNING: should divide by 65535, but approximate by 65536
	//voltage/=65535;	
	_ltc2942_last_mV=voltage;
	
	// Charge counter
	_ltc2942_last_chargectr = __ltc2942_trans_read.data[0];
	_ltc2942_last_chargectr<<=8;
	_ltc2942_last_chargectr|=__ltc2942_trans_read.data[1];
	
	// Charge counter in uAh
	// Conversion formula: qLSB=0.085mAh*M/128 with M the prescaler
	unsigned long lastcharge=_ltc2942_last_charge;
	//_ltc2942_last_charge = (85*(1<<__ltc2942_prescaler)*(unsigned long)_ltc2942_last_chargectr)/128;			// charge2: charge in uAh
	_ltc2942_last_charge = (85*__ltc2942_prescalerM*(unsigned long)_ltc2942_last_chargectr)/128;			// charge2: charge in uAh
	
	
	// Temperature: requires a 32-bit temp
	unsigned long temperature=__ltc2942_trans_read.data[10];
	temperature<<=8;
	temperature|=__ltc2942_trans_read.data[11];	
	temperature *= 600 * 10;		// *10 to get decimal kelvin
	temperature>>=16;				// WARNING: should divide by 65535, but approximate by 65536
	//temperature/=65535;	
	temperature-=2721;				// to decimal degree
	_ltc2942_last_temperature = temperature;
	
	// Only convert to power/current if a previous measurement of charge exists, otherwise leave unchanged the current/power.
	if(_ltc2942_previousreadexists)		
	{
		// Convert into current and power since last read. Negative: discharge
		signed long deltams = _ltc2942_last_updatetime-lasttime;
		signed long deltauAh = _ltc2942_last_charge-lastcharge;
		signed long mA;
		mA = deltauAh * 3600l / deltams;		// mA	
		_ltc2942_last_mA = (signed short)mA;
		_ltc2942_last_mW = ltc2942_getavgpower(lastcharge,_ltc2942_last_charge,_ltc2942_last_mV,deltams);
	}

	_ltc2942_previousreadexists=1;	// Indicate we have made a previous measurement of charge
	
	// Reset charge counter to midrange if getting close to top/bot
	if(_ltc2942_last_chargectr<1000 || _ltc2942_last_chargectr>64535)
	{
		unsigned char r = i2c_transaction_queue(1,0,&__ltc2942_trans_setctr);
		if(r)
		{
			// Failed to queue the transactions
		}	
		else
			_ltc2942_previousreadexists=0;			// Indicate there is no valid previous measurement of charge
	}
	

	
	return 0;
}

/******************************************************************************
	function: ltc2942_last_strstatus
*******************************************************************************	
	Returns a status string describing the battery mV, mA and mW.
	
	Maximum string buffer length: "V=-65535 mV; I=-65535 mA; P=-65535 mW"
	or 38 bytes + 1 null. 42 bytes.

	Returns:
		Power	-	Power in mW
*******************************************************************************/
char *ltc2942_last_strstatus(void)
{
	sprintf(_ltc2924_batterytext,"V=%d mV; I=%d mA; P=%d mW",ltc2942_last_mV(),ltc2942_last_mA(),ltc2942_last_mW());
	return _ltc2924_batterytext;
}


