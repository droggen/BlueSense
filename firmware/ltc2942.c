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

/*
	File: ltc2942
	
	LTC2942 functions
	
	This library interfaces with the LTC2942 gas gauge which allows to measure charge flowing through the battery.
	
	*Dependencies*
	
	* I2C library
	
	*Key functions*
	
	The key functions are:
	
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

	
	*Usage in interrupts*
	
	None of the functions are suitable to be called from interrupts. Doing so will crash the application.
	
*/

unsigned char __ltc2942_prescaler=0;		// This variable mirrors the prescaler set with ltc2942_setprescaler

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
	ltc2942_setchargectr(32768);	// Set charge counter at mid-point
	
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
	
	charge = ltc2942_getchargectr();
	charge2 = (85*(1<<__ltc2942_prescaler)*(unsigned long)charge)/128;			// dq: charge difference in uAh
	
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

unsigned short ltc2942_gettemperature(void)
{
	unsigned char v[2];
	unsigned long temperature;
	i2c_readregs(LTC2942_ADDRESS,0x0C,2,v);
	//printf("temperature: %02x %02x\n",v[0],v[1]);
	temperature = v[0];
	temperature<<=8;
	temperature|=v[1];
	temperature *= 600 * 10;		// *10 to get decimal kelvin
	temperature/=65535;
	temperature-=2731;			// to decimal degree
	return temperature;
}

/******************************************************************************
	ltc2942_setadc_prescaler
*******************************************************************************	
	Setups the LTC2942 conversion of voltage and temperature (single, automatic or disabled)
	and the charge prescaler. 
	
	Note: no error checking on the I2C interface.
	
	Parameters:
		adc			-	2-bits indicating: 11=temperature&voltage conversion every second;
						10: single voltage conversion;
						01: single temperature conversion;
						00: no conversion (sleep).
					
		prescaler	-	3-bits indicating the prescaler value M between 1 and 128. M = 2^prescaler.
		
	
	Returns:
		0			-	Success
		nonzero		-	Error
*******************************************************************************/
/*unsigned char ltc2942_setadc_prescaler(unsigned char adc,unsigned char prescaler)
{
	unsigned char rv;
	
	// Sets adc conversion, prescaler, disables the AL#/CC pin, disables shutdown.
	rv = i2c_writereg(LTC2942_ADDRESS,0x01,((adc&0b11)<<6)|((prescaler&0b111)<<3));
	return rv;
}*/


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
	control&=0b11000000;	// Shutdown=0, AL#/CC disabled, preserve ADC
	control|=prescaler<<3;
	rv = i2c_writereg(LTC2942_ADDRESS,0x01,control);
	return rv;
}
/******************************************************************************
	function: ltc2942_getprescaler
*******************************************************************************	
	Returns the 3-bit prescaler value.
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
		c1			-	First charge reading in uAh
		c2			-	Second charge reading in uAh
		voltage		-	Voltage in millivolts
		ms			-	Time interval between first and second charge readings
	
	Returns:
		dq			-	Average power in mW
*******************************************************************************/
signed long ltc2942_getavgpower(unsigned long c1,unsigned long c2,unsigned short voltage,unsigned long ms)
{
	signed long dq = c2-c1;	// delta uAh

	signed long den;
	signed long pwr;
	
	den = dq*voltage;		// delta energy nWh
	pwr = den*36/10/ms;		// power in mW
	
	return pwr;
}





