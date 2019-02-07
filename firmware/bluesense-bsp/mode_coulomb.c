#include "cpu.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/eeprom.h>
#include <util/atomic.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "main.h"
#include "adc.h"
#include "serial.h"
#include "i2c.h"
#include "ds3232.h"
#include "rn41.h"
#include "mpu.h"
#include "mpu_test.h"
#include "pkt.h"
#include "wait.h"
#include "init.h"
#include "lcd.h"
#include "fb.h"
#include "uiconfig.h"
#include "helper.h"
#include "i2c_internal.h"
#include "system.h"
#include "pkt.h"

#include "mode_sample_adc.h"

#include "commandset.h"
#include "mode_global.h"
#include "mode_coulomb.h"
#include "ltc2942.h"
#include "mode.h"

#if ENABLEMODECOULOMB==1

const char help_cq[] PROGMEM ="Q,<q>: sets counter charge";
const char help_cr[] PROGMEM ="R: reset accumulated charge";
const char help_cp[] PROGMEM ="P,<p>: with p on 3 bits, sets the prescaler to 2^p";
const char help_ca[] PROGMEM ="A,<a>: temp/voltage conversion: 3=auto, 2=single voltage, 1=single temp, 0=off";


signed long acccharge=0,accpwr=0,accuah=0,acctime=0;
unsigned long initcharge=0;

const COMMANDPARSER CommandParsersCoulomb[] =
{ 
	{'H', CommandParserHelp,help_h},
	{'Q', CommandParserCoulombCharge,help_cq},
	{'P', CommandParserCoulombPrescaler,help_cp},
	{'A', CommandParserCoulombADC,help_ca},
	{'R', CommandParserCoulombReset,help_cr},
	{'!', CommandParserQuit,help_quit}
};
unsigned char CommandParsersCoulombNum=sizeof(CommandParsersCoulomb)/sizeof(COMMANDPARSER);

unsigned char CommandParserCoulomb(char *buffer,unsigned char size)
{
	CommandChangeMode(APP_MODE_COULOMB);
		
	return 0;
}
unsigned char CommandParserCoulombReset(char *buffer,unsigned char size)
{
	acccharge=0;
	accpwr=0;
	accuah=0;
	acctime=0;
	initcharge = ltc2942_getcharge();
	
	return 0;
}

unsigned char CommandParserCoulombADC(char *buffer,unsigned char size)
{
	unsigned char rv;
	int adc;
	
	rv = ParseCommaGetInt(buffer,1,&adc);
	if(rv)
	{
		return 2;
	}
	if(ltc2942_setadc(adc))
		return 1;
	return 0;
}
unsigned char CommandParserCoulombPrescaler(char *buffer,unsigned char size)
{
	unsigned char rv;
	int prescaler;
	
	rv = ParseCommaGetInt(buffer,1,&prescaler);
	if(rv)
	{
		return 2;
	}
	if(ltc2942_setprescaler(prescaler))
		return 1;
	return 0;
}

unsigned char CommandParserCoulombCharge(char *buffer,unsigned char size)
{
	unsigned char rv;
	int charge;
	
	
	rv = ParseCommaGetInt(buffer,1,&charge);
	if(rv)
	{
		return 2;
	}
				
	if(ltc2942_setchargectr(charge))
		return 1;		
	
	return 0;
}

/*
void coulomb(unsigned short q1,unsigned short q2,unsigned short voltage,unsigned short elapsed,unsigned char prescaler)
{
	signed long dq;

	dq = q2-q1;									// dq: uncalibrated charge difference
	dq = (85*(1<<prescaler)*dq)/128;			// dq: charge difference in uAh
	
	signed long dpwr;
	dpwr = dq*voltage;							// dpwr: 
	
	
	
}*/

void mode_coulomb(void)
{
	WAITPERIOD p=0;
	
	fprintf_P(file_pri,PSTR("Mode coulomb\n"));
	//fprintf_P(file_pri,PSTR("Delta charge in last off period: %d\n"),system_offdeltacharge);	
	
		
	
	unsigned long refcharge = ltc2942_getcharge();
	initcharge=refcharge;
	acccharge=0;
	accpwr=0;
	accuah=0;
	acctime=0;
	
	// With prescaler=0, 1 LSB=85uAh. 1Ah=3600 Coulomb
	
	while(1)
	{
		while(CommandProcess(CommandParsersCoulomb,CommandParsersCoulombNum));		
		if(CommandShouldQuit())
			break;
		
		timer_waitperiod_ms(1000,&p);	
		acctime++;	
		
		//ltc2942_printreg(file_pri);
		unsigned long charge = ltc2942_getcharge();
		unsigned short chargectr = ltc2942_getchargectr();
		unsigned short voltage = ltc2942_getvoltage();
		unsigned short temperature = ltc2942_gettemperature();
		fprintf_P(file_pri,PSTR("%lds: chargectr: %u charge: %lu voltage: %u temperature: %u\n"),acctime,chargectr,charge,voltage,temperature);
		fprintf_P(file_pri,PSTR("%lds: dQ from start/reset: %ld uAh\n"),acctime,charge-initcharge);
		fprintf_P(file_pri,PSTR("%lds: Avg power from start/reset: %ld uW\n"),acctime,ltc2942_getavgpower(initcharge,charge,voltage,acctime*1000));
		
		
	
	
		
		refcharge = charge;
		
		
	}
}

#endif
