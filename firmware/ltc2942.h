#ifndef __LTC2942_H
#define __LTC2942_H


#include "cpu.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>

#include "i2c.h"

#define LTC2942_ADDRESS 0x64

void ltc2942_init(void);
unsigned short ltc2942_getchargectr(void);
unsigned long ltc2942_getcharge(void);
unsigned char ltc2942_setchargectr(unsigned short charge);
unsigned short ltc2942_getvoltage(void);
unsigned short ltc2942_gettemperature(void);
unsigned char ltc2942_setadc(unsigned char adc);
unsigned char ltc2942_setprescaler(unsigned char prescaler);
unsigned char ltc2942_getprescaler(void);
//unsigned char ltc2942_setcontrol(unsigned char adc,unsigned char prescaler);
void ltc2942_printreg(FILE *file);
//signed long ltc2942_deltaQ(unsigned short q1,unsigned short q2);
signed long ltc2942_getavgpower(unsigned long c1,unsigned long c2,unsigned short voltage,unsigned long ms);


#endif