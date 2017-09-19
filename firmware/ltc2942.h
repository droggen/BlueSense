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

// Short term battery status
#define LTC2942NUMLASTMW 10
// Long term battery status
// The number of entries in is equal to LTC2942NUMLONGBATSTAT-1
// The sensor has a battery life of <6 hours, hence 120 entries needed with a readout every 3mn.
#define LTC2942NUMLONGBATSTAT 120
//#define LTC2942NUMLONGBATSTAT_UPDATEEVERY 180000l
#define LTC2942NUMLONGBATSTAT_UPDATEEVERY 10000l

typedef struct {
	unsigned long t;
	signed short mW,mA,mV;
} LTC2942_BATSTAT;

extern volatile unsigned long int _ltc2942_last_updatetime;
extern volatile unsigned short _ltc2942_last_chargectr;			// Background read: charge counter (raw)
extern volatile unsigned long _ltc2942_last_charge;				// Background read: charge (uAh)
extern volatile unsigned short _ltc2942_last_mV;			// Background read: voltage 
extern volatile signed short _ltc2942_last_mA;					// Background read: average current in mA between two reads
extern volatile signed short _ltc2942_last_mW;			// Background read: average power in mW between two reads
extern volatile short _ltc2942_last_temperature;				// Background read: temperature
extern I2C_TRANSACTION __ltc2942_trans_read;	

void ltc2942_init(void);
unsigned short ltc2942_getchargectr(void);
unsigned long ltc2942_getcharge(void);
unsigned char ltc2942_setchargectr(unsigned short charge);
unsigned short ltc2942_getvoltage(void);
short ltc2942_gettemperature(void);
unsigned char ltc2942_setadc(unsigned char adc);
unsigned char ltc2942_setprescaler(unsigned char prescaler);
unsigned char ltc2942_getprescaler(void);
//unsigned char ltc2942_setcontrol(unsigned char adc,unsigned char prescaler);
void ltc2942_printreg(FILE *file);
//signed long ltc2942_deltaQ(unsigned short q1,unsigned short q2);
signed short ltc2942_getavgpower(unsigned long c1,unsigned long c2,unsigned short voltage,unsigned long ms);
unsigned char ltc2942_backgroundgetstate(unsigned char);
unsigned long ltc2942_last_updatetime(void);
unsigned short ltc2942_last_chargectr(void);
unsigned long ltc2942_last_charge(void);
unsigned short ltc2942_last_mV(void);
short ltc2942_last_temperature(void);
signed short ltc2942_last_mW(void);
signed short ltc2942_last_mA(void);
char *ltc2942_last_strstatus(void);
signed short ltc2942_last_mWs(unsigned char idx);

void ltc2942_clear_longbatstat();
unsigned char ltc2942_get_numlongbatstat();
void ltc2942_get_longbatstat(unsigned char idx,LTC2942_BATSTAT *batstat);
void _ltc2942_add_longbatstat(LTC2942_BATSTAT *batstat);
void _ltc2942_dump_longbatstat();
void ltc2942_print_longbatstat(FILE *f);

unsigned char __ltc2942_trans_read_done(I2C_TRANSACTION *t);


#endif