#ifndef __DS3232_H
#define __DS3232_H

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

#define DS3232_ADDRESS 104


void ds3232_init(void);
unsigned char ds3232_bcd2dec(unsigned char bcd);
unsigned char ds3232_readtime_sync(unsigned char *time);
unsigned char ds3232_readtime_sqwsync(unsigned char *time);
unsigned char ds3232_readtime(unsigned char *time);
unsigned char ds3232_readtime_sync_conv(unsigned char *hour,unsigned char *min,unsigned char *sec);
unsigned char ds3232_readtime_conv(unsigned char *hour,unsigned char *min,unsigned char *sec);
unsigned char ds3232_readdate_conv(unsigned char *date,unsigned char *month,unsigned char *year);
void ds3232_convtime(unsigned char *val,unsigned char *hour,unsigned char *min,unsigned char *sec);
unsigned char ds3232_write_control(unsigned char val);
unsigned char ds3232_write_status(unsigned char val);
unsigned char ds3232_writedate(unsigned char day,unsigned char date,unsigned char month,unsigned char year);
unsigned char ds3232_writedate_int(unsigned char day,unsigned char date,unsigned char month,unsigned char year);
unsigned char ds3232_writetime(unsigned char hour,unsigned char min,unsigned char sec);
void ds3232_printreg(FILE *file);


void ds3232_off(void);
void ds3232_alarm_in(unsigned short insec);

unsigned char ds3232_readtemp(signed short *temp);
unsigned char ds3232_readtemp_int_cb(void (*cb)(unsigned char,signed short));
unsigned char ds3232_readtemp_int_cb_cb(I2C_TRANSACTION *t);



// I2C transactional
unsigned char ds3232_readtime_int(unsigned char *time);
unsigned char ds3232_readtime_conv_int(unsigned char sync, unsigned char *hour,unsigned char *min,unsigned char *sec);
unsigned char ds3232_readdate_conv_int(unsigned char *date,unsigned char *month,unsigned char *year);

#endif