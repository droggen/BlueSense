#ifndef __MPU_I2C_H
#define __MPU_I2C_H

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

// Internal callbacks
unsigned char _mpu_i2c_trans_select7_cb(volatile struct _I2C_TRANSACTION *it);
unsigned char _mpu_i2c_trans_select3_cb(volatile struct _I2C_TRANSACTION *it);
unsigned char _mpu_i2c_trans_read_agt_cb(volatile struct _I2C_TRANSACTION *it);
unsigned char _mpu_i2c_trans_read_a_cb(volatile struct _I2C_TRANSACTION *it);
unsigned char _mpu_i2c_trans_read_g_cb(volatile struct _I2C_TRANSACTION *it);

// Predefined transactions
extern I2C_TRANSACTION mpu_i2c_trans_select_a;			// Select register accelerometer
extern I2C_TRANSACTION mpu_i2c_trans_select_g;			// Select register gyroscope
extern I2C_TRANSACTION mpu_i2c_trans_read_14;				// Read 14 bytes
extern I2C_TRANSACTION mpu_i2c_trans_read_6;				// Read 6 bytes


unsigned char mpu_readreg(unsigned char reg);
void mpu_readallregs(unsigned char *v);
void mpu_readregs_burst(unsigned char *d,unsigned char reg,unsigned char n);
void mpu_writereg(unsigned char reg,unsigned char v);


#endif