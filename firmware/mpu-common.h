#ifndef __MPU_COMMON_H
#define __MPU_COMMON_H

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

#include "i2c.h"
#include "mpu.h"
#include "wait.h"
#include "main.h"
#include "serial.h"
#include "adc.h"


void mpu_printreg(FILE *file);
void mpu_printreg2(FILE *file);
void mpu_printregdesc(FILE *file);
void mpu_printregdesc2(FILE *file);
void mpu_get_agt(signed short *ax,signed short *ay,signed short *az,signed short *gx,signed short *gy,signed short *gz,signed short *temp);
unsigned char mpu_get_agt_int_cb(MPU_READ_CALLBACK7 cb);
unsigned char mpu_get_a_int_cb(MPU_READ_CALLBACK3 cb);
unsigned char mpu_get_g_int_cb(MPU_READ_CALLBACK3 cb);
void mpu_get_g(signed short *gx,signed short *gy,signed short *gz);
void mpu_get_a(signed short *ax,signed short *ay,signed short *az);
unsigned short mpu_getfifocnt(void);
void mpu_fifoenable(unsigned char flags);
void mpu_readallregs(unsigned char *v);
void mpu_fiforead(unsigned char *fifo,unsigned short n);
void mpu_setaccodr(unsigned char odr);
void mpu_setacccfg2(unsigned char cfg);
void mpu_setusrctrl(unsigned char cfg);
unsigned char mpu_getwhoami(void);
void mpu_reset(void);
void mpu_setsrdiv(unsigned char div);
void mpu_setgyrosamplerate(unsigned char fchoice,unsigned char dlp);
void mpu_setaccsamplerate(unsigned char fchoice,unsigned char dlp);
void mpu_set_interrutenable(unsigned char wom,unsigned char fifo,unsigned char fsync,unsigned char datardy);
void mpu_mode_lpacc(unsigned char lpodr);
void mpu_mode_acc(unsigned char dlpenable,unsigned char dlpbw,unsigned char divider);
void mpu_mode_gyro(unsigned char gdlpe,unsigned char gdlpoffhbw,unsigned char gdlpbw,unsigned char divider);
void mpu_mode_accgyro(unsigned char gdlpe,unsigned char gdlpoffhbw,unsigned char gdlpbw,unsigned char adlpe,unsigned char adlpbw,unsigned char divider);
void mpu_mode_temp(unsigned char enable);
void mpu_mode_off(void);
void mpu_mode_clksel(unsigned char clk);
void mpu_mode_gyrostby(unsigned char stby);
void mpu_mode_sleep(unsigned char sleep);
signed short mpu_convtemp(signed short t);
void mpu_set_interruptpin(unsigned char p);

#endif