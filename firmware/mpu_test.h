#ifndef __MPU_TEST_H
#define __MPU_TEST_H


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

#include "mpu.h"

void mpu_test_lpacc(void);
void mpu_test_acc(void);
void mpu_test_accgyro(void);
void mpu_test_smplrt_div(void);
void mpu_test_lpodr(void);
void mpu_test_bypass(void);
void mpu_test_sample(void);
void mpu_test_sample2(void);
void mpu_test_sample3(void);
void mpu_test_sample4(void);
void mpu_test_sample5(void);
void mpu_test_benchmark_read(void);
void mpu_test_sample3_cb(unsigned char status,unsigned char error,signed short ax,signed short ay,signed short az,signed short gx,signed short gy,signed short gz,signed short temp);
void mpu_test_benchmark_intread(void);

void mpu_test_queue(void);

void mpu_test_intread(void);




#endif