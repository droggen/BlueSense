#ifndef __MPU_H
#define __MPU_H


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


// AK8963 address:0X0C
#define MAG_ADDRESS 0X0C
#define MPU_ADDRESS 105

#define MPU_R_SMPLRT_DIV 			0x19
#define MPU_R_CONFIG				0x1A
#define MPU_R_GYROCONFIG			0x1B
#define MPU_R_ACCELCONFIG			0x1C
#define MPU_R_ACCELCONFIG2			0x1D
#define MPU_R_LPODR					0x1E
#define MPU_R_FIFOEN 				0x23
#define MPU_R_INTERRUPTPIN			0x37
#define MPU_R_INTERRUPTENABLE 		0x38
#define MPU_R_USR_CTRL	 			106
#define MPU_R_PWR_MGMT_1 			0x6B
#define MPU_R_PWR_MGMT_2			0x6C
#define MPU_R_WHOAMI					0x75

#define MPU_R_I2C_SLV0_ADDR		37
#define MPU_R_I2C_SLV0_REG		38
#define MPU_R_I2C_SLV0_CTRL		39
#define MPU_R_I2C_SLV0_DO 99

#define MPU_R_I2C_SLV4_ADDR		49
#define MPU_R_I2C_SLV4_REG		50
#define MPU_R_I2C_SLV4_DO		51
#define MPU_R_I2C_SLV4_CTRL		52
#define MPU_R_I2C_SLV4_DI 		53

#define MPU_R_I2C_MST_CTRL		36
#define MPU_R_I2C_MST_STATUS	54
#define MPU_R_I2C_MST_DELAY_CTRL 103

#define MPU_LPODR_500 11
#define MPU_LPODR_250 10
#define MPU_LPODR_125 9
#define MPU_LPODR_62 8
#define MPU_LPODR_31 7
#define MPU_LPODR_16 6
#define MPU_LPODR_8 5
#define MPU_LPODR_4 4
#define MPU_LPODR_2 3
#define MPU_LPODR_1 2

#define MPU_ACC_LPF_460 0
#define MPU_ACC_LPF_184 1
#define MPU_ACC_LPF_92 2
#define MPU_ACC_LPF_41 3
#define MPU_ACC_LPF_20 4
#define MPU_ACC_LPF_10 5
#define MPU_ACC_LPF_5 6

#define MPU_GYR_LPF_250 0
#define MPU_GYR_LPF_184 1
#define MPU_GYR_LPF_92 2
#define MPU_GYR_LPF_41 3
#define MPU_GYR_LPF_20 4
#define MPU_GYR_LPF_10 5
#define MPU_GYR_LPF_5 6
#define MPU_GYR_LPF_3600 7

// Gyro scale
#define MPU_GYR_SCALE_250 0
#define MPU_GYR_SCALE_500 1
#define MPU_GYR_SCALE_1000 2
#define MPU_GYR_SCALE_2000 3

// Acc scale
#define MPU_ACC_SCALE_2		0
#define MPU_ACC_SCALE_4		1
#define MPU_ACC_SCALE_8		2
#define MPU_ACC_SCALE_16	3


#if HWVER==1
#include "mpu-i2c.h"
#endif
#if (HWVER==4) || (HWVER==5) || (HWVER==6)
#include "mpu-usart0.h"
#endif




extern unsigned char sample_mode;

// Conversion from gyro raw readings to radians per second
#ifdef __cplusplus
extern float mpu_gtorps;
#else
extern _Accum mpu_gtorps;
#endif

//typedef void (*MPU_READ_CALLBACK7)(unsigned char status,unsigned char error,signed short ax,signed short ay,signed short az,signed short gx,signed short gy,signed short gz,signed short temp);
//typedef void (*MPU_READ_CALLBACK3)(unsigned char status,unsigned char error,signed short x,signed short y,signed short z);





//unsigned long mpu_estimatefifofillrate(unsigned char fenflag);
//unsigned short mpu_estimateodr(void);



// Interrupt driven
void mpu_get_agt_int_init(void);
unsigned short mpu_getfifocnt_int(void);
unsigned char mpu_get_agt_int(signed short *ax,signed short *ay,signed short *az,signed short *gx,signed short *gy,signed short *gz,signed short *temp);
/*unsigned char mpu_get_agt_int_cb(MPU_READ_CALLBACK7 cb);
unsigned char mpu_get_a_int(signed short *ax,signed short *ay,signed short *az);
unsigned char mpu_get_a_int_cb(MPU_READ_CALLBACK3 cb);
unsigned char mpu_get_g_int(signed short *gx,signed short *gy,signed short *gz);
unsigned char mpu_get_g_int_cb(MPU_READ_CALLBACK3 cb);*/


extern unsigned char __mpu_sample_softdivider_ctr,__mpu_sample_softdivider_divider;



void mpu_isr(void);

// Data buffers
#define MPU_MOTIONBUFFERSIZE 32
extern volatile signed short mpu_data_ax[],mpu_data_ay[],mpu_data_az[],mpu_data_gx[],mpu_data_gy[],mpu_data_gz[],mpu_data_mx[],mpu_data_my[],mpu_data_mz[],mpu_data_temp[];
extern volatile unsigned long int mpu_data_time[];
extern volatile unsigned char mpu_data_ms[];
extern volatile unsigned short mpu_data_packetctr[];
extern volatile unsigned short __mpu_data_packetctr_current;
extern volatile unsigned char mpu_data_rdptr,mpu_data_wrptr;

// Magnetometer Axis Sensitivity Adjustment
extern unsigned char _mpu_mag_asa[3];
extern signed short _mpu_mag_calib_max[3];
extern signed short _mpu_mag_calib_min[3];
extern signed short _mpu_mag_bias[3];
extern signed short _mpu_mag_sens[3];
extern unsigned char _mpu_mag_correctionmode;


extern unsigned char __mpu_autoread;

// Automatic read statistic counters
extern unsigned long mpu_cnt_int, mpu_cnt_sample_tot, mpu_cnt_sample_succcess, mpu_cnt_sample_errbusy, mpu_cnt_sample_errfull;


void __mpu_read_cb(void);

// Motion data buffers
unsigned char mpu_data_isfull(void);
//unsigned char mpu_data_isempty(void);
unsigned char mpu_data_level(void);
void mpu_data_wrnext(void);
void mpu_data_rdnext(void);



void mpu_mode_accgyro(unsigned char gdlpe,unsigned char gdlpoffhbw,unsigned char gdlpbw,unsigned char adlpe,unsigned char adlpbw,unsigned char divider);
void mpu_mode_gyro(unsigned char gdlpe,unsigned char gdlpoffhbw,unsigned char gdlpbw,unsigned char divider);
void mpu_mode_acc(unsigned char dlpenable,unsigned char dlpbw,unsigned char divider);
void mpu_mode_lpacc(unsigned char lpodr);
void mpu_mode_off(void);

void _mpu_enableautoread(void);
void _mpu_disableautoread(void);


void mpu_init(void);
void mpu_get_agt(signed short *ax,signed short *ay,signed short *az,signed short *gx,signed short *gy,signed short *gz,signed short *temp);
void mpu_get_agmt(signed short *ax,signed short *ay,signed short *az,signed short *gx,signed short *gy,signed short *gz,signed short *mx,signed short *my,signed short *mz,unsigned char *ms,signed short *temp);
void mpu_get_g(signed short *gx,signed short *gy,signed short *gz);
void mpu_get_a(signed short *ax,signed short *ay,signed short *az);
/*unsigned char mpu_get_agt_int_cb(MPU_READ_CALLBACK7 cb);
unsigned char mpu_get_a_int_cb(MPU_READ_CALLBACK3 cb);
unsigned char mpu_get_g_int_cb(MPU_READ_CALLBACK3 cb);*/

unsigned short mpu_getfifocnt(void);
void mpu_fifoenable(unsigned char flags,unsigned char en,unsigned char reset);
void mpu_readallregs(unsigned char *v);
void mpu_fiforead(unsigned char *fifo,unsigned short n);
void mpu_fiforeadshort(short *fifo,unsigned short n);
void mpu_setgyrobias(short bgx,short bgy,short bgz);
void mpu_setaccodr(unsigned char odr);
void mpu_setacccfg2(unsigned char cfg);
void mpu_setusrctrl(unsigned char cfg);
void mpu_reset(void);
void mpu_setsrdiv(unsigned char div);
void mpu_setgyrosamplerate(unsigned char fchoice,unsigned char dlp);
void mpu_setaccsamplerate(unsigned char fchoice,unsigned char dlp);
void mpu_set_interrutenable(unsigned char wom,unsigned char fifo,unsigned char fsync,unsigned char datardy);

void mpu_setgyroscale(unsigned char scale);
unsigned char mpu_getgyroscale(void);
void mpu_setaccscale(unsigned char scale);
unsigned char mpu_getaccscale(void);
void mpu_temp_enable(unsigned char enable);
void mpu_clksel(unsigned char clk);
void mpu_mode_gyrostby(unsigned char stby);
void mpu_mode_sleep(unsigned char sleep);
signed short mpu_convtemp(signed short t);
void mpu_set_interruptpin(unsigned char p);
unsigned char mpu_getwhoami(void);

void mpu_acquirecalib(void);
void mpu_calibrate(void);

void _mpu_defaultdlpon(void);
void _mpu_wakefromsleep(void);

//void mpu_testsleep(void);
void mpu_printfifo(FILE *file);

void _mpu_mag_interfaceenable(unsigned char en);
void _mpu_mag_mode(unsigned char mode);
unsigned char mpu_mag_readreg(unsigned char reg);
void mpu_mag_writereg(unsigned char reg,unsigned char val);
void _mpu_mag_regshadow(unsigned char enable,unsigned char dly,unsigned char regstart,unsigned char numreg);
void _mpu_mag_readasa(void);
void mpu_mag_correct1(signed short mx,signed short my,signed short mz,volatile signed short *mx2,volatile signed short *my2,volatile signed short *mz2);
void mpu_mag_correct2(signed short mx,signed short my,signed short mz,volatile signed short *mx2,volatile signed short *my2,volatile signed short *mz2);
void mpu_mag_calibrate(void);
void mpu_mag_storecalib(void);
void mpu_mag_loadcalib(void);
void mpu_mag_correctionmode(unsigned char mode);

// Print functions
void mpu_mag_printreg(FILE *file);
void mpu_mag_printcalib(FILE *f);
void mpu_printreg(FILE *file);
void mpu_printextreg(FILE *file);
void mpu_printregdesc(FILE *file);
void mpu_printregdesc2(FILE *file);
void mpu_printstat(FILE *file);

#endif