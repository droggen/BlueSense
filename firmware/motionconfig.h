#ifndef __MOTIONCONFIG_H
#define __MOTIONCONFIG_H

#include <stdio.h>

// Families of modes
// The number is built using a bitmask: qmgal (l=low power)
#define MPU_MODE_BM_LP 1
#define MPU_MODE_BM_A 2
#define MPU_MODE_BM_G 4
#define MPU_MODE_BM_M 8
#define MPU_MODE_BM_Q 16
#define MPU_MODE_GYR 		MPU_MODE_BM_G
#define MPU_MODE_ACC 		MPU_MODE_BM_A
#define MPU_MODE_ACCGYR 	(MPU_MODE_BM_A|MPU_MODE_BM_G)
#define MPU_MODE_LPACC 		(MPU_MODE_BM_A|MPU_MODE_BM_LP)
#define MPU_MODE_ACCGYRMAG 	(MPU_MODE_BM_A|MPU_MODE_BM_G|MPU_MODE_BM_M)
#define MPU_MODE_ACCGYRMAGQ (MPU_MODE_BM_A|MPU_MODE_BM_G|MPU_MODE_BM_M|MPU_MODE_BM_Q)
#define MPU_MODE_Q 			MPU_MODE_BM_Q



#define MOTIONCONFIG_NUM 43
#define MPU_MODE_OFF 								0
#define MPU_MODE_500HZ_GYRO_BW250					1
#define MPU_MODE_500HZ_GYRO_BW184					2
#define MPU_MODE_200HZ_GYRO_BW92					3
#define MPU_MODE_100HZ_GYRO_BW41					4
#define MPU_MODE_50HZ_GYRO_BW20						5
#define MPU_MODE_10HZ_GYRO_BW5						6
#define MPU_MODE_1HZ_GYRO_BW5						7
#define MPU_MODE_1KHZ_ACC_BW460						8
#define MPU_MODE_500HZ_ACC_BW184					9
#define MPU_MODE_200HZ_ACC_BW92						10
#define MPU_MODE_100HZ_ACC_BW41						11
#define MPU_MODE_50HZ_ACC_BW20						12
#define MPU_MODE_10HZ_ACC_BW5						13
#define MPU_MODE_1HZ_ACC_BW5						14
#define MPU_MODE_1KHZ_ACC_BW460_GYRO_BW250			15
#define MPU_MODE_500HZ_ACC_BW184_GYRO_BW250			16
#define MPU_MODE_500HZ_ACC_BW184_GYRO_BW184			17
#define MPU_MODE_200HZ_ACC_BW92_GYRO_BW92			18
#define MPU_MODE_100HZ_ACC_BW41_GYRO_BW41			19
#define MPU_MODE_50HZ_ACC_BW20_GYRO_BW20			20
#define MPU_MODE_10HZ_ACC_BW5_GYRO_BW5				21
#define MPU_MODE_1HZ_ACC_BW5_GYRO_BW5				22
#define MPU_MODE_500HZ_LPACC						23
#define MPU_MODE_250HZ_LPACC						24
#define MPU_MODE_125HZ_LPACC						25
#define MPU_MODE_62HZ_LPACC							26
#define MPU_MODE_31HZ_LPACC							27
#define MPU_MODE_1HZ_LPACC							28

#define MPU_MODE_1KHZ_ACC_BW460_GYRO_BW250_MAG_8	29
#define MPU_MODE_500HZ_ACC_BW184_GYRO_BW250_MAG_8	30
#define MPU_MODE_500HZ_ACC_BW184_GYRO_BW184_MAG_8	31
#define MPU_MODE_100HZ_ACC_BW41_GYRO_BW41_MAG_8		32
#define MPU_MODE_50HZ_ACC_BW20_GYRO_BW20_MAG_8		33

#define MPU_MODE_1KHZ_ACC_BW460_GYRO_BW250_MAG_100	34
#define MPU_MODE_500HZ_ACC_BW184_GYRO_BW250_MAG_100	35
#define MPU_MODE_500HZ_ACC_BW184_GYRO_BW184_MAG_100	36
#define MPU_MODE_200HZ_ACC_BW92_GYRO_BW92_MAG_100	37
#define MPU_MODE_100HZ_ACC_BW41_GYRO_BW41_MAG_100	38

#define MPU_MODE_100HZ_ACC_BW41_GYRO_BW41_MAG_100_Q	39
#define MPU_MODE_200HZ_ACC_BW92_GYRO_BW92_MAG_100_Q	40
#define MPU_MODE_100HZ_Q							41
#define MPU_MODE_200HZ_Q							42


extern PGM_P const mc_options[];
void mpu_config_motionmode(unsigned char sensorsr,unsigned char autoread);
unsigned char mpu_get_motionmode(unsigned char *autoread);
void mpu_getmodename(unsigned char motionmode,char *buffer);
void mpu_printmotionmode(FILE *file);


#endif
