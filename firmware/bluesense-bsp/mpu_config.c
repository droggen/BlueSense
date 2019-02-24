#include "cpu.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include "mpu.h"
#include "main.h"
#include "mpu_config.h"
#include "MadgwickAHRS.h"
#include "init.h"

/*
	File: motionconfig
		
*/

// Exhaustive set of frequencies up to 1KHz
const char mc_0[] PROGMEM =  "Motion off";
const char mc_1[] PROGMEM =  "  500Hz Gyro (BW=250Hz) x";
const char mc_2[] PROGMEM =  "  500Hz Gyro (BW=184Hz)";
const char mc_3[] PROGMEM =  "  200Hz Gyro (BW= 92Hz)";
const char mc_4[] PROGMEM =  "  100Hz Gyro (BW= 41Hz)";
const char mc_5[] PROGMEM =  "   50Hz Gyro (BW= 20Hz)";
const char mc_6[] PROGMEM =  "   10Hz Gyro (BW=  5Hz)";
const char mc_7[] PROGMEM =  "    1Hz Gyro (BW=  5Hz)";
const char mc_8[] PROGMEM =  " 1000Hz Acc  (BW=460Hz) x";
const char mc_9[] PROGMEM =  "  500Hz Acc  (BW=184Hz)";
const char mc_10[] PROGMEM = "  200Hz Acc  (BW= 92Hz)";
const char mc_11[] PROGMEM = "  100Hz Acc  (BW= 41Hz)";
const char mc_12[] PROGMEM = "   50Hz Acc  (BW= 20Hz)";
const char mc_13[] PROGMEM = "   10Hz Acc  (BW=  5Hz)";
const char mc_14[] PROGMEM = "    1Hz Acc  (BW=  5Hz)";
const char mc_15[] PROGMEM = " 1000Hz Acc  (BW=460Hz) Gyro (BW=250Hz) x";
const char mc_16[] PROGMEM = "  500Hz Acc  (BW=184Hz) Gyro (BW=250Hz) x";
const char mc_17[] PROGMEM = "  500Hz Acc  (BW=184Hz) Gyro (BW=184Hz)";
const char mc_18[] PROGMEM = "  200Hz Acc  (BW= 92Hz) Gyro (BW= 92Hz)";
const char mc_19[] PROGMEM = "  100Hz Acc  (BW= 41Hz) Gyro (BW= 41Hz)";
const char mc_20[] PROGMEM = "   50Hz Acc  (BW= 20Hz) Gyro (BW= 20Hz)";
const char mc_21[] PROGMEM = "   10Hz Acc  (BW=  5Hz) Gyro (BW=  5Hz)";
const char mc_22[] PROGMEM = "    1Hz Acc  (BW=  5Hz) Gyro (BW=  5Hz)";
const char mc_23[] PROGMEM = "  500Hz Acc low power";
const char mc_24[] PROGMEM = "  250Hz Acc low power";
const char mc_25[] PROGMEM = "  125Hz Acc low power";
const char mc_26[] PROGMEM = " 62.5Hz Acc low power";
const char mc_27[] PROGMEM = "31.25Hz Acc low power";
const char mc_28[] PROGMEM = "    1Hz Acc low power";
const char mc_29[] PROGMEM = " 1000Hz Acc  (BW=460Hz) Gyro (BW=250Hz) Mag 8Hz x";
const char mc_30[] PROGMEM = "  500Hz Acc  (BW=184Hz) Gyro (BW=250Hz) Mag 8Hz x";
const char mc_31[] PROGMEM = "  500Hz Acc  (BW=184Hz) Gyro (BW=184Hz) Mag 8Hz";
const char mc_32[] PROGMEM = "  100Hz Acc  (BW= 41Hz) Gyro (BW= 41Hz) Mag 8Hz";
const char mc_33[] PROGMEM = "   50Hz Acc  (BW= 20Hz) Gyro (BW= 20Hz) Mag 8Hz";
const char mc_34[] PROGMEM = " 1000Hz Acc  (BW=460Hz) Gyro (BW=250Hz) Mag 100Hz x";
const char mc_35[] PROGMEM = "  500Hz Acc  (BW=184Hz) Gyro (BW=250Hz) Mag 100Hz x";
const char mc_36[] PROGMEM = "  500Hz Acc  (BW=184Hz) Gyro (BW=184Hz) Mag 100Hz";
const char mc_37[] PROGMEM = "  200Hz Acc  (BW= 92Hz) Gyro (BW= 92Hz) Mag 100Hz";
const char mc_38[] PROGMEM = "  100Hz Acc  (BW= 41Hz) Gyro (BW= 41Hz) Mag 100Hz";
//const char mc_39[] PROGMEM = "  500Hz Acc  (BW=184Hz) Gyro (BW=184Hz) Mag 8Hz Quaternions";
//const char mc_40[] PROGMEM = "  200Hz Acc  (BW= 92Hz) Gyro (BW= 92Hz) Mag 8Hz Quaternions";
//const char mc_41[] PROGMEM = "  100Hz Acc  (BW= 41Hz) Gyro (BW= 41Hz) Mag 8Hz Quaternions";
const char mc_39[] PROGMEM = "  500Hz Acc  (BW=184Hz) Gyro (BW=184Hz) Mag 100Hz Quaternions";
const char mc_40[] PROGMEM = "  200Hz Acc  (BW= 92Hz) Gyro (BW= 92Hz) Mag 100Hz Quaternions";
const char mc_41[] PROGMEM = "  100Hz Acc  (BW= 41Hz) Gyro (BW= 41Hz) Mag 100Hz Quaternions";
const char mc_42[] PROGMEM = "  500Hz Quaternions Qsg=(qw,qx,qy,qz); rotates vector in earth coords G into sensor coords S";
const char mc_43[] PROGMEM = "  200Hz Quaternions";
const char mc_44[] PROGMEM = "  100Hz Quaternions";
const char mc_45[] PROGMEM = "  100Hz Tait–Bryan/aerospace/zx'y\", intrinsic (yaw, pitch, roll)";
const char mc_46[] PROGMEM = "  100Hz Quaternions debug (angle, x,y,z)";

PGM_P const mc_options[MOTIONCONFIG_NUM] PROGMEM = 
{
    mc_0,
    mc_1,
    mc_2,
    mc_3,
    mc_4,
    mc_5,
    mc_6,
    mc_7,
    mc_8,
    mc_9,
    mc_10,
    mc_11,
    mc_12,
    mc_13,
    mc_14,
    mc_15,
    mc_16,
    mc_17,
    mc_18,
    mc_19,
    mc_20,
    mc_21,
    mc_22,
    mc_23,
    mc_24,
    mc_25,
    mc_26,
    mc_27,
    mc_28,
	mc_29,
	mc_30,
	mc_31,
	mc_32,
	mc_33,
	mc_34,
	mc_35,
	mc_36,
	mc_37,
	mc_38,
	mc_39,
	mc_40,
	mc_41,
	mc_42,
	mc_43,
	mc_44,
	mc_45,
	mc_46,
};





/******************************************************************************
	config_sensorsr_settings
*******************************************************************************	
	Configures the motion sensor based (active sensors and sample rate) on 
	config_sensorsr.
	
	config_sensorsr_settings contains for each option: mode gdlpe gdlpoffhbw gdlpbw adlpe adlpbw divider lpodr softdiv
	mode is 0=gyro, 1=acc, 2=gyroacc, 3=lpacc
	
	magmode: 0=off, 1=8Hz, 2=100Hz. Node that magmode is only used in moe MPU_MODE_ACCGYRMAG
	
	magdiv:	magnetometer ODR divider, reads data at ODR/(1+magdiv). The ODR is the sample frequency divided by (1+divider).	
******************************************************************************/
//const char hello[] PROGMEM = {1,2,3};

const short config_sensorsr_settings[MOTIONCONFIG_NUM][12] = {
					// mode              gdlpe gdlpoffhbw        gdlpbw     adlpe           adlpbw divider          lpodr softdiv	magmode	magdiv		splrate
					// Off
					{ MPU_MODE_OFF,        0,         0,               0,     0,               0,      0,             0,     0,     0,		0,			0},
					// 500Hz Gyro (BW=250Hz)
					{ MPU_MODE_GYR,        1,         0, MPU_GYR_LPF_250,     0,               0,      0,             0,     15,    0,		0,			500},			// ODR=8000Hz
					// 500Hz Gyro (BW=184Hz)
					{ MPU_MODE_GYR,        1,         0, MPU_GYR_LPF_184,     0,               0,      1,             0,     0,     0,		0,			500},			// ODR=500Hz
					// 200Hz Gyro (BW= 92Hz)
					{ MPU_MODE_GYR,        1,         0,  MPU_GYR_LPF_92,     0,               0,      4,             0,     0,     0,		0,			200},
					// 100Hz Gyro (BW= 41Hz)
					{ MPU_MODE_GYR,        1,         0,  MPU_GYR_LPF_41,     0,               0,      9,             0,     0,     0,		0,			100},
					// 50Hz Gyro (BW= 20Hz)
					{ MPU_MODE_GYR,        1,         0,  MPU_GYR_LPF_20,     0,               0,     19,             0,     0,     0,		0,			50},
					// 10Hz Gyro (BW=  5Hz)
					{ MPU_MODE_GYR,        1,         0,   MPU_GYR_LPF_5,     0,               0,     99,             0,     0,     0,		0,			10},
					// 1Hz Gyro (BW=  5Hz)
					{ MPU_MODE_GYR,        1,         0,   MPU_GYR_LPF_5,     0,               0,     99,             0,     9,     0,		0,			1},
					// 1000Hz Acc  (BW=460Hz)
					{ MPU_MODE_ACC,        0,         0,               0,     1, MPU_ACC_LPF_460,      0,             0,     0,     0,		0,			1000},			// ODR=1000Hz
					// 500Hz Acc  (BW=184Hz)
					{ MPU_MODE_ACC,        0,         0,               0,     1, MPU_ACC_LPF_184,      1,             0,     0,     0,		0,			500},
					// 200Hz Acc  (BW= 92Hz)
					{ MPU_MODE_ACC,        0,         0,               0,     1,  MPU_ACC_LPF_92,      4,             0,     0,     0,		0,			200},
					// 100Hz Acc  (BW= 41Hz)
					{ MPU_MODE_ACC,        0,         0,               0,     1,  MPU_ACC_LPF_41,      9,             0,     0,     0,		0,			100},
					// 50Hz Acc  (BW= 20Hz)
					{ MPU_MODE_ACC,        0,         0,               0,     1,  MPU_ACC_LPF_20,     19,             0,     0,     0,		0,			50},
					// 10Hz Acc  (BW=  5Hz)
					{ MPU_MODE_ACC,        0,         0,               0,     1,   MPU_ACC_LPF_5,     99,             0,     0,     0,		0,			10},
					// 1Hz Acc  (BW=  5Hz)
					{ MPU_MODE_ACC,        0,         0,               0,     1,   MPU_ACC_LPF_5,     99,             0,     9,     0,		0,			1},
					// 1000Hz Acc  (BW=460Hz) Gyro (BW=250Hz)
					{ MPU_MODE_ACCGYR,     1,         0, MPU_GYR_LPF_250,     1, MPU_ACC_LPF_460,      0,             0,     7,     0,		0,			1000},			// ODR=8000Hz
					// 500Hz Acc  (BW=184Hz) Gyro (BW=250Hz)
					{ MPU_MODE_ACCGYR,     1,         0, MPU_GYR_LPF_250,     1, MPU_ACC_LPF_184,      0,             0,    15,     0,		0,			500},			// ODR=8000Hz
					// 500Hz Acc  (BW=184Hz) Gyro (BW=184Hz)
					{ MPU_MODE_ACCGYR,     1,         0, MPU_GYR_LPF_184,     1, MPU_ACC_LPF_184,      1,             0,     0,     0,		0,			500},			// ODR=500Hz
					// 200Hz Acc  (BW= 92Hz) Gyro (BW= 92Hz)
					{ MPU_MODE_ACCGYR,     1,         0,  MPU_GYR_LPF_92,     1,  MPU_ACC_LPF_92,      4,             0,     0,     0,		0,			200},
					// 100Hz Acc  (BW= 41Hz) Gyro (BW= 41Hz)
					{ MPU_MODE_ACCGYR,     1,         0,  MPU_GYR_LPF_41,     1,  MPU_ACC_LPF_41,      9,             0,     0,     0,		0,			100},
					// 50Hz Acc  (BW= 20Hz) Gyro (BW= 20Hz)
					{ MPU_MODE_ACCGYR,     1,         0,  MPU_GYR_LPF_20,     1,  MPU_ACC_LPF_20,     19,             0,     0,     0,		0,			50},
					// 10Hz Acc  (BW=  5Hz) Gyro (BW=  5Hz)
					{ MPU_MODE_ACCGYR,     1,         0,   MPU_GYR_LPF_5,     1,   MPU_ACC_LPF_5,     99,             0,     0,     0,		0,			10},
					// 1Hz Acc  (BW=  5Hz) Gyro (BW=  5Hz)
					{ MPU_MODE_ACCGYR,     1,         0,   MPU_GYR_LPF_5,     1,   MPU_ACC_LPF_5,     99,             0,     9,     0,		0,			1},
					// 500Hz Acc low power
					{ MPU_MODE_LPACC,      0,         0,               0,     0,               0,      0, MPU_LPODR_500,     0,     0,		0,			500},
					// 250Hz Acc low power
					{ MPU_MODE_LPACC,      0,         0,               0,     0,               0,      0, MPU_LPODR_250,     0,     0,		0,			250},
					// 125Hz Acc low power
					{ MPU_MODE_LPACC,      0,         0,               0,     0,               0,      0, MPU_LPODR_125,     0,     0,		0,			125},
					// 62.5Hz Acc low power
					{ MPU_MODE_LPACC,      0,         0,               0,     0,               0,      0,  MPU_LPODR_62,     0,     0,		0,			63},
					// 31.25Hz Acc low power
					{ MPU_MODE_LPACC,      0,         0,               0,     0,               0,      0,  MPU_LPODR_31,     0,     0,		0,			31},
					// 1Hz Acc low power
					{ MPU_MODE_LPACC,      0,         0,               0,     0,               0,      0,   MPU_LPODR_1,     0,     0,		0,			1},
					//----Magn 8Hz
					// 1000Hz Acc (BW=460Hz) Gyro (BW=250Hz) Mag 8Hz
					{ MPU_MODE_ACCGYRMAG,  1,         0, MPU_GYR_LPF_250,     1, MPU_ACC_LPF_460,      0,             0,     7,     1,		31,			1000},	// ODR=8000Hz, mag=8HZ, magodr=8000/32=250Hz
					// 500Hz Acc  (BW=184Hz) Gyro (BW=250Hz) Mag 8Hz
					{ MPU_MODE_ACCGYRMAG,  1,         0, MPU_GYR_LPF_250,     1, MPU_ACC_LPF_184,      0,             0,    15,     1,		31,			500},	// ODR=8000Hz, mag=8HZ, magodr=8000/32=250Hz
					// 500Hz Acc  (BW=184Hz) Gyro (BW=184Hz) Mag 8Hz
					{ MPU_MODE_ACCGYRMAG,  1,         0, MPU_GYR_LPF_184,     1, MPU_ACC_LPF_184,      1,             0,     0,     1,		31,			500},	// ODR=500Hz, mag=8HZ, magodr=500/32=15.6HZ
					// 100Hz Acc  (BW= 41Hz) Gyro (BW= 41Hz) Mag 8Hz
					{ MPU_MODE_ACCGYRMAG,  1,         0,  MPU_GYR_LPF_41,     1,  MPU_ACC_LPF_41,      9,             0,     0,     1,		11,			100},	// ODR=100HZ, mag=8HZ, magodr=100/12=8.3HZ
					//  50Hz Acc  (BW= 20Hz) Gyro (BW= 20Hz) Mag 8Hz
					{ MPU_MODE_ACCGYRMAG,  1,         0,  MPU_GYR_LPF_20,     1,  MPU_ACC_LPF_20,     19,             0,     0,     1,		5,			50},		// ODR=50HZ, mag=8HZ, magodr=50/6=8.3HZ
					//----Magn 100Hz
					// 1000Hz Acc (BW=460Hz) Gyro (BW=250Hz) Mag 100Hz
					{ MPU_MODE_ACCGYRMAG,  1,         0, MPU_GYR_LPF_250,     1, MPU_ACC_LPF_460,      0,             0,     7,     2,		31,			1000},	// ODR=8000HZ, mag=100HZ, magodr=8000/32=250Hz
					// 500Hz Acc  (BW=184Hz) Gyro (BW=250Hz) Mag 100Hz
					{ MPU_MODE_ACCGYRMAG,  1,         0, MPU_GYR_LPF_250,     1, MPU_ACC_LPF_184,      0,             0,    15,     2,		31,			500},	// ODR=8000Hz, mag=8HZ, magodr=8000/32=250Hz					
					// 500Hz Acc  (BW=184Hz) Gyro (BW=184Hz) Mag 100Hz
					{ MPU_MODE_ACCGYRMAG,  1,         0, MPU_GYR_LPF_184,     1, MPU_ACC_LPF_184,      1,             0,     0,     2,		4,			500},		// ODR=500Hz, mag=100Hz, magodr=500/5=100Hz					
					// 200Hz Acc  (BW= 41Hz) Gyro (BW= 41Hz) Mag 100Hz
					{ MPU_MODE_ACCGYRMAG,  1,         0,  MPU_GYR_LPF_92,     1,  MPU_ACC_LPF_92,      4,             0,     0,     2,		1,			200},		// ODR=200HZ, mag=100HZ, magodr=200/2=100Hz
					// 100Hz Acc  (BW= 41Hz) Gyro (BW= 41Hz) Mag 100Hz
					{ MPU_MODE_ACCGYRMAG,  1,         0,  MPU_GYR_LPF_41,     1,  MPU_ACC_LPF_41,      9,             0,     0,     2,		0,			100},		// ODR=100HZ, mag=100HZ, magodr=100/1=100Hz
					
					
					//----Magn 100Hz + Quat
					// 500Hz Acc  (BW=184Hz) Gyro (BW=184Hz) Mag 8Hz
					{ MPU_MODE_ACCGYRMAGQ, 1,         0, MPU_GYR_LPF_184,     1, MPU_ACC_LPF_184,      1,             0,     0,     2,		4,			500},		// ODR=500HZ, mag=100Hz, magodr=500/5=100Hz					
					// 200Hz Acc  (BW= 41Hz) Gyro (BW= 41Hz) Mag 100Hz
					{ MPU_MODE_ACCGYRMAGQ, 1,         0,  MPU_GYR_LPF_92,     1,  MPU_ACC_LPF_92,      4,             0,     0,     2,		1,			200},		// ODR=200HZ, mag=100HZ, magodr=200/2=100Hz					
					// 100Hz Acc  (BW= 41Hz) Gyro (BW= 41Hz) Mag 100Hz
					{ MPU_MODE_ACCGYRMAGQ, 1,         0,  MPU_GYR_LPF_41,     1,  MPU_ACC_LPF_41,      9,             0,     0,     2,		0,			100},		// ODR=100HZ, mag=100HZ, magodr=100/1=100Hz					
					// 500Hz Acc  (BW=184Hz) Gyro (BW=184Hz) Mag 100Hz
					{ MPU_MODE_Q,          1,         0, MPU_GYR_LPF_184,     1, MPU_ACC_LPF_184,      1,             0,     0,     2,		4,			500},		// ODR=500HZ, mag=100HZ, magodr=500/5=100Hz					
					// 200Hz Acc  (BW= 41Hz) Gyro (BW= 41Hz) Mag 100Hz
					{ MPU_MODE_Q,          1,         0,  MPU_GYR_LPF_92,     1,  MPU_ACC_LPF_92,      4,             0,     0,     2,		1,			200},		// ODR=200Hz, mag=100Hz, magodr=200/2=100Hz					
					// 100Hz Acc  (BW= 41Hz) Gyro (BW= 41Hz) Mag 100Hz
					{ MPU_MODE_Q,          1,         0,  MPU_GYR_LPF_41,     1,  MPU_ACC_LPF_41,      9,             0,     0,     2,		0,			100},		// ODR=100HZ, mag=100HZ, magodr=100/1=100Hz
					// 100Hz Acc  (BW= 41Hz) Gyro (BW= 41Hz) Mag 100Hz
					{ MPU_MODE_E,          1,         0,  MPU_GYR_LPF_41,     1,  MPU_ACC_LPF_41,      9,             0,     0,     2,		0,			100},		// ODR=100HZ, mag=100HZ, magodr=100/1=100Hz
					// 100Hz Acc  (BW= 41Hz) Gyro (BW= 41Hz) Mag 100Hz
					{ MPU_MODE_QDBG,       1,         0,  MPU_GYR_LPF_41,     1,  MPU_ACC_LPF_41,      9,             0,     0,     2,		0,			100},		// ODR=100HZ, mag=100HZ, magodr=100/1=100Hz
					
					
					/*
					//----Magn 8Hz + Quat
					// 500Hz Acc  (BW=184Hz) Gyro (BW=184Hz) Mag 8Hz
					{ MPU_MODE_ACCGYRMAGQ, 1,         0, MPU_GYR_LPF_184,     1, MPU_ACC_LPF_184,      1,             0,     0,     1,		31,			500},		// ODR=500HZ, mag=8Hz, magodr=500/32=15.6Hz
					// 200Hz Acc  (BW= 41Hz) Gyro (BW= 41Hz) Mag 8Hz
					{ MPU_MODE_ACCGYRMAGQ, 1,         0,  MPU_GYR_LPF_92,     1,  MPU_ACC_LPF_92,      4,             0,     0,     1,		24,			200},		// ODR=200HZ, mag=8Hz, magodr=200/25=8Hz
					// 100Hz Acc  (BW= 41Hz) Gyro (BW= 41Hz) Mag 8Hz
					{ MPU_MODE_ACCGYRMAGQ, 1,         0,  MPU_GYR_LPF_41,     1,  MPU_ACC_LPF_41,      9,             0,     0,     1,		11,			100},		// ODR=100HZ, mag=8Hz, magodr=100/12=8.3Hz
					// 500Hz Acc  (BW=184Hz) Gyro (BW=184Hz) Mag 100Hz
					{ MPU_MODE_Q,          1,         0, MPU_GYR_LPF_184,     1, MPU_ACC_LPF_184,      1,             0,     0,     1,		31,			500},		// ODR=500HZ, mag=8Hz, magodr=500/32=15.6Hz					
					// 200Hz Acc  (BW= 41Hz) Gyro (BW= 41Hz) Mag 100Hz
					{ MPU_MODE_Q,          1,         0,  MPU_GYR_LPF_92,     1,  MPU_ACC_LPF_92,      4,             0,     0,     1,		24,			200},		// ODR=200Hz, mag=8Hz, magodr=200/32=8Hz					
					// 100Hz Acc  (BW= 41Hz) Gyro (BW= 41Hz) Mag 100Hz
					{ MPU_MODE_Q,          1,         0,  MPU_GYR_LPF_41,     1,  MPU_ACC_LPF_41,      9,             0,     0,     1,		11,			100},		// ODR=100HZ, mag=8Hz, magodr=100/21=8.3Hz
					// 100Hz Acc  (BW= 41Hz) Gyro (BW= 41Hz) Mag 100Hz
					{ MPU_MODE_E,          1,         0,  MPU_GYR_LPF_41,     1,  MPU_ACC_LPF_41,      9,             0,     0,     1,		11,			100},		// ODR=100HZ, mag=8Hz, magodr=100/12=8.3Hz
					// 100Hz Acc  (BW= 41Hz) Gyro (BW= 41Hz) Mag 100Hz
					{ MPU_MODE_QDBG,       1,         0,  MPU_GYR_LPF_41,     1,  MPU_ACC_LPF_41,      9,             0,     0,     1,	   11,			100},		// ODR=100HZ, mag=8Hz, magodr=100/11=8.3
					*/
			};
/******************************************************************************
	function: mpu_config_motionmode
*******************************************************************************	
	Activates a motion sensing mode among a predefined list of available modes.
	
	Parameters:
		sensorsr	-	Motion sensor mode
		autoread	-	Enables automatically reading the data from the motion 
						sensor and place it in a local buffer
*******************************************************************************/			
void mpu_config_motionmode(unsigned char sensorsr,unsigned char autoread)
{
 	// Sanitise
	if(sensorsr>=MOTIONCONFIG_NUM) 
		sensorsr=0;

	// Turn off MPU always
	_mpu_disableautoread();
	mpu_mode_off();
	
		
	//printf("%d\n",sensorsr);
		
	sample_mode = config_sensorsr_settings[sensorsr][0];
	__mpu_sample_softdivider_ctr=0;
	#if HWVER==9
		// In HW9+ softdiv is implemented via a timer/counter.
		//fprintf(file_pri,"softdiv: %d (timer)\n",config_sensorsr_settings[sensorsr][8]);
		__mpu_sample_softdivider_divider = 0;
		init_timer_mpucapture(config_sensorsr_settings[sensorsr][8]);
	#else
		//fprintf(file_pri,"softdiv: %d (legacy)\n",config_sensorsr_settings[sensorsr][8]);
		__mpu_sample_softdivider_divider = 	config_sensorsr_settings[sensorsr][8];
	#endif
	_mpu_samplerate=config_sensorsr_settings[sensorsr][11];
	switch(sample_mode)
	{
		case MPU_MODE_OFF:
			// Already off: do nothing, but continue executing to set the autoread below
			break;
		case MPU_MODE_GYR:
			mpu_mode_gyro(config_sensorsr_settings[sensorsr][1],config_sensorsr_settings[sensorsr][2],config_sensorsr_settings[sensorsr][3],config_sensorsr_settings[sensorsr][6]);
			break;
		case MPU_MODE_ACC:
			//printf("Setting mode acc. acc dlpe %d  acc dlp bw %d div %d softdiv %d\n",config_sensorsr_settings[sensorsr][4],config_sensorsr_settings[sensorsr][5],config_sensorsr_settings[sensorsr][6],config_sensorsr_settings[sensorsr][8]);
			mpu_mode_acc(config_sensorsr_settings[sensorsr][4],config_sensorsr_settings[sensorsr][5],config_sensorsr_settings[sensorsr][6]);
			break;
		case MPU_MODE_ACCGYR:
			mpu_mode_accgyro(config_sensorsr_settings[sensorsr][1],config_sensorsr_settings[sensorsr][2],config_sensorsr_settings[sensorsr][3],
												config_sensorsr_settings[sensorsr][4],config_sensorsr_settings[sensorsr][5],config_sensorsr_settings[sensorsr][6]);
			break;
		case MPU_MODE_ACCGYRMAG:
		case MPU_MODE_ACCGYRMAGQ:
		case MPU_MODE_Q:
		case MPU_MODE_E:
		case MPU_MODE_QDBG:
			// Enable the gyro
			mpu_mode_accgyro(config_sensorsr_settings[sensorsr][1],config_sensorsr_settings[sensorsr][2],config_sensorsr_settings[sensorsr][3],
												config_sensorsr_settings[sensorsr][4],config_sensorsr_settings[sensorsr][5],config_sensorsr_settings[sensorsr][6]);
			
			/*fprintf_P(file_pri,PSTR("Int en\n"));
			_delay_ms(100);
			mpu_mag_interfaceenable(1);
			_delay_ms(100);
			fprintf_P(file_pri,PSTR("Print reg\n"));
			mpu_mag_printreg(file_pri);
			_delay_ms(100);
			fprintf_P(file_pri,PSTR("Mag mode 1\n"));
			mpu_mag_mode(1);
			_delay_ms(100);
			fprintf_P(file_pri,PSTR("Shadow en\n"));
			mpu_mag_regshadow(1,0,3,7);
			fprintf_P(file_pri,PSTR("Done\n"));*/
			// Enable additionally the magnetic field
			//fprintf_P(file_pri,PSTR("Mag mode %d\n"),config_sensorsr_settings[sensorsr][9]);
			_mpu_mag_mode(config_sensorsr_settings[sensorsr][9],config_sensorsr_settings[sensorsr][10]);
			//fprintf_P(file_pri,PSTR("Done\n"));
			break;
		case MPU_MODE_LPACC:
		default:
			mpu_mode_lpacc(config_sensorsr_settings[sensorsr][7]);
	}
	
	if(autoread)
		_mpu_enableautoread();
	//printf("return from mpu_config_motionmode\n");
	
	// Initialise Madgwick
	#if ENABLEQUATERNION==1
	MadgwickAHRSinit(_mpu_samplerate,_mpu_beta,(_mpu_samplerate/100)*8-1);			// All -> 12.5Hz
	//MadgwickAHRSinit(_mpu_samplerate,_mpu_beta,(_mpu_samplerate/100)*4-1);			// All -> 25Hz
	//MadgwickAHRSinit(_mpu_samplerate,_mpu_beta,(_mpu_samplerate/100)*2-1);			// All -> 50Hz
	//MadgwickAHRSinit(_mpu_samplerate,_mpu_beta,(_mpu_samplerate/100)-1);				// All -> 100Hz
	#endif
	
	_mpu_current_motionmode = sensorsr;
}

/******************************************************************************
	function: mpu_get_motionmode
*******************************************************************************	
	Gets the current motion mode, which has been previously set with 
	mpu_config_motionmode.
	
	Parameters:
		autoread		-		Pointer to a variable receiving the autoread parameter.
								If null, then autoread is not returned.
	Returns:
		Current motion mode
*******************************************************************************/			
unsigned char mpu_get_motionmode(unsigned char *autoread)
{
	if(autoread)
		*autoread=__mpu_autoread;
	return _mpu_current_motionmode;
}

/******************************************************************************
	function: mpu_getmodename
*******************************************************************************	
	Returns the name of a motion mode from its ID.
	
	Parameters:
		motionmode	-	Motion sensor mode
		buffer		-	Long enough buffer to hold the name of the motion mode.
						The recommended length is 96 bytes (longest of mc_xx strings).
						The buffer is returned null-terminated.
		Returns:
			Name of the motion mode in buffer, or an empty string if the motionmode
			is invalid.
*******************************************************************************/
void mpu_getmodename(unsigned char motionmode,char *buffer)
{	
	buffer[0]=0;
	if(motionmode>=MOTIONCONFIG_NUM)
		return;
	strcpy_P(buffer,(PGM_P)pgm_read_word(mc_options+motionmode));
}


/******************************************************************************
	function: mpu_printmotionmode
*******************************************************************************	
	Prints the available MPU sensing modes
	
	Parameters:
		file	-	Output stream
*******************************************************************************/
void mpu_printmotionmode(FILE *file)
{	
	fprintf_P(file,PSTR("Motion modes (x indicates experimental modes; sample rate not respected):\n"));		
	for(unsigned char i=0;i<MOTIONCONFIG_NUM;i++)
	{
		char buf[96];
		mpu_getmodename(i,buf);
		fprintf_P(file,PSTR("[%d]\t"),i);
		fputs(buf,file);
		fputc('\n',file);
	}		
}


