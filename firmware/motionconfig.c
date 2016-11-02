#include "cpu.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include "mpu.h"
#include "main.h"
#include "motionconfig.h"

/*
	File: motionconfig
		
*/

const char mc_0[] PROGMEM = "Motion off";
const char mc_1[] PROGMEM = "  500Hz Gyro (BW=250Hz)";
const char mc_2[] PROGMEM = "  500Hz Gyro (BW=184Hz)";
const char mc_3[] PROGMEM = "  200Hz Gyro (BW= 92Hz)";
const char mc_4[] PROGMEM = "  100Hz Gyro (BW= 41Hz)";
const char mc_5[] PROGMEM = "   50Hz Gyro (BW= 20Hz)";
const char mc_6[] PROGMEM = "   10Hz Gyro (BW=  5Hz)";
const char mc_7[] PROGMEM = "    1Hz Gyro (BW=  5Hz)";
const char mc_8[] PROGMEM = " 1000Hz Acc  (BW=460Hz)";
const char mc_9[] PROGMEM = "  500Hz Acc  (BW=184Hz)";
const char mc_10[] PROGMEM = " 200Hz Acc  (BW= 92Hz)";
const char mc_11[] PROGMEM = " 100Hz Acc  (BW= 41Hz)";
const char mc_12[] PROGMEM = "  50Hz Acc  (BW= 20Hz)";
const char mc_13[] PROGMEM = "  10Hz Acc  (BW=  5Hz)";
const char mc_14[] PROGMEM = "   1Hz Acc  (BW=  5Hz)";
const char mc_15[] PROGMEM = " 1000Hz Acc  (BW=460Hz) Gyro (BW=250Hz)";
const char mc_16[] PROGMEM = "  500Hz Acc  (BW=184Hz) Gyro (BW=250Hz)";
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
const char mc_29[] PROGMEM = "  100Hz Acc  (BW= 41Hz) Gyro (BW= 41Hz) Mag 8Hz";
const char mc_30[] PROGMEM = "  100Hz Acc  (BW= 41Hz) Gyro (BW= 41Hz) Mag 100Hz";
const char mc_31[] PROGMEM = "  200Hz Acc  (BW= 92Hz) Gyro (BW= 92Hz) Mag 100Hz";
const char mc_32[] PROGMEM = "  100Hz Acc  (BW= 41Hz) Gyro (BW= 41Hz) Mag 100Hz Quaternions";
const char mc_33[] PROGMEM = "  200Hz Acc  (BW= 92Hz) Gyro (BW= 92Hz) Mag 100Hz Quaternions";
const char mc_34[] PROGMEM = "  100Hz Quaternions";
const char mc_35[] PROGMEM = "  200Hz Quaternions";

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
	mc_35
};



/******************************************************************************
	config_motion
*******************************************************************************	
	Configures the motion sensor based (active sensors and sample rate) on 
	config_sensorsr.
	
	config_mpu_settings contains for each option: mode gdlpe gdlpoffhbw gdlpbw adlpe adlpbw divider lpodr softdiv
	mode is 0=gyro, 1=acc, 2=gyroacc, 3=lpacc
	
	magmode: 0=off, 1=8Hz, 2=100Hz. Node that magmode is only used in moe MPU_MODE_ACCGYRMAG
******************************************************************************/
//const char hello[] PROGMEM = {1,2,3};

//const char config_sensorsr_settings[MOTIONCONFIG_NUM][10] PROGMEM = {
const char config_sensorsr_settings[MOTIONCONFIG_NUM][10] = {
					// mode              gdlpe gdlpoffhbw        gdlpbw     adlpe           adlpbw divider          lpodr softdiv	magmode
					// Off
					{ MPU_MODE_OFF,        0,         0,               0,     0,               0,      0,             0,     0,     0},
					// 500Hz Gyro (BW=250Hz)
					{ MPU_MODE_GYR,        1,         0, MPU_GYR_LPF_250,     0,               0,      0,             0,     15,    0},
					// 500Hz Gyro (BW=184Hz)
					{ MPU_MODE_GYR,        1,         0, MPU_GYR_LPF_184,     0,               0,      1,             0,     0,     0},
					// 200Hz Gyro (BW= 92Hz)
					{ MPU_MODE_GYR,        1,         0,  MPU_GYR_LPF_92,     0,               0,      4,             0,     0,     0},
					// 100Hz Gyro (BW= 41Hz)
					{ MPU_MODE_GYR,        1,         0,  MPU_GYR_LPF_41,     0,               0,      9,             0,     0,     0},
					// 50Hz Gyro (BW= 20Hz)
					{ MPU_MODE_GYR,        1,         0,  MPU_GYR_LPF_20,     0,               0,     19,             0,     0,     0},
					// 10Hz Gyro (BW=  5Hz)
					{ MPU_MODE_GYR,        1,         0,   MPU_GYR_LPF_5,     0,               0,     99,             0,     0,     0},
					// 1Hz Gyro (BW=  5Hz)
					{ MPU_MODE_GYR,        1,         0,   MPU_GYR_LPF_5,     0,               0,     99,             0,     9,     0},
					// 1000Hz Acc  (BW=460Hz)
					{ MPU_MODE_ACC,        0,         0,               0,     1, MPU_ACC_LPF_460,      0,             0,     0,     0},
					// 500Hz Acc  (BW=184Hz)
					{ MPU_MODE_ACC,        0,         0,               0,     1, MPU_ACC_LPF_184,      1,             0,     0,     0},
					// 200Hz Acc  (BW= 92Hz)
					{ MPU_MODE_ACC,        0,         0,               0,     1,  MPU_ACC_LPF_92,      4,             0,     0,     0},
					// 100Hz Acc  (BW= 41Hz)
					{ MPU_MODE_ACC,        0,         0,               0,     1,  MPU_ACC_LPF_41,      9,             0,     0,     0},
					// 50Hz Acc  (BW= 20Hz)
					{ MPU_MODE_ACC,        0,         0,               0,     1,  MPU_ACC_LPF_20,     19,             0,     0,     0},
					// 10Hz Acc  (BW=  5Hz)
					{ MPU_MODE_ACC,        0,         0,               0,     1,   MPU_ACC_LPF_5,     99,             0,     0,     0},
					// 1Hz Acc  (BW=  5Hz)
					{ MPU_MODE_ACC,        0,         0,               0,     1,   MPU_ACC_LPF_5,     99,             0,     9,     0},
					// 1000Hz Acc  (BW=460Hz) Gyro (BW=250Hz)
					{ MPU_MODE_ACCGYR,     1,         0, MPU_GYR_LPF_250,     1, MPU_ACC_LPF_460,      0,             0,     7,     0},
					// 500Hz Acc  (BW=184Hz) Gyro (BW=250Hz)
					{ MPU_MODE_ACCGYR,     1,         0, MPU_GYR_LPF_250,     1, MPU_ACC_LPF_184,      0,             0,    15,     0},
					// 500Hz Acc  (BW=184Hz) Gyro (BW=184Hz)
					{ MPU_MODE_ACCGYR,     1,         0, MPU_GYR_LPF_184,     1, MPU_ACC_LPF_184,      1,             0,     0,     0},
					// 200Hz Acc  (BW= 92Hz) Gyro (BW= 92Hz)
					{ MPU_MODE_ACCGYR,     1,         0,  MPU_GYR_LPF_92,     1,  MPU_ACC_LPF_92,      4,             0,     0,     0},
					// 100Hz Acc  (BW= 41Hz) Gyro (BW= 41Hz)
					{ MPU_MODE_ACCGYR,     1,         0,  MPU_GYR_LPF_41,     1,  MPU_ACC_LPF_41,      9,             0,     0,     0},
					// 50Hz Acc  (BW= 20Hz) Gyro (BW= 20Hz)
					{ MPU_MODE_ACCGYR,     1,         0,  MPU_GYR_LPF_20,     1,  MPU_ACC_LPF_20,     19,             0,     0,     0},
					// 10Hz Acc  (BW=  5Hz) Gyro (BW=  5Hz)
					{ MPU_MODE_ACCGYR,     1,         0,   MPU_GYR_LPF_5,     1,   MPU_ACC_LPF_5,     99,             0,     0,     0},
					// 1Hz Acc  (BW=  5Hz) Gyro (BW=  5Hz)
					{ MPU_MODE_ACCGYR,     1,         0,   MPU_GYR_LPF_5,     1,   MPU_ACC_LPF_5,     99,             0,     9,     0},
					// 500Hz Acc low power
					{ MPU_MODE_LPACC,      0,         0,               0,     0,               0,      0, MPU_LPODR_500,     0,     0},
					// 250Hz Acc low power
					{ MPU_MODE_LPACC,      0,         0,               0,     0,               0,      0, MPU_LPODR_250,     0,     0},
					// 125Hz Acc low power
					{ MPU_MODE_LPACC,      0,         0,               0,     0,               0,      0, MPU_LPODR_125,     0,     0},
					// 62.5Hz Acc low power
					{ MPU_MODE_LPACC,      0,         0,               0,     0,               0,      0,  MPU_LPODR_62,     0,     0},
					// 31.25Hz Acc low power
					{ MPU_MODE_LPACC,      0,         0,               0,     0,               0,      0,  MPU_LPODR_31,     0,     0},
					// 1Hz Acc low power
					{ MPU_MODE_LPACC,      0,         0,               0,     0,               0,      0,   MPU_LPODR_1,     0,     0},
					// 100Hz Acc  (BW= 41Hz) Gyro (BW= 41Hz) Mag 8Hz
					{ MPU_MODE_ACCGYRMAG,  1,         0,  MPU_GYR_LPF_41,     1,  MPU_ACC_LPF_41,      9,             0,     0,     1},
					// 100Hz Acc  (BW= 41Hz) Gyro (BW= 41Hz) Mag 100Hz
					{ MPU_MODE_ACCGYRMAG,  1,         0,  MPU_GYR_LPF_41,     1,  MPU_ACC_LPF_41,      9,             0,     0,     2},
					// 200Hz Acc  (BW= 41Hz) Gyro (BW= 41Hz) Mag 100Hz
					{ MPU_MODE_ACCGYRMAG,  1,         0,  MPU_GYR_LPF_92,     1,  MPU_ACC_LPF_92,      4,             0,     0,     2},
					// 100Hz Acc  (BW= 41Hz) Gyro (BW= 41Hz) Mag 100Hz
					{ MPU_MODE_ACCGYRMAGQ, 1,         0,  MPU_GYR_LPF_41,     1,  MPU_ACC_LPF_41,      9,             0,     0,     2},
					// 200Hz Acc  (BW= 41Hz) Gyro (BW= 41Hz) Mag 100Hz
					{ MPU_MODE_ACCGYRMAGQ, 1,         0,  MPU_GYR_LPF_92,     1,  MPU_ACC_LPF_92,      4,             0,     0,     2},
					// 100Hz Acc  (BW= 41Hz) Gyro (BW= 41Hz) Mag 100Hz
					{ MPU_MODE_Q,          1,         0,  MPU_GYR_LPF_41,     1,  MPU_ACC_LPF_41,      9,             0,     0,     2},
					// 200Hz Acc  (BW= 41Hz) Gyro (BW= 41Hz) Mag 100Hz
					{ MPU_MODE_Q,          1,         0,  MPU_GYR_LPF_92,     1,  MPU_ACC_LPF_92,      4,             0,     0,     2}
			};
/******************************************************************************
	function: config_motion
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
	__mpu_sample_softdivider_divider = 	config_sensorsr_settings[sensorsr][8];
	switch(sample_mode)
	{
		case MPU_MODE_OFF:
			// Already off: return immediately to avoid further processing and prevent code after the switch to be executed
			return;
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
			_mpu_mag_mode(config_sensorsr_settings[sensorsr][9]);
			//fprintf_P(file_pri,PSTR("Done\n"));
			break;
		case MPU_MODE_LPACC:
		default:
			mpu_mode_lpacc(config_sensorsr_settings[sensorsr][7]);
	}
	
	if(autoread)
		_mpu_enableautoread();
	//printf("return from mpu_config_motionmode\n");
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
	fprintf_P(file,PSTR("Motion modes:\n"));		
	for(unsigned char i=0;i<MOTIONCONFIG_NUM;i++)
	{
		char buf[64];
		strcpy_P(buf,(PGM_P)pgm_read_word(mc_options+i));
		fprintf_P(file,PSTR("[%d]\t"),i);
		fputs(buf,file);
		fputc('\n',file);
	}		
}
