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
#include <stdlib.h>
#include <stdfix.h>
#include <stdint.h>

#include "main.h"
#include "mpu.h"
#include "pkt.h"
#include "wait.h"
#include "lcd.h"
#include "fb.h"
#include "system.h"
#include "helper.h"
#include "serial.h"
#include "dbg.h"
#include "mode_mputest.h"
#include "mode_global.h"
#include "motionconfig.h"
#include "commandset.h"
#include "uiconfig.h"


#include "MadgwickAHRS.h"
#include "MadgwickAHRS2.h"
#include "mathfix.h"

#define PI 3.1415926535f

#define FP_FBITS        15 //fraction bits
//conversion Functions
#define itok(i)         ( (int32_t)( (int32_t)i<<(int32_t)FP_FBITS ) )
#define ktoi(k)         ( ( (int16_t)( (int32_t)k>>(int32_t)FP_FBITS ) )&0x0000ffff )
#define ftok(f)         ( (int32_t)(float)( (f)*(32768) ) )
extern float FP_FixedToFloat(int32_t);
extern int32_t FP_FloatToFixed(float);

/*
	Todo:
	- move to fixed point implementation only of reciprocal square root
	- test int24_t
	- benchmark 32-bit mul/add: 2400-3000uS
	- benchmark 24-bit mul/add: 1600-1977uS. 50% speedup
	- analyse needs for integer and 
*/

/*int32_t __attribute__((naked)) FP_FloatToFixed(float f) {
  asm volatile (
    //__fractsfsa
    "subi  r24, 0x80    \n"
    "sbci  r25, 0xf8    \n"
    "call __fixunssfsi \n"
    "set                \n"
    "cpse  r27, r1      \n"
    "rjmp  __fp_zero    \n"
    "ret                \n"
  );
}*/
/* 
float __attribute__((naked)) FP_FixedToFloat(int32_t k) {
  asm volatile (
    //__fractsasf
    "call __floatsisf \n"
    "tst   r25         \n"
    "breq  1f          \n"
    "subi  r24, 0x80   \n"
    "sbci  r25, 0x07   \n"
    "1:                \n"
    "ret               \n"
  );
}*/
/*
	file: mode_mputest
	
	Interactive test functions for the MPU9250
*/

const char help_mt_r[] PROGMEM="Dumps registers";
const char help_mt_S[] PROGMEM ="S[,<mode>,<autoread>] setup motion sensor mode";
const char help_mt_c[] PROGMEM ="Calibrate acc/gyro";
const char help_mt_x[] PROGMEM ="Reset";
const char help_mt_fd[] PROGMEM ="Dump FIFO content";
const char help_mt_fe[] PROGMEM ="f,flags,en,rst: Set FIFO flags (TEMP GX GY GZ ACC SLV2 SLV1 SLV0), enable (1), reset (1)";
const char help_mt_t[] PROGMEM ="Test acc/gyro sampling";
const char help_mt_M[] PROGMEM ="Magnetometer registers";
const char help_mt_m[] PROGMEM ="m,<0|1|2> magnetometer: power down|enable 8Hz|enable 100Hz";
const char help_mt_I[] PROGMEM ="I,<0|1> disable|enable I2C";
const char help_mt_E[] PROGMEM ="MPU external sensor registers";
const char help_mt_W[] PROGMEM ="W,en,dly,regstart,numreg: en=1|0 enables|disables shadowing of numreg registers from regstart at frequcency ODR/(1+dly)";
const char help_mt_O[] PROGMEM ="MPU off";
const char help_mt_P[] PROGMEM ="Poll acc, gyr, magn, temp";
const char help_mt_A[] PROGMEM ="A[,<mode] Auto acquire (interrupt-driven) test";
const char help_mt_B[] PROGMEM ="Benchmark overheads of auto acquire";
const char help_mt_Q[] PROGMEM ="Q[,<bitmap>] Quaternion test; 3-bit bitmap indicates acc|gyr|mag (mag is lsb)";
const char help_mt_G[] PROGMEM ="G[,<mode>] Calibrate magnetometer, or 0='no correction' 1='correction w/ factory', 2='user correction'";
const char help_mt_g[] PROGMEM ="Magnetic selt test";
const char help_mt_b[] PROGMEM ="bench math";



#define CommandParsersMPUTestNum 22
const COMMANDPARSER CommandParsersMPUTest[CommandParsersMPUTestNum] =
{ 
	{'H', CommandParserHelp,help_h},
	{'R', CommandParserMPUTest_ReadReg,help_mt_r},
	{'S', CommandParserMPUTest_Start,help_mt_S},
	{'C', CommandParserMPUTest_Calibrate,help_mt_c},
	{'X', CommandParserMPUTest_Reset,help_mt_x},
	{'F', CommandParserMPUTest_Fifo,help_mt_fd},
	{'f', CommandParserMPUTest_FifoEn,help_mt_fe},
	{'T', CommandParserMPUTest_Test,help_mt_t},
	{'P', CommandParserMPUTest_Poll,help_mt_P},
	{'A', CommandParserMPUTest_Auto,help_mt_A},
	{'M', CommandParserMPUTest_Magn,help_mt_M},
	{'m', CommandParserMPUTest_MagnMode,help_mt_m},
	{'I', CommandParserMPUTest_Interface,help_mt_I},
	{'E', CommandParserMPUTest_External,help_mt_E},
	{'W', CommandParserMPUTest_Shadow,help_mt_W},
	{'O', CommandParserMPUTest_Off,help_mt_O},
	{'B', CommandParserMPUTest_Bench,help_mt_B},
	{'Q', CommandParserMPUTest_Quaternion,help_mt_Q},
	{'G', CommandParserMPUTest_MagneticCalib,help_mt_G},
	{'g', CommandParserMPUTest_MagneticSelfTest,help_mt_g},
	{'b', CommandParserMPUTest_BenchMath,help_mt_b},
	{'!', CommandParserQuit,help_quit}
};



/******************************************************************************
	CommandParserMPUTest_ReadReg
*******************************************************************************
	Dump MPU registers
******************************************************************************/
unsigned char CommandParserMPUTest_ReadReg(unsigned char *buffer,unsigned char size)
{
	mpu_printreg(file_pri);
	mpu_printregdesc(file_pri);
	mpu_printregdesc2(file_pri);
	return 0;
}

/******************************************************************************
	CommandParserMPUTest_ReadReg
*******************************************************************************
	Dump MPU registers
******************************************************************************/
unsigned char CommandParserMPUTest_Start(unsigned char *buffer,unsigned char size)
{
	unsigned char rv;
	int mode,autoread;
	
	rv = ParseCommaGetInt((char*)buffer,2,&mode,&autoread);
	if(rv)
	{
		// print help
		mpu_printmotionmode(file_pri);
		return 2;
	}
	if(mode<0 || mode>MOTIONCONFIG_NUM)
		return 2;
	

	// Activate the mode
	mpu_config_motionmode(mode,autoread);

	return 0;
}






/******************************************************************************
	CommandParserMPUTest_Calibrate
*******************************************************************************
	Calibrate accelerometer and gyroscope. Requires the sensor to be at rest.
	Data is sampled in the FIFO and the averages computed. The offsets are 
	then loaded in the bias registers.
******************************************************************************/
unsigned char CommandParserMPUTest_Calibrate(unsigned char *buffer,unsigned char size)
{
	mpu_calibrate();
	
	
	return 0;
}

unsigned char CommandParserMPUTest_Reset(unsigned char *buffer,unsigned char size)
{
	mpu_reset();
	_delay_ms(100);
	return 0;
}

unsigned char CommandParserMPUTest_Fifo(unsigned char *buffer,unsigned char size)
{
	mpu_printfifo(file_pri);

	return 0;
}
unsigned char CommandParserMPUTest_FifoEn(unsigned char *buffer,unsigned char size)
{
	unsigned char rv;
	int flags,en,reset;
	
	rv = ParseCommaGetInt((char*)buffer,3,&flags,&en,&reset);
	if(rv)
	{
		return 2;
	}
	mpu_fifoenable(flags,en,reset);
	return 0;
}

unsigned char CommandParserMPUTest_Test(unsigned char *buffer,unsigned char size)
{
	mpu_acquirecalib();

	unsigned short n;
	signed short ax,ay,az,gx,gy,gz;
	n = mpu_getfifocnt();
	fprintf_P(file_pri,PSTR("FIFO level: %d\n"),n);
	unsigned short ns = n/12;
	for(unsigned short i=0;i<ns;i++)
	{
		mpu_fiforeadshort(&ax,1);
		mpu_fiforeadshort(&ay,1);
		mpu_fiforeadshort(&az,1);
		mpu_fiforeadshort(&gx,1);
		mpu_fiforeadshort(&gy,1);
		mpu_fiforeadshort(&gz,1);
		fprintf_P(file_pri,PSTR("%d %d %d  %d %d %d\n"),ax,ay,az,gx,gy,gz);
	}
	return 0;
}
unsigned char CommandParserMPUTest_Magn(unsigned char *buffer,unsigned char size)
{/*
	fprintf_P(file_pri,PSTR("Int en\n"));
	mpu_mag_interfaceenable(1);
	mpu_mag_printreg(file_pri);
	
	mpu_mag_writereg(0x0a,0b00010010);		// Continuous mode 1, 16 bit
	//mpu_mag_writereg(0x0a,0b00010001);		// Single mode, 16 bit
	
	for(int i=0;i<5;i++)
	{
		fprintf_P(file_pri,PSTR("%d\n"),i);
		mpu_mag_printreg(file_pri);
		_delay_ms(100);
	}*/
	
	mpu_mag_printreg(file_pri);
	
	
	return 0;
}

unsigned char CommandParserMPUTest_MagnMode(unsigned char *buffer,unsigned char size)
{
	unsigned char rv;
	int mode;
	
	rv = ParseCommaGetInt((char*)buffer,1,&mode);
	if(rv)
	{
		return 2;
	}
	if(mode>2) mode=2;
	fprintf_P(file_pri,PSTR("Magn mode: %d... "),mode);
	_mpu_mag_mode(mode);
	fprintf_P(file_pri,PSTR("done\n"));	

	return 0;



/*	_delay_ms(100);
	//mpu_mag_enable(1);
	mpu_mag_interfaceenable(1);
	_delay_ms(100);
	//fprintf_P(file_pri,PSTR("print reg\n"));
	//mpu_mag_printreg(file_pri);
	unsigned char r = mpu_mag_readreg(0);
	fprintf_P(file_pri,PSTR("mag reg %02x: %02x\n"),0,r);
	_delay_ms(100);
	fprintf_P(file_pri,PSTR("Before mag enable\n"));
	mpu_mag_enable(1);
	_delay_ms(100);
	fprintf_P(file_pri,PSTR("After mag enable\n"));

*/

	/*mpu_printextreg(file_pri);
	
	mpu_writereg(MPU_R_I2C_SLV4_CTRL,0b00000011);			// R 52: Slave 4 disabled; set I2C_MST_DLY
	
	
	unsigned char usr = mpu_readreg(MPU_R_USR_CTRL);		// R 106
	usr = usr&0b11011111;
	usr = usr|0b00110010;
	mpu_writereg(MPU_R_USR_CTRL,usr);						// R106 Enable MST_I2C, disable I2C_IF, reset I2C
	
	mpu_writereg(MPU_R_I2C_MST_CTRL,0b11000000);			// R 36: enable multi-master (needed?), wait_for_es, set I2C clock
	
	mpu_writereg(MPU_R_I2C_SLV0_ADDR,MAG_ADDRESS|0x80);
	mpu_writereg(MPU_R_I2C_SLV0_REG,0);
	mpu_writereg(MPU_R_I2C_SLV0_DO,0);
	mpu_writereg(103,0b00000001);							// R 103	I2C_SLV0_DLY_EN	// READS MULTIPLE TIMES!
	mpu_writereg(MPU_R_I2C_SLV0_CTRL,0b10001010);			// R 39: SLV0_EN, len=10
	*/
	
	//mpu_writereg(MPU_R_I2C_MST_DELAY_CTRL,0b00000000);	// No shadowing delay (TODO: maybe should have), no delay on slaves
	//mpu_writereg(MPU_R_I2C_MST_DELAY_CTRL,0b10000001);	// 
	//mpu_writereg(MPU_R_I2C_MST_DELAY_CTRL,0b00000000);	// No shadowing delay (TODO: maybe should have), no delay on slaves

	
	/*mpu_mag_regshadow(0,3,7);			// Read magnetic field and status register
	
	
	for(int i=0;i<10;i++)
	{
		fprintf_P(file_pri,PSTR("%d\n"),i);
		mpu_printextreg(file_pri);
		_delay_ms(100);
	}*/
	
	
	/*fprintf_P(file_pri,PSTR("dout: %02X\n"),mpu_readreg(MPU_R_I2C_SLV0_DO));
	
	_delay_ms(1000);
	
	fprintf_P(file_pri,PSTR("dout: %02X\n"),mpu_readreg(MPU_R_I2C_SLV0_DO));
	
	mpu_printextreg(file_pri);
	
	*/
	
	/*
	// Working with multiple reads
	// R103 must have I2C_SLVx_DLY_EN=1 to enable periodic read. Unsure whether R103 must have DELAY_ES_SHADOW
	unsigned char usr = mpu_readreg(MPU_R_USR_CTRL);		// R 106
	usr = usr&0b11011111;
	usr = usr|0b00110010;
	mpu_writereg(MPU_R_USR_CTRL,usr);
	
	
	mpu_writereg(MPU_R_I2C_MST_CTRL,0b11000000);			// R 36
	mpu_writereg(MPU_R_I2C_SLV4_ADDR,MAG_ADDRESS|0x80);
	mpu_writereg(MPU_R_I2C_SLV4_REG,3);
	mpu_writereg(MPU_R_I2C_SLV4_DO,0);
	mpu_writereg(103,0b00010000);							// R 103		// READS MULTIPLE TIMES!
	//mpu_writereg(103,0b10010000);							// R 103		// READS MULTIPLE TIMES!
	//mpu_writereg(103,0b00000000);							// R 103 		// reads only once
	fprintf_P(file_pri,PSTR("slv4ctrl: %02X do: %02X di: %02x mst: %02x\n"),mpu_readreg(MPU_R_I2C_SLV4_CTRL),mpu_readreg(MPU_R_I2C_SLV4_DO),mpu_readreg(MPU_R_I2C_SLV4_DI),mpu_readreg(MPU_R_I2C_MST_STATUS));
	mpu_writereg(MPU_R_I2C_SLV4_CTRL,0b10000111);							// reads only once
	//mpu_writereg(MPU_R_I2C_SLV4_CTRL,0b11000000);							// reads only once
	//mpu_writereg(MPU_R_I2C_SLV4_CTRL,0b10000000);			// R 52			// reads only once
	
	// MPU_R_I2C_MST_STATUS = R 54
	for(int i=0;i<20;i++)
	{
		fprintf_P(file_pri,PSTR("slv4ctrl: %02X do: %02X di: %02x mst: %02x\n"),mpu_readreg(MPU_R_I2C_SLV4_CTRL),mpu_readreg(MPU_R_I2C_SLV4_DO),mpu_readreg(MPU_R_I2C_SLV4_DI),mpu_readreg(MPU_R_I2C_MST_STATUS));
		_delay_ms(125);
	}*/
	
	
	
	return 0;
}
unsigned char CommandParserMPUTest_Interface(unsigned char *buffer,unsigned char size)
{
	unsigned char rv;
	int en;
	
	rv = ParseCommaGetInt((char*)buffer,1,&en);
	if(rv)
	{
		return 2;
	}
	en=en?1:0;
	fprintf_P(file_pri,PSTR("Int en: %d... "),en);
	_mpu_mag_interfaceenable(en);
	fprintf_P(file_pri,PSTR("done\n"));	

	return 0;
}
unsigned char CommandParserMPUTest_External(unsigned char *buffer,unsigned char size)
{
	mpu_printextreg(file_pri);
	return 0;
}
unsigned char CommandParserMPUTest_Shadow(unsigned char *buffer,unsigned char size)
{
	unsigned char rv;
	int en,dly,regstart,numreg;
	
	rv = ParseCommaGetInt((char*)buffer,4,&en,&dly,&regstart,&numreg);
	if(rv)
	{
		return 2;
	}
	en=en?1:0;
	if(dly>31) dly=31; if(dly<0) dly=0;
	if(numreg>15) numreg=15; if(numreg<0) numreg=0;
	fprintf_P(file_pri,PSTR("Shadow en %d dly %d regstart %d numreg %d... "),en,dly,regstart,numreg);
	_mpu_mag_regshadow(en,dly,regstart,numreg);
	fprintf_P(file_pri,PSTR("done\n"));	

	return 0;
}
unsigned char CommandParserMPUTest_Off(unsigned char *buffer,unsigned char size)
{
	mpu_mode_off();
	return 0;
}
unsigned char CommandParserMPUTest_Poll(unsigned char *buffer,unsigned char size)
{
	char str[128];
	signed short ax,ay,az,gx,gy,gz,mx,my,mz,temp;
	unsigned char ms;
	WAITPERIOD p=0;
	//unsigned long time;
	unsigned long t1,t2;

	for(unsigned int i=0;i<200;i++)
	{
		timer_waitperiod_us(5000,&p);
		
		mpu_get_agmt(&ax,&ay,&az,&gx,&gy,&gz,&mx,&my,&mz,&ms,&temp);
		//fprintf_P(file_pri,PSTR("Sample %d\n"),i);
		char *strptr=str;
		s16toa(ax,strptr);
		strptr+=6;
		*strptr=' ';
		strptr++;
		s16toa(ay,strptr);
		strptr+=6;
		*strptr=' ';
		strptr++;
		s16toa(az,strptr);
		strptr+=6;
		*strptr=' ';
		strptr++;
		*strptr=' ';
		strptr++;
		s16toa(gx,strptr);
		strptr+=6;
		*strptr=' ';
		strptr++;
		s16toa(gy,strptr);
		strptr+=6;
		*strptr=' ';
		strptr++;
		s16toa(gz,strptr);
		strptr+=6;
		*strptr=' ';
		strptr++;
		*strptr=' ';
		strptr++;
		s16toa(mx,strptr);
		strptr+=6;
		*strptr=' ';
		strptr++;
		s16toa(my,strptr);
		strptr+=6;
		*strptr=' ';
		strptr++;
		s16toa(mz,strptr);
		strptr+=6;
		*strptr=' ';
		strptr++;
		*strptr=' ';
		strptr++;
		s16toa(temp,strptr);
		strptr+=6;
		*strptr='\n';		
		strptr++;
		fputbuf(file_pri,str,strptr-str);
		
		
	}
	
	
	t1=timer_ms_get();
	for(unsigned short i=0;i<1000;i++)
	{
		mpu_get_agmt(&ax,&ay,&az,&gx,&gy,&gz,&mx,&my,&mz,&ms,&temp);
	}
	t2=timer_ms_get();
	fprintf_P(file_pri,PSTR("Time for 1K mpu_get_agmt: %ld ms\n"),t2-t1);
	t1=timer_ms_get();
	for(unsigned short i=0;i<1000;i++)
	{
		mpu_get_agt(&ax,&ay,&az,&gx,&gy,&gz,&temp);
	}
	t2=timer_ms_get();
	fprintf_P(file_pri,PSTR("Time for 1K mpu_get_agt: %ld ms\n"),t2-t1);
	t1=timer_ms_get();
	for(unsigned short i=0;i<1000;i++)
	{
		mpu_get_a(&ax,&ay,&az);
	}
	t2=timer_ms_get();
	fprintf_P(file_pri,PSTR("Time for 1K mpu_get_a: %ld ms\n"),t2-t1);
	
	return 0;
}

/******************************************************************************
	CommandParserMPUTest_Auto
*******************************************************************************
	
******************************************************************************/
unsigned char CommandParserMPUTest_Auto(unsigned char *buffer,unsigned char size)
{
	unsigned char rv;
	int mode;
	WAITPERIOD p=0;
	
	rv = ParseCommaGetInt((char*)buffer,1,&mode);
	if(rv)
	{
		// print help
		mpu_printmotionmode(file_pri);
		return 2;
	}
	if(mode<0 || mode>=MOTIONCONFIG_NUM)
		return 2;
	
	fprintf_P(file_pri,PSTR("Activating mode %d and testing until keypress\n"),mode);
	// Activate the mode
	mpu_config_motionmode(mode,1);
	
	while(1)
	{
		if( fgetc(file_pri) != -1)
			break;
		timer_waitperiod_ms(33,&p);
			
		for(unsigned char i=0;i<mpu_data_level();i++)
		{
			fprintf_P(file_pri,PSTR("%02u %ld: %d %d %d  *  "),i,mpu_data_time[mpu_data_rdptr],mpu_data_ax[mpu_data_rdptr],mpu_data_ay[mpu_data_rdptr],mpu_data_az[mpu_data_rdptr]);
			fprintf_P(file_pri,PSTR("%d %d %d  *  "),mpu_data_gx[mpu_data_rdptr],mpu_data_gy[mpu_data_rdptr],mpu_data_gz[mpu_data_rdptr]);
			fprintf_P(file_pri,PSTR("%d %d %d  (%d)  *  "),mpu_data_mx[mpu_data_rdptr],mpu_data_my[mpu_data_rdptr],mpu_data_mz[mpu_data_rdptr],mpu_data_ms[mpu_data_rdptr]);
			fprintf_P(file_pri,PSTR("%d\n"),mpu_data_temp[mpu_data_rdptr]);
			mpu_data_rdnext();
		}
	}
	mpu_config_motionmode(MPU_MODE_OFF,0);

	return 0;
}
/******************************************************************************
	CommandParserMPUTest_Bench
*******************************************************************************
	
******************************************************************************/
unsigned char CommandParserMPUTest_Bench(unsigned char *buffer,unsigned char size)
{
	long int perf,refperf;
	mpu_config_motionmode(MPU_MODE_OFF,0);
	fprintf_P(file_pri,PSTR("Benchmarking all auto acquire modes\n"));
	refperf = main_perfbench();
	fprintf_P(file_pri,PSTR("Reference performance: %lu\n"),refperf);
	for(unsigned char mode=MPU_MODE_OFF;mode<MOTIONCONFIG_NUM;mode++)
	{
		fprintf_P(file_pri,PSTR("Benchmarking mode %d\n"),mode);
		mpu_config_motionmode(mode,1);
		
		perf = main_perfbench();
		
		mpu_config_motionmode(MPU_MODE_OFF,0);
		mpu_printstat(file_pri);
		
		long load = 100-(perf*100/refperf);
		if(load<0)
			load=0;
		
		fprintf_P(file_pri,PSTR("Mode %d: perf: %lu (instead of %lu). CPU load %lu %%\n"),mode,perf,refperf,load);
	}
	

	return 0;
}
/******************************************************************************
	CommandParserMPUTest_Quaternion
*******************************************************************************
	
******************************************************************************/
#if FIXEDPOINTFILTER==0
unsigned char CommandParserMPUTest_Quaternion(unsigned char *buffer,unsigned char size)
{
	#if ENABLEQUATERNION==1
	//unsigned long t1,t2;
	float ax,ay,az,gx,gy,gz,mx,my,mz;
	mx=my=mz=0;
	
	unsigned char rv;
	int mode;
	
	rv = ParseCommaGetInt(buffer,1,&mode);
	if(rv)
	{
		mode=0x07;
	}
	mode=mode&0x07;
	
	
	/*ax=1.1;
	ay=0.8;
	az=-.1;
	gx=.2;
	gy=.9;
	gz=-2;
	mx=2;
	my=-3;
	mz=4;
	
	t1 = timer_ms_get();
	for(unsigned i=0;i<1000;i++)
	{
		MadgwickAHRSupdate(gx,gy,gz,ax,ay,az,mx,my,mz);
	}
	t2 = timer_ms_get();
	printf("Time for 1000: %ld ms\n",t2-t1);*/
	
	// Enable sampling
	mpu_config_motionmode(MPU_MODE_100HZ_ACC_BW41_GYRO_BW41_MAG_100,1);
	printf("Starting loop\n");
	
	float gtor=PI/180.0f/131.0f;
	float atog=1.0/16384;
	
	q0 = 1.0f;
	q1 = 0.0f;
	q2 = 0.0f;
	q3 = 0.0f;
	
	//t1 = timer_ms_get();
	//while(timer_ms_get()-t1<2000)
	while(1)
	{
		/*while(CommandProcess(CommandParsersMPUTest,CommandParsersMPUTestNum));		
		if(CommandShouldQuit())
			break;*/
		if( fgetc(file_pri) != -1)
			break;
	
		// Get the data if available
		if(mpu_data_level())
		{
			unsigned long tt1,tt2;
			tt1 = timer_us_get();
			/*ax = (float)mpu_data_ax[mpu_data_rdptr]/16384.;
			ay = (float)mpu_data_ay[mpu_data_rdptr]/16384.;
			az = (float)mpu_data_az[mpu_data_rdptr]/16384.;*/
			ax = (float)mpu_data_ax[mpu_data_rdptr]*atog;
			ay = (float)mpu_data_ay[mpu_data_rdptr]*atog;
			az = (float)mpu_data_az[mpu_data_rdptr]*atog;
			gx = (float)mpu_data_gx[mpu_data_rdptr]*gtor;
			gy = (float)mpu_data_gy[mpu_data_rdptr]*gtor;
			gz = (float)mpu_data_gz[mpu_data_rdptr]*gtor;
			
			mx = (float)mpu_data_mx[mpu_data_rdptr];
			my = (float)mpu_data_my[mpu_data_rdptr];
			mz = (float)mpu_data_mz[mpu_data_rdptr];
			//mx=my=mz=0;
			
			if(!(mode&0b100))
			{
				ax=ay=az=0;
			}
			if(!(mode&0b010))
			{
				gx=gy=gz=0;
			}
			if(!(mode&0b001))
			{
				mx=my=mz=0;
			}
		
			// Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of the magnetometer;
			// the magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
			
			MadgwickAHRSupdate(gx,gy,gz,ax,ay,az,-my,-mx,mz);		// seems to work
			
			
			tt2 = timer_us_get();
			//printf("%ld %f %f %f %f\n",tt2-tt1,q0,q1,q2,q3);
			//printf("%f %f %f %f\n",q0,q1,q2,q3);
			printf("%f %f %f %f %ld\n",q0,q1,q2,q3,tt2-tt1);
			
			mpu_data_rdnext();
		}
		
		

	}
	mpu_config_motionmode(MPU_MODE_OFF,0);
	#endif
	return 0;
}
#endif
#if FIXEDPOINTFILTER==1
unsigned char CommandParserMPUTest_Quaternion(unsigned char *buffer,unsigned char size)
{
	#if ENABLEQUATERNION==1
	//unsigned long t1,t2;
	FIXEDPOINTTYPE ax,ay,az,gx,gy,gz,mx,my,mz;
	mx=my=mz=0;
	
	unsigned char rv;
	int mode;
	
	rv = ParseCommaGetInt(buffer,1,&mode);
	if(rv)
	{
		mode=0x07;
	}
	mode=mode&0x07;
	
	
	/*ax=1.1;
	ay=0.8;
	az=-.1;
	gx=.2;
	gy=.9;
	gz=-2;
	mx=2;
	my=-3;
	mz=4;
	
	t1 = timer_ms_get();
	for(unsigned i=0;i<1000;i++)
	{
		MadgwickAHRSupdate(gx,gy,gz,ax,ay,az,mx,my,mz);
	}
	t2 = timer_ms_get();
	printf("Time for 1000: %ld ms\n",t2-t1);*/
	
	// Enable sampling
	mpu_config_motionmode(MPU_MODE_100HZ_ACC_BW41_GYRO_BW41_MAG_100,1);
	printf("Starting loop\n");
	
	FIXEDPOINTTYPE gtor=3.14159665k/180.0k/131.0k;
	FIXEDPOINTTYPE atog=1.0k/16384.0k;
    //FIXEDPOINTTYPE atog=1.0k/4.0k;
	
	q0 = 0.999999k;
	q1 = 0.0k;
	q2 = 0.0k;
	q3 = 0.0k;
	
	//t1 = timer_ms_get();
	//while(timer_ms_get()-t1<2000)
	while(1)
	{
		/*while(CommandProcess(CommandParsersMPUTest,CommandParsersMPUTestNum));		
		if(CommandShouldQuit())
			break;*/
		if( fgetc(file_pri) != -1)
			break;
	
		// Get the data if available
		if(mpu_data_level())
		{
			unsigned long tt1,tt2;
			tt1 = timer_us_get();
			ax = mpu_data_ax[mpu_data_rdptr]*atog;
			ay = mpu_data_ay[mpu_data_rdptr]*atog;
			az = mpu_data_az[mpu_data_rdptr]*atog;
			/*ax = mpu_data_ax[mpu_data_rdptr];
			ay = mpu_data_ay[mpu_data_rdptr];
			az = mpu_data_az[mpu_data_rdptr];*/
			gx = mpu_data_gx[mpu_data_rdptr]*gtor;
			gy = mpu_data_gy[mpu_data_rdptr]*gtor;
			gz = mpu_data_gz[mpu_data_rdptr]*gtor;			
			/*mx = mpu_data_mx[mpu_data_rdptr];
			my = mpu_data_my[mpu_data_rdptr];
			mz = mpu_data_mz[mpu_data_rdptr];*/
			//mx=my=mz=0;
			
			if(!(mode&0b100))
			{
				ax=ay=az=0.0k;
			}
			if(!(mode&0b010))
			{
				gx=gy=gz=0.0k;
			}
			if(!(mode&0b001))
			{
				mx=my=mz=0.0k;
			}
		
			// Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of the magnetometer;
			// the magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
			//MadgwickAHRSupdate(gx,gy,gz,ax,ay,az,mx,my,mz);
			//MadgwickAHRSupdate(gx,gy,gz,ax,ay,az,my,mx,mz);
			//MadgwickAHRSupdate(gx,gy,gz,ax,ay,az,my,mx,-mz);
			//MadgwickAHRSupdate(gx,gy,gz,ax,ay,az,-my,-mx,mz);		// seems to work
			//MadgwickAHRSupdate(gx,gy,gz,ax,ay,az,-my,-mx,mz);		// seems to work
			MadgwickAHRSupdate(gx,gy,gz,ax,ay,az,	-mpu_data_my[mpu_data_rdptr],
													-mpu_data_mx[mpu_data_rdptr],
													mpu_data_mz[mpu_data_rdptr]);
			
			
			
			
			tt2 = timer_us_get();
			//printf("%ld %f %f %f %f\n",tt2-tt1,q0,q1,q2,q3);
			//printf("%f %f %f %f\n",q0,q1,q2,q3);
			printf("%f %f %f %f %ld\n",(float)q0,(float)q1,(float)q2,(float)q3,tt2-tt1);
			
			mpu_data_rdnext();
		}
		
		

	}
	mpu_config_motionmode(MPU_MODE_OFF,0);
	
	#endif
	return 0;
}
#endif

unsigned char CommandParserMPUTest_MagneticCalib(unsigned char *buffer,unsigned char size)
{
	unsigned char rv;
	int mode;
	
	rv = ParseCommaGetInt((char*)buffer,1,&mode);
	if(rv==0)
	{
		// parameters
		if(mode<0 || mode>2)
			return 2;
		mpu_mag_correctionmode(mode);
		return 0;
	}


	// Activate a magnetic mode
	mpu_config_motionmode(MPU_MODE_100HZ_ACC_BW41_GYRO_BW41_MAG_100,1);

	mpu_mag_calibrate();

	/*t1=timer_ms_get();
	//while(timer_ms_get()-t1<10000)
	while(1)
	{
		if( fgetc(file_pri) != -1)
			break;
		time = timer_waitperiod_ms(33,&p);
		
		m[0] = mpu_data_mx[mpu_data_rdptr];
		m[1] = mpu_data_my[mpu_data_rdptr];
		m[2] = mpu_data_mz[mpu_data_rdptr];
		mpu_data_rdnext();
		
		signed short m1[3],m2[3];
		mpu_mag_correct1(m[0],m[1],m[2],m1[0],m1[1],m1[2]);
		mpu_mag_correct2(m[0],m[1],m[2],m2[0],m2[1],m2[2]);
		
		printf("%d %d %d   %d %d %d   %d %d %d\n",m[0],m[1],m[2],m1[0],m2[0],m3[0],m2[0],m2[1],m2[2]);
	}
	*/
	
	// Activate a magnetic mode
	//mpu_config_motionmode(MPU_MODE_OFF,0);
	
	return 0;
}
unsigned char CommandParserMPUTest_MagneticSelfTest(unsigned char *buffer,unsigned char size)
{
	unsigned char mag[7];
	signed short mx,my,mz;
	
	

	// Activate a magnetic mode
	mpu_config_motionmode(MPU_MODE_100HZ_ACC_BW41_GYRO_BW41_MAG_100,0);
	
	// REMOVE SHADOWING
	_mpu_mag_regshadow(0,0,0,0);			// Stop shadowing

	printf("Start self-test\n");
	// Self-test procedure	
	// (1) Set Power-down mode. (MODE[3:0]=“0000”)
	printf("Power down\n");
	mpu_mag_writereg(0x0a,0b00010000);		// Power down, 16 bit
	
	// extra read sense
	printf("Read sense\n");
	for(unsigned i=0;i<7;i++)
		mag[i]=mpu_mag_readreg(0x03+i);
	
	// (2) Write “1” to SELF bit of ASTC register (other bits in this register should be kept “0”)
	printf("SELF=1\n");
	mpu_mag_writereg(0x0c,0b01000000);		// SELF=1
	// (3) Set Self-test Mode. (MODE[3:0]=“1000”)
	printf("Self-test mode\n");
	mpu_mag_writereg(0x0a,0b00011000);		// Self-test, 16-bits
	// (4) Check Data Ready or not by polling DRDY bit of ST1 register
	printf("Wait DRDY\n");
	//_delay_ms(1000);
	while(!(mpu_mag_readreg(0x02)&1));
	printf("ST1: %02x\n",mpu_mag_readreg(0x02));
	// (5) Read measurement data (HXL to HZH)
	printf("Read sense\n");
	for(unsigned i=0;i<7;i++)
		mag[i]=mpu_mag_readreg(0x03+i);
	// (6) Write “0” to SELF bit of ASTC register
	printf("SELF=0\n");
	mpu_mag_writereg(0x0c,0b00000000);		// SELF=0
	// (7) Set Power-down mode. (MODE[3:0]=“0000”)
	printf("Power down\n");
	mpu_mag_writereg(0x0a,0b00010000);		// Power down, 16 bit
	
	mx=mag[1]; mx<<=8; mx|=mag[0];
	my=mag[3]; my<<=8; my|=mag[2];
	mz=mag[5]; mz<<=8; mz|=mag[4];
	
	printf("Mag self-test: %d %d %d\n",mx,my,mz);
	printf("Mag asa: %d %d %d\n",_mpu_mag_asa[0],_mpu_mag_asa[1],_mpu_mag_asa[2]);
	signed short mx2,my2,mz2;
	mpu_mag_correct1(mx,my,mz,&mx2,&my2,&mz2);
	printf("Mag self-test adj: %d %d %d\n",mx2,my2,mz2);
	
	

	
	// Activate a magnetic mode
	//mpu_config_motionmode(MPU_MODE_OFF,0);
	
	return 0;
}

unsigned long rand32(void)
{
	unsigned long r;
	r=rand();
	r<<=16;
	r|=rand();
	return r;
}
void tff(void)
{
	/*volatile _Accum fx1, fx2 = 2.33K, fx3 = 0.66K;
	volatile float fl1, fl2 = 2.33, fl3 = 0.66;

	fx1 = fx2 + fx3;
	fl1 = fl2 + fl3;
	
	fx1 = fx2 - fx3;
	fl1 = fl2 - fl3;

	fx1 = fx2 * fx3;
	fl1 = fl2 * fl3;

	fx3 = fx2 / fx3;
	fl3 = fl2 / fl3;
	
	printf("%ld %ld %ld\n",fx1,fx2,fx3);
	printf("%f %f %f\n",fl1,fl2,fl3);
	
	_Accum fx4;
	float fl4;
	fx4 = sqrt(fx3);
	fl4 = sqrt(fl3);
	
	printf("sqrt %ld\n",fx4);
	printf("sqrt %f\n",fl4);
	
	_Accum s0,s1;
	float t0,t1;
	*/
	
	/*_Accum s;
	int c1=0,c2=0;
	unsigned long long t1,t2;
	_delay_ms(100);
	t1=timer_us_get();
	for(unsigned i=0;i<10;i++)
	{
		int r = rand();
				
		s = r;
		//if(s==0)
		if(s==0.0k)
			c1++;
		if(r==0)
			c2++;
		
	}
	t2=timer_us_get();
	printf("c1: %d c2: %d\n",c1,c2);
	printf("dt: %lu\n",t2-t1);*/
	
	/*_Fract x=.9r;
	//_Accum x=.9k;
	_Accum y=2.0k*x;
	
	printf("y: %f\n",(float)y);*/
	
	/*srand(0);
	unsigned t=0;
	for(unsigned i=0;i<100;i++)
	{
		unsigned r = rand();
		t += sqrt(r);
	}*/
	
	/*for(unsigned i=0;i<130;i++)
	{
		unsigned long r1,r2,r4,r5;
		_Accum ai=i;
		_Accum r3;
		
		r1 = _FP_SquareRootX(i*32768l);
		r2 = sqrtF2F(i*32768l);
		r3 = sqrtF2Fa(ai);
		r4 = fisqrt(i);
		r5=root(i);
		
		printf("%d(%ld) %f: %ld %ld %ld %ld %ld\n",i,ai,sqrt(i),r1,r2,r3,r4,r5);
		
	}*/
	
	
	srand(0);
	unsigned long rands[256];
	float randf[256];
	unsigned int n=256;
	//unsigned long rands[]={0,1,2,3,32,256,1024,8192,32768,65536,131072,262144,16777216,1073741824,2100000000,2147483646,2147483647,3000000000l,4000000000l};
	
	for(unsigned i=0;i<n;i++)
	{
		
		rands[i]=rand32();				// Create a 32-bit number
		if(i<128)
			rands[i] /= 256;			// Convert to a .15 number between 0 and 256
		else
			rands[i] /= 65536;			// Convert to a .15 number between 0 and 1
		if(rands[i]==0) rands[i]++;
		randf[i] = rands[i]/32768.0;	// equivalent in float
	}
	/*for(unsigned i=0;i<n;i++)
		printf("%ld ",rands[i]);
	printf("\n");
	for(unsigned i=0;i<n;i++)
		printf("%f ",randf[i]);
	printf("\n");*/
	
	
	
	/*for(unsigned i=0;i<n;i++)
	{
		_Accum r1,r2,r3,r4;
		_Accum r=*(_Accum*)&rands[i];
		if(i<25)
			r*=10k;
		if(i<50)
			r*=10k;
		if(i<75)
			r*=10k;

		
		r1 = invSqrt(r);
		r2 = invSqrt2(r);
		r3 = invSqrt3(r);
		int32_t tmp = fixrsqrt15(rands[i]);
		r4 = *(_Accum*)&tmp;
		
		printf("%lu (%f): %f -> %lu (%f) %lu (%f) %lu (%f) %lu (%f) [%lu]\n",r,(float)r,1.0/sqrt(r),r1,(float)r1,r2,(float)r2,r3,(float)r3,r4,(float)r4,tmp);
		
	}
	
	for(int i=0;i<sizeof(rands)/sizeof(unsigned long);i++)
	{
		printf("in: %lu out: %lu\n",rands[i],fixrsqrt15(rands[i]));
	}*/
	
	
	// To benchmark
	// Invert square root with quake in float v.s. 1.0k/
	//uint64_t t;
	unsigned long t;
	float tf;
	_Accum tk;
	unsigned long t1,t2;
	t=0; tf=0;
	t1=timer_us_get();
	for(unsigned i=0;i<n;i++)
	{
		float rf = randf[i];
		tf += sqrt(rf);
	}
	t2=timer_us_get();
	printf("sqrt: %ld (%f)\n",t2-t1,tf);
	/*t=0; tf=0;
	t1=timer_us_get();
	for(unsigned i=0;i<n;i++)
	{
		unsigned long r = rands[i];
		//t += _FP_SquareRootX(r*32768l);
		t += _FP_SquareRootX(r);
	}
	t2=timer_us_get();
	printf("_FP_SquareRootX: %lu (%ld)\n",t2-t1,t);*/
	/*t=0; tf=0;
	t1=timer_us_get();
	for(unsigned i=0;i<n;i++)
	{
		unsigned long r = rands[i];
		t += sqrtF2F(r);
	}
	t2=timer_us_get();
	printf("sqrtF2F: %ld (%lu)\n",t2-t1,t);*/
	/*t=0;
	t1=timer_us_get();
	for(unsigned i=0;i<n;i++)
	{
		unsigned long r = rands[i];
		t += fisqrt(r);
	}
	t2=timer_us_get();
	printf("fisqrt: %ld (%ld)\n",t2-t1,t);*/
	/*t=0;
	t1=timer_us_get();
	for(unsigned i=0;i<n;i++)
	{
		unsigned long r = rands[i];
		t += fixsqrt16(r);
	}
	t2=timer_us_get();
	printf("fixsqrt16: %ld (%ld)\n",t2-t1,t);*/
	/*t=0;
	t1=timer_us_get();
	for(unsigned i=0;i<100;i++)
	{
		unsigned long r = rands[i];
		t += sqrtF2Fa(r*32768l);
	}
	t2=timer_us_get();
	printf("sqrtF2Fa: %ld (%ld)\n",t2-t1,t);
	t=0;
	t1=timer_us_get();
	for(unsigned i=0;i<100;i++)
	{
		unsigned long r = rand();
		t += root(r);
	}
	t2=timer_us_get();
	printf("root: %ld (%ld)\n",t2-t1,t);*/
	
	printf("\n\n");
	
	t=0; tf=0;
	t1=timer_us_get();
	for(unsigned i=0;i<n;i++)
	{
		float r = randf[i];
		tf += invSqrtflt_ref(r);
	}
	t2=timer_us_get();
	printf("invSqrtflt_ref: %lu (%f)\n",t2-t1,tf);
	t=0; tf=0;
	t1=timer_us_get();
	for(unsigned i=0;i<n;i++)
	{
		float r = randf[i];
		tf += invSqrtflt(r);
	}
	t2=timer_us_get();
	printf("invSqrtflt: %lu (%f)\n",t2-t1,tf);
	t=0; tf=0;
	t1=timer_us_get();
	for(unsigned i=0;i<n;i++)
	{
		unsigned long r = rands[i];
		_Accum x = invSqrt3(*(_Accum *)&r);
		t += *(unsigned long*)&x;
	}
	t2=timer_us_get();
	printf("invSqrt3: %lu (%f)\n",t2-t1,t/32768.0);
	/*t=0; tf=0;
	t1=timer_us_get();
	for(unsigned i=0;i<n;i++)
	{
		unsigned long r = rands[i];
		t+=fixrsqrt15(r);
	}
	t2=timer_us_get();
	printf("fixrsqrt15: %lu (%f)\n",t2-t1,t/32768.0);*/
	/*t=0; tf=0;
	t1=timer_us_get();
	for(unsigned i=0;i<n;i++)
	{
		unsigned long r = rands[i];
		t+=fixrsqrt15b(r);
	}
	t2=timer_us_get();
	printf("fixrsqrt15b: %lu (%f)\n",t2-t1,t/32768.0);*/
	/*t=0; tf=0;
	t1=timer_us_get();
	for(unsigned i=0;i<n;i++)
	{
		unsigned long r = rands[i];
		t+=invSqrt4(r);
	}
	t2=timer_us_get();
	printf("invSqrt4: %lu (%f)\n",t2-t1,t);*/
	t=0; tf=0;
	t1=timer_us_get();
	for(unsigned i=0;i<n;i++)
	{
		unsigned long r = rands[i];
		_Accum x = invSqrt2(*(_Accum *)&r);
		t += *(unsigned long*)&x;
	}
	t2=timer_us_get();
	printf("invSqrt2: %lu (%f)\n",t2-t1,t/32768.0);
	/*t=0; tf=0;
	t1=timer_us_get();
	for(unsigned i=0;i<n;i++)
	{
		unsigned long r = rands[i];
		_Accum x = fixrsqrt15a(*(_Accum *)&r);
		t += *(unsigned long*)&x;
	}
	t2=timer_us_get();
	printf("fixrsqrt15a: %lu (%f)\n",t2-t1,t/32768.0);*/
	/*t=0; tf=0;
	t1=timer_us_get();
	for(unsigned i=0;i<n;i++)
	{
		unsigned long r1 = rands[i];
		unsigned long r2 = rands[i+1];
		t+=fixmul16(r1,r2);
	}
	t2=timer_us_get();
	printf("fixmul16: %lu (%lu)\n",t2-t1,t);*/
	t=0; tf=0; tk=0;
	t1=timer_us_get();
	for(unsigned i=0;i<n;i++)
	{
		_Accum r1 = *(_Accum*)&rands[i];
		_Accum r2 = *(_Accum*)&rands[i+1];
		tk+=r1*r2;
	}
	t2=timer_us_get();
	printf("accummul: %lu (%lu)\n",t2-t1,tk);
	/*t=0;
	*/
	
	/*_Accum a;
	_Fract f;
	a=1.1k;
	f=a;
	printf("f: %lu %f a: %u %f\n",a,(float)a,f,(float)f);
	a=1.0k;
	f=a;
	printf("f: %lu %f a: %u %f\n",a,(float)a,f,(float)f);
	a=.999k;
	f=a;
	printf("f: %lu %f a: %u %f\n",a,(float)a,f,(float)f);
	a=.99k;
	f=a;
	printf("f: %lu %f a: %u %f\n",a,(float)a,f,(float)f);
	a=.98k;
	f=a;
	printf("f: %lu %f a: %u %f\n",a,(float)a,f,(float)f);
	a=.9k;
	f=a;
	printf("f: %lu %f a: %u %f\n",a,(float)a,f,(float)f);
	*/
	//printf("__builtin_clz %d\n",__builtin_clz(1));			// 15
	//printf("__builtin_clz %d\n",__builtin_clz(2));			// 14
	//printf("__builtin_clz %d\n",__builtin_clz(4));			// 13
	printf("__builtin_clz %d clzl %d\n",__builtin_clz(8),__builtin_clzl(8));
	printf("__builtin_clz %d clzl %d\n",__builtin_clz(32768),__builtin_clzl(32768));
	//printf("__builtin_clz %d\n",__builtin_clz(65536));
	
	//printf("%lu %lu %lu\n",fisqrt(32768),fisqrt(65536),fisqrt(16777216));
	
}
void test_rsqrt(void)
{
/*	 unsigned p=65536;
	 
	for(float i=-16;i<=15;i+=.1)
	{
		float r;
		r=pow(2.0,i);
			
		unsigned long r15,r16;
		
		r15 = r*32768.0;
		r16 = r*65536.0;
			
		float gt = 1.0/sqrt(r);
		unsigned long rr15 = fixrsqrt15(r15);
		unsigned long rr16 = fixrsqrt16(r16);
		float rr15f = rr15/32768.0;
		float rr16f = rr16/65536.0;
		
		printf("%f %f %f  %lu %lu %f  %lu %lu %f \n",i,r,gt,r15,rr15,rr15f,r16,rr16,rr16f);		
	}*/
}
/*void bench_math(void)
{
	srand(0);
	unsigned long tot,t1,t2;
	unsigned long rands[256];
	//int24_t tot24;
	__uint24 tot24;
	__uint24 rands24[256];
	
	srand(0);
	for(unsigned i=0;i<256;i++)
	{
		rands[i] = rand();
		rands24[i] = rands[i];
	}
	
	for(int i=0;i<10;i++)
	{
		tot=1;
		t1 = timer_us_get();
		for(unsigned i=0;i<256;i++)
		{
			tot=tot+tot*rands[i];
		}
		t2 = timer_us_get();
		printf("tot32: %lu. time: %lu\n",tot,t2-t1);
	}
	for(int i=0;i<10;i++)
	{
		tot24=1;
		t1 = timer_us_get();
		for(unsigned i=0;i<256;i++)
		{
			tot24=tot24+tot24*rands24[i];
		}
		t2 = timer_us_get();
		printf("tot24: %lu. time: %lu\n",(unsigned long)tot24,t2-t1);
	}
}
*/

unsigned char CommandParserMPUTest_BenchMath(unsigned char *buffer,unsigned char size)
{
	/*printf("bench math\n");
	int r[256];
	
	
	for(unsigned int i=0;i<256;i++)
	{
		r[i]=rand()/10;
		if(r[i]==0)
			r[i]=1;
	}
	
	unsigned long t1,t2;
	unsigned int n=256;
	int t=1;
	float tf=1;
	
	//_Fract ff = 0.1;
	
	
	t1=timer_us_get();
	for(unsigned int i=0;i<256;i++)
	{
		//printf("t bef: %d r:%d\n",t,r[i]);
		t*=r[i];
		if(t>1000)
			t=1000;
		//printf("t: %d\n",t);
		if(t==0)
			t=1;
	}
	t2=timer_us_get();
	printf("t: %d\n",t);
	printf("tint: %ld\n",t2-t1);
	
		
	t1=timer_us_get();
	for(unsigned int i=0;i<n;i++)
	{
		tf*=(float)r[i];
		if(tf>1000.0)
			tf=1000.0;
		if(tf==0)
			tf=1;
	}
	t2=timer_us_get();
	printf("t: %f\n",tf);
	printf("tfloat: %ld\n",t2-t1);
	
	
	signed short _Accum a0;		// 2 bytes
	signed _Accum a1;			// 4 bytes
	signed long _Accum a2;		// 8 bytes
	signed short _Fract b0;		// 1 bytes
	signed _Fract b1;			// 2 bytes
	signed long _Fract b2;		// 4 bytes
	
	printf("sizeof: %d %d %d  %d %d %d\n",sizeof(a0),sizeof(a1),sizeof(a2),sizeof(b0),sizeof(b1),sizeof(b2));
	*/
	//tff();
	//bench_math();
	
	return 0;
	
}

/******************************************************************************
	function: mode_mputest
*******************************************************************************
	Interactive MPU9250 test mode.
******************************************************************************/
void mode_mputest(void)
{	
	
	while(1)
	{
		while(CommandProcess(CommandParsersMPUTest,CommandParsersMPUTestNum));		
		if(CommandShouldQuit())
			break;
		_delay_ms(1);		
	}
	


	
		
	
	
}