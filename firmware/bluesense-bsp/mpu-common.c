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




void mpu_printreg(FILE *file)
{
	unsigned char v;
	
	fprintf_P(file,PSTR("MPU-9250 registers:\n"));	
	for(unsigned char i=0;i<=0x7F;i++)
	{
		fprintf_P(file,PSTR("%02X: "),i);
		v = mpu_readreg(i);
		fprintf_P(file,PSTR("%02X"),v);
		if((i+1)%8==0 || i==0x7F)
			fprintf_P(file,PSTR("\n"));
		else
			fprintf_P(file,PSTR("   "));
	}		
	
}

// This doesn't work well, as the address counter stops being incremented or is incremented weirdly from address 6D
void mpu_printreg2(FILE *file)
{
	unsigned char v[128];
	
	fprintf_P(file,PSTR("MPU-9250 registers:\n"));
	mpu_readallregs(v);
/*	r = i2c_readregs(address,0,128,v);	
	if(r!=0)
	{
		fprintf_P(file,PSTR("Error\n"));
		return;
	}*/
	for(unsigned char i=0;i<0x7F;i++)
	{
		fprintf_P(file,PSTR("%02X: "),i);
		fprintf_P(file,PSTR("%02X"),v[i]);
		if((i+1)%8==0 || i==0x7E)
			fprintf_P(file,PSTR("\n"));
		else
			fprintf_P(file,PSTR("   "));
	}		
}

void mpu_printregdesc(FILE *file)
{
//	unsigned char address=105;
	unsigned char v[128];
	unsigned short s1,s2,s3;
	
	fprintf_P(file,PSTR("MPU-9250 registers:\n"));
	mpu_readallregs(v);
	/*r = i2c_readregs(address,0,128,v);	
	if(r!=0)
	{
		printf("Error\n");
		return;
	}*/
	fprintf_P(file,PSTR("Self test gyro x,y,z:  %02X %02X %02X\n"),v[0],v[1],v[2]);
	fprintf_P(file,PSTR("Self test accel x,y,z: %02X %02X %02X\n"),v[0x0d],v[0x0e],v[0x0f]);
	s1=v[0x13]; s1<<=8; s1|=v[0x14];
	s2=v[0x15]; s2<<=8; s2|=v[0x16];
	s3=v[0x17]; s3<<=8; s3|=v[0x18];
	fprintf_P(file,PSTR("Gyro offset x,y,z: %d %d %d\n"),s1,s2,s3);
	fprintf_P(file,PSTR("SMPLRT_DIV:   %02X    CONFIG:  %02X    GYRO_CONFIG: %02X    ACCEL_CONFIG: %02X %02X\n"),v[25],v[26],v[27],v[28],v[29]);
	fprintf_P(file,PSTR("LP_ACCEL_ODR: %02X    WOM_THR: %02X    FIFO_EN:     %02X\n"),v[30],v[31],v[35]);
	fprintf_P(file,PSTR("I2C_MST_CTRL: %02X\n"),v[36]);
	fprintf_P(file,PSTR("I2C_SLV0 ADDR:%02x    REG:     %02X    CTRL:        %02X  DO: %02X\n"),v[37],v[38],v[39],v[99]);
	fprintf_P(file,PSTR("I2C_SLV1 ADDR:%02x    REG:     %02X    CTRL:        %02X  DO: %02X\n"),v[40],v[41],v[42],v[100]);
	fprintf_P(file,PSTR("I2C_SLV2 ADDR:%02x    REG:     %02X    CTRL:        %02X  DO: %02X\n"),v[43],v[44],v[45],v[101]);
	fprintf_P(file,PSTR("I2C_SLV3 ADDR:%02x    REG:     %02X    CTRL:        %02X  DO: %02X\n"),v[46],v[47],v[48],v[102]);
	fprintf_P(file,PSTR("I2C_SLV4 ADDR:%02x    REG:     %02X    CTRL:        %02X  DO: %02X DI: %02X\n"),v[49],v[50],v[52],v[51],v[53]);
	fprintf_P(file,PSTR("I2C_MST_STAT: %02X\n"),v[54]);
	fprintf_P(file,PSTR("INT_PIN(55):  %02X    INT_EN:  %02X    INT_STATUS:  %02X\n"),v[55],v[56],v[58]);
	s1=v[59]; s1<<=8; s1|=v[60];
	s2=v[61]; s2<<=8; s2|=v[62];
	s3=v[63]; s3<<=8; s3|=v[64];
	fprintf_P(file,PSTR("ACCEL: %05d %05d %05d\n"),s1,s2,s3);
	s1=v[65]; s1<<=8; s1|=v[66];
	fprintf_P(file,PSTR("TEMP:  %05d\n"),s1);
	s1=v[67]; s1<<=8; s1|=v[68];
	s2=v[69]; s2<<=8; s2|=v[70];
	s3=v[71]; s3<<=8; s3|=v[72];
	fprintf_P(file,PSTR("GYRO:  %05d %05d %05d\n"),s1,s2,s3);
	for(int i=0;i<24;i++)
	{
		fprintf_P(file,PSTR("EXT_SENS_DATA %02X: %02X"),i,v[73+i]);
		if((i+1)%4==0) fprintf_P(file,PSTR("\n")); else fprintf_P(file,PSTR("   "));
	}
	fprintf_P(file,PSTR("I2C_MST_DELAY_CTRL: %02X\n"),v[103]);
	fprintf_P(file,PSTR("SIGNAL_PATH_RESET:  %02X\n"),v[104]);
	fprintf_P(file,PSTR("MOT_DETECT_CTRL:    %02X  USER_CTRL:  %02X\n"),v[105],v[106]);
	fprintf_P(file,PSTR("PWR_MGMT_1:         %02X  PWR_MGMT_2: %02X\n"),v[107],v[108]);
	s1=v[114]; s1<<=8; s1|=v[115];
	fprintf_P(file,PSTR("FIFO_COUNT:         %d    FIFO_R_W:   %02X\n"),s1,v[116]);
	fprintf_P(file,PSTR("WHO_AM_I:           %02X\n"),v[117]);
	s1=v[119]; s1<<=7; s1|=(v[120])&0x7F;
	s2=v[122]; s2<<=7; s2|=(v[123])&0x7F;
	s3=v[125]; s3<<=7; s3|=(v[126])&0x7F;
	fprintf_P(file,PSTR("ACC_OFFSET: %05d %05d %05d\n"),s1,s2,s3);
	
}
void mpu_printregdesc2(FILE *file)
{
	unsigned char v[128];
	
	fprintf_P(file,PSTR("MPU-9250 registers:\n"));
	mpu_readallregs(v);
	fprintf_P(file,PSTR(" SMPLRT_DIV(19h):    %02X\n"),v[0x19]);
	
	unsigned char dlp_cfg = v[0x1A]&0b111;
	unsigned char gyro_fchoice = v[0x1B]&0b11;
	fprintf_P(file,PSTR(" Configuration(1Ah): %02X. FIFO mode: %s. Ext sync: %d. DLP_CFG: %dd\n"),v[0x1A],(v[0x1A]&0x40)?"No write on full":"Overwrite",(v[0x1A]>>3)&0b111,dlp_cfg);
	fprintf_P(file,PSTR(" GyroConfig(1Bh):    %02X. X ST: %d. Y ST: %d. Z ST: %d. FS: %d%db. fchoice_b: %d%db\n"),v[0x1B],v[0x1B]&0x80?1:0,v[0x1B]&0x40?1:0,v[0x1B]&0x20?1:0,(v[0x1B]>>4)&0b1,(v[0x1B]>>3)&0b1,gyro_fchoice>>1,gyro_fchoice&0b1);
	fprintf_P(file,PSTR(" AccelConfig(1Ch):   %02X. X ST: %d. Y ST: %d. Z ST: %d. FS: %d%db.\n"),v[0x1C],v[0x1C]&0x80?1:0,v[0x1C]&0x40?1:0,v[0x1C]&0x20?1:0,(v[0x1C]>>4)&0b1,(v[0x1C]>>3)&0b1);		
	fprintf_P(file,PSTR(" AccelConfig2(1Dh):  %02X. fchoice_b: %db. A_DLPCFG: %dd\n"),v[0x1D],v[0x1D]&0x8?1:0,v[0x1D]&0b111);
	fprintf_P(file,PSTR(" LPODR(1Eh):         %02X. clksel: %dd\n"),v[0x1E],v[0x1E]&0b1111);
	fprintf_P(file,PSTR(" PWR_MGMT_1(6Bh):    %02X. rst: %d sleep: %d cycle: %d gyro_stby: %d PD-PTAT: %d clksel: %dd\n"),v[0x6B],v[0x6B]&0x80?1:0,v[0x6B]&0x40?1:0,v[0x6B]&0x20?1:0,v[0x6B]&0x10?1:0,v[0x6B]&0x08?1:0,v[0x6B]&0b111);
	fprintf_P(file,PSTR(" PWR_MGMT_2(6Ch):    %02X. dis_xa: %d dis_ya: %d dis_za: %d dis_xg: %d dis_yg: %d dis_zg: %d\n"),v[0x6C],v[0x6C]&0x20?1:0,v[0x6C]&0x10?1:0,v[0x6C]&0x08?1:0,v[0x6C]&0x04?1:0,v[0x6C]&0x02?1:0,v[0x6C]&0x01);
		


	
}



/******************************************************************************
	mpu_get_agt
*******************************************************************************
	Blocking reads of the 14 registers comprising acceleration, gyroscope and temperature.
******************************************************************************/
void mpu_get_agt(signed short *ax,signed short *ay,signed short *az,signed short *gx,signed short *gy,signed short *gz,signed short *temp)
{
	unsigned char v[14];
	unsigned short s1,s2,s3;
	
	mpu_readregs_burst(v,59,14);
	
	s1=v[0]; s1<<=8; s1|=v[1];
	s2=v[2]; s2<<=8; s2|=v[3];
	s3=v[4]; s3<<=8; s3|=v[5];
	*ax = s1; *ay = s2; *az = s3;
	s1=v[6]; s1<<=8; s1|=v[7];
	*temp = s1;
	s1=v[8]; s1<<=8; s1|=v[9];
	s2=v[10]; s2<<=8; s2|=v[11];
	s3=v[12]; s3<<=8; s3|=v[13];
	*gx = s1; *gy = s2; *gz = s3;
}
/******************************************************************************
	mpu_get_g
*******************************************************************************	
	Blocking reads of the 6 registers comprising gyroscope.
	
******************************************************************************/
void mpu_get_g(signed short *gx,signed short *gy,signed short *gz)
{
	unsigned char v[6];
	unsigned short s1,s2,s3;
	
	mpu_readregs_burst(v,67,6);
	
	s1=v[0]; s1<<=8; s1|=v[1];
	s2=v[2]; s2<<=8; s2|=v[3];
	s3=v[4]; s3<<=8; s3|=v[5];
	*gx = s1; *gy = s2; *gz = s3;
}
/******************************************************************************
	mpu_get_a
*******************************************************************************	
	Blocking reads of the 6 registers comprising gyroscope.
******************************************************************************/
void mpu_get_a(signed short *ax,signed short *ay,signed short *az)
{
	unsigned char v[6];
	unsigned short s1,s2,s3;
	
	mpu_readregs_burst(v,59,6);
	
	s1=v[0]; s1<<=8; s1|=v[1];
	s2=v[2]; s2<<=8; s2|=v[3];
	s3=v[4]; s3<<=8; s3|=v[5];
	*ax = s1; *ay = s2; *az = s3;
}


/******************************************************************************
	mpu_getfifocnt
*******************************************************************************
	Returns the FIFO level
******************************************************************************/
unsigned short mpu_getfifocnt(void)
{
	return mpu_readreg16(114);
}
/******************************************************************************
	mpu_fifoenable
*******************************************************************************
	Sets the FIFO flags (register 35d)
******************************************************************************/
void mpu_fifoenable(unsigned char flags)
{
	mpu_writereg(35,flags);
}
/******************************************************************************
	mpu_readallregs
*******************************************************************************	
	Read all MPU registers		
******************************************************************************/
void mpu_readallregs(unsigned char *v)
{
	mpu_readregs(v,0,0x80);
}
/******************************************************************************
	mpu_fiforead
*******************************************************************************	
	Read n bytes from the 
******************************************************************************/
void mpu_fiforead(unsigned char *fifo,unsigned short n)
{
	for(unsigned short i=0;i<n;i++)
	{
		fifo[i] = mpu_readreg(116);
	}
}
/******************************************************************************
	mpu_setaccodr
*******************************************************************************	
	Sets low-power accelerometer output data rate (register 30d)
******************************************************************************/
void mpu_setaccodr(unsigned char odr)
{
	mpu_writereg(MPU_R_LPODR,odr);
}
/******************************************************************************
	mpu_setacccfg2
*******************************************************************************	
	Sets accelerometer configuration 2 register (register 29d)
******************************************************************************/
void mpu_setacccfg2(unsigned char cfg)
{
	mpu_writereg(MPU_R_ACCELCONFIG2,cfg);
}
/******************************************************************************
	mpu_setusrctrl
*******************************************************************************	
	Sets accelerometer user control register (register 106d)
******************************************************************************/
void mpu_setusrctrl(unsigned char cfg)
{
	mpu_writereg(106,cfg);
}
/******************************************************************************
	mpu_getwhoami
*******************************************************************************	
	Returns MPU WHOAMI register
******************************************************************************/
unsigned char mpu_getwhoami(void)
{
	return mpu_readreg(MPU_R_WHOAMI);
}
/******************************************************************************
	mpu_reset
*******************************************************************************	
	Resets the MPU
******************************************************************************/
void mpu_reset(void)
{
	mpu_writereg(MPU_R_PWR_MGMT_1,0b10000000);
}
/******************************************************************************
	mpu_setsrdiv
*******************************************************************************	
	Set sample rate divider register (register 25d).
	Sample rate = internal sample rate / (div+1)
	Only effective when fchoice=11 (fchoice_b=00) and 0<dlp_cfg<7
******************************************************************************/
void mpu_setsrdiv(unsigned char div)
{
	mpu_writereg(MPU_R_SMPLRT_DIV,div);
}
/******************************************************************************
	mpu_setgyrosamplerate
*******************************************************************************	
	Set the gyro sample rate and the digital low pass filter settings.

	fchoice:	2 bits
						x0: DLP disabled, BW=8800Hz, Fs=32KHz
						01: DLP disabled, BW=3600Hz, Fs=32KHz
						11: DLP enabled
	dlp:			3 bits
						0: BW=250Hz, Fs=8KHz
						1: BW=184Hz, FS=1KHz
						...
						7: BW=3600Hz, FS=8KHz

******************************************************************************/
void mpu_setgyrosamplerate(unsigned char fchoice,unsigned char dlp)
{
	unsigned char config,gconfig;
	
	// Sanitise inputs
	fchoice=~fchoice;			// Convert to fchoice_b for register
	fchoice&=0b11;
	dlp&=0b111;
	
	// Get register values	
	config = mpu_readreg(MPU_R_CONFIG);
	gconfig = mpu_readreg(MPU_R_GYROCONFIG);
	
	// Modify register values
	config = (config&0b11111000)+dlp;
	gconfig = (gconfig&11111100)+fchoice;
	
	// Write register values
	mpu_writereg(MPU_R_CONFIG,config);
	mpu_writereg(MPU_R_GYROCONFIG,gconfig);	
}

/******************************************************************************
	mpu_setaccsamplerate
*******************************************************************************	
	Sets accelerometer sample rate and digital low pass filter
******************************************************************************/
void mpu_setaccsamplerate(unsigned char fchoice,unsigned char dlp)
{
	unsigned char aconfig;
	
	// Sanitise inputs
	fchoice=~fchoice;			// Convert to fchoice_b for register
	fchoice&=0b1;
	dlp&=0b111;
	
	// Get register values	
	aconfig = mpu_readreg(MPU_R_ACCELCONFIG2);
	
	// Modify register values
	aconfig = (aconfig&0b11110000)+(fchoice<<3)+dlp;
	
	// Write register values
	mpu_writereg(MPU_R_ACCELCONFIG2,aconfig);
}


/******************************************************************************
	mpu_set_interrutenable
*******************************************************************************
	Enable the following interrupts:
	wom:     wake on motion
	fifo:    FIFO overflow
	fsync:   fsync interrupt
	datardy: new data acquired
******************************************************************************/
void mpu_set_interrutenable(unsigned char wom,unsigned char fifo,unsigned char fsync,unsigned char datardy)
{
	unsigned char v;
	v = (wom<<6)|(fifo<<4)|(fsync<<3)|datardy;
	mpu_writereg(MPU_R_INTERRUPTENABLE,v);	
}




/******************************************************************************
	mpu_mode_lpacc
*******************************************************************************
	Set the MPU9250 in low-power accelerometer-only mode.
	- Gyroscope: off
	- Temperature: off
	- Accelerometer: duty-cycled
******************************************************************************/
void mpu_mode_lpacc(unsigned char lpodr)
{
	printf_P(PSTR("mpu_mode_lpacc: %d\n"),lpodr);
	
	// Change to the accelerometer ODR require the accelerometer to be first on.
	mpu_writereg(MPU_R_PWR_MGMT_1,0b00000000);		// Make sure accel running
	mpu_writereg(MPU_R_PWR_MGMT_2,0b00000111);		// Activate accel, deactivate gyro
	// TODO: CHECK whether fchoice and DLP influence LP operation
	mpu_writereg(MPU_R_ACCELCONFIG2,0b00001000);	// fchoice_b=1: DLP disabled; LP=x
	mpu_writereg(MPU_R_LPODR,lpodr);							// LP ODR
	mpu_writereg(MPU_R_PWR_MGMT_1,0b00101000);		// Enable cycle mode (LP), deactivate temp
}
/******************************************************************************
	mpu_mode_acc
*******************************************************************************
	Set the MPU9250 in normal accelerometer-only mode.
	- Gyroscope: off
	- Temperature: off
	- Accelerometer: normal
	
	dlpenable: if enabled, dlpbw specifies the bandwidth of the digital low-pass filter
	dlpbw:     bandwidth of the DLP filter (MPU_ACC_LPF_460... MPU_ACC_LPF_5)
	divider:   divide the output of the DLP filter block by 1/(1+divider)
	
	The output data rate is 4KHz when dlpenable is deactivated; 1KHz/(1+divider) when dlp is enabled
******************************************************************************/
void mpu_mode_acc(unsigned char dlpenable,unsigned char dlpbw,unsigned char divider)
{	
	printf_P(PSTR("mpu_mode_acc: %d %d %d\n"),dlpenable,dlpbw,divider);
	
	// Sanitise
	dlpenable=dlpenable?0:1;		// Convert to fchoice_b
	if(dlpbw>MPU_ACC_LPF_5) dlpbw=MPU_ACC_LPF_5;
	
	mpu_writereg(MPU_R_PWR_MGMT_1,0b00000000);					// Make sure accel running
	mpu_writereg(MPU_R_PWR_MGMT_2,0b00000111);					// Activate accel, deactivate gyro
	mpu_writereg(MPU_R_ACCELCONFIG2,(dlpenable<<3)|dlpbw);
	mpu_setsrdiv(divider);
	mpu_writereg(MPU_R_PWR_MGMT_1,0b00001000);		// Disable cycle mode (LP), deactivate temp
	//mpu_writereg(MPU_R_PWR_MGMT_1,0b00000000);		// Disable cycle mode (LP), activate temp
}
/******************************************************************************
	mpu_mode_gyro
*******************************************************************************
	Set the MPU9250 in normal gyroscope-only mode
	- Gyroscope: on
	- Temperature: off
	- Accelerometer: off
	
	gdlpe:			1 to enable gyroscope DLP.
							if enabled, gdlpbw specifies the bandwidth.
	gdlpoffhbw: when DLP off, 1 to set high bandwidth (8800Hz) or 0 to set low bandwidth (3600Hz)
	gdlpbw:     when DLP on, set the DLP low pass filter. 
              Possible values: MPU_GYR_LPF_250, 184, 92, 41, 20, 10, 5, 3600
	divider:    divide the output of the DLP filter block by 1/(1+divider)
	
	The output data rate is as follows:
	           32KHz when gdlpe=0
	           8KHz when gdlpe=1 and gdlpbw=MPU_GYR_LPF_250 or MPU_GYR_LPF_3600
	           1KHz/(1+divider) when gdlp=1 and gdlpbw<=MPU_GYR_LPF_184<=MPU_GYR_LPF_5
******************************************************************************/
void mpu_mode_gyro(unsigned char gdlpe,unsigned char gdlpoffhbw,unsigned char gdlpbw,unsigned char divider)
{
	unsigned char conf,gconf;
	unsigned char gfchoice_b;
	
	printf_P(PSTR("mpu_mode_gyro: %d %d %d %d\n"),gdlpe,gdlpoffhbw,gdlpbw,divider);
	
	// Sanitise
	gdlpe=gdlpe?1:0;
	gdlpoffhbw=gdlpoffhbw?1:0;
	if(gdlpbw>MPU_GYR_LPF_3600) gdlpbw=MPU_GYR_LPF_3600;
	
	// Gyro fchoice
	if(gdlpe)
		gfchoice_b=0b00;
	else
		if(gdlpoffhbw)
			gfchoice_b=0b01;
		else
			gfchoice_b=0b10;
	
	mpu_writereg(MPU_R_PWR_MGMT_1,0b00001001);							// Enable PLL, disable temp, gyro sense
	//i2c_writereg(MPU_ADDRESS,MPU_R_PWR_MGMT_1,0b00000000);						// Internal osc, temp, gyro sense
	mpu_writereg(MPU_R_PWR_MGMT_2,0b00111000);							// Activate gyro
	conf = mpu_readreg(MPU_R_CONFIG);									
	gconf = mpu_readreg(MPU_R_GYROCONFIG);	
	mpu_writereg(MPU_R_CONFIG,(conf&0b11111000)|gdlpbw);		// Gyro lp filter
	mpu_writereg(MPU_R_GYROCONFIG,(gconf&0b11111100)|gfchoice_b);	// Gyro dlp
	
	
	mpu_setsrdiv(divider);
	
}
/******************************************************************************
	mpu_mode_accgyro
*******************************************************************************
	Set the MPU9250 in normal gyroscope and accelerometer mode
	- Gyroscope: on
	- Temperature: off
	- Accelerometer: normal
	
	gdlpe:			1 to enable gyroscope DLP.
							if enabled, gdlpbw specifies the bandwidth.
	gdlpoffhbw: when DLP off, 1 to set high bandwidth (8800Hz) or 0 to set low bandwidth (3600Hz)
	gdlpbw:     when DLP on, set the DLP low pass filter. 
              Possible values: MPU_GYR_LPF_250, 184, 92, 41, 20, 10, 5, 3600
  adlpe:      1 to enable the accelerometer DLP
	adlpbw:     bandwidth of the accelerometer DLP filter (MPU_ACC_LPF_460... MPU_ACC_LPF_5)
	divider:    divide the output of the DLP filter block by 1/(1+divider)
	
	The output data rate is as follows:
	           32KHz when gdlpe=0
	           8KHz when gdlpe=1 and gdlpbw=MPU_GYR_LPF_250 or MPU_GYR_LPF_3600
	           1KHz/(1+divider) when gdlp=1 and gdlpbw<=MPU_GYR_LPF_184<=MPU_GYR_LPF_5
******************************************************************************/
void mpu_mode_accgyro(unsigned char gdlpe,unsigned char gdlpoffhbw,unsigned char gdlpbw,unsigned char adlpe,unsigned char adlpbw,unsigned char divider)
{
	unsigned char conf,gconf;
	unsigned char gfchoice_b;
	
	printf_P(PSTR("mpu_mode_accgyro: %d %d %d %d %d %d\n"),gdlpe,gdlpoffhbw,gdlpbw,adlpe,adlpbw,divider);
	
	// Sanitise
	gdlpe=gdlpe?1:0;
	adlpe=adlpe?0:1;		// convert to fchoice
	gdlpoffhbw=gdlpoffhbw?1:0;
	if(gdlpbw>MPU_GYR_LPF_3600) gdlpbw=MPU_GYR_LPF_3600;
	if(adlpbw>MPU_ACC_LPF_5) adlpbw=MPU_ACC_LPF_5;
		
	// Gyro fchoice
	if(gdlpe)
		gfchoice_b=0b00;
	else
		if(gdlpoffhbw)
			gfchoice_b=0b01;
		else
			gfchoice_b=0b10;
	
	mpu_writereg(MPU_R_PWR_MGMT_1,0b00001001);							// Enable PLL, disable temp, gyro sense
	//mpu_writereg(MPU_R_PWR_MGMT_1,0b00000000);							// Internal osc, temp, gyro sense
	mpu_writereg(MPU_R_PWR_MGMT_2,0b00000000);							// Activate accel+gyro
	mpu_writereg(MPU_R_ACCELCONFIG2,(adlpe<<3)|adlpbw);			// Accel DLP and bandwidth
	conf = mpu_readreg(MPU_R_CONFIG);
	gconf = mpu_readreg(MPU_R_GYROCONFIG);	
	mpu_writereg(MPU_R_CONFIG,(conf&0b11111000)|gdlpbw);		// Gyro lp filter
	mpu_writereg(MPU_R_GYROCONFIG,(gconf&0b11111100)|gfchoice_b);	// Gyro dlp
	
	
	mpu_setsrdiv(divider);
	
}
/******************************************************************************
	mpu_mode_temp
*******************************************************************************
	Toggles the PD_PTAT bit in PWR_MGMT_1; i.e. enables or disables the 
	temperature conversion. 
	Does not affect any other setting.
******************************************************************************/
void mpu_mode_temp(unsigned char enable)
{
	unsigned char pwr1;
	
	pwr1 = mpu_readreg(MPU_R_PWR_MGMT_1);
	if(enable)
		pwr1&=0b11110111;
	else
		pwr1|=0b00001000;
	mpu_writereg(MPU_R_PWR_MGMT_1,pwr1);
}

/******************************************************************************
	mpu_mode_off
*******************************************************************************
	Turns of as many things to lower power consumtion.
	- Gyroscope: off
	- Temperature: off
	- Accelerometer: off
******************************************************************************/
void mpu_mode_off(void)
{
	unsigned char gconf;
	mpu_writereg(MPU_R_PWR_MGMT_1,0b00000000);							// Enable all 
	mpu_writereg(MPU_R_PWR_MGMT_2,0b00111111);							// Disable accel + gyro
	mpu_writereg(MPU_R_ACCELCONFIG2,0b00001000);						// Disable accel DLP
	gconf = mpu_readreg(MPU_R_GYROCONFIG);	
	mpu_writereg(MPU_R_GYROCONFIG,(gconf&0b11111100)|0b01);	// Disable gyro dlp
	mpu_writereg(MPU_R_PWR_MGMT_1,0b01001000);							// Sleep, no cycle, no gyro stby, disable temp, internal osc
}



/******************************************************************************
	mpu_mode_clksel
*******************************************************************************
	Changes the clksel bits in PWR_MGMT_1.
	Does not affect any other setting.
******************************************************************************/
void mpu_mode_clksel(unsigned char clk)
{
	unsigned char pwr1;
	
	if(clk>7)
		clk=7;
	pwr1 = mpu_readreg(MPU_R_PWR_MGMT_1);
	pwr1&=0b11111000;
	pwr1|=clk;
	mpu_writereg(MPU_R_PWR_MGMT_1,pwr1);
}

/******************************************************************************
	mpu_mode_gyrostby
  *******************************************************************************
	Changes the gyro_standby bit in PWR_MGMT_1.
	Does not affect any other setting.
******************************************************************************/
void mpu_mode_gyrostby(unsigned char stby)
{
	unsigned char pwr1;
	
	pwr1 = mpu_readreg(MPU_R_PWR_MGMT_1);
	if(stby)
		pwr1|=0b00010000;
	else
		pwr1&=0b11101111;
	mpu_writereg(MPU_R_PWR_MGMT_1,pwr1);
}
/******************************************************************************
	mpu_mode_sleep
*******************************************************************************
	Changes the sleep bit in PWR_MGMT_1.
	Does not affect any other setting.
******************************************************************************/
void mpu_mode_sleep(unsigned char sleep)
{
	unsigned char pwr1;
	
	pwr1 = mpu_readreg(MPU_R_PWR_MGMT_1);
	if(sleep)
		pwr1|=0b01000000;
	else
		pwr1&=0b10111111;
	mpu_writereg(MPU_R_PWR_MGMT_1,pwr1);
}

signed short mpu_convtemp(signed short t)
{
	// Return the temperature in *100;
	signed long int t2;
	signed long int ct;
	
	//ct = (t-0)/333.87 + 21;
	
	t2 = t;
	t2 *=10000l;
	ct = t2/33387 + 2100;
	//ct /= 100;
	
	return (signed short)ct;
}

void mpu_set_interruptpin(unsigned char p)
{
	mpu_writereg(MPU_R_INTERRUPTPIN,p);	
}


unsigned char _mpu_agt_data[16];
MPU_READ_CALLBACK7 _mpu_cb7;
MPU_READ_CALLBACK3 _mpu_cb3;
unsigned char _mpu_agt_ongoing=0;
/******************************************************************************
	_mpu_get_agt_int_cb_cb7
*******************************************************************************	
	Internal callback converting 7 registers 59-72 for user callback
******************************************************************************/
void _mpu_get_agt_int_cb_cb7(void)
{
	// Conversion
	unsigned short ax,ay,az,gx,gy,gz,temp;
	
	
	ax=_mpu_agt_data[0]; ax<<=8; ax|=_mpu_agt_data[1];
	ay=_mpu_agt_data[2]; ay<<=8; ay|=_mpu_agt_data[3];
	az=_mpu_agt_data[4]; az<<=8; az|=_mpu_agt_data[5];
	temp=_mpu_agt_data[6]; temp<<=8; temp|=_mpu_agt_data[7];
	gx=_mpu_agt_data[8]; gx<<=8; gx|=_mpu_agt_data[9];
	gy=_mpu_agt_data[10]; gy<<=8; gy|=_mpu_agt_data[11];
	gz=_mpu_agt_data[12]; gz<<=8; gz|=_mpu_agt_data[13];

	_mpu_cb7(0,0,ax,ay,az,gx,gy,gz,temp);
	
	_mpu_agt_ongoing=0;
}
/******************************************************************************
	_mpu_get_agt_int_cb_cb3
*******************************************************************************	
	Internal callback converting 3 registers (59-64 or 67-72) for user callback
******************************************************************************/
void _mpu_get_agt_int_cb_cb3(void)
{
	// Conversion
	unsigned short x,y,z;
	
	
	x=_mpu_agt_data[0]; x<<=8; x|=_mpu_agt_data[1];
	y=_mpu_agt_data[2]; y<<=8; y|=_mpu_agt_data[3];
	z=_mpu_agt_data[4]; z<<=8; z|=_mpu_agt_data[5];


	_mpu_cb3(0,0,x,y,z);
	
	_mpu_agt_ongoing=0;
}
/******************************************************************************
	mpu_get_agt_int_cb
*******************************************************************************	
	Interrupt-driven read, calling a callback on success or error
	
	Call mpu_get_agt_int_init for initialisation once prior to this.
	
	Only one call can be queued at a time - subsequent return error until transaction
	is completed.
		
	Return value:
		0:	Success
		1:	Error issuing transactions (no space available)
******************************************************************************/
unsigned char mpu_get_agt_int_cb(MPU_READ_CALLBACK7 cb)
{
	unsigned char r;

	if(_mpu_agt_ongoing)
		return 1;

	_mpu_agt_ongoing=1;
	_mpu_cb7=cb;
	r = mpu_readregs_burst_cb(_mpu_agt_data,59,14,_mpu_get_agt_int_cb_cb7);
	if(r)
	{
		_mpu_agt_ongoing=0;
		return 1;
	}

	return 0;
}
/******************************************************************************
	mpu_get_a_int_cb
*******************************************************************************	
	Interrupt-driven read, calling a callback on success or error
	
	Call mpu_get_agt_int_init for initialisation once prior to this.
	
	Only one call can be queued at a time - subsequent return error until transaction
	is completed.
		
	Return value:
		0:	Success
		1:	Error issuing transactions (no space available)
******************************************************************************/
unsigned char mpu_get_a_int_cb(MPU_READ_CALLBACK3 cb)
{
	unsigned char r;

	if(_mpu_agt_ongoing)
		return 1;

	_mpu_agt_ongoing=1;
	_mpu_cb3=cb;
	r = mpu_readregs_burst_cb(_mpu_agt_data,59,6,_mpu_get_agt_int_cb_cb3);
	if(r)
	{
		_mpu_agt_ongoing=0;
		return 1;
	}

	return 0;
}


/******************************************************************************
	mpu_get_g_int_cb
*******************************************************************************	
	Interrupt-driven read, calling a callback on success or error
	
	Call mpu_get_agt_int_init for initialisation once prior to this.
	
	Only one call can be queued at a time - subsequent return error until transaction
	is completed.
		
	Return value:
		0:	Success
		1:	Error issuing transactions (no space available)
******************************************************************************/
unsigned char mpu_get_g_int_cb(MPU_READ_CALLBACK3 cb)
{
	unsigned char r;

	if(_mpu_agt_ongoing)
		return 1;

	_mpu_agt_ongoing=1;
	_mpu_cb3=cb;
	r = mpu_readregs_burst_cb(_mpu_agt_data,67,6,_mpu_get_agt_int_cb_cb3);
	if(r)
	{
		_mpu_agt_ongoing=0;
		return 1;
	}

	return 0;
}
