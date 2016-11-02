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

#include "mpu.h"
#include "wait.h"
#include "main.h"
#include "motionconfig.h"
#include "system.h"

/*
	File: mpu
	
	MPU9250 functions
	
	The key functions are:
	
	* mpu_init:					initialise the mpu
	* mpu_config_motionmode:	selects the acquisition mode (which sensors are acquired and the sample rate) and whether "automatic read" into a memory buffer is performed.
	* mpu_get_a:				blocking read of acceleration when not in automatic read (can also be used in automatic read).
	* mpu_get_g:				blocking read of gyroscope when not in automatic read (can also be used in automatic read).
	* mpu_get_agt:				blocking read of acceleration, gyroscope and temperature when not in automatic read (can also be used in automatic read).
	* mpu_get_agmt:				blocking read of acceleration, acceleration, magnetic field and temperature when not in automatic read (can also be used in automatic read).
	
	
	*Automatic read*
	The recommended mode of operation is to use the "automatic read" in mpu_config_motionmode. In this mode, the MPU data is read
	at the indicated rate in background (interrupt routine) and made available in memory buffers.
	In automatic read mode, the following functions and variables are used to access the data in the buffer:
	
	* mpu_data_level:			Function indicating how many samples are in the buffer
	* mpu_data_rdnext:			Function incrementing the read pointer to access the next sample in the buffer
	* Buffers: 					mpu_data_ax[],mpu_data_ay[],mpu_data_az[], mpu_data_gx[],mpu_data_gy[],mpu_data_gz[], mpu_data_mx[],mpu_data_my[],mpu_data_mz[],mpu_data_ms[], mpu_data_temp[], mpu_data_time[]
	* Current sample:			mpu_data_rdptr
	
	For example, the acceleration x is accessed with mpu_data_ax[mpu_data_rdptr]. In order to access the next sample, call mpu_data_rdnext() and access the sample with mpu_data_ax[mpu_data_rdptr].

	
	In non automatic read, the functions mpu_get_a, mpu_get_g, mpu_get_agt or mpu_get_agmt must be used to acquire the MPU data. These functions can also be called in automatic
	read, however this is suboptimal and increases CPU load as the data would be acquired in the interrupt routine and 	through this function call.
	
	
	*Statistics*
	The following counters are available to monitor the interrupt-driven automatic read. These counters are cleared upon calling mpu_config_motionmode with a new acquisition mode:
	
	* mpu_cnt_int:				counts how often the MPU isr has been triggered
	* mpu_cnt_sample_tot:		counts how many samples occurred, independently of whether successful or not
	* mpu_cnt_sample_succcess:	counts how many samples were acquired and successfully stored in the buffer; mpu_cnt_sample_succcess-mpu_cnt_sample_tot indicates the number of errors that occurred
	* mpu_cnt_sample_errbusy:	counts how many samples were skipped because the interface was busy (e.g. from an ongoing transfer)
	* mpu_cnt_sample_errfull: 	counts how many samples were skipped because the buffer was full 
	
	
	*Units*
	The variable mpu_gtor can be used to convert the gyroscope readings to radians per second. This variable is updated when mpu_setgyroscale is called.
	It is a fixed-point (_Accum) variable. Convert the gyroscope reading to radians per second as follows: 
	
	_Accum gx_rps = mpu_data_gx[mpu_data_rdptr] * mpu_gtor;
	
	*Magnetometer*	
	Functions to handle the magnetometer - prefixed by mpu_mag_... - can only be used when the MPU is configured
	in a mode where the magnetic field sensor is turned on; otherwise the function time-out due to the magnetic field
	sensor being inaccessible via the internal I2C interface.
	
	The function mpu_isr must be called on the rising edge of the MPU interrupt pin. 
	
	
	Todo:
	V - Objective: this is the main file doing background acquisition of all the sensor data into in-memory structures
	V - Only user-facing function to setup mode is mpu_config_motionmode(xxx)
	- new function "enableall" which enables communication with magnetometer; only used when setting up the magnetometer
	V - quaternion here
	- Calibration using only FIFO
		
	
	Todo: 
	V ensure direct register rw possible if interrupt-driven transfer: wait for interrupt transfer
	V move interrupt handling in mpu.c
	V move buffer handling in interrupt.c
	V add calibration 
	V read magnetic calibration on init
	V move modes from main to mpu.c	
	V #defines for the modes, instead of numbers
	V Clarify _mpu_agt_ongoing and _spiusart0_ongoing
	- rationalise/merge mpu.c & motionconfig.c
	- document sample_mode
	- rename sample_mode
	- consider struct instead of individual arrays - faster transfer
	- move formatting for packet / strings to here?
	V benchmark
	- rename MOTIONCONFIG_NUM
	- Convert temperature to degrees
	- Longer calibration period
	
	
*/


unsigned char __mpu_sample_softdivider_ctr=0,__mpu_sample_softdivider_divider=0;

// Data buffers
volatile signed short mpu_data_ax[MPU_MOTIONBUFFERSIZE],mpu_data_ay[MPU_MOTIONBUFFERSIZE],mpu_data_az[MPU_MOTIONBUFFERSIZE],mpu_data_gx[MPU_MOTIONBUFFERSIZE],mpu_data_gy[MPU_MOTIONBUFFERSIZE],mpu_data_gz[MPU_MOTIONBUFFERSIZE],mpu_data_mx[MPU_MOTIONBUFFERSIZE],mpu_data_my[MPU_MOTIONBUFFERSIZE],mpu_data_mz[MPU_MOTIONBUFFERSIZE],mpu_data_temp[MPU_MOTIONBUFFERSIZE];
volatile unsigned long int mpu_data_time[MPU_MOTIONBUFFERSIZE];
volatile unsigned char mpu_data_ms[MPU_MOTIONBUFFERSIZE];
volatile unsigned short mpu_data_packetctr[MPU_MOTIONBUFFERSIZE];
volatile unsigned short __mpu_data_packetctr_current;
volatile unsigned char mpu_data_rdptr,mpu_data_wrptr;

// Magnetometer Axis Sensitivity Adjustment
unsigned char _mpu_mag_asa[3];
signed short _mpu_mag_calib_max[3];
signed short _mpu_mag_calib_min[3];
signed short _mpu_mag_bias[3];
signed short _mpu_mag_sens[3];
unsigned char _mpu_mag_correctionmode;

// Automatic read statistic counters
unsigned long mpu_cnt_int, mpu_cnt_sample_tot, mpu_cnt_sample_succcess, mpu_cnt_sample_errbusy, mpu_cnt_sample_errfull;

unsigned char __mpu_autoread=0;

// Motion ISR
void (*isr_motionint)(void) = 0;
void (*isr_motionint_ds)(void) = 0;

// Conversion from Gyro readings to rad/sec
_Accum mpu_gtor=3.14159665k/180.0k/131.0k;

// 
unsigned char sample_mode;

/******************************************************************************
*******************************************************************************
MPU ISR   MPU ISR   MPU ISR   MPU ISR   MPU ISR   MPU ISR   MPU ISR   MPU ISR   
*******************************************************************************
*******************************************************************************/
inline void mpu_isr(void)
{
	// motionint always called (e.g. WoM)
	/*if(isr_motionint!=0)
			isr_motionint();	*/
			
	// Statistics
	mpu_cnt_int++;
			
	
	__mpu_sample_softdivider_ctr++;
	if(__mpu_sample_softdivider_ctr>__mpu_sample_softdivider_divider)
	{
		__mpu_sample_softdivider_ctr=0;

		// Statistics
		mpu_cnt_sample_tot++;
	
		// Automatically read data
		if(__mpu_autoread)
		{
			__mpu_data_packetctr_current=(unsigned short)mpu_cnt_sample_tot;
			// Initiate readout: 3xA+3*G+1*T+3*M+Ms = 21 bytes
			unsigned char r = mpu_readregs_int_cb(0,59,21,__mpu_read_cb);
			if(r)
			{
				mpu_cnt_sample_errbusy++;
			}
		
		}
	
		// motionint_ds called after software downsampling)
		/*if(isr_motionint_ds!=0)
		{
			isr_motionint_ds();	
		
			
			
		}*/
	}
}

void _mpu_enableautoread(void)
{
	// Clear the software divider counter
	__mpu_sample_softdivider_ctr=0;
	// Clear statistics counters
	mpu_cnt_int=0;
	mpu_cnt_sample_tot=0;
	mpu_cnt_sample_succcess=0;
	mpu_cnt_sample_errbusy=0;
	mpu_cnt_sample_errfull=0;
	// Clear data buffers
	mpu_data_rdptr=mpu_data_wrptr=0;
	// Enable automatic read
	__mpu_autoread=1;
	// Enable motion interrupts
	mpu_set_interrutenable(0,0,0,1);
	
}
void _mpu_disableautoread(void)
{
	// Disable motion interrupts
	mpu_set_interrutenable(0,0,0,0);
	// Disable automatic read
	__mpu_autoread=0;
}

/******************************************************************************
	__mpu_read_cb
*******************************************************************************	
	Callback called when the interrupt-driven MPU data acquisition completes.
	Copies the data in the temporary _mpu_tmp_reg into the mpu_data_xx buffers.
*******************************************************************************/
void __mpu_read_cb(void)
{
	// Two variants: immediately return if buffer is full, keeping the oldest data; or discard the oldest data and store new one
	
	// Immediately return if the buffer is full
	/*if(mpu_data_isfull())
	{
		mpu_cnt_sample_errfull++;
		return;
	}*/
	
	// Discard oldest data and store new one
	if(mpu_data_isfull())
	{
		mpu_data_rdnext();
		mpu_cnt_sample_errfull++;
		mpu_cnt_sample_succcess--;
	}

	// Conver the data
	signed short ax,ay,az,gx,gy,gz,mx,my,mz,temp;
	unsigned char ms;	
	ax=_mpu_tmp_reg[0]; ax<<=8; ax|=_mpu_tmp_reg[1];
	ay=_mpu_tmp_reg[2]; ay<<=8; ay|=_mpu_tmp_reg[3];
	az=_mpu_tmp_reg[4]; az<<=8; az|=_mpu_tmp_reg[5];
	temp=_mpu_tmp_reg[6]; temp<<=8; temp|=_mpu_tmp_reg[7];
	gx=_mpu_tmp_reg[8]; gx<<=8; gx|=_mpu_tmp_reg[9];
	gy=_mpu_tmp_reg[10]; gy<<=8; gy|=_mpu_tmp_reg[11];
	gz=_mpu_tmp_reg[12]; gz<<=8; gz|=_mpu_tmp_reg[13];

	mx=_mpu_tmp_reg[15]; mx<<=8; mx|=_mpu_tmp_reg[14];
	my=_mpu_tmp_reg[17]; my<<=8; my|=_mpu_tmp_reg[16];
	mz=_mpu_tmp_reg[19]; mz<<=8; mz|=_mpu_tmp_reg[18];
	ms=_mpu_tmp_reg[20];
	
	// correct the magnetometer
	switch(_mpu_mag_correctionmode)
	{
		case 0:
			mpu_data_mx[mpu_data_wrptr]=mx;
			mpu_data_my[mpu_data_wrptr]=my;
			mpu_data_mz[mpu_data_wrptr]=mz;
			break;
		case 1:
			mpu_mag_correct1(mx,my,mz,&mpu_data_mx[mpu_data_wrptr],&mpu_data_my[mpu_data_wrptr],&mpu_data_mz[mpu_data_wrptr]);
			break;			
		default:
			mpu_mag_correct2(mx,my,mz,&mpu_data_mx[mpu_data_wrptr],&mpu_data_my[mpu_data_wrptr],&mpu_data_mz[mpu_data_wrptr]);
	}
	
	
	// Fill the buffer
	mpu_data_ax[mpu_data_wrptr]=ax;
	mpu_data_ay[mpu_data_wrptr]=ay;
	mpu_data_az[mpu_data_wrptr]=az;
	mpu_data_gx[mpu_data_wrptr]=gx;
	mpu_data_gy[mpu_data_wrptr]=gy;
	mpu_data_gz[mpu_data_wrptr]=gz;
	mpu_data_ms[mpu_data_wrptr]=ms;
	mpu_data_temp[mpu_data_wrptr]=temp;
	mpu_data_time[mpu_data_wrptr]=timer_ms_get();	
	mpu_data_packetctr[mpu_data_wrptr]=__mpu_data_packetctr_current;
	mpu_data_wrnext();
	
	// Statistics
	mpu_cnt_sample_succcess++;
}

/******************************************************************************
	function: mpu_data_isfull
*******************************************************************************	
	Returns 1 if the buffer is full, 0 otherwise.
*******************************************************************************/
unsigned char mpu_data_isfull(void)
{
	if( ((mpu_data_wrptr+1)&(MPU_MOTIONBUFFERSIZE-1)) == mpu_data_rdptr )
		return 1;
	return 0;
}
/*unsigned char mpu_data_isempty(void)
{
	if(mpu_data_rdptr==mpu_data_wrptr)
		return 1;
	return 0;
}*/
/******************************************************************************
	function: mpu_data_level
*******************************************************************************	
	Returns how many samples are in the buffer
*******************************************************************************/
unsigned char mpu_data_level(void)
{
	return (mpu_data_wrptr-mpu_data_rdptr)&(MPU_MOTIONBUFFERSIZE-1);
}
/******************************************************************************
	mpu_data_wrnext
*******************************************************************************	
	Advances the write pointer. Do not call if the buffer is full.
*******************************************************************************/
void mpu_data_wrnext(void)
{
	mpu_data_wrptr = (mpu_data_wrptr+1)&(MPU_MOTIONBUFFERSIZE-1);
}
/******************************************************************************
	function: mpu_data_rdnext
*******************************************************************************	
	Advances the read pointer to access the next sample in the data buffer
	at index mpu_data_rdptr. 
	Do not call if the buffer is empty.
*******************************************************************************/
void mpu_data_rdnext(void)
{
	mpu_data_rdptr = (mpu_data_rdptr+1)&(MPU_MOTIONBUFFERSIZE-1);
}

/******************************************************************************
*******************************************************************************
ACC GYRO CONFIG   ACC GYRO CONFIG   ACC GYRO CONFIG   ACC GYRO CONFIG   ACC GYR
*******************************************************************************
*******************************************************************************/


/******************************************************************************
	function: mpu_init
*******************************************************************************	
	Initialise the MPU, the interrupt driven read, the magnetometer shadowing
	and puts the MPU and magnetometer in off mode.
	
*******************************************************************************/
void mpu_init(void)
{
	// Inititialize the interrupt-based mpu read
	//system_led_set(0b111); _delay_ms(800);
	mpu_get_agt_int_init();
	// Reset the MPU
	//system_led_set(0b110); _delay_ms(800);
	mpu_reset();
	// Deactivates the I2C interface (only SPI communication from now on)
	//system_led_set(0b101); _delay_ms(800);
	mpu_writereg(MPU_R_USR_CTRL,0x10);
	// Initialize the mpu interrupt 'style'	
	//system_led_set(0b100); _delay_ms(800);
	mpu_set_interruptpin(0x00);
	// Disable interrupt
	//system_led_set(0b011); _delay_ms(800);
	mpu_set_interrutenable(0,0,0,0);
	// Calibration
	//system_led_set(0b010); _delay_ms(800);
	mpu_calibrate();
	// Read magnetic field parameters
	//system_led_set(0b001); _delay_ms(800);
	mpu_config_motionmode(MPU_MODE_100HZ_ACC_BW41_GYRO_BW41_MAG_8,0);
	//system_led_set(0b111); _delay_ms(800);
	_mpu_mag_readasa();
	//system_led_set(0b110); _delay_ms(800);
	fprintf_P(file_pri,PSTR("Magnetometer ASA: %02X %02X %02X\n"),_mpu_mag_asa[0],_mpu_mag_asa[1],_mpu_mag_asa[2]);
	//system_led_set(0b101); _delay_ms(800);
	mpu_mag_loadcalib();	
	//system_led_set(0b100); _delay_ms(800);
	mpu_mag_printcalib(file_pri);
	// Turn off
	//system_led_set(0b011); _delay_ms(800);
	mpu_mode_off();
	// Dump status
	//system_led_set(0b010); _delay_ms(800);
	mpu_printregdesc(file_pri);
	//mpu_printregdesc(file_fb);	
	

}

/******************************************************************************
	Function: mpu_mode_accgyro
*******************************************************************************
	Set the MPU9250 in normal gyroscope and accelerometer mode
	- Gyroscope: on
	- Temperature: on
	- Accelerometer: on
	- clock: PLL
	
	Parameters:
		gdlpe		-		1 to enable gyroscope DLP.
							if enabled, gdlpbw specifies the bandwidth.
		gdlpoffhbw	-		when DLP off, 1 to set high bandwidth (8800Hz) or 0 to set low bandwidth (3600Hz)
		gdlpbw		-		when DLP on, set the DLP low pass filter. 
							Possible values: MPU_GYR_LPF_250, 184, 92, 41, 20, 10, 5, 3600
		adlpe		-		1 to enable the accelerometer DLP
		adlpbw		-		bandwidth of the accelerometer DLP filter (MPU_ACC_LPF_460... MPU_ACC_LPF_5)
		divider		-		divide the output of the DLP filter block by 1/(1+divider)
	
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
	
	
	_mpu_wakefromsleep();
	mpu_clksel(0b001);												// PLL
	mpu_temp_enable(1);												// Temperature
	mpu_writereg(MPU_R_PWR_MGMT_2,0b00000000);						// Enable accel, enable gyro
	mpu_writereg(MPU_R_ACCELCONFIG2,(adlpe<<3)|adlpbw);				// Accel DLP and bandwidth
	conf = mpu_readreg(MPU_R_CONFIG);
	gconf = mpu_readreg(MPU_R_GYROCONFIG);	
	mpu_writereg(MPU_R_CONFIG,(conf&0b11111000)|gdlpbw);			// Gyro lp filter
	mpu_writereg(MPU_R_GYROCONFIG,(gconf&0b11111100)|gfchoice_b);	// Gyro dlp
	mpu_setsrdiv(divider);	
}

/******************************************************************************
	mpu_mode_gyro
*******************************************************************************
	Set the MPU9250 in normal gyroscope-only mode
	- Gyroscope: on
	- Temperature: on
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
	
	_mpu_wakefromsleep();
	mpu_clksel(0b001);												// PLL
	mpu_temp_enable(1);												// Temperature
	mpu_writereg(MPU_R_PWR_MGMT_2,0b00111000);						// Enable gyro, disable accel
	conf = mpu_readreg(MPU_R_CONFIG);									
	gconf = mpu_readreg(MPU_R_GYROCONFIG);	
	mpu_writereg(MPU_R_CONFIG,(conf&0b11111000)|gdlpbw);			// Gyro lp filter
	mpu_writereg(MPU_R_GYROCONFIG,(gconf&0b11111100)|gfchoice_b);	// Gyro dlp	
	mpu_setsrdiv(divider);
	
}

/******************************************************************************
	mpu_mode_acc
*******************************************************************************
	Set the MPU9250 in normal accelerometer-only mode.
	- Gyroscope: off
	- Temperature: on
	- Accelerometer: on
	
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
	
	
	_mpu_wakefromsleep();
	mpu_clksel(0b000);												// Internal oscillator
	mpu_temp_enable(1);												// Temperature
	mpu_writereg(MPU_R_PWR_MGMT_2,0b00000111);						// Enable accel, disable gyro
	mpu_writereg(MPU_R_ACCELCONFIG2,(dlpenable<<3)|dlpbw);
	mpu_setsrdiv(divider);
}


/******************************************************************************
	mpu_mode_lpacc
*******************************************************************************
	Set the MPU9250 in low-power accelerometer-only mode.
	- Gyroscope: off
	- Temperature: off
	- Accelerometer: duty-cycled (low power)
******************************************************************************/
void mpu_mode_lpacc(unsigned char lpodr)
{
	printf_P(PSTR("mpu_mode_lpacc: %d\n"),lpodr);
	
	_mpu_wakefromsleep();
	mpu_clksel(0b000);												// Internal oscillator
	mpu_writereg(MPU_R_PWR_MGMT_2,0b00000111);						// Enable accel, disable gyro
	// TODO: CHECK whether fchoice and DLP influence LP operation
	mpu_writereg(MPU_R_ACCELCONFIG2,0b00001000);					// fchoice_b=1: DLP disabled; LP=x
	mpu_writereg(MPU_R_LPODR,lpodr);								// LP ODR
	mpu_writereg(MPU_R_PWR_MGMT_1,0b00101000);						// Enable cycle mode (LP), deactivate temp
}

/******************************************************************************
	function: mpu_mode_off
*******************************************************************************
	Turns of as many things to lower power consumtion.
	- Gyroscope: off
	- Temperature: off
	- Accelerometer: off
	- Magnetometer off
	
	This function can be called anytime. 
	First, the communication with the magnetometer is shortly enabled 
	(by activating the motion processor) in order to turn it off. Then the 
	accelerometer, gyroscope and temperature sensors are turned off.
******************************************************************************/
void mpu_mode_off(void)
{
	unsigned char gconf;
	
	// Enable the motion processor for magnetometer communication
	_mpu_defaultdlpon();
	// Shut down magnetometer sampling
	_mpu_mag_mode(0);
	
	mpu_writereg(MPU_R_PWR_MGMT_2,0b00111111);							// Disable accel + gyro
	mpu_writereg(MPU_R_ACCELCONFIG2,0b00001000);						// Disable accel DLP
	gconf = mpu_readreg(MPU_R_GYROCONFIG);	
	mpu_writereg(MPU_R_GYROCONFIG,(gconf&0b11111100)|0b01);				// Disable gyro dlp
	mpu_writereg(MPU_R_PWR_MGMT_1,0b01001000);							// Sleep, no cycle, no gyro stby, disable temp, internal osc
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
	
	mpu_readregs_int(v,59,14);
	
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
	mpu_get_agmt
*******************************************************************************
	Blocking read of acceleration, gyroscope, magnetometer and temperature.
******************************************************************************/
void mpu_get_agmt(signed short *ax,signed short *ay,signed short *az,signed short *gx,signed short *gy,signed short *gz,signed short *mx,signed short *my,signed short *mz,unsigned char *ms,signed short *temp)
{
	unsigned char v[21];
	unsigned short s1,s2,s3;
	
	mpu_readregs_int(v,59,21);
	
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
	s1=v[15]; s1<<=8; s1|=v[14];
	s2=v[17]; s2<<=8; s2|=v[16];
	s3=v[19]; s3<<=8; s3|=v[18];
	*mx = s1; *my = s2; *mz = s3;
	*ms=v[20];
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
	
	mpu_readregs_int(v,67,6);
	
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
	
	mpu_readregs_int(v,59,6);
	
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
	function: mpu_fifoenable
*******************************************************************************
	Sets the FIFO flags (register FIFO Enable=35d), sets the FIFO enable bit
	(register User Control=106d) and sets the FIFO reset bit (register User Control=106d).
	
	Parameters:
		flags	-	FIFO enable flags (reg 35d). This bitmask is composed of the following bits:
					
					TEMP GX GY GZ ACC SLV2 SLV1 SLV0
		en		-	1 to enable the FIFO (reg 106d)
		rst		-	1 to reset the FIFO (reg 106d)
******************************************************************************/
void mpu_fifoenable(unsigned char flags,unsigned char en,unsigned char reset)
{
	en=en?1:0;
	reset=reset?1:0;
	mpu_writereg(MPU_R_FIFOEN,flags);
	unsigned char usr = mpu_readreg(MPU_R_USR_CTRL);
	usr = usr&0b10111011;
	usr = usr|(en<<6)|(reset<<2);	
	mpu_writereg(MPU_R_USR_CTRL,usr);
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
	Read n bytes from the FIFO
******************************************************************************/
void mpu_fiforead(unsigned char *fifo,unsigned short n)
{
	for(unsigned short i=0;i<n;i++)
	{
		fifo[i] = mpu_readreg(116);
	}
}
/******************************************************************************
	mpu_fiforeadshort
*******************************************************************************	
	Read n 16-bit shorts from the FIFO
******************************************************************************/
void mpu_fiforeadshort(short *fifo,unsigned short n)
{
	unsigned char *t1=((unsigned char*)fifo)+1;
	unsigned char *t2=((unsigned char*)fifo)+0;
	for(unsigned short i=0;i<n;i++)
	{
		*t1=mpu_readreg(116);
		*t2=mpu_readreg(116);
		t1+=2;
		t2+=2;
	}
}

/******************************************************************************
	Function: mpu_setgyrobias
*******************************************************************************	
	Sets the gyro bias registers
******************************************************************************/
void mpu_setgyrobias(short bgx,short bgy,short bgz)
{
	mpu_writereg(19,bgx>>8);
	mpu_writereg(20,bgx&0xff);
	mpu_writereg(21,bgy>>8);
	mpu_writereg(22,bgy&0xff);
	mpu_writereg(23,bgz>>8);
	mpu_writereg(24,bgz&0xff);
}
/******************************************************************************
	Function: mpu_setgyroscale
*******************************************************************************	
	Sets the gyro scale.
	
	Parameters:
		scale	-	One of MPU_GYR_SCALE_250, MPU_GYR_SCALE_500, MPU_GYR_SCALE_1000 or MPU_GYR_SCALE_2000
******************************************************************************/
void mpu_setgyroscale(unsigned char scale)
{
	scale&=0b11;
	unsigned char gconf = mpu_readreg(MPU_R_GYROCONFIG);	
	mpu_writereg(MPU_R_GYROCONFIG,(gconf&0b11100111)|(scale<<3));
	
	switch(scale)
	{
		case MPU_GYR_SCALE_250:
			mpu_gtor=3.14159665k/180.0k/131.072k;
			break;
		case MPU_GYR_SCALE_500:
			mpu_gtor=3.14159665k/180.0k/65.536k;
			break;
		case MPU_GYR_SCALE_1000:
			mpu_gtor=3.14159665k/180.0k/32.768k;
			break;
		default:
			mpu_gtor=3.14159665k/180.0k/16.384k;
			break;
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
	_delay_ms(100);
}
/******************************************************************************
	function: mpu_setsrdiv
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
/*void mpu_setgyrosamplerate(unsigned char fchoice,unsigned char dlp)
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
}*/

/******************************************************************************
	mpu_setaccsamplerate
*******************************************************************************	
	Sets accelerometer sample rate and digital low pass filter
******************************************************************************/
/*void mpu_setaccsamplerate(unsigned char fchoice,unsigned char dlp)
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
}*/


/******************************************************************************
	function: mpu_set_interrutenable
*******************************************************************************
	Enable the interrupts of the MPU. The parameters take a binary value to 
	activate the corresponding interrupt source (1) or deactivate it (0).
	
	Parameters:
		wom		-	wake on motion
		fifo	-	FIFO overflow
		fsync	-	fsync interrupt
		datardy	-	new data acquired
******************************************************************************/
void mpu_set_interrutenable(unsigned char wom,unsigned char fifo,unsigned char fsync,unsigned char datardy)
{
	unsigned char v;
	v = (wom<<6)|(fifo<<4)|(fsync<<3)|datardy;
	mpu_writereg(MPU_R_INTERRUPTENABLE,v);	
}



/******************************************************************************
	function: mpu_temp_enable
*******************************************************************************
	Toggles the PD_PTAT bit in PWR_MGMT_1; i.e. enables or disables the 
	temperature conversion. 
	Does not affect any other setting.
******************************************************************************/
void mpu_temp_enable(unsigned char enable)
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
	function: _mpu_wakefromsleep
*******************************************************************************
	This function wakes up the MPU from sleep mode and waits until ready.	
	This function modifies the clock selection and the CYCLE and PTAT bits.
******************************************************************************/
void _mpu_wakefromsleep(void)
{
	// Clear SLEEP, clear STANDBY, clear CYCLE, enable PTAT, select internal oscillator
	mpu_writereg(MPU_R_PWR_MGMT_1,0);
	// Wait to wake up
	_delay_us(50);												
}



/******************************************************************************
	function: mpu_clksel
*******************************************************************************
	Changes the clksel bits in PWR_MGMT_1. Does not affect any other setting.
	
	Parameters:
		clk	-	Value between 0 and 7 inclusive corresponding to the clock source.
				
				0 Internal 20MHz oscillator
				
				1 Auto selects the best available clock source (Gyro X) – PLL if ready, else use the Internal oscillator. 
				
				2 Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
				
				3 Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
				
				4 Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
				
				5 Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
				
				6 Internal 20MHz oscillator
				
				7 Stops the clock and keeps timing generator in reset
******************************************************************************/
void mpu_clksel(unsigned char clk)
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




/*void mpu_testsleep()
{
	unsigned char r;
	while(1)
	{
		mpu_writereg(MPU_R_PWR_MGMT_1,0b01001000);							// Sleep, no cycle, no gyro stby, disable temp, internal osc
		printf("Sleep\n");
		_delay_ms(1);
		r = mpu_readreg(MPU_R_ACCELCONFIG2);
		printf("Config 2: %d\n",r);
		_delay_ms(1);
		mpu_writereg(MPU_R_ACCELCONFIG2,1);
		_delay_ms(1);
		r = mpu_readreg(MPU_R_ACCELCONFIG2);
		printf("Config 2: %d\n",r);
		_delay_ms(1);
		mpu_writereg(MPU_R_ACCELCONFIG2,2);
		_delay_ms(1);
		r = mpu_readreg(MPU_R_ACCELCONFIG2);
		printf("Config 2: %d\n",r);
		_delay_ms(1);
		mpu_writereg(MPU_R_ACCELCONFIG2,3);
		_delay_ms(1);
		r = mpu_readreg(MPU_R_ACCELCONFIG2);
		printf("Config 2: %d\n",r);
		printf("Wakeup\n");
		_delay_ms(500);
		
		mpu_writereg(MPU_R_PWR_MGMT_1,0b00001000);							// Sleep, no cycle, no gyro stby, disable temp, internal osc
		_delay_us(50);
		mpu_writereg(MPU_R_ACCELCONFIG2,4);
		r = mpu_readreg(MPU_R_ACCELCONFIG2);
		printf("Config 2: %d\n",r);
		mpu_writereg(MPU_R_ACCELCONFIG2,5);
		r = mpu_readreg(MPU_R_ACCELCONFIG2);
		printf("Config 2: %d\n",r);
		mpu_writereg(MPU_R_ACCELCONFIG2,6);
		r = mpu_readreg(MPU_R_ACCELCONFIG2);
		printf("Config 2: %d\n",r);
		//mpu_writereg(MPU_R_ACCELCONFIG2,1);		
		_delay_ms(2000);
	}
	
}*/


/******************************************************************************
	function: _mpu_defaultdlpon
*******************************************************************************	
	Enables the MPU in a default mode. Typically it:
	- activates the accelerometer and gyroscope at 500Hz with LPF at 184Hz
	- selects the PLL oscillator
	
	This mode allows to communicate with the magnetometer via the internal I2C
	interface.
	
	Note: do not use in user code.	
*******************************************************************************/
void _mpu_defaultdlpon(void)
{
	// Set gyro+acc mode, 184Hz BW for both, output rate 500Hz.
	//{ SAMPLE_MODE_ACCGYR,     1,         0, MPU_GYR_LPF_184,     1, MPU_ACC_LPF_184,      1,             0,     0},
	mpu_mode_accgyro(1,0,MPU_GYR_LPF_184, 1,MPU_ACC_LPF_184,1);
}

void mpu_acquirecalib(void)
{
	mpu_config_motionmode(MPU_MODE_500HZ_ACC_BW184_GYRO_BW184,0);
	//mpu_config_motionmode(MPU_MODE_500HZ_GYRO_BW184,0);
	
	_delay_ms(100);
	// Sensitivity should be 250dps and 2G
	mpu_fifoenable(0b01111000,1,1);			// Set FIFO for accel+gyro, enable FIFO, reset FIFO
	//mpu_fifoenable(0b01110000,1,1);				// Set FIFO for gyro, enable FIFO, reset FIFO
	// In Acc+Gyro: output is 12 bytes per samples. FIFO can hold 42 samples. Fills in 84ms at 500Hz. In 60ms expect 360 bytes.
	// In Gyro: output is 6 bytes per samples. FIFO can hold 85 samples. Fills in 170ms at 500Hz. In 130ms expect 390 bytes.
	_delay_ms(65);
	//_delay_ms(130);
	// Keep FIFO enabled but stop logging
	mpu_fifoenable(0b00000000,1,0);
}
void mpu_calibrate(void)
{
	unsigned short n;
	signed short ax,ay,az,gx,gy,gz;
	long gyro_bias[3];
	long acc_mean[3];
	long acc_std[3];
	gyro_bias[0]=gyro_bias[1]=gyro_bias[2]=acc_mean[0]=acc_mean[1]=acc_mean[2]=0;
	acc_std[0]=acc_std[1]=acc_std[2]=0;
	
	/*system_led_set(0b100); _delay_ms(200);
	system_led_set(0b000); _delay_ms(200);
	system_led_set(0b100); _delay_ms(200);
	system_led_set(0b000); _delay_ms(200);
	system_led_set(0b100); _delay_ms(200);
	system_led_set(0b000); _delay_ms(200);*/

	//system_led_set(0b111); _delay_ms(800);
	fprintf_P(file_pri,PSTR("MPU calibration\n"));
	//system_led_set(0b110); _delay_ms(800);
	mpu_acquirecalib();
	// Transfer the data into a shared buffer in order to compute standard deviation
	signed short *buffer = (signed short*)sharedbuffer;
	//system_led_set(0b101); _delay_ms(800);
	n = mpu_getfifocnt();
	//system_led_set(0b100); _delay_ms(800);
	fprintf_P(file_pri,PSTR(" FIFO level: %d\n"),n);
	unsigned short ns = n/12;
	for(unsigned short i=0;i<ns;i++)
	{
		mpu_fiforeadshort(&ax,1);
		mpu_fiforeadshort(&ay,1);
		mpu_fiforeadshort(&az,1);
		mpu_fiforeadshort(&gx,1);
		mpu_fiforeadshort(&gy,1);
		mpu_fiforeadshort(&gz,1);
		buffer[i*6+0] = ax;
		buffer[i*6+1] = ay;
		buffer[i*6+2] = az;
		buffer[i*6+3] = gx;
		buffer[i*6+4] = gy;
		buffer[i*6+5] = gz;
	}
	/*n = mpu_getfifocnt();
	fprintf_P(file_pri,PSTR("FIFO level after read: %d\n"),n);
	// Dump the values
	for(unsigned short i=0;i<ns;i++)
	{
		fprintf_P(file_pri,PSTR("%d %d %d  %d %d %d\n"),buffer[i*6+0],buffer[i*6+1],buffer[i*6+2],buffer[i*6+3],buffer[i*6+4],buffer[i*6+5]);
	}*/
	
	
	// Compute mean
	for(unsigned short i=0;i<ns;i++)
	{
		acc_mean[0]+=buffer[i*6+0];
		acc_mean[1]+=buffer[i*6+1];
		acc_mean[2]+=buffer[i*6+2];
		gyro_bias[0]+=buffer[i*6+3];
		gyro_bias[1]+=buffer[i*6+4];
		gyro_bias[2]+=buffer[i*6+5];		
	}
	acc_mean[0]/=ns;
	acc_mean[1]/=ns;
	acc_mean[2]/=ns;
	gyro_bias[0]=gyro_bias[0]/ns;
	gyro_bias[1]=gyro_bias[1]/ns;
	gyro_bias[2]=gyro_bias[2]/ns;
	// Compute standard deviation
	for(unsigned short i=0;i<ns;i++)
	{
		acc_std[0]+=(buffer[i*6+0]-acc_mean[0])*(buffer[i*6+0]-acc_mean[0]);
		acc_std[1]+=(buffer[i*6+1]-acc_mean[1])*(buffer[i*6+1]-acc_mean[1]);
		acc_std[2]+=(buffer[i*6+2]-acc_mean[2])*(buffer[i*6+2]-acc_mean[2]);
	}
	acc_std[0]=sqrt(acc_std[0]/(ns-1));
	acc_std[1]=sqrt(acc_std[1]/(ns-1));
	acc_std[2]=sqrt(acc_std[2]/(ns-1));
	//system_led_set(0b011); _delay_ms(800);
	fprintf_P(file_pri,PSTR("Avg acc: %ld %ld %ld\n"),acc_mean[0],acc_mean[1],acc_mean[2]);
	fprintf_P(file_pri,PSTR("Std acc: %ld %ld %ld\n"),acc_std[0],acc_std[1],acc_std[2]);	
	fprintf_P(file_pri,PSTR("Avg gyro: %ld %ld %ld\n"),gyro_bias[0],gyro_bias[1],gyro_bias[2]);
	gyro_bias[0]=-gyro_bias[0]/4;
	gyro_bias[1]=-gyro_bias[1]/4;
	gyro_bias[2]=-gyro_bias[2]/4;
	//system_led_set(0b010); _delay_ms(800);
	fprintf_P(file_pri,PSTR("Gyro bias: %ld %ld %ld\n"),gyro_bias[0],gyro_bias[1],gyro_bias[2]);
	
	long totstd = acc_std[0]+acc_std[1]+acc_std[2];
	if(totstd>500)
	{
		/*system_led_set(0b111); _delay_ms(800);
		system_led_set(0b000); _delay_ms(800);
		system_led_set(0b111); _delay_ms(800);
		system_led_set(0b000); _delay_ms(800);
		system_led_set(0b111); _delay_ms(800);
		system_led_set(0b000); _delay_ms(800);*/
		
		fprintf_P(file_pri,PSTR("******************************************\n"));
		fprintf_P(file_pri,PSTR("*TOO MUCH MOVEMENT-RISK OF MISCALIBRATION*\n"));
		fprintf_P(file_pri,PSTR("******************************************\n"));
	}
	//system_led_set(0b001); _delay_ms(800);
	mpu_setgyrobias(gyro_bias[0],gyro_bias[1],gyro_bias[2]);
	
	
}

/******************************************************************************
*******************************************************************************
MAGNETOMETER   MAGNETOMETER   MAGNETOMETER   MAGNETOMETER   MAGNETOMETER   MAGN
*******************************************************************************
*******************************************************************************
Slave 0 is used for shadowing.
Slave 4 is used for individual register access with mpu_mag_readreg and mpu_mag_writereg.
Slave 4 is only temporarily enabled during mpu_mag_readreg and mpu_mag_writereg calls, and 
then deactivated.

In all cases the magnetometer functions should only be called when the motion 
processor is active.
*******************************************************************************/

/******************************************************************************
	function: mpu_mag_readreg
*******************************************************************************	
	Reads the content of a magnotemeter register. 
	
	This function blocks until the data is read or a timeout occurs.
		
	In order for this function to work, the MPU I2C interface must be active
	(mpu_mag_interfaceenable) and the MPU must be configured to sample 
	acceleration or gyroscope data (not in low power mode).
	If not, the I2C interface is not active and the function will hang.
	
	Parameters:
		reg	-	Magnetometer register to read
	Returns:
		register 
*******************************************************************************/
unsigned char mpu_mag_readreg(unsigned char reg)
{
	
	mpu_writereg(MPU_R_I2C_SLV4_ADDR,MAG_ADDRESS|0x80);			// Magnetometer read address
	mpu_writereg(MPU_R_I2C_SLV4_REG,reg);						// Register to read
	mpu_writereg(MPU_R_I2C_SLV4_CTRL,0b10000000);				// Enable slave 4	
	unsigned long t1 = timer_ms_get();
	while( !(mpu_readreg(MPU_R_I2C_MST_STATUS)&0x40) )			// Wait for I2C_SLV4_DONE bit		
	{
		if(timer_ms_get()-t1>250)
		{
			fprintf_P(file_pri,PSTR("mag rd err\n"));
			mpu_writereg(MPU_R_I2C_SLV4_CTRL,0b00000000);		// Disable slave 4			
			return 0xff;
		}
	}
	mpu_writereg(MPU_R_I2C_SLV4_CTRL,0b00000000);				// Disable slave 4
	return mpu_readreg(MPU_R_I2C_SLV4_DI);						// Return input data
}
/******************************************************************************
	function: mpu_mag_writereg
*******************************************************************************	
	Writes the content of a magnotemeter register. 
	
	This function blocks until the data is written or a timeout occurs.
	
	In order for this function to work, the MPU I2C interface must be active
	(mpu_mag_interfaceenable) and the MPU must be configured to sample 
	acceleration or gyroscope data (not in low power mode).
	If not, the I2C interface is not active and the function will hang.
	
	Parameters:
		reg	-	Magnetometer register to write to
		val	-	Value to write in register
*******************************************************************************/
void mpu_mag_writereg(unsigned char reg,unsigned char val)
{	
	mpu_writereg(MPU_R_I2C_SLV4_ADDR,MAG_ADDRESS);				// Magnetometer write address
	mpu_writereg(MPU_R_I2C_SLV4_REG,reg);						// Register to write
	mpu_writereg(MPU_R_I2C_SLV4_DO,val);						// Data to write
	mpu_writereg(MPU_R_I2C_SLV4_CTRL,0b10000000);				// Enable slave 4
	unsigned long t1 = timer_ms_get();
	while( !(mpu_readreg(MPU_R_I2C_MST_STATUS)&0x40))			// Wait for I2C_SLV4_DONE bit
	{
		if(timer_ms_get()-t1>250)
		{
			fprintf_P(file_pri,PSTR("mag rd err\n"));
			break;
		}
	}
	mpu_writereg(MPU_R_I2C_SLV4_CTRL,0b00000000);				// Disable slave 4
}


/******************************************************************************
	function: _mpu_mag_interfaceenable
*******************************************************************************	
	Enables the I2C communication interface between the magnetometer and the 
	I2C master within the MPU.
	
	In addition the MPU must be configured to sample acceleration or gyroscope data 
	(not in low power mode).	
	
	Parameters:
		en	-	Enable (1) or disable (0) the interface
	
*******************************************************************************/
void _mpu_mag_interfaceenable(unsigned char en)
{
	// Ensures the slaves are disabled
	mpu_writereg(MPU_R_I2C_SLV4_CTRL,0);
	mpu_writereg(MPU_R_I2C_SLV0_CTRL,0);
	mpu_writereg(MPU_R_I2C_MST_DELAY_CTRL,0b10000000);		// Set DELAY_ES_SHADOW, do not enable periodic slave access (i.e. I2C enabled but not active)
	if(en)
	{
		unsigned char usr = mpu_readreg(MPU_R_USR_CTRL);
		mpu_writereg(MPU_R_USR_CTRL,usr|0b00100000);		// Set I2C_MST_EN			
		mpu_writereg(MPU_R_I2C_MST_CTRL,0b11000000);		// Set MULT_MST_EN, WAIT_FOR_ES. TODO: check if needed.
	}
	else
	{
		// Deactivates periodic slave access		
		mpu_writereg(MPU_R_I2C_MST_CTRL,0b00000000);		// Clear MULT_MST_EN, clear WAIT_FOR_ES. TODO: check if needed.
		unsigned char usr = mpu_readreg(MPU_R_USR_CTRL);
		mpu_writereg(MPU_R_USR_CTRL,usr&0b11011111);		// Clears I2C_MST_EN		
	}
}


/******************************************************************************
	function: _mpu_mag_mode
*******************************************************************************	
	Enables or disable the conversion of the magnetometer sensors.

	This function must only be called if the communication interface is 
	enabled (mpu_mag_interfaceenable) and if the motion processor is active.
	Otherwise, timeout occur.
	
	Note: do not use in user code.
		
	Parameters:
		en	-	0: sleep mode. 1: 8Hz conversion. 2: 100Hz conversion.
	
*******************************************************************************/
void _mpu_mag_mode(unsigned char mode)
{
	// Always enable the interface, even if the mode is to sleep, as 
	// this could be called while the magnetometer is already sleeping
	_mpu_mag_interfaceenable(1);					
	switch(mode)
	{
		case 0:
			mpu_mag_writereg(0x0a,0b00010000);		// Power down, 16 bit
			_mpu_mag_regshadow(0,0,0,0);			// Stop shadowing
			_mpu_mag_interfaceenable(0);			// Stop I2C interface
			break;
		case 1:
			mpu_mag_writereg(0x0a,0b00010010);		// Continuous mode 1, 16 bit
			_mpu_mag_regshadow(1,0,3,7);			// Start shadowing
			break;
		case 2:
		default:
			mpu_mag_writereg(0x0a,0b00010110);		// Continuous mode 2, 16 bit
			_mpu_mag_regshadow(1,0,3,7);			// Start shadowing
			break;
	}
}



/******************************************************************************
	function: _mpu_mag_regshadow
*******************************************************************************	
	Setups the MPU to shadow the registers for the magnetometer into the MPU
	external sensor memory (reg 73d to 96d).
	
	In order for this function to work, the MPU I2C interface must be active
	(mpu_mag_interfaceenable) and the MPU must be configured to sample 
	acceleration or gyroscope data (not in low power mode).
	
	In this mode, the MPU reads at periodic interval the magnetometer using the 
	on-chip I2C interface. This allows the application to transparently read the 
	magnetometer data using the mpu_readreg function, instead of the mpu_mag_readreg
	function. 
	
	The magnetometer must be configured separately for continuous conversion mode.
	
	This only works when the MPU is not in low-power acceleration mode.
	
	The magnetometer is read at the output data rate ODR/(1+dly), where dly is 
	configurable.

	
	Parameters:
		enable		-	Enable (1) or disable (0) register shadowing
		dly			-	Read magnetometer at frequency ODR/(1+dly). dly e [0;31]
		regstart	-	First magnetometer register to mirror
		numreg		-	Number of magnetometer registers to mirror. numreg e [0;15]
	Returns:
		register 
*******************************************************************************/
void _mpu_mag_regshadow(unsigned char enable,unsigned char dly,unsigned char regstart,unsigned char numreg)
{
	if(dly>31) dly=31;
	if(numreg>15) numreg=15;

	if(enable)
	{
		mpu_writereg(MPU_R_I2C_SLV4_CTRL,dly);					// R52: slave 4 disabled (it is always disabled anyways); set I2C_MST_DLY	
		
		/*
		// 
		unsigned char usr = mpu_readreg(MPU_R_USR_CTRL);		// R106 USER_CTRL read current status
		usr = usr&0b11011111;									// Keep other settings
		usr = usr|0b00110000;									// Enable MST_I2C, disable I2C_IF
		mpu_writereg(MPU_R_USR_CTRL,usr);						// R106 USER_CTRL update*/
		
		//mpu_writereg(MPU_R_I2C_MST_CTRL,0b11000000);			// R36: enable multi-master (needed?), wait_for_es, set I2C clock 	// already setup when interface is enabled
		
		mpu_writereg(MPU_R_I2C_SLV0_ADDR,MAG_ADDRESS|0x80);		// SLV0 read mode
		mpu_writereg(MPU_R_I2C_SLV0_REG,regstart);				// SLV0 start register
		mpu_writereg(MPU_R_I2C_MST_DELAY_CTRL,0b10000001);		// R 103 DELAY_ES_SHADOW | I2C_SLV0_DLY_EN
		mpu_writereg(MPU_R_I2C_SLV0_CTRL,0x80 | numreg);		// R 39: SLV0_EN | number of registers to copy
	}
	else
	{
		mpu_writereg(MPU_R_I2C_SLV4_CTRL,0);					
		mpu_writereg(MPU_R_I2C_SLV0_CTRL,0);	
//		_delay_ms(10);
		mpu_writereg(MPU_R_I2C_MST_DELAY_CTRL,0b10000000);		
//		_delay_ms(10);
	}
}


/******************************************************************************
	function: _mpu_mag_readasa
*******************************************************************************	
	Read the magnetometer axis sensitivity adjustment value.
	Assumes MPU is in a mode allowing communication with the magnetometer.	
*******************************************************************************/
void _mpu_mag_readasa(void)
{
	for(unsigned i=0;i<3;i++)
		_mpu_mag_asa[i] = mpu_mag_readreg(0x10+i);
	/*
	// Datasheet indicates to enable "fuse mode", however readout of asa is identical
	printf("Mag asa: %d %d %d\n",_mpu_mag_asa[0],_mpu_mag_asa[1],_mpu_mag_asa[2]);
	// enable fuse mode
	mpu_mag_writereg(0x0a,0b00011111);		// Fuse mode, 16 bit
	for(unsigned i=0;i<3;i++)
		m[i] = mpu_mag_readreg(0x10+i);
	// power down
	mpu_mag_writereg(0x0a,0b00010000);		// Power down, 16 bit
	printf("Mag asa with fuse mode: %d %d %d\n",m[0],m[1],m[2]);
	*/
}


// Magnetic correction using factory parameters in fuse rom
void mpu_mag_correct1(signed short mx,signed short my,signed short mz,signed short *mx2,signed short *my2,signed short *mz2)
{
	signed long lmx,lmy,lmz;
	lmx=mx;
	lmy=my;
	lmz=mz;
	
	signed long ax=_mpu_mag_asa[0],ay=_mpu_mag_asa[1],az=_mpu_mag_asa[2];
	
	ax=ax-128;
	ay=ay-128;
	az=az-128;
	
	//printf("%ld %ld %ld  %ld %ld %ld\n",lmx,lmy,lmz,ax,ay,az);
	
	lmx=lmx+(lmx*ax)/256;
	lmy=lmy+(lmy*ay)/256;
	lmz=lmz+(lmz*az)/256;
	
	//printf("aft %ld %ld %ld\n",lmx,lmy,lmz);
	
	// Could wrap around as the operation is from H-0.5H to H+0.5H and the H can be up to +/-32768; however if measuring earth field the max values are ~450 so no overflow
	
	*mx2=lmx;
	*my2=lmy;
	*mz2=lmz;
	
	
}
// Magnetic correction using 
void mpu_mag_correct2(signed short mx,signed short my,signed short mz,signed short *mx2,signed short *my2,signed short *mz2)
{
	*mx2=(mx+_mpu_mag_bias[0])*_mpu_mag_sens[0]/128;
	*my2=(my+_mpu_mag_bias[1])*_mpu_mag_sens[1]/128;
	*mz2=(mz+_mpu_mag_bias[2])*_mpu_mag_sens[2]/128;
}

void mpu_mag_calibrate(void)
{
	signed m[3];
	WAITPERIOD p=0;
	unsigned long t1;

	// Activate a magnetic mode
	mpu_config_motionmode(MPU_MODE_100HZ_ACC_BW41_GYRO_BW41_MAG_100,1);
	
	// Deactivate the automatic correction
	unsigned char _mpu_mag_correctionmode_back = _mpu_mag_correctionmode;
	_mpu_mag_correctionmode=0;

	_mpu_mag_calib_max[0]=_mpu_mag_calib_max[1]=_mpu_mag_calib_max[2]=-32768;
	_mpu_mag_calib_min[0]=_mpu_mag_calib_min[1]=_mpu_mag_calib_min[2]=+32767;

	fprintf_P(file_pri,PSTR("Magnetometer calibration: far from any metal, move the sensor in all orientations until no new numbers appear on screen and then press a key\n"));

	t1=timer_ms_get();
	while(1)
	{
		if( fgetc(file_pri) != -1)
			break;
		timer_waitperiod_ms(10,&p);
		
		m[0] = mpu_data_mx[mpu_data_rdptr];
		m[1] = mpu_data_my[mpu_data_rdptr];
		m[2] = mpu_data_mz[mpu_data_rdptr];
		mpu_data_rdnext();
		
		unsigned char dirty=0;
		for(unsigned char i=0;i<3;i++)
		{
			if(m[i]<_mpu_mag_calib_min[i])
			{
				_mpu_mag_calib_min[i]=m[i];
				dirty=1;
			}
			if(m[i]>_mpu_mag_calib_max[i])
			{
				_mpu_mag_calib_max[i]=m[i];
				dirty=1;
			}
		}
		
		if(dirty)
		{
			printf("[%d %d %d] - [%d %d %d]\n",_mpu_mag_calib_min[0],_mpu_mag_calib_min[1],_mpu_mag_calib_min[2],_mpu_mag_calib_max[0],_mpu_mag_calib_max[1],_mpu_mag_calib_max[2]);
		}
		if(timer_ms_get()-t1>1000)
		{
			printf("%d %d %d\n",m[0],m[1],m[2]);
			t1=timer_ms_get();
		}
	}

	// Restore the automatic correction
	_mpu_mag_correctionmode=_mpu_mag_correctionmode_back;

	
	//mpu_config_motionmode(MPU_MODE_OFF,0);
	
	// compute the coefficients
	for(unsigned char i=0;i<3;i++)
	{
		_mpu_mag_bias[i] = -(_mpu_mag_calib_max[i]+_mpu_mag_calib_min[i])/2;
		if(_mpu_mag_calib_max[i]-_mpu_mag_calib_min[i]==0)
			_mpu_mag_sens[i]=1;
		else
			_mpu_mag_sens[i] = (128*128)/(_mpu_mag_calib_max[i]-_mpu_mag_calib_min[i]);
	}
	mpu_mag_printcalib(file_pri);	
	mpu_mag_storecalib();	
}

void mpu_mag_storecalib(void)
{
	for(unsigned char i=0;i<3;i++)
	{
		eeprom_write_byte((uint8_t*)(CONFIG_ADDR_MAG_BIASXL+i*2),_mpu_mag_bias[i]&0xff);
		eeprom_write_byte((uint8_t*)CONFIG_ADDR_MAG_BIASXL+i*2+1,(_mpu_mag_bias[i]>>8)&0xff);
		
		eeprom_write_byte((uint8_t*)CONFIG_ADDR_MAG_SENSXL+i*2,_mpu_mag_sens[i]&0xff);
		eeprom_write_byte((uint8_t*)CONFIG_ADDR_MAG_SENSXL+i*2+1,(_mpu_mag_sens[i]>>8)&0xff);
	}
}
void mpu_mag_loadcalib(void)
{
	unsigned short t;
	for(unsigned char i=0;i<3;i++)
	{
		t = 0;
		t = eeprom_read_byte((uint8_t*)CONFIG_ADDR_MAG_BIASXL+i*2+1);
		t<<=8;
		t |= eeprom_read_byte((uint8_t*)CONFIG_ADDR_MAG_BIASXL+i*2);
		_mpu_mag_bias[i]=t;
		
		t = 0;
		t = eeprom_read_byte((uint8_t*)CONFIG_ADDR_MAG_SENSXL+i*2+1);
		t<<=8;
		t |= eeprom_read_byte((uint8_t*)CONFIG_ADDR_MAG_SENSXL+i*2);
		_mpu_mag_sens[i]=t;
	}
	_mpu_mag_correctionmode=eeprom_read_byte((uint8_t*)(CONFIG_ADDR_MAG_CORMOD));
}
void mpu_mag_correctionmode(unsigned char mode)
{
	_mpu_mag_correctionmode=mode;
	eeprom_write_byte((uint8_t*)(CONFIG_ADDR_MAG_CORMOD),mode);
}

/******************************************************************************
*******************************************************************************
STATUS PRINT   STATUS PRINT   STATUS PRINT   STATUS PRINT   STATUS PRINT   STAT
*******************************************************************************
*******************************************************************************/



/******************************************************************************
	function: mpu_printreg
*******************************************************************************	
	Prints the 128 registers from the MPU9250
	
	Parameters:
		file	-	Output stream
*******************************************************************************/
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

/******************************************************************************
	function: mpu_printreg
*******************************************************************************	
	Prints the 128 registers from the MPU9250
	
	Parameters:
		file	-	Output stream
*******************************************************************************/
void mpu_printextreg(FILE *file)
{
	unsigned char v;
	unsigned char j=0;
	
	fprintf_P(file,PSTR("External sensor:\n"));	
	for(unsigned char i=73;i<=96;i++)
	{
		fprintf_P(file,PSTR("%02X: "),i);
		v = mpu_readreg(i);
		fprintf_P(file,PSTR("%02X"),v);
		if((j&0x7)==7 || i==96)
			fprintf_P(file,PSTR("\n"));
		else
			fprintf_P(file,PSTR("   "));
		j++;
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
	fprintf_P(file,PSTR(" PWR_MGMT_1(6Bh):    %02X. rst: %d sleep: %d cycle: %d gyro_stby: %d PD-PTAT: %d sel: %dd\n"),v[0x6B],v[0x6B]&0x80?1:0,v[0x6B]&0x40?1:0,v[0x6B]&0x20?1:0,v[0x6B]&0x10?1:0,v[0x6B]&0x08?1:0,v[0x6B]&0b111);
	fprintf_P(file,PSTR(" PWR_MGMT_2(6Ch):    %02X. dis_xa: %d dis_ya: %d dis_za: %d dis_xg: %d dis_yg: %d dis_zg: %d\n"),v[0x6C],v[0x6C]&0x20?1:0,v[0x6C]&0x10?1:0,v[0x6C]&0x08?1:0,v[0x6C]&0x04?1:0,v[0x6C]&0x02?1:0,v[0x6C]&0x01);
}

/******************************************************************************
	function: mpu_printfifo
*******************************************************************************	
	Prints the content of the FIFO 
	
	Parameters:
		file	-	Output stream
*******************************************************************************/
void mpu_printfifo(FILE *file)
{
	unsigned char d;
	unsigned short n;
	n = mpu_getfifocnt();
	fprintf_P(file_pri,PSTR("FIFO level: %d\n"),n);
	if(n)
	{
		for(unsigned short i=0;i<n;i++)
		{
			mpu_fiforead(&d,1);
			fprintf_P(file_pri,PSTR("%02X "),(int)d);
			if((i&7)==7 || i==n-1)
				fprintf_P(file_pri,PSTR("\n"));
		}
		n = mpu_getfifocnt();
		fprintf_P(file_pri,PSTR("FIFO level: %d\n"),n);
	}
}

/******************************************************************************
	function: mpu_mag_printreg
*******************************************************************************	
	Prints the registers from the magnetometer.
	
	The MPU I2C interface must be active (mpu_mag_interfaceenable) 
	and the MPU must be configured to sample acceleration or gyroscope data 
	(not in low power mode).
	
	Parameters:
		file	-	Output stream
*******************************************************************************/
void mpu_mag_printreg(FILE *file)
{
	unsigned char v;
	
	fprintf_P(file,PSTR("Mag registers:\n"));	
	for(unsigned char i=0;i<=0x13;i++)
	{
		fprintf_P(file,PSTR("%02X: "),i);
		v = mpu_mag_readreg(i);
		fprintf_P(file,PSTR("%02X"),v);
		if( (i&7)==7 || i==0x13)
			fprintf_P(file,PSTR("\n"));
		else
			fprintf_P(file,PSTR("   "));
	}		
	
}

void mpu_mag_printcalib(FILE *f)
{
	fprintf_P(f,PSTR("M.bias: %d %d %d\n"),_mpu_mag_bias[0],_mpu_mag_bias[1],_mpu_mag_bias[2]);
	fprintf_P(f,PSTR("M.sens: %d %d %d\n"),_mpu_mag_sens[0],_mpu_mag_sens[1],_mpu_mag_sens[2]);
	fprintf_P(f,PSTR("M.corrmode: %d\n"),_mpu_mag_correctionmode);
}

/******************************************************************************
	function: mpu_printstat
*******************************************************************************	
	Prints interrupt-driven (automatic mode) data acquisition statistics.
	
	Parameters:
		file	-	Output stream
*******************************************************************************/
void mpu_printstat(FILE *file)
{
	fprintf_P(file,PSTR("MPU acquisition statistics:\n"));
	fprintf_P(file,PSTR(" MPU interrupts: %lu\n"),mpu_cnt_int);
	fprintf_P(file,PSTR(" Samples: %lu\n"),mpu_cnt_sample_tot);
	fprintf_P(file,PSTR(" Samples success: %lu\n"),mpu_cnt_sample_succcess);
	fprintf_P(file,PSTR(" Errors: busy=%lu buffer=%lu\n"),mpu_cnt_sample_errbusy,mpu_cnt_sample_errfull);
	fprintf_P(file,PSTR(" Buffer level: %u\n"),mpu_data_level());
}



/*unsigned short mpu_estimateodr(void)
{
	// Estimate ODR of accelerometer
	unsigned long fr;
	unsigned short odr;
	fr = mpu_estimatefifofillrate(0b00001000);
	printf_P(PSTR("FIFO accel fill rate: %ld. ODR: %ld\n"),fr,fr/6);
	odr=fr/6;
	//fr = mpu_estimatefifofillrate(0b01000000);
	//printf_P(PSTR("FIFO gyro x fill rate: %ld. ODR: %ld\n"),fr,fr/2);
	//fr = mpu_estimatefifofillrate(0b01110000);
	//printf_P(PSTR("FIFO gyro x,y,z fill rate: %ld. ODR: %ld\n"),fr,fr/6);
	//fr = mpu_estimatefifofillrate(0b10000000);
	//printf_P(PSTR("FIFO temp fill rate: %ld. ODR: %ld\n"),fr,fr/2);
	//fr = mpu_estimatefifofillrate(0b11111000);
	//printf_P(PSTR("FIFO temp+acc+gyro fill rate: %ld. ODR: %ld\n"),fr,fr/14);
	
	return odr;
}*/

/*unsigned long mpu_estimatefifofillrate(unsigned char fenflag)
{
	unsigned char d;
	unsigned short n;
	unsigned long t1,t2;
	unsigned char debug=0;
	
	if(debug) printf_P(PSTR("Disable FIFO\n"));
	mpu_fifoenable(0x00);
	n = mpu_getfifocnt();
	if(debug) printf_P(PSTR("Empty FIFO: %d bytes\n"),n);
	for(unsigned short i=0;i<n;i++)
		mpu_fiforead(&d,1);
	n = mpu_getfifocnt();
	if(debug) printf_P(PSTR("FIFO level: %d\n"),n);
	_delay_ms(10);
	n = mpu_getfifocnt();
	if(debug) printf_P(PSTR("FIFO level: %d\n"),n);
	if(n!=0)
	{
		printf_P(PSTR("Error emptying FIFO: level %d\n"),n);
		return 0;
	}
	if(debug) printf_P(PSTR("Start ODR estimation - enable FIFO\n"));
	mpu_fifoenable(fenflag);		
	t1=timer_ms_get();
	while(1)
	{
		n = mpu_getfifocnt();
		t2 = timer_ms_get();
		
		if(n>450)
			break;
	}
	if(debug) printf_P(PSTR("End ODR estimation - disable FIFO\n"));
	mpu_fifoenable(0x00);
	if(debug) printf_P(PSTR("Got %d bytes in %ld ms\n"),n,t2-t1);
	unsigned long odr;
	odr = n*1000l/(t2-t1);
	if(debug) printf_P(PSTR("FIFO fills at %ld bytes/s\n"),odr);
	return odr;
}*/


