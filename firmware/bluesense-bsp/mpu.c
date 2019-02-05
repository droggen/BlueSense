#include "cpu.h"
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
#include "mpu_config.h"
#include "system.h"
#include "dbg.h"
#include "helper.h"
#include "uiconfig.h"
#include "mpu_geometry.h"

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
	* mpu_data_getnext_raw:		Returns the next data in the buffer (when automatic read is active).
	* mpu_data_getnext:			Returns the next raw and geometry data (when automatic read is active).
	
	
	In non automatic read, the functions mpu_get_a, mpu_get_g, mpu_get_agt or mpu_get_agmt must be used to acquire the MPU data. These functions can also be called in automatic
	read, however this is suboptimal and increases CPU load as the data would be acquired in the interrupt routine and 	through this function call.
	
	
	*Statistics*
	The following counters are available to monitor the interrupt-driven automatic read. These counters are cleared upon calling mpu_config_motionmode with a new acquisition mode:
	
	* mpu_cnt_int:				counts how often the MPU isr has been triggered
	* mpu_cnt_sample_tot:		counts how many samples occurred, independently of whether successful or not (this is equal or lower than mpu_cnt_int due to possible software downsampling)
	* mpu_cnt_sample_succcess:	counts how many samples were acquired and successfully stored in the buffer; mpu_cnt_sample_succcess-mpu_cnt_sample_tot indicates the number of errors that occurred
	* mpu_cnt_sample_errbusy:	counts how many samples were skipped because the interface was busy (e.g. from an ongoing transfer)
	* mpu_cnt_sample_errfull: 	counts how many samples were skipped because the buffer was full 
	
	
	*Units*
	The variable mpu_gtorps can be used to convert the gyroscope readings to radians per second. This variable is updated when mpu_setgyroscale is called.
	Depending on the compiler it is a fixed-point (_Accum) or float variable. Convert the gyroscope reading to radians per second as follows: 
	
	_Accum gx_rps = mpu_data_gx[mpu_data_rdptr] * mpu_gtorps;
	or
	float gx_rps = mpu_data_gx[mpu_data_rdptr] * mpu_gtorps;
	
	Gyroscope sensitivity: 
		full scale 250:			131.072LSB/dps
		full scale 500:			65.536LSB/dps
		full scale 1000:		32.768LSB/dps
		full scale 2000:		16.384LSB/dps
	
	
	*Magnetometer*	
	
	Functions to handle the magnetometer - prefixed by mpu_mag_... - can only be used when the MPU is configured
	in a mode where the magnetic field sensor is turned on; otherwise the function time-out due to the magnetic field
	sensor being inaccessible via the internal I2C interface.
	
	
	*Interrupt Service Routine (ISR)*
	
	The function mpu_isr must be called on the *falling* edge of the MPU interrupt pin. 
	See next section for the rationale for using the falling edge.
	
	*Interrupt Service Routine (ISR) Internal notes*
	
	The MPU can be configured with register 55d (MPU_R_INTERRUPTPIN):
		- Logic low or logic high
		- Open-drain or push-pull
		- Level held until cleared or generate a 50uS pulse
		- Interrupt cleared after any read operation or after reading the INT_STATUS register
		
	Here "logic high", "push pull" and "50uS" is used. 
	Rationale: in the high gyro high bandwidth mode, the ODR is 8000Hz; performing a read
	to clear the interrupt (required with the level held interrupt) is significantly more costly than doing nothing and returning from 
	the ISR (possible in the pulse mode). 	While not tested, the overhead of checking the status register appears prohibitive.
	Therefore, the pulse mode is more suitable.
	
	The ISR should react on the positive edge of the pulse. The initial approach was for the ISR checks that the pin is high and then
	call mpu_isr. An issue arises when other interrupts may delay the pin change interrrupt by more than 50uS (553 clk): 
	the ISR would sample the pin level as zero and would not call the mpu_isr, essentially missing the MPU interrupt.
	
	The alternative is to trigger mpu_isr on the falling edge of the pulse. As the pulse stays most of the time in the low
	state, this gives a higher likelihood for the ISR to be triggered when the pin state is low. 
	
	In a gyro high bandwidth mode, the interrupts are triggered at ODR=8KHz or every 125 uS.
	At 8KHz the pin would transition as follows: ...000I111-50uS111I00000-75uS00000I111-50uS111I000...
	                                                   I<          125uS          >I
	
													   
	At ODR=1KHz interrupts are triggered every 125 uS. This is the 500Hz low-bandwidth mode
	At 1KHz the pin would transition as follows: ...000I111-50uS111I000000000-950uS000000000I111-50uS111I000...
	                                                   I<              1000uS              >I
	The 950uS where interrupt pin is low provides enough time for the ISR to be called, check that the pin is low
	and call the mpu_isr. 
	
	Therefore, triggering the mpu_isr when the pin is low is recommended.
	
	Possible issue:
			000I111-50uS111I000000000-950uS000000000I111-50uS111I000...
			   i           i												Pin change interrupt				
			   ...........I..C..M...R...I?									
																			
	I indicates when the pin change ISR is called, possibly with significant delay.
	C indicates when the pin change ISR checks the pin state, which is now low: mpu_isr is called (M) and returns (R).
	The interrupt pin toggles from high to low during the pin change ISR.
	The AVR datasheet indicates about the pin change interrupt flag: "The flag is cleared when the interrupt routine is executed."
	Interpreting this as when the interrupt is executed, the scenario above would call the mpu_isr a second time 
	(indicated by the 2nd I) while in reality there is only one interrupt.
	
	This can be addressed by reading the interrupt status register in the MPU routine at a cost of overhead.
	
	Solution decided: clear PCIFR bit 2 to avoid superfluous calls to mpu_isr and check in mpu_isr for spurious motion interrupts.
	
	In practice the firmware only handles up to ODR=500Hz (500Hz LBW) without data loss.
	
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

// Various
const char _str_mpu[] = "MPU: ";

unsigned char __mpu_sample_softdivider_ctr=0,__mpu_sample_softdivider_divider=0;

// Data buffers
MPUMOTIONDATA mpu_data[MPU_MOTIONBUFFERSIZE];
volatile unsigned long __mpu_data_packetctr_current;
volatile unsigned char mpu_data_rdptr,mpu_data_wrptr;
volatile MPUMOTIONDATA _mpumotiondata_test;

// Magnetometer Axis Sensitivity Adjustment
unsigned char _mpu_mag_asa[3];
signed short _mpu_mag_calib_max[3];
signed short _mpu_mag_calib_min[3];
signed short _mpu_mag_bias[3];
signed short _mpu_mag_sens[3];
unsigned char _mpu_mag_correctionmode;

// Automatic read statistic counters
unsigned long mpu_cnt_int, mpu_cnt_sample_tot, mpu_cnt_sample_succcess, mpu_cnt_sample_errbusy, mpu_cnt_sample_errfull;

unsigned long mpu_cnt_spurious;

unsigned char __mpu_autoread=0;

unsigned char _mpu_current_motionmode=0;

unsigned char _mpu_kill=0;
unsigned short _mpu_samplerate;
float _mpu_beta;

// Motion ISR
void (*isr_motionint)(void) = 0;

// Conversion from Gyro readings to rad/sec
#ifdef __cplusplus
float mpu_gtorps=3.14159665/180.0/131.072;
#else
_Accum mpu_gtorps=3.14159665k/180.0k/131.0k;
#endif



// 
unsigned char sample_mode;

/******************************************************************************
*******************************************************************************
MPU ISR   MPU ISR   MPU ISR   MPU ISR   MPU ISR   MPU ISR   MPU ISR   MPU ISR   
*******************************************************************************
*******************************************************************************/
void mpu_isr_o(void)	// Non-blocking SPI read triggered by this interrupt
{
	// motionint always called (e.g. WoM)
	/*if(isr_motionint!=0)
			isr_motionint();	*/
			

	// Check if we really have an interrupt (high overhead)
	unsigned char s[4];
	unsigned char r=mpu_readregs_int_try_raw(s,MPU_R_INT_STATUS,1);
	if(r)
	{
		mpu_cnt_spurious++;
		//return;
	}
	if( (s[1]&1) == 0)
	{
		mpu_cnt_spurious++;
		//return;
	}


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
			__mpu_data_packetctr_current=mpu_cnt_sample_tot;
			// Initiate readout: 3xA+3*G+1*T+3*M+Ms = 21 bytes
			// Registers start at 59d (ACCEL_XOUT_H) until 79 (EXT_SENS_DATA_06). 
			// The EXT_SENS_DATA_xx is populated from the magnetometer
			unsigned char r = mpu_readregs_int_cb(0,59,21,__mpu_read_cb);
			//unsigned char r = mpu_readregs_int_cb_raw(59,21,__mpu_read_cb);
			if(r)
			{
				mpu_cnt_sample_errbusy++;
			}
		
		}
	


	}
}
/*
	Tests indicate that blocking SPI read within this interrupt with an SPI clock at 5MHz (caveat: this is beyond the range for write to registers)
	leads to lower overhead than interrupt-driven SPI read at 920KHz.
	
	Benchmark of mpu_isr: __mpu_sample_softdivider_divider=0; __mpu_autoread=1; no check of spurious interrupt:
	134uS/call @ SPI=5529.6 KHz
	231uS/call @ SPI=1382 KHz
	300uS/call @ SPI=921.6 KHz 
	
	Benchmark of mpu_isr: __mpu_sample_softdivider_divider=0; __mpu_autoread=1; check of spurious interrupt:
	150uS/call @ SPI=5529.6 KHz
	254uS/call @ SPI=1382 KHz
	330uS/call @ SPI=921.6 KHz 
	
	Benchmark of mpu_readregs_int_try_raw only:
	87uS/call @ SPI=5529.6 KHz
	266uS/call @ SPI=921.6 KHz 
	
	Choice: use check of spurious interrupts and overclock to 1382 KHz.
	
*/
void mpu_isr(void)	// Blocking SPI read within this interrupt
{
	//static signed short mxo=0,myo=0,mzo=0;

	// motionint always called (e.g. WoM)
	/*if(isr_motionint!=0)
			isr_motionint();	*/
			
	// Check if we really have an interrupt (high overhead)
	// Experimentally, this seems unnecessary if the PCIF interrupt is cleared after the mpu_isr returns.
	
	
	unsigned char s[4];
	unsigned char r=mpu_readregs_int_try_raw(s,MPU_R_INT_STATUS,1);
	if(r)
	{
		mpu_cnt_spurious++;
		return;
	}
	if( (s[1]&1) == 0)
	{
		mpu_cnt_spurious++;
		return;
	}
	
	
	
	// Statistics
	mpu_cnt_int++;

	#if HWVER!=9
	// HW9+ implements the softdivider by means of a timer/counter
	__mpu_sample_softdivider_ctr++;
	if(__mpu_sample_softdivider_ctr>__mpu_sample_softdivider_divider)
	{
		__mpu_sample_softdivider_ctr=0;
	#endif

		// Statistics
		mpu_cnt_sample_tot++;
		
			
		// Automatically read data
		if(__mpu_autoread)
		{
			__mpu_data_packetctr_current=mpu_cnt_sample_tot;
			// Initiate readout: 3xA+3*G+1*T+3*M+Ms = 21 bytes
			// Registers start at 59d (ACCEL_XOUT_H) until 79 (EXT_SENS_DATA_06). 
			// The EXT_SENS_DATA_xx is populated from the magnetometer
			unsigned char spibuf[32];
			unsigned char r = mpu_readregs_int_try_raw(spibuf,59,21);
			if(r)
			{
				mpu_cnt_sample_errbusy++;
				return;
			}
			// Discard oldest data and store new one
			if(mpu_data_isfull())
			{
				_mpu_data_rdnext();
				mpu_cnt_sample_errfull++;
				mpu_cnt_sample_succcess--;		// This plays with the increment of mpu_cnt_sample_succcess on the last line of this function; i.e. mpu_cnt_sample_succcess does not change.
			}
			
			// Pointer to memory structure
			MPUMOTIONDATA *mdata = &mpu_data[mpu_data_wrptr];
			
			mdata->time=timer_ms_get();											
			
			//__mpu_copy_spibuf_to_mpumotiondata_asm(spibuf+1,mdata);			// Copy and conver the spi buffer to MPUMOTIONDATA; if this function is used, the correction must be manually done as below.
			//__mpu_copy_spibuf_to_mpumotiondata_magcor_asm(spibuf+1,mdata);		// Copy and conver the spi buffer to MPUMOTIONDATA including changing the magnetic coordinate system (mx <= -my; my<= -mx) (Dan's version)
			__mpu_copy_spibuf_to_mpumotiondata_magcor_asm_mathias(spibuf+1,mdata);		// Copy and conver the spi buffer to MPUMOTIONDATA including changing the magnetic coordinate system (mx <= my; my<= mx; mz<=-mz) (Mathias's version)
			
			
			// Alternative to __mpu_copy_spibuf_to_mpumotiondata_magcor_asm: manual change
			/*signed t = mdata->mx;
			mdata->mx=-mdata->my;
			mdata->my=-t;*/
			
			
			// Mathias - changes where is the yaw=0 by 180 (not needed if __mpu_copy_spibuf_to_mpumotiondata_magcor_asm_mathias is used)
			/*signed t = mdata->mx;
			mdata->mx=mdata->my;
			mdata->my=t;
			mdata->mz=-mdata->mz;*/
			
			//mdata->mz=0;
			
			/*mdata->gx=0;
			mdata->gy=0;
			mdata->gz=0;*/
			
			//mdata->mx=0;
			//mdata->my=0;
			//mdata->mz=0;
			
			
			
			
		
		
			
			
			mdata->packetctr=__mpu_data_packetctr_current;
			
			// correct the magnetometer
			if(_mpu_mag_correctionmode==1)
				//mpu_mag_correct1(mdata->mx,mdata->my,mdata->mz,&mdata->mx,&mdata->my,&mdata->mz);		// This call to be used with __mpu_copy_spibuf_to_mpumotiondata_asm
				mpu_mag_correct1(mdata->my,mdata->mx,mdata->mz,&mdata->my,&mdata->mx,&mdata->mz);		// This call to be used with __mpu_copy_spibuf_to_mpumotiondata_magcor_asm: swap mx and my to ensure the right ASA coefficients are applied
			if(_mpu_mag_correctionmode==2)
				mpu_mag_correct2_inplace(&mdata->mx,&mdata->my,&mdata->mz);								// Call identical regardless of __mpu_copy_spibuf_to_mpumotiondata_asm or __mpu_copy_spibuf_to_mpumotiondata_magcor_asm as calibration routine uses corrected coordinate system.
						

			// Implement the channel kill
			if(_mpu_kill&1)
			{
				mdata->mx=mdata->my=mdata->mz=0;
			}
			if(_mpu_kill&2)
			{
				mdata->gx=mdata->gy=mdata->gz=0;
			}
			if(_mpu_kill&4)
			{
				mdata->ax=mdata->ay=mdata->az=0;
			}		
			
			// Magnetic filter
			
			/*mdata->mx = (mdata->mx+7*mxo)/8;
			mdata->my = (mdata->my+7*myo)/8;
			mdata->mz = (mdata->mz+7*mzo)/8;
			mxo = mdata->mx;
			myo = mdata->my;
			mzo = mdata->mz;*/
			
			


			// Next buffer	
			_mpu_data_wrnext();	
			
			// Statistics
			mpu_cnt_sample_succcess++;
		
		}
	#if HWVER!=9
	} // __mpu_sample_softdivider_ctr
	#endif
}

/******************************************************************************
	function: mpu_clearstat
*******************************************************************************	
	Clear the MPU acquisition statistics.
	
	Returns:
		-
*******************************************************************************/
void mpu_clearstat(void)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		mpu_cnt_int=0;
		mpu_cnt_sample_tot=0;
		mpu_cnt_sample_succcess=0;
		mpu_cnt_sample_errbusy=0;
		mpu_cnt_sample_errfull=0;
		mpu_cnt_spurious=0;
	}
}
/******************************************************************************
	function: mpu_getstat
*******************************************************************************	
	Get the MPU acquisition statistics.

	All parameters are pointer to a variable holding the return values. 
	If the pointer is null, the corresponding parameter is not returned.
	
	Parameters:
		cnt_int					-	Number of MPU ISR calls
		cnt_sample_tot			-	Number of MPU samples so far, regardless of successfull or not
		cnt_sample_succcess		-	Number of successful MPU samples acquired
		cnt_sample_errbusy		-	Number of unsuccessful MPU sample acquisition due to the MPU interface being busy
		cnt_sample_errfull		-	Number of unsuccessful MPU samples acquisition due to the MPU sample buffer being full
	
	Returns:
		-
*******************************************************************************/
void mpu_getstat(unsigned long *cnt_int, unsigned long *cnt_sample_tot, unsigned long *cnt_sample_succcess, unsigned long *cnt_sample_errbusy, unsigned long *cnt_sample_errfull)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		if(cnt_int)
			*cnt_int=mpu_cnt_int;
		if(cnt_sample_tot)
			*cnt_sample_tot=mpu_cnt_sample_tot;
		if(cnt_sample_succcess)
			*cnt_sample_succcess=mpu_cnt_sample_succcess;
		if(cnt_sample_errbusy)
			*cnt_sample_errbusy=mpu_cnt_sample_errbusy;
		if(cnt_sample_errfull)
			*cnt_sample_errfull=mpu_cnt_sample_errfull;
	}
}

/******************************************************************************
	function: mpu_clearbuffer
*******************************************************************************	
	Clears all sensor data held in the MPU buffers.
	
	Parameters:
		-
	
	Returns:
		-
*******************************************************************************/
void mpu_clearbuffer(void)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		mpu_data_rdptr=mpu_data_wrptr=0;
	}	
}

void _mpu_enableautoread(void)
{
	_mpu_disableautoread();		// Temporarily disable the interrupts to allow clearing the old statistics+buffer
	_delay_ms(1);				// Wait that the last potential interrupt transfer completes
	// Clear the software divider counter
	__mpu_sample_softdivider_ctr=0;
	// Clear statistics counters
	mpu_clearstat();	
	// Clear data buffers
	mpu_clearbuffer();
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
		_mpu_data_rdnext();
		mpu_cnt_sample_errfull++;
		mpu_cnt_sample_succcess--;		// This plays with the increment of mpu_cnt_sample_succcess on the last line of this function; i.e. mpu_cnt_sample_succcess does not change.
	}
	
	// Pointer to memory structure
	MPUMOTIONDATA *mdata = &mpu_data[mpu_data_wrptr];
	
	//__mpu_copy_spibuf_to_mpumotiondata_asm(_mpu_tmp_reg,mdata);		// Copy and conver the spi buffer to MPUMOTIONDATA
	__mpu_copy_spibuf_to_mpumotiondata_magcor_asm(_mpu_tmp_reg,mdata);	// Copy and conver the spi buffer to MPUMOTIONDATA including changing the magnetic coordinate system (mx <= -my; my<= -mx)
	/*
	// Alternative to __mpu_copy_spibuf_to_mpumotiondata_magcor_asm: manual change
	signed t = mdata->mx;
	mdata->mx=-mdata->my;
	mdata->my=-t;*/
	mdata->time=timer_ms_get();										// Fill remaining fields
	mdata->packetctr=__mpu_data_packetctr_current;
	
	// correct the magnetometer
	if(_mpu_mag_correctionmode==1)
		//mpu_mag_correct1(mdata->mx,mdata->my,mdata->mz,&mdata->mx,&mdata->my,&mdata->mz);		// This call to be used with __mpu_copy_spibuf_to_mpumotiondata_asm
		mpu_mag_correct1(mdata->my,mdata->mx,mdata->mz,&mdata->my,&mdata->mx,&mdata->mz);		// This call to be used with __mpu_copy_spibuf_to_mpumotiondata_magcor_asm: swap mx and my to ensure the right ASA coefficients are applied
	if(_mpu_mag_correctionmode==2)
		mpu_mag_correct2_inplace(&mdata->mx,&mdata->my,&mdata->mz);								// Call identical regardless of __mpu_copy_spibuf_to_mpumotiondata_asm or __mpu_copy_spibuf_to_mpumotiondata_magcor_asm as calibration routine uses corrected coordinate system.
	
	// Implement the channel kill
	if(_mpu_kill&1)
	{
		mdata->mx=mdata->my=mdata->mz=0;
	}
	if(_mpu_kill&2)
	{
		mdata->gx=mdata->gy=mdata->gz=0;
	}
	if(_mpu_kill&4)
	{
		mdata->ax=mdata->ay=mdata->az=0;
	}		
	

			
	// Next buffer	
	_mpu_data_wrnext();	
	
	// Statistics
	mpu_cnt_sample_succcess++;
	
	//_mpu_ongoing=0;		// Required when using mpu_readregs_int_cb_raw otherwise no further transactions possible
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
	Returns how many samples are in the buffer, when automatic read is active.
*******************************************************************************/
unsigned char mpu_data_level(void)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		return (mpu_data_wrptr-mpu_data_rdptr)&(MPU_MOTIONBUFFERSIZE-1);
	}
	return 0;	// To avoid compiler warning
}
/******************************************************************************
	function: mpu_data_getnext_raw
*******************************************************************************	
	Returns the next data in the buffer, when automatic read is active and data
	is available.	
	This function returns raw reads, without applying the calibration to take into 
	account the accelerometer and gyroscope scale.
	
	This function removes the data from the automatic read buffer and the next call 
	to this function will return the next available data.
	
	If no data is available, the function returns an error.
	
	Returns:
		0	-	Success
		1	-	Error (no data available in the buffer)
*******************************************************************************/
unsigned char mpu_data_getnext_raw(MPUMOTIONDATA &data)
{
	//return (mpu_data_wrptr-mpu_data_rdptr)&(MPU_MOTIONBUFFERSIZE-1);
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		// Check if buffer is empty
		if(mpu_data_wrptr==mpu_data_rdptr)
			return 1;
		// Copy the data
		//memcpy((void*)&data,(void*)&mpu_data[mpu_data_rdptr],sizeof(MPUMOTIONDATA));
		data = *(MPUMOTIONDATA*)&mpu_data[mpu_data_rdptr];
		//data = mpu_data[mpu_data_rdptr];
		// Increment the read pointer
		_mpu_data_rdnext();
		return 0;
	}
	return 1;	// To avoid compiler warning
}
/******************************************************************************
	function: mpu_data_getnext
*******************************************************************************	
	Returns the next raw and geometry data, when automatic read is active and data
	is available.
	This function returns the raw reads and also computes the geometry (e.g. quaternions)
	from the raw data.
	
	This function removes the data from the automatic read buffer and the next call 
	to this function will return the next available data.
	
	If no data is available, the function returns an error.
	
	Returns:
		0	-	Success
		1	-	Error (no data available in the buffer)
*******************************************************************************/
unsigned char mpu_data_getnext(MPUMOTIONDATA &data,MPUMOTIONGEOMETRY &geometry)
{
	//return (mpu_data_wrptr-mpu_data_rdptr)&(MPU_MOTIONBUFFERSIZE-1);
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		// Check if buffer is empty
		if(mpu_data_wrptr==mpu_data_rdptr)
			return 1;
		// Copy the data
		//memcpy((void*)&data,(void*)&mpu_data[mpu_data_rdptr],sizeof(MPUMOTIONDATA));
		data = *(MPUMOTIONDATA*)&mpu_data[mpu_data_rdptr];
		//data = mpu_data[mpu_data_rdptr];
		// Increment the read pointer
		_mpu_data_rdnext();
	}
	// Compute the geometry
	mpu_compute_geometry(data,geometry);
	
	return 0;
}


/******************************************************************************
	_mpu_data_wrnext
*******************************************************************************	
	Advances the write pointer. Do not call if the buffer is full.
*******************************************************************************/
void _mpu_data_wrnext(void)
{
	mpu_data_wrptr = (mpu_data_wrptr+1)&(MPU_MOTIONBUFFERSIZE-1);
}
/******************************************************************************
	function: _mpu_data_rdnext
*******************************************************************************	
	Advances the read pointer to access the next sample in the data buffer
	at index mpu_data_rdptr. 
	Do not call if the buffer is empty.
*******************************************************************************/
void _mpu_data_rdnext(void)
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
	
	Loads the non-volatile parameters from EEPROM which are: 
	- magnetic correction mode and user magnetic correction coefficient
	- sensitivity of the accelerometer
	- sensitivity of the gyroscope
	
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
	// Optionally print register content
	mpu_printreg(file_pri);	
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
	fprintf_P(file_pri,PSTR("Mag ASA: %02X %02X %02X\n"),_mpu_mag_asa[0],_mpu_mag_asa[1],_mpu_mag_asa[2]);
	//system_led_set(0b101); _delay_ms(800);
	mpu_mag_loadcalib();	
	//system_led_set(0b100); _delay_ms(800);
	mpu_mag_printcalib(file_pri);
	// Load the non-volatile acceleration and gyroscope scales
	unsigned char scale;
	scale = mpu_LoadAccScale();
	mpu_setaccscale(scale);
	fprintf_P(file_pri,PSTR("%sAcc scale: %d\n"),_str_mpu,scale);
	scale = mpu_LoadGyroScale()&0b11;
	mpu_setgyroscale(scale);
	fprintf_P(file_pri,PSTR("%sGyro scale: %d\n"),_str_mpu,scale);
	// Load beta
	mpu_LoadBeta();
	fprintf_P(file_pri,PSTR("%sBeta: %f\n"),_str_mpu,_mpu_beta);
	// Dump status
	//system_led_set(0b010); _delay_ms(800);
	//mpu_printregdesc(file_pri);	
	// Turn off
	//system_led_set(0b011); _delay_ms(800);
	mpu_config_motionmode(MPU_MODE_OFF,0);
	// Clear possible auto-acquire buffers, if mpu_init is called multiple times.
	mpu_clearbuffer();	
	// Clear statistics
	mpu_clearstat();
	// Don't kill any channel
	_mpu_kill=0;
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
	
	//printf_P(PSTR("mpu_mode_accgyro: %d %d %d %d %d %d\n"),gdlpe,gdlpoffhbw,gdlpbw,adlpe,adlpbw,divider);
	
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
	gdlpoffhbw: 	when DLP off, 1 to set high bandwidth (8800Hz) or 0 to set low bandwidth (3600Hz)
	gdlpbw:     	when DLP on, set the DLP low pass filter. 
					Possible values: MPU_GYR_LPF_250, 184, 92, 41, 20, 10, 5, 3600
	divider:    	divide the output of the DLP filter block by 1/(1+divider)
	
	The output data rate is as follows:
	           32KHz when gdlpe=0
	           8KHz when gdlpe=1 and gdlpbw=MPU_GYR_LPF_250 or MPU_GYR_LPF_3600
	           1KHz/(1+divider) when gdlp=1 and gdlpbw<=MPU_GYR_LPF_184<=MPU_GYR_LPF_5
******************************************************************************/
void mpu_mode_gyro(unsigned char gdlpe,unsigned char gdlpoffhbw,unsigned char gdlpbw,unsigned char divider)
{
	unsigned char conf,gconf;
	unsigned char gfchoice_b;
	
	//printf_P(PSTR("mpu_mode_gyro: %d %d %d %d\n"),gdlpe,gdlpoffhbw,gdlpbw,divider);
	
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
	//printf_P(PSTR("mpu_mode_acc: %d %d %d\n"),dlpenable,dlpbw,divider);
	
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
	//printf_P(PSTR("mpu_mode_lpacc: %d\n"),lpodr);
	
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
	_mpu_mag_mode(0,0);
	
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
	
	If the MPU is off it is turned on and then back off to get the parameter.
	
	Parameters:
		scale	-	One of MPU_GYR_SCALE_250, MPU_GYR_SCALE_500, MPU_GYR_SCALE_1000 or MPU_GYR_SCALE_2000
******************************************************************************/
void mpu_setgyroscale(unsigned char scale)
{
	scale&=0b11;					// Sanitise	
	unsigned char oldmode,oldautoread;
	
	//printf("set gyro scale\n");
	
	// Save mode
	oldmode=mpu_get_motionmode(&oldautoread);
	mpu_config_motionmode(MPU_MODE_100HZ_ACC_BW41_GYRO_BW41_MAG_8,0);
	
	unsigned char gconf = mpu_readreg(MPU_R_GYROCONFIG);	
	mpu_writereg(MPU_R_GYROCONFIG,(gconf&0b11100111)|(scale<<3));
	
	switch(scale)
	{
		case MPU_GYR_SCALE_250:
			#ifdef __cplusplus
			mpu_gtorps=3.14159665/180.0/131.072;
			#else
			mpu_gtorps=3.14159665k/180.0k/131.072k;
			#endif			
			break;
		case MPU_GYR_SCALE_500:
			#ifdef __cplusplus
			mpu_gtorps=3.14159665/180.0/65.536;
			#else
			mpu_gtorps=3.14159665k/180.0k/65.536k;
			#endif
			break;
		case MPU_GYR_SCALE_1000:
			#ifdef __cplusplus
			mpu_gtorps=3.14159665/180.0/32.768;
			#else
			mpu_gtorps=3.14159665k/180.0k/32.768k;
			#endif
			break;
		default:
			#ifdef __cplusplus
			mpu_gtorps=3.14159665/180.0/16.384;
			#else
			mpu_gtorps=3.14159665k/180.0k/16.384k;
			#endif
			break;
	}
	// Restore
	mpu_config_motionmode(oldmode,oldautoread);
}
/******************************************************************************
	Function: mpu_getgyroscale
*******************************************************************************	
	Sets the gyro scale.
	
	If the MPU is off it is turned on and then back off to get the parameter.
	
	Returns:
		One of MPU_GYR_SCALE_250, MPU_GYR_SCALE_500, MPU_GYR_SCALE_1000 or MPU_GYR_SCALE_2000
******************************************************************************/
unsigned char mpu_getgyroscale(void)
{
	unsigned char scale;
	unsigned char oldmode,oldautoread;
	
	//printf("get gyro scale\n");
	
	// Save mode
	oldmode=mpu_get_motionmode(&oldautoread);
	mpu_config_motionmode(MPU_MODE_100HZ_ACC_BW41_GYRO_BW41_MAG_8,0);
	
	unsigned char gconf = mpu_readreg(MPU_R_GYROCONFIG);	
	scale = (gconf>>3)&0b11;
	
	// Restore
	mpu_config_motionmode(oldmode,oldautoread);
	
	return scale;
}

/******************************************************************************
	Function: mpu_setaccscale
*******************************************************************************	
	Sets the accelerometer scale.
	
	If the MPU is off it is turned on and then back off to set the parameter.
	
	Parameters:
		scale	-	One of MPU_ACC_SCALE_2, MPU_ACC_SCALE_4, MPU_ACC_SCALE_8, MPU_ACC_SCALE_16
******************************************************************************/
void mpu_setaccscale(unsigned char scale)
{
	scale&=0b11;					// Sanitise
	unsigned char oldmode,oldautoread;
	
	//printf("set acc scale\n");
	
	// Save mode
	oldmode=mpu_get_motionmode(&oldautoread);
	mpu_config_motionmode(MPU_MODE_100HZ_ACC_BW41_GYRO_BW41_MAG_8,0);

	// Set the parameter
	unsigned char aconf = mpu_readreg(MPU_R_ACCELCONFIG);	
	aconf=(aconf&0b11100111)|(scale<<3);
	mpu_writereg(MPU_R_ACCELCONFIG,aconf);
	
	// Restore
	mpu_config_motionmode(oldmode,oldautoread);
	
	/*
	// Must complete code for acceleration
	switch(scale)
	{
		case MPU_ACC_SCALE_2:
			//mpu_gtorps=3.14159665k/180.0k/131.072k;
			break;
		case MPU_ACC_SCALE_4:
			//mpu_gtorps=3.14159665k/180.0k/65.536k;
			break;
		case MPU_ACC_SCALE_8:
			//mpu_gtorps=3.14159665k/180.0k/32.768k;
			break;
		default:
			//mpu_gtorps=3.14159665k/180.0k/16.384k;
			break;
	}*/
}
/******************************************************************************
	Function: mpu_getaccscale
*******************************************************************************	
	Gets the acceleromter scale.
	
	If the MPU is off it is turned on and then back off to get the parameter.
	
	Returns:
		One of MPU_ACC_SCALE_2, MPU_ACC_SCALE_4, MPU_ACC_SCALE_8, MPU_ACC_SCALE_16
******************************************************************************/
unsigned char mpu_getaccscale(void)
{
	unsigned char scale;
	unsigned char oldmode,oldautoread;
	
	//printf("get acc scale\n");
	
	// Save mode
	oldmode=mpu_get_motionmode(&oldautoread);
	mpu_config_motionmode(MPU_MODE_100HZ_ACC_BW41_GYRO_BW41_MAG_8,0);
	
	//printf("turn on\n");
	mpu_config_motionmode(MPU_MODE_100HZ_ACC_BW41_GYRO_BW41_MAG_8,0);
		
	unsigned char aconf = mpu_readreg(MPU_R_ACCELCONFIG);	
	scale = (aconf>>3)&0b11;
	
	//printf("aconf: %02X\n",aconf);
	
	// Restore
	mpu_config_motionmode(oldmode,oldautoread);
		
	return scale;
}
/******************************************************************************
	Function: mpu_setandstoreaccscale
*******************************************************************************	
	Sets the accelerometer scale and store the parameter in non-volatile memory.
	
	If the MPU is off it is turned on and then back off to set the parameter.
	
	Parameters:
		scale	-	One of MPU_ACC_SCALE_2, MPU_ACC_SCALE_4, MPU_ACC_SCALE_8, MPU_ACC_SCALE_16
******************************************************************************/
void mpu_setandstoreaccscale(unsigned char scale)
{
	mpu_setaccscale(scale);
	eeprom_write_byte((uint8_t*)CONFIG_ADDR_ACC_SCALE,scale&0b11);
}
/******************************************************************************
	Function: mpu_setandstoregyroscale
*******************************************************************************	
	Sets the gyro scale and store the parameter in non-volatile memory.
	
	If the MPU is off it is turned on and then back off to set the parameter.
	
	Parameters:
		scale	-	One of MPU_ACC_SCALE_2, MPU_ACC_SCALE_4, MPU_ACC_SCALE_8, MPU_ACC_SCALE_16
******************************************************************************/
void mpu_setandstoregyrocale(unsigned char scale)
{
	mpu_setgyroscale(scale);
	eeprom_write_byte((uint8_t*)CONFIG_ADDR_GYRO_SCALE,scale&0b11);
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
	function: mpu_getwhoami
*******************************************************************************	
	Returns MPU WHOAMI register
	
	Parameters:
		-	
	Returns
		Who am I value, 0x71 for the MPU9250
******************************************************************************/
unsigned char mpu_getwhoami(void)
{
	return mpu_readreg(MPU_R_WHOAMI);
}
/******************************************************************************
	function: mpu_reset
*******************************************************************************	
	Resets the MPU. 
	
	According to the datasheet, all the registers are reset to 0, except register
	107 (PWR_MGMT_1) which is set to 1 (Auto select best available clock, PLL or 
	internal oscillator) and register WHO_AM_I.
	
	Parameters:
		-
	Returns:
		-
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
/******************************************************************************
	function: _mpu_acquirecalib
*******************************************************************************	
	Acquires calibration data in the FIFO.
	
	Assumes the acceleration and gyroscope sensitivity are 2G and 250dps and
	that the gyro bias registers are 0.
	
*******************************************************************************/
void _mpu_acquirecalib(unsigned char clearbias)
{
	// Enable high speed
	mpu_config_motionmode(MPU_MODE_500HZ_ACC_BW184_GYRO_BW184,0);	
	
	// Sensitivity should be 250dps and 2G
	mpu_setaccscale(MPU_ACC_SCALE_2);
	mpu_setgyroscale(MPU_GYR_SCALE_250);
	
	// Clear the gyro bias
	if(clearbias)
		mpu_setgyrobias(0,0,0);

	// The gyro takes about 40 milliseconds to stabilise from waking up; wait 60ms.
	_delay_ms(60);

	
	
	mpu_fifoenable(0b01111000,1,1);			// Set FIFO for accel+gyro, enable FIFO, reset FIFO
	//mpu_fifoenable(0b01110000,1,1);				// Set FIFO for gyro, enable FIFO, reset FIFO
	// In Acc+Gyro: output is 12 bytes per samples. FIFO can hold 42 samples. Experimentally, 78ms delay incl. overheads acquires 492 bytes (41 samples)
	// In Gyro: output is 6 bytes per samples. FIFO can hold 85 samples. Fills in 170ms at 500Hz. In 130ms expect 390 bytes.
	_delay_ms(78);
	//_delay_ms(130);
	// Keep FIFO enabled but stop logging
	mpu_fifoenable(0b00000000,1,0);
}
void mpu_calibrate(void)
{
	unsigned short n;
	unsigned char numtries=4;
	signed short ax,ay,az,gx,gy,gz;
	long acc_mean[3];
	long acc_std[3];
	
	long acc_totstd[numtries];
	long gyro_bias_best[numtries][3];

	fprintf_P(file_pri,PSTR("MPU calibration:\n"));
	
	// Repeat calibration multiple times
	for(unsigned char tries=0;tries<numtries;tries++)
	{
		// Acquire calibration clearing the bias register
		_mpu_acquirecalib(1);
		// Transfer the data into a shared buffer in order to compute standard deviation
		signed short *buffer = (signed short*)sharedbuffer;
		n = mpu_getfifocnt();
		unsigned short ns = n/12;
		fprintf_P(file_pri,PSTR(" %d: %02d spl. "),tries,n);
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
		
		// Init variables for statistics
		for(unsigned char axis=0;axis<3;axis++)
		{
			acc_mean[axis]=0;
			acc_std[axis]=0;
			gyro_bias_best[tries][axis]=0;
		}
		// Compute mean
		for(unsigned short i=0;i<ns;i++)
		{
			for(unsigned char axis=0;axis<3;axis++)
			{
				acc_mean[axis]+=buffer[i*6+axis];
				gyro_bias_best[tries][axis]+=buffer[i*6+3+axis];
			}		
		}
		for(unsigned char axis=0;axis<3;axis++)
		{
			acc_mean[axis]/=ns;
			gyro_bias_best[tries][axis]/=ns;
		}
		// Compute standard deviation
		for(unsigned short i=0;i<ns;i++)
		{
			for(unsigned char axis=0;axis<3;axis++)
				acc_std[axis]+=(buffer[i*6+axis]-acc_mean[axis])*(buffer[i*6+axis]-acc_mean[axis]);
		}
		for(unsigned char axis=0;axis<3;axis++)
			acc_std[axis]=sqrt(acc_std[axis]/(ns-1));
			
		//system_led_set(0b011); _delay_ms(800);
		//fprintf_P(file_pri,PSTR("Mean(Acc): %ld %ld %ld "),acc_mean[0],acc_mean[1],acc_mean[2]);
		//fprintf_P(file_pri,PSTR("Std(Acc): %ld %ld %ld "),acc_std[0],acc_std[1],acc_std[2]);	
		//fprintf_P(file_pri,PSTR("Mean(Gyro): %ld %ld %ld "),gyro_bias_best[tries][0],gyro_bias_best[tries][1],gyro_bias_best[tries][2]);
		for(unsigned char axis=0;axis<3;axis++)
			gyro_bias_best[tries][axis]=-gyro_bias_best[tries][axis]/4;
		
		//system_led_set(0b010); _delay_ms(800);
		fprintf_P(file_pri,PSTR("Bias: %3ld %3ld %3ld "),gyro_bias_best[tries][0],gyro_bias_best[tries][1],gyro_bias_best[tries][2]);
		
		long totstd = acc_std[0]+acc_std[1]+acc_std[2];
		acc_totstd[tries]=totstd;
		fprintf_P(file_pri,PSTR("(std: %lu)\n"),totstd);
	}
	// Find best
	long beststd = acc_totstd[0];
	unsigned char best=0;
	for(unsigned char tries=1;tries<numtries;tries++)
	{
		if(acc_totstd[tries]<beststd)
		{
			beststd=acc_totstd[tries];
			best=tries;
		}
	}
	mpu_setgyrobias(gyro_bias_best[best][0],gyro_bias_best[best][1],gyro_bias_best[best][2]);
	fprintf_P(file_pri,PSTR(" Calibrated with bias: %ld %ld %ld\n"),gyro_bias_best[best][0],gyro_bias_best[best][1],gyro_bias_best[best][2]);
	if(beststd>200)
	{
		fprintf_P(file_pri,PSTR(" **TOO MUCH MOVEMENT-RISK OF MISCALIBRATION**\n"));
	}
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
		mpu_writereg(MPU_R_I2C_MST_CTRL,0b00000000);		// Clear MULT_MST_EN, clear WAIT_FOR_ES. I2C 348KHz. TODO: check if needed.		(Default)
		//mpu_writereg(MPU_R_I2C_MST_CTRL,0b00001001);		// Clear MULT_MST_EN, clear WAIT_FOR_ES. I2C 500KHz. TODO: check if needed.		(Increasing the I2C clock does not seem to improve/change anything in the behaviour)
		unsigned char usr = mpu_readreg(MPU_R_USR_CTRL);
		mpu_writereg(MPU_R_USR_CTRL,usr&0b11011111);		// Clears I2C_MST_EN		
	}
}


/******************************************************************************
	function: _mpu_mag_mode
*******************************************************************************	
	Enables or disable the conversion of the magnetometer sensors and.
	enables the shadowing of the magnetometer registers.

	This function must only be called if the communication interface is 
	enabled (mpu_mag_interfaceenable) and if the motion processor is active.
	Otherwise, timeout occur.
	
	Note: do not use in user code.
		
	Parameters:
		en		-	0: sleep mode. 1: 8Hz conversion. 2: 100Hz conversion.
		magdiv	-	Shadowing frequency divider; shadows register at ODR/(1+magdiv)
	
*******************************************************************************/
void _mpu_mag_mode(unsigned char mode,unsigned char magdiv)
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
			mpu_mag_writereg(0x0a,0b00010010);		// Continuous mode 1 (8Hz conversion), 16 bit
			_mpu_mag_regshadow(1,magdiv,3,7);		// Start shadowing
			break;
		case 2:
		default:
			mpu_mag_writereg(0x0a,0b00010110);		// Continuous mode 2 (100Hz conversion), 16 bit
			_mpu_mag_regshadow(1,magdiv,3,7);		// Start shadowing
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
	
	*Magnetometer read rate*
	
	The magnetometer is read at the output data rate ODR/(1+dly), where dly is 
	configurable. This is independent of the magnetometer ADC which is 8Hz or 100 Hz.
	
	When the MPU is in some modes with high internal ODR (e.g. 8KHz with the 
	gyro bandwidth is 250Hz) then the dly parameter should be set to higher values,
	otherwise the internal I2C interface to the magnetometer will block the acquisition
	of the gyro and acceleration data, leading to an effective sample rate lower than desired.
	
	Note that the ODR is the sample rate (1KHz) divided by the acc/gyro sample rate divider (except
	in the gyro 250BW mode, where the ODR is 8KHz).
	
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


/******************************************************************************
	function: mpu_mag_correct1
*******************************************************************************	
	Magnetic correction using factory parameters in fuse rom.
	
	                                (ASA-128)x0.5
	The correction is: Hadj = H x (--------------- + 1)
	                                    128
	
	With ASA=128 the output is equal to the input. 
	Values of ASA higher than 128 imply |Hadj|>|H| (sign is preserved). 
	Values of ASA lower than 128 imply |Hadj|<|H| (sign in preserved).
*******************************************************************************/

void mpu_mag_correct1(signed short mx,signed short my,signed short mz,volatile signed short *mx2,volatile signed short *my2,volatile signed short *mz2)
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
/******************************************************************************
	function: mpu_mag_correct2
*******************************************************************************	
	Magnetic correction using bias/sensitivity parameters found during calibration.
	
	The sensitivity is a N.7 fixed point number.
	
	Code size is smallest with /127, and >>7 is smaller than /127:
		/256 123250
		/128 123236
		/64 123248
		/32 123260
		/16 123248
		
		>>8 123196
		>>7 123192
		>>6 123216
	
*******************************************************************************/
void mpu_mag_correct2(signed short mx,signed short my,signed short mz,signed short *mx2,signed short *my2,signed short *mz2)
{
	// For correct rounding a better version would do (expr +64)/128
	*mx2=(mx+_mpu_mag_bias[0])*_mpu_mag_sens[0]/128;
	*my2=(my+_mpu_mag_bias[1])*_mpu_mag_sens[1]/128;
	*mz2=(mz+_mpu_mag_bias[2])*_mpu_mag_sens[2]/128;
}
void mpu_mag_correct2_inplace(signed short *mx,signed short *my,signed short *mz)
{
	
	*mx=(*mx+_mpu_mag_bias[0])*_mpu_mag_sens[0]/128;
	*my=(*my+_mpu_mag_bias[1])*_mpu_mag_sens[1]/128;
	*mz=(*mz+_mpu_mag_bias[2])*_mpu_mag_sens[2]/128;
}
void mpu_mag_correct2b(signed short mx,signed short my,signed short mz,signed short *mx2,signed short *my2,signed short *mz2)
{
	
	/**mx2=(mx+_mpu_mag_bias[0])*_mpu_mag_sens[0]/128;
	*my2=(my+_mpu_mag_bias[1])*_mpu_mag_sens[1]/128;
	*mz2=(mz+_mpu_mag_bias[2])*_mpu_mag_sens[2]/128;*/
	
	*mx2=((mx+_mpu_mag_bias[0])*_mpu_mag_sens[0]+64)>>7;
	*my2=((my+_mpu_mag_bias[1])*_mpu_mag_sens[1]+64)>>7;
	*mz2=((mz+_mpu_mag_bias[2])*_mpu_mag_sens[2]+64)>>7;
	
	/**mx2=((mx+_mpu_mag_bias[0])*_mpu_mag_sens[0])>>7;
	*my2=((my+_mpu_mag_bias[1])*_mpu_mag_sens[1])>>7;
	*mz2=((mz+_mpu_mag_bias[2])*_mpu_mag_sens[2])>>7;*/
}
void mpu_mag_correct2c(signed short mx,signed short my,signed short mz,signed short *mx2,signed short *my2,signed short *mz2)
{
	
	/**mx2=(mx+_mpu_mag_bias[0])*_mpu_mag_sens[0]/128;
	*my2=(my+_mpu_mag_bias[1])*_mpu_mag_sens[1]/128;
	*mz2=(mz+_mpu_mag_bias[2])*_mpu_mag_sens[2]/128;*/
	
	/**mx2=((mx+_mpu_mag_bias[0])*_mpu_mag_sens[0]+64)>>7;
	*my2=((my+_mpu_mag_bias[1])*_mpu_mag_sens[1]+64)>>7;
	*mz2=((mz+_mpu_mag_bias[2])*_mpu_mag_sens[2]+64)>>7;*/
	
	*mx2=((mx+_mpu_mag_bias[0])*_mpu_mag_sens[0])>>7;
	*my2=((my+_mpu_mag_bias[1])*_mpu_mag_sens[1])>>7;
	*mz2=((mz+_mpu_mag_bias[2])*_mpu_mag_sens[2])>>7;
}
/******************************************************************************
	function: mpu_mag_calibrate
*******************************************************************************	
	Find the magnetometer calibration coefficients.

	Finds a bias term to add to the magnetic data to ensure zero average.
	Finds a sensitivity term to multiply the zero-average magnetic data to span the range [-128,128]
	The sensitivity is a N.7 fixed-point number to multiply the integer magnetic field.
	
	The earth magnetic field is up to ~0.65 Gauss=65 uT. The sensitivity of the AK8963 is .15uT/LSB. 
	Hence the maximum readout is: 65uT/.15uT=433LSB. 
	Therefore assume readout < 450 for earth magnetic field.
	
	The maximum value of sens is 128*128=16384.
		
	Parameters:
		-
	Returns:
		Stores calibration coefficients in _mpu_mag_sens
*******************************************************************************/
void mpu_mag_calibrate(void)
{
	signed m[3];
	WAITPERIOD p=0;
	unsigned long t1;
	MPUMOTIONDATA data;

	// Deactivate the automatic correction
	unsigned char _mpu_mag_correctionmode_back = _mpu_mag_correctionmode;
	_mpu_mag_correctionmode=0;
	
	// Activate a magnetic mode
	mpu_config_motionmode(MPU_MODE_100HZ_ACC_BW41_GYRO_BW41_MAG_100,1);
	
	_mpu_mag_calib_max[0]=_mpu_mag_calib_max[1]=_mpu_mag_calib_max[2]=-32768;
	_mpu_mag_calib_min[0]=_mpu_mag_calib_min[1]=_mpu_mag_calib_min[2]=+32767;

	fprintf_P(file_pri,PSTR("Magnetometer calibration: far from any metal, move the sensor in all orientations until no new numbers appear on screen and then press a key\n"));

	t1=timer_ms_get();
	while(1)
	{
		if( fgetc(file_pri) != -1)
			break;
		timer_waitperiod_ms(10,&p);

		if(mpu_data_getnext_raw(data))
			continue;
			
		m[0] = data.mx;
		m[1] = data.my;
		m[2] = data.mz;
		
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
			fprintf_P(file_pri,PSTR("[%d %d %d] - [%d %d %d]\n"),_mpu_mag_calib_min[0],_mpu_mag_calib_min[1],_mpu_mag_calib_min[2],_mpu_mag_calib_max[0],_mpu_mag_calib_max[1],_mpu_mag_calib_max[2]);
		}
		if(timer_ms_get()-t1>1000)
		{
			fprintf_P(file_pri,PSTR("%d %d %d\n"),m[0],m[1],m[2]);
			t1=timer_ms_get();
		}
	}

	// Restore the automatic correction
	_mpu_mag_correctionmode=_mpu_mag_correctionmode_back;

	mpu_config_motionmode(MPU_MODE_OFF,0);
	
	// compute the calibration coefficients
	for(unsigned char i=0;i<3;i++)
	{
		// Bias: term added to magnetic data to ensure zero mean
		_mpu_mag_bias[i] = -(_mpu_mag_calib_max[i]+_mpu_mag_calib_min[i])/2;
		// Sensitivity: N.7 number to obtain a range [-256;256]
		if(_mpu_mag_calib_max[i]-_mpu_mag_calib_min[i]==0)
			_mpu_mag_sens[i]=1;
		else
			_mpu_mag_sens[i] = (256*128l)/(_mpu_mag_calib_max[i]-_mpu_mag_calib_min[i]);
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
	if(_mpu_mag_correctionmode>2) _mpu_mag_correctionmode=2;			// Sanitise: must be 0, 1, 2
}
void mpu_mag_correctionmode(unsigned char mode)
{
	_mpu_mag_correctionmode=mode;
	eeprom_write_byte((uint8_t*)(CONFIG_ADDR_MAG_CORMOD),mode);
}
unsigned char mpu_LoadAccScale(void)
{
	return eeprom_read_byte((uint8_t*)CONFIG_ADDR_ACC_SCALE)&0b11;
}
unsigned char mpu_LoadGyroScale(void)
{
	return eeprom_read_byte((uint8_t*)CONFIG_ADDR_GYRO_SCALE)&0b11;
}
void mpu_LoadBeta(void)
{
	float beta;
	unsigned long b = eeprom_read_dword((uint32_t*)CONFIG_ADDR_BETA);
	if(b==4294967295l)
	{
		mpu_StoreBeta(0.35);
		beta=0.35;
	}
	else
	{
		beta=*((float*)&b);
	}
	_mpu_beta = beta;
}
void mpu_StoreBeta(float beta)
{
	unsigned long b;	
	b=*((unsigned long*)&beta);	
	eeprom_write_dword((uint32_t*)CONFIG_ADDR_BETA,b);	
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
	fprintf_P(f,PSTR("Mag bias: %d %d %d\n"),_mpu_mag_bias[0],_mpu_mag_bias[1],_mpu_mag_bias[2]);
	fprintf_P(f,PSTR("Mag sens: %d %d %d\n"),_mpu_mag_sens[0],_mpu_mag_sens[1],_mpu_mag_sens[2]);
	fprintf_P(f,PSTR("Mag corrmode: %d\n"),_mpu_mag_correctionmode);
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
	fprintf_P(file,PSTR(" Errors: MPU I/O busy=%lu buffer=%lu\n"),mpu_cnt_sample_errbusy,mpu_cnt_sample_errfull);
	fprintf_P(file,PSTR(" Buffer level: %u/%u\n"),mpu_data_level(),MPU_MOTIONBUFFERSIZE);
	fprintf_P(file,PSTR(" Spurious ISR: %lu\n"),mpu_cnt_spurious);
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



// Try to speedup the conversion from the spi buffer to motiondata
// In benchmarks __mpu_copy_spibuf_to_mpumotiondata_1 is faster than __mpu_copy_spibuf_to_mpumotiondata_2 which is faster than __mpu_copy_spibuf_to_mpumotiondata_3. The asm version is 50% faster.
void __mpu_copy_spibuf_to_mpumotiondata_1(unsigned char *spibuf,unsigned char *mpumotiondata)
{
	spibuf++;				
	// Acc (big endian)
	*mpumotiondata=*spibuf;
	mpumotiondata++;
	spibuf--;
	*mpumotiondata=*spibuf;
	mpumotiondata++;
	spibuf+=3;
	
	*mpumotiondata=*spibuf;
	mpumotiondata++;
	spibuf--;
	*mpumotiondata=*spibuf;
	mpumotiondata++;
	spibuf+=3;
	
	*mpumotiondata=*spibuf;
	mpumotiondata++;
	spibuf--;
	*mpumotiondata=*spibuf;
	spibuf+=3;
	
	// Temp (big endian)
	mpumotiondata+=14;	
	*mpumotiondata=*spibuf;
	mpumotiondata++;
	spibuf--;
	*mpumotiondata=*spibuf;
	spibuf+=3;
	
	// Gyr (big endian)
	mpumotiondata-=14;	
	*mpumotiondata=*spibuf;
	mpumotiondata++;
	spibuf--;
	*mpumotiondata=*spibuf;
	mpumotiondata++;
	spibuf+=3;
	
	*mpumotiondata=*spibuf;
	mpumotiondata++;
	spibuf--;
	*mpumotiondata=*spibuf;
	mpumotiondata++;
	spibuf+=3;
	
	*mpumotiondata=*spibuf;
	mpumotiondata++;
	spibuf--;
	*mpumotiondata=*spibuf;
	mpumotiondata++;
	
	// Mag
	spibuf+=2;
	*mpumotiondata=*spibuf;
	mpumotiondata++;
	spibuf++;
	*mpumotiondata=*spibuf;
	mpumotiondata++;
	spibuf++;
	
	*mpumotiondata=*spibuf;
	mpumotiondata++;
	spibuf++;
	*mpumotiondata=*spibuf;
	mpumotiondata++;
	spibuf++;
	
	*mpumotiondata=*spibuf;
	mpumotiondata++;
	spibuf++;
	*mpumotiondata=*spibuf;
	mpumotiondata++;
	spibuf++;
	
	*mpumotiondata=*spibuf;
	mpumotiondata++;
	spibuf++;
}
void __mpu_copy_spibuf_to_mpumotiondata_2(unsigned char *spibuf,unsigned char *mpumotiondata)
{

	spibuf++;				
	// Acc (big endian)
	*mpumotiondata++=*spibuf--;
	*mpumotiondata++=*spibuf;
	spibuf+=3;
	
	*mpumotiondata++=*spibuf--;
	*mpumotiondata++=*spibuf;
	spibuf+=3;
	
	*mpumotiondata++=*spibuf--;
	*mpumotiondata=*spibuf;
	spibuf+=3;
	
	// Temp (big endian)
	mpumotiondata+=14;
	*mpumotiondata++=*spibuf--;
	*mpumotiondata=*spibuf;
	spibuf+=3;
	
	// Gyr (big endian)
	mpumotiondata-=14;	
	*mpumotiondata++=*spibuf--;
	*mpumotiondata++=*spibuf;
	spibuf+=3;
	
	*mpumotiondata++=*spibuf--;
	*mpumotiondata++=*spibuf;
	spibuf+=3;
	
	*mpumotiondata++=*spibuf--;
	*mpumotiondata++=*spibuf;

	// Mag
	spibuf+=2;
	*mpumotiondata++=*spibuf++;
	*mpumotiondata++=*spibuf++;
	
	*mpumotiondata++=*spibuf++;
	*mpumotiondata++=*spibuf++;
	
	*mpumotiondata++=*spibuf++;
	*mpumotiondata++=*spibuf++;

	*mpumotiondata++=*spibuf++;
}
void __mpu_copy_spibuf_to_mpumotiondata_3(unsigned char *spibuf,MPUMOTIONDATA *mpumotiondata)
{
	signed short ax,ay,az,gx,gy,gz,mx,my,mz,temp;
	unsigned char ms;	
	
	ax=spibuf[0]; ax<<=8; ax|=spibuf[1];
	ay=spibuf[2]; ay<<=8; ay|=spibuf[3];
	az=spibuf[4]; az<<=8; az|=spibuf[5];
	temp=spibuf[6]; temp<<=8; temp|=spibuf[7];
	gx=spibuf[8]; gx<<=8; gx|=spibuf[9];
	gy=spibuf[10]; gy<<=8; gy|=spibuf[11];
	gz=spibuf[12]; gz<<=8; gz|=spibuf[13];

	mx=spibuf[15]; mx<<=8; mx|=spibuf[14];
	my=spibuf[17]; my<<=8; my|=spibuf[16];
	mz=spibuf[19]; mz<<=8; mz|=spibuf[18];
	ms=spibuf[20];
	
	mpumotiondata->ax=ax;
	mpumotiondata->ay=ay;
	mpumotiondata->az=az;
	mpumotiondata->gx=gx;
	mpumotiondata->gy=gy;
	mpumotiondata->gz=gz;
	mpumotiondata->mx=mx;
	mpumotiondata->my=my;
	mpumotiondata->mz=mz;
	mpumotiondata->ms=ms;
	mpumotiondata->temp=temp;
}

void mpu_benchmark_isr(void)
{
	// Benchmark ISR
	unsigned long t1,t2;
	__mpu_sample_softdivider_divider=0;
	__mpu_autoread=1;
	//unsigned char spibuf[32];
	t1=timer_ms_get();
	for(unsigned i=0;i<50000;i++)
		mpu_isr();
		//unsigned char r = mpu_readregs_int_try_raw(spibuf,59,21);
	t2=timer_ms_get();
	printf("%ld\n",t2-t1);
	
	
}

/******************************************************************************
	function: mpu_kill
*******************************************************************************	
	Kills (i.e. sets to null) the sensors specified by the provided bitmap.
	
	Parameters:
		bitmap		-	3-bit bitmap indicating whether to null acc|gyr|mag (mag is LSB).
	
	Returns:
		-
	
*******************************************************************************/
void mpu_kill(unsigned char bitmap)
{
	_mpu_kill = bitmap;
}


