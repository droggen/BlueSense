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



// Predefined transactions
I2C_TRANSACTION mpu_i2c_trans_select_a;			// Select register accelerometer
I2C_TRANSACTION mpu_i2c_trans_select_g;			// Select register gyroscope
I2C_TRANSACTION mpu_i2c_trans_read_14;			// Read 14 bytes
I2C_TRANSACTION mpu_i2c_trans_read_6;				// Read 6 bytes



unsigned char mpu_readreg(unsigned char reg)
{
	unsigned char r,v;
	r = i2c_readreg(MPU_ADDRESS,reg,&v);
	if(r==0)
		return v;
	return 0xFF;
}
void mpu_readregs_burst(unsigned char *d,unsigned char reg,unsigned char n)
{
	// No burst implemented yet
	for(unsigned char i=0;i<n;i++)
	{
		d[i] = mpu_readreg(reg+i);
	}
}
/******************************************************************************
	mpu_readregs
*******************************************************************************	
	Read several MPU registers.
		
******************************************************************************/
void mpu_readregs(unsigned char *d,unsigned char reg,unsigned char n)
{
	for(unsigned char i=0;i<n;i++)
	{
		d[i] = mpu_readreg(reg+i);
	}
}


/******************************************************************************
	mpu_writereg
*******************************************************************************	
	Writes an MPU register
		
******************************************************************************/
void mpu_writereg(unsigned char reg,unsigned char v)
{
	i2c_writereg(MPU_ADDRESS,reg,v);
}


/******************************************************************************
	mpu_get_agt_int_init
*******************************************************************************	
	This MUST be called to initialise internal data structures at least once 
	prior to:
		mput_get_agt_int, mpu_get_agt_int_cb
		mpu_get_a_int, mpu_get_a_int_cb
		mpu_get_g_int, mpu_get_g_int_cb	
******************************************************************************/
void mpu_get_agt_int_init(void)
{
	// Select acceleration register transaction
	i2c_transaction_setup(&mpu_i2c_trans_select_a,MPU_ADDRESS,I2C_WRITE,0,1);
	mpu_i2c_trans_select_a.data[0]=59;													// data to send/receive
	// Select gyroscope register transaction
	i2c_transaction_setup(&mpu_i2c_trans_select_g,MPU_ADDRESS,I2C_WRITE,0,1);
	mpu_i2c_trans_select_g.data[0]=67;													// data to send/receive
	// Read agt transaction	
	i2c_transaction_setup(&mpu_i2c_trans_read_14,MPU_ADDRESS,I2C_READ,1,14);
	// Read a or g transaction	
	i2c_transaction_setup(&mpu_i2c_trans_read_6,MPU_ADDRESS,I2C_READ,1,6);

	// No ongoing transactions
	_mpu_get_agt_int_cb_numtrans=0;
}









/******************************************************************************
	mpu_getfifocnt_int
*******************************************************************************	
	Transactional interrupt-driven read, blocking until data available.
	
	Call mpu_get_agt_int_init for initialisation once prior to this.
	
	Return value:
		0:	Success
		1:	Error issuing transactions (no space available)
		2:	Error in register select
		3:	Error in register read
******************************************************************************/
unsigned short mpu_getfifocnt_int(void)
{
	unsigned short cnt;
	unsigned char d[2];
	unsigned char r;
	
	r = i2c_readregs_int(MPU_ADDRESS,114,2,d);
	if(r!=0)
	{
		printf_P(PSTR("MPU: getfifocnt error\n"));
		return 0;		
	}
	cnt=d[0];
	cnt<<=8;
	cnt+=d[1];	
	//cnt&=0b1111111111111;		// Only bits 12..0 valid
	return cnt;
}

/******************************************************************************
	mpu_get_agt_int
*******************************************************************************	
	Transactional interrupt-driven read, blocking until data available.
	
	Call mpu_get_agt_int_init for initialisation once prior to this.
	
	Return value:
		0:	Success
		1:	Error issuing transactions (no space available)
		2:	Error in register select
		3:	Error in register read
******************************************************************************/
unsigned char mpu_get_agt_int(signed short *ax,signed short *ay,signed short *az,signed short *gx,signed short *gy,signed short *gz,signed short *temp)
{
	unsigned short s1,s2,s3;
	unsigned char r;
	
	// Clear callback (mpu_get_agt_int_cb may have been called by user and callback assigned)
	mpu_i2c_trans_select_a.callback=0;
	mpu_i2c_trans_read_14.callback=0;
		
	// Queue the write/read transactions - they must be preliminarily initialised.
	r = i2c_transaction_queue(2,1,&mpu_i2c_trans_select_a,&mpu_i2c_trans_read_14);
	
	// Error
	if(r!=0)
			return 1;
	
	// Check register select error
	if(mpu_i2c_trans_select_a.status!=0)
		return 2;
	// Check register read error
	if(mpu_i2c_trans_read_14.status!=0)
		return 3;
	
	// Conversion	
	s1=mpu_i2c_trans_read_14.data[0]; s1<<=8; s1|=mpu_i2c_trans_read_14.data[1];
	s2=mpu_i2c_trans_read_14.data[2]; s2<<=8; s2|=mpu_i2c_trans_read_14.data[3];
	s3=mpu_i2c_trans_read_14.data[4]; s3<<=8; s3|=mpu_i2c_trans_read_14.data[5];
	*ax = s1; *ay = s2; *az = s3;
	s1=mpu_i2c_trans_read_14.data[6]; s1<<=8; s1|=mpu_i2c_trans_read_14.data[7];
	*temp = s1;
	s1=mpu_i2c_trans_read_14.data[8]; s1<<=8; s1|=mpu_i2c_trans_read_14.data[9];
	s2=mpu_i2c_trans_read_14.data[10]; s2<<=8; s2|=mpu_i2c_trans_read_14.data[11];
	s3=mpu_i2c_trans_read_14.data[12]; s3<<=8; s3|=mpu_i2c_trans_read_14.data[13];
	*gx = s1; *gy = s2; *gz = s3;
	
	return 0;
}

/******************************************************************************
	mpu_get_agt_int_cb
*******************************************************************************	
	Transactional interrupt-driven read, calling a callback on success or error
	
	Call mpu_get_agt_int_init for initialisation once prior to this.
	
	The user callback receives status and error.
	If status is nonzero, the transaction failed. 
	There is no distinction between	a failure to select the registers and read 
	the registers.
	
	Multiple calls to this function without waiting for the callback to be called are 
	possible.
		
	
	Return value:
		0:	Success
		1:	Error issuing transactions (no space available)
******************************************************************************/
unsigned char mpu_get_agt_int_cb(MPU_READ_CALLBACK7 cb)
{
	// Internal callbacks
	mpu_i2c_trans_select_a.callback=(void*)_mpu_i2c_trans_select7_cb;
	mpu_i2c_trans_read_14.callback=(void*)_mpu_i2c_trans_read_agt_cb;
	// Indicate user callback to internal callbacks
	mpu_i2c_trans_select_a.user = (void*)cb;
	mpu_i2c_trans_read_14.user = (void*)cb;
	// Non-blocking transaction
	
	unsigned char r = i2c_transaction_queue(2,0,&mpu_i2c_trans_select_a,&mpu_i2c_trans_read_14);		
	if(r!=0)
		return 1;
	_mpu_get_agt_int_cb_numtrans++;
	return 0;
}

/******************************************************************************
	mpu_get_a_int
*******************************************************************************	
	Transactional interrupt-driven read, blocking until data available.
	
	Call mpu_get_agt_int for initialisation once prior to this.
	
	Return value:
		0:	Success
		1:	Error issuing transactions (no space available)
		2:	Error in register select
		3:	Error in register read
******************************************************************************/
unsigned char mpu_get_a_int(signed short *ax,signed short *ay,signed short *az)
{
	unsigned short s1,s2,s3;
	unsigned char r;
	
	// Clear callback (mpu_get_agt_int_cb may have been called by user and callback assigned)
	mpu_i2c_trans_select_a.callback=0;
	mpu_i2c_trans_read_6.callback=0;
	
	// Queue the write/read transactions - they must be preliminarily initialised.
	r = i2c_transaction_queue(2,1,&mpu_i2c_trans_select_a,&mpu_i2c_trans_read_6);
	
	// Error
	if(r!=0)
			return 1;
	
	// Check register select error
	if(mpu_i2c_trans_select_a.status!=0)
		return 2;
	// Check register read error
	if(mpu_i2c_trans_read_6.status!=0)
		return 3;
	
	// Conversion	
	s1=mpu_i2c_trans_read_6.data[0]; s1<<=8; s1|=mpu_i2c_trans_read_6.data[1];
	s2=mpu_i2c_trans_read_6.data[2]; s2<<=8; s2|=mpu_i2c_trans_read_6.data[3];
	s3=mpu_i2c_trans_read_6.data[4]; s3<<=8; s3|=mpu_i2c_trans_read_6.data[5];
	*ax = s1; *ay = s2; *az = s3;
		
	return 0;
}
/******************************************************************************
	mpu_get_a_int_cb
*******************************************************************************	
	Transactional interrupt-driven read, calling a callback on success or error
	
	Call mpu_get_agt_int_init for initialisation once prior to this.
	
	The user callback receives status and error.
	If status is nonzero, the transaction failed. 
	There is no distinction between	a failure to select the registers and read 
	the registers.
	
	Multiple calls to this function without waiting for the callback to be called are 
	possible.
		
	
	Return value:
		0:	Success
		1:	Error issuing transactions (no space available)
******************************************************************************/
unsigned char mpu_get_a_int_cb(MPU_READ_CALLBACK3 cb)
{
	// Internal callbacks
	mpu_i2c_trans_select_a.callback=(void*)_mpu_i2c_trans_select3_cb;
	mpu_i2c_trans_read_6.callback=(void*)_mpu_i2c_trans_read_a_cb;
	// Indicate user callback to internal callbacks
	mpu_i2c_trans_select_a.user = (void*)cb;
	mpu_i2c_trans_read_6.user = (void*)cb;
		
	unsigned char r = i2c_transaction_queue(2,0,&mpu_i2c_trans_select_a,&mpu_i2c_trans_read_6);		
	if(r!=0)
		return 1;
	_mpu_get_agt_int_cb_numtrans++;
	return 0;
}


/******************************************************************************
	mpu_get_g_int
*******************************************************************************	
	Transactional interrupt-driven read, blocking until data available.
	
	Call mpu_get_agt_int_init for initialisation once prior to this.
	
	Return value:
		0:	Success
		1:	Error issuing transactions (no space available)
		2:	Error in register select
		3:	Error in register read
******************************************************************************/
unsigned char mpu_get_g_int(signed short *gx,signed short *gy,signed short *gz)
{
	unsigned short s1,s2,s3;
	unsigned char r;
	
	// Clear callback (mpu_get_agt_int_cb may have been called by user and callback assigned)
	mpu_i2c_trans_select_g.callback=0;
	mpu_i2c_trans_read_6.callback=0;
	
	// Queue the write/read transactions - they must be preliminarily initialised.
	r = i2c_transaction_queue(2,1,&mpu_i2c_trans_select_g,&mpu_i2c_trans_read_6);
	
	// Error
	if(r!=0)
			return 1;
	
	// Check register select error
	if(mpu_i2c_trans_select_g.status!=0)
		return 2;
	// Check register read error
	if(mpu_i2c_trans_read_6.status!=0)
		return 3;
	
	// Conversion	
	s1=mpu_i2c_trans_read_6.data[0]; s1<<=8; s1|=mpu_i2c_trans_read_6.data[1];
	s2=mpu_i2c_trans_read_6.data[2]; s2<<=8; s2|=mpu_i2c_trans_read_6.data[3];
	s3=mpu_i2c_trans_read_6.data[4]; s3<<=8; s3|=mpu_i2c_trans_read_6.data[5];
	*gx = s1; *gy = s2; *gz = s3;
	
	return 0;
}
/******************************************************************************
	mpu_get_g_int_cb
*******************************************************************************	
	Transactional interrupt-driven read, calling a callback on success or error
	
	Call mpu_get_agt_int_init for initialisation once prior to this.
	
	The user callback receives status and error.
	If status is nonzero, the transaction failed. 
	There is no distinction between	a failure to select the registers and read 
	the registers.
	
	Multiple calls to this function without waiting for the callback to be called are 
	possible.
		
	
	Return value:
		0:	Success
		1:	Error issuing transactions (no space available)
******************************************************************************/
unsigned char mpu_get_g_int_cb(MPU_READ_CALLBACK3 cb)
{
	// Internal callbacks
	mpu_i2c_trans_select_g.callback=(void*)_mpu_i2c_trans_select3_cb;
	mpu_i2c_trans_read_6.callback=(void*)_mpu_i2c_trans_read_g_cb;
	// Indicate user callback to internal callbacks
	mpu_i2c_trans_select_g.user = (void*)cb;
	mpu_i2c_trans_read_6.user = (void*)cb;
	
	unsigned char r = i2c_transaction_queue(2,0,&mpu_i2c_trans_select_g,&mpu_i2c_trans_read_6);		
	if(r!=0)
		return 1;
	_mpu_get_agt_int_cb_numtrans++;
	return 0;
}


/******************************************************************************
	mpu_get_agt_int_cb_idle
*******************************************************************************	
	Indicates whether there is an ongoing agt read with callback.
	
	
	Return value:
		0:					No transactions in progress
		nonzero:		Number of transactions in progress
******************************************************************************/
unsigned char mpu_get_agt_int_cb_busy(void)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		return _mpu_get_agt_int_cb_numtrans;
	}
	return 0;	// To avoid compiler warning
}


volatile unsigned char _mpu_get_agt_int_cb_numtrans=0;		// Number of ongoing transactions

/******************************************************************************
	_mpu_i2c_trans_select7_cb
*******************************************************************************	
	Internal use.
	
	Callback for register select.
******************************************************************************/
unsigned char _mpu_i2c_trans_select7_cb(volatile struct _I2C_TRANSACTION *it)
{
	if(it->status!=0)
	{
		MPU_READ_CALLBACK7 cb = (MPU_READ_CALLBACK7)it->user;	
		cb(it->status,it->i2cerror,0,0,0,0,0,0,0);
		
		// Remove the next transaction from the queue (read).
		i2c_transaction_removenext();
		_mpu_get_agt_int_cb_numtrans--;
	}
	return 0;
}
/******************************************************************************
	_mpu_i2c_trans_select3_cb
*******************************************************************************	
	Internal use.
	
	Callback for register select.
******************************************************************************/
unsigned char _mpu_i2c_trans_select3_cb(volatile struct _I2C_TRANSACTION *it)
{
	if(it->status!=0)
	{
		MPU_READ_CALLBACK3 cb = (MPU_READ_CALLBACK3)it->user;	
		cb(it->status,it->i2cerror,0,0,0);
		
		// Remove the next transaction from the queue (read).
		i2c_transaction_removenext();
		_mpu_get_agt_int_cb_numtrans--;
	}
	return 0;
}
/******************************************************************************
	_mpu_i2c_trans_read_agt_cb
*******************************************************************************	
	Internal use.
	
	Callback for register read.
******************************************************************************/
unsigned char _mpu_i2c_trans_read_agt_cb(volatile struct _I2C_TRANSACTION *it)
{
	signed short ax,ay,az,gx,gy,gz,temp;
	unsigned short s1,s2,s3;
	
	
	MPU_READ_CALLBACK7 cb = (MPU_READ_CALLBACK7)it->user;	

	if(it->status!=0)
	{
		cb(it->status,it->i2cerror,0,0,0,0,0,0,0);
	}
	else
	{	
		s1=it->data[0]; s1<<=8; s1|=it->data[1];
		s2=it->data[2]; s2<<=8; s2|=it->data[3];
		s3=it->data[4]; s3<<=8; s3|=it->data[5];
		ax = s1; ay = s2; az = s3;
		s1=it->data[6]; s1<<=8; s1|=it->data[7];
		temp = s1;
		s1=it->data[8]; s1<<=8; s1|=it->data[9];
		s2=it->data[10]; s2<<=8; s2|=it->data[11];
		s3=it->data[12]; s3<<=8; s3|=it->data[13];
		gx = s1; gy = s2; gz = s3;
		cb(it->status,it->i2cerror,ax,ay,az,gx,gy,gz,temp);
		//cb(0x11,it->i2cerror,ax,ay,az,gx,gy,gz,temp);
	}
	_mpu_get_agt_int_cb_numtrans--;
	return 0;
}
/******************************************************************************
	_mpu_i2c_trans_read_a_cb
*******************************************************************************	
	Internal use.
	
	Callback for register read.
******************************************************************************/
unsigned char _mpu_i2c_trans_read_a_cb(volatile struct _I2C_TRANSACTION *it)
{
	signed short ax,ay,az;
	unsigned short s1,s2,s3;
			
	MPU_READ_CALLBACK3 cb = (MPU_READ_CALLBACK3)it->user;	
	if(it->status!=0)
	{
		cb(it->status,it->i2cerror,0,0,0);
	}
	else
	{	
		s1=it->data[0]; s1<<=8; s1|=it->data[1];
		s2=it->data[2]; s2<<=8; s2|=it->data[3];
		s3=it->data[4]; s3<<=8; s3|=it->data[5];
		ax = s1; ay = s2; az = s3;
		cb(it->status,it->i2cerror,ax,ay,az);
		//cb(0x22,it->i2cerror,ax,ay,az);
	}
	_mpu_get_agt_int_cb_numtrans--;
	return 0;
}
/******************************************************************************
	_mpu_i2c_trans_read_g_cb
*******************************************************************************	
	Internal use.
	
	Callback for register read.
******************************************************************************/
unsigned char _mpu_i2c_trans_read_g_cb(volatile struct _I2C_TRANSACTION *it)
{
	signed short gx,gy,gz;
	unsigned short s1,s2,s3;
			
	MPU_READ_CALLBACK3 cb = (MPU_READ_CALLBACK3)it->user;	
	if(it->status!=0)
	{
		cb(it->status,it->i2cerror,0,0,0);
	}
	else
	{	
		s1=it->data[0]; s1<<=8; s1|=it->data[1];
		s2=it->data[2]; s2<<=8; s2|=it->data[3];
		s3=it->data[4]; s3<<=8; s3|=it->data[5];
		gx = s1; gy = s2; gz = s3;
		cb(it->status,it->i2cerror,gx,gy,gz);
		//cb(0x33,it->i2cerror,gx,gy,gz);
	}
	_mpu_get_agt_int_cb_numtrans--;
	return 0;
}

