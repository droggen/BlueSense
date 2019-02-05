/*
	Changes:
		- Remove the "busy" and rely on the underlying spi-usart0 busy
		- Either: give up on fixing the first byte and document it should be ignored, or fix the read/interrupt logic to not write the 1st byte.
		-- In this case, exchange n bytes means sending n bytes and receiving n-1
*/

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

#include "main.h"
#include "adc.h"
#include "serial.h"
#include "i2c.h"
#include "ds3232.h"
#include "rn41.h"
#include "mpu.h"
#include "mpu_test.h"
#include "pkt.h"
#include "wait.h"
#include "init.h"
#include "lcd.h"
#include "fb.h"
#include "uiconfig.h"
#include "helper.h"
#include "i2c_internal.h"
#include "mode_demo.h"
#include "dbg.h"
//#include "mpu-common.h"
#include "mpu-usart0.h"
#include "spi-usart0.h"
#include "system.h"

/*
	File: mpu-usart0.c
	
	Low-level register access functions to the MPU registers over the SPI interface (using spi-usart0).
	
	The key functions are:
	
	* mpu_writereg:	writes one register. Waits for peripheral available before start, waits for completion. Do not use in interrupts.
	* mpu_readreg: reads one register. Waits for peripheral available before start, waits for completion. Do not use in interrupts.
	* mpu_readreg16: reads a 16-bit register. Waits for peripheral available before start, waits for completion. Do not use in interrupt.
	* mpu_readregs: reads several registers. Waits for peripheral available before start, waits for completion. Do not use in interrupts.
	* mpu_readregs_int: reads several registers using interrupts (faster). Waits for peripheral available before start, waits for completion. Do not use in interrupts.
	* mpu_readregs_int_cb: reads several registers using interrupts. Returns if peripheral not available, calls a callback on transfer completion. Interrupt safe.
	
	All functions will wait for the peripheral to be available, except mpu_readregs_int_cb which will return immediately with an error if the peripheral is used. This
	allows calls to all functions to be mixed from main code or interrupts (within interrupts only mpu_readregs_int_cb is allowed). 
	
*/


#define _MPU_FASTREAD_LIM 33
unsigned char _mpu_tmp[_MPU_FASTREAD_LIM+1];
unsigned char *_mpu_tmp_reg = _mpu_tmp+1;
void (*_mpu_cb)(void);
volatile unsigned char _mpu_ongoing=0;				// _mpu_ongoing is used to block access to _mpu_tmp and other static/global variables which can only be used by a single transaction at a time.
unsigned char *_mpu_result;
unsigned short _mpu_n;



/******************************************************************************
	function: mpu_writereg
*******************************************************************************	
	Writes an MPU register.
	
	Do not call from an interrupt.
	
	Parameters:
		reg		-		Register to write data to
		v		-		Data to write in register		
******************************************************************************/
void mpu_writereg(unsigned char reg,unsigned char v)
{
	unsigned char buf[2];
	buf[0] = reg;
	buf[1] = v;
	spiusart0_rwn(buf,2);
}


/******************************************************************************
	function: mpu_readreg
*******************************************************************************	
	Reads an 8-bit MPU register.
	
	Do not call from an interrupt.
	
	Parameters:
		reg		-		Register to read data from 
******************************************************************************/
unsigned char mpu_readreg(unsigned char reg)
{
	unsigned char buf[2];
	buf[0] = 0x80|reg;
	spiusart0_rwn(buf,2);
	return buf[1];
}
/******************************************************************************
	function: mpu_readreg16
*******************************************************************************	
	Reads a 16-bit MPU register.
	
	Do not call from an interrupt.
	
	Parameters:
		reg		-		First register to read data from; the next 8 bits are read
						from register reg+1
******************************************************************************/
unsigned short mpu_readreg16(unsigned char reg)
{
	unsigned short v;
	unsigned char buf[3];
	buf[0] = 0x80|reg;
	spiusart0_rwn(buf,3);
	v=(buf[1]<<8)|buf[2];
	return v;
}

/******************************************************************************
	function: mpu_readregs
*******************************************************************************	
	Reads several MPU registers.
	
	Do not call from an interrupt.
	
	Parameters:
		d		-		Buffer receiving the register data
		reg		-		First register to read data from
		n		-		Number of registers to read		
	Returns:
		Bla
******************************************************************************/
void mpu_readregs(unsigned char *d,unsigned char reg,unsigned char n)
{
	for(unsigned char i=0;i<n;i++)
	{
		d[i] = mpu_readreg(reg+i);
	}
}
/******************************************************************************
	function: mpu_readregs_int
*******************************************************************************	
	Reads several MPU registers using interrupt transfer. 
	Waits until transfer completion. 
	
	This is faster than calling mpu_readregs for transfer size smaller or equal
	to _MPU_FASTREAD_LIM; for larger transfer the function calls mpu_readregs.
	
	Do not call from an interrupt.
	
	Parameters:
		d		-		Buffer receiving the register data
		reg		-		First register to read data from
		n		-		Number of registers to read		
******************************************************************************/
void mpu_readregs_int(unsigned char *d,unsigned char reg,unsigned char n)
{
	if(n<=_MPU_FASTREAD_LIM)
	{
		// Allocate enough memory for _MPU_FASTREAD_LIM+1 exchange
		unsigned char tmp[_MPU_FASTREAD_LIM+1],*tmp2;
		tmp[0]=0x80|reg;
		tmp2=tmp+1;
		
		spiusart0_rwn_int(tmp,n+1);		// Waits for peripheral available, and blocks until transfer complete
		// Discard the first byte which is meaningless and copy to return buffer
		for(unsigned char i=0;i<n;i++)
			d[i]=tmp2[i];
	}
	else
	{
		mpu_readregs(d,reg,n);
	}
}
/******************************************************************************
	function: mpu_readregs_int_try_raw
*******************************************************************************	
	Reads several MPU registers using polling. Waits until transfer completion. 
	
	Suitable for calls from interrupts.
	
	Note that d must be an n+1 buffer, and the first register read will be
	stored at d[1].
	
	Parameters:
		d		-		Buffer receiving the register data; this must be a n+1 bytes register
		reg		-		First register to read data from
		n		-		Number of registers to read		
	
	Returns:
		0		-		Success
		1		-		Error: transaction too large or peripheral busy
	
******************************************************************************/
unsigned char mpu_readregs_int_try_raw(unsigned char *d,unsigned char reg,unsigned char n)
{
	if(n>=_MPU_FASTREAD_LIM)
		return 1;

	d[0]=0x80|reg;
	return spiusart0_rwn_try(d,n+1);
}
/******************************************************************************
	function: mpu_readregs_int_cb
*******************************************************************************	
	Read several MPU registers using interrupt transfer. 
	The maximum transfer size must be smaller or equal to _MPU_FASTREAD_LIM.
	
	The function returns immediately if another mpu_readregs_int_cb transaction
	is ongoing. It calls a callback upon completion.
	
	This function is safe to be called from an interrupt.
	
	The maximum number of exchanged bytes is _MPU_FASTREAD_LIM.
	
	Parameters:
		d		-		Buffer receiving the register data.
						
						If d is null, then the received data will be held
						in the temporary buffer _mpu_tmp_reg.
						The data in _mpu_tmp_reg must be used within the callback
						as subsequent transactions will overwrite the results.
		reg		-		First register to read data from
		n		-		Number of registers to read		
		cb		-		User callback to call when the transaction is completed.
						The result of the transaction is available in the 
						unsigned char array _mpu_result, which is set to point to
						d.
	Returns:
		0		-		Success
		1		-		Error
******************************************************************************/
unsigned char mpu_readregs_int_cb(unsigned char *d,unsigned char reg,unsigned char n,void (*cb)(void))
{
	// Transaction too large
	if(n>=_MPU_FASTREAD_LIM)
		return 1;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		// Another mpu transaction ongoing: return error.
		if(_mpu_ongoing)
		{
			return 1;
		}
		// Block from other mpu transactions being initiated
		_mpu_ongoing=1;
	}
		
	_mpu_result = d;
	_mpu_n = n;	
	_mpu_cb = cb;
	_mpu_tmp[0]=0x80|reg;
	if(spiusart0_rwn_int_cb(_mpu_tmp,n+1,__mpu_readregs_int_cb_cb))
	{
		// Transaction failed
		_mpu_ongoing=0;
		return 1;
	}
	return 0;
}
/******************************************************************************
	__mpu_readregs_int_cb_cb
*******************************************************************************	
	Callback called when a transfer initiated by mpu_readregs_int_cb is
	completed.
	
******************************************************************************/
void __mpu_readregs_int_cb_cb(void)
{
	//dbg_fputchar_nonblock('y',0);
	// Copy data to user buffer, if a buffer was provided
	if(_mpu_result)
	{
		for(unsigned char i=0;i<_mpu_n;i++)
			_mpu_result[i] = _mpu_tmp_reg[i];	// _mpu_tmp_reg is equal to _mpu_tmp+1
	}
	if(_mpu_cb)
		_mpu_cb();
	//dbg_fputchar_nonblock('Y',0);
	_mpu_ongoing=0;
}
/******************************************************************************
	function: mpu_readregs_int_cb_raw
*******************************************************************************	
	Read several MPU registers using interrupt transfer. 
	
	This is a "raw" variant of mpu_readregs_int_cb which is slightly faster.
	
	The result of the register read is in _mpu_tmp_reg must be used within the callback
	as subsequent transactions will overwrite the results.
	
	The callback must set _mpu_ongoing=0, otherwise no further transactions are possible.	
	
	The maximum transfer size must be smaller or equal to _MPU_FASTREAD_LIM.
	
	The function returns immediately if another mpu_readregs_int_cb transaction
	is ongoing. It calls a callback upon completion.
	
	This function is safe to be called from an interrupt.
	
	The maximum number of exchanged bytes is _MPU_FASTREAD_LIM.
	
	Parameters:
		reg		-		First register to read data from
		n		-		Number of registers to read		
		cb		-		User callback to call when the transaction is completed.
						The result of the transaction is available in the 
						unsigned char array _mpu_result, which is set to point to
						d.
	Returns:
		0		-		Success
		1		-		Error
		
******************************************************************************/
unsigned char mpu_readregs_int_cb_raw(unsigned char reg,unsigned char n,void (*cb)(void))
{
	// Transaction too large
	if(n>=_MPU_FASTREAD_LIM)
		return 1;
	// Disable interrupts and reserve the transaction
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		// Another mpu transaction ongoing: return error.
		if(_mpu_ongoing)
		{
			return 1;
		}
		// Block from other mpu transactions being initiated
		_mpu_ongoing=1;
	}
		
	_mpu_n = n;	
	_mpu_cb = cb;
	_mpu_tmp[0]=0x80|reg;
	if(spiusart0_rwn_int_cb(_mpu_tmp,n+1,cb))
	{
		// Transaction failed
		_mpu_ongoing=0;
		return 1;
	}
	return 0;
}
/******************************************************************************
	function: mpu_get_agt_int_init
*******************************************************************************	
	For compatibility with the I2C version of the MPU interface. 
	No need to call this function.
******************************************************************************/
void mpu_get_agt_int_init(void)
{
	// Nothing to do in the SPI version
}












/*unsigned char mpuxg( unsigned char data)
{
	// Wait for empty transmit buffer
	//while ( !( UCSR0A & 0b00100000) );
	_delay_ms(10);
	// Put data into buffer, sends the data 
	UDR0 = data;
	// Wait for data to be received
	// while ( !(UCSR0A & 0b10000000) );
	_delay_ms(10);
	
	// Get and return received data from buffer
	data = UDR0;
	
	printf("udre: %d tx: %d rx: %d\n",_int_udre,_int_tx,_int_rx);
	
	return data;
}*/


/*void mpux1(void)
{
	unsigned char r;
	
	// Select PA4
	PORTA=(PORTA&0b11101111);
	_delay_ms(1);
	PORTA=(PORTA|0b00010000);
	_delay_ms(1);
	
	// Send data to flush stuff
	r = mpuxg(0x80|117);
	r = mpuxg(0x80|117);
	r = mpuxg(0x80|117);
	
	
	PORTA=(PORTA&0b11101111);
	_delay_ms(1);
	
	//r = mpuxg(117);
	r = mpuxg(0x80|117);
	printf("r: %02x\n",r);
	r = mpuxg(0x80|117);
	printf("r: %02x\n",r);
	r = mpuxg(0x80|117);
	printf("r: %02x\n",r);
	r = mpuxg(0x80|117);
	printf("r: %02x\n",r);
	r = mpuxg(0x80|117);
	printf("r: %02x\n",r);
	
	// Deselect PA4
	PORTA=(PORTA|0b00010000);
	
	printf("udre: %d tx: %d rx: %d\n",_int_udre,_int_tx,_int_rx);
	
	while(1);
}
*/
/*void mpux2(void)
{
	unsigned char regs[128];
	
	memset(regs,0x55,128);
	
	//_mpuusart0_buffer = regs;
	//_mpuusart0_bufferptr = regs;
	//_mpuusart0_n=16;
	//_mpuusart0_ongoing=1;
	//_mpuusart0_moderead=1;
	//_mpuusart0_callback=0;
	
	printf("About to start\n");
	// Start the transfer
	PORTA=(PORTA&0b11101111);
	UDR0 = 0x80|117;
	
	// Wait completion
	//while(_mpuusart0_ongoing);
	
	
	
	printf("done\n");
	printf("udre: %d tx: %d rx: %d\n",_int_udre,_int_tx,_int_rx);
	for(unsigned i=0;i<32;i++)
		printf("%02X: %02X\n",i,regs[i]);
	
	while(1);
	
	
}*/

/*void mpux3(void)
{
	unsigned char regs[128];
	
	memset(regs,0x55,128);
	
	
	regs[0] = 0x80|117;
	//spiusart0_rwn(regs,16);
	spiusart0_rwn_int(regs,16);
	
	
	printf("done\n");
	for(unsigned i=0;i<32;i++)
		printf("%02X: %02X\n",i,regs[i]);
	
	while(1)
	{
		system_led_toggle(1);
		_delay_ms(500);
	}
	
	
}*/
/*void mpux4_cb(void)
{
	printf("cb\n");
}*/
/*void mpux4(void)
{
	unsigned char regs[128];
	
	memset(regs,0x55,128);
	
	
	regs[0] = 0x80|117;
	//spiusart0_rwn(regs,16);
	spiusart0_rwn_int_cb(regs,16,mpux4_cb);
	
	
	printf("done %d\n",spiusart0_isongoing());
	for(unsigned i=0;i<32;i++)
		printf("%02X: %02X\n",i,regs[i]);
		
	printf("now %d\n",spiusart0_isongoing());
	for(unsigned i=0;i<32;i++)
		printf("%02X: %02X\n",i,regs[i]);


	printf("with readreg\n");
	for(unsigned i=0;i<32;i++)
		printf("%02X: %02X\n",i,mpu_readreg(117+i));
	
	printf("with readreg again\n");
	for(unsigned i=0;i<32;i++)
		printf("%02X: %02X\n",i,mpu_readreg(117+i));

	
	while(1)
	{
		system_led_toggle(1);
		_delay_ms(500);
	}
	
	
}*/

/*void cb7(unsigned char status,unsigned char error,signed short ax,signed short ay,signed short az,signed short gx,signed short gy,signed short gz,signed short temp)
{
	dbg_fputchar_nonblock('z',0);*/
	/*dbg_fputchar_nonblock(hex2chr((ax>>12)&0xf),0);
	dbg_fputchar_nonblock(hex2chr((ax>>8)&0xf),0);
	dbg_fputchar_nonblock(hex2chr((ax>>4)&0xf),0);
	dbg_fputchar_nonblock(hex2chr((ax>>0)&0xf),0);
	dbg_fputchar_nonblock(' ',0);
	dbg_fputchar_nonblock(hex2chr((ay>>12)&0xf),0);
	dbg_fputchar_nonblock(hex2chr((ay>>8)&0xf),0);
	dbg_fputchar_nonblock(hex2chr((ay>>4)&0xf),0);
	dbg_fputchar_nonblock(hex2chr((ay>>0)&0xf),0);
	dbg_fputchar_nonblock(' ',0);
	dbg_fputchar_nonblock(hex2chr((az>>12)&0xf),0);
	dbg_fputchar_nonblock(hex2chr((az>>8)&0xf),0);
	dbg_fputchar_nonblock(hex2chr((az>>4)&0xf),0);
	dbg_fputchar_nonblock(hex2chr((az>>0)&0xf),0);
	dbg_fputchar_nonblock(' ',0);
	dbg_fputchar_nonblock(hex2chr((gx>>12)&0xf),0);
	dbg_fputchar_nonblock(hex2chr((gx>>8)&0xf),0);
	dbg_fputchar_nonblock(hex2chr((gx>>4)&0xf),0);
	dbg_fputchar_nonblock(hex2chr((gx>>0)&0xf),0);
	dbg_fputchar_nonblock(' ',0);
	dbg_fputchar_nonblock(hex2chr((gy>>12)&0xf),0);
	dbg_fputchar_nonblock(hex2chr((gy>>8)&0xf),0);
	dbg_fputchar_nonblock(hex2chr((gy>>4)&0xf),0);
	dbg_fputchar_nonblock(hex2chr((gy>>0)&0xf),0);
	dbg_fputchar_nonblock(' ',0);
	dbg_fputchar_nonblock(hex2chr((gz>>12)&0xf),0);
	dbg_fputchar_nonblock(hex2chr((gz>>8)&0xf),0);
	dbg_fputchar_nonblock(hex2chr((gz>>4)&0xf),0);
	dbg_fputchar_nonblock(hex2chr((gz>>0)&0xf),0);
	dbg_fputchar_nonblock(' ',0);
	dbg_fputchar_nonblock(hex2chr((temp>>12)&0xf),0);
	dbg_fputchar_nonblock(hex2chr((temp>>8)&0xf),0);
	dbg_fputchar_nonblock(hex2chr((temp>>4)&0xf),0);
	dbg_fputchar_nonblock(hex2chr((temp>>0)&0xf),0);
	dbg_fputchar_nonblock('\n',0);*/
	/*char str[64];
	unsigned char s;
//	sprintf(str,"%04X %04X %04X  %04X %04X %04X  %04X\n",ax,ay,az,gx,gy,gz,temp);
	//int n = strlen(s);
	//int n=32;
	//sprintf(s,"len: %d\n",n);
	//dbg_putbuf(str,strlen(str));
	
	u32toa(0,str);
	str[10]=' ';
	s16toa(temp,str+11);
	str[17]=' ';
	s16toa(ax,str+18);
	str[24]=' ';
	s16toa(ay,str+25);
	str[31]=' ';
	s16toa(az,str+32);
	str[38]=' ';
	s16toa(gx,str+39);		
	str[45]=' ';
	s16toa(gy,str+46);
	str[52]=' ';
	s16toa(gz,str+53);		
	str[59]='\n';
	str[60]='0';
	s=60;*/
	/*			
	u32toa(0,str);
	str[10]=' ';
	s16toa(ax,str+11);
	str[17]=' ';
	s16toa(ay,str+18);
	str[24]=' ';
	s16toa(az,str+25);
	str[31]='\n';
	str[32]=0;
	s=32;*/
/*	dbg_putbuf(str,s);
	
}*/
/*void cb3(unsigned char status,unsigned char error,signed short ax,signed short ay,signed short az)
{
	dbg_fputchar_nonblock('z',0);
	char str[64];
	unsigned char s;
	
	u32toa(0,str);
	str[10]=' ';
	s16toa(0,str+11);
	str[17]=' ';
	s16toa(ax,str+18);
	str[24]=' ';
	s16toa(ay,str+25);
	str[31]=' ';
	s16toa(az,str+32);
	str[38]='\n';
	str[39]=0;
	s=39;*/
	/*			
	u32toa(0,str);
	str[10]=' ';
	s16toa(ax,str+11);
	str[17]=' ';
	s16toa(ay,str+18);
	str[24]=' ';
	s16toa(az,str+25);
	str[31]='\n';
	str[32]=0;
	s=32;*/
/*	dbg_putbuf(str,s);
	
}*/
/*unsigned char mydata[64];
unsigned long pf=0;*/
/*void mycb(void)
{
	dbg_fputchar_nonblock('z',0);	
	for(unsigned char i=0;i<14;i++)
	{
		pf+=dbg_fputchar_nonblock(hex2chr((mydata[i]>>4)&0xf),0);
		pf+=dbg_fputchar_nonblock(hex2chr(mydata[i]&0xf),0);
		pf+=dbg_fputchar_nonblock(' ',0);
	}
	pf+=dbg_fputchar_nonblock('\n',0);
}*/
/*void mputestint(void)
{
	unsigned long t1;
	signed short ax,ay,az,gx,gy,gz,temp;
	unsigned short ctr=0;
	
	printf_P(PSTR("Start test\n"));
	_delay_ms(100);
	t1 = timer_ms_get();
	while(timer_ms_get()-t1<5000)
	{
		dbg_fputchar_nonblock('s',0);	
		
		//unsigned char r = mpu_get_agt_int_cb(cb7);
		unsigned char r = mpu_get_a_int_cb(cb3);
		if(r)
		{
			dbg_fputchar_nonblock('f',0);	
			
		}
		else
			dbg_fputchar_nonblock('S',0);	
		dbg_fputchar_nonblock('\n',0);	
		
		//_delay_ms(1000);
		_delay_ms(20);
	}
	printf("Samples/sec: %u\n",ctr);	
	printf("printf err: %lu\n",pf);
}*/


/*void mputestpoll(void)
{
	unsigned long t1;
	signed short ax,ay,az,gx,gy,gz,temp;
	unsigned short ctr=0;
	
	t1 = timer_ms_get();
	while(timer_ms_get()-t1<1000)
	{
		mpu_get_agt(&ax,&ay,&az,&gx,&gy,&gz,&temp);
		printf("agt: %d %d %d  %d %d %d  %d\n",ax,ay,az,gx,gy,gz,temp);
		_delay_ms(50);
	}
	printf("Samples/sec: %u\n",ctr);	
}*/
/*void mpubenchpoll(void)
{
	unsigned long t1;
	signed short ax,ay,az,gx,gy,gz,temp;
	unsigned short ctr=0;
	
	t1 = timer_ms_get();
	while(timer_ms_get()-t1<1000)
	{
		mpu_get_agt(&ax,&ay,&az,&gx,&gy,&gz,&temp);
		ctr++;
	}
	printf("Samples/sec: %u\n",ctr);
	printf("agt: %d %d %d  %d %d %d  %d\n",ax,ay,az,gx,gy,gz,temp);
}*/
/*void mpu_dumpreg(void)
{
	while(1)
	{
		printf("mpu_readreg (spiusart0_rwn) single\n");
		_delay_ms(1000);
		mpu_printreg(file_usb);
		
		_delay_ms(1000);

		//printf("mpu_readreg (spiusart0_rwn) burst\n");
		//mpu_printregx(file_usb);

		printf("mpu_readreg (spiusart0_rwn) burst int\n");
		_delay_ms(1000);
		mpu_printregx2(file_usb);
		
		_delay_ms(1000);
	}
	
	
	//mpu_printreg2(file_usb);
	//mpu_printregdesc(file_usb);
	//mpu_printregdesc2(file_usb);
}*/
/*void mpux(void)
{
	spiusart0_init();
	//mpux1();
	//mpux2();
	//mpux3();
	//mpux4();
	
	
	//mpu_dumpreg();
	//mputestpoll();
	//mpubenchpoll();
	
	mputestint();
	
	while(1);
}*/