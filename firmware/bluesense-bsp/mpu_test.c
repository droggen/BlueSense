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
#include "mpu_test.h"
#include "config.h"
#include "streaming.h"



void mpu_test_smplrt_div(void)
{
	printf_P(PSTR("Test SMPLRT_DIV\n"));
	for(unsigned char div=0;div<3;div++)
	{
		printf_P(PSTR("SMPLRT_DIV: %d\n"),div);
		mpu_setsrdiv(div);
		mpu_estimateodr();
	}
}

void mpu_test_lpodr(void)
{
	printf_P(PSTR("Reset\n"));
	mpu_reset();
	
	mpu_printregdesc2(file_usb);

	printf_P(PSTR("Base config\n"));
	i2c_writereg(MPU_ADDRESS,MPU_R_PWR_MGMT_1,0x00);
	i2c_writereg(MPU_ADDRESS,MPU_R_PWR_MGMT_2,0x00);
	
	i2c_writereg(MPU_ADDRESS,MPU_R_CONFIG,0x00);								// DLP=0
	i2c_writereg(MPU_ADDRESS,MPU_R_GYROCONFIG,0b00000010);			// fchoice=11 -> 8KHz
	
	mpu_printregdesc2(file_usb);
	
	printf_P(PSTR("LP config\n"));
	i2c_writereg(MPU_ADDRESS,MPU_R_PWR_MGMT_1,0b00000000);		// Make sure accel running
	i2c_writereg(MPU_ADDRESS,MPU_R_PWR_MGMT_2,0b00000111);		// Activate only accel
	//i2c_writereg(MPU_ADDRESS,MPU_R_PWR_MGMT_2,0b00000110);		// Activate accel + 1 gyro
	//i2c_writereg(MPU_ADDRESS,MPU_R_PWR_MGMT_1,0b00101000);		// !sleep, cycle, !gstby, !ptat
	//i2c_writereg(MPU_ADDRESS,MPU_R_PWR_MGMT_1,0b00000000);		// !sleep, cycle, !gstby, !ptat
	//i2c_writereg(MPU_ADDRESS,MPU_R_ACCELCONFIG2,0b00001000);	// Disable DLP fchoice=0 (fchoice_b=1)
	i2c_writereg(MPU_ADDRESS,MPU_R_ACCELCONFIG2,0b00000001);	// Enable DLP fchoice=1, 1KHz
	//i2c_writereg(MPU_ADDRESS,MPU_R_LPODR,11);									// Write ODR 500Hz
	//i2c_writereg(MPU_ADDRESS,MPU_R_LPODR,10);									// Write ODR 250Hz
	i2c_writereg(MPU_ADDRESS,MPU_R_LPODR,9);									// Write ODR 125Hz
	//i2c_writereg(MPU_ADDRESS,MPU_R_LPODR,3);									// Write ODR 1.95Hz
	i2c_writereg(MPU_ADDRESS,MPU_R_PWR_MGMT_1,0b00100000);		// Enable cycle mode
	mpu_setsrdiv(0);		// Not used in LP
	//mpu_setsrdiv(249);		// Not used in LP
	mpu_fifoenable(0x00);
	
	
	
	i2c_writereg(MPU_ADDRESS,56,0b10001);	// Activate interrupt
	
	//mpu_setgyrosamplerate(0b00,0x00);	
	//mpu_setaccsamplerate(0b0,0x00);	
	
	/*i2c_writereg(MPU_ADDRESS,106,0b01000000);	// Activate FIFO
	printf_P(PSTR("FIFO level: %d\n"),mpu_getfifocnt());
	i2c_writereg(MPU_ADDRESS,106,0b01000100);	// Activate FIFO + reset
	_delay_ms(1);
	printf_P(PSTR("FIFO level: %d\n"),mpu_getfifocnt());*/

	
	
	mpu_printregdesc2(file_usb);
	
	
	
	while(1)
	{
		i2c_writereg(MPU_ADDRESS,106,0b01000101);	// Activate FIFO + reset + signal path reset
		_delay_ms(10);
		i2c_writereg(MPU_ADDRESS,106,0b01000000);
		mpu_fifoenable(0b00001000);
		unsigned short cnt1,cnt2;
		mpu_printregdesc2(file_usb);
		for(int i=0;i<10;i++)
		{
			unsigned char status=0;
			unsigned char r = i2c_readreg(MPU_ADDRESS,i,&status);
			
			cnt1  = mpu_getfifocnt();
			mpu_fiforead(0,cnt1);
			cnt2  = mpu_getfifocnt();
			printf_P(PSTR("%d. FIFO level: %d -> %d. Status: %02X\n"),(short)i,(short)cnt1,(short)cnt2,(short)status);
			
			// Read fifo;
			
			//cnt  = mpu_getfifocnt();
			//printf_P(PSTR("New FIFO level: %d.\n"),cnt);
			
			_delay_ms(100);
		}
		//mpu_fifoenable(0x00);
		//i2c_writereg(MPU_ADDRESS,106,0b10000100);	// Stop FIFO + reset
		//printf_P(PSTR("FIFO level after reset: %d.\n"),mpu_getfifocnt());
		cnt1  = mpu_getfifocnt();
		unsigned long long cc1,cc2;
		cc1 = motion_int_ctr;
		_delay_ms(1000);
		cc2 = motion_int_ctr;
		cnt2  = mpu_getfifocnt();
		printf_P(PSTR("Bytes/s: %d. Int/s: %ld\n"),cnt2-cnt1,cc2-cc1);
		
		
	}
	//mpu_estimatefifofillrate(0b00001000);
	
	
}

void mpu_test_lpacc(void)
{
	unsigned long long cc1,cc2;
	unsigned short cnt1,cnt2;
	printf_P(PSTR("Test LP ACC mode\n"));
	//mpu_printregdesc2(file_usb);
	
	
	mpu_fifoenable(0b00001000);
	
	mpu_set_interrutenable(0,0,0,1);	
	for(unsigned char odr=MPU_LPODR_1;odr<=MPU_LPODR_500;odr++)
	{
		mpu_mode_lpacc(odr);
		
		for(unsigned char i=0;i<3;i++)
		{
			i2c_writereg(MPU_ADDRESS,106,0b01000100);	// Activate FIFO + reset FIFO
			cnt1  = mpu_getfifocnt();
			cc1 = motion_int_ctr;
			_delay_ms(1000);
			cc2 = motion_int_ctr;
			cnt2  = mpu_getfifocnt();
			printf_P(PSTR("ODR %d. Bytes/s: %d. Int/s: %ld\n"),odr,cnt2-cnt1,cc2-cc1);
		}
	}	
}

void mpu_test_acc(void)
{
	unsigned long long cc1,cc2;
	unsigned short cnt1,cnt2;
	printf_P(PSTR("Test normal ACC mode\n"));
	//mpu_printregdesc2(file_usb);
	
	
	
	mpu_fifoenable(0b00001000);			// FIFO for accelerometer
	//mpu_fifoenable(0b11111000);		// FIFO for all
	mpu_set_interrutenable(0,0,0,1);		
	
	for(unsigned char dlpenable=0;dlpenable<=1;dlpenable++)
	{
		for(unsigned char bw=MPU_ACC_LPF_460;bw<=MPU_ACC_LPF_5;bw++)
		{
			for(unsigned char divider=0;divider<8;divider+=3)
			{
				printf_P(PSTR("DLP enabled: %d. DLP BW: %d. Divider: %d\n"),dlpenable,bw,divider);
				mpu_mode_acc(dlpenable,bw,divider);
				
				for(unsigned char i=0;i<3;i++)
				{
					i2c_writereg(MPU_ADDRESS,106,0b01000100);	// Activate FIFO + reset FIFO
					cnt1  = mpu_getfifocnt();
					cc1 = motion_int_ctr;
					_delay_ms(1000);
					cc2 = motion_int_ctr;
					cnt2  = mpu_getfifocnt();
					printf_P(PSTR(" Bytes/s: %d. Int/s: %ld\n"),cnt2-cnt1,cc2-cc1);
				}
			}
		}	
	}
	printf("end\n");
}
void mpu_test_accgyro(void)
{
	unsigned long cc1,cc2;
	unsigned long cnt1,cnt2;
	unsigned long t1,t2;
	unsigned char fifoopt[5] = {0b11111000,0b10000000,0b01000000,0b01110000,0b00001000};
	
	printf_P(PSTR("Test accelerometer+gyroscope mode mode\n"));
	
	mpu_fifoenable(0b00001000);			// FIFO for accelerometer
	//mpu_fifoenable(0b11111000);		// FIFO for all
	mpu_set_interrutenable(0,0,0,1);	
	
		
	
	for(unsigned char gdlpe=0;gdlpe<=1;gdlpe++)
	{
		for(unsigned char gdlpoffhbw=0;gdlpoffhbw<=1;gdlpoffhbw++)
		{
			for(unsigned char gbw=MPU_GYR_LPF_250;gbw<=MPU_GYR_LPF_3600;gbw++)
			{
				for(unsigned char adlpe=0;adlpe<=1;adlpe++)	
				{
					for(unsigned char abw=MPU_ACC_LPF_460;abw<=MPU_ACC_LPF_5;abw++)
					{
						for(unsigned char divider=0;divider<8;divider+=3)
						{
							printf_P(PSTR("GDLPE: %d. GDLPOFFHBW: %d. GBW: %d. ADLPE: %d. ABW: %d. divider: %d\n"),gdlpe,gdlpoffhbw,gbw,adlpe,abw,divider);
							mpu_mode_accgyro(gdlpe,gdlpoffhbw,gbw,adlpe,abw,divider);
							
							for(unsigned char fo=0;fo<5;fo++)
							{
								printf(" FIFO: %02X\n",fifoopt[fo]);
								for(unsigned char i=0;i<1;i++)
								{
									mpu_fifoenable(0x00);														// Disable
									i2c_writereg(MPU_ADDRESS,106,0b01000100);	// Activate FIFO + reset FIFO
									_delay_ms(10);
									printf_P(PSTR("  lvl: %d."),mpu_getfifocnt());
									mpu_fifoenable(fifoopt[fo]);									// Select what to put in FIFO
									
									unsigned mindelay=1000;
									unsigned minsample=25;
									
									cnt1  = mpu_getfifocnt();
									cc1 = motion_int_ctr;
									t1 = timer_ms_get();
									while((motion_int_ctr-cc1<minsample) || (timer_ms_get()-t1)<mindelay);				// Wait at least 25 samples (350 bytes) or 10 ms
									t2 = timer_ms_get();
									cc2 = motion_int_ctr;									
									cnt2  = mpu_getfifocnt();							
									printf_P(PSTR("  lvl %ld->%ld. spl: %ld->%ld. dt: %ld."),cnt1,cnt2,cc1,cc2,t2-t1);
									if(cnt2==512)
										printf_P(PSTR(" Bytes/s: xxx"));
									else
										printf_P(PSTR(" Bytes/s: %ld."),(cnt2-cnt1)*1000/(t2-t1));
									printf_P(PSTR(" Int/s: %ld\n"),(cc2-cc1)*1000/(t2-t1));
									
									/*unsigned long delay=1;									
									cnt1  = mpu_getfifocnt();
									cc1 = motion_int_ctr;
									_delay_ms(delay);
									cc2 = motion_int_ctr;
									cnt2  = mpu_getfifocnt();
									printf_P(PSTR("  %ld->%ld. Bytes/s: %ld. Int/s: %ld\n"),cnt1,cnt2,(cnt2-cnt1)*(1000/delay),(cc2-cc1)*(1000/delay));*/
								}
							} // fifo
						} // divider
						if(!adlpe) break;
					}	// abw
				} // adlpe
				if(!gdlpe) break;
			} // gbw
			if(gdlpe) break;
		} // gdlpoffhbw
	}	// gdlpe
	printf("end\n");
}


void mpu_test_bypass(void)
{
	unsigned char r,v;
	
	i2c_check();
	for(int i=0;i<3;i++)
	{
		r = i2c_readreg(MAG_ADDRESS,0x00,&v);	
		printf("r: %02X v: %02X\n",r,v);
	}
	printf("Enable bypass\n");
	r = i2c_writereg(MPU_ADDRESS,55,0b00000010);				// BYPASS_EN set
	
	i2c_check();
	
	for(int i=0;i<3;i++)
	{
		r = i2c_readreg(MAG_ADDRESS,0x00,&v);	
		printf("r: %02X v: %02X\n",r,v);
	}
	printf("Disable bypass\n");
	r = i2c_writereg(MPU_ADDRESS,55,0b00000000);				// BYPASS_EN clear
	for(int i=0;i<3;i++)
	{
		r = i2c_readreg(MAG_ADDRESS,0x00,&v);	
		printf("r: %02X v: %02X\n",r,v);
	}
	
	
}
void mpu_test_sample(void)
{
	// Benchmark sampling speed.
	signed short ax,ay,az,gx,gy,gz,temp;
	unsigned long t1,t2;
	
	mpu_set_interruptpin(0x00);
	
	mpu_printregdesc(file_usb);
	
	//mpu_mode_accgyro(1,0,MPU_GYR_LPF_5,1,MPU_ACC_LPF_5,255);					// LPF 5Hz, ODR=1000/256=3.9Hz
	mpu_mode_accgyro(0,0,MPU_GYR_LPF_250,1,MPU_ACC_LPF_5,255);					// LPF 5Hz, ODR=1000/256=3.9Hz
	mpu_set_interrutenable(0,0,0,1);
	
	
	while(1)
	{
		unsigned long sc=0;
		t1 = timer_ms_get();
		while( (t2=timer_ms_get())-t1<1000)
		{
			mpu_get_agt(&ax,&ay,&az,&gx,&gy,&gz,&temp);
			sc++;
		}
		printf_P(PSTR("t: %ld->%ld. #%ld. rps: %ld\n"),t1,t2,sc,sc*1000/(t2-t1));
		
		t1=timer_ms_get();
		for(unsigned i=0;i<1000;i++)
		{
			mpu_get_agt(&ax,&ay,&az,&gx,&gy,&gz,&temp);
		}
		t2=timer_ms_get();
		printf_P(PSTR("t: %ld->%ld (%ld). #1000. rps: %ld\n\n"),t1,t2,t2-t1,1000l*1000l/(t2-t1));
		
		_delay_ms(1000);
		
	}
	
		
}

void mpu_test_sample2(void)
{
	// Benchmark sampling speed.
	signed short ax,ay,az,gx,gy,gz,temp;
	unsigned long t1,t2;
	
	mpu_set_interruptpin(0x00);
	
	mpu_printregdesc(file_usb);
	
	//mpu_mode_accgyro(1,0,MPU_GYR_LPF_5,1,MPU_ACC_LPF_5,255);					// LPF 5Hz, ODR=1000/256=3.9Hz
	mpu_mode_accgyro(0,0,MPU_GYR_LPF_250,1,MPU_ACC_LPF_5,255);					// LPF 5Hz, ODR=1000/256=3.9Hz
	mpu_set_interrutenable(0,0,0,1);
	
	
	while(1)
	{
		unsigned long sc=0;
		t1 = timer_ms_get();
		while( (t2=timer_ms_get())-t1<1000)
		{
			mpu_get_agt_int(&ax,&ay,&az,&gx,&gy,&gz,&temp);
			sc++;
		}
		printf_P(PSTR("t: %ld->%ld. #%ld. rps: %ld\n"),t1,t2,sc,sc*1000/(t2-t1));
		
		t1=timer_ms_get();
		for(unsigned i=0;i<1000;i++)
		{
			mpu_get_agt_int(&ax,&ay,&az,&gx,&gy,&gz,&temp);
		}
		t2=timer_ms_get();
		printf_P(PSTR("t: %ld->%ld (%ld). #1000. rps: %ld\n\n"),t1,t2,t2-t1,1000l*1000l/(t2-t1));
		
		_delay_ms(1000);
		
	}
	
		
}


void mpu_test_sample3_cb(unsigned char status,unsigned char error,signed short ax,signed short ay,signed short az,signed short gx,signed short gy,signed short gz,signed short temp)
{
	uart0_putchar('D');
	uart0_putchar(hex2chr(status>>4));
	uart0_putchar(hex2chr(status&0xf));
	uart0_putchar('E');
	uart0_putchar(hex2chr(error>>4));
	uart0_putchar(hex2chr(error&0xf));
	uart0_putchar(' ');
	uart0_putchar(hex2chr((ax>>12)&0xf));
	uart0_putchar(hex2chr((ax>>8)&0xf));
	uart0_putchar(hex2chr((ax>>4)&0xf));
	uart0_putchar(hex2chr((ax>>0)&0xf));
	uart0_putchar(' ');
	uart0_putchar(hex2chr((ay>>12)&0xf));
	uart0_putchar(hex2chr((ay>>8)&0xf));
	uart0_putchar(hex2chr((ay>>4)&0xf));
	uart0_putchar(hex2chr((ay>>0)&0xf));
	uart0_putchar(' ');
	uart0_putchar(hex2chr((az>>12)&0xf));
	uart0_putchar(hex2chr((az>>8)&0xf));
	uart0_putchar(hex2chr((az>>4)&0xf));
	uart0_putchar(hex2chr((az>>0)&0xf));
	uart0_putchar('\r');
	//printf_P(PSTR("cb %d %d. Acc %d %d %d. Gyr %d %d %d. Temp %d.\n"),status,error,ax,ay,az,gx,gy,gz,temp);
}

void mpu_test_sample3_cb2(unsigned char status,unsigned char error,signed short ax,signed short ay,signed short az,signed short gx,signed short gy,signed short gz,signed short temp)
{
	//mpu_test_sample3_cb(status,error,ax,ay,az,gx,gy,gz,temp);
	
	
}




void mpu_test_sample3(void)
{
	// Benchmark sampling speed.
	signed short ax,ay,az,gx,gy,gz,temp;
	unsigned long t1,t2;
	
	mpu_set_interruptpin(0x00);
	
	mpu_printregdesc(file_usb);
	
	mpu_mode_accgyro(1,0,MPU_GYR_LPF_5,1,MPU_ACC_LPF_5,255);					// LPF 5Hz, ODR=1000/256=3.9Hz
	//mpu_mode_accgyro(0,0,MPU_GYR_LPF_250,1,MPU_ACC_LPF_5,255);					// LPF 5Hz, ODR=1000/256=3.9Hz
	mpu_set_interrutenable(0,0,0,1);
	
	mpu_get_agt_int_init();
	
	printf_P(PSTR("Sizeof(void*): %d sizeof fcnptr: %d\n"),sizeof(void *),sizeof(void (*)(unsigned char)));
	printf_P(PSTR("User callback: %p\n"),mpu_test_sample3_cb);
	
	while(1)
	{
		printf_P(PSTR("Start of acquisition\n"));
		//mpu_get_agt_int_cb(mpu_test_sample3_cb);
		//mpu_get_agt_int_cb(mpu_test_sample3_cb);
		//while(!mpu_get_agt_int_cb_idle());
		//printf_P(PSTR("Done\n"));
		//mpu_get_agt_int(&ax,&ay,&az,&gx,&gy,&gz,&temp);
		//printf("%d %d %d\n",ax,ay,az);
		//printf_P(PSTR("all issued\n"));
		
		// Without callback
		
		unsigned long sc;
		sc=0;
		t1 = timer_ms_get();
		while( (t2=timer_ms_get())-t1<1000)
		{
			mpu_get_agt_int(&ax,&ay,&az,&gx,&gy,&gz,&temp);
			sc++;
		}
		printf_P(PSTR("Blocking I2C interrupt time-based t: %ld->%ld. #%ld. rps: %ld\n"),t1,t2,sc,sc*1000/(t2-t1));
		
		t1=timer_ms_get();
		for(unsigned i=0;i<1000;i++)
		{
			mpu_get_agt_int(&ax,&ay,&az,&gx,&gy,&gz,&temp);
		}
		t2=timer_ms_get();
		printf_P(PSTR("Blocking I2C interrupt sample-based t: %ld->%ld (%ld). #1000. rps: %ld\n\n"),t1,t2,t2-t1,1000l*1000l/(t2-t1));
		
		
		// With callback
		
		sc=0;
		t1 = timer_ms_get();
		while( (t2=timer_ms_get())-t1<1000)
		{
			mpu_get_agt_int_cb(mpu_test_sample3_cb2);
			while(mpu_get_agt_int_cb_busy());
			sc++;
		}
		printf_P(PSTR("Callback I2C interrupt time-based t: %ld->%ld. #%ld. rps: %ld\n"),t1,t2,sc,sc*1000/(t2-t1));
		
		t1=timer_ms_get();
		for(unsigned i=0;i<1000;i++)
		{
			mpu_get_agt_int(&ax,&ay,&az,&gx,&gy,&gz,&temp);
		}
		t2=timer_ms_get();
		printf_P(PSTR("Callback I2C interrupt sample-based t: %ld->%ld (%ld). #1000. rps: %ld\n\n"),t1,t2,t2-t1,1000l*1000l/(t2-t1));
		
		//_delay_ms(5000);
		//_delay_ms(2);
		
	}
	
		
}

void test(void)
{
	signed short ax,ay,az,gx,gy,gz,temp;
	unsigned long t1,t2;
	
	mpu_fifoenable(0x00);														// Disable
	i2c_writereg(MPU_ADDRESS,106,0b01000100);	// Activate FIFO + reset FIFO
	_delay_ms(10);
	printf_P(PSTR("  lvl: %d."),mpu_getfifocnt());
	mpu_fifoenable(0b11111000);									// Select what to put in FIFO
	t1=timer_ms_get();
	while(1)
	{
		unsigned long c1=motion_int_ctr;
		while(motion_int_ctr==c1);
		t2=timer_ms_get();
		printf_P(PSTR("%d %d. "),(PINC&0b00100000)>>5,mpu_getfifocnt());
		printf_P(PSTR("%d.\n"),t2-t1);
		t1=t2;
		for(unsigned char i=0;i<3;i++)
		{
			mpu_get_agt(&ax,&ay,&az,&gx,&gy,&gz,&temp);
			printf_P(PSTR("   %-04d %-04d %-04d  %-04d %-04d %-04d  %d\n"),ax,ay,az,gx,gy,gz,temp);
		}
	}
}

void mpu_test_sample4(void)
{
	unsigned char r;
	
	I2C_TRANSACTION t1,t2,t3,t4,t5,t6;
	
	i2c_transaction_setup(&t1,MPU_ADDRESS,I2C_WRITE,0,1);
	t1.data[0]=59;													// data to send/receive
	
	i2c_transaction_setup(&t2,MPU_ADDRESS,I2C_READ,1,14);
	
	i2c_transaction_setup(&t3,MPU_ADDRESS,I2C_READ,0,1);
	t3.data[0]=117;													// data to send/receive
	
	i2c_transaction_setup(&t4,MPU_ADDRESS,I2C_READ,1,1);
	
	i2c_transaction_setup(&t4,104,I2C_WRITE,0,1);
	
	i2c_transaction_setup(&t4,104,I2C_READ,1,7);
	
	i2c_transaction_init();
	
	i2c_transaction_printall(file_usb);
	
	printf("Queuing tryouts\n");
	printf("Avail transactions: %p %p %p %p %p %p\n",&t1,&t2,&t3,&t4,&t5,&t6);

	while(1)
	{
		unsigned long int tim1,tim2;
		tim1 = timer_ms_get();
		//r = i2c_transaction_queue(7,1,&t1,&t2,&t3,&t4,&t5,&t6,&t3);
		r = i2c_transaction_queue(7,0,&t1,&t2,&t3,&t4,&t5,&t6,&t3);
		tim2 = timer_ms_get();
		printf("Queuing return: %d\n",r);
	
	
		printf("done. Elapsed: %ld\n",tim2-tim1);
	
		i2c_transaction_printall(file_usb);		
		i2c_transaction_print(&t1,file_usb);
		i2c_transaction_print(&t2,file_usb);
		i2c_transaction_print(&t3,file_usb);
		i2c_transaction_print(&t4,file_usb);
		i2c_transaction_print(&t5,file_usb);
		i2c_transaction_print(&t6,file_usb);
		//_delay_ms(1000);
	}
	
}



unsigned char xx1(volatile struct _I2C_TRANSACTION *it)
{
	uart0_fputchar_int('1',0);
	return 0;
}
unsigned char xx2(volatile struct _I2C_TRANSACTION *it)
{
//	uart0_fputchar_int('2',0);
	unsigned short s1,s2,s3;
	signed short ax,ay,az,gx,gy,gz,temp;
	
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
	mpu_test_sample3_cb2(it->status,it->i2cerror,ax,ay,az,gx,gy,gz,temp);
	return 0;
}

void mpu_test_sample5(void)
{
	// Benchmark sampling speed.
	signed short ax,ay,az,gx,gy,gz,temp;
	unsigned long tt1,tt2;
	unsigned char r;
	
	I2C_TRANSACTION t1,t2;
	
	i2c_transaction_setup(&t1,MPU_ADDRESS,I2C_WRITE,0,1);
	t1.data[0]=59;													// data to send/receive
	
	i2c_transaction_setup(&t2,MPU_ADDRESS,I2C_READ,1,14);
	t2.callback=xx2;
	t2.user=0;
	
	i2c_transaction_init();
	
	
	mpu_set_interruptpin(0x00);
	
	mpu_printregdesc(file_usb);
	
	mpu_mode_accgyro(1,0,MPU_GYR_LPF_5,1,MPU_ACC_LPF_5,255);					// LPF 5Hz, ODR=1000/256=3.9Hz
	//mpu_mode_accgyro(0,0,MPU_GYR_LPF_250,1,MPU_ACC_LPF_5,255);					// LPF 5Hz, ODR=1000/256=3.9Hz
	mpu_set_interrutenable(0,0,0,1);
	
	mpu_get_agt_int_init();
	
	printf_P(PSTR("Sizeof(void*): %d sizeof fcnptr: %d\n"),sizeof(void *),sizeof(void (*)(unsigned char)));
	printf_P(PSTR("User callback: %p\n"),mpu_test_sample3_cb);
	
	while(1)
	{
		printf_P(PSTR("Start of acquisition\n"));
		//mpu_get_agt_int_cb(mpu_test_sample3_cb);
		//mpu_get_agt_int_cb(mpu_test_sample3_cb);
		//while(!mpu_get_agt_int_cb_idle());
		//printf_P(PSTR("Done\n"));
		//mpu_get_agt_int(&ax,&ay,&az,&gx,&gy,&gz,&temp);
		//printf("%d %d %d\n",ax,ay,az);
		//printf_P(PSTR("all issued\n"));
		
		// Without callback
		
		unsigned long sc;
		sc=0;
		tt1 = timer_ms_get();
		while( (tt2=timer_ms_get())-tt1<1000)
		{
			//mpu_get_agt_int(&ax,&ay,&az,&gx,&gy,&gz,&temp);
			r = i2c_transaction_queue(2,1,&t1,&t2);
			sc++;
		}
		printf_P(PSTR("Blocking I2C interrupt time-based t: %ld->%ld. #%ld. rps: %ld\n"),tt1,tt2,sc,sc*1000/(tt2-tt1));
		
		tt1=timer_ms_get();
		for(unsigned i=0;i<1000;i++)
		{
			//mpu_get_agt_int(&ax,&ay,&az,&gx,&gy,&gz,&temp);
			r = i2c_transaction_queue(2,1,&t1,&t2);
		}
		tt2=timer_ms_get();
		printf_P(PSTR("Blocking I2C interrupt sample-based t: %ld->%ld (%ld). #1000. rps: %ld\n\n"),tt1,tt2,tt2-tt1,1000l*1000l/(tt2-tt1));
		
		
		// With callback
		/*
		sc=0;
		t1 = timer_ms_get();
		while( (t2=timer_ms_get())-t1<1000)
		{
			mpu_get_agt_int_cb(mpu_test_sample3_cb2);
			while(!mpu_get_agt_int_cb_idle());
			sc++;
		}
		printf_P(PSTR("Callback I2C interrupt time-based t: %ld->%ld. #%ld. rps: %ld\n"),t1,t2,sc,sc*1000/(t2-t1));
		
		t1=timer_ms_get();
		for(unsigned i=0;i<1000;i++)
		{
			mpu_get_agt_int(&ax,&ay,&az,&gx,&gy,&gz,&temp);
		}
		t2=timer_ms_get();
		printf_P(PSTR("Callback I2C interrupt sample-based t: %ld->%ld (%ld). #1000. rps: %ld\n\n"),t1,t2,t2-t1,1000l*1000l/(t2-t1));
		*/
		//_delay_ms(5000);
		//_delay_ms(2);
		
	}
	
		
}


void mpu_test_benchmark_read(void)
{
	// Benchmark sampling speed.
	signed short ax,ay,az,gx,gy,gz,temp;
	unsigned long tt1,tt2;
	unsigned char r;
	
	
	
	
	mpu_set_interruptpin(0x00);
	
	mpu_printregdesc(file_usb);
	
	mpu_mode_accgyro(1,0,MPU_GYR_LPF_5,1,MPU_ACC_LPF_5,255);					// LPF 5Hz, ODR=1000/256=3.9Hz
	//mpu_mode_accgyro(0,0,MPU_GYR_LPF_250,1,MPU_ACC_LPF_5,255);					// LPF 5Hz, ODR=1000/256=3.9Hz
	mpu_set_interrutenable(0,0,0,1);
	
	mpu_get_agt_int_init();
	
	
	while(1)
	{
		printf_P(PSTR("Start of acquisition\n"));
		//mpu_get_agt_int_cb(mpu_test_sample3_cb);
		//mpu_get_agt_int_cb(mpu_test_sample3_cb);
		//while(!mpu_get_agt_int_cb_idle());
		//printf_P(PSTR("Done\n"));
		//mpu_get_agt_int(&ax,&ay,&az,&gx,&gy,&gz,&temp);
		//printf("%d %d %d\n",ax,ay,az);
		//printf_P(PSTR("all issued\n"));
		
		// Without callback
		
		unsigned long sc;
		
		sc=0;
		tt1 = timer_ms_get();
		while( (tt2=timer_ms_get())-tt1<1000)
		{
			mpu_get_agt(&ax,&ay,&az,&gx,&gy,&gz,&temp);
			sc++;
		}
		printf_P(PSTR("Blocking I2C polling time-based t: %ld->%ld. #%ld. rps: %ld\n"),tt1,tt2,sc,sc*1000/(tt2-tt1));
		
		tt1=timer_ms_get();
		for(unsigned i=0;i<1000;i++)
		{
			mpu_get_agt(&ax,&ay,&az,&gx,&gy,&gz,&temp);
		}
		tt2=timer_ms_get();
		printf_P(PSTR("Blocking I2C polling sample-based t: %ld->%ld (%ld). #1000. rps: %ld\n\n"),tt1,tt2,tt2-tt1,1000l*1000l/(tt2-tt1));
		
		
		sc=0;
		tt1 = timer_ms_get();
		while( (tt2=timer_ms_get())-tt1<1000)
		{
			r = mpu_get_agt_int(&ax,&ay,&az,&gx,&gy,&gz,&temp);
			sc++;
		}
		printf_P(PSTR("Blocking transactional I2C interrupt time-based t: %ld->%ld. #%ld. rps: %ld\n"),tt1,tt2,sc,sc*1000/(tt2-tt1));
		
		tt1=timer_ms_get();
		for(unsigned i=0;i<1000;i++)
		{
			r = mpu_get_agt_int(&ax,&ay,&az,&gx,&gy,&gz,&temp);
		}
		tt2=timer_ms_get();
		printf_P(PSTR("Blocking transactional I2C interrupt sample-based t: %ld->%ld (%ld). #1000. rps: %ld\n\n"),tt1,tt2,tt2-tt1,1000l*1000l/(tt2-tt1));
		
		printf_P(PSTR("%d %d %d %d %d %d %d\n"),ax,ay,az,gx,gy,gz,temp);
		
		
		// With callback
		
		sc=0;
		tt1 = timer_ms_get();
		while( (tt2=timer_ms_get())-tt1<1000)
		{
			mpu_get_agt_int_cb(mpu_test_sample3_cb2);
			while(mpu_get_agt_int_cb_busy());
			sc++;
		}
		printf_P(PSTR("Callback transactional I2C interrupt time-based t: %ld->%ld. #%ld. rps: %ld\n"),tt1,tt2,sc,sc*1000/(tt2-tt1));
		
		tt1=timer_ms_get();
		for(unsigned i=0;i<1000;i++)
		{
			mpu_get_agt_int_cb(mpu_test_sample3_cb2);
			while(mpu_get_agt_int_cb_busy());
		}
		tt2=timer_ms_get();
		printf_P(PSTR("Callback transactional I2C interrupt sample-based t: %ld->%ld (%ld). #1000. rps: %ld\n\n"),tt1,tt2,tt2-tt1,1000l*1000l/(tt2-tt1));
		
		//_delay_ms(5000);
		//_delay_ms(2);
		
	}
	
		
}


void mpu_test_queue(void)
{
	// Benchmark sampling speed.
	signed short ax,ay,az,gx,gy,gz,temp;
	unsigned long tt1,tt2;
	unsigned char r;
	
	
	
	
	mpu_set_interruptpin(0x00);
	
	mpu_printregdesc(file_usb);
	
	mpu_mode_accgyro(1,0,MPU_GYR_LPF_5,1,MPU_ACC_LPF_5,255);					// LPF 5Hz, ODR=1000/256=3.9Hz
	//mpu_mode_accgyro(0,0,MPU_GYR_LPF_250,1,MPU_ACC_LPF_5,255);					// LPF 5Hz, ODR=1000/256=3.9Hz
	mpu_set_interrutenable(0,0,0,1);
	
	mpu_get_agt_int_init();
	
	
	printf_P(PSTR("Test queuing\n"));
	
	for(unsigned char i=0;i<100;i++)
	{
		r = mpu_get_agt_int_cb(mpu_test_sample3_cb2);
		printf_P(PSTR("%d: %d. %d. %d. %d %d\n"),i,r,mpu_get_agt_int_cb_busy(),i2c_transaction_getqueued(),_i2c_transaction_buffer_rd,_i2c_transaction_buffer_wr);
	}
	while(r=mpu_get_agt_int_cb_busy())
	{
		printf_P(PSTR("%d. %d. %d. %d\n"),r,i2c_transaction_getqueued(),_i2c_transaction_buffer_rd,_i2c_transaction_buffer_wr);
	}
	
	printf_P(PSTR("Done\n"));
	
		
		
	
		
}

void mpu_test_intread7_cb(unsigned char status,unsigned char error,signed short ax,signed short ay,signed short az,signed short gx,signed short gy,signed short gz,signed short temp)
{
	unsigned long int time = timer_ms_get();

	char str[256];

	sprintf(str,"%05ld %03d %03d %05d %05d %05d %05d %05d %05d %05d\n",time,status,error,ax,ay,az,gx,gy,gz,temp);
	
	uart0_fputbuf_int(str,strlen(str));
}
void mpu_test_intread3_cb(unsigned char status,unsigned char error,signed short x,signed short y,signed short z)
{
	unsigned long int time = timer_ms_get();

	char str[256];

	sprintf(str,"%04ld %03d %03d %05d %05d %05d\n",time,status,error,x,y,z);
	
	uart0_fputbuf_int(str,strlen(str));
}


void mpu_test_intread(void)
{
	signed short ax,ay,az,gx,gy,gz,temp;
	unsigned char r;
	
	printf_P(PSTR("MPU reset\n"));
	mpu_reset();
	_delay_ms(100);
	
	printf_P(PSTR("MPU subsystem init\n"));
	mpu_get_agt_int_init();	
	mpu_set_interruptpin(0x00);
	mpu_set_interrutenable(0,0,0,0);																// Deactivate interrupts, benchmark uses timer interrupt instead
	
	//do_sample=0;
	mpu_set_interrutenable(0,0,0,1);																// Deactivate interrupts, benchmark uses timer interrupt instead


	//unsigned char sensorsr[] = {5,11,18,24};
	unsigned char sensorsr[] = {5,11,18,23};


		
	//for(unsigned char i=0;i<4;i++)
	for(unsigned char i=3;i<4;i++)
	{		
		unsigned char ssr = sensorsr[i];
	
		printf_P(PSTR("Setting up mode %d\n"),ssr);
			
			
		
		// Config the sensor in one of the pre-defined settings
		config_motion(ssr);
		
		// Enable temp
		//mpu_mode_temp(1);
		//mpu_mode_clksel(0);
		//mpu_mode_clksel(1);
		//mpu_mode_clksel(7);
		//mpu_mode_sleep(0);
		//mpu_mode_gyrostby(1);
		
		//mpu_set_interruptpin(0x00);
				
		mpu_printregdesc2(file_usb);
		
		unsigned duration=2000;
		
		
		unsigned long int tcur,tlast;
		
		printf_P(PSTR("------------- INT+BLOCKING -------------\n"));
		printf_P(PSTR("mpu_get_agt\n"));
		for(unsigned char j=0;j<5;j++)
		{
			
			unsigned long int t1,t2;
			t1 = timer_ms_get();
			r=mpu_get_agt_int(&ax,&ay,&az,&gx,&gy,&gz,&temp);
			t2 = timer_ms_get();
			printf_P(PSTR("r: %d dt: %ld. intctr: %ld\n"),r,t2-t1,motion_int_ctr2);
			printf_P(PSTR("%05d %05d %05d (%04X %04X %04X)  %05d %05d %05d (%04X %04X %04X)   %05d (%04X) (%05d)\n"),ax,ay,az,ax,ay,az,gx,gy,gz,gx,gy,gz,temp,temp,mpu_convtemp(temp));
			_delay_ms(duration);
		}
		printf_P(PSTR("mpu_get_a\n"));
		for(unsigned char j=0;j<5;j++)
		{
			unsigned long int t1,t2;
			t1 = timer_ms_get();
			r=mpu_get_a_int(&ax,&ay,&az);
			t2 = timer_ms_get();
			printf_P(PSTR("r: %d dt: %ld. intctr: %ld\n"),r,t2-t1,motion_int_ctr2);
			printf_P(PSTR("%05d %05d %05d (%04X %04X %04X)\n"),ax,ay,az,ax,ay,az);
			_delay_ms(duration);
		}
		printf_P(PSTR("mpu_get_g\n"));
		for(unsigned char j=0;j<5;j++)
		{
			unsigned long int t1,t2;
			t1 = timer_ms_get();
			r=mpu_get_g_int(&gx,&gy,&gz);
			t2 = timer_ms_get();
			printf_P(PSTR("r: %d dt: %ld. intctr: %ld\n"),r,t2-t1,motion_int_ctr2);
			printf_P(PSTR("%05d %05d %05d (%04X %04X %04X)\n"),gx,gy,gz,gx,gy,gz);
			_delay_ms(duration);
		}
		printf_P(PSTR("------------- INT+CALLBACK -------------\n"));
		
		printf_P(PSTR("mpu_get_agt_int_cb\n"));
		for(unsigned char j=0;j<5;j++)
		{
			unsigned long int t1,t2;
			t1 = timer_ms_get();
			r=mpu_get_agt_int_cb(mpu_test_intread7_cb);
			t2 = timer_ms_get();
			printf_P(PSTR("r: %d. dt: %ld. startt: %ld. intctr: %ld\n"),r,t2-t1,t1,motion_int_ctr2);
			_delay_ms(duration);
		}
		printf_P(PSTR("mpu_get_a_int_cb\n"));
		for(unsigned char j=0;j<5;j++)
		{
			unsigned long int t1,t2;
			t1 = timer_ms_get();
			r=mpu_get_a_int_cb(mpu_test_intread3_cb);
			t2 = timer_ms_get();
			printf_P(PSTR("r: %d. dt: %ld. startt: %ld. intctr: %ld\n"),r,t2-t1,t1,motion_int_ctr2);
			_delay_ms(duration);
		}
		printf_P(PSTR("mpu_get_g_int_cb\n"));
		for(unsigned char j=0;j<5;j++)
		{
			unsigned long int t1,t2;
			t1 = timer_ms_get();
			r=mpu_get_g_int_cb(mpu_test_intread3_cb);
			t2 = timer_ms_get();
			printf_P(PSTR("r: %d. dt: %ld. startt: %ld. intctr: %ld\n"),r,t2-t1,t1,motion_int_ctr2);
			_delay_ms(duration);
		}
	}
	printf_P(PSTR("Done\n"));
}

