#if ENABLEGFXDEMO==1

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

#include "main.h"
#include "adc.h"
#include "serial.h"
#include "i2c.h"
#include "ds3232.h"
#include "rn41.h"
#include "mpu.h"
#include "mpu_test.h"
#include "test_mmc.h"
#include "pkt.h"
#include "wait.h"
#include "init.h"
#include "ui.h"
#include "lcd.h"
#include "fb.h"
#include "uiconfig.h"
#include "helper.h"
#include "i2c_internal.h"
#include "motionconfig.h"

#include "mode_demo.h"
#include "commandset.h"

#define CommandParsersDemoNum 2
const COMMANDPARSER CommandParsersDemo[CommandParsersDemoNum] =
{ 
	{'!', CommandParserQuit,help_quit},
	{'H', CommandParserHelp,help_h}
};


void mode_demo(void)
{
	unsigned long int t1,t2;
	
	while(1)
	{
		lcd_clear565(0);
		if(demo_clock(10000))
			return;
		
		lcd_clear565(0);		
		if(demo_acc(30000))
			return;
		
		lcd_clear565(0);		
		if(demo_gyr(30000))
			return;
		
		
		
		/*unsigned char r,g,b;
		unsigned short color;
		
		for(unsigned char c=0;c<32;c+=4)
		{
			r=c;
			//r=0;
			g=c<<1;
			//g=0;
			b=c;
			//b=0;
			color=r;
			color<<=11;
			color|=(g<<5);
			color|=b;
			
			printf("about to call clear565. i: %d. color: %04X\n",c,color);
			t1=timer_ms_get();
			lcd_clear565(color);
			t2=timer_ms_get();
			printf("Time: %ld\n",t2-t1);
			
			//_delay_ms(500);
		}*/
		
		
	}
	
	
}



char demo_acc(unsigned long int dt)
{
	signed short dx[129],dy[129],dz[129];
	
	memset(dx,0,129*sizeof(signed short));
	memset(dy,0,129*sizeof(signed short));
	memset(dz,0,129*sizeof(signed short));
	
	lcd_writestring("Acceleration",16,0,2,0x0000,0xffff);	
	
	
	
	mpu_get_agt_int_init();	
	mpu_reset();
	_delay_ms(100);
	//mpu_set_interruptpin(0x00);
	//mpu_set_interrutenable(0,0,0,0);				
	//config_motion(11);		// 10Hz with 5Hz BW
	//config_motion(10);		// 50Hz with 20Hz BW
	mpu_config_motionmode(MPU_MODE_100HZ_ACC_BW41,0);		// 100Hz with 41Hz BW
	
	
	//printf("rand max: %d\n",RAND_MAX);
	// randMax: 32767
	
	// Simulate a sine wave
	/*or(int i=0;i<128;i++)
	{
		dx[i] = (i%32)-16;
	}
	
	// Plot pixels
	for(int i=0;i<128;i++)
	{
		lcd_pixel(i,64+dx[i],0xffff);
	}*/
	
	//while(1);
	unsigned long int t1;
	
	t1 = timer_ms_get();
	
	while( (dt==0) || (dt!=0 && (timer_ms_get()-t1<dt) ) )
	{
		
		while(CommandProcess(&CommandParsersDemo,CommandParsersDemoNum));
		if(CommandShouldQuit())
			return 1;		
		
		
		// Time left
		if(dt)
		{
			char s[32];
			sprintf(s,"%06ld",dt-(timer_ms_get()-t1));
			lcd_writestring(s,52,122,1,0x0000,0xffff);	
		}
		
		
		// Read sensor
		signed short ax,ay,az;
		mpu_get_a(&ax,&ay,&az);
		
		
		if(ax>32768) ax=32768;
		if(ax<-32768) ax=-32768;
		if(ay>32768) ay=32768;
		if(ay<-32768) ay=-32768;
		if(az>32768) az=32768;
		if(az<-32768) az=-32768;
		
		// Simulate get new data
		//int nd = rand()&0x3f; // 0-63
		//nd-=32;
		
		// must be in range [-32;+32]
		//int nd = ax>>10;
		
		
		
		dx[128] = ax>>9;			
		dy[128] = ay>>9;			
		dz[128] = az>>9;			
		
		/*dx[128] = ax>>8;			
		dy[128] = ay>>8;			
		dz[128] = az>>8;			*/
		
		// from n to 0, erase at n, draw at n+1
		for(int i=127;i>=0;i--)
		{
			lcd_pixel(i,64+dx[i],0x0000);
			lcd_pixel(i,64+dy[i],0x0000);
			lcd_pixel(i,64+dz[i],0x0000);
			
			lcd_pixel(i,64+dx[i+1],0b1111100000000000);			
			lcd_pixel(i,64+dy[i+1],0b0000011111100000);
			lcd_pixel(i,64+dz[i+1],0b0000000000011111);
		}
		for(int i=0;i<128;i++)
		{
			dx[i]=dx[i+1];
			dy[i]=dy[i+1];
			dz[i]=dz[i+1];
		}
		
		
		//_delay_ms(1);
		
	}
	
	
	return 0;
}


char demo_gyr(unsigned long int dt)
{
	signed short dx[129],dy[129],dz[129];
	
	memset(dx,0,129*sizeof(signed short));
	memset(dy,0,129*sizeof(signed short));
	memset(dz,0,129*sizeof(signed short));
	
	
	lcd_writestring("Gyroscope",28,0,2,0x0000,0xffff);	
	
	mpu_get_agt_int_init();	
	mpu_reset();
	_delay_ms(100);
	//mpu_set_interruptpin(0x00);
	//mpu_set_interrutenable(0,0,0,0);				
	mpu_config_motionmode(MPU_MODE_50HZ_GYRO_BW20,0);		// 50Hz with 20Hz BW
	
	
	//printf("rand max: %d\n",RAND_MAX);
	// randMax: 32767
	
	// Simulate a sine wave
	/*or(int i=0;i<128;i++)
	{
		dx[i] = (i%32)-16;
	}
	
	// Plot pixels
	for(int i=0;i<128;i++)
	{
		lcd_pixel(i,64+dx[i],0xffff);
	}*/
	
	//while(1);
	unsigned long int t1;
	
	t1 = timer_ms_get();
	
	while( (dt==0) || (dt!=0 && (timer_ms_get()-t1<dt) ) )
	{
		while(CommandProcess(&CommandParsersDemo,CommandParsersDemoNum));
		if(CommandShouldQuit())
			return 1;
		
		// Time left
		if(dt)
		{
			char s[32];
			sprintf(s,"%06ld",dt-(timer_ms_get()-t1));
			lcd_writestring(s,52,122,1,0x0000,0xffff);	
		}
		
		// Read sensor
		signed short ax,ay,az;
		mpu_get_g(&ax,&ay,&az);
		
		
		// Simulate get new data
		//int nd = rand()&0x3f; // 0-63
		//nd-=32;
		
		// must be in range [-32;+32]
		//int nd = ax>>10;
		
		if(ax>16384) ax=16384;
		if(ax<-16384) ax=-16384;
		if(ay>16384) ay=16384;
		if(ay<-16384) ay=-16384;
		if(az>16384) az=16384;
		if(az<-16384) az=-16384;
		
		dx[128] = ax>>9;			
		dy[128] = ay>>9;			
		dz[128] = az>>9;			
		
		/*dx[128] = ax>>8;			
		dy[128] = ay>>8;			
		dz[128] = az>>8;			*/
		
		// from n to 0, erase at n, draw at n+1
		for(int i=127;i>=0;i--)
		{
			lcd_pixel(i,64+dx[i],0x0000);
			lcd_pixel(i,64+dy[i],0x0000);
			lcd_pixel(i,64+dz[i],0x0000);
			
			lcd_pixel(i,64+dx[i+1],0b1111100000000000);			
			lcd_pixel(i,64+dy[i+1],0b0000011111100000);
			lcd_pixel(i,64+dz[i+1],0b0000000000011111);
		}
		for(int i=0;i<128;i++)
		{
			dx[i]=dx[i+1];
			dy[i]=dy[i+1];
			dz[i]=dz[i+1];
		}
		
		
		//_delay_ms(1);
		
	}
	
	return 0;
	
}


#endif