#include "cpu.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <stdio.h>
#include <string.h>

#include "global.h"
#include "adc.h"
#include "serial.h"
#include "serial1.h"
#include "i2c.h"
#include "ds3232.h"
#include "rn41.h"
#include "mpu.h"
#include "mpu_test.h"
#include "pkt.h"
#include "wait.h"
#include "init.h"
#include "uiconfig.h"
#include "helper.h"
#include "ufat.h"
#include "i2c_internal.h"
#include "dbg.h"
#include "mpu-usart0.h"
#include "system.h"
#include "boot.h"
#include "interface.h"
#include "commandset.h"
#include "mode_sample_adc.h"
#include "mode_bench.h"
#include "mode_idle.h"
#include "mode_bt.h"
#include "mode_sd.h"
#include "mode_coulomb.h"
#include "mode_sample_motion.h"
#include "mode_motionrecog.h"
#include "mode_teststream.h"
#include "mode_testsync.h"
#include "mode_mputest.h"
#include "mode_demo.h"
#include "mode_demo_clock.h"
#include "ltc2942.h"
#include "mpu_config.h"
#include "wait.h"
#include "mode.h"
#include "a3d.h"
#include "pio.h"






/*void test_timer1(void)
{
	unsigned char hour,min,sec;
	// Sync to RTC
	ds3232_readtime_sync_conv(&hour,&min,&sec);
	cli();
	rtc_time_sec=0;
	rtc_time=0;
	cpu_time=0;
	timer_init(0);
	TCNT1=0;
	TCNT2=0;
	sei();
	
	while(1)
	{
		_delay_ms(5);
		unsigned long t = timer_ms_get();
		printf("%05lu %05lu\n",rtc_time_sec,t);
	}
}


void test_timer(void)
{
	unsigned long t1,t2;
	while(1)
	{
		t1 = rtc_time;
		t2 = cpu_time;
		fprintf(file_usb,"%lu %lu %ld\n",t1,t2,t1-t2);
		_delay_ms(10);
	}
}
*/



void rtc_status(FILE *f)
{
	printf("RTC registers:\n");
	ds3232_printreg(f);
}




unsigned char echo_uart1_rx_callback(unsigned char c)
{
	dbg_fputchar(c,0);				// Echo on USB
	//fputc(c, file_usb);
	
	
	return 1;										// Keep byte
	//return 0;										// Discard byte
}



void test_pio_adc(void)
{
while(1)
	{
		unsigned short adc[8];
		
		for(int pin=0;pin<=3;pin++)
		{
			fprintf(file_pri,"====== PIN %d ======\n",pin);
			
			for(int i=0;i<5;i++)
			{		
				ADCReadMulti(0xff,adc);
				for(int c=0;c<8;c++)
					fprintf(file_pri,"%d ",adc[c]);
				fprintf(file_pri,"\n");
				
				_delay_ms(250);
			}
			
			// Set pin to output
			fprintf(file_pri,"Set to output 0\n");
			PIOPinMode(pin,PIOMODE_OUTPUT);
			PIODigitalWrite(pin,0);
			
			for(int i=0;i<5;i++)
			{		
				ADCReadMulti(0xff,adc);
				for(int c=0;c<8;c++)
					fprintf(file_pri,"%d ",adc[c]);
				fprintf(file_pri,"\n");
				
				_delay_ms(250);
			}
			
			// Set pin to output
			fprintf(file_pri,"Set to output 1\n");
			PIOPinMode(pin,PIOMODE_OUTPUT);
			PIODigitalWrite(pin,1);
			
			for(int i=0;i<5;i++)
			{		
				ADCReadMulti(0xff,adc);
				for(int c=0;c<8;c++)
					fprintf(file_pri,"%d ",adc[c]);
				fprintf(file_pri,"\n");
				
				_delay_ms(250);
			}
			
			// Set pin to inpt
			fprintf(file_pri,"Set to input no pull up\n");
			PIOPinMode(pin,PIOMODE_INPUT);
			
			for(int i=0;i<5;i++)
			{		
				ADCReadMulti(0xff,adc);
				for(int c=0;c<8;c++)
					fprintf(file_pri,"%d ",adc[c]);
				fprintf(file_pri,"\n");
				
				_delay_ms(250);
			}
			
			// Set pin to inpt
			fprintf(file_pri,"Set to input pull up\n");
			PIOPinMode(pin,PIOMODE_INPUTPULLUP);
			
			for(int i=0;i<5;i++)
			{		
				ADCReadMulti(0xff,adc);
				for(int c=0;c<8;c++)
					fprintf(file_pri,"%d ",adc[c]);
				fprintf(file_pri,"\n");
				
				_delay_ms(250);
			}
		}
	}
}

