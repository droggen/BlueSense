/*
	file: Main file
	
	todo
	- ADC sample multiple channels in A mode
	- ADC sample/return multiple channels
	- Increase the size of the command buffer
	- Reactivate the bootloader
	- Label in the motion mode
	- Add test in wait function that would allow to detect non-monoticity/wraparound of uS counter
	- Log becoming an system-wide functionality that redirects all serial comm to file?
	
	
	1) Auto-detect SD card on boot?
	H 2) Motion stream mode: two parameters: mode and logging 
	3) Power measurements in background
	4) PC interface
	5) Default mode (e.g. to automatically stream or enter demo or clock mode)
	6) Background blinking
	7) Speedup boot
	8) Download logs?
	9) Minimise flash usage
	10) Fix warnings in codebase
	V 11) Malloc pulled in from fdevopen ? 
	12) Script to launch at start
	12.1: CommandGet: must be split in one command reading from file_pri and filling internal buffer, and one reading internal buffer until it is empty
	
	
	Not linking unused functions: http://www.avrfreaks.net/forum/not-linking-unused-functions
	
	Tests on HW6:
	- SD card
	- Interface connect/disconnect and interface swap
	- ADC
	
	
	
	




	
*/
/*
TODO: ds3232_writetime / commandset.CommandParserSync: must check whether setting the second clock of the RTC is reset by writing to the registers, otherwise must wait for a rising/falling edge of rtc clock to set the time

TODO-FIXED:	bug in motion acquisition buffering: when the MPU interrupt callback triggers the strategy is to discard the oldest sample when the motion buffer is full with a call to rdnext. However
		the user app may be currently accessing that oldest sample. Need to have a new interrupt-blocking function which transfers data from the interrupt buffer to user code.
		
TODO:	bug in the mode_sample_motion quaternion (which may have no effect): acceleration is converted into mg using a fixed conversion factor, which is not reflecting the settings of the accelerometer range

TODO-DONE:	Time synchronisation including epoch
TODO-DONE:	Time synchronisation including epoch on boot
TODO:	Verify time synchronisation including epoch
TODO-DONE:	PC program to synchronise clocks
TODO:	Verify sample rate regularity with high sample rate modes (200Hz-1KHz)
TODO:	Readout of files (never practical due to slow transfer)
TODO:	PC program to converto to quaternions
TODO:	Magnetic field: check effect of quantisation and sample rate (8/100) on quaternions
TODO:	Improve idle bluetooth power efficiency w/ inquiry and scan pages (SI,S
J)
TODO-DONE:	RN41: power optimisation of radio
TODO:	RN41 I/O ports best settings ?
TODO:	Check stability of magnetic field calibration
TODO:	Reactivate the bootloader....
TODO-DONE:	Upon time synchronisation ensure the system LED blink synchronously
TODO:	Check sampling returns calibrated magnetic field
TODO:	Convert to quaternion when logging without, taking into account dt
TODO:	Identify the regular >50ms hicckups in the logging
TODO:	Sometimes error writing log 4 (F,10) on S10 - ?
BUG:	Stopeeing logging and quitting motion sampling mode with ! does not write battery statistics data unlike stopping logging with L first, then quitting with !
*/


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

#include "main.h"
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

FILE *file_bt;			// Bluetooth
FILE *file_usb;			// USB
FILE *file_fb;			// Screen
FILE *file_dbg;			// Debug (assigned to file_bt or file_usb)
FILE *file_pri;			// Primary (assigned to file_bt or file_usb)

unsigned char sharedbuffer[520];


// Device name
unsigned char system_devname[5];

// Configuration parameters
unsigned char config_enable_id,config_enable_timestamp,config_enable_battery,config_enable_temperature,config_enable_acceleration,config_enable_gyroscope,config_enable_checksum,config_data_format;
unsigned char config_sensorsr;
unsigned char config_streaming_format;


// Other
unsigned char system_mode=APP_MODE_IDLE;				// idle
//unsigned char system_mode=3;			// clock
//unsigned char system_mode=4;			// demo
//unsigned char system_mode=APP_MODE_ADC;			// adc
//unsigned char system_mode=APP_MODE_BT;



// System status
unsigned long int system_perf=0;
volatile signed short system_temperature=0;
volatile unsigned long int system_temperature_time=0;
signed short system_offdeltacharge;

/*void uiprintf(const char *fmt,...)
{
	va_list args;	
	va_start(args,fmt);
	vfprintf(file_bt,fmt,args);
//	vfprintf(file_dbg,fmt,args);
}*/

















/******************************************************************************
	function: main_perfbench
*******************************************************************************	
	Benchmarks the processor, here using simple counting. Use to assess
	CPU load during sampling.
	
	Parameters:
		mintime		-		Minimum duration of the benchmark in ms. 
							If 0 (default if no parameters) does a 1000ms test.
	
	Return value:	performance result
******************************************************************************/
unsigned long main_perfbench(unsigned long mintime)
{
	unsigned long int t_last,t_cur;
	unsigned long int ctr,cps;
	MPUMOTIONDATA mpumotiondata;
	ctr=0;
	/*
	// Using millisecond timer
	t_last=timer_ms_get();
	while((t_cur=timer_ms_get())-t_last<=mintime)
	{
		ctr++;
	}
	cps = ctr*1000/(t_cur-t_last);*/
	
	t_last=timer_s_wait();
	mpu_clearstat();
	unsigned long tint1=timer_ms_get_intclk();
	while((t_cur=timer_s_get())-t_last<mintime)
	{
		ctr++;
		
		// Simulate reading out the data from the buffers
		unsigned char l = mpu_data_level();
		for(unsigned char i=0;i<l;i++)
		{
			// Get the data from the auto read buffer; if no data available break
			if(mpu_data_getnext_raw(mpumotiondata))
				break;
			//_mpu_data_rdnext();
		}		
	}
	unsigned long tint2=timer_ms_get_intclk();
	cps = ctr/(t_cur-t_last);
	
	fprintf_P(file_pri,PSTR("main_perfbench: %lu perf (%lu intclk ms)\n"),cps,tint2-tint1);
	
	return cps;
}



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













/******************************************************************************
	system_gettemperature
*******************************************************************************	
	Returns the temperature from the DS332 in hundredth of degree.
	The temperature is sampled in the background through a timer callback.
	
******************************************************************************/
signed short system_gettemperature(void)
{
	signed short t;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		t = system_temperature;	
	}
	return t;
}
/******************************************************************************
	system_gettemperaturetime
*******************************************************************************	
	Returns the temperature from the DS332 in hundredth of degree.
	The temperature is sampled in the background through a timer callback.	
******************************************************************************/
unsigned long system_gettemperaturetime(void)
{
	unsigned long t;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		t = system_temperature_time;	
	}
	return t;
}

unsigned char system_sampletemperature(unsigned char p)
{
	ds3232_readtemp_int_cb(system_sampletemperature_cb);
	return 0;
}

void system_sampletemperature_cb(unsigned char s,signed short t)
{
	system_temperature = t;
	system_temperature_time = timer_ms_get();
}







extern void stk500(void);

/*void boottest(void)
{
	fprintf(file_bt,"Boot test\n");
	fprintf(file_bt,"Setting passing register\n");
	fprintf(file_bt,"Calling in 1\n");
	_delay_ms(1000);
	//boot_dbg();
	boot_bluetooth();
	//void boot_bluetooth(void);
	fprintf(file_bt,"Done\n");
	while(1);
}*/

/******************************************************************************
	function: bootloaderhook_dbg
*******************************************************************************	
	Hook checking the data received on the usb interface for the byte indicating
	that an stk500v2 bootloader message is received. 
	In which case, enter the bootloader	
	
	Parameters:
		c			-		Character received on the interface
	Returns:
		1			-		To place the character in the receive buffer
		otherwise	-		The character is discarded
******************************************************************************/
unsigned char bootloaderhook_dbg(unsigned char c)
{
	// Enters the bootloader on reception of 0x1B
	if(c==0x1B)
	//if(c=='a')		// to help debugging
	{
		for(char i=0;i<5;i++)
		{
			system_led_set(0b000);
			_delay_ms(50);
			system_led_set(0b111);
			_delay_ms(50);
		}
		boot_dbg();
	}
	return 1;
}
unsigned char bootloaderhook_bluetooth(unsigned char c)
{
	// Enters the bootloader on reception of 0x1B
	if(c==0x1B)
		boot_bluetooth();
	return 1;
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


/******************************************************************************
Main program loop
******************************************************************************/
int main(void)
{
	// Initialises the port IO, timers, interrupts and "printf" debug interface over I2C
	init_basic();
	
	// Initialise the rest of the system
	init_extended();

	//init_wdt();

	
	// test_pio_adc();
	
	

	/*while(1)
	{
		fprintf(file_pri,"%d\n",system_isusbconnected());
		_delay_ms(100);
	}
	*/
	
	
	mode_main();			// This never returns.
	
	// This is never executed.
	while(1);
	return 0;
}







   







