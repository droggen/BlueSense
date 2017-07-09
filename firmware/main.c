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
	
	
	
	

	When "idle": must automatically detect which channel is used to send commands
	Must print information to both channels (if connected).

	What should be the features of the firmware?
	
	- Upon connect, auto stream data
	- set time
	- configure: sample rate, format, etc
	- start recording 
	- stop recording
	- list files
	- dump files
	



	
*/
/*
TODO: ds3232_writetime / commandset.CommandParserSync: must check whether setting the second clock of the RTC is reset by writing to the registers, otherwise must wait for a rising/falling edge of rtc clock to set the time

TODO-FIXED:	bug in motion acquisition buffering: when the MPU interrupt callback triggers the strategy is to discard the oldest sample when the motion buffer is full with a call to rdnext. However
		the user app may be currently accessing that oldest sample. Need to have a new interrupt-blocking function which transfers data from the interrupt buffer to user code.
		
TODO:	bug in the mode_sample_motion quaternion (which may have no effect): acceleration is converted into mg using a fixed conversion factor, which is not reflecting the settings of the accelerometer range
*/
/*
	Power:
		1. Idle, BTconn: 124/115mW
		2. Idle, BTnoconn: 61/57mW
		Idle, BTnoconn, quiet nondisc nonconn (Q): 42/39
		Idle, BTnoconn, quiet nondisc conn (Q,2): 49
		Idle, BTnoconn, SI,0012, SJ,0012 : 43/42	(discovery difficult) 43
		Idle, BTnoconn, SI,0019, SJ,0019 : 44/43	(discovery ok)	(Seems to be an ideal setting in idle: -15mW compared to default)
		Idle, BTnoconn, SI,0020, SJ,0020 : 44/43	(conn ok)
		Idle, BTnoconn, SI,0100, SJ,0100 : 61/59 	(default)
		Idle, BTnoconn, SI,0800, SJ,0800 : 177/167	(max window; power decreases after connection!)
		Idle, BTconn, SI,0012, SJ,0012 : 121/118
		Idle, BTnoconn, SI,0020, SJ,0020 : 118/115 (conn ok)
		Idle, BTnoconn, S@,1000:			61/58
		Idle, BTnoconn, SI,0019, SJ,0019, S|,0401:	x (connection difficult)
		Idle, BTnoconn, SI,0050, SJ,0050, S|,0401:	x (connection difficult)
		Idle, BTnoconn, SI,0100, SJ,0100, S|,0401:	49 (conn ok with moderate delay)
		Idle, BTnoconn, SI,0019, SJ,0019, S|,0301:	42/40
		Idle, BTnoconn, SI,0019, SJ,0019, S|,0201:	42 (conn ok)
		Idle, BTnoconn, SI,0019, SJ,0019, S|,0101:	43/42 (conn ok)
		Idle, BTnoconn, SW,9900:					(4 sec deep sleep, connection doesn't work) |
		Idle, BTnoconn, SW,8C80:					(2 sec deep sleep, connection doesn't work)	|	Not clear if needs a reset or not, or a sniff enabled dongle
		Idle, BTnoconn, SW,8640:					64/58 (1 sec deep sleep, conn ok)			|

		Idle, BTnoconn, S|,0101: 59
		Idle, BTnoconn, S|,0201: 55/51
		Idle, BTnoconn, S|,0801: 48/42						(long connection time, ~30 sec)
		S1. Idle w/sleep, BTnoconn, S|,0801: 23/21			(long connection time, ~30 sec)
		S2. Idle w/sleep, BTconn, S|,0801: 107 (should be identical to S4)
		S3. Idle w/ sleep, BTnoconn, 39/36
		S4. Idle w/ sleep, BTconn, 109/102
		
		
		TODO: S%,1000 (used on power up) or S@,1000 (used instantaneously)
		
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
#include "motionconfig.h"
#include "wait.h"
#include "mode.h"

FILE *file_bt;			// Bluetooth
FILE *file_usb;			// USB
FILE *file_fb;			// Screen
FILE *file_dbg;			// Debug (assigned to file_bt or file_usb)
FILE *file_pri;			// Primary (assigned to file_bt or file_usb)

unsigned char sharedbuffer[520];

volatile unsigned char bluetoothrts=0;

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




// Strings











/******************************************************************************
	Timer interrupt vectors
******************************************************************************/
// CPU 1024Hz
ISR(TIMER1_COMPA_vect)
{
	// check whether higher priority interrupts are there.
	/*if(UCSR1A&(1<<RXC1))
	{
		USART1_RX_vect_core();
	}*/
	//wdt_reset();
	_timer_tick_1024hz();
}
// RTC 1024Hz
#if HWVER==1
ISR(TIMER2_COMPA_vect)
{
	//_timer_tick_1024hz();
}
#endif
// Pin change RTC
ISR(PCINT0_vect)
{
	//dbg_fputchar_nonblock('1',0);
	// Check pin state; second counter incremented on falling edge
	if(!(PINA&0b01000000))
	{
		_timer_tick_hz();
	}
}
// Pin change motion
ISR(PCINT2_vect)
{
	//dbg_fputchar_nonblock('m',0);
	//motion_int_ctr2++;
	// React on motion_int pin
	if(PINC&0x20)
	{
		mpu_isr();
	}
}
// Pin change: BT connect (PD7) USB connect (PD6) and RTS (PD4)
ISR(PCINT3_vect)
{
	//char b[16];
	
	
	//static unsigned char lastpind=0;
	unsigned char pind;
	
	pind = PIND;
	
	/*b[0]='P';
	b[1]=hex2chr((pind>>4)&0xf);
	b[2]=hex2chr(pind&0xf);
	b[3]='\n';
	b[4]=0;
	dbg_putbuf(b,4);*/
	

	// Check RTS change
	unsigned char rts = (pind&0x10)?1:0;
	if(rts != bluetoothrts)
	{
		Serial1RTSToggle(rts);
		//char buf[16]="rts 0\n";
		//buf[4]='0'+rts;
		//fputbuf(file_usb,buf,5);
		PORTC&=0b10111111;
		PORTC|=(rts<<6);
	}
	bluetoothrts = rts;		

	// Check connection change
	signed char cur_bt_connected,cur_usb_connected=-1;
	cur_bt_connected = system_isbtconnected();		
	#if (HWVER==5) || (HWVER==6) || (HWVER==7)
	cur_usb_connected = system_isusbconnected();	
	#endif	
	interface_signalchange(cur_bt_connected,cur_usb_connected);
}





/******************************************************************************
	function: main_perfbench
*******************************************************************************	
	Benchmarks the processor, here using simple counting. Use to assess
	CPU load during sampling.
	
	Return value:	performance result
******************************************************************************/
unsigned long main_perfbench(void)
{
	unsigned long int t_last,t_cur;
	unsigned long int ctr,cps;
	const unsigned long int mintime=1000;
		
	ctr=0;
	t_last=timer_ms_get();
	while((t_cur=timer_ms_get())-t_last<mintime)
	{
		ctr++;
	}
	cps = ctr*1000/(t_cur-t_last);
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



/******************************************************************************
Main program loop
******************************************************************************/
int main(void)
{
	// Initialises the port IO, timers, interrupts and "printf" debug interface over I2C
	init_basic();
	
	// Initialise the rest of it
	init_extended();
	

	// 
	/*while(1)
	{
		fprintf(file_pri,"Battery: %ld V: %d C: %u lsb Q: %lu uAh T: %d\n",ltc2942_last_updatetime(),ltc2942_last_voltage(),ltc2942_last_chargectr(),ltc2942_last_charge(),ltc2942_last_temperature());
		
		fprintf(file_pri,"i: %ld mA P: %ld mW\n",_ltc2942_last_mA,_ltc2942_last_milliwatt);
		_delay_ms(500);
		
		// Benchmark the bettery conversion functions
		//unsigned long t1,t2;
		//t1 = timer_ms_get();
		//for(int i=0;i<1000;i++)
		//	__ltc2942_trans_read_done(0);
		//t2 = timer_ms_get();
		//printf("dt: %ld\n",t2-t1);
		
	}*/
	
	
	mode_main();			// This never returns.
	
	while(1);
	return 0;
}







   





