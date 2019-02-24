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


void ledtest()
{
	//char pin=3;
	
	for(char pin=0;pin<=6;pin++)
		PIOPinMode(pin,PIOMODE_OUTPUT);
	while(1)
	{
		for(char pin=0;pin<=6;pin++)
			PIODigitalWrite(pin,0);
		_delay_ms(500);
		for(char pin=0;pin<=6;pin++)
			PIODigitalWrite(pin,1);
		_delay_ms(500);
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
	
	//ledtest();
		
			
	mode_main();			// This never returns.
	
	// This is never executed.
	while(1);
	return 0;
}







   







