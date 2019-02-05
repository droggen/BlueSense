/*
	file: mode
	
	Main 'application mode' files, handling the main loop waiting for user commands.
	
		
*/

#include "cpu.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/eeprom.h>
#include <util/atomic.h>
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
#include "pkt.h"
#include "wait.h"
#include "init.h"
#include "lcd.h"
#include "fb.h"
#include "uiconfig.h"
#include "helper.h"
#include "i2c_internal.h"
#include "system.h"
#include "pkt.h"
#include "sd.h"
#include "ufat.h"

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

#include "mode.h"


void mode_main(void)
{
	
	while(1)
	{
		//fprintf_P(file_pri,PSTR("Current mode: %d\n"),system_mode);
		//_delay_ms(500);
		//system_status_ok(system_mode);
		//_delay_ms(500);
		switch(system_mode)
		{
			
			case APP_MODE_IDLE:
				mode_idle();
				break;			
			case 1:
				//stream();
				//mpu_testsleep();
				system_mode=0;
				break;
			case 2:
				//main_log();
				system_mode=0;
				break;
			case APP_MODE_CLOCK:
			#if ENABLEGFXDEMO==1
//				mode_clock();
			#endif
				system_mode=0;
				break;
			case APP_MODE_DEMO:
			#if ENABLEGFXDEMO==1
	//			mode_demo();
			#endif
				system_mode=0;
				break;
			case APP_MODE_BENCHIO:
				//mode_bench();
				system_mode=0;
				break;
			case APP_MODE_BT:
				mode_bt();
				system_mode=0;
				break;
			case APP_MODE_MOTIONSTREAM:
				mode_motionstream();
				system_mode=0;
				break;
			case APP_MODE_MPUTEST:
				mode_mputest();
				system_mode=0;
				break;
			case APP_MODE_MOTIONRECOG:
				//mode_motionrecog();
				system_mode=0;
				break;
			#if ENABLEMODECOULOMB==1
			case APP_MODE_COULOMB:
				mode_coulomb();
				system_mode=0;
				break;
			#endif
			case APP_MODE_SD:
				mode_sd();
				system_mode=0;
				break;
			case APP_MODE_TS:
				mode_teststream();
				system_mode=0;
				break;
			case APP_MODE_TESTSYNC:
				printf("mode testsync\n");
				mode_testsync();
				system_mode=0;
				break;

			case APP_MODE_ADC:
			default:
				mode_adc();
				system_mode=0;
				break;
		}
	}
	
}
