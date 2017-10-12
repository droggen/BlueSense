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

#include "mode.h"
#include "mode_demo.h"
#include "mode_sample_adc.h"
#include "mode_bench.h"
#include "mode_bt.h"

#include "commandset.h"
#include "mode_idle.h"
#include "mode_sample_motion.h"
#include "mode_motionrecog.h"
#include "mode_coulomb.h"
#include "mode_sd.h"
#include "mode_teststream.h"

const char help_x[] PROGMEM ="x";

unsigned char CommandParserx(char *buffer,unsigned char size)
{

	/*for(unsigned char i=0;i<5;i++)
	{
		fputs(_poweruse_off.str[i],file_pri);
	}*/
	
	
		
	return 0;
}



const COMMANDPARSER CommandParsersIdle[] =
{ 
	{'Z',CommandParserSync,help_z},
	//{'z',CommandParserSyncFromRTC,help_zsyncfromrtc},
	{'Y',CommandParserTestSync,help_y},
	{'R',CommandParserBT,help_r},
	{'L',CommandParserLCD,help_l},
	{'T', CommandParserTime,help_t},
	{'t', CommandParserTime_Test,help_ttest},
	{'D', CommandParserDate,help_d},
	{'H', CommandParserHelp,help_h},
	{'A', CommandParserADC,help_a},
	{'C', CommandParserClock,help_c},
	{'V', CommandParserDemo,help_demo},
	//{'B', CommandParserBench,help_b},
	{'I', CommandParserIO,help_i},
	{'M', CommandParserMotion,help_M},
	{'m', CommandParserMPUTest,help_m},
	//{'G', CommandParserMotionRecog,help_g},
	{'W', CommandParserSwap,help_w},
	{'O', CommandParserOff,help_O},
	{'o', CommandParserOffPower,help_o},
	{'F', CommandParserStreamFormat,help_f},
	{'i', CommandParserInfo,help_info},
#if ENABLEMODECOULOMB==1	
	{'c', CommandParserCoulomb,help_coulomb},
#endif
	{'Q', CommandParserBatteryInfoLong,help_batterylong},
	{'q', CommandParserBatteryInfo,help_battery},
	{'c', CommandParserCallback,help_callback},
	{'X', CommandParserSD,help_sd},
	//{'p', CommandParserPowerTest,help_powertest},
	{'S', CommandParserTeststream,help_s},
	{'b', CommandParserBootScript,help_bootscript},
	{'?', CommandParserIdentify,help_identify},
	{'~', CommandParserClearBootCounter,help_clearbootctr},
	{'x', CommandParserx,help_x}
};
const unsigned char CommandParsersIdleNum=sizeof(CommandParsersIdle)/sizeof(COMMANDPARSER);

unsigned char CommandParserClock(char *buffer,unsigned char size)
{
	CommandChangeMode(APP_MODE_CLOCK);
		
	return 0;
}
unsigned char CommandParserDemo(char *buffer,unsigned char size)
{
	CommandChangeMode(APP_MODE_DEMO);
		
	return 0;
}




void mode_idle(void)
{
	//char valid;
	//int c;	
	//unsigned long stat_timemsstart=0;
	//unsigned long stat_timemsend=0;
	//unsigned long time_laststatus=0;
	
	lcd_clear565(0);
	lcd_writestring("Idle",48,0,2,0x0000,0xffff);	
	
	fprintf_P(file_pri,PSTR("IDLE>\n"));
	
	
	set_sleep_mode(SLEEP_MODE_IDLE); 
	//set_sleep_mode(SLEEP_MODE_PWR_DOWN); 
	sleep_enable();
	
	//time_laststatus=stat_timemsstart = timer_ms_get();
	while(1)
	{
		//_delay_ms(1000);
		// Check connections
		/*if(!system_isbtconnected() && !system_isusbconnected())
		{
			//_delay_ms(10);	// Do something still
			//_delay_ms(2);	// Do something still
			_delay_us(250);	// Do something still
			// No active connections - go in deeper sleep
			set_sleep_mode(SLEEP_MODE_PWR_DOWN); 				// Problem of deeper sleeps: time is lost and must be recovered from the RTC
			//set_sleep_mode(SLEEP_MODE_IDLE); 		
		}
		else
		{
			// Some connection - go in idle
			set_sleep_mode(SLEEP_MODE_IDLE); 
		}*/
				
		sleep_cpu();
	//	fprintf_P(file_pri,PSTR("Some ADC stuff: %d. period: %d. mask: %02X. pri: %p dbg: %p\n"),ctr,mode_adc_period,mode_adc_mask,file_pri,file_dbg);

		CommandProcess(CommandParsersIdle,CommandParsersIdleNum);
		if(CommandShouldQuit())
			break;
			
		
		
		// Display debug status
		/*if(timer_ms_get()-time_laststatus>750)
		{
			fprintf_P(file_dbg,PSTR("Idle mode. PINA: %02X DDRA: %02X PORTA: %02X\n"),PINA,DDRA,PORTA);
			time_laststatus = timer_ms_get();
			
			//fprintf_P(file_usb,PSTR("toto usb\n"));
			//fprintf_P(file_bt,PSTR("toto bt\n"));
		}*/
	}
	fprintf_P(file_pri,PSTR("<IDLE\n"));
}