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
#include <stdlib.h>

#include "main.h"
#include "helper.h"
#include "wait.h"
#include "command.h"
#include "commandset.h"
#include "ds3232.h"
#include "system.h"
#include "uiconfig.h"
#include "init.h"
#include "dbg.h"
#include "interface.h"
#include "mode_global.h"
#include "ltc2942.h"
#include "mode.h"
#include "ufat.h"

// Command help

const char help_z[] PROGMEM ="Z[,<hh><mm><ss>]: without parameters resets local time; otherwise sets the RTC and local time to hhmmss";
//const char help_zsyncfromrtc[] PROGMEM ="z sync system time from RTC time";
const char help_y[] PROGMEM ="Test sync";
const char help_demo[] PROGMEM ="Demo mode";
const char help_c[] PROGMEM ="Clock mode";
const char help_w[] PROGMEM ="Swap primary and secondary interfaces";
const char help_i[] PROGMEM ="I<period>,<txslots>: Sets USB IO parameters. IO routine is called at (period+1)/1024Hz; txslots transmit slots are used before receiving data.";
const char help_r[] PROGMEM ="RN-41 terminal";
const char help_b[] PROGMEM ="B,<if>: Benchmark IO. if=0 for USB, 1 for BT";
const char help_l[] PROGMEM ="L,<en>: en=1 to enable LCD, 0 to disable";
const char help_t[] PROGMEM ="T[,<hh><mm><ss>] Query or set time";
const char help_ttest[] PROGMEM ="Time-related tests";
const char help_d[] PROGMEM ="D[,<dd><mm><yy>] Query or set date";
const char help_quit[] PROGMEM ="Exit current mode";
const char help_h[] PROGMEM ="Help";
const char help_a[] PROGMEM ="A,<hex>,<us>: ADC mode. hex: ADC channel bitmask in hex; us: sample period in microseconds";
const char help_s[] PROGMEM ="S,<us>: test streaming/logging mode; us: sample period in microseconds";
const char help_f[] PROGMEM ="F,<bin>,<pktctr>,<ts>,<bat>,<label>: bin: 1 for binary, 0 for text; for others: 1 to stream, 0 otherwise";
const char help_M[] PROGMEM ="M[,<mode>[,<logfile>[,<duration>]]: without parameters lists available modes, otherwise enters the specified mode.\n\t\tOptionally logs to logfile (use -1 not to log) and runs for the specified duration in seconds.";
const char help_m[] PROGMEM ="MPU test mode";
const char help_g[] PROGMEM ="G,<mode> enters motion recognition mode. The parameter is the sample rate/channels to acquire. Use G? to find more about modes";
const char help_o[] PROGMEM ="O[,sec] Power off and no wakeup, or wakeup after sec seconds";
const char help_coulomb[] PROGMEM ="Coulomb counter test mode";
const char help_sd[] PROGMEM ="SD card test mode";
const char help_power[] PROGMEM ="P,<low>: with low=1 the power supply enters low-power (max 10mA) mode; low=0 for normal operation";
const char help_identify[] PROGMEM ="Identify device by blinking LEDs";
const char help_annotation[] PROGMEM ="N,<number>: sets the current annotation";
const char help_bootscript[] PROGMEM ="b[,run;a boot;script] Prints the current bootscript or sets a new one; multiple commands are delimited by a ;";
const char help_info[] PROGMEM ="i,<ien> Prints battery and logging information when streaming/logging when ien=1";
const char help_battery[] PROGMEM="Battery info";
const char help_powertest[] PROGMEM="Power tests";


unsigned CurrentAnnotation=0;

/******************************************************************************
	CommandParser format
*******************************************************************************	
Parser must return 0 for success, 1 for error decoding command
0=OK
1=INVALID
Exec must return 0 for success, 1 for error executing command	
******************************************************************************/
/*
New command format:

	0:	Message execution ok (message valid)
	1:	Message execution error (message valid)
	2:	Message invalid 
*/
const unsigned char CommandParsersDefaultNum=4;
const COMMANDPARSER CommandParsersDefault[4] =
{ 
	{'T', CommandParserTime},
	{'D', CommandParserDate},
	{'S', CommandParserSuccess,0},
	{'E', CommandParserError,0},
};

unsigned char __CommandQuit=0;
/*
	Parsers must return:
		0:	Message execution ok (message valid)
		1:	Message execution error (message valid)
		2:	Message invalid 		
*/

unsigned char CommandParserTime(char *buffer,unsigned char size)
{
	unsigned char h,m,s;
	if(size==0)
	{
		// Query time
		ds3232_readtime_conv_int(1,&h,&m,&s);	
		fprintf_P(file_pri,PSTR("%02d:%02d:%02d\n"),h,m,s);
		return 0;	
	}
	// Set time
	if(size!=7 || buffer[0]!=',')
		return 2;
	
	buffer++;	// Skip comma

		
	// Check the digits are in range
	if(checkdigits(buffer,6))
		return 2;
		
	
	h = (buffer[0]-'0')*10+(buffer[1]-'0');
	m = (buffer[2]-'0')*10+(buffer[3]-'0');
	s = (buffer[4]-'0')*10+(buffer[5]-'0');
	
	fprintf_P(file_dbg,PSTR("Time: %02d:%02d:%02d\n"),h,m,s);
	
	if(h>23 || m>59 || s>59)
	{
		return 2;
	}
	
	// Execute
	unsigned char rv = ds3232_writetime(h,m,s);
	if(rv==0)
		return 0;
	return 1;
}

unsigned char CommandParserDate(char *buffer,unsigned char size)
{
	unsigned char d,m,y;
	if(size==0)
	{
		// Query date
		ds3232_readdate_conv_int(&d,&m,&y);	
		fprintf_P(file_pri,PSTR("%02d.%02d.%02d\n"),d,m,y);
		return 0;	
	}
	// Set date
	if(size!=7 || buffer[0]!=',')
		return 2;
	
	buffer++;	// Skip comma
		
	// Check the digits are in range
	if(checkdigits(buffer,6))
		return 2;
		
	d = (buffer[0]-'0')*10+(buffer[1]-'0');
	m = (buffer[2]-'0')*10+(buffer[3]-'0');
	y = (buffer[4]-'0')*10+(buffer[5]-'0');
	
	fprintf_P(file_dbg,PSTR("Date: %02d.%02d.%02d\n"),d,m,y);
	
	if(d>31 || d<1 || m>12 || m<1 || y>99)
	{
		return 2;
	}
		
	unsigned char rv = ds3232_writedate_int(1,d,m,y);
	if(rv==0)
		return 0;
	return 1;
}

unsigned char CommandParserTime_Test(char *buffer,unsigned char size)
{
	/*unsigned char h,m,s;
	if(size==0)
	{
		// Query time
		ds3232_readtime_conv_int(&h,&m,&s);	
		fprintf_P(file_pri,PSTR("%02d:%02d:%02d\n"),h,m,s);
		return 0;	
	}
	// Set time
	if(size!=7 || buffer[0]!=',')
		return 2;
	
	buffer++;	// Skip comma

		
	// Check the digits are in range
	if(checkdigits(buffer,6))
		return 2;
		
	
	h = (buffer[0]-'0')*10+(buffer[1]-'0');
	m = (buffer[2]-'0')*10+(buffer[3]-'0');
	s = (buffer[4]-'0')*10+(buffer[5]-'0');
	
	fprintf_P(file_dbg,PSTR("Time: %02d:%02d:%02d\n"),h,m,s);
	
	if(h>23 || m>59 || s>59)
	{
		return 2;
	}
	
	// Execute
	unsigned char rv = ds3232_writetime(h,m,s);
	if(rv==0)
		return 0;
	return 1;*/
	
	// Demonstrate the second transition at the falling edge of the NT#/SQW signal.
	unsigned char h,m,s;
	unsigned long t1,t2,t3;
	t1=timer_ms_get_intclk();
	while(timer_ms_get_intclk()-t1<3000)
	{
		ds3232_readtime_conv(&h,&m,&s);
		printf("%02d:%02d:%02d %d\n",h,m,s,(PINA>>6)&1);
		_delay_ms(100);
	}
	printf("\n");
	while(1)
	{
		unsigned char p1,p2;
		// Wait for falling edge
		p1=(PINA>>6)&1;
		while(1)
		{
			p2=(PINA>>6)&1;
			if(p1!=p2 && p2==0)
				break;
			p1=p2;
		}
		_delay_ms(750);
		ds3232_writetime(10,11,12);
	}
	
	// Wait for 
	for(unsigned char i=0;i<3;i++)
	{
		unsigned char p1,p2;
		
		// Wait for falling edge
		p1=(PINA>>6)&1;
		while(1)
		{
			p2=(PINA>>6)&1;
			if(p1!=p2 && p2==0)
				break;
			p1=p2;
		}
		t1 = timer_ms_get_intclk();
		// Wait for another falling edge
		p1=(PINA>>6)&1;
		while(1)
		{
			p2=(PINA>>6)&1;
			if(p1!=p2 && p2==0)
				break;
			p1=p2;
		}
		t2 = timer_ms_get_intclk();
		printf("dt1: %ld\n",t2-t1);
		ds3232_readtime_conv(&h,&m,&s);
		printf("%02d:%02d:%02d %d\n",h,m,s,(PINA>>6)&1);
		// Wait 300ms
		_delay_ms(50);
		printf("dt2: %ld\n",timer_ms_get_intclk()-t1);
		ds3232_readtime_conv(&h,&m,&s);
		printf("dt2b: %ld\n",timer_ms_get_intclk()-t1);
		printf("+%02d:%02d:%02d %d\n",h,m,s,(PINA>>6)&1);
		// Set the time
		printf("dt2c: %ld\n",timer_ms_get_intclk()-t1);
		unsigned long tt1,tt2;
		tt1=timer_ms_get_intclk();
		printf("b%d",(PINA>>6)&1);
		ds3232_writetime(10,11,12);
		printf("a%d\n",(PINA>>6)&1);
		tt2=timer_ms_get_intclk();
		printf("dt2d: %ld tt: %ld\n",timer_ms_get_intclk()-t1,tt2-tt1);
		ds3232_readtime_conv(&h,&m,&s);
		printf("dt2e: %ld\n",timer_ms_get_intclk()-t1);
		printf("*%02d:%02d:%02d %d\n",h,m,s,(PINA>>6)&1);
		
		printf("dt3: %ld\n",timer_ms_get_intclk()-t1);
		
		// Wait for another falling edge
		p1=(PINA>>6)&1;
		while(1)
		{
			p2=(PINA>>6)&1;
			if(p1!=p2 && p2==0)
				break;
			p1=p2;
		}
		t3 = timer_ms_get_intclk();
		printf("dt4: %ld\n",t3-t1);
		ds3232_readtime_conv(&h,&m,&s);
		printf("%02d:%02d:%02d %d\n",h,m,s,(PINA>>6)&1);
		
		_delay_ms(2500);
	}
	/*unsigned long ctr=0;
	while(1)
	{
		ctr++;
		_delay_ms(77);
		unsigned char rv;
		if( (ctr&0xF) ==0)
			rv=ds3232_writetime(10,11,12);
		unsigned long tt1=timer_ms_get();
		unsigned long tt1s,tt2s,tt11000,tt21000;
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			tt1s = _timer_time_1_in_ms;
			tt11000 = _timer_time_1000;
		}
		printf("%ld %ld.%ld (%d)\n",tt1,tt1s,tt11000,rv);
		rv=0x55;
	}*/
	/*for(int i=0;i<100;i++)
	{
		_delay_ms(rand()%1000);
		unsigned long tt1=timer_ms_get_intclk();
		unsigned long tt1s,tt2s,tt11000,tt21000;
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			tt1s = _timer_time_1_in_ms;
			tt11000 = _timer_time_1000;
		}
		
		unsigned char p1 = (PINA>>6)&1;
		ds3232_writetime(10,11,12);
		unsigned char p2 = (PINA>>6)&1;
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			tt2s = _timer_time_1_in_ms;
			tt21000 = _timer_time_1000;
		}
		unsigned long tt2=timer_ms_get_intclk();
		printf("dt: %ld (%ld %ld) %ld.%ld -> %ld.%ld %d->%d\n",tt2-tt1,tt1,tt2,tt1s,tt11000,tt2s,tt21000,p1,p2);
	}*/
	
	
	
	return 0;
}

// Always success
unsigned char CommandParserQuit(char *buffer,unsigned char size)
{
	__CommandQuit=1;
	return 0;
}
// Always success
unsigned char CommandParserHelp(char *buffer,unsigned char size)
{
	fprintf_P(file_pri,PSTR("Available commands:\n"));
	for(unsigned char i=0;i<CommandParsersCurrentNum;i++)
	{
		fprintf_P(file_pri,PSTR("\t%c\t"),CommandParsersCurrent[i].cmd);
		if(CommandParsersCurrent[i].help)
			fputs_P(CommandParsersCurrent[i].help,file_pri);
		fputc('\n',file_pri);
	}	
	return 0;
}
unsigned char CommandParserSuccess(char *buffer,unsigned char size)
{
	return 0;
}
unsigned char CommandParserError(char *buffer,unsigned char size)
{
	return 1;
}

unsigned char CommandParserLCD(char *buffer,unsigned char size)
{	
	int lcden;
	
	unsigned char rv = ParseCommaGetInt(buffer,1,&lcden);
	if(rv)
		return 2;
	
	// Store 
	system_enable_lcd = lcden;
	ConfigSaveEnableLCD(lcden);	
	fprintf_P(file_pri,PSTR("LCD enabled: %d\n"),lcden);
	if(lcden==1)
		init_lcd();
	else
		deinit_lcd();
	return 0;
}
unsigned char CommandParserInfo(char *buffer,unsigned char size)
{	
	int ien;
	
	unsigned char rv = ParseCommaGetInt(buffer,1,&ien);
	if(rv)
		return 2;
	
	// Store 
	ConfigSaveEnableInfo(ien);	
	fprintf_P(file_pri,PSTR("Info enabled: %d\n"),ien);
	return 0;
}

unsigned char CommandShouldQuit(void)
{
	unsigned char t=__CommandQuit;
	__CommandQuit=0;
	return t;
}

unsigned char CommandParserIO(char *buffer,unsigned char size)
{
	unsigned char rv;
	int period,txslot;
	
	rv = ParseCommaGetInt((char*)buffer,2,&period,&txslot);
	if(rv || period<0 || txslot<=0)
		return 2;

	// Check params, clamp at some reasonable value
	if(period>999)
		period=999;
	
	if(txslot>256)
		txslot=256;
		
	dbg_setioparam(period,txslot);
		
	return 0;
}
unsigned char CommandParserSwap(char *buffer,unsigned char size)
{
	interface_swap();
		
	return 0;
}
unsigned char CommandParserOff(char *buffer,unsigned char size)
{
	unsigned char rv;
	char *p1;
	int sec;
	rv = ParseComma((char*)buffer,1,&p1);
	if(rv)
	{
		fprintf_P(file_pri,PSTR("Shutting down, no wakeup\n"));
	}
	else
	{
		if(sscanf(p1,"%u",&sec)!=1)
		{
			return 2;
		}	
		fprintf_P(file_pri,PSTR("Shutting down, wakeup in %d s\n"),sec);
	}
	/*return 0;
	
	fprintf_P(file_pri,PSTR("Shutting down in... "));
	for(signed char i=3;i>=0;i--)
	{
		fprintf_P(file_pri,PSTR("%d... "),i);
		_delay_ms(1000);		
	}*/
	
	#if (HWVER==6) || (HWVER==7)
	// Read the charge and store it
	unsigned long charge = ltc2942_getcharge();
	eeprom_write_byte((uint8_t*)STATUS_ADDR_OFFCURRENT_CHARGE0,(charge>>0)&0xff);
	eeprom_write_byte((uint8_t*)STATUS_ADDR_OFFCURRENT_CHARGE1,(charge>>8)&0xff);
	eeprom_write_byte((uint8_t*)STATUS_ADDR_OFFCURRENT_CHARGE2,(charge>>16)&0xff);
	eeprom_write_byte((uint8_t*)STATUS_ADDR_OFFCURRENT_CHARGE3,(charge>>24)&0xff);
	
	// Reconfigure the DS3232M to turn off the one second oscillation
	ds3232_printreg(file_pri);
	//rtc_off();
	
	ds3232_alarm_in(sec);
	
	
	ds3232_printreg(file_pri);
	_delay_ms(1000);
	#endif
	
	system_off();
	return 0;
}


unsigned char CommandParserStreamFormat(char *buffer,unsigned char size)
{
	unsigned char rv;
	int bin,pktctr,ts,bat,label;
	
	//printf("string: '%s'\n",buffer);
	
	rv = ParseCommaGetInt((char*)buffer,5,&bin,&pktctr,&ts,&bat,&label);
	if(rv)
		return 2;
	//printf("%d %d %d %d %d\n",bin,pktctr,ts,bat,label);
		
	
	bin=bin?1:0;
	pktctr=pktctr?1:0;
	ts=ts?1:0;
	bat=bat?1:0;
	label=label?1:0;
				
	
	mode_stream_format_bin = bin;	
	mode_stream_format_ts = ts;		
	mode_stream_format_bat = bat;		
	mode_stream_format_label = label;
	mode_stream_format_pktctr = pktctr;
	
	fprintf_P(file_pri,PSTR("bin: %d. pktctr: %d ts: %d bat: %d label: %d\n"),bin,pktctr,ts,bat,label);
	
	ConfigSaveStreamBinary(bin);
	ConfigSaveStreamPktCtr(pktctr);
	ConfigSaveStreamTimestamp(ts);
	ConfigSaveStreamBattery(bat);
	ConfigSaveStreamLabel(label);
		
	return 0;
}

unsigned char CommandParserPower(char *buffer,unsigned char size)
{
	unsigned char rv;
	char *p1;
	rv = ParseComma((char*)buffer,1,&p1);
	if(rv)
		return 2;
	unsigned low;
	if(sscanf(p1,"%u",&low)!=1)
	{
		printf("failed low\n");
		return 2;
	}	
	if(!(low==0 || low==1))
		return 2;
	if(low==0)
		system_power_normal();
	else
		system_power_low();
	
	return 0;
}
/*
	Some power tests
	
	Available sleep modes:
	SLEEP_MODE_IDLE				// 20mW reduction
	SLEEP_MODE_PWR_DOWN			// Cannot wake up
	SLEEP_MODE_PWR_SAVE			// Cannot wake up
	SLEEP_MODE_STANDBY			// Cannot wake up
	SLEEP_MODE_EXT_STANDBY		// Cannot wake up
	
	
*/
unsigned char CommandParserPowerTest(char *buffer,unsigned char size)
{
	unsigned long t1,tlast,tcur,stat_loop;
	fprintf_P(file_pri,PSTR("Power tests\n"));
	
	// Unregister lifesign
	timer_unregister_slowcallback(system_lifesign);
	// Turn off all LEDs
	system_led_set(0b000);

	/*fprintf_P(file_pri,PSTR("Busy loop, LED off\n"));	
	t1=tlast=timer_ms_get();
	stat_loop=0;
	while(1)
	{
		tcur=timer_ms_get();
		if(tcur-t1>20000)
			break;
		stat_loop++;
		if(tcur-tlast>=10000)
		{
			fprintf_P(file_pri,PSTR("%lu loops. %s\n"),stat_loop,ltc2942_last_strstatus());
			tlast=tcur;
			stat_loop=0;
		}
	}
	
	system_led_set(0b111);

	fprintf_P(file_pri,PSTR("Busy loop, LED on\n"));	
	t1=tlast=timer_ms_get();
	stat_loop=0;
	while(1)
	{
		tcur=timer_ms_get();
		if(tcur-t1>20000)
			break;
		stat_loop++;
		if(tcur-tlast>=10000)
		{
			fprintf_P(file_pri,PSTR("%lu loops. %s\n"),stat_loop,ltc2942_last_strstatus());
			tlast=tcur;
			stat_loop=0;
		}
	}*/
	
	/*system_led_set(0b000);
	
	fprintf_P(file_pri,PSTR("Busy loop with idle sleep, LED off\n"));
	set_sleep_mode(SLEEP_MODE_IDLE); 		// 20mW reduction

	sleep_enable();
	t1=tlast=timer_ms_get();
	stat_loop=0;
	while(1)
	{
		sleep_cpu();
		tcur=timer_ms_get();
		if(tcur-t1>30000)
			break;
		stat_loop++;
		if(tcur-tlast>=10000)
		{
			fprintf_P(file_pri,PSTR("%lu loops. %s\n"),stat_loop,ltc2942_last_strstatus());
			tlast=tcur;
			stat_loop=0;
		}
	}*/


	/*fprintf_P(file_pri,PSTR("Busy loop with idle sleep, LED off, analog off\n"));
	set_sleep_mode(SLEEP_MODE_IDLE); 		// 20mW reduction
	ACSR=0x10;		// Clear analog compare interrupt (by setting ACI/bit4 to 1) and clear interrupt enable (setting ACIE/bit3 to 0)
	ACSR=0x80;		// Disable analog compare	(setting ACD/bit7 to 1)
	fprintf_P(file_pri,PSTR("ACSR: %02X\n"),ACSR);

	sleep_enable();
	t1=tlast=timer_ms_get();
	stat_loop=0;
	while(1)
	{
		sleep_cpu();
		tcur=timer_ms_get();
		if(tcur-t1>30000)
			break;
		stat_loop++;
		if(tcur-tlast>=10000)
		{
			fprintf_P(file_pri,PSTR("%lu loops. %s\n"),stat_loop,ltc2942_last_strstatus());
			tlast=tcur;
			stat_loop=0;
		}
	}*/

	fprintf_P(file_pri,PSTR("Busy loop with idle sleep, LED off, analog off\n"));
	set_sleep_mode(SLEEP_MODE_IDLE); 		// 20mW reduction
	fprintf_P(file_pri,PSTR("ASSR: %02X\n"),ASSR);
	fprintf_P(file_pri,PSTR("PRR0: %02X\n"),PRR0);
	PRR0 = 0b01100001;
	fprintf_P(file_pri,PSTR("PRR0: %02X\n"),PRR0);
	

	sleep_enable();
	t1=tlast=timer_ms_get();
	stat_loop=0;
	while(1)
	{
		sleep_cpu();
		tcur=timer_ms_get();
		if(tcur-t1>30000)
			break;
		stat_loop++;
		if(tcur-tlast>=10000)
		{
			fprintf_P(file_pri,PSTR("%lu loops. %s\n"),stat_loop,ltc2942_last_strstatus());
			tlast=tcur;
			stat_loop=0;
		}
	}
	
	fprintf_P(file_pri,PSTR("Power tests done\n"));
	
	timer_register_slowcallback(system_lifesign,0);
	
	return 0;
}

/******************************************************************************
	function: CommandParserSync
*******************************************************************************	
	Receive hhmmss time, update the RTC and synchronise the local time to the 
	RTC.
	
	This function operates in two steps:
	- 	First, it immediately sets the RTC time - this ensures the RTC is synchronised
		to the provided time.
	-	Then, it synchronises the local time to the RTC using system_settimefromrtc.
		
	Parameters:
		buffer	-		Pointer to the command string
		size	-		Size of the command string

	Returns:
		0		-		Success
		1		-		Message execution error (message valid)
		2		-		Message invalid 

******************************************************************************/
unsigned char CommandParserSync(char *buffer,unsigned char size)
{
	if(size!=0)
	{
		// Parameters are passed: check validity
		if(size!=7 || buffer[0]!=',')
			return 2;
		
		buffer++;	// Skip comma

		// Check the digits are in range
		if(checkdigits(buffer,6))
			return 2;
			
		unsigned char h,m,s;
		h = (buffer[0]-'0')*10+(buffer[1]-'0');
		m = (buffer[2]-'0')*10+(buffer[3]-'0');
		s = (buffer[4]-'0')*10+(buffer[5]-'0');
		
		//fprintf_P(file_dbg,PSTR("Time: %02d:%02d:%02d\n"),h,m,s);
		
		if(h>23 || m>59 || s>59)
		{
			return 2;
		}
		
		// Update the RTC time
		ds3232_writetime(h,m,s);
		// Synchronise local time to RTC time
		system_settimefromrtc();
		return 0;
	}
	timer_init(0);
	return 0;
}
unsigned char CommandParserSyncFromRTC(char  *buffer,unsigned char size)
{
	system_settimefromrtc();
	return 0;
}
unsigned char CommandParserTestSync(char *buffer,unsigned char size)
{
	fprintf_P(file_pri,PSTR("Test sync command\n"));
	CommandChangeMode(APP_MODE_TESTSYNC);
	return 0;
}

unsigned char CommandParserIdentify(char *buffer,unsigned char size)
{
	fprintf(file_pri,"My name is %s\n",system_getdevicename());
	system_blink(10,100,0);
	return 0;
}


unsigned char CommandParserAnnotation(char *buffer,unsigned char size)
{
	unsigned char rv;
	char *p1;
	rv = ParseComma((char*)buffer,1,&p1);
	if(rv)
		return 2;
	unsigned a;
	if(sscanf(p1,"%u",&a)!=1)
	{
		//printf("failed low\n");
		return 2;
	}	
	
	CurrentAnnotation=a;
	
	return 0;
}

unsigned char CommandParserMPUTest(char *buffer,unsigned char size)
{
	CommandChangeMode(APP_MODE_MPUTEST);
	return 0;
}
/******************************************************************************
	function: CommandParserBootScript
*******************************************************************************	
	Parses the boot script command. 
	
	If no parameter is passed it prints the current boot script.
	
	If a parameter is passed, the rest of the string after the comma is stored
	as boot script.
	
	Parameters:
		buffer	-		Pointer to the command string
		size	-		Size of the command string

	Returns:
		0		-		Success
		1		-		Message execution error (message valid)
		2		-		Message invalid 

******************************************************************************/
unsigned char CommandParserBootScript(char *buffer,unsigned char size)
{
	//printf("size: %d\n",size);
	if(size!=0)
	{
		// If the first character is not a comma return an error.
		if(buffer[0]!=',')
			return 2;
		// Skip the comma, the size decreases accordingly.
		buffer++;
		size--;
		// The maximum scrip length is CONFIG_ADDR_SCRIPTLEN-2, as the last byte must be NULL and the one prior must be a command delimiter
		if(size>CONFIG_ADDR_SCRIPTLEN-2)
		{
			fprintf_P(file_pri,PSTR("Boot script too long\n"));
			return 1;
		}
		// Add a terminating newline (semicolon)
		buffer[size++]=';';
		buffer[size++]=0;
		//printf("saving %d bytes\n",size);
		ConfigSaveScript((char*)buffer,size);
	}
	
	// Read and print script
	char buf[CONFIG_ADDR_SCRIPTLEN];
	ConfigLoadScript(buf);
	fprintf_P(file_pri,PSTR("Boot script: \""));
	prettyprint_hexascii(file_pri,buf,strlen(buf),0);
	fprintf_P(file_pri,PSTR("\"\n"));
	
	return 0;
}
unsigned char CommandParserBatteryInfo(char *buffer,unsigned char size)
{
	fprintf(file_pri,"#%s\n",ltc2942_last_strstatus());
	for(unsigned char i=0;i<LTC2942NUMLASTMW;i++)
		fprintf(file_pri,"%d ",ltc2942_last_mWs(i));
	fprintf(file_pri,"\n");

	return 0;
}

void CommandChangeMode(unsigned char newmode)
{
	//if(system_mode!=newmode)
		__CommandQuit=1;
	system_mode = newmode;
}









