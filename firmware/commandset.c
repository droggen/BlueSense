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

const char help_z[] PROGMEM ="Z[,<hh><mm><ss>]: Resets the absolute time counter to zero, and optionally sets the RTC time";
const char help_y[] PROGMEM ="Test sync";
const char help_demo[] PROGMEM ="Demo mode";
const char help_c[] PROGMEM ="Clock mode";
const char help_w[] PROGMEM ="Swap primary and secondary interfaces";
const char help_i[] PROGMEM ="I<period>,<txslots>: Sets USB IO parameters. IO routine is called at (period+1)/1024Hz; txslots transmit slots are used before receiving data.";
const char help_r[] PROGMEM ="RN-41 terminal";
const char help_b[] PROGMEM ="B,<if>: Benchmark IO. if=0 for USB, 1 for BT";
const char help_l[] PROGMEM ="L,<en>: en=1 to enable LCD, 0 to disable";
const char help_t[] PROGMEM ="T[,<hh><mm><ss>] Query or set time";
const char help_d[] PROGMEM ="D[,<dd><mm><yy>] Query or set date";
const char help_quit[] PROGMEM ="Exit current mode";
const char help_h[] PROGMEM ="Help";
const char help_a[] PROGMEM ="A,<hex>,<us>: ADC mode. hex: ADC channel bitmask in hex; us: sample period in microseconds";
const char help_s[] PROGMEM ="S,<us>: test streaming/logging mode; us: sample period in microseconds";
const char help_f[] PROGMEM ="F,<bin>,<pktctr>,<ts>,<bat>,<label>: bin: 1 for binary, 0 for text; for others: 1 to stream, 0 otherwise";
const char help_m[] PROGMEM ="M[,<mode>] stream motion data in specified mode; use M for available modes";
const char help_n[] PROGMEM ="MPU test mode";
const char help_g[] PROGMEM ="G,<mode> enters motion recognition mode. The parameter is the sample rate/channels to acquire. Use G? to find more about modes";
const char help_o[] PROGMEM ="O[,sec] Power off and no wakeup, or wakeup after sec seconds";
const char help_coulomb[] PROGMEM ="Coulomb counter test mode";
const char help_sd[] PROGMEM ="SD card test mode";
const char help_power[] PROGMEM ="P,<low>: with low=1 the power supply enters low-power (max 10mA) mode; low=0 for normal operation";
const char help_identify[] PROGMEM ="Identify device by blinking LEDs";
const char help_annotation[] PROGMEM ="N,<number>: sets the current annotation";
const char help_bootscript[] PROGMEM ="b[,run;a boot;script] Prints the current bootscript or sets a new one; multiple commands are delimited by a ;";
const char help_info[] PROGMEM ="i,<ien> Enables additional info when ien=1";


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
	
	rtc_alarm_in(sec);
	
	
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

unsigned char CommandParserSync(char *buffer,unsigned char size)
{
	unsigned char rv;
	
	rv = 0;
	if(size!=0)
	{
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
		
		fprintf_P(file_dbg,PSTR("Time: %02d:%02d:%02d\n"),h,m,s);
		
		if(h>23 || m>59 || s>59)
		{
			return 2;
		}
		
		// Execute
		rv = ds3232_writetime(h,m,s);
	}	
	timer_init(0);
	fprintf_P(file_pri,PSTR("Time sync'd\n"));
	if(rv!=0)
		return 1;
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
unsigned char CommandParserBootScript(char *buffer,unsigned char size)
{

	printf("size: %d\n",size);
	if(size==0)
	{
		// Read and print script
		char buf[CONFIG_ADDR_SCRIPTLEN];
		ConfigLoadScript(buf);
		// Put a null at the end
		for(unsigned i=0;i<CONFIG_ADDR_SCRIPTLEN;i++)
			printf("%02X ",buf[i]);
		// Massage the script: replace newlines by semicolons
		/*for(unsigned char i=0;i<CONFIG_ADDR_SCRIPTLEN;i++)
			if(buf[i]=='\n') 
				buf[i]=';';*/
		fprintf_P(file_pri,PSTR("Boot script: '%s'\n"),buf);
	}
	else
	{
		if(buffer[0]!=',')
			return 2;
		buffer++;
		size--;
		if(size>CONFIG_ADDR_SCRIPTLEN-2)
		{
			fprintf_P(file_pri,PSTR("Boot script too long\n"));
			return 1;
		}
		// Add a terminating newline (semicolon)
		buffer[size++]=';';
		buffer[size++]=0;
		// Massage the script: replace semicolons by newlines
		/*for(unsigned char i=1;i<size;i++)
			if(buffer[i]==';') 
				buffer[i]='\n';*/
		// Save the message
		//ConfigSaveScript((char*)buffer,size+1);
		ConfigSaveScript((char*)buffer,size);
	}
	return 0;
}

void CommandChangeMode(unsigned char newmode)
{
	//if(system_mode!=newmode)
		__CommandQuit=1;
	system_mode = newmode;
}









