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

#include "rn41.h"
#include "serial.h"

#define RN41DEBUG 1


/*
	Notes about RN41:
		Changing a setting (i.e. ST for the configuration timeout) requires a reset either with "R,1" or with the reset button
		Setting the speed to 230.4 and higher leads to data overrun. Decreasing dbg callback frequency decrease the occurrence, but it still happens. Issue if receiving data (such as bootloader). Not if streaming out.
		F,1 to enable fast mode is only a valid command during connection, otherwise ERR
		ST with 0, 253, 254, 255 work as advertised. With another timeout the module still enters command mode
		
		Prefer to use S-,<name> to set the name of the device as it is serialisation friendly
		
		Rerunning the configuration (not clear which parameters: name, pin, mode, devclass, srvclass) requires to remove/readd the device on windows otherwise the connection fails. Likely some encryption key that changed.
		
		Streaming data: 
			115.2K, w/o RTS, ST,255: speed 11563-11565 bytes/sec, with RTS often going off, then RN41 crashes (1-2 bursts of 128KB, not reboot); RTS led goes off, blinks, then stays mostly off with sporadic on; connection stays on. PC must disconnect, but this fails, rn41 must be reset
			115.2K, w/  RTS, ST,255: RN41 reboots (notifies REBOOT) and drops connection, but less frequently (3-6 bursts), 11143 bytes/sec. After reboot, PC can reconnect successfully after disconnection timeout on PC side.
			115.2K, w/o RTS, ST,0: 	no visible discontinuity in RTS light; speed 11563 bytes/sec. No crash after 6 bursts.
			115.2K, w/  RTS, ST,0: 	no visible discontinuity in RTS light; speed 11564 bytes/sec. No crash after 6 bursts.
			
			
			230.4K, w/  RTS, ST,255: RTS flickers continuously, speed 166606-16956, reset on burst 3 and 4
			230.4K, w/o RTS, ST,255: twice, crash halfway through burst; node receive garbage (likely "reboot" at 115k2), disconnection
			230.4K, w/o RTS, ST,0: no visible discontinuity in RTS light; speed 22870-22874 bytes/sec. No crash after 6 bursts.
			230.4K, w/  RTS, ST,0: no visible discontinuity in RTS light; speed 22870-22875 bytes/sec. No crash after 6 bursts.		
			
			460.6K, w/  RTS, ST,255: RTS flickers continuously, speed 20805-21389; once AVR rebooted; other time disconnect after 30 bursts
			460.6K, w/o RTS, ST,255: twice crash halfway through burst; node receive garbage (likely "reboot" at 115k2), disconnection
			460.4K, w/o RTS, ST,0: no visible discontinuity in RTS light; speed 24828-22874 bytes/sec. No crash after 6 bursts.
			460.6K, w/  RTS, ST,0: no visible discontinuity in RTS light; speed 24481-24504 bytes/sec. No crash after 6 bursts.
		(above benchmark with dbg_callback 3. with dbg_callback 0 speed at 230k4 is 22500 instead of 22800; with dbg_callback 1 speed at 230k4 is 22600 instead of 22800 - negligible impact on transmit, but important on receive)
			
		Speeds higher than 115k2 lead to data overrun on reception in the AVR; issue: AVR not fast enough to read data and move to a buffer.
		Recommended: 230k4; although reception of data must be handled by a slow sender on PC side
*/

/*
	This requires:
	1) Interrupt reading function in fdevopen
	2) Enough buffer in rx buffer, as cts is not used.
*/

void rn41_CmdEnter(FILE *fileinfo,FILE *filebt)
{
	// No character 1 second before and after the enter command sequence
	_delay_ms(1100);
	rn41_CmdResp(fileinfo,filebt,"$$$");
}
void rn41_CmdLeave(FILE *fileinfo,FILE *filebt)
{
	// No character 1 second before and after the enter command sequence
	_delay_ms(1100);
	rn41_CmdResp(fileinfo,filebt,"---\n");
}
void rn41_CmdResp(FILE *fileinfo,FILE *filebt,char *cmd)
{
	char buffer[64];
	
	#if RN41DEBUG==1
		fprintf_P(fileinfo,PSTR("BT CmdResp '%s'\n"),cmd);
	#endif
	fputs(cmd,filebt);
	fgets(buffer,64,filebt);
	#if RN41DEBUG==1
		fprintf_P(fileinfo,PSTR("BT CmdResp returned %d bytes\n"),strlen(buffer));
	#endif
	buffer[strlen(buffer)-2] = 0;		// Strip cr-lf	
	#if RN41DEBUG==1
		fprintf_P(fileinfo,PSTR("BT CmdResp: %s\n"),buffer);
	#endif
}
void rn41_CmdRespn(FILE *fileinfo,FILE *filebt,char *cmd,unsigned char n)
{
	char buffer[256];
	//int l;
	
	//_delay_ms(1000);
	//l = fgettxbuflevel(filebt);
	//fprintf_P(fileinfo,PSTR("before cmd Buffer level: %d\n",l));
	
	fputs(cmd,filebt);
	//_delay_ms(1000);
	//l = fgettxbuflevel(filebt);
	//fprintf_P(fileinfo,PSTR("after cmd Buffer level: %d\n",l));
	for(unsigned char i=0;i<n;i++)
	{
		fgets(buffer,256,filebt);
		buffer[strlen(buffer)-2] = 0;		// Strip cr-lf
		fprintf_P(fileinfo,PSTR("CmdRespn %02d: %s\n"),i,buffer);
		//l = fgettxbuflevel(filebt);
	}
}

void rn41_Reset(FILE *fileinfo)
{
	#if RN41DEBUG==1
		fprintf_P(fileinfo,PSTR("BT hard reset\n"));
	#endif
	//fprintf_P(fileinfo,PSTR("PIND: %02X\n"),PIND);
	PORTA &= 0b11011111;
	//fprintf_P(fileinfo,PSTR("BT hard reset: set to 0\n"));
	//fprintf_P(fileinfo,PSTR("PIND: %02X\n"),PIND);
	//_delay_ms(5000);
	_delay_ms(200);
	//fprintf_P(fileinfo,PSTR("PIND: %02X\n"),PIND);
	//fprintf_P(fileinfo,PSTR("BT hard reset: set to 1\n"));
	PORTA |= 0b00100000;
	//fprintf_P(fileinfo,PSTR("PIND: %02X\n"),PIND);
	//fprintf_P(fileinfo,PSTR("rst4\n"));
	//_delay_ms(5000);
	_delay_ms(200);
	#if RN41DEBUG==1
		fprintf_P(fileinfo,PSTR("BT hard reset done\n"));
	#endif
}
unsigned char rn41_SetConfigTimer(FILE *fileinfo,FILE *filebt,unsigned char v)
{
	char buffer[64];
	sprintf(buffer,"ST,%d\n",v);
	rn41_CmdResp(fileinfo,filebt,buffer);
	return 0;
}
unsigned char rn41_SetPassData(FILE *fileinfo,FILE *filebt,unsigned char v)
{
	char buffer[64];
	sprintf(buffer,"T,%d\n",v?1:0);
	rn41_CmdResp(fileinfo,filebt,buffer);
	return 0;
}
unsigned char rn41_SetPIN(FILE *fileinfo,FILE *filebt,char *pin)
{
	char buffer[64];
	sprintf(buffer,"SP,%s\n",pin);
	rn41_CmdResp(fileinfo,filebt,buffer);
	return 0;
}
unsigned char rn41_SetTempBaudrate(FILE *fileinfo,FILE *filebt,char *b)
{
	char buffer[64];
	sprintf(buffer,"U,%s,N\n",b);
	rn41_CmdResp(fileinfo,filebt,buffer);
	return 0;
}
unsigned char rn41_SetName(FILE *fileinfo,FILE *filebt,char *name)
{
	char buffer[64];
	sprintf(buffer,"SN,%s\n",name);
	rn41_CmdResp(fileinfo,filebt,buffer);
	return 0;
}
unsigned char rn41_SetSerializedName(FILE *fileinfo,FILE *filebt,char *name)
{
	char buffer[64];
	sprintf(buffer,"S-,%s\n",name);
	rn41_CmdResp(fileinfo,filebt,buffer);
	return 0;
}
unsigned char rn41_SetAuthentication(FILE *fileinfo,FILE *filebt,unsigned char a)
{
	char buffer[64];
	sprintf(buffer,"SA,%d\n",a);
	rn41_CmdResp(fileinfo,filebt,buffer);
	return 0;
}
unsigned char rn41_SetMode(FILE *fileinfo,FILE *filebt,unsigned char a)
{
	char buffer[64];
	sprintf(buffer,"SM,%d\n",a);
	rn41_CmdResp(fileinfo,filebt,buffer);
	return 0;
}
unsigned char rn41_SetServiceClass(FILE *fileinfo,FILE *filebt,unsigned short c)
{
	char buffer[64];
	sprintf(buffer,"SC,%04X\n",c);
	rn41_CmdResp(fileinfo,filebt,buffer);
	return 0;
}
unsigned char rn41_SetDeviceClass(FILE *fileinfo,FILE *filebt,unsigned short c)
{
	char buffer[64];
	sprintf(buffer,"SD,%04X\n",c);
	rn41_CmdResp(fileinfo,filebt,buffer);
	return 0;
}
unsigned char rn41_GetBasic(FILE *fileinfo,FILE *filebt)
{
	rn41_CmdRespn(fileinfo,filebt,"D\n",9);
	return 0;
}
unsigned char rn41_GetBasicParam(FILE *filebt,char *mac,char *name,char *baud,char *mode,char *auth,char *pin)
{
	char buffer[256];
	char *p;
	
	fputs("D\n",filebt);

	fgets(buffer,256,filebt);				// Settings
	fgets(buffer,256,filebt);				// MAC
	buffer[strlen(buffer)-2] = 0;
	p = strchr(buffer,'=');
	strcpy(mac,p+1);
	fgets(buffer,256,filebt);				// Name
	buffer[strlen(buffer)-2] = 0;
	p = strchr(buffer,'=');
	strcpy(name,p+1);
	fgets(buffer,256,filebt);				// Baudrate
	buffer[strlen(buffer)-2] = 0;
	p = strchr(buffer,'=');
	strcpy(baud,p+1);
	fgets(buffer,256,filebt);				// Mode
	buffer[strlen(buffer)-2] = 0;
	p = strchr(buffer,'=');
	if(strcmp(p+1,"Slav")==0)
		*mode = 0;
	else 
		*mode = 255;
	fgets(buffer,256,filebt);				// Authentication
	buffer[strlen(buffer)-2] = 0;
	p = strchr(buffer,'=');
	*auth = atoi(p);
	fgets(buffer,256,filebt);				// Pin
	buffer[strlen(buffer)-2] = 0;
	p = strchr(buffer,'=');
	strcpy(pin,p+1);
	
	fgets(buffer,256,filebt);				// Bonded
	fgets(buffer,256,filebt);				// Rem
	
	return 0;
}
unsigned char rn41_GetExtended(FILE *fileinfo,FILE *filebt)
{
	rn41_CmdRespn(fileinfo,filebt,"E\n",11);
	return 0;
}
unsigned char rn41_GetExtendedParam(FILE *filebt,unsigned short *srvclass,unsigned short *devclass,unsigned char *cfgtimer)
{
	char buffer[256];
	char *p;
	
	fputs("E\n",filebt);

	fgets(buffer,256,filebt);				// Settings
	fgets(buffer,256,filebt);				// SrvName
	fgets(buffer,256,filebt);				// SrvClass
	buffer[strlen(buffer)-2] = 0;
	p = strchr(buffer,'=');
	*srvclass = strtoul(p+1,0,16);
	fgets(buffer,256,filebt);				// DevClass
	buffer[strlen(buffer)-2] = 0;
	p = strchr(buffer,'=');
	*devclass = strtoul(p+1,0,16);
	fgets(buffer,256,filebt);				// InqWindw
	fgets(buffer,256,filebt);				// PagWindw
	fgets(buffer,256,filebt);				// CfgTimer
	buffer[strlen(buffer)-2] = 0;
	p = strchr(buffer,'=');
	*cfgtimer = atoi(p+1);
	fgets(buffer,256,filebt);				// StatusStr
	fgets(buffer,256,filebt);				// HidFlags
	fgets(buffer,256,filebt);				// DTRTimer
	fgets(buffer,256,filebt);				// KeySwapr
	
	return 0;	
}



void rn41_PrintBasicParam(FILE *file,char *mac,char *name,char *baud,char mode,char auth,char *pin)
{
	fprintf_P(file,PSTR("Basic parameters:\n"));
	fprintf_P(file,PSTR("\tMAC: %s\n"),mac);
	fprintf_P(file,PSTR("\tname: %s\n"),name);
	fprintf_P(file,PSTR("\tbaud: %s\n"),baud);
	fprintf_P(file,PSTR("\tmode: %d\n"),mode);
	fprintf_P(file,PSTR("\tauth: %d\n"),auth);
	fprintf_P(file,PSTR("\tpin: %s\n"),pin);
}
void rn41_PrintExtParam(FILE *file,unsigned short srvclass,unsigned short devclass, unsigned char cfgtimer)
{
	fprintf_P(file,PSTR("Extended parameters:\n"));
	fprintf_P(file,PSTR("\tsrvclass: %04X\n"),srvclass);
	fprintf_P(file,PSTR("\tdevclass: %04X\n"),devclass);
	fprintf_P(file,PSTR("\tcfgtimer: %d\n"),cfgtimer);
}

/*
	file: device where to send information about the setup (typically USB)
	filebt: device where the bt chip is hooked up to
*/
void rn41_Setup(FILE *file,FILE *filebt,unsigned char *devname)
{
	/*
		Default:
			CmdRespn 02: 'SrvClass=0000'
			CmdRespn 03: 'DevClass=1F00'
	*/
	// Current parameters
	char mac[16],name[24],baud[8],mode,auth,pin[8];
	unsigned short srvclass,devclass;
	unsigned char cfgtimer;
	
	
	serial_setblocking(filebt,1);			// Must be in blocking mode for initialisation
	
	// Desired parameters
	unsigned short p_srvclass=0b00001001000;			// Service: capturing, positioning
	unsigned short p_devclass=0b0011100011000;		// Device: wearable, wearable computer
	//unsigned char p_cfgtimer = 255;								// 255=unlimited local/remote; 0=local only when not connected. 255 leads to instability (reboots) under high throughput, also lowers bitrate
	unsigned char p_cfgtimer = 0;									// 255=unlimited local/remote; 0=local only when not connected. 255 leads to instability (reboots) under high throughput, also lowers bitrate
	char p_mode = 0;															// Slave
	char p_auth = 0;															// No authentication
	char *p_pin="0000";														// PIN
	char *p_name="BlueSense";											// Name
	
	fprintf_P(file,PSTR("Reset and enter command mode\n"));
	rn41_Reset(file);		
	fprintf_P(file,PSTR("cmd\n"));
	rn41_CmdEnter(file,filebt);
	//fprintf_P(file,PSTR("Status\n"));
	//rn41_GetBasic();
	//rn41_GetExtended();
	
	
	// Get basic params
	
	fprintf_P(file,PSTR("Getting basic params\n"));
	rn41_GetBasicParam(filebt,mac,name,baud,&mode,&auth,pin);
	rn41_PrintBasicParam(file,mac,name,baud,mode,auth,pin);
	
	fprintf_P(file,PSTR("Getting extended params\n"));	
	rn41_GetExtendedParam(filebt,&srvclass,&devclass,&cfgtimer);
	rn41_PrintExtParam(file,srvclass,devclass,cfgtimer);
	
	
	// Check if discrepancy between desired and target params
	char config = 0;
	if(p_srvclass != srvclass) config = 1;
	if(p_devclass != devclass) config = 1;
	if(p_cfgtimer != cfgtimer) config = 1;
	if(p_mode != mode) config = 1;
	if(p_auth != auth) config = 1;
	if(strcmp(p_pin,pin)!=0) config = 1;
	if(strncmp(p_name,name,strlen(p_name))!=0 || strlen(name)!=strlen(p_name)+5) config = 1;
	
	if(config==1)
	{
		fprintf_P(file,PSTR("BT must be configured\n"));
		fprintf_P(file,PSTR("Set authentication\n"));
		rn41_SetAuthentication(file,filebt,p_auth);
		fprintf_P(file,PSTR("Set configuration timer\n"));
		rn41_SetConfigTimer(file,filebt,p_cfgtimer);
		fprintf_P(file,PSTR("Set name\n"));
		rn41_SetSerializedName(file,filebt,p_name);		
		fprintf_P(file,PSTR("Set PIN\n"));
		rn41_SetPIN(file,filebt,p_pin);
		fprintf_P(file,PSTR("Set mode\n"));
		rn41_SetMode(file,filebt,p_mode);						
		fprintf_P(file,PSTR("Set service class\n"));
		rn41_SetServiceClass(file,filebt,p_srvclass);				
		fprintf_P(file,PSTR("Set device class\n"));
		rn41_SetDeviceClass(file,filebt,p_devclass);				
		
		fprintf_P(file,PSTR("Reset, re-enter command mode and verify\n"));
		rn41_Reset(file);		
		rn41_CmdEnter(file,filebt);
		rn41_GetBasicParam(filebt,mac,name,baud,&mode,&auth,pin);
		rn41_PrintBasicParam(file,mac,name,baud,mode,auth,pin);	
		rn41_GetExtendedParam(filebt,&srvclass,&devclass,&cfgtimer);
		rn41_PrintExtParam(file,srvclass,devclass,cfgtimer);
	}
	else
		fprintf_P(file,PSTR("BT configuration OK\n"));
		
	// Copy the device name
	if(devname)
	{
		for(int i=0;i<4;i++) devname[i] = name[strlen(p_name)+1+i];
		devname[4]=0;
	}
	
	//printf("Set pass data in cmd mode\n");
	//rn41_SetPassData(0);	
	
	rn41_CmdLeave(file,filebt);
	
	if(1)
	{
		rn41_CmdEnter(file,filebt);
		
		//rn41_SetTempBaudrate("460K");
		//uart1_init(2,1);	// 460800bps  @ 11.06 Mhz
		
		rn41_SetTempBaudrate(file,filebt,"230K");
		uart1_init(2,0);	// 230400bps  @ 11.06 Mhz
		
		// Change the baudrate 
		//rn41_SetTempBaudrate("115K");
		
		//rn41_SetTempBaudrate("57.6");		
		
		
		//uart1_init(5,0);	// 115200bps  @ 11.06 Mhz
		//uart1_init(11,0);	// 57600bps @ 11.06 Mhz		
		
		/*_delay_ms(1000);
		rn41_CmdEnter();
		printf("Status\n");
		rn41_GetBasic();
		//rn41_GetExtended();
		_delay_ms(2000);
		//rn41_CmdResp("F,1\n");		
		rn41_CmdLeave();
		_delay_ms(1000);*/
		
	}		
	
	
	
	serial_setblocking(filebt,0);			// Return in non blocking mdoe
	
	fprintf_P(file,PSTR("bt init: returning\n"));
}

