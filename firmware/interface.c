/**
\brief Handles bring communication interfaces up and down

The node has communication interfaces: USB and Bluetooth.
The firmware considers one of the interface to be a primary interface, while the other one is a debug interface.

When a connection or disconnection is detect, this module takes care of updating which is the primary and debug interface.

Call interface_signalchange to notify a possible change in connection status.
Use interface_changedetectenable to enable/disable change detection.
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
#include "dbg.h"
#include "serial.h"
#include "system.h"


signed char interface_bt_connected_past=0;
signed char interface_usb_connected_past=0;
unsigned char _interface_changedetectenabled=0;

/*
	Interface primary/debug assignment logic
	
	First connection becomes primary, second connection becomes debug.
	If two connections occur simultaneously primary is usb.
	If one connection becomes active, it becomes primary if no primary is assisgned yet, otherwise debug.
	If one connection is disconnected (primary or secondary), it gets inactive; the other connection remains identical.
*/
void interface_init(void)
{
	file_pri=file_usb;
	file_dbg=file_bt;
	uart1_clearbuffers();
	dbg_clearbuffers();
}
void interface_btup(void)
{
	uart1_clearbuffers();
}
void interface_btdown(void)
{
	uart1_clearbuffers();
}
void interface_usbup(void)
{
	dbg_clearbuffers();
}
void interface_usbdown(void)
{
	dbg_clearbuffers();
}
void interface_hellopri(void)
{
	// TODO: use putbuf nonblock
	char tmp[32];
	memcpy(tmp,system_getdevicename(),4);
	strcpy(tmp+4,": Hello primary\n");
	fputbuf(file_pri,tmp,strlen(tmp));
}
void interface_hellodbg(void)
{
	// TODO: use putbuf nonblock
	char tmp[32];
	memcpy(tmp,system_getdevicename(),4);
	strcpy(tmp+4,": Hello debug\n");
	fputbuf(file_dbg,tmp,strlen(tmp));
}


/*
	This _interface_update assigns first interface connected to primary, second connected to secondary; 
	if the primary is disconnected while the secondary is connected the primary will remain primary when reconnected (i.e. the only way to "swap" interfaces is to have both disconnected)
*/
/*void _interface_update(signed char cur_bt_connected,signed char cur_usb_connected)
{
	//fprintf(file_usb,"_interface_update: %d %d\n",cur_bt_connected,cur_usb_connected);
	char tmp[128];
	memcpy(tmp,system_getdevicename(),4);
	strcpy(tmp+4,": Interface update\n");
	fputbuf(file_dbg,tmp,strlen(tmp));
	fputbuf(file_pri,tmp,strlen(tmp));
	
	
	// Handle the possibility that one interface is "unchanged" (-1): in which case assign the past state.
	if(cur_bt_connected==-1)
		cur_bt_connected = interface_bt_connected_past;
	if(cur_usb_connected==-1)
		cur_usb_connected = interface_usb_connected_past;
	
	
	// HACK fix pri and dbg
	//file_pri=file_bt;
	//file_dbg=file_usb;
	//stdin=stdout=stderr=file_pri;
	//return;
	

	
	// No change -> do nothing
	if( (cur_bt_connected==interface_bt_connected_past) && (cur_usb_connected==interface_usb_connected_past) )
		return;
		
	// Connection detection
	// USB up
	if( (interface_usb_connected_past==0||interface_usb_connected_past==-1) && cur_usb_connected==1)
		interface_usbup();
	// BT up
	if( (interface_bt_connected_past==0||interface_bt_connected_past==-1) && cur_bt_connected==1)
		interface_btup();
	
		
	// Assignment of primary/secondary if simultaneous connection
	if( ((interface_bt_connected_past==0||interface_bt_connected_past==-1) && cur_bt_connected==1) && ((interface_usb_connected_past==0||interface_usb_connected_past==-1) && cur_usb_connected==1))
	{
		file_pri = file_usb;
		stdin=stdout=stderr=file_pri;
		file_dbg = file_bt;
		interface_hellodbg();
		interface_hellopri();
	}
	else
	{
		// Not simultaneous connection
		// BT up
		if( (interface_bt_connected_past==0||interface_bt_connected_past==-1) && cur_bt_connected==1)
		{
			// bt is primary: pri doesn't change
			if(file_pri==file_bt)
			{
				interface_hellopri();
				//sprintf(tmp,"%s: Primary up\n",system_getdevicename());
				//fputbuf(file_dbg,tmp,strlen(tmp));
			}
			else
			{
				// bt is debug
				// bt stays dbg if usb is connected
				if(cur_usb_connected==1)
				{
					file_pri = file_usb;
					file_dbg = file_bt;
					stdin=stdout=stderr=file_pri;
					interface_hellodbg();
					//sprintf(tmp,"%s: Debug up. cur_usb: %d usb_past: %d cur_bt: %d bt_past: %d\n",system_getdevicename(),cur_usb_connected,interface_usb_connected_past,cur_bt_connected,interface_bt_connected_past);
					//fputbuf(file_dbg,tmp,strlen(tmp));

				}
				else
				{
					// usb isn't connected -> bt becomes primary
					file_pri = file_bt;				
					file_dbg = file_usb;	
					stdin=stdout=stderr=file_pri;
					interface_hellopri();
					//sprintf(tmp,"%s: Primary up\n",system_getdevicename());
					//fputbuf(file_dbg,tmp,strlen(tmp));

				}
			}				
		}
		// USB up
		if( (interface_usb_connected_past==0||interface_usb_connected_past==-1) && cur_usb_connected==1)
		{
			// usb is primary: pri doesn't change
			if(file_pri==file_usb)
			{
				interface_hellopri();
				sprintf(tmp,"%s: Primary up\n",system_getdevicename());
				fputbuf(file_dbg,tmp,strlen(tmp));
			}
			else
			{
				// usb is debug
				// usb stays dbg if bt is connected
				if(cur_bt_connected==1)
				{
					file_pri=file_bt;
					file_dbg=file_usb;
					stdin=stdout=stderr=file_pri;
					interface_hellodbg();
					//sprintf(tmp,"%s: Debug up. cur_usb: %d usb_past: %d cur_bt: %d bt_past: %d\n",system_getdevicename(),cur_usb_connected,interface_usb_connected_past,cur_bt_connected,interface_bt_connected_past);
					//fputbuf(file_dbg,tmp,strlen(tmp));

				}
				else
				{
					// bt isn't connected -> usb becomes primary
					file_pri = file_usb;
					file_dbg = file_bt;
					interface_hellopri();
					//sprintf(tmp,"%s: Primary up\n",system_getdevicename());
					//fputbuf(file_dbg,tmp,strlen(tmp));

				}
			}
		}
		// BT down
		if(cur_bt_connected==0 && interface_bt_connected_past==1)
		{
			if(file_pri == file_bt)
			{
				//file_pri=0;
				//stdin=stdout=stderr=file_pri;
				//sprintf(tmp,"%s: Primary dn\n",system_getdevicename());
				//fputbuf(file_dbg,tmp,strlen(tmp));

			}
			//if(file_dbg == file_bt)
				//file_dbg=0;
		}
		// USB down
		if(cur_usb_connected==0 && interface_usb_connected_past==1)
		{
			if(file_pri == file_usb)
			{
				//file_pri=0;
				//stdin=stdout=stderr=file_pri;
				//sprintf(tmp,"%s: Primary dn\n",system_getdevicename());
				//fputbuf(file_dbg,tmp,strlen(tmp));
			}
			//if(file_dbg == file_usb)
//				file_dbg=0;
		}
	}
	
	
	// USB down 
	if(interface_usb_connected_past==1 && cur_usb_connected==0)
		interface_usbdown();
	// BT down
	if(interface_bt_connected_past==1 && cur_bt_connected==0)
		interface_btdown(); 
		
	interface_usb_connected_past = cur_usb_connected;
	interface_bt_connected_past = cur_bt_connected;
}*/
/*
	The logic of the interfaces is as follows:
	- first connected is primary
	- if a second one is connected then bluetooth is the primary and usb the secondary
	- if the second interface is disconnected, the one connected becomes primary
*/
void _interface_update(signed char cur_bt_connected,signed char cur_usb_connected)
{
	// Handle the possibility that one interface is "unchanged" (-1): in which case assign the past state.
	if(cur_bt_connected==-1)
		cur_bt_connected = interface_bt_connected_past;
	if(cur_usb_connected==-1)
		cur_usb_connected = interface_usb_connected_past;
		
	// No actual change -> do nothing
	if( (cur_bt_connected==interface_bt_connected_past) && (cur_usb_connected==interface_usb_connected_past) )
		return;
		
	// Change: print some info
	char tmp[128];
	memcpy(tmp,system_getdevicename(),4);
	strcpy(tmp+4,": Interface update 0 0\n");
	*(tmp+23)=*(tmp+23)+cur_bt_connected;
	*(tmp+25)=*(tmp+25)+cur_usb_connected;
	fputbuf(file_dbg,tmp,strlen(tmp));
	fputbuf(file_pri,tmp,strlen(tmp));

		
	// Connection detection
	// USB up
	if( (interface_usb_connected_past==0||interface_usb_connected_past==-1) && cur_usb_connected==1)
		interface_usbup();
	// BT up
	if( (interface_bt_connected_past==0||interface_bt_connected_past==-1) && cur_bt_connected==1)
		interface_btup();
	
		
	// Assignment of primary/secondary if simultaneous connection: bluetooth is primary
	if( ((interface_bt_connected_past==0||interface_bt_connected_past==-1) && cur_bt_connected==1) && ((interface_usb_connected_past==0||interface_usb_connected_past==-1) && cur_usb_connected==1))
	{
		file_pri = file_bt;
		stdin=stdout=stderr=file_pri;
		file_dbg = file_usb;
		interface_hellodbg();
		interface_hellopri();
	}
	else
	{
		// Not simultaneous connection
		// BT up
		if( (interface_bt_connected_past==0||interface_bt_connected_past==-1) && cur_bt_connected==1)
		{
			// bt is primary: pri doesn't change
			file_pri=file_bt;
			file_dbg=file_usb;
			interface_hellopri();
			interface_hellodbg();
		}
		// BT down
		if(cur_bt_connected==0 && interface_bt_connected_past==1)
		{
			file_pri=file_usb;
			file_dbg=file_bt;
			interface_hellopri();
			interface_hellodbg();
		}
	}
	
	
	// USB down 
	if(interface_usb_connected_past==1 && cur_usb_connected==0)
		interface_usbdown();
	// BT down
	if(interface_bt_connected_past==1 && cur_bt_connected==0)
		interface_btdown(); 
		
	interface_usb_connected_past = cur_usb_connected;
	interface_bt_connected_past = cur_bt_connected;
}
void interface_swap(void)
{
	FILE *f;
	f=file_pri;
	file_pri=file_dbg;
	file_dbg=f;
	stdin=stdout=stderr=file_pri;	
	interface_hellopri();
	interface_hellodbg();
}

void interface_test(void)
{
	fprintf_P(file_usb,PSTR("Interface USB. BT: %d USB: %d.\n"),interface_bt_connected_past,interface_usb_connected_past);
	fprintf_P(file_bt,PSTR("Interface BT. BT: %d USB: %d.\n"),interface_bt_connected_past,interface_usb_connected_past);
	fprintf_P(file_usb,PSTR("Interface USB. file_usb: %p file_bt: %p file_pri: %p file_dbg: %p\n"),file_usb,file_bt,file_pri,file_dbg);
	fprintf_P(file_bt,PSTR("Interface BT. file_usb: %p file_bt: %p file_pri: %p file_dbg: %p\n"),file_usb,file_bt,file_pri,file_dbg);
	if(file_pri)
		fprintf_P(file_pri,PSTR("Interface primary\n"));
	if(file_dbg)
		fprintf(file_dbg,PSTR("Interface debug\n"));

		
}

// Indicate whether to act on interface change
void interface_changedetectenable(unsigned char enable)
{
	_interface_changedetectenabled=enable;
}

// Must be called when an interface change is detected (e.g. by an interrupt vector) to react accordingly
// Parameter: 1=connected; 0=disconnected; -1=no change/unknown
void interface_signalchange(signed char cur_bt_connected,signed char cur_usb_connected)
{
	//fprintf(file_usb,"interface_signalchange: %d %d\n",cur_bt_connected,cur_usb_connected);
	
	if(!_interface_changedetectenabled)
		return;
	
	_interface_update(cur_bt_connected,cur_usb_connected);			
}