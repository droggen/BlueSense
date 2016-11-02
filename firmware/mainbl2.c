#include "cpu.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <stdio.h>
#include <string.h>


#include "system.h"




/******************************************************************************
Main program loop
******************************************************************************/
int main(void)
{
	unsigned char r,v;
	
	// INIT MODULE
	//init_module();
	init_ports();
	//init_timers();
	//i2c_init();
	//dbg_init();
	//file_usb = serial_open(10,1);
	//file_bt=serial_open(1,1);
	//serial_setblocking(file_usb,0);
	
	
	int n=3;
	//fprintf(0,"a");			// 1560
	//fprintf(0,"a %d",4);			// 2886, 2528, 4428
	//fputs("a",0);					// 1560
	
	char *b;
	b = malloc(128);
	
	//scanf("%d",&n);
	
	while(1)
	{
		system_blink(10,50,0b01);
		system_led_set(0b00);
		_delay_ms(500);
	}
	
	
	
	
	


}





