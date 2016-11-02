#include "cpu.h"
#include <avr/io.h>

/*
	File: boot
	
	Functions to call the bootloader from the firmware. 
	
	The assumption is that the firmware is able to detect a programming command start and jumps to the bootloader code which is capable of modifying the firmware.
	The registers UBRR0 and UBRR1 are used to indicate
	
	In order to indic
	
*/



void boot_dbg(void)
{
	void (*bl)(void) = 0x1E000;
	UBRR0=250;
	bl();
}
void boot_bluetooth(void)
{
	void (*bl)(void) = 0x1E000;
	UBRR1=250;
	bl();
}