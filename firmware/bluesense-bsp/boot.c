/*
	File: boot
	
	Functions to call the bootloader from the firmware. 
	
	The assumption is that the firmware is able to detect a programming command start and jumps to the bootloader code which is capable of modifying the firmware.
	The register UBRR0 is used to indicate whether the bootloader should listen to the dbg/usb interface (UBRR0=250) or the Bluetooth interface (UBRR0=251).
	
	The bootloader does not put in place an interrupt routine for all the peripherals used in the application code.
	It is therefore mandatory to disable the interrupts on all the peripherals prior to entering the bootloder, otherwise if an interrupt is fired which is not handled
	by the bootloader the bootloader would reset or enter a boot loop.
	
	The application code uses the following interrupt vectors (marked with * the ones used by the bootloader as well):
		4	PCINT0
		6	PCINT2
		7	PCINT3
		*13	TIMER1_COMPA
		22	USART0_TX
		24	ADC
		*26	TWI
		*28	USART1_RX
		*29	USART1_UDRE

	
*/

#include "cpu.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "init.h"
#include "serial1.h"
#include "spi-usart0.h"
#include "adc.h"
#include "i2c.h"
#include "system.h"
#include "boot.h"



/******************************************************************************
	function: boot_dbg
*******************************************************************************	
	Calls the bootloader and passes in UBRR0 the indication that the bootloader
	command has been received on the USB interface.
******************************************************************************/
void boot_dbg(void)
{
	// Deactivate port change interrupts
	deinit_portchangeint();
	// Deactivate timers
	deinit_timers();
	// Deactivate the SPI interrupt on USART0
	spiusart0_deinit();
	// Deactivate the ADC
	ADCDeinit();
	// Deactivate the TWI
	i2c_deinit();
	// Deactivate uart1
	uart1_deinit();

	cli();
	_delay_ms(100);
	
	void (*bl)(void) = (void(*)(void))0xF000;		// Address of bootloader in words
	UBRR0=250;
	bl();
}
/******************************************************************************
	function: boot_bluetooth
*******************************************************************************	
	Calls the bootloader and passes in UBRR0 the indication that the bootloader
	command has been received on the Bluetooth interface.
******************************************************************************/
void boot_bluetooth(void)
{
	// Deactivate port change interrupts
	deinit_portchangeint();
	// Deactivate timers
	deinit_timers();
	// Deactivate the SPI interrupt on USART0
	spiusart0_deinit();
	// Deactivate the ADC
	ADCDeinit();
	// Deactivate the TWI
	i2c_deinit();
	// Deactivate uart1
	uart1_deinit();
	
	cli();
	_delay_ms(100);
	
	void (*bl)(void) = (void(*)(void))0xF000;		// Address of bootloader in words
	UBRR0=251;
	bl();
}

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
	//if(c==0x1B)
	if(c=='a')		// to help debugging
	{
		system_blink_enterbootloader();
		boot_dbg();
	}
	return 1;
}
unsigned char bootloaderhook_bluetooth(unsigned char c)
{
	// Enters the bootloader on reception of 0x1B
	if(c==0x1B)
	{
		system_blink_enterbootloader();
		boot_bluetooth();
	}
	return 1;
}


