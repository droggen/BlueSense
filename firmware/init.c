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

#if BOOTLOADER==0
//#include "adc.h"
#include "lcd.h"
#include "fb.h"
#include "serial.h"
#include "main.h"
#include "spi.h"
//#include "i2c.h"
//#include "ds3232.h"
//#include "rn41.h"
//#include "mpu.h"
#include "spi-usart0.h"
#endif

unsigned char init_ddra;
unsigned char init_porta;
unsigned char init_ddrb;
unsigned char init_portb;

#if HWVER==1
void init_ports(void)
{
	
	//DDxn, PORTxn, and PINx
	// DDxn =1 : output
	
	DDRA  = 0b00111000;
	PORTA = 0b01100111;
	
	
	DDRB  = 0b10110011;
	PORTB = 0b00011111;
	
	DDRC  = 0b10001000;
	PORTC = 0b00010000;	        
	
		
	DDRD  = 0b01101010;
	PORTD = 0b00000001;	// What it should be
	//PORTD = 0b01000001;		// Arm factory reset
	
	// Interrupt on change on PA6 (RTC int)
	// PA6 is PCINT6
	PCMSK0 = 0b01000000;			// Mask to select interrupt PA6
		
	// Interrupt on change on PC5 (Motion int)
	// PC5 is PCINT21
	PCMSK2 = 0b00100000;			// Mask to select interrupt PC5
	
	// Enable PCIE0 and PCIE2
	PCICR = 0b00000101;				// Enable interrupt on port A and C
}
#endif

#if (HWVER==4) || (HWVER==5) || (HWVER==6)
void init_ports(void)
{
	
	//DDxn, PORTxn, and PINx
	//DDxn =1 : output
	
	init_ddra = 0b00110000;
	init_porta = 0b01111111;
	DDRA  = init_ddra;
	PORTA = init_porta;
	
	
	init_ddrb = 0b10110011;
	init_portb = 0b10111111;
	DDRB  = init_ddrb;
	PORTB = init_portb;
	
	DDRC  = 0b11001000;		// Default
	#if (HWVER==4) 
	PORTC = 0b11010000;		// Default for V4
	#endif
	#if (HWVER==5)
	PORTC = 0b01010000;		// Default for V5
	#endif
	#if (HWVER==6)
	PORTC = 0b11011000;		// Default for V6
	#endif
	
	
	/////////////////////////////////
	//DDRC  = 0b11001011;		// Test i2c, set as output
	//PORTC = 0b11010011;		// test i2c
	//PORTA &= 0b11011111;	// BT hard reset
	/////////////////////////////////
	
	#if (HWVER==4) 	
	DDRD  = 0b01101010;	// V4
	PORTD = 0b00000011;	// V4
	#endif
	#if (HWVER==5)
	DDRD  = 0b00101010;	// V5
	PORTD = 0b01000011;	// V5 pullup on pwren
	#endif
	#if (HWVER==6)
	DDRD  = 0b00101010;	// V6
	PORTD = 0b00000011;	// V6 
	#endif
	
	
	
}
void init_portchangeint(void)
{

	// Interrupt on change on PA6 (RTC int)
	// PA6 is PCINT6
	PCMSK0 = 0b01000000;			// Mask to select interrupt PA6
		
	// Interrupt on change on PC5 (Motion int)
	// PC5 is PCINT21
	PCMSK2 = 0b00100000;			// Mask to select interrupt PC5
	
	#if HWVER==4 	
	// Interrupt on change on PD7 (Bluetooth connect) and PD4 (Bluetooth RTS)
	// PD7 is PCINT31
	PCMSK3 = 0b10010000;
	#endif
	#if (HWVER==5) || (HWVER==6)
	// Interrupt on change on PD7 (Bluetooth connect), PD4 (Bluetooth RTS), and PD6 (USB connect)
	// PD7 is PCINT31
	PCMSK3 = 0b11010000;
	#endif
	
	
	// Enable PCIE0 and PCIE2
	PCICR = 0b00001101;				// Enable interrupt on port A and C
}

#endif

void init_timers(void)
{
	// Timer 0: CPU
	TCCR1A = 0x00;									// Clear timer on compare
	TCCR1B = 0x08|0x01;								// Clear timer on compare, prescaler 1
	TIMSK1 = (1<<OCIE1A);							// Output compare match A interrupt enable
	#if HWVER==1
	OCR1A = 7199;									// Top value: divides by OCR1A+1; 7199 leads to divide by 7200
	#endif
	#if (HWVER==4) || (HWVER==5) || (HWVER==6)
	OCR1A = 10799;									// Top value: divides by OCR1A+1; 10799 leads to divide by 10800
	#endif
	
	/*
	// Timer 2: asynchronous from RTC
	ASSR = (1<<EXCLK)|(1<<AS2);				// External clock, asynchronous
	TCCR2A = 0x02;										// Clear timer on compare
	TCCR2B = 0x01;										// Prescaler 1
	TIMSK2 = (1<<OCIE2A);							// Output compare match A interrupt enable
	//OCR2A = 31;												// Top value: divides by OCR2A+1; 31 leads to divide by 32
	//OCR2A = 32;												// Top value
	OCR2A = 31;												// Top value
	//OCR2A = 10;												// Top value
	*/
	
	

}


void init_module(void)
{
	init_ports();
	init_portchangeint();
	init_timers();
	
#if BOOTLOADER==0
	spi_init(SPI_DIV_2);
#endif
	
#if BOOTLOADER==0
	
	sei();				// enable all interrupts
	
	
	
	// 
	// Setup the serial port
	/*UCSR0B = 0x00; 				  //disable while setting baud rate
	UCSR0C = 0x06; 				  // Asyn,NoParity,1StopBit,8Bit,
	UCSR0A = 0x00;					// u2x=0
	UBRR0H = 0;
	UBRR0L = 3;
	//UCSR0B = (1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0);			// RX/TX Enable, RX/TX Interrupt Enable
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);			// RX/TX Enable, RX/TX Interrupt Enable*/
	
	// Communication setup
	//uart_init(2);
	// UART0=USB in v1, motion in v4
	// UART1=BT
	
	// (3,0) 115200bps  @ 7.37 Mhz
	// (5,0) 115200bps  @ 11.06 Mhz
	// (1,0) 230400bps  @ 7.37 Mhz
	//uart0_init(3,0);	
	//uart0_init(1,0);	
	
	#if HWVER==1
		uart1_init(3,0);	
	#endif
	#if (HWVER==4) || (HWVER==5) || (HWVER==6)
		uart1_init(5,0);	// 115200bps  @ 11.06 Mhz
		//uart1_init(2,0);	// 230400bps  @ 11.06 Mhz
	#endif
	
	#if HWVER==1
		uart0_init(1,0);	
	#endif
	
	#if (HWVER==4) || (HWVER==5) || (HWVER==6)
		#if BOOTLOADER==0
			spiusart0_init();
		#endif
	#endif
	
#endif
}

#if BOOTLOADER==0
void init_lcd(void)
{
	lcd_spi_init();
	file_fb = fb_initfb();
	// Reset display (PB2)
	PORTB &= 0b11111011;
	_delay_ms(100);
	PORTB |= 0b00000100;
	_delay_ms(100);	
	lcd_led(1);	
	lcd_init();
	lcd_clear565(0);
	lcd_writestring("BlueSense2",4,55,3,0x0000,0xffff);
}
void deinit_lcd(void)
{
	lcd_spi_deinit();
	file_fb = 0;
}



#endif