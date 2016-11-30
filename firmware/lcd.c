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

#include "main.h"
#include "wait.h"
#include "lcd.h"
#include "fb.h"
#include "fnt4x6.h"
#include "system.h"
#include "init.h"

/*
Display 128x128 pixels
driver: ILI9163
a-Si TFT LCD Single Chip Driver
132RGBx162 Resolution and 262K color
Datasheet
Version: V0.18
Document No.: ILI9163DS_V0.18.pdf
ILI TECHNOLOGY

LCD interface version 1: 
X_ADC0 = SS# = PA0
X_ADC1 = DC# = PA1
X_ADC2 = MOSI = PA2
X_AIN0 = SCK = PB2
X_AIN1 = LED = PB3
X_0 = RST# = N/A

LCD interface version 2: 
X_AIN0 = SS# = PB2
X_ADC2 = DC# = PA2
X_ADC0 = MOSI = PA0
X_ADC1 = SCK = PA1
X_AIN1 = LED = PB3
X_0 = RST# = N/A
*/

/*
With function calls.

Time fillscreen: 820 819 818 819 819 
Time sendframe: 515 513 515 514 514 

With variable to store 0/1 scl:
Time fillscreen: 756 756 757 757 756 
Time sendframe: 476 476 477 476 477 

With variable to store 0/1 scl and sda:
Time fillscreen: 727 729 728 728 728 
Time sendframe: 465 466 467 466 467 

With variable to store 0/1 scl and sda and unrolled:
Time fillscreen: 416 416 416 417 417 
Time sendframe: 267 267 268 268 267 

writen with variables and unrolled
Time fillscreen: 416 417 416 416 416 
Time sendframe: 197 198 197 198 197 



With variables and unrolled, v1:
Time fillscreen: 413 413 414 413 413 
Time sendframe: 195 195 197 195 195 

With function calls v1:
Time fillscreen: 814 813 813 812 814 
Time sendframe: 510 510 510 510 509 

With variables and unrolled, v2:
Time fillscreen: 391 390 391 390 391 
Time sendframe: 175 175 174 174 174 

ASM, v2 (GCC 4.3)
Time fillscreen: 314 315 314 314 315 
Time sendframe: 152 154 152 152 153 

ASM, v2 (GCC 4.9 or GCC 5)
Time fillscreen: 266 267 266 267 267 
Time sendframe: 154 154 155 154 154 

*/


#if defined(__GAMMASET1)
	const unsigned char pGammaSet[15]= {0x36,0x29,0x12,0x22,0x1C,0x15,0x42,0xB7,0x2F,0x13,0x12,0x0A,0x11,0x0B,0x06};
	const unsigned char nGammaSet[15]= {0x09,0x16,0x2D,0x0D,0x13,0x15,0x40,0x48,0x53,0x0C,0x1D,0x25,0x2E,0x34,0x39};
#elif defined(__GAMMASET2)
	const unsigned char pGammaSet[15]= {0x3F,0x21,0x12,0x22,0x1C,0x15,0x42,0xB7,0x2F,0x13,0x02,0x0A,0x01,0x00,0x00};
	const unsigned char nGammaSet[15]= {0x09,0x18,0x2D,0x0D,0x13,0x15,0x40,0x48,0x53,0x0C,0x1D,0x25,0x2E,0x24,0x29};
#elif defined(__GAMMASET3)
	const unsigned char pGammaSet[15]= {0x3F,0x26,0x23,0x30,0x28,0x10,0x55,0xB7,0x40,0x19,0x10,0x1E,0x02,0x01,0x00};
	const unsigned char nGammaSet[15]= {0x09,0x18,0x2D,0x0D,0x13,0x15,0x40,0x48,0x53,0x0C,0x1D,0x25,0x2E,0x24,0x29};
#else
	const unsigned char pGammaSet[15]= {0x3F,0x25,0x1C,0x1E,0x20,0x12,0x2A,0x90,0x24,0x11,0x00,0x00,0x00,0x00,0x00};
	const unsigned char nGammaSet[15]= {0x20,0x20,0x20,0x20,0x05,0x15,0x00,0xA7,0x3D,0x18,0x25,0x2A,0x2B,0x2B,0x3A};
#endif

#ifdef LCD_V1
// SDA on PORTA
#define SPI_SDAMASK 0b00000100
// SCL on PORTB
#define SPI_SCLMASK 0b00000100
#endif
#ifdef LCD_V2
// SDA on PORTA
#define SPI_SDAMASK 0b00000001
// SCL on PORTA
#define SPI_SCLMASK 0b00000010
#endif

/*

	RGB 444: 12 bit/pixel, 3 byte writes for 2 pixels
	
	RGB444: RRR GGG BBB rrr ggg bbb

*/

// Nibble (4 pixels) to RGB444 pixel data. 4 pixels -> 6 bytes
const unsigned char lcd_nibble_to_rgb444[16][6] = {
																					/* 0000 */ {0b00000000,0b00000000,0b00000000,   0b00000000,0b00000000,0b00000000},
																					/* 0001 */ {0b00000000,0b00000000,0b00000000,   0b00000000,0b00001111,0b11111111},
																					/* 0010 */ {0b00000000,0b00000000,0b00000000,   0b11111111,0b11110000,0b00000000},
																					/* 0011 */ {0b00000000,0b00000000,0b00000000,   0b11111111,0b11111111,0b11111111},
																					/* 0100 */ {0b00000000,0b00001111,0b11111111,   0b00000000,0b00000000,0b00000000},
																					/* 0101 */ {0b00000000,0b00001111,0b11111111,   0b00000000,0b00001111,0b11111111},
																					/* 0110 */ {0b00000000,0b00001111,0b11111111,   0b11111111,0b11110000,0b00000000},
																					/* 0111 */ {0b00000000,0b00001111,0b11111111,   0b11111111,0b11111111,0b11111111},
																					/* 1000 */ {0b11111111,0b11110000,0b00000000,   0b00000000,0b00000000,0b00000000},
																					/* 1001 */ {0b11111111,0b11110000,0b00000000,   0b00000000,0b00001111,0b11111111},
																					/* 1010 */ {0b11111111,0b11110000,0b00000000,   0b11111111,0b11110000,0b00000000},
																					/* 1011 */ {0b11111111,0b11110000,0b00000000,   0b11111111,0b11111111,0b11111111},
																					/* 1100 */ {0b11111111,0b11111111,0b11111111,   0b00000000,0b00000000,0b00000000},
																					/* 1101 */ {0b11111111,0b11111111,0b11111111,   0b00000000,0b00001111,0b11111111},
																					/* 1110 */ {0b11111111,0b11111111,0b11111111,   0b11111111,0b11110000,0b00000000},
																					/* 1111 */ {0b11111111,0b11111111,0b11111111,   0b11111111,0b11111111,0b11111111}
																				};

// Nibble (4 pixels) to RGB565 pixel data. 4 pixels -> 8 bytes
const unsigned char lcd_nibble_to_rgb565[16][8] = {
																					/* 0000 */ {0b00000000,0b00000000,   0b00000000,0b00000000,   0b00000000,0b00000000,   0b00000000,0b00000000},
																					/* 0001 */ {0b00000000,0b00000000,   0b00000000,0b00000000,   0b00000000,0b00000000,   0b11111111,0b11111111},
																					/* 0010 */ {0b00000000,0b00000000,   0b00000000,0b00000000,   0b11111111,0b11111111,   0b00000000,0b00000000},
																					/* 0011 */ {0b00000000,0b00000000,   0b00000000,0b00000000,   0b11111111,0b11111111,   0b11111111,0b11111111},
																					/* 0100 */ {0b00000000,0b00000000,   0b11111111,0b11111111,   0b00000000,0b00000000,   0b00000000,0b00000000},
																					/* 0101 */ {0b00000000,0b00000000,   0b11111111,0b11111111,   0b00000000,0b00000000,   0b11111111,0b11111111},
																					/* 0110 */ {0b00000000,0b00000000,   0b11111111,0b11111111,   0b11111111,0b11111111,   0b00000000,0b00000000},
																					/* 0111 */ {0b00000000,0b00000000,   0b11111111,0b11111111,   0b11111111,0b11111111,   0b11111111,0b11111111},
																					/* 1000 */ {0b11111111,0b11111111,   0b00000000,0b00000000,   0b00000000,0b00000000,   0b00000000,0b00000000},
																					/* 1001 */ {0b11111111,0b11111111,   0b00000000,0b00000000,   0b00000000,0b00000000,   0b11111111,0b11111111},
																					/* 1010 */ {0b11111111,0b11111111,   0b00000000,0b00000000,   0b11111111,0b11111111,   0b00000000,0b00000000},
																					/* 1011 */ {0b11111111,0b11111111,   0b00000000,0b00000000,   0b11111111,0b11111111,   0b11111111,0b11111111},
																					/* 1100 */ {0b11111111,0b11111111,   0b11111111,0b11111111,   0b00000000,0b00000000,   0b00000000,0b00000000},
																					/* 1101 */ {0b11111111,0b11111111,   0b11111111,0b11111111,   0b00000000,0b00000000,   0b11111111,0b11111111},
																					/* 1110 */ {0b11111111,0b11111111,   0b11111111,0b11111111,   0b11111111,0b11111111,   0b00000000,0b00000000},
																					/* 1111 */ {0b11111111,0b11111111,   0b11111111,0b11111111,   0b11111111,0b11111111,   0b11111111,0b11111111}
																				};



/******************************************************************************
	lcd_spi_init
*******************************************************************************	
	Initialise the SPI ports for the LCD.
	
	Compatible with: V1, V2	
******************************************************************************/
void lcd_spi_init(void)
{
	CHECKLCDENABLED;
	/*
		LCD interface display-v1 on pcb-v1: 
		X_ADC0 = SS# = PA0
		X_ADC1 = DC# = PA1
		X_ADC2 = MOSI = PA2
		X_AIN0 = SCK = PB2
		X_AIN1 = LED = PB3
		X_0 = RST# = N/A
		
		LCD interface display-v2 on pcb-v1:
		X_AIN0 = SS# = PB2
		X_ADC2 = DC# = PA2
		X_ADC0 = MOSI = PA0
		X_ADC1 = SCK = PA1
		X_AIN1 = LED = PB3
		X_0 = RST# = N/A
	
		LCD interface display-v3 on pcb-v2
		Display on 	v4 (dv3)	v1 (dv2)
v			X_ADC2 = SS# = PA2
v			X_ADC3 = DC# = PA3
v			X_ADC0 = MOSI = PA0
v			X_ADC1 = SCK = PA1
v			X_AIN1 = LED = PB3
v			X_AIN0 = RST# = PB2

		
		
		DDxn=1: output
	*/

	#if HWVER==1	
	// Setup ports as output driving low.
	DDRA  |= 0b00000111;
	//PORTA &= 0b11111000;
	DDRB |= 0b00001100;
	//PORTB&= 0b11110011;
	#endif
	
	#if (HWVER==4) || (HWVER==5) || (HWVER==6)
	// Same as HWVER1
	// Setup ports as output driving low.
	DDRA  |= 0b00001111;
	//PORTA &= 0b11111000;
	DDRB |= 0b00001100;
	//PORTB&= 0b11110011;
	#endif
	
	
	
	
	// Deselect the interface
	lcd_nselect(1);
	// Clock high by default
	//lcd_clk(1);
	// Clock low by default
	lcd_clk(0);
	
	
}

void lcd_spi_deinit(void)
{
	// LCD uses port a, port b: restore to default state
	DDRA = init_ddra;
	PORTA = init_porta;
	DDRB = init_ddrb;
	PORTB = init_portb;
}


// Toggle the LED pin. 1: display on; 2: display off
// V1+V2
void lcd_led(char c)
{
	CHECKLCDENABLED;
	// V1+V2 identical
	if(c==0)
	{
		PORTB&=0b11110111;
	}
	else
	{
		PORTB|=0b00001000;
	}

			
}

/******************************************************************************
	lcd_cmd
*******************************************************************************	
	Select command mode.
	
	Compatible with: V1, V2	
******************************************************************************/
inline void lcd_cmd(void)
{
	#ifdef LCD_V1
		PORTA&=0b11111101;
	#endif
	#ifdef LCD_V2
		#if HWVER==1
			PORTA&=0b11111011;
		#endif
		#if (HWVER==4) || (HWVER==5) || (HWVER==6)
			// ADC3=PA3
			PORTA&=0b11110111;
		#endif
	#endif
}
/******************************************************************************
	lcd_data
*******************************************************************************	
	Select data mode.
	
	Compatible with: V1, V2	
******************************************************************************/
inline void lcd_data(void)
{
	#ifdef LCD_V1
		PORTA|=0b00000010;
	#endif
	#ifdef LCD_V2
		#if HWVER==1
			PORTA|=0b00000100;
		#endif
		#if (HWVER==4) || (HWVER==5) || (HWVER==6)
			// ADC3=PA3
			PORTA|=0b00001000;
		#endif
	#endif
}
/******************************************************************************
	lcd_bit
*******************************************************************************	
	Set MOSI pin.
	
	Compatible with: V1, V2	
******************************************************************************/
inline void lcd_bit(unsigned char b)
{
	#ifdef LCD_V1
		if(b!=0)
			PORTA|=0b00000100;
		else
			PORTA&=0b11111011;
	#endif
	#ifdef LCD_V2
		if(b!=0)
			PORTA|=0b00000001;
		else
			PORTA&=0b11111110;
	#endif
	
}
/******************************************************************************
	lcd_clk
*******************************************************************************	
	Set SCLK pin.
	
	Compatible with: V1, V2	
******************************************************************************/
inline void lcd_clk(unsigned char c)
{
	#ifdef LCD_V1
		if(c!=0)
			PORTB|=0b00000100;
		else
			PORTB&=0b11111011;
	#endif
	#ifdef LCD_V2
		if(c!=0)
			PORTA|=0b00000010;
		else
			PORTA&=0b11111101;
	#endif
}
/******************************************************************************
	lcd_nselect
*******************************************************************************	
	Set SS# pin.
	
	Compatible with: V1, V2	
******************************************************************************/
void lcd_nselect(unsigned char ns)
{
	#ifdef LCD_V1
		if(ns!=0)
			PORTA|=0b00000001;
		else
			PORTA&=0b11111110;
	#endif
	#ifdef LCD_V2
		#if HWVER==1
			if(ns!=0)
				PORTB|=0b00000100;
			else
				PORTB&=0b11111011;
		#endif
		#if (HWVER==4) || (HWVER==5) || (HWVER==6)
			// SS=ADC2=PA2
			if(ns!=0)
				PORTA|=0b00000100;
			else
				PORTA&=0b11111011;
		#endif
	#endif
}

/******************************************************************************
	lcd_spi_write8_c
*******************************************************************************	
	Write one byte to the LCD SPI interface, using lcd_bit and lcd_clk which take
	care of hardware v1/v2 differences.
	
******************************************************************************/
void lcd_spi_write8_c(unsigned char d)
{
	// MSB transmitted first
	// SDIN sampled at positive edge of clock. Hence: select; setup data; clk l-h (latch); clk h-l (latch); setup data; ....
	
	for(unsigned i=0;i<8;i++)
	{
		lcd_bit(d&0x80);
		lcd_clk(1);
		d<<=1;
		lcd_clk(0);
	}
}

/******************************************************************************
	lcd_spi_write8_v1
*******************************************************************************	
	Write one byte to the LCD SPI interface, using version 1 interface.
	
	This function does not tolerate changes to port A or port B during the execution
	of the function (e.g. in an interrupt), as the state of the ports is cached for 
	faster writes.
	
	#define LCD_V1 if the hardware is v1.
	
******************************************************************************/

#ifdef LCD_V1
void lcd_spi_write8(unsigned char d)
#else
void lcd_spi_write8_v1(unsigned char d)
#endif
{
	unsigned char dsda0=PORTA&(~SPI_SDAMASK);
	unsigned char dsda1=dsda0|SPI_SDAMASK;
	unsigned char dscl0=PORTB&(~SPI_SCLMASK);
	unsigned char dscl1=dscl0|SPI_SCLMASK;
	
	if(d&0x80)
		PORTA=dsda1;
	else
		PORTA=dsda0;
	PORTB=dscl1;
	PORTB=dscl0;
	if(d&0x40)
		PORTA=dsda1;
	else
		PORTA=dsda0;
	PORTB=dscl1;
	PORTB=dscl0;
	if(d&0x20)
		PORTA=dsda1;
	else
		PORTA=dsda0;
	PORTB=dscl1;
	PORTB=dscl0;
	if(d&0x10)
		PORTA=dsda1;
	else
		PORTA=dsda0;
	PORTB=dscl1;
	PORTB=dscl0;
	if(d&0x08)
		PORTA=dsda1;
	else
		PORTA=dsda0;
	PORTB=dscl1;
	PORTB=dscl0;
	if(d&0x04)
		PORTA=dsda1;
	else
		PORTA=dsda0;
	PORTB=dscl1;
	PORTB=dscl0;
	if(d&0x02)
		PORTA=dsda1;
	else
		PORTA=dsda0;
	PORTB=dscl1;
	PORTB=dscl0;
	if(d&0x01)
		PORTA=dsda1;
	else
		PORTA=dsda0;
	PORTB=dscl1;
	PORTB=dscl0;
}

/******************************************************************************
	lcd_spi_write8_v2
*******************************************************************************	
	Write one byte to the LCD SPI interface, using version 2 interface.
	
	This function does not tolerate changes to port A or port B during the execution
	of the function (e.g. in an interrupt), as the state of the ports is cached for 
	faster writes.
	
	#define LCD_V2 if the hardware is v2.
	
	Optionally, #define LCD_V2ASM to use a fast assembler version.	
******************************************************************************/
#ifdef LCD_V2
#ifdef LCD_V2ASM
void lcd_spi_write8_v2(unsigned char d)
#else
void lcd_spi_write8(unsigned char d)
#endif
#else
void lcd_spi_write8_v2(unsigned char d)
#endif
{
	//X_ADC0 = MOSI = PA0
	//X_ADC1 = SCK = PA1
	
	unsigned char dsda0=PORTA&(~(SPI_SDAMASK+SPI_SCLMASK));			// CLK=0, D=0
	unsigned char dsda1=dsda0|SPI_SDAMASK;											// CLK=0, D=1
	unsigned char dsda0b=(PORTA&(~SPI_SDAMASK))|SPI_SCLMASK;		// CLK=1, D=0
	unsigned char dsda1b=dsda0b|SPI_SDAMASK;										// CLK=1, D=1
	
	
	//lcd_bit(d&0x80);
	if(d&0x80)
	{
		PORTA=dsda1;
		PORTA=dsda1b;
	}
	else
	{
		PORTA=dsda0;
		PORTA=dsda0b;
	}
	if(d&0x40)
	{
		PORTA=dsda1;
		PORTA=dsda1b;
	}
	else
	{
		PORTA=dsda0;
		PORTA=dsda0b;
	}
	if(d&0x20)
	{
		PORTA=dsda1;
		PORTA=dsda1b;
	}
	else
	{
		PORTA=dsda0;
		PORTA=dsda0b;
	}
	if(d&0x10)
	{
		PORTA=dsda1;
		PORTA=dsda1b;
	}
	else
	{
		PORTA=dsda0;
		PORTA=dsda0b;
	}
	if(d&0x08)
	{
		PORTA=dsda1;
		PORTA=dsda1b;
	}
	else
	{
		PORTA=dsda0;
		PORTA=dsda0b;
	}
	if(d&0x04)
	{
		PORTA=dsda1;
		PORTA=dsda1b;
	}
	else
	{
		PORTA=dsda0;
		PORTA=dsda0b;
	}
	if(d&0x02)
	{
		PORTA=dsda1;
		PORTA=dsda1b;
	}
	else
	{
		PORTA=dsda0;
		PORTA=dsda0b;
	}
	if(d&0x01)
	{
		PORTA=dsda1;
		PORTA=dsda1b;
	}
	else
	{
		PORTA=dsda0;
		PORTA=dsda0b;
	}
}
void lcd_spi_write16(unsigned short d)
{
	lcd_spi_write8(d>>8);
	lcd_spi_write8(d&0xff);	
}
void lcd_spi_writenb(const unsigned char *d,unsigned short n)
{
	for(unsigned short i=0;i<n;i++)
		lcd_spi_write8(d[i]);
}

/******************************************************************************
	lcd_spi_writen_v1
*******************************************************************************	
	Write n bytes to the LCD SPI interface, using version 1 interface.
	
	This function does not tolerate changes to port A or port B during the execution
	of the function (e.g. in an interrupt), as the state of the ports is cached for 
	faster writes.
	
	#define LCD_V1 if the hardware is v1.
	
******************************************************************************/
#ifdef LCD_V1
void lcd_spi_writen(const unsigned char *data,unsigned short n)
#else
void lcd_spi_writen_v1(const unsigned char *data,unsigned short n)
#endif
{
	unsigned char dsda0=PORTA&(~SPI_SDAMASK);
	unsigned char dsda1=dsda0|SPI_SDAMASK;
	unsigned char dscl0=PORTB&(~SPI_SCLMASK);
	unsigned char dscl1=dscl0|SPI_SCLMASK;
	
	for(unsigned short i=0;i<n;i++)
	{
		unsigned char d = data[i];
		if(d&0x80)
			PORTA=dsda1;
		else
			PORTA=dsda0;
		PORTB=dscl1;
		PORTB=dscl0;
		if(d&0x40)
			PORTA=dsda1;
		else
			PORTA=dsda0;
		PORTB=dscl1;
		PORTB=dscl0;
		if(d&0x20)
			PORTA=dsda1;
		else
			PORTA=dsda0;
		PORTB=dscl1;
		PORTB=dscl0;
		if(d&0x10)
			PORTA=dsda1;
		else
			PORTA=dsda0;
		PORTB=dscl1;
		PORTB=dscl0;
		if(d&0x08)
			PORTA=dsda1;
		else
			PORTA=dsda0;
		PORTB=dscl1;
		PORTB=dscl0;
		if(d&0x04)
			PORTA=dsda1;
		else
			PORTA=dsda0;
		PORTB=dscl1;
		PORTB=dscl0;
		if(d&0x02)
			PORTA=dsda1;
		else
			PORTA=dsda0;
		PORTB=dscl1;
		PORTB=dscl0;
		if(d&0x01)
			PORTA=dsda1;
		else
			PORTA=dsda0;
		PORTB=dscl1;
		PORTB=dscl0;
	}
}

/******************************************************************************
	lcd_spi_writen_v2
*******************************************************************************	
	Write n bytes to the LCD SPI interface, using version 2 interface.
	
	This function does not tolerate changes to port A or port B during the execution
	of the function (e.g. in an interrupt), as the state of the ports is cached for 
	faster writes.
	
	#define LCD_V2 if the hardware is v2.
	
******************************************************************************/
#ifdef LCD_V2
#ifdef LCD_V2ASM
void lcd_spi_writen_v2(const unsigned char *data,unsigned short n)
#else
void lcd_spi_writen(const unsigned char *data,unsigned short n)
#endif
#else
void lcd_spi_writen_v2(const unsigned char *data,unsigned short n)
#endif
{
	//X_ADC0 = MOSI = PA0
	//X_ADC1 = SCK = PA1
	
	unsigned char dsda0=PORTA&(~(SPI_SDAMASK+SPI_SCLMASK));			// CLK=0, D=0
	unsigned char dsda1=dsda0|SPI_SDAMASK;											// CLK=0, D=1
	unsigned char dsda0b=(PORTA&(~SPI_SDAMASK))|SPI_SCLMASK;		// CLK=1, D=0
	unsigned char dsda1b=dsda0b|SPI_SDAMASK;										// CLK=1, D=1
	
	for(unsigned short i=0;i<n;i++)
	{
		unsigned char d = data[i];
		
		if(d&0x80)
		{
			PORTA=dsda1;
			PORTA=dsda1b;
		}
		else
		{
			PORTA=dsda0;
			PORTA=dsda0b;
		}
		if(d&0x40)
		{
			PORTA=dsda1;
			PORTA=dsda1b;
		}
		else
		{
			PORTA=dsda0;
			PORTA=dsda0b;
		}
		if(d&0x20)
		{
			PORTA=dsda1;
			PORTA=dsda1b;
		}
		else
		{
			PORTA=dsda0;
			PORTA=dsda0b;
		}
		if(d&0x10)
		{
			PORTA=dsda1;
			PORTA=dsda1b;
		}
		else
		{
			PORTA=dsda0;
			PORTA=dsda0b;
		}
		if(d&0x08)
		{
			PORTA=dsda1;
			PORTA=dsda1b;
		}
		else
		{
			PORTA=dsda0;
			PORTA=dsda0b;
		}
		if(d&0x04)
		{
			PORTA=dsda1;
			PORTA=dsda1b;
		}
		else
		{
			PORTA=dsda0;
			PORTA=dsda0b;
		}
		if(d&0x02)
		{
			PORTA=dsda1;
			PORTA=dsda1b;
		}
		else
		{
			PORTA=dsda0;
			PORTA=dsda0b;
		}
		if(d&0x01)
		{
			PORTA=dsda1;
			PORTA=dsda1b;
		}
		else
		{
			PORTA=dsda0;
			PORTA=dsda0b;
		}
	}
}


void lcd_write(unsigned char d)
{
	lcd_nselect(0);
	lcd_spi_write8(d);
	lcd_nselect(1);
}

void lcd_write_cmd(unsigned char d)
{
	lcd_cmd();
	lcd_write(d);
}

void lcd_write_cmd_cont(unsigned char d)
{
	lcd_cmd();
	lcd_spi_write8(d);
}

void lcd_write_data8(unsigned char d)
{
	lcd_data();
	lcd_write(d);
}
void lcd_write_data16(unsigned char d)
{
	lcd_data();
	lcd_write(d>>8);
	lcd_write(d&0xff);
}
void lcd_write_data8_cont(unsigned char d)
{
	lcd_data();
	lcd_spi_write8(d);
}
void lcd_write_data16_cont(unsigned char d)
{
	lcd_data();
	lcd_spi_write16(d);
}


void lcd_write_cmd_data16(unsigned char c,unsigned char dh,unsigned char dl)
{
	lcd_nselect(0);
	
	lcd_cmd();
	lcd_spi_write8(c);
	lcd_data();
	lcd_spi_write8(dh);
	lcd_spi_write8(dl);
	
	lcd_nselect(1);
}
void lcd_write_cmd_data24(unsigned char c,unsigned char d1,unsigned char d2,unsigned char d3)
{
	lcd_nselect(0);	
	lcd_cmd();
	lcd_spi_write8(c);
	lcd_data();
	lcd_spi_write8(d1);
	lcd_spi_write8(d2);
	lcd_spi_write8(d3);	
	lcd_nselect(1);
}
void lcd_write_cmd_data32(unsigned char c,unsigned char dhh,unsigned char dhl,unsigned char dlh,unsigned char dll)
{
	lcd_nselect(0);	
	lcd_cmd();
	lcd_spi_write8(c);
	lcd_data();
	lcd_spi_write8(dhh);
	lcd_spi_write8(dhl);
	lcd_spi_write8(dlh);
	lcd_spi_write8(dll);
	lcd_nselect(1);
}
void lcd_write_cmd_datan(unsigned char c,unsigned char n,const unsigned char *data)
{
	lcd_nselect(0);
	
	lcd_cmd();
	lcd_spi_write8(c);
	lcd_data();
	lcd_spi_writen(data,n);
	lcd_nselect(1);
}
void lcd_write_cmd_data8(unsigned char c,unsigned char d)
{
	lcd_nselect(0);
	
	lcd_cmd();
	lcd_spi_write8(c);
	lcd_data();
	lcd_spi_write8(d);
	
	lcd_nselect(1);
}

void lcd_init(void)
{
	CHECKLCDENABLED;
	// Write many nop just in case
	//for(int i=i;i<30;i++)
	//	lcd_write_cmd(CMD_NOP);
		
	
	//writecommand_cont(CMD_SWRESET);//software reset
		//delay(500);
		
	
		
		
		
	lcd_write_cmd(CMD_SWRESET);
	_delay_ms(500);
	
	
	
		
		//writecommand_cont(CMD_SLPOUT);//exit sleep
		//delay(5);
		
	lcd_write_cmd(CMD_SLPOUT);
	_delay_ms(5);
		
		//writecommand_cont(CMD_PIXFMT);//Set Color Format 16bit   
		//writedata8_cont(0x05);
		//delay(5);
	
	//lcd_write_cmd(CMD_PIXFMT);
	//lcd_write_data8(0x05);
	lcd_write_cmd_data8(CMD_PIXFMT,0x05);				// Dan: works, default, datasheet says 3D for RGB 565 16-bit mode
	//lcd_write_cmd_data8(CMD_PIXFMT,0x3D);					// Also works RGB 565
	//lcd_write_cmd_data8(CMD_PIXFMT,0x3B);					// RGB 444
	
	
	_delay_ms(5);
	
		
		//writecommand_cont(CMD_GAMMASET);//default gamma curve 3
		//writedata8_cont(0x08);//0x04
		//delay(1);
		
	//lcd_write_cmd(CMD_GAMMASET);
	//lcd_write_data8(0x08);
	//lcd_write_data8(0x04);
	lcd_write_cmd_data8(CMD_GAMMASET,0x04);	
	_delay_ms(1);
		
		//writecommand_cont(CMD_GAMRSEL);//Enable Gamma adj    
		//writedata8_cont(0x01); 
		//delay(1);
		
	//lcd_write_cmd(CMD_GAMRSEL);
	//lcd_write_data8(0x01);
	lcd_write_cmd_data8(CMD_GAMRSEL,0x01);	
	_delay_ms(1);
		
		//writecommand_cont(CMD_NORML);
		
	lcd_write_cmd(CMD_NORML);
	
		//writecommand_cont(CMD_DFUNCTR);
		//writedata8_cont(0b11111111);//
		//writedata8_cont(0b00000110);//
		
	//lcd_write_cmd(CMD_DFUNCTR);
	//lcd_write_data8(0b11111111);
	//lcd_write_data8(0b00000110);
	
	lcd_write_cmd_data16(CMD_DFUNCTR,0b11111111,0b00000110);
	

		//writecommand_cont(CMD_PGAMMAC);//Positive Gamma Correction Setting
		//for (i=0;i<15;i++){
//			writedata8_cont(pGammaSet[i]);
	//	}
	
	/*
	lcd_write_cmd(CMD_PGAMMAC);
	for(int i=0;i<15;i++)
		lcd_write_data8(pGammaSet[i]);*/
	/*lcd_nselect(0);
	lcd_cmd();
	lcd_spi_write8(CMD_PGAMMAC);
	lcd_data();
	for(int i=0;i<15;i++)
		lcd_spi_write8(pGammaSet[i]);
	lcd_nselect(1);*/
	lcd_write_cmd_datan(CMD_PGAMMAC,16,pGammaSet);
	
		
		//writecommand_cont(CMD_NGAMMAC);//Negative Gamma Correction Setting
		//for (i=0;i<15;i++){
//			writedata8_cont(nGammaSet[i]);
		//}
	
	/*lcd_write_cmd(CMD_NGAMMAC);
	for(int i=0;i<15;i++)
		lcd_write_data8(nGammaSet[i]);*/
		
	/*lcd_nselect(0);
	lcd_cmd();
	lcd_spi_write8(CMD_NGAMMAC);
	lcd_data();
	for(int i=0;i<15;i++)
		lcd_spi_write8(nGammaSet[i]);
	lcd_nselect(1);*/
	lcd_write_cmd_datan(CMD_NGAMMAC,16,nGammaSet);
	
		//writecommand_cont(CMD_FRMCTR1);//Frame Rate Control (In normal mode/Full colors)
		//writedata8_cont(0x08);//0x0C//0x08
		//writedata8_cont(0x02);//0x14//0x08
		//delay(1);
		
	/*lcd_write_cmd(CMD_FRMCTR1);
	lcd_write_data8(0x08);
	lcd_write_data8(0x02);*/
	lcd_write_cmd_data16(CMD_FRMCTR1,0x08,0x02);
	_delay_ms(1);
	
		//writecommand_cont(CMD_DINVCTR);//display inversion 
		//writedata8_cont(0x07);
		//delay(1);
		
	//lcd_write_cmd(CMD_DINVCTR);
	//lcd_write_data8(0x07);
	lcd_write_cmd_data8(CMD_DINVCTR,0x07);
	_delay_ms(1);
		
	
		//writecommand_cont(CMD_PWCTR1);//Set VRH1[4:0] & VC[2:0] for VCI1 & GVDD   
		//writedata8_cont(0x0A);//4.30 - 0x0A
		//writedata8_cont(0x02);//0x05
		//delay(1);
		
	//lcd_write_cmd(CMD_PWCTR1);
	//lcd_write_data8(0x0A);
	//lcd_write_data8(0x02);
	lcd_write_cmd_data16(CMD_PWCTR1,0x0A,0x02);
	_delay_ms(1);

	
		//writecommand_cont(CMD_PWCTR2);//Set BT[2:0] for AVDD & VCL & VGH & VGL   
		//writedata8_cont(0x02);
		//delay(1);
		
	//lcd_write_cmd(CMD_PWCTR2);
	//lcd_write_data8(0x02);
	lcd_write_cmd_data8(CMD_PWCTR2,0x02);
	_delay_ms(1);
	
		//writecommand_cont(CMD_VCOMCTR1);//Set VMH[6:0] & VML[6:0] for VOMH & VCOML   
		//writedata8_cont(0x50);//0x50
		//writedata8_cont(99);//0x5b
		//delay(1);
		
	//lcd_write_cmd(CMD_VCOMCTR1);
	//lcd_write_data8(0x50);
	//lcd_write_data8(99);
	lcd_write_cmd_data16(CMD_VCOMCTR1,0x50,99);
	_delay_ms(1);
	
		//writecommand_cont(CMD_VCOMOFFS);
		//writedata8_cont(0);//0x40
		//delay(1);
		
	//lcd_write_cmd(CMD_VCOMOFFS);
	//lcd_write_data8(0);
	lcd_write_cmd_data8(CMD_VCOMOFFS,0);
	_delay_ms(1);
  
		//writecommand_cont(CMD_CLMADRS);//Set Column Address  
		//writedata16_cont(0x00);
		//writedata16_cont(_GRAMWIDTH); 
	
	//lcd_write_cmd(CMD_CLMADRS);
	//lcd_write_data16(0);
	//lcd_write_data16(_GRAMWIDTH);
	lcd_write_cmd_data32(CMD_CLMADRS,0,0,_GRAMWIDTH>>8,_GRAMWIDTH);
	  
		//writecommand_cont(CMD_PGEADRS);//Set Page Address  
		//writedata16_cont(0x00);
		//writedata16_cont(_GRAMHEIGH); 
		
	//lcd_write_cmd(CMD_PGEADRS);
	//lcd_write_data16(0);
	//lcd_write_data16(_GRAMHEIGH);
	lcd_write_cmd_data32(CMD_PGEADRS,0,0,_GRAMHEIGH>>8,_GRAMHEIGH);
	
	
		
		
		
		// set scroll area (thanks Masuda)
		//writecommand_cont(CMD_VSCLLDEF);
		//writedata16_cont(__OFFSET);
		//writedata16_cont(_GRAMHEIGH - __OFFSET);
		//writedata16_last(0);
		
	//lcd_write_cmd(CMD_VSCLLDEF);
	//lcd_write_data16(__OFFSET);
	//lcd_write_data16(_GRAMHEIGH - __OFFSET);
	//lcd_write_data16(0);
	lcd_write_cmd_data32(CMD_VSCLLDEF,__OFFSET>>8,__OFFSET,(_GRAMHEIGH - __OFFSET)>>8,_GRAMHEIGH - __OFFSET);
	
	/*
	colorSpace(_colorspaceData);
		
		setRotation(0);
		writecommand(CMD_DISPON);//display ON 
		delay(1);
		writecommand(CMD_RAMWR);//Memory Write
		
		delay(1);
	#endif
	if (_bklPin != 255) digitalWrite(_bklPin,HIGH);
	fillScreen(_defaultBackground);
	*/		
	//lcd_write_cmd_data8(CMD_MADCTL,0x36);		// Default
	lcd_write_cmd_data8(CMD_MADCTL,0x08); 		// 

		
	
	
	lcd_write_cmd(CMD_DISPON);//display ON 
	_delay_ms(1);
	
	
	
	//lcd_write_cmd(CMD_RAMWR);//Memory Write

	//lcd_fillScreen(0x1234);
	
	lcd_fillScreen(0xffff);
	
	// Try to figure out the color mapping
	// RGB-565: 16 bits for one pixel. RRRR RGGG GGGB BBBB
	for(int y=30;y<40;y++)
	{
		for(int x=30;x<40;x++)
		{
			// Red in RGB-565
			lcd_pixel(x,y,0b1111100000000000);
		}
		for(int x=40;x<50;x++)
		{
			// Green in RGB-565
			lcd_pixel(x,y,0b0000011111100000);
		}
		for(int x=50;x<70;x++)
		{
			// Blue in RGB-565
			lcd_pixel(x,y,0b0000000000011111);
		}
		for(int x=70;x<80;x++)
		{
			// Blue in RGB-565
			lcd_pixel(x,y,0b0000000000010000);
		}
	}
	// RGB-444: 12 bits for one pixel. RRRR GGGG BBBB rrrr gggg bbbb (lowrcase: next pixel; write 3 bytes for 2 pixels)
	for(int y=40;y<50;y++)
	{
		for(int x=30;x<40;x+=2)
		{
			// Red in RGB-444
			lcd_pixel2(x,y,0b11110000,0b00001111,0b00000000);
		}
		for(int x=40;x<50;x+=2)
		{
			// Green in RGB-444
			lcd_pixel2(x,y,0b00001111,0b00000000,0b11110000);
		}
		for(int x=50;x<70;x+=2)
		{
			// Blue in RGB-444
			lcd_pixel2(x,y,0b00000000,0b11110000,0b00001111);
		}
		for(int x=70;x<80;x+=2)
		{
			// Blue in RGB-444
			lcd_pixel2(x,y,0b00000000,0b10000000,0b00001000);
		}
	}
	
	
}
void lcd_fillScreen(unsigned short color)
{
	CHECKLCDENABLED;
	unsigned short px;
	
	
	
	lcd_setWindowAdr(0x00,0x00,_GRAMWIDTH,_GRAMHEIGH);
	
	lcd_nselect(0);
	lcd_write_cmd_cont(CMD_RAMWR);
	lcd_data();
	for(px=0;px<_GRAMSIZE;px++)
	{ 
		lcd_spi_write16(color+px);
	}
	lcd_nselect(1);
	
	
	
}

// Clear screen fast
void lcd_clear565(unsigned short color)
{
	CHECKLCDENABLED;
	const unsigned char n=64;
	unsigned short px[n];
	unsigned short color2;
	
	color2=(color>>8)|((color&0xff)<<8);
	
	for(unsigned char i=0;i<n;i++)
		px[i]=color2;
		
	//lcd_setWindowAdr(0x00,0x00,_GRAMWIDTH,_GRAMHEIGH);
	lcd_setWindowAdr(0x00,0x00,127,127);
	
	lcd_nselect(0);
	lcd_write_cmd_cont(CMD_RAMWR);
	lcd_data();
	unsigned short it = 128*128/n;
	for(unsigned short i=0;i<it;i++)
	//for(unsigned short i=0;i<128*128;i++)
	{ 
		lcd_spi_writen((unsigned char*)px,n*sizeof(unsigned short));
		//lcd_spi_write16(color);
	}
	lcd_nselect(1);
	
	
	
}

void lcd_setWindowAdr(unsigned short x0,unsigned short y0,unsigned short x1,unsigned short y1) 
{
	CHECKLCDENABLED;
/*
	writecommand_cont(CMD_CLMADRS); // Column
		
			writedata16_cont(x0);
			writedata16_cont(x1);*/
	lcd_write_cmd_data32(CMD_CLMADRS,x0>>8,x0,x1>>8,x1);		
	
		
		/*writecommand_cont(CMD_PGEADRS); // Page
		
			writedata16_cont(y0 + __OFFSET);
			writedata16_cont(y1 + __OFFSET);*/
	lcd_write_cmd_data32(CMD_PGEADRS,(y0 + __OFFSET)>>8,y0 + __OFFSET,(y1 + __OFFSET)>>8,y1 + __OFFSET);
		
}

/*
#else
		writecommand(CMD_SWRESET);//software reset
		delay(500);
		
		writecommand(CMD_SLPOUT);//exit sleep
		delay(5);
		
		writecommand(CMD_PIXFMT);//Set Color Format 16bit   
		writedata(0x05);
		delay(5);
		
		writecommand(CMD_GAMMASET);//default gamma curve 3
		writedata(0x04);//0x04
		delay(1);
		
		writecommand(CMD_GAMRSEL);//Enable Gamma adj    
		writedata(0x01); 
		delay(1);
		
		writecommand(CMD_NORML);
	
		writecommand(CMD_DFUNCTR);
		writedata(0b11111111);//
		writedata(0b00000110);//

		writecommand(CMD_PGAMMAC);//Positive Gamma Correction Setting
		for (i=0;i<15;i++){
			writedata(pGammaSet[i]);
		}
		
		writecommand(CMD_NGAMMAC);//Negative Gamma Correction Setting
		for (i=0;i<15;i++){
			writedata(nGammaSet[i]);
		}

		writecommand(CMD_FRMCTR1);//Frame Rate Control (In normal mode/Full colors)
		writedata(0x08);//0x0C//0x08
		writedata(0x02);//0x14//0x08
		delay(1);
		
		writecommand(CMD_DINVCTR);//display inversion 
		writedata(0x07);
		delay(1);
		
		writecommand(CMD_PWCTR1);//Set VRH1[4:0] & VC[2:0] for VCI1 & GVDD   
		writedata(0x0A);//4.30 - 0x0A
		writedata(0x02);//0x05
		delay(1);
		
		writecommand(CMD_PWCTR2);//Set BT[2:0] for AVDD & VCL & VGH & VGL   
		writedata(0x02);
		delay(1);
		
		writecommand(CMD_VCOMCTR1);//Set VMH[6:0] & VML[6:0] for VOMH & VCOML   
		writedata(0x50);//0x50
		writedata(99);//0x5b
		delay(1);
		
		writecommand(CMD_VCOMOFFS);
		writedata(0);//0x40
		delay(1);
  
		writecommand(CMD_CLMADRS);//Set Column Address  
		writedata16(0x00); 
		writedata16(_GRAMWIDTH); 
  
		writecommand(CMD_PGEADRS);//Set Page Address  
		writedata16(0X00); 
		writedata16(_GRAMHEIGH);
		// set scroll area (thanks Masuda)
		writecommand(CMD_VSCLLDEF);
		writedata16(__OFFSET);
		writedata16(_GRAMHEIGH - __OFFSET);
		writedata16(0);
		
		colorSpace(_colorspaceData);
		
		setRotation(0);
		writecommand(CMD_DISPON);//display ON 
		delay(1);
		writecommand(CMD_RAMWR);//Memory Write
		
		delay(1);
	#endif
	if (_bklPin != 255) digitalWrite(_bklPin,HIGH);
	fillScreen(_defaultBackground);
*/






void lcd_pixel(unsigned short x, unsigned short y, unsigned short color)
{
	CHECKLCDENABLED;
	//lcd_nselect(0);
	//lcd_addrwindow(x,y,x+1,y+1);
	
	//lcd_write_data16_cont(color); 
	
	//lcd_nselect(1);
	
	lcd_setWindowAdr(x,y,x+1,y+1);
	
	lcd_write_cmd_data16(CMD_RAMWR,color>>8,color);
	/*lcd_nselect(0);
	lcd_write_cmd_cont(CMD_RAMWR);
	lcd_data();
	lcd_spi_write16(color);
	lcd_nselect(1);*/
}


/******************************************************************************
	lcd_invert
*******************************************************************************	
	Sets display inversion
	
******************************************************************************/
void lcd_invert(unsigned char inv)
{
	CHECKLCDENABLED;
	if(inv)
		lcd_write_cmd(CMD_DINVON);
	else
		lcd_write_cmd(CMD_DINVOF);
}

// Used for RGB 444
void lcd_pixel2(unsigned short x, unsigned short y, unsigned char p1,unsigned char p2,unsigned char p3)
{
	CHECKLCDENABLED;
	
	//lcd_nselect(0);
	//lcd_addrwindow(x,y,x+1,y+1);
	
	//lcd_write_data16_cont(color); 
	
	//lcd_nselect(1);
	
	lcd_setWindowAdr(x,y,x+2,y+1);
	
	lcd_write_cmd_data24(CMD_RAMWR,p1,p2,p3);
	/*lcd_nselect(0);
	lcd_write_cmd_cont(CMD_RAMWR);
	lcd_data();
	lcd_spi_write16(color);
	lcd_nselect(1);*/
	
	
	
}


// send framebuffer to display

/*
	Numbe of SPI writes
	
	One line: 128 pixels, 3bytes/2pixels: 128/2*3=192 bytes=1536 bits.
	Assuming 16 clocks/bit (not optimal): 1536*16 = 24576 clocks/line (without setup). At 7.3MHz: 1/300s/ line: 3.33ms/line
	
	Target for background update: sub ms.
	
	Goal: 0.5ms: 1/8 faster -> can update 16 pixels in each interrupt. 16pixels/ms. 8ms / line -> 1024ms/frame (1Hz update?)
	
	
	
		
*/
void lcd_sendframe(void)
{
	CHECKLCDENABLED;
	//lcd_sendframe444();
	lcd_sendframe565();
}

void lcd_sendframe444(void)
{
	unsigned char b;
	const unsigned char *d;
	const unsigned char *fb = fb_framebuffer;
	// Set address to 0,0,limits to 128x128
	
	
	//printf("before setwindowadr\n");
	lcd_setWindowAdr(0,0,128,128);
	//printf("aftersetwindowadr\n");
	
	//_delay_ms(5000);
	//printf("before nselect\n");
	//_delay_ms(5000);
	
	// Start write command
	lcd_nselect(0);
	
	//printf("after nselect\n");
	
	//_delay_ms(5000);
	//printf("before write cmd_cont\n");
	
	lcd_write_cmd_cont(CMD_RAMWR);
	
	//printf("after write cmd_cont\n");
	
	//_delay_ms(5000);
	//printf("before write data\n");
	
	lcd_data();
	// 128 lines
	//for(y=0;y<128;y++)			
	//{
		// 128 columns - 16 bytes (32 nibbles)
		//for(x=0;x<16;x++)
		for(unsigned short i=0;i<2048;i++)
		{
			b = *fb;
			// High nibble to RGB444
			d = lcd_nibble_to_rgb444[b>>4];
			// Send to panel
			lcd_spi_writen(d,6);
			// Low nibble to RGB444
			d = lcd_nibble_to_rgb444[b&0x0F];
			// Send to panel
			lcd_spi_writen(d,6);
			fb++;
		}
	//}
	lcd_nselect(1);
}				
void lcd_sendframe565(void)
{
	unsigned char b;
	const unsigned char *d;
	const unsigned char *fb = fb_framebuffer;
	// Set address to 0,0,limits to 128x128
	
	
	//printf("before setwindowadr\n");
	//lcd_setWindowAdr(0,0,128,128);
	lcd_setWindowAdr(0,0,127,127);
	//printf("aftersetwindowadr\n");
	
	//_delay_ms(5000);
	//printf("before nselect\n");
	//_delay_ms(5000);
	
	// Start write command
	lcd_nselect(0);
	
	//printf("after nselect\n");
	
	//_delay_ms(5000);
	//printf("before write cmd_cont\n");
	
	lcd_write_cmd_cont(CMD_RAMWR);
	
	//printf("after write cmd_cont\n");
	
	//_delay_ms(5000);
	//printf("before write data\n");
	
	lcd_data();
	// 128 lines
	//for(y=0;y<128;y++)			
	//{
		// 128 columns - 16 bytes (32 nibbles)
		//for(x=0;x<16;x++)
		for(unsigned short i=0;i<2048;i++)
		{
			b = *fb;
			// High nibble to RGB444
			d = lcd_nibble_to_rgb565[b>>4];
			// Send to panel
			lcd_spi_writen(d,8);
			// Low nibble to RGB444
			d = lcd_nibble_to_rgb565[b&0x0F];
			// Send to panel
			lcd_spi_writen(d,8);
			fb++;
		}
	//}
	lcd_nselect(1);
}				

void lcd_bench(void)
{
	
	
	
	// screen fill: 820ms
	
	
	printf("LCD bench\n");
	
	//_delay_ms(5000);
	//printf("before fb_clear\n");
	
	fb_clear();
	
	//_delay_ms(5000);
	//printf("before sendframe\n");
	
	lcd_sendframe();
	
	//_delay_ms(5000);
	//printf("before loop\n");
	for(int x=40;x<50;x++)
		for(int y=70;y<80;y++)
			fb_putpixel(x,y,1);
	fb_putstring(0,0,"Hello world!!");
	fb_putchar(1,1,'B');
	fb_putchar(2,2,'C');
	lcd_sendframe();
	//while(1);
	unsigned long int t1,t2;
	printf("Time fillscreen: ");
	for(int i=0;i<5;i++)
	{
		t1 = timer_ms_get();
		lcd_fillScreen(0xffff);
		t2 = timer_ms_get();
		printf("%ld ",t2-t1);
	}
	printf("\nTime sendframe: ");
	for(int i=0;i<5;i++)
	{
		t1 = timer_ms_get();
		lcd_sendframe();
		t2 = timer_ms_get();
		printf("%ld ",t2-t1);
	}
	printf("\n");
	
	
	for(int j=0;j<8;j++)
	{
		char str[35];
		for(int i=0;i<32;i++)
		{
			str[i] = j*32+i;
		}
		str[33]=0;
		fprintf(file_fb,"%s\n",str);
	}
	lcd_sendframe();
	
	
	// Test display inversion
	for(unsigned i=0;i<9;i++)
	{
		lcd_invert(i&1);
		_delay_ms(200);
	}
	
	//lcd_fillScreen(0x1234);
	printf("LCD bench done\n");
	
	while(1);
}


void lcd_writechar(unsigned char c,unsigned char x,unsigned char y,unsigned s,unsigned short cbg,unsigned short cfg)
{
	CHECKLCDENABLED;
	//printf("Write '%c' at %d,%d\n",c,x,y);
	
	unsigned char pix[2][16];
	//memset(pix[0],0x00,16);
	//memset(pix[1],0xff,16);
	for(int i=0;i<8;i++)
	{
		pix[0][i*2]=cbg>>8;
		pix[0][i*2+1]=cbg&0xFf;
		pix[1][i*2]=cfg>>8;
		pix[1][i*2+1]=cfg&0xFf;
	}
	
	if(c<32 || c>126)
		return;
	if(s==0 || s>8)
		return;
	
	// Define a window around the character
	lcd_setWindowAdr(x,y,x+4*s-1,y+6*s);
//	lcd_setWindowAdr(x,y,x+4*s,y+6*s);

	
	// Start write command
	lcd_nselect(0);
	
	lcd_write_cmd_cont(CMD_RAMWR);
	
	lcd_data();
	for(unsigned char y=0;y<3;y++)			
	{
		// Read 2 nibbles for 2 lines
		unsigned char l2 = __font_4x6[(c-32)*3 + y];	
		
		//l2=0x5a;
		
		// Write s lines
		for(unsigned char si=0;si<s;si++)
		{
			unsigned char b = l2;					
			// Write s columns of pixels on or off for each bit set in b1
			for(unsigned char x=0;x<4;x++)
			{
				//lcd_spi_writen(pix[1],s*2);
				if(b&0x80)
				{
					lcd_spi_writen(pix[1],s*2);
				}
				else
				{
					lcd_spi_writen(pix[0],s*2);
				}
				b<<=1;
			}
		}
		// 
		l2<<=4;
		// Write s lines
		for(unsigned char si=0;si<s;si++)
		{
			unsigned char b = l2;					
			// Write s columns of pixels on or off for each bit set in b1
			for(unsigned char x=0;x<4;x++)
			{
				//lcd_spi_writen(pix[1],s*2);
				if(b&0x80)
				{
					lcd_spi_writen(pix[1],s*2);
				}
				else
				{
					lcd_spi_writen(pix[0],s*2);
				}
				b<<=1;
			}
		}		
	}
	lcd_nselect(1);
}
void lcd_writestring(const char *str,unsigned char x,unsigned char y,unsigned s,unsigned short cbg,unsigned short cfg)
{
	CHECKLCDENABLED;
	for(unsigned char i=0;i<strlen(str);i++)
	{
		lcd_writechar(str[i],x+i*s*4,y,s,cbg,cfg);
	}
}

/*void lcd_line(unsigned char x0,unsigned char y0,unsigned char x1,unsigned char y1)
{
	signed short deltax,deltay,error,deltaerr;
	
	deltax=x1-x0;
	deltay=y1-y0;
	error=0;
	deltaerr = abs(deltay/deltax
	
	 function line(x0, y0, x1, y1)
     real deltax := x1 - x0
     real deltay := y1 - y0
     real error := 0
     real deltaerr := abs (deltay / deltax)    // Assume deltax != 0 (line is not vertical),
           // note that this division needs to be done in a way that preserves the fractional part
     int y := y0
     for x from x0 to x1
         plot(x,y)
         error := error + deltaerr
         while error = 0.5 then
             plot(x, y)
             y := y + sign(y1 - y0)
             error := error - 1.0
}

*/