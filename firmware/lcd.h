#ifndef __LCD_H
#define __LCD_H


#define CHECKLCDENABLED if(!system_enable_lcd) return;

// Define for extension version 1
//#define LCD_V1
// Define for extension version 2
#define LCD_V2

//#define LCD_V2ASM


//ILI9163C registers-----------------------
#define CMD_NOP     	0x00//Non operation
#define CMD_SWRESET 	0x01//Soft Reset
#define CMD_SLPIN   	0x10//Sleep ON
#define CMD_SLPOUT  	0x11//Sleep OFF
#define CMD_PTLON   	0x12//Partial Mode ON
#define CMD_NORML   	0x13//Normal Display ON
#define CMD_DINVOF  	0x20//Display Inversion OFF
#define CMD_DINVON   	0x21//Display Inversion ON
#define CMD_GAMMASET 	0x26//Gamma Set (0x01[1],0x02[2],0x04[3],0x08[4])
#define CMD_DISPOFF 	0x28//Display OFF
#define CMD_DISPON  	0x29//Display ON
#define CMD_IDLEON  	0x39//Idle Mode ON
#define CMD_IDLEOF  	0x38//Idle Mode OFF
#define CMD_CLMADRS   	0x2A//Column Address Set
#define CMD_PGEADRS   	0x2B//Page Address Set

#define CMD_RAMWR   	0x2C//Memory Write
#define CMD_RAMRD   	0x2E//Memory Read
#define CMD_CLRSPACE   	0x2D//Color Space : 4K/65K/262K
#define CMD_PARTAREA	0x30//Partial Area
#define CMD_VSCLLDEF	0x33//Vertical Scroll Definition
#define CMD_TEFXLON		0x35//Tearing Effect Line ON
#define CMD_TEFXLOF		0x34//Tearing Effect Line OFF
#define CMD_MADCTL  	0x36//Memory Access Control
#define CMD_VSSTADRS	0x37//Vertical Scrolling Start address
#define CMD_PIXFMT  	0x3A//Interface Pixel Format
#define CMD_FRMCTR1 	0xB1//Frame Rate Control (In normal mode/Full colors)
#define CMD_FRMCTR2 	0xB2//Frame Rate Control(In Idle mode/8-colors)
#define CMD_FRMCTR3 	0xB3//Frame Rate Control(In Partial mode/full colors)
#define CMD_DINVCTR		0xB4//Display Inversion Control
#define CMD_RGBBLK		0xB5//RGB Interface Blanking Porch setting
#define CMD_DFUNCTR 	0xB6//Display Fuction set 5
#define CMD_SDRVDIR 	0xB7//Source Driver Direction Control
#define CMD_GDRVDIR 	0xB8//Gate Driver Direction Control 

#define CMD_PWCTR1  	0xC0//Power_Control1
#define CMD_PWCTR2  	0xC1//Power_Control2
#define CMD_PWCTR3  	0xC2//Power_Control3
#define CMD_PWCTR4  	0xC3//Power_Control4
#define CMD_PWCTR5  	0xC4//Power_Control5
#define CMD_VCOMCTR1  	0xC5//VCOM_Control 1
#define CMD_VCOMCTR2  	0xC6//VCOM_Control 2
#define CMD_VCOMOFFS  	0xC7//VCOM Offset Control
#define CMD_PGAMMAC		0xE0//Positive Gamma Correction Setting
#define CMD_NGAMMAC		0xE1//Negative Gamma Correction Setting
#define CMD_GAMRSEL		0xF2//GAM_R_SEL


#define _TFTWIDTH  		128//the REAL W resolution of the TFT
#define _TFTHEIGHT 		128//the REAL H resolution of the TFT
#define _GRAMWIDTH      128
#define _GRAMHEIGH      160//160
#define _GRAMSIZE		_GRAMWIDTH * _GRAMHEIGH//*see note 1
#define __COLORSPC		1// 1:GBR - 0:RGB
#define __GAMMASET3		//uncomment for another gamma
#define __OFFSET		32//*see note 2


// -----------------------------------------------------------------------------
// LCD SPI LOW-LEVEL   LCD SPI LOW-LEVEL   LCD SPI LOW-LEVEL   LCD SPI LOW-LEVEL   
// -----------------------------------------------------------------------------
void lcd_spi_init(void);
void lcd_spi_deinit(void);
void lcd_led(char c);
void lcd_cmd(void);
void lcd_data(void);
void lcd_bit(unsigned char b);
void lcd_clk(unsigned char c);
void lcd_nselect(unsigned char ns);
void lcd_write(unsigned char d);

void lcd_spi_write16(unsigned short d);
#ifdef LCD_V2ASM
	extern void lcd_spi_write8(unsigned char d);
	extern void lcd_spi_writen(const unsigned char *data,unsigned char n);
#else
	void lcd_spi_write8(unsigned char d);
	void lcd_spi_writen(const unsigned char *d,unsigned short n);
#endif

// ---------------------------------------------------------------------------------
// LCD CMD HIGH-LEVEL   LCD CMD HIGH-LEVEL   LCD CMD HIGH-LEVEL   LCD CMD HIGH-LEVEL   
// ---------------------------------------------------------------------------------

void lcd_write_cmd(unsigned char d);
void lcd_write_cmd_cont(unsigned char d);
void lcd_write_data8(unsigned char d);
void lcd_write_data16(unsigned char d);
void lcd_write_data8_cont(unsigned char d);
void lcd_write_data16_cont(unsigned char d);

void lcd_write8_cont(unsigned char d);
void lcd_write16_cont(unsigned short d);


void lcd_write_cmd_data16(unsigned char c,unsigned char dh,unsigned char dl);
void lcd_write_cmd_data24(unsigned char c,unsigned char d1,unsigned char d2,unsigned char d3);
void lcd_write_cmd_data32(unsigned char c,unsigned char dhh,unsigned char dhl,unsigned char dlh,unsigned char dll);
void lcd_write_cmd_datan(unsigned char c,unsigned char n,const unsigned char *data);

// 

void lcd_init(void);
void lcd_bench(void);

void lcd_invert(unsigned char inv);


void lcd_addrwindow(unsigned short x0, unsigned short y0, unsigned short x1, unsigned short y1) ;
void lcd_pixel(unsigned short x, unsigned short y, unsigned short color);
void lcd_pixel2(unsigned short x, unsigned short y, unsigned char p1,unsigned char p2,unsigned char p3);

void lcd_fillScreen(unsigned short color);
void lcd_setWindowAdr(unsigned short x0,unsigned short y0,unsigned short x1,unsigned short y1);

void lcd_clear565(unsigned short color);

void lcd_sendframe(void);
void lcd_sendframe444(void);
void lcd_sendframe565(void);
void lcd_writechar(unsigned char c,unsigned char x,unsigned char y,unsigned s,unsigned short cbg,unsigned short cfg);
void lcd_writestring(const char *str,unsigned char x,unsigned char y,unsigned s,unsigned short cbg,unsigned short cfg);


#endif