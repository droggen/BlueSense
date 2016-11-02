/*

	RGB 444: 12 bit/pixel, 3 byte writes for 2 pixels
	
	RGB444: RRR GGG BBB rrr ggg bbb

*/

// Nibble (4 pixels) to pixel data. 4 pixels -> 6 bytes
const char lcd_nibble_to_rgb444[16][6] = {
																					/* 0000 */ 0b00000000,0b00000000,0b00000000,   0b00000000,0b00000000,0b00000000,
																					/* 0001 */ 0b00000000,0b00000000,0b00000000,   0b00000000,0b00001111,0b11111111,
																					/* 0010 */ 0b00000000,0b00000000,0b00000000,   0b11111111,0b11110000,0b00000000,
																					/* 0011 */ 0b00000000,0b00000000,0b00000000,   0b11111111,0b11111111,0b11111111,
																					/* 0100 */ 0b00000000,0b00001111,0b11111111,   0b00000000,0b00000000,0b00000000,
																					/* 0101 */ 0b00000000,0b00001111,0b11111111,   0b00000000,0b00001111,0b11111111,
																					/* 0110 */ 0b00000000,0b00001111,0b11111111,   0b11111111,0b11110000,0b00000000,
																					/* 0111 */ 0b00000000,0b00001111,0b11111111,   0b11111111,0b11111111,0b11111111,
																					/* 1000 */ 0b11111111,0b11110000,0b00000000,   0b00000000,0b00000000,0b00000000,
																					/* 1001 */ 0b11111111,0b11110000,0b00000000,   0b00000000,0b00001111,0b11111111,
																					/* 1010 */ 0b11111111,0b11110000,0b00000000,   0b11111111,0b11110000,0b00000000,
																					/* 1011 */ 0b11111111,0b11110000,0b00000000,   0b11111111,0b11111111,0b11111111,
																					/* 1100 */ 0b11111111,0b11111111,0b11111111,   0b00000000,0b00000000,0b00000000,
																					/* 1101 */ 0b11111111,0b11111111,0b11111111,   0b00000000,0b00001111,0b11111111,
																					/* 1110 */ 0b11111111,0b11111111,0b11111111,   0b11111111,0b11110000,0b00000000,
																					/* 1111 */ 0b11111111,0b11111111,0b11111111,   0b11111111,0b11111111,0b11111111
																				};

// frame buffer
char lcd_framebuffer[128*128/8];

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
	unsigned char x,y;
	unsigned char b;
	unsigned char *d;
	
	// Set address to 0,0,limits to 128x128
	lcd_nselect(0);
	lcd_cmd(0);
	lcd_addrwindow(0,0,128,128);
	// Start write command
	//lcd_write(CMD_RAMWR);
	lcd_data();
	// 128 lines
	for(y=0;y<128;y++)			
	{
		// 128 columns - 16 bytes (32 nibbles)
		for(x=0;x<16;x++)
		{
			b = lcd_framebuffer[y*16 + x];
			// High nibble to RGB444
			d = lcd_nibble_to_rgb444[b>>4];
			// Send to panel
			lcd_spi_write(d,16);
			// Low nibble to RGB444
			d = lcd_nibble_to_rgb444[b&0x0F];
			// Send to panel
			lcd_spi_write(d,16);
		}
	}
	lcd_nselect(1);
}				
									
