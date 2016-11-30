#include <string.h>
#include "fb.h"
#include "fnt4x6.h"
#include "lcd.h"

// frame buffer
unsigned char fb_framebuffer[128*128/8];
FILE _file_fb;			// Screen file

FILE *fb_initfb(void)
{
	FILE *f;
	f = &_file_fb;
	fdev_setup_stream(f,fb_term_fputchar,0,_FDEV_SETUP_WRITE);
	return f;
}

void fb_clear(void)
{
	memset(fb_framebuffer,0,2048);
}
void fb_putpixel(unsigned char x,unsigned char y,unsigned char p)
{
	// Get byte at target position
	unsigned char d = fb_framebuffer[y*16+x/8];
	
	if(p)
		d|=(1<<(7-(x%8)));
	else
		d&=~(1<<(7-(x%8)));
	
	
	fb_framebuffer[y*16+x/8] = d;
}
// x coordinate in nibble number: 0 is high nibble of pixel 0, 1 is low nibble of pixel 0, etc.
void fb_putnibble(unsigned char x,unsigned char y,unsigned char n)
{
	// Get byte at target position
	unsigned char d = fb_framebuffer[y*16+x/2];
	if(!(x&1))
		d=(d&0x0f) | (n<<4);
	else
		d=(d&0xf0) | (n);
	fb_framebuffer[y*16+x/2] = d;		
}
// For speed reasons, text is only allowed on a regular Coordinates are in characters, for speed reasons
// Display of 128x128 -> 32x21 characters
void fb_putchar(unsigned char x,unsigned char y,unsigned char t)
{
	if(t<32 || t>126)
		return;
	for(unsigned char i=0;i<3;i++)
	{
		fb_putnibble(x,y*6+2*i,__font_4x6[(t-32)*3 + i]>>4);
		fb_putnibble(x,y*6+2*i+1,__font_4x6[(t-32)*3 + i]&0x0f);
		
		//fb_putnibble(x,y*6+i,0xff);
	}
}						
void fb_putstring(unsigned char x,unsigned char y,const char *t)
{
	for(unsigned char i=0;i<strlen(t);i++)
		fb_putchar(x+i,y,t[i]);
}

unsigned char fb_term_x=0;
unsigned char fb_term_y=0;
unsigned char fb_term_buffer[21][32];

int fb_term_fputchar(char c, FILE*stream)
{
	// Put a character in the framebuffer
	if(c==10)							// Newline
	{
		fb_term_y++;					// Go to next line
		if(fb_term_y>20)			// If beyond fb, scroll all lines up and put pointer on last line
		{
			for(unsigned char i=1;i<21;i++)
			{
				memcpy(fb_term_buffer[i-1],fb_term_buffer[i],32);
			}
			memset(fb_term_buffer[20],' ',32);
			fb_term_y=20;
			fb_term_refresh(0,20,0,31);
		}
		fb_term_x=0;					// Newline does CR-LF
		
		lcd_sendframe();			// Force frame update on newline
	}
	else if(c==13)					// Carrige return
	{
		fb_term_x=0;
	}
	else
	{
		if(fb_term_x<32)
		{
			fb_term_buffer[fb_term_y][fb_term_x]=c;
			fb_term_refresh(fb_term_y,fb_term_y,fb_term_x,fb_term_x);
			fb_term_x++;
		}
	}
	
	return 0;
}
void fb_term_refresh(unsigned char y1,unsigned char y2,unsigned char x1,unsigned char x2)
{
	//for(unsigned char y=0;y<21;y++)
	for(unsigned char y=y1;y<=y2;y++)
	{
		for(unsigned char x=x1;x<=x2;x++)
		{
			fb_putchar(x,y,fb_term_buffer[y][x]);
		}
	}
}