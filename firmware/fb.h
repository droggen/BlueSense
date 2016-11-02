#ifndef __FB_H
#define __FB_H

#include <stdio.h>

extern unsigned char fb_framebuffer[];
extern FILE _file_fb;

FILE *fb_initfb(void);
void fb_clear(void);
void fb_putpixel(unsigned char x,unsigned char y,unsigned char p);
void fb_putchar(unsigned char x,unsigned char y,unsigned char t);
void fb_putstring(unsigned char x,unsigned char y,char *t);

extern unsigned char fb_term_x,fb_term_y;
extern unsigned char fb_term_buffer[21][32];
int fb_term_fputchar(char c, FILE *stream);
void fb_term_refresh(unsigned char y1,unsigned char y2,unsigned char x1,unsigned char x2);

#endif
