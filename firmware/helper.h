#ifndef __HELPER_H
#define __HELPER_H

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


extern const char crlf[] PROGMEM;

char *trim(char *string);
void bin2bcd(char *digits,unsigned int val );
void bin2bcd2(char *digit,unsigned short inValue);
void utoaz (unsigned char *outstring, unsigned int number);
void u16toascii(char *digits,unsigned int number);
void s16toascii(char *digits,unsigned int number);

unsigned char hex2chr(unsigned char v);

void helper_test(void);

extern unsigned char ht2(unsigned short v,unsigned char *ptr);
extern unsigned char ht3(unsigned short v,unsigned char *ptr);
extern unsigned char ht3s(unsigned short v,unsigned char *ptr);

void u16toa(unsigned short v,char *ptr);
void s16toa(signed short v,char *ptr);
void u32toa(unsigned long v,char *ptr);
void s32toa(signed long v,char *ptr);
void f16toa(_Fract a,char *ptr);

int peek(FILE *file);
void swalloweol(FILE *file);
unsigned char checkdigits(const char *str,unsigned char n);
unsigned char ParseComma(const char *str,unsigned char n,...);
unsigned char ParseCommaGetInt(const char *str,int n,...);

unsigned char TimeAddSeconds(unsigned short hour, unsigned short min, unsigned short sec, unsigned short ds,unsigned short *ohour, unsigned short *omin, unsigned short *osec);

char *format3s16(char *strptr,signed short x,signed short y,signed short z);
char *format1u32(char *strptr,unsigned long a);
char *format1u16(char *strptr,unsigned short a);
char *format4f16(char *strptr,_Fract q0,_Fract q1,_Fract q2,_Fract q3);



#endif