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

#include "helper.h"
#include "serial.h"

/*
	File: helper

	Library of various helper functions.
	
*/

// Various
const char crlf[] PROGMEM = "\r\n";

/*
	Returns a pointer to a subset of string, removing any leading CR or LF or SPC 
	String is modified to remove any trailing CR or LF or SPC.
*/
char *trim(char *string)
{
	char *r;
	for(size_t i=strlen(string)-1;i>=0;i--)
	{
		if(string[i]==32 || string[i]==0xA || string[i]==0xD)
			string[i]=0;
		else
			break;
	}
	r = string;
	for(size_t i=0;i<strlen(string);i++)
	{
		if(string[i]==32 || string[i]==0xA || string[i]==0xD)
			r=string+i+1;
		else
			break;
	}
	return r;
}




void bin2bcd(char *digit,unsigned int val )
{
  char i;

  i = '0' - 1;
  do
    i++;
  while( !((val -= 10000) & 0x8000) );
  digit[0] = i;

  i = '0' + 10;
  do
    i--;
  while( (val += 1000) & 0x8000 );
  digit[1] = i;

  i = '0' - 1;
  do
    i++;
  while( !((val -= 100) & 0x8000) );
  digit[2] = i;

  i = '0' + 10;
  do
    i--;
  while( (val += 10) & 0x8000 );
  digit[3] = i;

  digit[4] = val | '0';
  
  digit[5] = 0;
}
void bin2bcd2(char *digit,unsigned short inValue)
{
	int loop;
	unsigned char acc0 = 0;
	unsigned char acc1 = 0;
	unsigned char acc2 = 0;
	unsigned char acc3 = 0;
	unsigned char cIn0;

	for (loop = 0;loop < 16; loop++)
	{
                cIn0 = !!(inValue & 0x8000);
                inValue <<= 1;

                if(acc0 >= 5) { acc0 -= 5; }
                acc0 = acc0 * 2 + cIn0;

                if(acc1 >= 5) { acc1 -= 5; }
                acc1 = acc1 * 2 + cIn0;

                if(acc2 >= 5) { acc2 -= 5; }
                acc2 = acc2 * 2 + cIn0;

                if(acc3 >= 5) { acc3 -= 5; }
                acc3 = acc3 * 2 + cIn0;
        }

	digit[0]=acc3+'0';
	digit[1]=acc2+'0';
	digit[2]=acc1+'0';
	digit[3]=acc0+'0';
	digit[4]=0;
}
//eeprom 
/*unsigned int powers[] = {10000, 1000, 100, 10};

void utoaz (unsigned char *outstring, unsigned int number)
{
unsigned int        work;
unsigned char        digit;    // this BCD digit to be output
unsigned char        index;    // which character are we working on.
unsigned int        power;
unsigned char zero = 0;

    index = 0;                // start at the left-most character

    work = (unsigned int) number;

    for (index = 0; index < 4; index++)
        {

           digit = 0;
           power = powers;
           while (work >= power)
               {
               work -= power;
               digit++;
               }
           zero += digit;

         if (zero > 0)
               {
               *outstring++ = digit | 0x30;
               }
           else
               {
               *outstring++ = ' ';
               }
        }

    *outstring++ = ((unsigned char)work) | 0x30;  // last character, even if 0.
    *outstring++ = 0; // null termination

}*/

void u16toascii(char *digits,unsigned int number)
{
	signed long n = number;
	unsigned char i;
	
	i='0'-1;
	do
		i++;
	while( (n-=10000)>=0 );
	digits[0]=i;
	
	n+=10000;
	
	i='0'-1;
	do
		i++;
	while( (n-=1000)>=0 );
	digits[1]=i;
	
	n+=1000;
	
	i='0'-1;
	do
		i++;
	while( (n-=100)>=0 );
	digits[2]=i;
	
	n+=100;
	
	i='0'-1;
	do
		i++;
	while( (n-=10)>=0 );
	digits[3]=i;
	
	n+=10;
	
	i='0'-1;
	do
		i++;
	while( (n-=1)>=0 );
	digits[4]=i;
	
	digits[5]=0;
	
}
void s16toascii(char *digits,unsigned int number)
{
	if(number&0x8000)
	{
		number=-number;
		digits[0]='-';
	}
	else
		digits[0]=' ';
	u16toascii(digits+1,number);
}
/*
    Data types: char is 8 bits, int is 16 bits, long is 32 bits, long long is 64 bits, float and double are 32 bits (this is the only supported floating point format), pointers are 16 bits (function pointers are word addresses, to allow addressing up to 128K program memory space). There is a -mint8 option (see Options for the C compiler avr-gcc) to make int 8 bits, but that is not supported by avr-libc and violates C standards (int must be at least 16 bits). It may be removed in a future release.

    Call-used registers (r18-r27, r30-r31): May be allocated by gcc for local data. You may use them freely in assembler subroutines. Calling C subroutines can clobber any of them - the caller is responsible for saving and restoring.

    Call-saved registers (r2-r17, r28-r29): May be allocated by gcc for local data. Calling C subroutines leaves them unchanged. Assembler subroutines are responsible for saving and restoring these registers, if changed. r29:r28 (Y pointer) is used as a frame pointer (points to local data on stack) if necessary. The requirement for the callee to save/preserve the contents of these registers even applies in situations where the compiler assigns them for argument passing.

    Fixed registers (r0, r1): Never allocated by gcc for local data, but often used for fixed purposes: 

r0 - temporary register, can be clobbered by any C code (except interrupt handlers which save it), may be used to remember something for a while within one piece of assembler code

r1 - assumed to be always zero in any C code, may be used to remember something for a while within one piece of assembler code, but must then be cleared after use (clr r1). This includes any use of the [f]mul[s[u]] instructions, which return their result in r1:r0. Interrupt handlers save and clear r1 on entry, and restore r1 on exit (in case it was non-zero).

    Function call conventions: Arguments - allocated left to right, r25 to r8. All arguments are aligned to start in even-numbered registers (odd-sized arguments, including char, have one free register above them). This allows making better use of the movw instruction on the enhanced core. 

If too many, those that don't fit are passed on the stack.

Return values: 8-bit in r24 (not r25!), 16-bit in r25:r24, up to 32 bits in r22-r25, up to 64 bits in r18-r25. 8-bit return values are zero/sign-extended to 16 bits by the called function (unsigned char is more efficient than signed char - just clr r25). Arguments to functions with variable argument lists (printf etc.) are all passed on stack, and char is extended to int. 
*/
extern unsigned char add1(unsigned char v1);
/*unsigned char add1(unsigned char v1)
{
	// Do nothing, should return v1.
	//addiw r24,1
	%asm volatile(
		"nop\n\t"
		"nop\n\t"
		"addiw %0,%1 : "=d" (r24:r25) : "I" (1)\n\t"
		"nop\n\t"
		);
		
}*/

extern unsigned char ht1(unsigned char v1,unsigned char *ptr);
extern unsigned char ht2(unsigned short v,unsigned char *ptr);
extern unsigned char ht3(unsigned short v,unsigned char *ptr);
extern unsigned char ht3s(unsigned short v,unsigned char *ptr);
extern unsigned char ht4(unsigned short v,unsigned char *ptr);
extern unsigned char ht4l(unsigned long v,unsigned char *ptr);

/******************************************************************************
	function: s16toa
*******************************************************************************	
	Converts a signed 16-bit number into a 7 byte ascii string (6 for the number 
	and sign + 1 null-terminator).
******************************************************************************/
extern void s16toa(signed short v,char *ptr)
{
	if(v&0x8000)
	{
		ptr[0]='-';
		v=-v;
	}
	else
		ptr[0]=' ';
	u16toa(v,ptr+1);	
}
/******************************************************************************
	function: s32toa
*******************************************************************************	
	Converts a signed 32-bit number into a 12 byte ascii string (11 for the number 
	and sign + 1 null-terminator).
******************************************************************************/
extern void s32toa(signed long v,char *ptr)
{
	if(v&0x80000000)
	{
		ptr[0]='-';
		v=-v;
	}
	else
		ptr[0]=' ';
	u32toa(v,ptr+1);	
}

/******************************************************************************
	function: fract16toa
*******************************************************************************	
	Converts a fixed point _Fract number into a 7-bytes ascii string (6 for the number
	+ 1 null-terminator).	
******************************************************************************/
#ifndef __cplusplus
void fract16toa(_Fract a,char *ptr)
{
	// Decide how many digits to print
	// Number is in .15 format so smallest number is 0.000030517578125; worth printing 5 digits after decimal point
	
	_Accum k=a*10000k;	// 4 digits after decimal point
	signed short v = k;
	char c;
	if(v&0x8000)
	{
		c='-';
		v=-v;
	}
	else
		c=' ';
	
	u16toa(v,ptr+1);
	ptr[0]=c;	
	ptr[1]='.';	
}
#endif
/******************************************************************************
	function: floattoa
*******************************************************************************	
	Converts a float number into a 7-bytes ascii string (6 for the number
	+ 1 null-terminator).	
******************************************************************************/
#ifdef __cplusplus
void floattoa(float a,char *ptr)
{
	float k=a*10000.0;	// 4 digits after decimal point
	signed short v = k;
	char c;
	if(v&0x8000)
	{
		c='-';
		v=-v;
	}
	else
		c=' ';
	
	u16toa(v,ptr+1);
	ptr[0]=c;	
	ptr[1]='.';	
}
#endif

void helper_test(void)
{
	unsigned short i;
	unsigned long l;
	char ptr[64];
	
	
	printf_P(PSTR("Helper test\n"));
	
	i = 1234;
	u16toa(i,ptr);
	printf("%u: %s\n",i,ptr);
	
	i = -i;
	s16toa(i,ptr);
	printf("%d: %s\n",i,ptr);
	
	i = 29999;
	u16toa(i,ptr);
	printf("%u: %s\n",i,ptr);
	
	i = -i;
	s16toa(i,ptr);
	printf("%d: %s\n",i,ptr);
	
	l=1234569;
	u32toa(l,ptr);
	printf("%lu: %s\n",l,ptr);
	
	l=-l;
	s32toa(l,ptr);
	printf("%ld: %s\n",l,ptr);
		
	l=2131234567;
	u32toa(l,ptr);
	printf("%lu: %s\n",l,ptr);
	
	l=-l;
	s32toa(l,ptr);
	printf("%ld: %s\n",l,ptr);
	
	while(1);
}


unsigned char hex2chr(unsigned char v)
{
	if(v>9)
		return 'A'+v-10;
	return '0'+v;
}
/******************************************************************************
	peek
*******************************************************************************	
	Returns the next character in the stream or EOF
	
		Return value:
		EOF (-1):	No character in stream
		other:		Character in stream
******************************************************************************/
/*int peek(FILE *file)
{
	// Save the blocking state and set to non-blocking
	unsigned char blocking = fdev_get_udata(file);
	serial_setblocking(file,0);
	
	// Read byte
	int c = fgetc(file);
	ungetc(c,file);
	
	// Restore blocking state
	serial_setblocking(file,blocking);
	
	return c;
}*/

/******************************************************************************
	swalloweol
*******************************************************************************	
	Reads as many characters from stream until one that is not 10 or 13 (newline
	or carriage return) is encountered.
	
******************************************************************************/
/*void swalloweol(FILE *file)
{
	unsigned char c;
	while(1)
	{
		c = peek(file);
		if(c==-1 || (c!=10 && c!=13))
			return;
		fgetc(file);
	}
}*/
/******************************************************************************
	checkdigits
*******************************************************************************	
	Check the n specified characters to make sure they are digits [0;9].
	
	Return value:
		0: 	the n characters are digits
		1:	some characters are not digits
	
******************************************************************************/
unsigned char checkdigits(const char *str,unsigned char n)
{
	for(unsigned char i=0;i<n;i++)
	{
		if(str[i]<'0' || str[i]>'9')
			return 1;
	}
	return 0;
}
/******************************************************************************
	function: ParseComma
*******************************************************************************	
	Identify the n tokens delimited by a comma. 
	
	Takes as parameter as many pointers to a pointer that will contain the 
	address of the first character after each of the commas, or null if a comma is not found.
	
	Warning: Modifies the passed string to have null instead of commas to facilitate further token parsing
	
	Return value:
		0: 				success
		nonzero:	parse error
	
******************************************************************************/
unsigned char ParseComma(const char *str,unsigned char n,...)
{
	va_list args;	
	
	// Initialise the return values to null
	va_start(args,n);
	for(unsigned char i=0;i<n;i++)
	{
		 char **rv = va_arg(args, char **);
		*rv = 0;	
	}
	
	va_start(args,n);
	for(unsigned char i=0;i<n;i++)
	{
		char *p = strchr(str,',');
		*p=0;
		// Not found -> return
		if(!p)
			return 1;
		p++;
		// End of string -> return
		if(!*p)
			return 1;
					
		char **rv = va_arg(args,char **);
		*rv = p;
		
		str=p;
		
	}
	return 0;
}
/******************************************************************************
	function: ParseCommaGetInt
*******************************************************************************	
	Identify the n tokens delimited by a comma. Decodes the tokens assuming they 
	are ints.
	
	For example, the string "M,45,128,-99" is decoded in the tokens 45, 128, -99.
	
	Takes as parameter as many pointers to an int that will contain the integer token.
		
	Warning: Modifies the passed string to have null instead of commas to facilitate further token parsing
	
	Return value:
		0: 				success
		nonzero:	parse error
	
******************************************************************************/
unsigned char ParseCommaGetInt(const char *str,int n,...)
{
	va_list args;	
	int integer;
	
	// Initialise the return values to null
	va_start(args,n);
	for(unsigned char i=0;i<n;i++)
	{
		int *rv = va_arg(args,int *);
		*rv = 0;
	}
	
	va_start(args,n);
	for(unsigned char i=0;i<n;i++)
	{
		char *p = strchr(str,',');
		// Not found -> return
		if(!p)
			return 1;
		*p=0;		
		p++;
		// End of string -> return
		if(!*p)
			return 1;

		if(sscanf(p,"%d",&integer)!=1)
			return 1;

		int *rv = va_arg(args,int *);
		*rv = integer;
		
		str=p;
		
	}
	return 0;
}


/******************************************************************************
	function: TimeAddSeconds
*******************************************************************************	
	Adds ds seconds to a time specified in hh:mm:ss. If the time wraps around the 
	return value indicates how many days to add to the current date.
		
	hour: 00-23
	min: 00-59
	sec: 00-59
	
	ds: seconds to add to current time
	
	Return value:
		0: 				success, no wrap around
		nonzero:	time wrapped around, return value indicates how many days?
	
******************************************************************************/
unsigned char TimeAddSeconds(unsigned short hour, unsigned short min, unsigned short sec, unsigned short ds,unsigned short *ohour, unsigned short *omin, unsigned short *osec)
{
	unsigned char day=0;
	// No check on input values
	sec+=ds;
	while(sec>=60)
	{
		sec-=60;
		min++;
	}
	while(min>=60)
	{
		min-=60;
		hour++;
	}
	while(hour>=24)
	{
		hour-=24;
		day++;
	}
	*ohour=hour;
	*omin=min;
	*osec=sec;
	return day;
	
}


/******************************************************************************
	Function: format3s16
*******************************************************************************
	Formats 3 signed short numbers into an ascii string.
	Numbers are space separated, including a space after the last number, 
	however the string is not null terminated.
	
	The function returns a pointer to the first byte after the end of the string.
	
	Parameters:
		strptr		-		pointer to the buffer that will receive the string
		x			-		First number to format
		y			-		Second number to format
		z			-		Third number to format
	
******************************************************************************/
char *format3s16(char *strptr,signed short x,signed short y,signed short z)
{
	s16toa(x,strptr);
	strptr+=6;
	*strptr=' ';
	strptr++;
	s16toa(y,strptr);
	strptr+=6;
	*strptr=' ';
	strptr++;
	s16toa(z,strptr);
	strptr+=6;
	*strptr=' ';
	strptr++;
	return strptr;
}
/******************************************************************************
	Function: format1u32
*******************************************************************************
	Formats 1 unsigned 32-bit number into an ascii string.
	The string includes a space after the number, however the string is not null terminated.
	
	The function returns a pointer to the first byte after the end of the string.
	
	Parameters:
		strptr		-		pointer to the buffer that will receive the string
		a			-		Number to format
	
******************************************************************************/
char *format1u32(char *strptr,unsigned long a)
{
	u32toa(a,strptr);
	strptr+=10;
	*strptr=' ';
	strptr++;
	return strptr;
}
/******************************************************************************
	Function: format1u16
*******************************************************************************
	Formats 1 unsigned 16-bit number into an ascii string.
	The string includes a space after the number, however the string is not null terminated.
	
	The function returns a pointer to the first byte after the end of the string.
	
	Parameters:
		strptr		-		pointer to the buffer that will receive the string
		a			-		Number to format
	
******************************************************************************/
char *format1u16(char *strptr,unsigned short a)
{
	u16toa(a,strptr);
	strptr+=5;
	*strptr=' ';
	strptr++;
	return strptr;
}
/******************************************************************************
	Function: format4fract16
*******************************************************************************
	Formats 3 _Fract numbers into an ascii string.
	Numbers are space separated, including a space after the last number, 
	however the string is not null terminated.
	
	The function returns a pointer to the first byte after the end of the string.
	
	Parameters:
		strptr		-		pointer to the buffer that will receive the string
		q0			-		First number to format
		q1			-		Second number to format
		q2			-		Third number to format
		q3			-		Fourth number to format
	
******************************************************************************/
#ifndef __cplusplus
char *format4fract16(char *strptr,_Fract q0,_Fract q1,_Fract q2,_Fract q3)
{
	f16toa(q0,strptr);
	strptr+=6;
	*strptr=' ';
	strptr++;
	f16toa(q1,strptr);
	strptr+=6;
	*strptr=' ';
	strptr++;
	f16toa(q2,strptr);
	strptr+=6;
	*strptr=' ';
	strptr++;
	f16toa(q3,strptr);
	strptr+=6;
	*strptr=' ';
	strptr++;
	return strptr;
}
#endif
/******************************************************************************
	Function: format4fract16
*******************************************************************************
	Formats 3 _Fract numbers into an ascii string.
	Numbers are space separated, including a space after the last number, 
	however the string is not null terminated.
	
	The function returns a pointer to the first byte after the end of the string.
	
	Parameters:
		strptr		-		pointer to the buffer that will receive the string
		q0			-		First number to format
		q1			-		Second number to format
		q2			-		Third number to format
		q3			-		Fourth number to format
	
******************************************************************************/
#ifdef __cplusplus
char *format4float(char *strptr,float q0,float q1,float q2,float q3)
{
	floattoa(q0,strptr);
	strptr+=6;
	*strptr=' ';
	strptr++;
	floattoa(q1,strptr);
	strptr+=6;
	*strptr=' ';
	strptr++;
	floattoa(q2,strptr);
	strptr+=6;
	*strptr=' ';
	strptr++;
	floattoa(q3,strptr);
	strptr+=6;
	*strptr=' ';
	strptr++;
	return strptr;
}
#endif



