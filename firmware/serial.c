/*
   Copyright (C) 2009-2016:
         Daniel Roggen, droggen@gmail.com

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/
/*
	Serial functions
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "serial.h"

// Statically allocate streams
FILE serial_file[3];
SERIALPARAM serial_param[3];
unsigned char serial_numopen=0;

/*
	Returns the size of the uart buffers.
*/
/*void uart_getbuffersize(unsigned short *rx0,unsigned short *tx0,unsigned short *rx1,unsigned short *tx1)
{
	*rx0 = SerialData0.rx.size;
	*tx0 = SerialData0.tx.size;
	*rx1 = SerialData1.rx.size;
	*tx1 = SerialData1.tx.size;
}*/

/******************************************************************************
	serial_open
*******************************************************************************	
	Opens a serial communication peripheral.
	
	periph:		0:		UART0
						1:		UART1
						10:		I2C FTDI DBG interface
	interrupt:
						1:		indicates that an interrupt-driven function is desired (if available)
						0:		indicates that a non-interrupt-driven function is desired (if available)
			
	Return value:	
		0:				success
		nonzero:	error
******************************************************************************/

FILE *serial_open(unsigned char periph,unsigned char interrupt)
{

	FILE *f;
	SERIALPARAM *p;
	
	//p = (SERIALPARAM*)malloc(sizeof(SERIALPARAM));
	//if(p==0)
		//return 0;
	p = &serial_param[serial_numopen];
	f = &serial_file[serial_numopen];
		
	p->blocking=1;
	switch(periph)
	{
		#if ENABLE_SERIAL0==1
		case 0:
			// UART0
			if(interrupt)
				//f = fdevopen(uart0_fputchar_int,uart0_fgetchar_int);
				fdev_setup_stream(f,uart0_fputchar_int, uart0_fgetchar_int,_FDEV_SETUP_RW);
			else
				//f = fdevopen(uart0_fputchar,uart0_fgetchar);
				fdev_setup_stream(f,uart0_fputchar, uart0_fgetchar,_FDEV_SETUP_RW);
			p->putbuf = uart0_fputbuf_int;
			p->txbuf = uart0_get_txbuf();
			p->rxbuf = uart0_get_rxbuf();
			break;
		#endif
		#if ENABLE_SERIAL1==1
		case 1:
			// UART1
			#if BOOTLOADER==1
			//f = fdevopen(uart1_fputchar_int,uart1_fgetchar_int);
			fdev_setup_stream(f,uart1_fputchar_int, uart1_fgetchar_int,_FDEV_SETUP_RW);
			#else
			if(interrupt)
				//f = fdevopen(uart1_fputchar_int,uart1_fgetchar_int);
				fdev_setup_stream(f,uart1_fputchar_int, uart1_fgetchar_int,_FDEV_SETUP_RW);
			else
				//f = fdevopen(uart1_fputchar,uart1_fgetchar);
				fdev_setup_stream(f,uart1_fputchar, uart1_fgetchar,_FDEV_SETUP_RW);
			p->putbuf = uart1_fputbuf_int;
			#endif			
			p->txbuf = uart1_get_txbuf();
			p->rxbuf = uart1_get_rxbuf();
			break;
		#endif
		case 10:
		default:
			// DBG
			//f = fdevopen(dbg_fputchar,dbg_fgetchar);
			fdev_setup_stream(f,dbg_fputchar, dbg_fgetchar,_FDEV_SETUP_RW);
			p->putbuf = dbg_putbuf;
			p->txbuf = dbg_get_txbuf();
			p->rxbuf = dbg_get_rxbuf();
			break;
	}	
	
	fdev_set_udata(f,(void*)p);
	if(serial_numopen==0)
	{
		// Assign standard io
		stdin = f;
		stdout = f;
		stderr = f;
	}
	serial_numopen++;
	
	return f;	
}


/*
	Sets whether the read functions must be blocking or non blocking.
	In non-blocking mode, when no characters are available, read functions return EOF.
*/
void serial_setblocking(FILE *file,unsigned char blocking)
{
	SERIALPARAM *p = (SERIALPARAM*)fdev_get_udata(file);
	p->blocking = blocking;
}
unsigned char serial_isblocking(FILE *file)
{
	SERIALPARAM *p = (SERIALPARAM*)fdev_get_udata(file);
	return p->blocking;
}


unsigned char fputbuf(FILE *stream,char *data,unsigned char n)
{
	if(!stream)
		return 0;
	SERIALPARAM *p = (SERIALPARAM*)fdev_get_udata(stream);
	return p->putbuf(data,n);
}

unsigned short fgettxbuflevel(FILE *stream)
{
	SERIALPARAM *p = (SERIALPARAM*)fdev_get_udata(stream);
	return buffer_level(p->txbuf);
}
unsigned short fgetrxbuflevel(FILE *stream)
{
	SERIALPARAM *p = (SERIALPARAM*)fdev_get_udata(stream);
	return buffer_level(p->rxbuf);
}
unsigned short fgettxbuffree(FILE *stream)
{
	SERIALPARAM *p = (SERIALPARAM*)fdev_get_udata(stream);
	return buffer_freespace(p->txbuf);
}
unsigned short fgetrxbuffree(FILE *stream)
{
	SERIALPARAM *p = (SERIALPARAM*)fdev_get_udata(stream);
	return buffer_freespace(p->rxbuf);
}

#if BOOTLOADER==0
/*
	Wrap functions check whether the file is null and do nothing in this case
*/
/*
	fputc used by: 
	printf (parameters), printf_P (no parameters), printf_P (parameters), fprintf (parameters), 
	fprintf_P (no parameters), fprintf_P (parameters)
	
	fputc is not overridden as this would have serious performance impact; instead the string
	print function are overridden.
	Note: fputc is not safe to be called with a null file.
	
*/
/*int __real_fputc( int __c, FILE * __stream);
int __wrap_fputc( int __c, FILE * __stream)
{
	dbg_fputchar('*',0);
	//dbg_fputchar(__c,0);
	//dbg_fputchar('"',0);
	return 0;
}*/
/*
	vfprintf used by:
	printf (parameters), printf_P (no parameters), printf_P (parameters), 
	fprintf (parameters), fprintf_P (no parameters), fprintf_P (parameters)	
*/

//#define WRAPPRINT

#ifdef WRAPPRINT

int __real_vfprintf(FILE *__stream,const char *__fmt,va_list __ap);
int __wrap_vfprintf(FILE *__stream,const char *__fmt,va_list __ap)
{
	if(!__stream)
		return 0;
	return __real_vfprintf(__stream,__fmt,__ap);
}
/*
	fputs_P used by: 
	fputs_P	
*/
int __real_fputs_P (const char *__str, FILE *__stream);
int __wrap_fputs_P (const char *__str, FILE *__stream)
{
//	dbg_fputchar('^',0);
//	return 0;
	if(!__stream)
		return 0;
	return __real_fputs_P(__str,__stream);
}
/*
	puts used by: 
	printf (no parameters)	
*/
int __real_puts(const char *__str);
int __wrap_puts(const char *__str)
{
//	dbg_fputchar('!',0);
//	return 0;
	if(!stdout)
		return 0;
	return __real_puts(__str);
}
/*
	fwrite used by: 
	fprintf (no parameters), fputs (no parameters)
*/
size_t __real_fwrite(const void *__ptr, size_t __size, size_t __nmemb, FILE *__stream);
size_t __wrap_fwrite(const void *__ptr, size_t __size, size_t __nmemb, FILE *__stream)
{
	//dbg_fputchar('+',0);
	//return 0;
	if(!__stream)
		return 0;
	return __real_fwrite(__ptr,__size,__nmemb,__stream);
}

#endif

#endif
/*
	Flushes f by reading until empty
*/
/*void flush(FILE *file)
{
	int c;
	// Non-blocking mode
	uart_setblocking(file,0);
	while((c=fgetc(file))!=EOF);
	// Return to blocking mode
	uart_setblocking(file,1);
}*/


