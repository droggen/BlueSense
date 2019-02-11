#ifndef __STK500_H
#define __STK500_H

#include "stk500-command.h"

extern FILE *file_bl;
extern FILE *file_dbg;

typedef uint32_t address_t;

unsigned long int timer_ms_get(void);

void sendchar(char c);
int getchar_timeout(void);

unsigned char sendmessage(const unsigned char *buffer, unsigned short size, unsigned char seq);
void msprintf(char *buffer,char *format,...);
void mfprintf(FILE *f,const char *format,...);
FILE *detectbl(void);
extern int bootmain(void);
extern void stk500(void);

#endif
