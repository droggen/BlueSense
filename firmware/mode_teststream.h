#ifndef __MODE_TESTSTREAM_H
#define __MODE_TESTSTREAM_H

#include "command.h"


extern unsigned long mode_ts_period;


void mode_teststream(void);

unsigned char CommandParserTeststream(unsigned char *buffer,unsigned char size);
unsigned char CommandParserTSLog(unsigned char *buffer,unsigned char size);

#endif
