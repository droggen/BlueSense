#ifndef __MODE_IDLE
#define __MODE_IDLE

#include "command.h"

void mode_idle(void);
unsigned char CommandParserClock(unsigned char *buffer,unsigned char size);
unsigned char CommandParserDemo(unsigned char *buffer,unsigned char size);

#endif