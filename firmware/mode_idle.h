#ifndef __MODE_IDLE
#define __MODE_IDLE

#include "command.h"

void mode_idle(void);
unsigned char CommandParserClock(char *buffer,unsigned char size);
unsigned char CommandParserDemo(char *buffer,unsigned char size);

#endif