#ifndef __MODE_SD_H
#define __MODE_SD_H

#include "command.h"

extern const char help_sdinit[];
extern const char help_sdbench[];

unsigned char CommandParserSD(unsigned char *buffer,unsigned char size);
unsigned char CommandParserSDInit(unsigned char *buffer,unsigned char size);
unsigned char CommandParserSDWrite(unsigned char *buffer,unsigned char size);
unsigned char CommandParserSDRead(unsigned char *buffer,unsigned char size);
unsigned char CommandParserSDStream(unsigned char *buffer,unsigned char size);
unsigned char CommandParserSDVolume(unsigned char *buffer,unsigned char size);
unsigned char CommandParserSDFormat(unsigned char *buffer,unsigned char size);
unsigned char CommandParserSDLogTest(unsigned char *buffer,unsigned char size);
unsigned char CommandParserSDBench(unsigned char *buffer,unsigned char size);

void mode_sd(void);


#endif
