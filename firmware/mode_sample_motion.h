#ifndef __MODE_MOTIONSTREAM_H
#define __MODE_MOTIONSTREAM_H

#include "command.h"

//#define MSM_LOGBAT

extern const char help_streamlog[] PROGMEM;

unsigned char stream_sample(FILE *f);

// Structure to hold the volatile parameters of this mode
typedef struct {
	unsigned char mode;
	int logfile;
	unsigned long duration;
} MODE_SAMPLE_MOTION_PARAM;



void stream(void);

void mode_sample_motion_setparam(unsigned char mode, int logfile, int duration);
unsigned char CommandParserSampleLogMPU(char *buffer,unsigned char size);
unsigned char CommandParserSampleStatus(char *buffer,unsigned char size);
unsigned char CommandParserBatBench(char *buffer,unsigned char size);
void stream_status(FILE *f,unsigned char bin);
unsigned char CommandParserMotion(char *buffer,unsigned char size);
void mode_motionstream(void);


#endif
