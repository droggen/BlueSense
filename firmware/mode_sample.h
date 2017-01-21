#ifndef __MODE_SAMPLE_H
#define __MODE_SAMPLE_H


extern FILE *mode_sample_file_log;

extern const char help_samplelog[];

unsigned char CommandParserSampleLog(char *buffer,unsigned char size);
void mode_sample_logend(void);


#endif