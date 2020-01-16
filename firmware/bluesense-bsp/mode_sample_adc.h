#ifndef __MODE_ADC_H
#define __MODE_ADC_H

#include "command.h"


extern unsigned long mode_adc_period;
extern unsigned char mode_adc_mask;
extern unsigned char mode_adc_fast;

void mode_adc(void);

unsigned char CommandParserADCPullup(char *buffer,unsigned char size);
unsigned char CommandParserADC(char *buffer,unsigned char size);

#endif
