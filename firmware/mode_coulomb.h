#ifndef __MODE_COULOMB_H
#define __MODE_COULOMB_H

#if ENABLEMODECOULOMB==1

#include "command.h"

extern const char help_cq[];
extern const char help_cc[];
extern const char help_cr[];


void mode_coulomb(void);

unsigned char CommandParserCoulomb(unsigned char *buffer,unsigned char size);
unsigned char CommandParserCoulombCharge(unsigned char *buffer,unsigned char size);
//unsigned char CommandParserCoulombControl(unsigned char *buffer,unsigned char size);
unsigned char CommandParserCoulombPrescaler(unsigned char *buffer,unsigned char size);
unsigned char CommandParserCoulombADC(unsigned char *buffer,unsigned char size);
unsigned char CommandParserCoulombReset(unsigned char *buffer,unsigned char size);

#endif

#endif
