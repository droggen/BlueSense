#ifndef __MODE_BENCH_H
#define __MODE_BENCH_H

#include "command.h"


extern unsigned char mode_bench_io;

void mode_bench(void);
unsigned char CommandParserBench(unsigned char *buffer,unsigned char size);

#endif

