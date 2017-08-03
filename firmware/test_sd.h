#ifndef __TEST_SD_H
#define __TEST_SD_H

#include "cpu.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/eeprom.h>
#include <util/atomic.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "main.h"
#include "adc.h"
#include "serial.h"
#include "i2c.h"
#include "ds3232.h"
#include "rn41.h"
#include "mpu.h"
#include "mpu_test.h"
#include "pkt.h"
#include "wait.h"
#include "init.h"
#include "lcd.h"
#include "fb.h"
#include "uiconfig.h"
#include "helper.h"
#include "i2c_internal.h"
#include "system.h"
#include "pkt.h"

#include "mode_sd.h"
#include "commandset.h"
#include "mode_global.h"
#include "spi.h"
#include "sd.h"
#include "ufat.h"



void test_sd_benchmarkwriteblock(unsigned long capacity_sectors);
void test_sd_benchmarkwriteblockmulti(unsigned long capacity_sectors);
void test_sdmulti(void);
void sd_bench_streamcache_write(unsigned long startsect,unsigned long size);
void sd_bench_stream_write(unsigned long startsect,unsigned long size,unsigned long preerase);
void sd_bench_write(unsigned long startsect,unsigned long size);
void sd_bench_write2(unsigned long startsect,unsigned long size);
void sd_bench_stream_write2(unsigned long startsect,unsigned long size,unsigned long preerase);
void sd_bench_streamcache_write2(unsigned long startsect,unsigned long size,unsigned long preerase);

#endif

