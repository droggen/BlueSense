#ifndef __SYSTEM_H
#define __SYSTEM_H

#include "power.h"

#define BATTERY_THRESHOLD_CONNECTED 4500

#if BOOTLOADER==0
//extern volatile unsigned short system_battery_voltage;
//extern volatile unsigned long int system_battery_updatetime;
//extern volatile unsigned long int system_battery_updatetime;
extern unsigned char system_enable_lcd;
#endif

#define BATTERY_VERYVERYLOW 3350
#define BATTERY_VERYLOW 3600
#define BATTERY_LOW 3700


void system_delay_ms(unsigned short t);
void system_led_set(unsigned char led);
void system_led_on(unsigned char ledn);
void system_led_off(unsigned char ledn);
void system_led_toggle(unsigned char led);
void system_blink(unsigned char n,unsigned char delay,unsigned char init);
void system_blink_led(unsigned char n,unsigned char timeon,unsigned char timeoff,unsigned char led);
void system_status_ok(unsigned char status);
void system_status_ok2(unsigned char status);
void system_status_error(unsigned char error,unsigned char forever);
void system_led_test(void);
unsigned char system_lifesign(unsigned char unused);
unsigned char system_lifesign2(unsigned char sec);
unsigned char system_batterystat(unsigned char unused);

unsigned char system_isbtconnected(void);
unsigned char system_isusbconnected(void);

#if BOOTLOADER==0
void system_callback_battery_sample_init(void);
unsigned char system_callback_battery_sample(unsigned char);
unsigned short system_getbattery(void);
unsigned long system_getbatterytime(void);
void system_off(void);
void system_power_low(void);
void system_power_normal(void);
void system_adcpu_off(void);
void system_adcpu_on(void);
unsigned char *system_getdevicename(void);
#endif

unsigned char system_getrtcint(void);
void system_settimefromrtc(void);
void system_storepoweroffdata(void);
void system_storepoweroffdata2(void);
void system_loadpoweroffdata2(_POWERUSE_OFF &pu);

#endif
