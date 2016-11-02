#ifndef __MODE_DEMO_H
#define __MODE_DEMO_H

#if ENABLEGFXDEMO==1

void mode_demo(void);
char demo_clock(unsigned long int dt);
char demo_acc(unsigned long int dt);
char demo_gyr(unsigned long int dt);



#endif

#endif
