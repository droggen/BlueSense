#ifndef __CPU_H
#define __CPU_H

#if HWVER==1
#define F_CPU 7372800UL
#endif
#if (HWVER==4) || (HWVER==5) || (HWVER==6) || (HWVER==7) || (HWVER==9)
#define F_CPU 11059200UL
#endif



#endif