#ifndef __MADGWICKAHRS_H
#define __MADGWICKAHRS_H


#if ENABLEQUATERNION==1
#if FIXEDPOINTQUATERNION==1
#include "MadgwickAHRS_fixed.h"
#else
#include "MadgwickAHRS_float.h"
#endif
#endif

#endif