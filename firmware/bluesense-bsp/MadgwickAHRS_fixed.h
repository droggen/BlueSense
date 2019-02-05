//=====================================================================================================
// MadgwickAHRS.h
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================

#ifndef MadgwickAHRS_FIXED_H
#define MadgwickAHRS_FIXED_H


#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdfix.h>

#define FIXEDPOINTTYPE _Accum
//#define FIXEDPOINTTYPE float
//#define FIXEDPOINTTYPE long _Accum


#if ENABLEQUATERNION==1
#if FIXEDPOINTQUATERNION==1




//#define FIXEDPOINTTYPE short _Accum

//#define FIXEDPOINTTYPE _Accum
//#define FIXEDPOINTTYPE long _Accum

#define FIXEDPOINTTYPE_FRACT _Fract
//#define FIXEDPOINTTYPE_FRACT _Accum
//#define FIXEDPOINTTYPE_FRACT float
//#define FIXEDPOINTTYPE_FRACT long _Accum

//----------------------------------------------------------------------------------------------------
// Variable declaration

//extern FIXEDPOINTTYPE beta;			// algorithm gain
//extern FIXEDPOINTTYPE q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame
extern FIXEDPOINTTYPE_FRACT q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame

//---------------------------------------------------------------------------------------------------
// Function declarations




//void MadgwickAHRSupdate(FIXEDPOINTTYPE gx, FIXEDPOINTTYPE gy, FIXEDPOINTTYPE gz, FIXEDPOINTTYPE ax, FIXEDPOINTTYPE ay, FIXEDPOINTTYPE az, FIXEDPOINTTYPE mx, FIXEDPOINTTYPE my, FIXEDPOINTTYPE mz);
void MadgwickAHRSupdate_fixed(FIXEDPOINTTYPE gx, FIXEDPOINTTYPE gy, FIXEDPOINTTYPE gz, FIXEDPOINTTYPE ax, FIXEDPOINTTYPE ay, FIXEDPOINTTYPE az, signed short mx, signed short my, signed short mz);
//void MadgwickAHRSupdateIMU(FIXEDPOINTTYPE gx, FIXEDPOINTTYPE gy, FIXEDPOINTTYPE gz, FIXEDPOINTTYPE ax, FIXEDPOINTTYPE ay, FIXEDPOINTTYPE az);

#endif

#endif
#endif
//=====================================================================================================
// End of file
//=====================================================================================================
