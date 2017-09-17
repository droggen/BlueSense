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

#if ENABLEQUATERNION==1
#if FIXEDPOINTQUATERNION==0

#ifndef MadgwickAHRS_FLOAT_H
#define MadgwickAHRS_FLOAT_H



//----------------------------------------------------------------------------------------------------
// Variable declaration

extern volatile float beta;				// algorithm gain
extern volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame

//---------------------------------------------------------------------------------------------------
// Function declarations
void testf(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) ;
void MadgwickAHRSinit(void);
void MadgwickAHRSupdate_float(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);


#endif
#endif
#endif
//=====================================================================================================
// End of file
//=====================================================================================================
