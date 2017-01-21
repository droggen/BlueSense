//=====================================================================================================
// MadgwickAHRS.c
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 19/02/2012	SOH Madgwick	Magnetometer measurement is normalised
// 2016			D Roggen		Fixed point implementation, invsqrt optimisation
//=====================================================================================================

//---------------------------------------------------------------------------------------------------
// Header files

#if ENABLEQUATERNION==1
#if FIXEDPOINTQUATERNION==1


#include "MadgwickAHRS2.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdfix.h>

#include "mathfix.h"

//---------------------------------------------------------------------------------------------------
// Definitions

//#define sampleFreq	512.0k		// sample frequency in Hz
#define sampleFreq	100.0k		// sample frequency in Hz
//#define sampleFreq	200.0k		// sample frequency in Hz

// 0.1 too slow
//#define betaDef		0.1k		// 2 * proportional gain
//#define betaDef		1.1k		// 2 * proportional gain		// dan - too much
// betadef in 0-1
#define betaDef		0.4K		// 2 * proportional gain		// dan 



//---------------------------------------------------------------------------------------------------
// Variable definitions
//FIXEDPOINTTYPE invSampleFreq = 1.0k/sampleFreq;
_Fract invSampleFreq = 1.0k/sampleFreq;
//FIXEDPOINTTYPE beta = betaDef;										// 2 * proportional gain (Kp)
unsigned _Fract beta = betaDef;
//FIXEDPOINTTYPE q0 = 1.0k, q1 = 0.0k, q2 = 0.0k, q3 = 0.0k;	// quaternion of sensor frame relative to auxiliary frame
_Fract q0 = 0.999999k, q1 = 0.0k, q2 = 0.0k, q3 = 0.0k;	// quaternion of sensor frame relative to auxiliary frame



//---------------------------------------------------------------------------------------------------
// AHRS algorithm update

/**********************************
	qnqn FIXEDPOINTTYPE
	q? FIXEDPOINTTYPE
		124684	2720-2900
	
	q? _Fract
		122522	2400-2600
		
	q? q?q? _Fract
		122518
		
	ax _Fract
		122604	2450
	
	mx _Fract
		122540	2400
	
	
	TODO:
	1) Normalisation of magnetic field: the numbers _mx are [-64;64], when squaring, adding and converting to _Accum there is possibility of overflow (64^2=4096, 4096*32768 = 134217728)?
	2) Multiplication by 2, 4, 8: check bit shift instead: Done, multiplications replaced by shift where code saving occurs.
	3) _2bx = fsqrt(hx * hx + hy * hy);		currently float; maybe integer square root?
	
********************************/
/*
	SQRT: 
	Normalisation of _2bx: S.15->S.15
		-> Solvable with https://github.com/chmike/fpsqrt/blob/master/fpsqrt.c
	
	Normalisation of magnetic field: numbers are -64->+64 -> 12288
		-> Integer to fixed
		
	Normalisation of s, a, q: 16.15 -> 16.15
		-> fixed to fixed .15 to .15
*/


void MadgwickAHRSupdate(FIXEDPOINTTYPE gx, FIXEDPOINTTYPE gy, FIXEDPOINTTYPE gz, FIXEDPOINTTYPE _ax, FIXEDPOINTTYPE _ay, FIXEDPOINTTYPE _az, signed short _mx, signed short _my, signed short _mz) {
	FIXEDPOINTTYPE recipNorm;
	FIXEDPOINTTYPE s0, s1, s2, s3;
	FIXEDPOINTTYPE qDot1, qDot2, qDot3, qDot4;
	FIXEDPOINTTYPE hx, hy;
	FIXEDPOINTTYPE _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3;
	FIXEDPOINTTYPE _4q0, _4q1, _4q2 ,_8q1, _8q2;
	// The following numbers are fractional (<1)
	_Fract q0q0,q1q1,q2q2,q3q3,q0q1,q0q2,q0q3,q1q2,q1q3,q2q3;
	_Fract ax,ay,az;
	_Fract mx,my,mz;
	
	// Rate of change of quaternion from gyroscope
	// q e [-1;1] g e [-inf;inf] qDot e [-inf;inf]
	// qDot must be _Accum
	/*qDot1 = 0.5k * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5k * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5k * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5k * (q0 * gz + q1 * gy - q2 * gx);*/
	qDot1 = (-q1 * gx - q2 * gy - q3 * gz)>>1;
	qDot2 = (q0 * gx + q2 * gz - q3 * gy)>>1;
	qDot3 = (q0 * gy - q1 * gz + q3 * gx)>>1;
	qDot4 = (q0 * gz + q1 * gy - q2 * gx)>>1;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((_ax == 0.0k) && (_ay == 0.0k) && (_az == 0.0k))) {

		// Normalise accelerometer measurement
		/*recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;  */
		recipNorm = invSqrt(_ax * _ax + _ay * _ay + _az * _az);
		ax = _ax*recipNorm;
		ay = _ay*recipNorm;
		az = _az*recipNorm;  

		// Auxiliary variables to avoid repeated arithmetic
		/*_2q0 = 2.0k * q0;
		_2q1 = 2.0k * q1;
		_2q2 = 2.0k * q2;
		_2q3 = 2.0k * q3;*/
		// Shift by 1 saves 48 bytes
		_2q0 = ((_Accum)q0)<<1;
		_2q1 = ((_Accum)q1)<<1;
		_2q2 = ((_Accum)q2)<<1;
		_2q3 = ((_Accum)q3)<<1;
		
		// q0q0,q1q1,q2q2,q3q3 e [-1;1]
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;


		
		if(!((_mx == 0.0k) && (_my == 0.0k) && (_mz == 0.0k)))
		{
			// mag is non null
		
			// Normalise magnetometer measurement
			/*recipNorm = invSqrt(mx * mx + my * my + mz * mz);
			mx *= recipNorm;
			my *= recipNorm;
			mz *= recipNorm;*/
			recipNorm = invSqrt(_mx * _mx + _my * _my + _mz * _mz);
			mx = _mx*recipNorm;
			my = _my*recipNorm;
			mz = _mz*recipNorm;

			// Auxiliary variables to avoid repeated arithmetic
			
			/*_2q0mx = 2.0k * q0 * mx;
			_2q0my = 2.0k * q0 * my;
			_2q0mz = 2.0k * q0 * mz;
			_2q1mx = 2.0k * q1 * mx;
			_2q0q2 = 2.0k * q0 * q2;
			_2q2q3 = 2.0k * q2 * q3;*/
			// Shift by 1 saves 156 bytes
			_2q0mx = (q0 * mx)<<1;
			_2q0my = (q0 * my)<<1;
			_2q0mz = (q0 * mz)<<1;
			_2q1mx = (q1 * mx)<<1;
			_2q0q2 = (q0 * q2)<<1;
			_2q2q3 = (q2 * q3)<<1;
			q0q1 = q0 * q1;
			q0q2 = q0 * q2;
			q0q3 = q0 * q3;
			q1q2 = q1 * q2;
			q1q3 = q1 * q3;
			q2q3 = q2 * q3;

			// Reference direction of Earth's magnetic field
			hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
			hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
			_2bx = fsqrt(hx * hx + hy * hy);		// ====Optimisation?====
			//_2bx = sqrts15(hx * hx + hy * hy);
			_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
			
			/*
			_4bx = 2.0k * _2bx;
			_4bz = 2.0k * _2bz;
			*/
			// Shift by 1 saves 22 bytes
			_4bx = _2bx<<1;
			_4bz = _2bz<<1;	

			// Gradient decent algorithm corrective step
			s0 = -_2q2 * (2.0k * q1q3 - _2q0q2 - ax) + _2q1 * (2.0k * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5k - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5k - q1q1 - q2q2) - mz);
			s1 = _2q3 * (2.0k * q1q3 - _2q0q2 - ax) + _2q0 * (2.0k * q0q1 + _2q2q3 - ay) - 4.0k * q1 * (1 - 2.0k * q1q1 - 2.0k * q2q2 - az) + _2bz * q3 * (_2bx * (0.5k - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5k - q1q1 - q2q2) - mz);
			s2 = -_2q0 * (2.0k * q1q3 - _2q0q2 - ax) + _2q3 * (2.0k * q0q1 + _2q2q3 - ay) - 4.0k * q2 * (1 - 2.0k * q1q1 - 2.0k * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5k - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5k - q1q1 - q2q2) - mz);
			s3 = _2q1 * (2.0k * q1q3 - _2q0q2 - ax) + _2q2 * (2.0k * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5k - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5k - q1q1 - q2q2) - mz);
		}
		else
		{
			// mag is null
			// Auxiliary variables to avoid repeated arithmetic
			
			_4q0 = 4.0k * q0;
			_4q1 = 4.0k * q1;
			_4q2 = 4.0k * q2;
			_8q1 = 8.0k * q1;
			_8q2 = 8.0k * q2;
			
			//_4q0 = ((_Accum)q0)<<2;
			/*_4q0 = ((_Accum)q0)<<2;
			_4q1 = ((_Accum)q1)<<2;
			_4q2 = ((_Accum)q2)<<2;
			_8q1 = ((_Accum)q1)<<3;
			_8q2 = ((_Accum)q2)<<3;*/

			// Gradient decent algorithm corrective step
			s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
			s1 = _4q1 * q3q3 - _2q3 * ax + 4.0k * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
			s2 = 4.0k * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
			s3 = 4.0k * q1q1 * q3 - _2q1 * ax + 4.0k * q2q2 * q3 - _2q2 * ay;			
		}
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		
		
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	/*
	q0 += qDot1 * (1.0f / sampleFreq);
	q1 += qDot2 * (1.0f / sampleFreq);
	q2 += qDot3 * (1.0f / sampleFreq);
	q3 += qDot4 * (1.0f / sampleFreq);*/
	
	FIXEDPOINTTYPE nq0,nq1,nq2,nq3;
	
	/*q0 += qDot1 * invSampleFreq;
	q1 += qDot2 * invSampleFreq;
	q2 += qDot3 * invSampleFreq;
	q3 += qDot4 * invSampleFreq;*/
	nq0 = q0 + qDot1 * invSampleFreq;
	nq1 = q1 + qDot2 * invSampleFreq;
	nq2 = q2 + qDot3 * invSampleFreq;
	nq3 = q3 + qDot4 * invSampleFreq;

	// Normalise quaternion
	//recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	recipNorm = invSqrt(nq0 * nq0 + nq1 * nq1 + nq2 * nq2 + nq3 * nq3);
	/*q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;*/
	nq0 *= recipNorm;
	nq1 *= recipNorm;
	nq2 *= recipNorm;
	nq3 *= recipNorm;
	// nq?e[-1;1]
	q0=nq0;
	q1=nq1;
	q2=nq2;
	q3=nq3;
	
}


//====================================================================================================
// END OF CODE
//====================================================================================================

#endif
#endif