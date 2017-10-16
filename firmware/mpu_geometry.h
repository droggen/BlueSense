#ifndef __MPU_GEOMETRY_H
#define __MPU_GEOMETRY_H

#include "mpu.h"

typedef struct {
	float yaw,pitch,roll;		// Aerospace
	float alpha,x,y,z;			// Quaternion debug
} MPUMOTIONGEOMETRY;


float rad_to_deg(float rad);
void mpu_quaternion_to_aerospace(float &yaw,float &pitch, float&roll);
void mpu_compute_geometry(MPUMOTIONDATA &mpumotiondata,MPUMOTIONGEOMETRY &mpumotiongeometry);

#endif
