#ifndef IMU_H
#define IMU_H

#include "stdio.h"	
#include <stdint.h>
typedef struct {
	short a[3];
	short T;
}SAcc;
typedef struct {
	short w[3];
	short T;
}SGyro;
typedef struct {
	short Angle[3];
	short T;
}SAngle;
void CopeData(unsigned char ucData);
void USART6_Get_IMU(void);
void IMUinit(void);
void USART6_IRQHandler(void);
void GetIMUData(float* out_roll, float* out_pitch, float* out_yaw, 
                float* out_wx, float* out_wy, float* out_wz);

#endif
