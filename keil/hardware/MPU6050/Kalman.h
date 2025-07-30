#ifndef __KALMAN_H
#define __KALMAN_H

#include "ti_msp_dl_config.h"
#include "math.h"
#include "mpu6050.h"

void Kalman_Init(void);
void Kalman_GetMPU6050_Offset(float *Offset_ax, float *Offset_ay, float *Offset_az, 
                                float *Offset_gx, float *Offset_gy, float *Offset_gz);
void Kalman_GetMPU6050_Data(float *AccX, float *AccY, float *AccZ,
                            float *GyroX, float *GyroY, float *GyroZ);
void Kalman_GetEuler_TempAngle(float *Roll_A, float *Pitch_A,
                                float *Roll_G, float *Pitch_G, float *Yaw_G);
float Kalman_Filter_Roll(float Angle_a, float Angle_g);
float Klaman_Filter_Pitct(float Angle_a, float Angle_g);
void Kalman_GetEuler_Angle(float *Roll, float *Pitch, float *Yaw);

#endif