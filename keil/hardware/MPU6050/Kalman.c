#include "stdio.h"
#include "mpu6050.h"
#include "Kalman.h"
#include <math.h>
#include "ti_msp_dl_config.h"
#include "drv_oled.h"
 
#define AccSPL        16384.0        
#define GyroSPL       16.4  

uint8_t Yaw[50];

float Offset_ax = 0.0;
float Offset_ay = 0.0;
float Offset_az = 0.0;
float Offset_gx = 0.0;
float Offset_gy = 0.0;
float Offset_gz = 0.0;
 
void Kalman_Init(void)
{
    MPU6050_Init();
    
    Kalman_GetMPU6050_Offset(&Offset_ax, &Offset_ay, &Offset_az, &Offset_gx, &Offset_gy, &Offset_gz);
}

void Kalman_GetMPU6050_Offset(float *Offset_ax, float *Offset_ay, float *Offset_az, 
                                float *Offset_gx, float *Offset_gy, float *Offset_gz)
{
    uint16_t i; 
    float Count = 100.0; 
    
    float AccX, AccY, AccZ;
    float GyroX, GyroY, GyroZ;
    
    static float Offset_a[3];
    static float Offset_g[3];
    
    delay_ms(10);
    
    for (i = 0; i < Count; i++)
    {
        delay_us(20);
        Kalman_GetMPU6050_Data(&AccX, &AccY, &AccZ, &GyroX, &GyroY, &GyroZ); 
        
        Offset_a[0] += AccX;
        Offset_a[1] += AccY;
        Offset_a[2] += AccZ;
        Offset_g[0] += GyroX;
        Offset_g[1] += GyroY;
        Offset_g[2] += GyroZ; 
    }
    
    Offset_a[0] /= Count;
    Offset_a[1] /= Count;
    Offset_a[2] /= Count;
    Offset_g[0] /= Count;
    Offset_g[1] /= Count;
    Offset_g[2] /= Count;
    
    *Offset_ax = Offset_a[0] - 0.0;
    *Offset_ay = Offset_a[1] - 0.0;
    *Offset_az = Offset_a[2] - 1.0; 
    *Offset_gx = Offset_g[0] - 0.0;
    *Offset_gy = Offset_g[1] - 0.0;
    *Offset_gz = Offset_g[2] - 0.0; 
}    


void Kalman_GetMPU6050_Data(float *AccX, float *AccY, float *AccZ,
                            float *GyroX, float *GyroY, float *GyroZ)
{
    int16_t AccX_Temp;
    int16_t AccY_Temp;
    int16_t AccZ_Temp;
    
    static int16_t GyroX_Temp;
    static int16_t GyroY_Temp;
    static int16_t GyroZ_Temp;
    
    MPU6050_GetData(&AccX_Temp, &AccY_Temp, &AccZ_Temp, &GyroX_Temp, &GyroY_Temp, &GyroZ_Temp);
    
    *AccX = AccX_Temp / AccSPL - Offset_ax;    
    *AccY = AccY_Temp / AccSPL - Offset_ay;
    *AccZ = AccZ_Temp / AccSPL - Offset_az;

    *GyroX = GyroX_Temp / GyroSPL - Offset_gx;
    *GyroY = GyroY_Temp / GyroSPL - Offset_gy;
    *GyroZ = GyroZ_Temp / GyroSPL - Offset_gz;
    
} 

void Kalman_GetEuler_TempAngle(float *Roll_A, float *Pitch_A,
                                float *Roll_G, float *Pitch_G, float *Yaw_G)
{
    static float dt = 0.01;       
    
    float AccX, AccY, AccZ;
    static float GyroX, GyroY, GyroZ;
 
    Kalman_GetMPU6050_Data(&AccX, &AccY, &AccZ, &GyroX, &GyroY, &GyroZ);
    
    *Roll_A = atan2(AccY, AccZ) * 180 / 3.141593;
    
    *Pitch_A = -atan2(AccX, sqrt( AccY * AccY + AccZ * AccZ ) ) * 180 / 3.141593;
    
    
    *Roll_G  += GyroX *dt; 
    *Pitch_G += GyroY *dt;
    *Yaw_G   += GyroZ *dt;
    
}


float Kalman_Filter_Roll(float Angle_a, float Angle_g)
{
    static float Angle;  
    
    float dt      = 0.01;   
    
    float Q_Angle = 0.001; 
    float Q_Gyro  = 0.003; 
    float R_Angle = 0.5;  
    
    float Q_Bias;        
    float Angle_err;      
    float K_0;            
    float K_1;     
    
    static float P[2][2] = { {1, 0}, {0, 1} }; 
    
    Angle += (Angle_g - Q_Bias) * dt;          
    
    P[0][0] += ( Q_Angle - P[0][1] - P[1][0] * dt );
    P[0][1] += -P[1][1] * dt;
    P[1][0] += -P[1][1] * dt;
    P[1][1] += Q_Gyro * dt; 
    
    K_0 = P[0][0] / ( R_Angle + P[0][0] );
    K_1 = P[0][1] / ( R_Angle + P[0][0] );
    
    P[0][0] -= K_0 * P[0][0];
    P[0][1] -= K_0 * P[0][1];
    P[1][0] -= K_1 * P[0][0];
    P[1][1] -= K_1 * P[0][1];
    
    Angle_err = Angle_a - Angle;  
    Angle     += K_0 * Angle_err;  
    Q_Bias   += K_1 * Angle_err; 
    
    return Angle;
}

float Klaman_Filter_Pitct(float Angle_a, float Angle_g)
{
    static float Angle;
    
    static float dt      = 0.01;
    static float Q_Angle = 0.001;
    static float Q_Gyro  = 0.003; 
    static float R_Angle = 0.5; 
    
    static float Q_Bias;           
    static float Angle_err;    
    static float K_0;        
    static float K_1;              
    
    static float P[2][2] = { {1, 0}, {0, 1} };   
    
    Angle += (Angle_g - Q_Bias) * dt;
    
    P[0][0] += ( Q_Angle - P[0][1] - P[1][0] * dt );
    P[0][1] += -P[1][1] * dt;
    P[1][0] += -P[1][1] * dt;
    P[1][1] += Q_Gyro * dt;
    
    K_0 = P[0][0] / ( R_Angle + P[0][0] );
    K_1 = P[0][1] / ( R_Angle + P[0][0] );
    
    P[0][0] -= K_0 * P[0][0];
    P[0][1] -= K_0 * P[0][1];
    P[1][0] -= K_1 * P[0][0];
    P[1][1] -= K_1 * P[0][1];
    
    Angle_err = Angle_a - Angle;  
    Angle    +=K_0 * Angle_err;   
    Q_Bias   +=K_1 * Angle_err; 
    
    return Angle;
}

void Kalman_GetEuler_Angle(float *Roll, float *Pitch, float *Yaw)
{
    static float Roll_a;
    static float Pitch_a;
    static float Yaw_a;  

    static float Roll_g;
    static float Pitch_g;
    static float Yaw_g;
     
    Kalman_GetEuler_TempAngle(&Roll_a, &Pitch_a, &Roll_g, &Pitch_g, &Yaw_g);
    
    *Roll  = Kalman_Filter_Roll(Roll_a, Roll_g);
    *Pitch = Klaman_Filter_Pitct(Pitch_a, Pitch_g);
    *Yaw   = Yaw_g * 5.25;
}