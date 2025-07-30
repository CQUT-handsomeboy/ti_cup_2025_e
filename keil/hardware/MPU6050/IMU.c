
//#include "IMU.h"
//#include "ti_msp_dl_config.h"

//#define kp 				20.00f
//#define ki 				0.001f
//#define cycle_T 		0.005f//200hz
//#define half_T 			0.0025f

//int16_t Ax,Ay,Az,Gx,Gy,Gz;
//param_imu imu_data;
//param_Angle imu_Angle;
//float q[4] = {1.0,0.0,0.0,0.0};
//float exInt = 0.0 ,eyInt = 0.0 ,ezInt = 0.0 ;
////////////////////////////////////////////////////////////////////////////////////
//float fast_sqrt(float x)
//{
//    float halfx = 0.5f * x;
//    float y = x;
//    long i = *(long *) &y;
//    i = 0x5f3759df - (i >> 1);  //0x5f3759dfê?ò?????·??ùμ1êy?ù??·¨
//    y = *(float *) &i;
//    y = y * (1.5f - (halfx * y * y));
//    return y;
//}
////////////////////////////////////////////////////////////////////////////////////

//void IMU_GetValues(void)
//{
//	MPU6050_GetData(&Ax,&Ay,&Az,&Gx,&Gy,&Gz);
//	
//	 imu_data.AX = ((float)Ax)/2048;
//	 imu_data.AY = ((float)Ay)/2048;
//     imu_data.AZ = ((float)Az)/2048;
//	
//     imu_data.GX = ((float)Gx)*0.001064;
//     imu_data.GY = ((float)Gy)*0.001064;
//     imu_data.GZ = ((float)Gz)*0.001064;
//}
//void IMU_AHRSupdate(param_imu* imu_temp)
//{
//	float ax,ay,az;
//	float gx,gy,gz;
//	ax = imu_temp->AX;
//	ay = imu_temp->AY;
//	az = imu_temp->AZ;
//	gx = imu_temp->GX;
//	gy = imu_temp->GY;
//	gz = imu_temp->GZ;
//	float vx, vy, vz; 
//	float ex, ey, ez; 
//	float q0 = q[0];
//  float q1 = q[1];
//  float q2 = q[2];
//  float q3 = q[3];
//	float norm = fast_sqrt(ax*ax+ay*ay+az*az);
//    ax = ax * norm;
//    ay = ay * norm;
//    az = az * norm;
//	
//	vx = 2 * (q1*q3 - q0*q2);
//	vy = 2 * (q2*q3 + q0*q1);
//  vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
//	
//    ex = (ay * vz - az * vy);//  |A|*|B|*sin<A,B>
//    ey = (az * vx - ax * vz);
//    ez = (ax * vy - ay * vx);
//	
//	exInt += ki * ex;
//	eyInt += ki * ey;
//	ezInt += ki * ez;
//	gx += kp * ex + exInt;
//	gy += kp * ey + eyInt;
//	gz += kp * ez + ezInt;
//	q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * half_T;
//    q1 = q1 + (q0 * gx + q2 * gz - q3 * gy)  * half_T;
//    q2 = q2 + (q0 * gy - q1 * gz + q3 * gx)  * half_T;
//    q3 = q3 + (q0 * gz + q1 * gy - q2 * gx)  * half_T;
//	norm = fast_sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
//	q[0] = q0 * norm;
//	q[1] = q1 * norm;
//	q[2] = q2 * norm;
//	q[3] = q3 * norm;
//	
//}
#include "IMU.h"  
#include "ti_msp_dl_config.h"  
  
#define cycle_T 		0.005f // 200Hz  
#define half_T 			0.0025f  
  
int16_t Ax, Ay, Az, Gx, Gy, Gz;  
param_imu imu_data;  
param_Angle imu_Angle;
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};  
  
float fast_sqrt(float x)  
{  
    float halfx = 0.5f * x;  
    float y = x;  
    long i = *(long *) &y;  
    i = 0x5f3759df - (i >> 1);  
    y = *(float *) &i;  
    y = y * (1.5f - (halfx * y * y));  
    return y;  
}  
  
void IMU_GetValues(void)  
{  
    MPU6050_GetData(&Ax, &Ay, &Az, &Gx, &Gy, &Gz);  
  
    imu_data.AX = ((float)Ax) / 2048.0f;  
    imu_data.AY = ((float)Ay) / 2048.0f;  
    imu_data.AZ = ((float)Az) / 2048.0f;  
  
    imu_data.GX = ((float)Gx) * 0.001064f;  
    imu_data.GY = ((float)Gy) * 0.001064f;  
    imu_data.GZ = ((float)Gz) * 0.001064f;  
}  
  
void IMU_AHRSupdate(param_imu* imu_temp)  
{  
    float ax, ay, az;  
    float gx, gy, gz;  
    float q0, q1, q2, q3;  
    float vx, vy, vz;  
    float norm;  
  
    ax = imu_temp->AX;  
    ay = imu_temp->AY;  
    az = imu_temp->AZ;  
    gx = imu_temp->GX;  
    gy = imu_temp->GY;  
    gz = imu_temp->GZ;  
  
    q0 = q[0];  
    q1 = q[1];  
    q2 = q[2];  
    q3 = q[3];  
  
    norm = fast_sqrt(ax * ax + ay * ay + az * az);  
    ax = ax / norm;  
    ay = ay / norm;  
    az = az / norm;  
  
    vx = 2 * (q1 * q3 - q0 * q2);  
    vy = 2 * (q2 * q3 + q0 * q1);  
    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;  
  
    // 使用陀螺仪数据来预测四元数的变化  
    q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * half_T;  
    q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * half_T;  
    q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * half_T;  
    q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * half_T;  
  
    // 规范化四元数  
    norm = fast_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);  
    q[0] = q0 / norm;  
    q[1] = q1 / norm;  
    q[2] = q2 / norm;  
    q[3] = q3 / norm;  
}
void IMU_getEuleranAngles(void)
{
	IMU_GetValues();
	IMU_AHRSupdate(&imu_data);
	
	imu_Angle.Pitch = asin(-2 * q[1] * q[3] + 2 * q[0]* q[2])* 57.2957;
	imu_Angle.Roll  = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2]* q[2] + 1)* 57.2957;
	
	imu_Angle.Yaw  = atan2(2*(q[1] * q[2]+q[0] * q[3]),q[0] * q[0]+q[1] * q[1]-q[2] * q[2]-q[3] * q[3])*57.3;
	
}   

//uint8_t mpu_dmp_get_data(float *pitch,float *roll,float *yaw)
//{
//	float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
//	unsigned long sensor_timestamp;
//	short gyro[3], accel[3], sensors;
//	unsigned char more;
//	long quat[4]; 
//	if(dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors,&more))return 1;
//	if(sensors&INV_WXYZ_QUAT) 
//	{
//	q0 = quat[0] / q30; //q30 格式转换为浮点数
//	q1 = quat[1] / q30;
//	q2 = quat[2] / q30;
//	q3 = quat[3] / q30; 
//	//计算得到俯仰角/横滚角/航向角
//	*pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; // pitch
//	*roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3;// roll
//	*yaw= atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;//yaw
//	}else return 2;
//	return 0;
//}
