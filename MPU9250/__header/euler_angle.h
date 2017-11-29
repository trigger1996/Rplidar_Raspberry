#ifndef __EULER_ANGLE_H

#define __EULER_ANGLE_H

#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <sys/time.h>

#include "mpu9250.h"

using namespace std;

typedef struct {

	float X;
	float Y;
	float Z;

} __vec3f;

class __e_angle {

public:
	__e_angle();
	~__e_angle();


	float Pitch;
	float Roll;
	float Yaw;
	__vec3f Acc, Gyro, Mag;


	float LPF(float data, float data_last, float gain);
	float process_Data(__vec3f a, __vec3f g, __vec3f h);

private:

	__vec3f Acc_Raw,  Acc_Last;
	__vec3f Gyro_Raw, Gyro_Last;
	__vec3f Mag_Raw,  Mag_Last;

	// 低通滤波器增益
	float acc_lpf_gain;
	float gyro_lpf_gain;
	float mag_lpf_gain;

	// 四元数
	float q0, q1, q2, q3;						// 四元数的元素，代表估计方向
	float exInt, eyInt, ezInt;					// 按比例缩小积分误差
	float Kp;									// 比例增益支配率收敛到加速度计/磁强计
	float Ki;									// 积分增益支配率的陀螺仪偏见的衔接
	float halfT;								// 采样周期的一半

	timeval t_now, t_last;						// 时间
	int t;

	void calc_Q(float gx, float gy, float gz, float ax, float ay, float az);

};



#endif // __EULER_ANGLE_H
