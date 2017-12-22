#include "euler_angle.h"

// http://blog.csdn.net/haishaoli/article/details/52965457

__e_angle::__e_angle() {

	// 归零数据
	__vec3f zero_v = { 0.0f };
    Acc  = Acc_Raw  = Acc_Last  = zero_v;
    Gyro = Gyro_Raw = Gyro_Last = zero_v;
    Mag  = Mag_Raw  = Mag_Last  = zero_v;


	acc_lpf_gain  = 0.45f;
	gyro_lpf_gain = 0.45f;
	mag_lpf_gain  = 0.45f;

	q0 = 1;
	q1 = 0;
	q2 = 0;
	q3 = 0;									// 四元数的元素，代表估计方向
	exInt = 0;
	eyInt = 0;
	ezInt = 0;								// 按比例缩小积分误差
	Kp = 100.0f;							// 比例增益支配率收敛到加速度计/磁强计
	Ki = 0.002f;							// 积分增益支配率的陀螺仪偏见的衔接
	halfT = 0.001f;

	t_now.tv_sec = t_now.tv_usec = 0;
	t_last.tv_sec = t_last.tv_usec = 0;

}// __e_angle::e_angle()

__e_angle::~__e_angle() {



}// __e_angle::~__e_angle()

float __e_angle::process_Data(__vec3f a, __vec3f g, __vec3f h) {

	// 得到新数据
	Acc_Raw  = a;
	Gyro_Raw = g;
	Mag_Raw  = h;

	// 迭代
    Acc_Last  = Acc;
    Gyro_Last = Gyro;
    Mag_Last  = Mag;

	// 过低通滤波器
	Acc.X = LPF(Acc_Raw.X, Acc_Last.X, acc_lpf_gain);
	Acc.Y = LPF(Acc_Raw.Y, Acc_Last.Y, acc_lpf_gain);
	Acc.Z = LPF(Acc_Raw.Z, Acc_Last.Z, acc_lpf_gain);

	Gyro.X = LPF(Gyro_Raw.X, Gyro_Last.X, gyro_lpf_gain);
	Gyro.Y = LPF(Gyro_Raw.Y, Gyro_Last.Y, gyro_lpf_gain);
	Gyro.Z = LPF(Gyro_Raw.Z, Gyro_Last.Z, gyro_lpf_gain);

	Mag.X = LPF(Mag_Raw.X, Mag_Last.X, mag_lpf_gain);
    Mag.Y = LPF(Mag_Raw.Y, Mag_Last.Y, mag_lpf_gain);
    Mag.Z = LPF(Mag_Raw.Z, Mag_Last.Z, mag_lpf_gain);

	// 获得时间，用于积分
	t_last = t_now;
	gettimeofday(&t_now, NULL);
	// 获得经过的时间
	t = t_now.tv_usec - t_last.tv_usec;
	if (t < 0)
		t = t_now.tv_usec + 1000000 - t_last.tv_usec;

	// 互补滤波
    float angle_s, angle_i;		// 传感器获得的角度，积分获得的角度
	float threshold = 0.75f;
	float dt = (float)t / 1000000;

    // pitch
	angle_s = atan(-Acc.X / Acc.Z) * 180.0f / PI;
	angle_i = Pitch + Gyro.Y * dt;
	Pitch = angle_s * threshold + angle_i * (1 - threshold);

	// roll
	angle_s = atan(Acc.Y / Acc.Z) * 180.0f / PI;
	angle_i = Roll + Gyro.X * dt;
	Roll = angle_s * threshold + angle_i * (1 - threshold);

	// yaw
	// 地磁倾角补偿
	// http://blog.csdn.net/aileenyuxiao/article/details/44172061
	float hx = Mag.Y*cos(Pitch) + Mag.Y*sin(Pitch)*sin(Roll) - Mag.Z*cos(Roll)*sin(Pitch);
	float hy = Mag.X*cos(Roll) + Mag.Z*sin(Roll);
	threshold = 0.988f;

	angle_s = atan2(hy, hx) * 180.0f / PI;
//	calc_Q(Gyro.X, Gyro.Y, Gyro.Z, Acc.X, Acc.Y, Acc.Z);

	// 陀螺仪积分解算航向角
	angle_i = Yaw + Gyro.Z * dt;

	// 地磁解算的航向角与陀螺仪积分解算的航向角进行互补融合
	if((angle_s>90 && Yaw<-90) || (angle_s<-90 && Yaw>90))
		Yaw = -angle_i * (1 - threshold) + angle_s * threshold;
	else
		Yaw = angle_i * (1 - threshold) + angle_s * threshold;


	return 0.0f;

}// float __e_angle::process_Data(__vec3f a, __vec3f g, __vec3f h)


//卡尔曼滤波
// http://blog.csdn.net/zsn15702422216/article/details/52223799
// http://blog.csdn.net/haishaoli/article/details/52965457
// http://blog.csdn.net/dyl74500196/article/details/59108807
/*
float Kalman_Filter(float angle_m, float gyro_m, float dt)//angleAx 和 gyroGy
{

	//卡尔曼滤波参数与函数
	//static float dt=0.001;//注意：dt的取值为kalman滤波器采样时间
	static float angle, angle_dot;//角度和角速度
	static float P[2][2] = {{ 1, 0 },
							 { 0, 1 }};
	static float Pdot[4] ={ 0,0,0,0 };
	static float Q_angle=0.001, Q_gyro=0.005; //角度数据置信度,角速度数据置信度
	static float R_angle=0.5 ,C_0 = 1;
	static float q_bias, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;

	angle += (gyro_m-q_bias) * dt;
	angle_err = angle_m - angle;
	Pdot[0]=Q_angle - P[0][1] - P[1][0];
	Pdot[1]=- P[1][1];
	Pdot[2]=- P[1][1];
	Pdot[3]=Q_gyro;
	P[0][0] += Pdot[0] * dt;
	P[0][1] += Pdot[1] * dt;
	P[1][0] += Pdot[2] * dt;
	P[1][1] += Pdot[3] * dt;
	PCt_0 = C_0 * P[0][0];
	PCt_1 = C_0 * P[1][0];
	E = R_angle + C_0 * PCt_0;
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	t_0 = PCt_0;
	t_1 = C_0 * P[0][1];
	P[0][0] -= K_0 * t_0;
	P[0][1] -= K_0 * t_1;
	P[1][0] -= K_1 * t_0;
	P[1][1] -= K_1 * t_1;
	angle += K_0 * angle_err; //最优角度
	q_bias += K_1 * angle_err;
	angle_dot = gyro_m-q_bias;//最优角速度

	return angle;
}
*/

float __e_angle::LPF(float data, float data_last, float gain) {

	float ret;

	ret = data * gain + (1 - gain) * data_last;
	return ret;

}// float __e_angle::LPF()

void __e_angle::calc_Q(float gx, float gy, float gz, float ax, float ay, float az)
{
	// 四元数姿态融合求偶拉角
	// http://blog.csdn.net/zsn15702422216/article/details/52223799

	float norm;
	float vx, vy, vz;
	float ex, ey, ez;

	halfT = (float)t / 1000000 / 2;

	// 测量正常化
	norm = sqrt(ax*ax + ay*ay + az*az);
	ax = ax / norm;		   //单位化
	ay = ay / norm;
	az = az / norm;

	// 估计方向的重力
	vx = 2*(q1*q3 - q0*q2);
	vy = 2*(q0*q1 + q2*q3);
	vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

	// 错误的领域和方向传感器测量参考方向之间的交叉乘积的总和
	ex = (ay*vz - az*vy);
	ey = (az*vx - ax*vz);
	ez = (ax*vy - ay*vx);

	// 积分误差比例积分增益
	exInt = exInt + ex*Ki;
	eyInt = eyInt + ey*Ki;
	ezInt = ezInt + ez*Ki;

	// 调整后的陀螺仪测量
	gx = gx + Kp*ex + exInt;
	gy = gy + Kp*ey + eyInt;
	gz = gz + Kp*ez + ezInt;

	// 整合四元数率和正常化
	q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
	q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
	q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
	q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

	// 正常化四元
	norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 / norm;
	q1 = q1 / norm;
	q2 = q2 / norm;
	q3 = q3 / norm;

	Pitch  = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; // pitch ,转换为度数
	Roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // rollv
	Yaw = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;		//此处没有价值，注掉
}
