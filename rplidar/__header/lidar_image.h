#pragma once


#include "config.h"
#include <cmath>

#define Q_Spd	9000.0f		// 速度的估计噪声，速度的估计和加速度有关系，这里我们用加速度的值的波动的平方来计算，加速度的波动大概在0.18f~-0.12f波动，单位应该是m/s^2
#define R_Spd	22500.0f	// 速度的测量噪声，测量噪声可以通过观测计算值得，测量误差会达到150mm左右

#define Q_Dst	R_Spd		// 距离的的估计噪声，我们这里用速度的测量噪声带入，因为距离的的估计和速度有关
#define R_Dst	0.25f		// 距离的测量噪声，测量噪声和距离本身的精度有关系，我们带入0.5mm，这里直接平方即0.25

class __lidar_img
{
public:
	__lidar_img();		// 构造
	~__lidar_img();		// 销毁

	vector<__scandot> Data;
	vector<__scandot> Data_Last;
	vector<__scandot> Vlct;

	float Vx;							// x方向: 平行于车辆前进方向的方向
	float Vy;							// y方向: 垂直于车辆前进方向的方向

	float Scan_Speed;

	Mat Img_Dst_Raw;					// 距离图，原始数据

	// 归一化数据
	float Data_NArray[ANGLE_ALL];		// 归一化后的数据
	float Data_NLast[ANGLE_ALL];		// 归一化后数据备份

	float Vlct_NArray[ANGLE_ALL];		// 速度

	// 卡尔曼滤波器
	__kalman KF_Speed[ANGLE_ALL];		// 速度卡尔曼滤波器
	__kalman KF_Dst[ANGLE_ALL];			// 距离卡尔曼滤波器

	float Data_KArray[ANGLE_ALL];		// 卡尔曼滤波器输出
	float Vlct_KArray[ANGLE_ALL];


	int scanData(rplidar_response_measurement_node_t *buffer, size_t count, float frequency);
	int Draw(Mat &dst, vector<__scandot> data, char window_name[]);
	int Draw(Mat &dst, float data[], char window_name[]);

	int Normalize_Data(vector<__scandot> data);

	int calc_Velocity(void);

	int Kalman_Filter(float acc_x, float acc_y);

	int Vlct_Orthogonal_Decomposition(float vlct[]);

	int normalize_Orentation(float data[], int yaw);

private:

};
