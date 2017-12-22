#ifndef __EKF_SLAM_H

#define __EKF_SLAM_H

#include "config.h"

#include <iostream>
#include <cmath>
#include <math.h>
#include <vector>
#include <stdio.h>
#include <Eigen/Dense>

// http://eigen.tuxfamily.org/dox/GettingStarted.html#title2
// https://www.cnblogs.com/goingupeveryday/p/5699053.html

using namespace std;
using Eigen::MatrixXd;
using Eigen::MatrixXf;

//#define PI 3.1415926535897932384626

class __ekf_slam
{
public:

	__ekf_slam();

	int get_Sensor(vector<__point2p> lidar, __Vec3f a, __Vec3f w, float yaw, float t);
	int run();


private:

	MatrixXd Xe, X;				// 观测值
	MatrixXd Pe, P;				// 协方差

	MatrixXd K;					// 卡尔曼增益

	MatrixXd F, Jh, W, V;		// 过程变量
	MatrixXd Q, R;				// 过程噪声和观测噪声

	MatrixXd H;					// 传感器模型，这边用Xe去掉前面三项带入

	MatrixXd Z;			// 传感器输入

	int landmark_num, landmark_num_last;

	float phi;			// Yaw
	__Vec3f Acc;
	__Vec3f Gyro;

	float dt;			// 单位: s

};

#endif // __EKF_SLAM_H
