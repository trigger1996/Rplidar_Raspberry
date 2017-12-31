#ifndef __EKF_SLAM_H

#define __EKF_SLAM_H

#include "config.h"

#include <iostream>
#include <cmath>
#include <math.h>
#include <vector>
#include <stdio.h>
#include <Eigen/Dense>

using namespace std;
using Eigen::MatrixXd;
using Eigen::MatrixXd;

//#define PI 3.1415926f

bool is_nan(double dval);

typedef struct
{
	__point2p_rad PosData;
	__point2f Grid;
	__point2f VehicleGrid_Recorded;
} __landmark;

class __ekf_slam
{
public:

	__ekf_slam();

	__point2f current_Pos;

	int get_Sensors(vector<__point2p> lidar, __Vec3f a, __Vec3f w, __Vec3f v, double pitch, double roll, double yaw, double t);
	int run(bool is_updated);

private:

	MatrixXd Xe, X, Xe_Last;
	MatrixXd Pe, P;
	MatrixXd Z,  innov;
	MatrixXd H;

	MatrixXd F, W, R;			// F->A
	MatrixXd Q, V;

	MatrixXd Jh;
	MatrixXd K;

	__Vec3f Acc, Gyro;
	__Vec3f Vlct;
	double G;				// 重力加速度

	double Pitch, Roll;
	double phi;			// Yaw
	double dt;				// 时间, 单位: s
	double Time;			// 总时间, 单位: s

	int numStates;
	int landmark_num, landmark_num_last;

	vector<__landmark> landmark;

	__point2p_rad nearby_pt;

	int init();

	int match_Landmark(vector<__point2p> lidar);

	int new_State(__point2p_rad z_lidar);
	int update_New(__point2p_rad z);
	int update_Existing(__point2p_rad z, int landmark);

	int nearby_LMs();

	double normalize_Angle(double in);

};


#endif // __EKF_SLAM_H
