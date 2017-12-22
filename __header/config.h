#pragma once

//#define WIN32

#include <iostream>
#include <vector>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef WIN32
	#include <time.h>
#else
	#include <sys/time.h>
#endif

#include <rplidar.h>
#include <physical_constant.h>


#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/legacy/legacy.hpp>

using namespace std;
using namespace rp::standalone::rplidar;
using namespace cv;

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifndef MIN
	#define MIN(x,y) (x)<(y)?(x):(y)
#endif

#ifndef MAX
	#define MAX(x,y) (x)>(y)?(x):(y)
#endif

#define SWAP(x, y) (x ^= y ^= x ^= y)

//#define PI 3.141592653f
#define ANGLE_ALL 360
#define Rplidar_Max_Range 6000		// 最大测量距离，单位: mm

#define LidarImageSize   600
#define LidarImageWidth  LidarImageSize
#define LidarImageHeight LidarImageSize

#define SUCCESS 0
#define FAILED  1

#define GROUP_MAX 120

typedef struct {
	_u8   Quality;
	float Angle;
	float Dst;
	float Vlct;		// 速度变量，这里本来该用联合体的，后面为了偷懒直接用这样表示

} __scandot;

typedef struct {

	float Angle;					// 角度
	float Angle_Last;				// 上次角度，这个变量是由于激光雷达转着转着会缺相而保留的，切记，数组同一位置的两个元素的角度可能不同

	double X, X_mid, X_last;		// 输入输出值，分别表示当前最优估计值，当前系统状态值，上次最优估计值
	double P, P_mid, P_last;		// 观测协方差，分别表示当前最优估计值，当前系统状态值，上次最优估计值

	double K;						// 卡尔曼增益

} __kalman;

typedef struct {

	__scandot Start_Pos;
	__scandot End_Pos;

	float Position[ANGLE_ALL];

	int Dst_Pixel;					// 在图像上线上最近点与中心点距离的像素点
	int Dst_mm;						// 在图像上线上最近点与中心点的实际距离

	float Theta;					// 直线在极坐标系内参数1
	float Rho;						// 直线在极坐标系内参数2

	float Angle_2_Center;			// 目标中心和雷达指向的实际相差的角度

} __obstacle_line;

typedef struct {

	__scandot Start_Pos;
	__scandot End_Pos;

	float Position[ANGLE_ALL];

	int   R;			// Radius 半径
	Point Center;		// 圆心

	int Dst_Pixel;
	int Dst_mm;

	float Angle_2_Center;

} __obstacle_circle;

typedef struct
{
	int R;
	int G;
	int B;

} __color;

//  速度
typedef struct
{
	float X;			// X分量
	float Y;			// Y分量

	float Yaw;			// 指向角度
	float Val;			// 模值

} __v;

typedef struct
{
	float X;			// X坐标
	float Y;			// Y坐标
	float Yaw;			// 偏航方向

} __pos;

// 这个是用来建图的类
typedef struct
{
	bool is_Line;
	bool is_Circle;
	bool is_New;
	bool is_Updated;

	__pos Center;

	__obstacle_line l;
	__obstacle_circle c;

} __obstacle_map;

// TinySLAM用的相关信息

typedef struct
{
	double X;			// in mm
	double Y;

	double Yaw;			// in degrees

} __ts_pos_t;

typedef struct {
	double X[ANGLE_ALL], Y[ANGLE_ALL];
	int val[ANGLE_ALL];
	int nb_points;
} __ts_scan_t;


typedef struct {

	float Pitch;
	float Roll;
	float Yaw;

} __AHRS;

typedef struct
{
	int X;
	int Y;

} __point2f;

typedef struct
{
	float r;
	float deg;

} __point2p;

typedef struct
{
	float X;
	float Y;
	float Z;

} __Vec3f;
