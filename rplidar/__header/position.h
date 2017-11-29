#pragma once

#include "config.h"

/*

模块资料：
	负责读取分组后的障碍物的数据，然后进行



*/

#define PlainMap_Size   1200
#define PlainMap_Width  PlainMap_Size
#define PlainMap_Heigth PlainMap_Size

// 卡尔曼滤波器参数
#define QPos_Dst	22500.0f	// 距离的的估计噪声，我们这里用速度的测量噪声带入，因为距离的的估计和速度有关
#define RPos_Dst	0.25f		// 距离的测量噪声，测量噪声和距离本身的精度有关系，我们带入0.5mm，这里直接平方即0.25

extern float PlainMap_Scale;		// 600 / 6000 / 2

class __positioning
{
public:

	__positioning();											// 构造
	~__positioning();											// 删除

	Mat						Map;								// 地图图片
	__pos Pos, Pos_Last, Pos_Last_2, StartPos;					// 坐标
	vector<__obstacle_line> Line_Group, Line_GLast;				// 读过来的线的距离
	vector<__obstacle_map>  OB_All;								// 所有障碍物的集合
	__v						V;									// 速度
	float					Freq;								// 频率


	int num;													// 有效的障碍物数量
	

	int update_LineGroup(vector<__obstacle_line> ob);

	int calc_Grid_Velocity(float vx, float vy, float freq);		// 坐标和速度计算

	int draw_Map();												// 作图

private:

	vector<__v>				rV;									// 各组障碍物的速度
	vector<__pos>			rGrid, rGrid_Last;					// 每组障碍物移动的相对坐标，注意这边的Yaw不用

	__kalman				gk_x, gk_y;							// 位置卡尔曼滤波器参数

	int calc_GridGroup();										// 计算坐标组
	int calc_Grid(float freq);									// 计算坐标
	int fuse_Obstacle();										// 障碍物整合

	int calc_Velocity(float freq);								// 计算速度组

	int V_Complementary_Filter(float vx, float vy);				// 速度互补滤波器，和别的部分的速度用来做互补滤波

	int G_Kalman_Filter();										// 位置卡尔曼滤波器


};




