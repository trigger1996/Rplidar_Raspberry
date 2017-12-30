#ifndef __Obstacle_Group_H

#define __Obstacle_Group_H

#include "config.h"


#define Line_Threshold 5.0f

///
/// 这个模块主要负责单帧的图像建立以及障碍物的分组
/// 主要通过霍夫变换和方差来达到目的
class __obstacle_group
{
public:
	__obstacle_group();			// 构造方法
	~__obstacle_group();		// KO

	float Data[ANGLE_ALL];
	float Data_Last[ANGLE_ALL];

	vector<__obstacle_line> OLines;

	Mat Img_Initial;			// 初始的图
	Mat Img_Lines, Img_Lines_Last;

	int get_Array(float data[]);
	int draw();
	int draw(Mat &dst, vector<__obstacle_line> ob);
	int draw_lines(Mat &dst, vector<__obstacle_line> ob);
	int calc_Lines();

	int surf();

private:


	int merge_LineGroup();
	int remove_Standlone_Pts();


};


#endif	/* __Obstacle_Group_H */
