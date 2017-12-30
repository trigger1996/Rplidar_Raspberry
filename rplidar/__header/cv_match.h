#pragma once

#include "config.h"

#include <iostream>
#include <fstream>
#include <sstream>

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

double calc_Distance_Pt2Line(Point pt1, Point pt2, Point core);
double calc_Distance_Pt2Pt(Point pt1, Point pt2);
double tan_yaw(double x, double y);

class __cv_match
{
public:
	__cv_match();
	~__cv_match();

	int set_Data(float raw[]);
	int run();
	vector<__point2p> get_DataArray();

private:

	Mat Img_Raw;
	float Data_Raw[ANGLE_ALL];

	vector<Vec4i> lines;						// 定义一个矢量结构lines用于存放得到的线段矢量集合
	vector<Vec4i> lines_out;					// 输出

	vector<__point2p> Data_Out;

	int draw(bool is_show);
	int show_lines(bool is_show_raw_data);

	int do_HoughTransformP(bool is_show, bool is_show_raw_data);
	int merge_LineGroup();

};
