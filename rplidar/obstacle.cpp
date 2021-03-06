﻿#include <obstacle.h>

__obstacle_group::__obstacle_group()
{
	Mat zero(LidarImageHeight, LidarImageWidth, CV_8UC3, Scalar(0, 0, 0));		// 图片格式： BGR

	Img_Initial = zero.clone();
	Img_Lines = zero.clone();
	Img_Lines_Last = zero.clone();

	zero.release();


}// __obstacle_group::__obstacle_group()

__obstacle_group::~__obstacle_group()
{


}// __obstacle_group::~__obstacle_group()

int __obstacle_group::get_Array(float data[])
{
	// 获得算出来的角度-距离信息
	int i;

	for (i = 0; i < ANGLE_ALL; i++)
	{
		Data_Last[i] = Data[i];
		Data[i] = data[i];

		// 增加一个死区
		if (Data[i] < Rplidar_Max_Range / 10.0f)
			Data[i] = 0.0f;

	}
	
	return SUCCESS;

}// int __obstacle_group::get_Array(float data[])

int __obstacle_group::draw()
{

	Mat zero(LidarImageHeight, LidarImageWidth, CV_8UC3, Scalar(0, 0, 0));		// 图片格式： BGR
	Img_Initial = zero.clone();
	zero.release();

	//在中心加上一个圆心
	//circle(Img_Initial, Point(Img_Initial.cols / 2, Img_Initial.rows / 2), 3, Scalar(255, 0, 0), -1, 8, 0);

	int x, y;
	double theta, rho;
	int halfWidth = Img_Initial.cols / 2;
	int halfHeight = Img_Initial.rows / 2;


	for (unsigned int i = 0; i < ANGLE_ALL; i++)	// scan_data.size()
	{
		theta = i * PI / 180;
		rho = Data[i];

		if (rho <= Rplidar_Max_Range / 200.0f)
			continue;

		x = (int)(rho  * sin(theta) / 20) + halfWidth;
		y = (int)(-rho * cos(theta) / 20) + halfHeight;

		if ((x >= 0 && x < LidarImageWidth) &&
			(y >= 0 && y < LidarImageHeight))		// 检查这些点在不在图内
		{
			circle(Img_Initial, Point(x, y), 1, Scalar(255, 255, 255), -1, 8, 0);
		}
	}

	cvtColor(Img_Initial, Img_Initial, CV_BGR2GRAY);
	imshow("Data_Init", Img_Initial);

	return SUCCESS;


}// __obstacle_group::draw()

int __obstacle_group::draw(Mat &dst, vector<__obstacle_line> ob)
{
	Mat zero(LidarImageHeight, LidarImageWidth, CV_8UC3, Scalar(0, 0, 0));		// 图片格式： BGR
	dst = zero.clone();
	zero.release();

	//在中心加上一个圆心
	//circle(Img_Initial, Point(Img_Initial.cols / 2, Img_Initial.rows / 2), 3, Scalar(255, 0, 0), -1, 8, 0);
	int i, j;

	int x, y;
	double theta, rho;
	int halfWidth = dst.cols / 2;
	int halfHeight = dst.rows / 2;

	__color rgb;
	srand(time(0));
	rgb.R = rand() % 256;
	rgb.G = rand() % 256;
	rgb.B = rand() % 256;

	for (i = 0; i < ob.size(); i++)
	{
		// 随机取色，用于分组
		rgb.R = rand() % 256;
		rgb.G = rand() % 256;
		rgb.B = rand() % 256;

		for (j = 0; j < ANGLE_ALL; j++)
		{
			if (ob[i].Position[j] != 0.0f)		
			{
				theta = j * PI / 180;
				rho = ob[i].Position[j];

				x = (int)(rho  * sin(theta) / 20) + halfWidth;
				y = (int)(-rho * cos(theta) / 20) + halfHeight;

				if ((x >= 0 && x < LidarImageWidth) &&
					(y >= 0 && y < LidarImageHeight))
				{
					circle(dst, Point(x, y), 1, Scalar(rgb.B, rgb.G, rgb.R), -1, 8, 0);
				}
			}
		}
	}


	//cvtColor(dst, dst, CV_BGR2GRAY);
	imshow("Data", dst);

	return SUCCESS;


}// __obstacle_group::draw(Mat &dst, vector<__obstacle_line> ob)

int __obstacle_group::draw_lines(Mat &dst, vector<__obstacle_line> ob)
{

	Mat zero(LidarImageHeight, LidarImageWidth, CV_8UC3, Scalar(0, 0, 0));		// 图片格式： BGR
	dst = zero.clone();
	zero.release();

	//在中心加上一个圆心
	//circle(Img_Initial, Point(Img_Initial.cols / 2, Img_Initial.rows / 2), 3, Scalar(255, 0, 0), -1, 8, 0);
	int i, j;

	int x, y;
	int x_min, x_max;
	int y_1, y_2;				// y_1对应x_min，y_2对应x_max
	double theta, rho;
	int halfWidth = dst.cols / 2;
	int halfHeight = dst.rows / 2;

	// 颜色
	__color rgb;
	srand(time(0));
	rgb.R = rand() % 256;
	rgb.G = rand() % 256;
	rgb.B = rand() % 256;


	for (i = 0; i < ob.size(); i++)
	{
		// 首先找到组里面x的最大/最小值，然后算出对应的y，用来画直线
		// 虽然这样求出来的x不一定是线上的x，但是可近似到线上
		// 求最大值
		x_max = 0;
		for (j = 0; j < ANGLE_ALL; j++)
		{
			float theta = j * PI / 180;
			float rho = ob[i].Position[j];

			x = (int)(rho  * sin(theta) / 20) + halfWidth;

			if (x_max <= x)
				x_max = x;
		}

		// 求最小值
		x_min = dst.cols;
		for (j = 0; j < ANGLE_ALL; j++)
		{
			float theta = j * PI / 180;
			float rho = ob[i].Position[j];

			x = (int)(rho  * sin(theta) / 20) + halfWidth;

			if (x_min >= x)
				x_min = x;
		}

		// 直线方程
		// r = x * cos + y * sin
		// y = r / sin - x * cos / sin
		float rho = ob[i].Rho;
		float theta = ob[i].Theta;
		double a = cos(theta), b = sin(theta);

		y_1 = rho - x_min * a;
		y_1 = y_1 / b;
		y_2 = rho - x_max * a;
		y_2 = y_2 / b;

		Point pt1; pt1.x = x_min; pt1.y = y_1;
		Point pt2; pt2.x = x_max; pt2.y = y_2;

		// 随机取色，用于分组
		rgb.R = rand() % 256;
		rgb.G = rand() % 256;
		rgb.B = rand() % 256;

		line(dst, pt1, pt2, Scalar(rgb.B, rgb.G, rgb.R), 1, CV_AA);

	}

	//cvtColor(dst, dst, CV_BGR2GRAY);
	imshow("Data_lines", dst);

	return SUCCESS;

}// __obstacle_group::draw_lines(Mat &dst, vector<__obstacle_line> ob)

int __obstacle_group::calc_Lines()
{

	int i, j, k;					// 计数变量

	// 求直线的分组
	int halfWidth = Img_Initial.cols / 2;
	int halfHeight = Img_Initial.rows / 2;
	bool is_checked[ANGLE_ALL] = { false };				// 这个是在整个遍历中检查这个点用过没
	__obstacle_line obstacle;
	Mat dst;


	// 求“方差”工具，这里用绝对值平均值代替方差
	int sum;
	int pt_num;
	int min_pos;
	float avg[GROUP_MAX];		// 30不够

	// 2017.11.10补充
	// 增加一个 闭运算，会让识别率上去非常多
	Mat element = getStructuringElement(MORPH_RECT, Size(10, 10));
	morphologyEx(Img_Initial, Img_Initial, MORPH_CLOSE, element);
	//imshow("Close", Img_Initial);

	// 霍夫变换
	vector<Vec2f> lines;
	HoughLines(Img_Initial, lines, 0.5, CV_PI / 360 * 1.5, 32);	// 前面就跑过一次cvtColor了
																	// 返回直线坐标对 CV_PI / 360
																	// Img, lines, 0.5, CV_PI / 360 * 15, 35
	// 其实首先还是要对变量进行粗分组，这样得到大致的分组以后，然后进行一轮计算
	//
	//
	OLines.clear();

	// 遍历所有的点，检查是否属于某一条直线
	for (j = 0; j < lines.size(); j++)
	{

		// 这个检查单个的障碍物用到了哪些点
		__obstacle_line obstacle;
		bool is_checked_array[ANGLE_ALL] = { false };

		obstacle = { 0 };
		memset(is_checked_array, false, sizeof(bool) * ANGLE_ALL);
		for (i = 0; i < ANGLE_ALL; i++)
		{
			// 霍夫变换查找出来的线的变量
			float rho = lines[j][0], theta = lines[j][1];
			double a = cos(theta), b = sin(theta);

			// 激光雷达测出来的点
			float theta_pt = i * PI / 180;
			float rho_pt = Data[i];
			int x_pt = (int)(rho_pt  * sin(theta_pt) / 20) + halfWidth;
			int y_pt = (int)(-rho_pt * cos(theta_pt) / 20) + halfHeight;

			// 直线方程
			// r = x * cos + y * sin
			if (!is_checked[i])
			{
				// 检查是否用过这些点
				// 然后开始检查到底是不是在阈值内，且这些点必须在图像内
				if ((x_pt >= 0 && x_pt < LidarImageWidth) &&
					(y_pt >= 0 && y_pt < LidarImageHeight))
				{
					if (fabs(x_pt * a + y_pt * b - rho) <= Line_Threshold)
					{
						is_checked[i] = true;
						obstacle.Position[i] = Data[i];
					}
				}
			}
		}

		// 把找到的障碍物信息push到动态数组内
		OLines.push_back(obstacle);
	}

	// 防止爆炸，有的时候阈值降低很容易炸程序，溢出
	if (OLines.size() > GROUP_MAX)
		return FAILED;

	// 第二轮，精匹配
	// 求最拟合的直线给每一组障碍物，用来对其进行表示
	for (i = 0; i < OLines.size(); i++)
	{
	// 用一个数组表示查出来的直线的方差，这里的方差在最后取最小的，那么这个最小的对应的直线即为拟合的最好的
	memset(avg, 0, sizeof(float) * 30);
	for (j = 0; j < lines.size(); j++)
	{
		sum = 0;
		pt_num = 0;
		for (k = 0; k < ANGLE_ALL; k++)
		{
			if (OLines[i].Position[k] != 0.0f)
			{
				// 霍夫变换查找出来的线的变量
				float rho = lines[j][0], theta = lines[j][1];
				double a = cos(theta), b = sin(theta);

				// 激光雷达测出来的点
				float theta_pt = k * PI / 180;
				float rho_pt = OLines[i].Position[k];
				int x_pt = (int)(rho_pt  * sin(theta_pt) / 20) + halfWidth;
				int y_pt = (int)(-rho_pt * cos(theta_pt) / 20) + halfHeight;

				// 直线方程
				// r = x * cos + y * sin
				if ((x_pt >= 0 && x_pt < LidarImageWidth) &&
					(y_pt >= 0 && y_pt < LidarImageHeight))		// 在不在图内
				{
					sum += fabs(x_pt * a + y_pt * b - rho);		// 求和
					pt_num++;									// 增加有效点数量
				}
			}
		}

		// 求平均
		if (pt_num != 0)
			avg[j] = sum / pt_num;
	}

	// 寻找方差最小的有效点
	min_pos = 0;
	for (j = 0; j < lines.size(); j++)
	{
		if (avg[min_pos] > avg[j])
			min_pos = j;
	}
	// 求出最小值后则得到方差最小的直线数据
	float rho = lines[min_pos][0], theta = lines[min_pos][1];
	OLines[i].Theta = theta;
	OLines[i].Rho   = rho;

	}
	
	// 尝试删除多余的
	merge_LineGroup();

	//draw(dst, OLines);			// 点图
	draw_lines(dst, OLines);		// 拟合的直线图

	Img_Lines_Last = Img_Lines.clone();								// 迭代
	Img_Lines = dst.clone();
	threshold(Img_Lines, Img_Lines, 0.1f, 255, THRESH_BINARY);		// 二值化一下，匹配度会高非常多
	cvtColor(Img_Lines, Img_Lines, CV_BGR2GRAY);					// 尝试：后面用新的代码颜色分组，尝试解决这个问题，就不必二值化了

	return SUCCESS;

}// int __obstacle_group::calc_Lines()


int __obstacle_group::surf()
{
	//【2】使用SURF算子检测关键点
	int minHessian  = 5500;					// 700	//SURF算法中的hessian阈值  
	SurfFeatureDetector detector(minHessian);//定义一个SurfFeatureDetector（SURF） 特征检测类对象    
	std::vector<KeyPoint> keyPoint1, keyPoints2;//vector模板类，存放任意类型的动态数组  

												//【3】调用detect函数检测出SURF特征关键点，保存在vector容器中  
	detector.detect(Img_Lines, keyPoint1 );
	detector.detect(Img_Lines_Last, keyPoints2 );

	//【4】计算描述符（特征向量）  
	SurfDescriptorExtractor extractor;
	Mat descriptors1, descriptors2;
	extractor.compute(Img_Lines, keyPoint1, descriptors1 );
	extractor.compute(Img_Lines_Last, keyPoints2, descriptors2 );

	if (keyPoint1.size() == 0 || keyPoints2.size() == 0)
		return FAILED;

	//【5】使用BruteForce进行匹配  
	// 实例化一个匹配器  
	BruteForceMatcher< L2<float> > matcher;
	std::vector< DMatch > matches;
	//匹配两幅图中的描述子（descriptors）  
	matcher.match(descriptors1, descriptors2, matches );

	//【6】绘制从两个图像中匹配出的关键点  
	Mat imgMatches;
	drawMatches(Img_Lines, keyPoint1, Img_Lines_Last, keyPoints2, matches, imgMatches );//进行绘制  

	double min_dist = 100, max_dist = 0;
	for (int i = 0; i < descriptors1.rows; i++)
	{
		double dist = matches[i].distance;
		if (dist < min_dist) min_dist = dist;
		if (dist > min_dist) max_dist = dist;
	}
	printf("MAX: %f\n", max_dist);
	printf("MIN: %f\n", min_dist);

	//【7】显示效果图  
	imshow("匹配图", imgMatches );

	return SUCCESS;

}// int __obstacle_group::surf()

int __obstacle_group::merge_LineGroup()
{
	// 将相似的线组进行合并
	float theta_threshold = 0.55f;		// 0.15f
	float rho_threshold = 25.0f;		// 10.0f		// LidarImageWidth / 60

	int i, j, k;
	vector<bool> is_selected;				// 对于整轮选中的
	vector<__obstacle_line> temp;

	is_selected.resize(OLines.size(), false);
	temp.clear();

	for (i = 0; i < OLines.size(); i++)
	{
		for (j = i + 1; j < OLines.size(); j++)
		{
			if (is_selected[j] == false)
			{
				if ((fabs(OLines[i].Theta - OLines[j].Theta) <= theta_threshold) &&
					(fabs(OLines[i].Rho - OLines[j].Rho) <= rho_threshold))
				{
					// 这两个值近似则可以认为是同一条直线
					is_selected[j] = true;

					for (k = 0; k < ANGLE_ALL; k++)
					{
						if (OLines[j].Position[k] != 0.0f &&
							OLines[i].Position[k] == 0.0f)
							OLines[i].Position[k] = OLines[j].Position[k];
					}

				}
			}
		}

		if (is_selected[i] == false)
			temp.push_back(OLines[i]);

	}

	OLines.clear();
	for (i = 0; i < temp.size(); i++)
	{
		OLines.push_back(temp[i]);
	}

	return SUCCESS;

}// int __obstacle_group::merge_LineGroup()