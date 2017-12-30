#include "__header/cv_match.h"

int __cv_match::run()
{
	draw(false);										// 作图

	do_HoughTransformP(false, false);					// 用霍夫变换计算出图中的直线，关于其他形状的障碍物不予理会，因为理论上其实只要有一个基准的障碍物就能完成所有的计算

	merge_LineGroup();									// 剔除多余的噪声点

	show_lines(false);									// 画出结果


	// 注意，这边的匹配其实不能交给EKF以外的模块来完成
	// 这是因为即使y = k * x + b中，斜率k锁死了，b会随着运动而变换，这个b和当前坐标有关
	// 即使可以假设匀速运动，或者坐标每次的变化量都很小，但是时间长以后这么假设进行匹配，必然造成失真和数据爆炸

	// 所以这边放弃匹配，将数据打包后传给EKF
	Point pt1, pt2;

	const int halfWidth = Img_Raw.cols / 2;
	const int halfHeight = Img_Raw.rows / 2;
	const Point core(halfWidth, halfHeight);

	Data_Out.clear();
	for (int i = 0; i < lines_out.size(); i++)
	{
		__point2p temp;

		pt1.x = lines[i][0], pt1.y = lines[i][1];
		pt2.x = lines[i][2], pt2.y = lines[i][3];

		// 计算距离，距离好算，直接点到直线距离即可
		temp.r = calc_Distance_Pt2Line(pt1, pt2, core) * 20.0f;		// 图和实际距离的比例尺为20

		// 计算角度，角度复杂一些，角度可以认为是直线中点和图中点的连线的斜率
		// 先算出中点
		Point midpt;
		midpt.x = (pt1.x + pt2.x) / 2;
		midpt.y = (pt1.y + pt2.y) / 2;

		int x_in = midpt.x - core.x;
		int y_in = core.y - midpt.y;		// 注意这边的正负
		temp.deg = tan_yaw(x_in, y_in);

		Data_Out.push_back(temp);
	}

	return SUCCESS;

}// int __cv_match::run()

vector<__point2p> __cv_match::get_DataArray()
{

	return Data_Out;

}// vector<__point2p> __cv_match::get_DataArray()

__cv_match::__cv_match()
{


}// __cv_match::__cv_match()

__cv_match::~__cv_match()
{


}// __cv_match::~__cv_match()

int __cv_match::set_Data(float raw[])
{
	int i;

	for (i = 0; i < ANGLE_ALL; i++)
	{
		Data_Raw[i] = raw[i];
	}

	return SUCCESS;

}// int __cv_match::get_Data(float raw[])

int __cv_match::do_HoughTransformP(bool is_show, bool is_show_raw_data)
{
	Mat midImage;
	Mat dstImage(LidarImageHeight, LidarImageWidth, CV_8UC3, Scalar(0, 0, 0));	//临时变量和目标图的定义

	// 进行边缘检测和转化为灰度图
	Canny(Img_Raw, midImage, 50, 200, 3);										//进行一此canny边缘检测
	if (is_show && is_show_raw_data)
		cvtColor(midImage, dstImage, CV_GRAY2BGR);								//转化边缘检测后的图为灰度图

	// 进行霍夫线变换
	HoughLinesP(midImage, lines, 0.5, CV_PI / 180, 25, 25, 10);					// midImage, lines, 1, CV_PI / 180, 80, 50, 10

	if (is_show)
	{
		// 依次在图中绘制出每条线段
		for (size_t i = 0; i < lines.size(); i++)
		{
			Vec4i l = lines[i];
			line(dstImage, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(186, 88, 255), 1, CV_AA);
		}
		imshow("Raw", Img_Raw);
		imshow("Canny", midImage);
		imshow("Result", dstImage);
	}

	return SUCCESS;

}// int __cv_match::do_HoughTransformP(bool is_show, bool is_show_raw_data)

int __cv_match::merge_LineGroup()
{
	// 匹配相似的组
	int i, j;
	vector<bool> is_selected;					// 对于整轮选中的的标识
	float theta, rho;
	float theta_ref, rho_ref;
	Point pt1, pt2, pt_ref1, pt_ref2;

	// 增加四个东西
	// 增加这个是为了解决arctan在垂直角度情况下表现很差的办法，把这两类分出来判断
	bool is_level_ref, is_level;
	bool is_vertical_ref, is_vertical;

	// 阈值
	const int   vertical_level_threshold = 2;					// Vec4i表示两点式，如果x2 - x1小于阈值，则可认为是垂直的，如果y1-y2小于阈值则可认为是水平的
	const float theta_threshold = 27.5f;		// 0.45f
	const float rho_threshold   = 17.5f;		// 35.0f		// LidarImageWidth / 60
	const int halfWidth = Img_Raw.cols / 2;
	const int halfHeight = Img_Raw.rows / 2;
	const Point core(halfWidth, halfHeight);

	is_selected.resize(lines.size(), false);
	lines_out.clear();

	for (i = 0; i < lines.size(); i++)
	{
		pt_ref1.x = lines[i][0], pt_ref1.y = lines[i][1];
		pt_ref2.x = lines[i][2], pt_ref2.y = lines[i][3];

		//theta_ref = (lines[i][3] - lines[i][1]) / (lines[i][2] - lines[i][0]);
		is_level_ref = is_vertical_ref = false;
		if (lines[i][2] - lines[i][0] < vertical_level_threshold)		// x
		{
			is_vertical_ref = true;
			theta_ref = 90.0f;
		}
		else if (lines[i][3] - lines[i][1] < vertical_level_threshold)	// y
		{
			is_level_ref = true;
			theta_ref = 0.0f;
		}
		else
			theta_ref = atan2(lines[i][3] - lines[i][1], lines[i][2] - lines[i][0]) * 180.0f / CV_PI;

		rho_ref = calc_Distance_Pt2Line(pt_ref1, pt_ref2, core);

		for (j = i + 1; j < lines.size(); j++)
		{
			// y = k * x + b, theta->k, rho->b
			theta = 0.0f;
			rho   = 0.0f;
			if (is_selected[j] == false)
			{
				pt1.x = lines[j][0], pt1.y = lines[j][1];
				pt2.x = lines[j][2], pt2.y = lines[j][3];

				// 这边先判断是不是垂直和水平，再计算斜率
				//theta = (lines[j][3] - lines[j][1]) / (lines[j][2] - lines[j][0]);
				is_level = is_vertical = false;
				if (lines[j][2] - lines[j][0] < vertical_level_threshold)		// x
				{
					is_vertical = true;
					theta = 90.0f;
				}
				else if (lines[j][3] - lines[j][1] < vertical_level_threshold)	// y
				{
					is_level = true;
					theta = 0.0f;
				}
				else
					theta = atan2(lines[j][3] - lines[j][1], lines[j][2] - lines[j][0]) * 180.0f / CV_PI;

				rho   = calc_Distance_Pt2Line(pt1, pt2, core);

				// 如果满足阈值
				if ((is_level == true && is_level_ref == true) &&
					fabs(rho - rho_ref) <= rho_threshold)
					is_selected[j] = true;
				else if ((is_vertical == true && is_vertical_ref == true) &&
						 fabs(rho - rho_ref) <= rho_threshold)
					is_selected[j] = true;
				else if (fabs(theta - theta_ref) <= theta_threshold &&
						 fabs(rho   - rho_ref)   <= rho_threshold)
					is_selected[j] = true;


			}
		}
	}

		for (i = 0; i < lines.size(); i++)
		{
			if (is_selected[i] == false)
				lines_out.push_back(lines[i]);
		}

		return lines_out.size();

}// int __cv_match::merge_LineGroup()

int __cv_match::draw(bool is_show)
{
	Mat zero(LidarImageHeight, LidarImageWidth, CV_8UC3, Scalar(0, 0, 0));		// 图片格式： BGR
	Img_Raw = zero.clone();
	zero.release();

	//在中心加上一个圆心
	circle(Img_Raw, Point(Img_Raw.cols / 2, Img_Raw.rows / 2), 3, Scalar(255, 0, 0), -1, 8, 0);

	int x, y;
	double theta, rho;
	const int halfWidth = Img_Raw.cols / 2;
	const int halfHeight = Img_Raw.rows / 2;

	for (unsigned int i = 0; i < ANGLE_ALL; i++)	// scan_data.size()
	{

		theta = i * PI / 180;
		rho = Data_Raw[i];

		x = (int)(rho  * sin(theta) / 20) + halfWidth;
		y = (int)(-rho * cos(theta) / 20) + halfHeight;

		circle(Img_Raw, Point(x, y), 2, Scalar(255, 255, 255), -1, 8, 0);

	}

	if (is_show)
		imshow("Img_Raw", Img_Raw);

	return SUCCESS;

}// int __cv_match::draw()

int __cv_match::show_lines(bool is_show_raw_data)
{
	// 依次在图中绘制出每条线段
	Mat zero(LidarImageHeight, LidarImageWidth, CV_8UC3, Scalar(0, 0, 0));		// 图片格式： BGR
	Mat dst = zero.clone();

	if (is_show_raw_data)
	{
		Canny(Img_Raw, dst, 50, 200, 3);									//进行一此canny边缘检测
		cvtColor(dst, dst, CV_GRAY2BGR);									//转化边缘检测后的图为灰度图
	}

	for (size_t i = 0; i < lines_out.size(); i++)
	{
		Vec4i l = lines_out[i];
		line(dst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(186, 88, 255), 1, CV_AA);
	}
	imshow("Result_OverAll", dst);

	return SUCCESS;

}// int __cv_match::draw_lines(bool is_show, bool is_show_raw_data)


double calc_Distance_Pt2Line(Point pt1, Point pt2, Point core)
{
	// 根据中心点与直线的距离 排除干扰直线
	// 点(x0,y0)到直线Ax+By+C=0的距离为d = (A*x0+B*y0+C)/sqrt(A^2+B^2)
	double A, B, C, dst;
	// 化简两点式为一般式
	// 两点式公式为(y - y1)/(x - x1) = (y2 - y1)/ (x2 - x1)
	// 化简为一般式为(y2 - y1)x + (x1 - x2)y + (x2y1 - x1y2) = 0
	// A = y2 - y1
	// B = x1 - x2
	// C = x2y1 - x1y2
	A = pt2.y - pt1.y;
	B = pt1.x - pt2.x;
	C = pt2.x * pt1.y - pt1.x * pt2.y;
	//中心点坐标(coreX,coreY)
	double coreX, coreY;
	coreX = core.x;
	coreY = core.y;
	// 距离公式为d = |A*x0 + B*y0 + C|/√(A^2 + B^2)
	dst = abs(A * coreX + B * coreY + C) / sqrt(A * A + B * B);
	//=========================================================================================

	return dst;
}

double calc_Distance_Pt2Pt(Point pt1, Point pt2)
{
	double dst;
	double x1, y1, x2, y2;

	x1 = pt1.x, y1 = pt1.y;
	x2 = pt2.x, y2 = pt2.y;

	dst = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));


	return dst;

}// double calc_Distance_Pt2Pt(Point pt1, Point pt2)

double tan_yaw(double x, double y)
{
	// 这个是给图像用的tan，注意这边x是对边，y是邻边
	// x是图像的x轴，y是图像的y轴
	// 返回角度值

	double ret;
	const double vertical_level_threshold = 2;

	if (fabs(x) <= vertical_level_threshold)	// 如果x差不多的话说明角度在0度和180这样
	{
		if (y < 0)
			ret = 0;
		else
			ret = 180;
	}
	if (fabs(y) <= vertical_level_threshold)	//  如果y差不多的话说明角度在90度和270这样
	{
		if (x < 0)
			ret = 270;
		else
			ret = 0;
	}
	else
	{
		ret = atan2(fabs(x), fabs(y)) * 180.0f / CV_PI;		// 全部取绝对值来计算

		// 归一化至0~360度
		if (x > 0 && y > 0)
			ret = ret;						// 第一象限
		else if (x < 0 && y > 0)
			ret = ret += 270;				// 第二象限
		else if (x < 0 && y < 0)
			ret = ret + 180;				// 第三象限
		else if (x > 0 && y < 0)
			ret = ret + 90;					// 第四象限
	}

	return ret;

}// double tan_yaw(double y, double x)



