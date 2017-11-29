#include <position.h>

// static float
float PlainMap_Scale = (float)LidarImageSize / (Rplidar_Max_Range * 2);		// 图长 / 测量直径 = 比例尺

__positioning::__positioning()
{
	// 地图初始化
	Mat zero(PlainMap_Heigth, PlainMap_Width, CV_8UC3, Scalar(0, 0, 0));		// 图片格式： BGR
	Map = zero.clone();
	zero.release();

	// 设定初始位置
	//Pos.X = Pos.Y = PlainMap_Size * PlainMap_Scale * 0.25f;
	Pos.X = Pos.Y = 2500.0f;
	Pos.Yaw = 0.0f;

	Pos_Last =  Pos_Last_2 = StartPos = Pos;		// 设定初始位置和初始指向

	Line_Group.clear();
	Line_GLast.clear();

	OB_All.clear();

	rV.clear();


}// __positioning::__positioning()


__positioning::~__positioning()
{


}// __positioning::~__positioning()


int __positioning::update_LineGroup(vector<__obstacle_line> ob)
{
	int i = 0;

	// 迭代，保存上一次的数据
	Line_GLast.clear();
	for (i = 0; i < Line_Group.size(); i++)
	{
		Line_GLast.push_back(Line_Group[i]);
	}

	// 存储这一个数据
	Line_Group.clear();
	for (i = 0; i < ob.size(); i++)
	{
		Line_Group.push_back(ob[i]);
	}

	return ob.size();

}// int __positioning::update_LineGroup(vector<__obstacle_line> ob)

int __positioning::calc_Grid_Velocity(float vx, float vy, float freq)
{
	// 整个模块的主要功能实现函数
	if (!freq) return FAILED;

	Freq = freq;

	calc_GridGroup();

	calc_Velocity(Freq);			// 计算速度
	V_Complementary_Filter(vx, vy);

	calc_Grid(Freq);				// 计算坐标
	//G_Kalman_Filter();			// 因为两组数据都是减帧法得到的，所以估计这里的卡尔曼用处不大

	fuse_Obstacle();				// 做建图处理

	draw_Map();						// 画出来整张图

	return SUCCESS;


}// int __positioning::calc_Grid_Velocity(float vx, float vy)

int __positioning::calc_GridGroup()
{
	int i, j;
	int pt_num;
	float avg_x, avg_y;

	float theta, rho, x, y;

	// 首先备份上一次的坐标数据
	rGrid_Last.clear();
	for (i = 0; i < rGrid.size(); i++)
	{
		rGrid_Last.push_back(rGrid[i]);
	}

	// 然后开始计算这一次的坐标
	rGrid.clear();
	avg_x = avg_y = 0.0f;
	for (i = 0; i < Line_Group.size(); i++)
	{
		pt_num = 0;
		for (j = 0; j < ANGLE_ALL; j++)
		{
			if (Line_Group[i].Position[j] != 0.0f)
			{
				theta = j * PI / 180;
				rho = Line_Group[i].Position[j];

				x = (int)(rho  * sin(theta) * PlainMap_Scale);
				y = (int)(-rho * cos(theta) * PlainMap_Scale);


				avg_x += x;
				avg_y += y;
				pt_num++;

			}
		}
		avg_x /= pt_num;
		avg_y /= pt_num;

		__pos temp;
		temp.X = avg_x;
		temp.Y = avg_y;
		temp.Yaw = atan2(x, y) * 180.0f / PI;

		rGrid.push_back(temp);
	}

	return SUCCESS;

}// int __positioning::calc_GridGroup()

int __positioning::calc_Grid(float freq)
{
	// 计算坐标
	Pos.X += V.X * freq;
	Pos.Y += V.Y * freq;

	return SUCCESS;

}// int __positioning::calc_Grid(float freq)

int __positioning::fuse_Obstacle()
{
	// 障碍物融合
	// 首先匹配障碍物，和已有障碍物进行比对，如果不存在则添加进数据库
	int i, j, k;
	bool is_matched[GROUP_MAX] = { false };

	int d_pt_eff_threshold = ANGLE_ALL / 20;		// 有效点数量差值
	float d_theta_threshold = 0.05f;				// 角度差值阈值
	float d_rho_threshold = 15.0f;				// 距离差值阈值

	__obstacle_map      temp;
	//__obstacle_line     line_t;
	//__obstacle_circle circle_t;

	// 首先是线障碍物
	temp.is_Line = true;
	for (i = 0; i < Line_Group.size(); i++)
	{
		for (j = 0; j < OB_All.size(); j++)
		{
			// 检测匹配
			int pt_eff, pt_eff_last;
			int d_pt_eff;		// 有效点数量
			float d_theta;		// 角度差值
			float d_rho;		// 距离差值

								// 计算有效点个数
			pt_eff = 0;
			for (k = 0; k < ANGLE_ALL; k++)
				if (Line_Group[i].Position[k] != 0.0f)
					pt_eff++;

			pt_eff_last = 0;
			for (k = 0; k < ANGLE_ALL; k++)
				if (OB_All[j].l.Position[k] != 0.0f)
					pt_eff_last++;

			d_pt_eff = abs(pt_eff - pt_eff_last);
			d_theta = fabs(Line_Group[i].Theta - OB_All[j].l.Theta);
			d_rho = fabs(Line_Group[i].Rho - OB_All[j].l.Rho);

			// 差值小于阈值则认为有效
			if (d_pt_eff <= d_pt_eff_threshold &&
				d_theta <= d_theta_threshold  &&
				d_rho <= d_rho_threshold)
			{
				// 如果认为是同一组则开始计算

				is_matched[i] = true;
				break;
			}

		}

		// 如果没有匹配项则认为这个障碍物是在这一轮新发现的
		if (is_matched[i] == false)
		{
			temp.l = Line_Group[i];
			temp.Center = Pos;

			OB_All.push_back(temp);
		}
	}

	return SUCCESS;
}// int __positioning::fuse_Obstacle()

int __positioning::calc_Velocity(float freq)
{
	// 通过匹配障碍物来计算
	// 直接对特征进行判断,特征值：点的数目，直线的theta和rho

	// 阈值项
	int d_pt_eff_threshold = ANGLE_ALL / 20;		// 有效点数量差值
	float d_theta_threshold = 0.05f;				// 角度差值阈值
	float d_rho_threshold   = 15.0f;				// 距离差值阈值

	int i, j, k;
	bool is_matched[GROUP_MAX] = { false };


	//if (Line_Group.size() != rGrid.size() ||
	//	Line_GLast.size() != rGrid_Last.size())
	//	return FAILED;

	rV.clear();
	for (i = 0; i < Line_Group.size(); i++)
	{
		for (j = 0; j < Line_GLast.size(); j++)		// Line_Group的大小和坐标的大小是一一对应的
		{
			if (is_matched[j] == false)
			{
				// 在这里进行匹配
				// 因为高速采样的条件下，每次载具前进和转向的角度都是有限的，利用这个特性我们进行匹配
				int pt_eff, pt_eff_last;
				int d_pt_eff;		// 有效点数量
				float d_theta;		// 角度差值
				float d_rho;		// 距离差值

				// 计算有效点个数
				pt_eff = 0;
				for (k = 0; k < ANGLE_ALL; k++)
				{
					if (Line_Group[i].Position[k] != 0.0f)
						pt_eff++;
				}

				pt_eff_last = 0;
				for (k = 0; k < ANGLE_ALL; k++)
				{
					if (Line_GLast[j].Position[k] != 0.0f)
						pt_eff_last++;
				}

				d_pt_eff = abs(pt_eff - pt_eff_last);
				d_theta  = fabs(Line_Group[i].Theta - Line_GLast[j].Theta);
				d_rho    = fabs(Line_Group[i].Rho   - Line_GLast[j].Rho);


				// 差值小于阈值则认为有效
				if (d_pt_eff <= d_pt_eff_threshold &&
					d_theta  <= d_theta_threshold  &&
					d_rho    <= d_rho_threshold)
				{
					// 如果认为是同一组则开始计算

					__v temp;

					temp.X = rGrid[i].X - rGrid_Last[j].X;
					temp.Y = rGrid[i].Y - rGrid_Last[j].Y;
					temp.X *= freq;
					temp.Y *= freq;

					temp.Val = sqrt(temp.X * temp.X + temp.Y * temp.Y);	// 计算模值

					temp.Yaw = rGrid[i].Yaw - rGrid_Last[j].Yaw;		// 尝试计算角速度
					temp.Yaw *= freq;

					rV.push_back(temp);

					is_matched[j] = true;
					break;
				}
			}
		}

	}

	// 求平均
	if (rV.size() == 0)
	{
		V.X = V.Y = 0.0f;
		return FAILED;
	}

	for (i = 0; i < rV.size(); i++)
	{
		V.X += rV[i].X;
		V.Y += rV[i].Y;
		V.Val += rV[i].Val;

		V.Yaw += rV[i].Yaw;
	}
	V.X /= rV.size();
	V.Y /= rV.size();
	V.Val /= rV.size();
	V.Yaw /= rV.size();

	// 速度是物体相对于载具的，所以要取负值，模值没有负的，偏航角两个取负值算出来的还是一样
	V.X = -V.X;
	V.Y = -V.Y;


	return SUCCESS;

}// int __positioning::calc_Velocity()

int __positioning::V_Complementary_Filter(float vx, float vy)
{
	// 互补滤波器
	// 这个阈值要调的
	int i;
	float threshold_x = 0.667f;
	float threshold_y = 0.333f;

	V.X = threshold_x * V.X + (1 - threshold_x) * vx;
	V.Y = threshold_y * V.Y + (1 - threshold_y) * vy;
	V.Val = sqrt(V.X * V.X + V.Y * V.Y);

	V.Yaw = atan2(V.X, V.Y) * 180.0f / PI;

	return SUCCESS;

}// int V_Complementary_Filter(float vx, float vy)

int __positioning::G_Kalman_Filter()
{
	// 位置卡尔曼滤波器
	// QPos_Dst和RPos_Dst这两个东西的阈值也是要调的

	// x方向
	gk_x.X_last = gk_x.X;
	gk_x.P_last = gk_x.P;

	gk_x.X_mid = gk_x.X_last + V.X + QPos_Dst;

	gk_x.K = gk_x.P_mid / (gk_x.P_mid + RPos_Dst);
	gk_x.X = gk_x.X_mid + gk_x.K * (Pos.X - gk_x.X_mid);
	gk_x.P = (1 - gk_x.K) * gk_x.P_mid;

	// y方向
	gk_y.X_last = gk_y.X;
	gk_y.P_last = gk_y.P;

	gk_y.X_mid = gk_y.X_last + V.Y + QPos_Dst;

	gk_y.K = gk_y.P_mid / (gk_y.P_mid + RPos_Dst);
	gk_y.X = gk_y.X_mid + gk_y.K * (Pos.Y - gk_y.X_mid);
	gk_y.P = (1 - gk_y.K) * gk_y.P_mid;

	return SUCCESS;

}// int __positioning::G_Kalman_Filter()

int __positioning::draw_Map()
{

	int i, j;
	float theta;
	float rho;
	int x, y;

	// 作线
	for (i = 0; i < OB_All.size(); i++)
	{
		if (OB_All[i].is_Line == true)
		for (j = 0; j < ANGLE_ALL; j++)	// scan_data.size()
		{
			theta = j * PI / 180;
			rho = OB_All[i].l.Position[j];

			if (rho <= Rplidar_Max_Range / 200.0f)
				continue;

			x = (int)(rho  * sin(theta) / 20) + OB_All[i].Center.X * PlainMap_Scale;
			y = (int)(-rho * cos(theta) / 20) + OB_All[i].Center.Y * PlainMap_Scale;

			if ((x >= 0 && x < LidarImageWidth) &&
				(y >= 0 && y < LidarImageHeight))		// 检查这些点在不在图内
			{
				circle(Map, Point(x, y), 1, Scalar(255, 255, 255), -1, 8, 0);
			}
		}
	}

	imshow("SLAM_Result", Map);

	return SUCCESS;

}// int __positioning::draw_Map()
