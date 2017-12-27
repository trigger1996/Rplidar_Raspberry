#include <z_lidar.h>

// static float
float PlainMap_Scale = (float)LidarImageSize / (Rplidar_Max_Range * 2);		// 图长 / 测量直径 = 比例尺


int __z_lidar::calc_Lidar(float freq)
{
	// 整个模块的主要功能实现函数
	if (!freq) return FAILED;

	Freq = freq;

	//calc_GridGroup();

	fuse_Obstacle();				// 做建图处理

	calc_OB_Centres_Measured();		//  计算所有障碍物的代替点，用一个点代替一组障碍物用于定位

	//draw_Map(true);						// 画出来整张图

	return SUCCESS;


}// int __z_lidar::calc_Lidar(float freq)

__z_lidar::__z_lidar()
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



}// __z_lidar::__z_lidar()


__z_lidar::~__z_lidar()
{


}// __z_lidar::~__z_lidar()


int __z_lidar::update_LineGroup(vector<__obstacle_line> ob)
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

}// int __z_lidar::update_LineGroup(vector<__obstacle_line> ob)

int __z_lidar::fuse_Obstacle()
{
	// 障碍物融合
	// 首先匹配障碍物，和已有障碍物进行比对，如果不存在则添加进数据库
	int i, j, k;
	bool is_matched[GROUP_MAX] = { false };

	const int d_pt_eff_threshold = ANGLE_ALL / 10;		// 有效点数量差值	// ANGLE_ALL / 20
	const float d_theta_threshold = 0.45f;				// 角度差值阈值	// 0.05f
	const float d_rho_threshold = 25.0f;				// 距离差值阈值	// 15.0f

	__obstacle_map      temp;
	//__obstacle_line     line_t;
	//__obstacle_circle circle_t;

	// 先把所有测过的障碍物变旧
	for (i = 0; i < OB_All.size(); i++)
		OB_All[i].is_New = OB_All[i].is_Updated = false;

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
				OB_All[j].is_New = false;
				OB_All[j].is_Updated = true;

				is_matched[i] = true;
				break;
			}

		}

		// 如果没有匹配项则认为这个障碍物是在这一轮新发现的
		if (is_matched[i] == false)
		{
			temp.l = Line_Group[i];
			temp.Center = Pos;
			temp.is_New = true;
			temp.is_Updated = false;

			OB_All.push_back(temp);
		}
	}


	//cout << "ZLidar Size: " << OB_All.size() << endl;
	return SUCCESS;

}// int __z_lidar::fuse_Obstacle()

int __z_lidar::calc_OB_Centres_All()
{
	int i, j;
	int pt_num;
	float avg_theta, avg_rho;

	// 首先备份上一次的坐标数据
	rGrid_Last.clear();
	for (i = 0; i < rGrid.size(); i++)
	{
		rGrid_Last.push_back(rGrid[i]);
	}

	// 然后开始计算这一次的坐标
	rGrid.clear();
	for (i = 0; i < OB_All.size(); i++)
	{
		__point2p temp;
		pt_num = 0;
		avg_theta = avg_rho = 0.0f;
		for (j = 0; j < ANGLE_ALL; j++)
		{
			if (OB_All[i].is_Line == true)
			{
				if (OB_All[i].l.Position[j] != 0.0f)
				{
					avg_theta += j;
					avg_rho   += OB_All[i].l.Position[j];
					pt_num++;

				}


			}
			else if (OB_All[i].is_Circle == true)
			{
				// 直线可以直接加，弧形就要自己补偿了


			}

		}

		avg_theta /= (float)pt_num;
		avg_rho   /= (float)pt_num;

		temp.r   = avg_rho;
		temp.deg = avg_theta;

		//cout << "avg_theta:" << avg_theta << endl;

		rGrid.push_back(temp);

	}

	//cout << "rGrid Size: " << rGrid.size() << endl;
	return SUCCESS;

}// int __z_lidar::calc_OB_Centres_All()

int __z_lidar::calc_OB_Centres_Measured()
{

	int i, j;
	int pt_num;
	float avg_theta, avg_rho;

	// 首先备份上一次的坐标数据
	rGrid_Last.clear();
	for (i = 0; i < rGrid.size(); i++)
	{
		rGrid_Last.push_back(rGrid[i]);
	}

	// 然后开始计算这一次的坐标
	rGrid.clear();
	for (i = 0; i < OB_All.size(); i++)
	{
		__point2p temp;
		pt_num = 0;
		avg_theta = avg_rho = 0.0f;

		if (OB_All[i].is_New == true || OB_All[i].is_Updated == true)
		{
			for (j = 0; j < ANGLE_ALL; j++)
			{
				if (OB_All[i].is_Line == true)
				{
					if (OB_All[i].l.Position[j] != 0.0f)
					{
						avg_theta += j;
						avg_rho   += OB_All[i].l.Position[j];
						pt_num++;

					}


				}
				else if (OB_All[i].is_Circle == true)
				{
					// 直线可以直接加，弧形就要自己补偿了


				}

			}

			avg_theta /= (float)pt_num;
			avg_rho   /= (float)pt_num;

			temp.r   = avg_rho;
			temp.deg = avg_theta;

			//cout << "avg_theta:" << avg_theta << endl;
		}
		else
		{
			temp.r = temp.deg = 0;
		}

		rGrid.push_back(temp);
	}

	//for (int i = 0; i < rGrid.size(); i++)
	//	cout << "rGrid[i].r: " << rGrid[i].r << "rGrid[i].deg: " << rGrid[i].deg << endl;

	cout << "rGrid Size: " << rGrid.size() << endl;
	return SUCCESS;

}// int __z_lidar::calc_OB_Centres_All()

int __z_lidar::draw_Map(bool is_show)
{
	// 这边不能每次都clear

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

	if (is_show == true)
		imshow("SLAM_Result", Map);

	return SUCCESS;

}// int __z_lidar::draw_Map()

int __z_lidar::update_Position_on_Map(__point2f input, float yaw, bool is_show)
{
	// input, 单位: m

	__pos temp;
	temp.X = StartPos.X + input.X * 1000.0f;	// mm->m
	temp.Y = StartPos.Y + input.Y * 1000.0f;
	temp.Yaw = yaw;

	Pos_Last_2 = Pos_Last;
	Pos_Last   = Pos;
	Pos        = temp;

	int x_map, y_map;

	// 本次位置
	x_map = Pos.X * PlainMap_Scale;
	y_map = Pos.Y * PlainMap_Scale;
	circle(Map, Point(x_map, y_map), 2, Scalar(0, 125, 255), -1, 8, 0);

	// 上次位置
	x_map = Pos_Last.X * PlainMap_Scale;
	y_map = Pos_Last.Y * PlainMap_Scale;
	circle(Map, Point(x_map, y_map), 2, Scalar(255, 125, 0), -1, 8, 0);

	if (is_show == true)
		imshow("SLAM_Result", Map);

	return SUCCESS;

}// int __z_lidar::update_Position_on_Map(bool is_show)

