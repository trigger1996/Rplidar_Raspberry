#include <z_lidar.h>

// static float
float PlainMap_Scale = (float)LidarImageSize / (Rplidar_Max_Range * 2);		// ͼ�� / ����ֱ�� = ������


int __z_lidar::calc_Lidar(float freq)
{
	// ����ģ�����Ҫ����ʵ�ֺ���
	if (!freq) return FAILED;

	Freq = freq;

	//calc_GridGroup();

	fuse_Obstacle();				// ����ͼ����

	calc_OB_Centres_Measured();		//  ���������ϰ���Ĵ���㣬��һ�������һ���ϰ������ڶ�λ

	draw_Map();						// ����������ͼ

	return SUCCESS;


}// int __z_lidar::calc_Lidar(float freq)

__z_lidar::__z_lidar()
{
	// ��ͼ��ʼ��
	Mat zero(PlainMap_Heigth, PlainMap_Width, CV_8UC3, Scalar(0, 0, 0));		// ͼƬ��ʽ�� BGR
	Map = zero.clone();
	zero.release();

	// �趨��ʼλ��
	//Pos.X = Pos.Y = PlainMap_Size * PlainMap_Scale * 0.25f;
	Pos.X = Pos.Y = 2500.0f;
	Pos.Yaw = 0.0f;

	Pos_Last =  Pos_Last_2 = StartPos = Pos;		// �趨��ʼλ�úͳ�ʼָ��

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

	// ������������һ�ε�����
	Line_GLast.clear();
	for (i = 0; i < Line_Group.size(); i++)
	{
		Line_GLast.push_back(Line_Group[i]);
	}

	// �洢��һ������
	Line_Group.clear();
	for (i = 0; i < ob.size(); i++)
	{
		Line_Group.push_back(ob[i]);
	}

	return ob.size();

}// int __z_lidar::update_LineGroup(vector<__obstacle_line> ob)

int __z_lidar::fuse_Obstacle()
{
	// �ϰ����ں�
	// ����ƥ���ϰ���������ϰ�����бȶԣ���������������ӽ����ݿ�
	int i, j, k;
	bool is_matched[GROUP_MAX] = { false };

	const int d_pt_eff_threshold = ANGLE_ALL / 15;		// ��Ч��������ֵ	// ANGLE_ALL / 20
	const float d_theta_threshold = 0.05f;				// �ǶȲ�ֵ��ֵ	// 0.05f
	const float d_rho_threshold = 15.0f;				// �����ֵ��ֵ	// 15.0f

	__obstacle_map      temp;
	//__obstacle_line     line_t;
	//__obstacle_circle circle_t;

	// �Ȱ����в�����ϰ�����
	for (i = 0; i < OB_All.size(); i++)
		OB_All[i].is_New = OB_All[i].is_Updated = false;

	// ���������ϰ���
	temp.is_Line = true;
	for (i = 0; i < Line_Group.size(); i++)
	{
		for (j = 0; j < OB_All.size(); j++)
		{
			// ���ƥ��
			int pt_eff, pt_eff_last;
			int d_pt_eff;		// ��Ч������
			float d_theta;		// �ǶȲ�ֵ
			float d_rho;		// �����ֵ

								// ������Ч�����
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

			// ��ֵС����ֵ����Ϊ��Ч
			if (d_pt_eff <= d_pt_eff_threshold &&
				d_theta <= d_theta_threshold  &&
				d_rho <= d_rho_threshold)
			{
				// �����Ϊ��ͬһ����ʼ����
				OB_All[j].is_New = false;
				OB_All[j].is_Updated = true;

				is_matched[i] = true;
				break;
			}

		}

		// ���û��ƥ��������Ϊ����ϰ���������һ���·��ֵ�
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

	// ���ȱ�����һ�ε���������
	rGrid_Last.clear();
	for (i = 0; i < rGrid.size(); i++)
	{
		rGrid_Last.push_back(rGrid[i]);
	}

	// Ȼ��ʼ������һ�ε�����
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
				// ֱ�߿���ֱ�Ӽӣ����ξ�Ҫ�Լ�������


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

	// ���ȱ�����һ�ε���������
	rGrid_Last.clear();
	for (i = 0; i < rGrid.size(); i++)
	{
		rGrid_Last.push_back(rGrid[i]);
	}

	// Ȼ��ʼ������һ�ε�����
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
					// ֱ�߿���ֱ�Ӽӣ����ξ�Ҫ�Լ�������


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

int __z_lidar::draw_Map()
{

	int i, j;
	float theta;
	float rho;
	int x, y;

	// ����
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
				(y >= 0 && y < LidarImageHeight))		// �����Щ���ڲ���ͼ��
			{
				circle(Map, Point(x, y), 1, Scalar(255, 255, 255), -1, 8, 0);
			}
		}
	}

	imshow("SLAM_Result", Map);

	return SUCCESS;

}// int __z_lidar::draw_Map()

