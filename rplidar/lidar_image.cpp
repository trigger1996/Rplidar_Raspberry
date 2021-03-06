#include <lidar_image.h>

__lidar_img::__lidar_img()
{
	// 构造

}// __lidar_img::__lidar_img()

__lidar_img::~__lidar_img()
{
	// 销毁

}// __lidar_img::~__lidar_img()

 //将扫面的原始数据转化为实际距离和角度，这个可以参考ultra_simple
int __lidar_img::scanData(rplidar_response_measurement_node_t *buffer, size_t count, float frequency)
{

	if (count == 0) return FAILED;

	// 首先备份旧数据
	Data_Last.clear();
	for (int pos = 0; pos < Data.size(); ++pos)
	{
		__scandot dot;
		dot.Quality = Data[pos].Quality;
		dot.Angle = Data[pos].Angle;
		dot.Dst = Data[pos].Dst;

		Data_Last.push_back(dot);
	}


	// 然后写入新数据
	Data.clear();
	for (int pos = 0; pos < (int)count; ++pos) {
		__scandot dot;
		if (!buffer[pos].distance_q2) continue;

		dot.Quality = (buffer[pos].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
		dot.Angle = (buffer[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f;
		dot.Dst = buffer[pos].distance_q2 / 4.0f;
		Data.push_back(dot);
	}

	Scan_Speed = frequency;

	return SUCCESS;
}// int __lidar_img::scanData(rplidar_response_measurement_node_t *buffer, size_t count, float frequency)

//将扫描点映射到画布上

int __lidar_img::Draw(Mat &dst, vector<__scandot> data, char window_name[])
{
	Mat zero(LidarImageHeight, LidarImageWidth, CV_8UC3, Scalar(29, 230, 181));		// 图片格式： BGR
	dst = zero.clone();
	zero.release();

	//在中心加上一个圆心
	circle(dst, Point(dst.cols / 2, dst.rows / 2), 3, Scalar(255, 0, 0), -1, 8, 0);

	int x, y;
	double theta, rho;
	int halfWidth = dst.cols / 2;
	int halfHeight = dst.rows / 2;


	for (unsigned int i = 0; i < data.size(); i++)	// scan_data.size()
	{
		__scandot dot;
		dot = data[i];		// 未滤波:Data[i] 滤波后:data_dst[i]

		theta = dot.Angle * PI / 180;
		rho = dot.Dst;

		x = (int)(rho  * sin(theta) / 20) + halfWidth;
		y = (int)(-rho * cos(theta) / 20) + halfHeight;

		circle(dst, Point(x, y), 4, Scalar(0, 0, 255), -1, 8, 0);

	}

	char s[35];
	sprintf(s, "Value Count: %d, Scan Speed: %0.2f", Data.size(), Scan_Speed);
	putText(dst, s, Point(50, 50), 4, .5, Scalar(0, 0, 0), 2);

	imshow(window_name, dst);

	return SUCCESS;

}// int __lidar_img::Draw(Mat dst, char window_name[])

int __lidar_img::Draw(Mat &dst, float data[], char window_name[])
{
	Mat zero(LidarImageHeight, LidarImageWidth, CV_8UC3, Scalar(29, 230, 181));		// 图片格式： BGR
	dst = zero.clone();
	zero.release();

	//在中心加上一个圆心
	circle(dst, Point(dst.cols / 2, dst.rows / 2), 3, Scalar(255, 0, 0), -1, 8, 0);

	int x, y;
	double theta, rho;
	int halfWidth = dst.cols / 2;
	int halfHeight = dst.rows / 2;


	for (unsigned int i = 0; i < ANGLE_ALL; i++)	// scan_data.size()
	{

		theta = i * PI / 180;
		rho = data[i];

		x = (int)(rho  * sin(theta) / 20) + halfWidth;
		y = (int)(-rho * cos(theta) / 20) + halfHeight;

		circle(dst, Point(x, y), 4, Scalar(0, 0, 255), -1, 8, 0);

	}

	char s[35];
	sprintf(s, "Value Count: %d, Scan Speed: %0.2f", Data.size(), Scan_Speed);
	putText(dst, s, Point(50, 50), 4, .5, Scalar(0, 0, 0), 2);

	imshow(window_name, dst);

	return SUCCESS;

}// int __lidar_img::Draw(Mat dst, char window_name[])

int __lidar_img::Normalize_Data(vector<__scandot> data)
{
	int i;
	bool has_upper[360] = { false };
	bool has_lower[360] = { false };

	// 备份数据
	for (i = 0; i < ANGLE_ALL; i++)
		Data_NLast[i] = Data_NArray[i];

	memset(Data_NArray, 0.0f, ANGLE_ALL * sizeof(float));
	for (i = 0; i < data.size() - 1; i++)
	{
		float angle   = data[i].Angle;
		int angle_neg = (int)data[i].Angle;
		int angle_pos = (int)data[i].Angle + 1;
		float dst = data[i].Dst;

		// TODO 把角度向两边分解
		Data_NArray[angle_neg] = data[i].Dst * cos((angle - angle_neg) * PI / 180.0f);
		Data_NArray[angle_pos] = data[i].Dst * cos((angle_pos - angle) * PI / 180.0f);

	}

	return SUCCESS;


}// int __lidar_img::Normalize_Data(vector<__scandot> data)

int __lidar_img::calc_Velocity(void)
{
	// 计算速度
	// 手段：求导
	// 结果：噪声很大，效果不好
	int i, j;

	//int postive_id = 0;		// 计数变量，用来判断多少点是可以用的
	//__scandot dot;

	for (i = 0; i < ANGLE_ALL; i++)
	{
		Vlct_NArray[i] = Data_NArray[i] - Data_NLast[i];
		//Vlct_NArray[i] = Vlct_NArray[i] * Scan_Speed;
	}

	/*
	Vlct.clear();
	for (i = 0; i < Data.size() && i < Data_Last.size(); i++)
	{
		for (j = MIN(abs(i - 10), 0); j < MIN(i + 10, Data_Last.size()); j++)
		{
			if (fabs(Data[i].Angle - Data_Last[j].Angle) <= 0.707f * 1.0f)	// 测量精度1度，这边取其的0.707则认为是同一点
			{
				postive_id++;

				dot.Quality = Data[i].Quality * 0.5f + Data_Last[j].Quality * 0.5f;		// 取的点的质量为二者加权平均数
				dot.Angle   = Data[i].Angle   * 0.5f + Data_Last[j].Angle   * 0.5f;

				// 速度
				dot.Vlct = (Data[i].Dst - Data_Last[j].Dst) * Scan_Speed;

				Vlct.push_back(dot);

				break;
			}
		}

	}
	*/

	return SUCCESS;

}// int __lidar_img::calc_Velocity(void)

int __lidar_img::Kalman_Filter(float acc_x, float acc_y)
{
	// acc_x 指向机头方向的加速度
	// acc_y 垂直机头方向的加速度
	int i;

	memset(Vlct_KArray, 0, sizeof(float) * ANGLE_ALL);
	for (i = 0; i < ANGLE_ALL; i++)
	{
		// 迭代
		KF_Speed[i].X_last = KF_Speed[i].X;
		KF_Speed[i].P_last = KF_Speed[i].P;

		KF_Speed[i].X_mid = KF_Speed[i].X_last + cos(i * PI / 180.0f) * acc_x + sin(i * PI / 180.0f) * acc_y;
		KF_Speed[i].P_mid = KF_Speed[i].P_last + Q_Spd;

		KF_Speed[i].K = KF_Speed[i].P_mid / (KF_Speed[i].P_mid + R_Spd);
		KF_Speed[i].X = KF_Speed[i].X_mid + KF_Speed[i].K * (Vlct_NArray[i] - KF_Speed[i].X_mid);
		KF_Speed[i].P = (1 - KF_Speed[i].K) * KF_Speed[i].P_mid;

		Vlct_KArray[i] = KF_Speed[i].X;
	}

	memset(Data_KArray, 0, sizeof(float) * ANGLE_ALL);
	for (i = 0; i < ANGLE_ALL; i++)
	{
		KF_Dst[i].X_last = KF_Dst[i].X;
		KF_Dst[i].P_last = KF_Dst[i].P;

		KF_Dst[i].X_mid = KF_Dst[i].X_last + KF_Speed[i].X;
		KF_Dst[i].P_mid = KF_Dst[i].P_last + Q_Dst;

		KF_Dst[i].K = KF_Dst[i].P_mid / (KF_Dst[i].P_mid + R_Dst);
		KF_Dst[i].X = KF_Dst[i].X_mid + KF_Dst[i].K * (Data_NArray[i] - KF_Dst[i].X_mid);
		KF_Dst[i].P = (1 - KF_Dst[i].K) * KF_Speed[i].P_mid;

		Data_KArray[i] = KF_Dst[i].X;

	}

	return SUCCESS;

}// int __lidar_img::Kalman_Filter(float acc_x, float acc_y)

int __lidar_img::Vlct_Orthogonal_Decomposition(float vlct[])
{
	// 速度正交分解
	// 并且对两个分量去掉0的值，求平均
	int i;
	double vx, vy;

	// 速度正交分解，求平均
	vx = vy = 0;
	for (i = 0; i < ANGLE_ALL; i++)
	{
		vx += vlct[i] * cos(i * PI / 180.0f);
		vy += vlct[i] * sin(i * PI / 180.0f);
	}

	Vx = vx / ANGLE_ALL;
	Vy = vy / ANGLE_ALL;

	Vx = -Vx;
	Vy = -Vy;

	return SUCCESS;

}// int __lidar_img::Vlct_Orthogonal_Decomposition(void)


int __lidar_img::normalize_Orentation(float data[], int yaw)
{
	// 根据当前偏航方向旋转数据
	int i;
	float temp[ANGLE_ALL] = { 0 };

	// 先把数据复制一份出去
	for (i = 0; i < ANGLE_ALL; i++)
	{
		temp[i] = data[i];
	}

	// 进行旋转
	for (i = 0; i < ANGLE_ALL; i++)
	{
		data[(i + yaw) % ANGLE_ALL] = temp[i];
	}

	return SUCCESS;


}// int __lidar_img::normalize_Orentation(int yaw)

