#include "ekf_slam.h"

int __ekf_slam::run(bool is_updated)
{
	// 这个函数跟get_Sensor()放一起，都放在lidar刷新的标志位内
	// 所以不用像Matlab里面一样等待刷新
	// 但是这边还是写吧，强迫症，提醒自己这个东西该怎么用
	int stat = 0;

	// 备份上一时刻的状态
	Xe_Last = Xe;
	// 打印姿态
	cout << "X: " << Xe(0, 0) << " Y: " << Xe(1, 0)<< endl;

	// 预测位置，注意，这边换了模型
	// 这边是因为偏航角锁了，然后模型的X和Y换了，所以算法不一样了
	// x
	Xe(0, 0) = Xe(0, 0) + (Acc.X * dt * dt + 0.5 * Acc.X * dt * dt) * sin(phi);
	// y
	Xe(1, 0) = Xe(1, 0) + (Acc.Y * dt * dt + 0.5 * Acc.Y * dt * dt) * cos(phi);
	// z
	Xe(2, 0) = phi;
	Xe(2, 0) = normalize_Angle(Xe(2, 0));      // keep phi between 0 and 360 degrees


	// 对于剩下的landmark都不用进行计算了，直接用旧的就好了，因为这边没有迭代处理，所以一个式子都不用写

	// 进行协方差的预测
	// Calculate Jacobian A based on vehicle model(df / dx)
	F(0, 0) = 1; F(0, 1) = 0; F(0, 2) = (Acc.X * dt * dt + 0.5 * Acc.X * dt * dt) *  cos(phi);;
	F(1, 0) = 0; F(1, 1) = 1; F(1, 2) = (Acc.Y * dt * dt + 0.5 * Acc.Y * dt * dt) * -sin(phi);
	F(2, 0) = 0; F(2, 1) = 0; F(2, 2) = 1;
	// rest of A(which is just 2 null matrices and 1 identity) is added in new_state function

	MatrixXd F_T = F.transpose();
	MatrixXd W_T = W.transpose();

	Pe = F * Pe * F_T + W * Q * W_T;			// 这边的X和P都用Xe和Pe直接迭代

	if (is_updated)
	{
		int i;

		for (i = 0; i < landmark_num; i++)		// i = 0; i < landmark_num; i++	取i = 0或者i = 1容易发散
		{
			// z.r == 0则不要计算了
			if (Z(2 * i, 0) == 0)
				continue;


			if (numStates == 3)
			{
				// 如果是第一次测量到的话
				__point2p_rad z_lidar;
				z_lidar.r = Z(2 * i, 0);			// 2 * landmark_num - 2
				z_lidar.rad = Z(2 * i + 1, 0);		// 2 * landmark_num - 2 + 1

				// 检查了数据，静止的时候准倒是挺准，就是有时候经常会变成0
				// 变成0能直接毁掉整个滤波器
				// 这里工程手段直接解决
				new_State(z_lidar);
				numStates = numStates + 2;			// increase number of states by 2 (1 for xi, 1 for yi)
				stat = update_New(z_lidar);

			}
			else
			{
				// 如果不是第一次计算
				// 这边我们不用官方的手法，官方是通过距离进行匹配的，或者理解成根本不匹配
				// 因为他认为：只要是刚进入测量范围的就一定是新的，所以直接当做新的更新矩阵

				// 而我们使用匹配来做，手法就会不一样，我们通过landmark的数量来做，因为前一级已经做好匹配，新的总是排在最后
				// 所以只要上一次的landmark数量小于本次的，则认为本次多出来的那些是新的
				__point2p_rad z_lidar;
				z_lidar.r = Z(2 * i, 0);
				z_lidar.rad = Z(2 * i + 1, 0);

				// 检查了数据，静止的时候准倒是挺准，就是有时候经常会变成0
				// 变成0能直接毁掉整个滤波器
				// 这里工程手段直接解决

				if (landmark_num < landmark_num_last)
					return -1;

				if (i < landmark_num && i >= landmark_num_last)		// i > landmark_num_last
				{
					// 如果是多出来的这一部分则认为是新的
					new_State(z_lidar);								// add new state(i.e.increase all matrix sizes)
					numStates = numStates + 2;						// increase number of states by 2 (1 for xi, 1 for yi)
					stat = update_New(z_lidar);						// update state and covariance
				}
				else
				{
					// 因为已知i < landmark_num，所以此处i < landmark_num_last
					// 原版的matlab的代码这边直接取找了个最近的来用，我们这边试着用下所有的看看效果怎么样
					//__point2p_rad z_lidar;
					//z_lidar.r = Z(2 * i, 0);
					//z_lidar.rad = Z(2 * i + 1, 0);

					// 这边做个调整，用最小的那个作为激光雷达的刷新点
					int min_dst_seq = 0;
					for (int j = 0; j < landmark_num; j++)
					{
						// 找到距离最小的点
                        if (Z(2 * min_dst_seq, 0) > Z(2 * j, 0) &&
							Z(2 * j, 0) != 0)
							min_dst_seq = j;
					}
					if (min_dst_seq <= 0)
						return 0;									// 如果得不到数据则放弃

					z_lidar.r = Z(2 * min_dst_seq, 0);				// 2 * landmark_num - 2
					z_lidar.rad = Z(2 * min_dst_seq + 1, 0);		// 2 * landmark_num - 2 + 1
					stat = update_Existing(z_lidar, min_dst_seq);

					if (stat == 0)
						return 0;
				}
			}

			// 这个部分是预留着用于重置卡尔曼滤波器的，因为有时候会出现NaN，这个时候就需要重置整个卡尔曼滤波器
			if (stat == -5)
			{
				init();

				// 只要上一次数据没问题，则用上一次的数据来作为位置，这样避免位置重置
				if (is_nan(Xe_Last(0, 0)) == false &&
					is_nan(Xe_Last(1, 0)) == false)
                    Xe = Xe_Last;
			}
		}
	}


	return 0;

}// int __ekf_slam::run()

__ekf_slam::__ekf_slam()
{
	// 这边多加了一个init()其实主要是考虑重置滤波器的，不然初始化直接写构造函数里面就行了
	init();

}// __ekf_slam::__ekf_slam()

int __ekf_slam::init()
{
	Xe.resize(3, 1);
	Xe.setZero();
	X = Xe;			// 设成3 * 1的0矩阵
	Xe_Last = Xe;

	Pe.resize(3, 3);
	Pe.setZero();
	P = Pe;			// 设成3 * 3的0矩阵

	F.resize(3, 3);
	F.setZero();

	W.resize(3, 3);
	for (int i = 0; i < W.rows(); i++)
		W(i, i) = 1;

	Q.resize(3, 3);
	Q << 0.49, 0,    0,
		 0,    0.49, 0,
		 0,    0,   (7 * PI / 180) * (7 * PI / 180);		// Q先带个值进去，然后整定

	Jh.resize(2, 3);
	Jh.setZero();

	K.resize(3, 3);
	K.setZero();

	Z.resize(0, 1);
	Z.setZero();

	landmark_num_last = landmark_num = 0;
	numStates = 3;											// 可以理解为Xe的行数，Xe.rows()

	current_Pos.X = current_Pos.Y = 0.0f;

	return 0;

}// int __ekf_slam::init()


int __ekf_slam::get_Sensors(vector<__point2p> lidar, __Vec3f a, __Vec3f w, __Vec3f v, double pitch, double roll, double yaw, double t)
{
	// 输入传感器参数
	//MatrixXd temp;

	const double G = 9.8;

	landmark_num_last = landmark_num;
	landmark_num = lidar.size();
	//numStates = lidar.size() + 3;

	//temp.resize(Z.rows(), Z.cols());
	//temp = Z;
	Z.resize(lidar.size() * 2, 1);
	Z.setZero();


	// 激光雷达参数
	// 前次状态的保存的任务交给旧的建图和匹配模块，故这里无需关心，因为前一级已经处理好了
	for (int i = 0, j = 0; i < Z.rows() - 1; i += 2, j++)
	{
		Z(i, 0) = lidar[j].r / 1000.0f;				// mm/s^2 -> m/s^2
		Z(i + 1, 0) = lidar[j].deg * PI / 180.0f;
	}

	//for (int i = 0; i < lidar.size(); i++)
	//	cout << "lidar.r: " << lidar[i].r << "lidar.deg: " << lidar[i].deg << endl;
	cout << "Z: " << endl << Z << endl;

	// 加速度
	Acc.X = a.X;
	Acc.Y = a.Y;
	Acc.Z = a.Z;

	//cout << "Acc.X:" << Acc.X << " " << "Acc.Y:" << Acc.Y << endl;

	// 角速度
	Gyro.X = w.X;
	Gyro.Y = w.Y;
	Gyro.Z = w.Z;

	// 速度
	Vlct.X = v.X;
	Vlct.Y = v.Y;
	Vlct.Z = v.Z;

	cout << "Vlct.X:" << Vlct.X << " " << "Vlct.Y" << Vlct.Y << endl;

	Pitch = pitch * PI / 180.0f;
	Roll  = roll * PI / 180.0f;

	// 偏航角
	phi = yaw * PI / 180.0f;

	cout << "Pitch: " << Pitch << " Roll: " << Roll << " Yaw: " << phi << endl;

	// 这边对做个处理，剪掉重力加速度在x轴和y轴的分量，然后对角度做个补偿，即加速度仅仅包含水平分量
	Acc.X = Acc.X - G * sin(Pitch);
	Acc.X = Acc.X * cos(Pitch);
	Acc.Y = Acc.Y - G * sin(Roll);
	Acc.Y = Acc.Y * cos(Roll);

	cout << "Acc.X:" << Acc.X << " " << "Acc.Y:" << Acc.Y << endl;

	dt = t;

	return 0;

}// int __ekf_slam::get_Sensors(vector<__point2p> lidar, __Vec3f a, __Vec3f w, double yaw, double t)

int __ekf_slam::new_State(__point2p_rad z_lidar)
{
	MatrixXd temp;
	int i, j;

	double xi;
	double yi;
	double theta;

	theta = z_lidar.rad;			// 因为在实际情况下已经归一化旋转了, deg: 0~360度
	xi = Xe(0, 0) + z_lidar.r * sin(theta);
	yi = Xe(1, 0) + z_lidar.r * cos(theta);

	// 扩展Xe矩阵，为了多塞下一个元素
	//temp.resize(X.rows(), X.cols());
	temp = Xe;
	Xe.resize(Xe.rows() + 2, 1);		// X是个2n + 3行1列的矩阵
	Xe.setZero();
	for (int i = 0; i < temp.rows(); i++)
		Xe(i, 0) = temp(i, 0);
	Xe(temp.rows() - 1 + 1, 0) = xi;		// 输入这个障碍物的初始位置
	Xe(temp.rows() - 1 + 2, 0) = yi;		// 上一次做的主要错在这里，漏了这个

	cout << "Xe: " << endl << Xe << endl;

	// 扩展Pe矩阵
	// 一般新扩展出来的协方差在理论上可以认为是无穷大
	temp = Pe;
	Pe.resize(Pe.rows() + 2, Pe.cols() + 2);
	Pe.setZero();
	for (i = 0; i < temp.rows(); i++)
		for (j = 0; j < temp.cols(); j++)	// 这边最好不要int j = 0, 容易被释放掉造成出错
			Pe(i, j) = temp(i, j);
	Pe(Pe.rows() - 1, Pe.cols() - 1) = 1000000;		// 10^6，模拟无穷大
	Pe(Pe.rows() - 2, Pe.cols() - 2) = 1000000;

	cout << "Pe: " << endl << Pe << endl;

	// 扩展A矩阵，没写错F就是A矩阵
	temp = F;
	F.resize(F.rows() + 2, F.cols() + 2);
	F.setZero();
	for (i = 0; i < temp.rows(); i++)
		for (j = 0; j < temp.cols(); j++)	// 这边最好不要int j = 0, 容易被释放掉造成出错
			F(i, j) = temp(i, j);
	F(F.rows() - 1, F.cols() - 1) = 1;
	F(F.rows() - 2, F.cols() - 2) = 1;

	cout << "F: " << endl << F << endl;

	// 扩展Q矩阵和W矩阵，这两个都不用怎么搞，扩展完就不用管了
	temp = Q;
	Q.resize(Q.rows() + 2, Q.cols() + 2);
	Q.setZero();
	for (i = 0; i < temp.rows(); i++)
		for (j = 0; j < temp.cols(); j++)	// 这边最好不要int j = 0, 容易被释放掉造成出错
			Q(i, j) = temp(i, j);

	cout << "Q: " << endl << Q << endl;

	temp = W;
	W.resize(W.rows() + 2, W.cols() + 2);
	W.setZero();
	for (i = 0; i < temp.rows(); i++)
		for (j = 0; j < temp.cols(); j++)	// 这边最好不要int j = 0, 容易被释放掉造成出错
			W(i, j) = temp(i, j);

	cout << "W: " << endl << W << endl;

	return 0;

}//int __ekf_slam::new_State()

int __ekf_slam::update_New(__point2p_rad z)
{
	double dx, dy;
	double r;

	// 规范输入
	// z: 半径，单位: m, 范围: 0~6
	// z: 角度，单位: rad, 范围: 0~2PI
	if (z.r <= 0.0f || z.r > 6.5f)
		return -1;
	z.rad = normalize_Angle(z.rad);

	dx = Xe(numStates - 2, 0) - Xe(0, 0);
	dy = Xe(numStates - 1, 0) - Xe(1, 0);
	r  = sqrt(dx * dx + dy * dy);

	H.resize(2, 1);
	H(0, 0) = r;
	H(1, 0) = atan2(dx, dy);							// atan2(y, x)是表示X - Y平面上所对应的(x, y)坐标的角度，它的值域范围是(-Pi, Pi)
	H(1, 0) = normalize_Angle(H(1, 0));

	cout << "H: " << endl << H << endl;

	// Calculate Jacobian Jh(dh / dx)
	Jh(0, 0) = dx / r;		Jh(0, 1) = dy / r;			Jh(0, 2) = 0;		// 这里的模型我们改了，因为角度锁了，而且x和y的方向有点调整
	Jh(1, 0) = dy / (r*r);	Jh(1, 1) = dx / (r*r);		Jh(1, 2) = 0;

	cout << "dx: " << dx << " " << "r: " << r << endl;

	if (numStates > 3)
	{
		// 如果第一次有东西过来
		MatrixXd temp;
		int i, j;

		// 扩展Jh
		temp = Jh;
		Jh.resize(2, numStates);
		Jh.setZero();
		for (i = 0; i < temp.rows(); i++)
			for (j = 0; j < temp.cols(); j++)	// 这边最好不要int j = 0, 容易被释放掉造成出错
				Jh(i, j) = temp(i, j);
	}

	Jh(0, numStates - 2) = -dx / r;			Jh(0, numStates - 1) = -dy / r;			// this is for a new landmark, so the last two columns are modified
	Jh(0, numStates - 2) = -dy / (r*r);     Jh(1, numStates - 1) = -dx / (r*r);		// 注意，这边模型也换了

	cout << "Jh: " << endl << Jh << endl;

	// 不知道下面两行什么意思
	// --LB_debug: changed from Jh(1, ..) = -dx / r, four operator are all reversed
	// --LB_comment : this is the biggest error in the debug procedure


	V.resize(2, 2);
	V << 1, 0,
		 0, 1;

	cout << "V: " << endl << V << endl;

	R.resize(2, 2);
	R << 0.01, 0,
		 0,    (PI / 180.0f) * (PI / 180.0f);

	cout << "R: " << endl << R << endl;

	innov.resize(2, 1);							// 小心这个地方会爆炸
	innov(0, 0) = z.r   - H(0, 0);
	innov(1, 0) = z.rad - H(1, 0);
	innov(1, 0) = normalize_Angle(innov(1, 0));

	cout << "innov: " << endl << innov << endl;

	MatrixXd Jh_T = Jh.transpose();
	MatrixXd V_T  = V.transpose();
	MatrixXd inv = (Jh * Pe * Jh_T + V * R * V_T);

	inv = inv.inverse();
	K = Pe * Jh_T * inv;

	cout << "K: " << endl << K << endl;

	// 注意，EKF这边直接用Xe代替X了，好算
	Xe = Xe + K * innov;					// update state matrix
	Xe(2, 0) = normalize_Angle(Xe(2, 0));

	cout << "X_out:" << endl << Xe << endl;

	MatrixXd I;
	I.resize((K * Jh).rows(), (K * Jh).cols());
	I.setZero();
	for (int i = 0; i < I.rows(); i++)
		I(i, i) = 1;
	Pe = (I - K * Jh) * Pe;          // udpate covariance matrix

	cout << "I - K * Jh: " << endl << I - K * Jh << endl;
	cout << "K * Jh: " << endl << K * Jh << endl;
	cout << "P_out:" << endl << Pe << endl;

	current_Pos.X = Xe(0, 0);
	current_Pos.Y = Xe(1, 0);

	// 这里的-5是用来判断滤波器是否整个爆炸的，如果爆炸了则重置
	// 下同
	if (is_nan(Pe(0, 0)) == true)
	{
		//init();
		return -5;
	}

	return 0;

}// int __ekf_slam::update_New(__point2p_rad z)

int __ekf_slam::update_Existing(__point2p_rad z, int landmark)
{
	double dx, dy;
	double r;

	// 补充一个这个，不然会内存溢出
	landmark--;

	// 规范输入
	// z: 半径，单位: m, 范围: 0~6
	// z: 角度，单位: rad, 范围: 0~2PI
	// landmark: 不严格的障碍物序列号，这个值可以理解成Xe的指针
	if (z.r < 0.0f || z.r > 6.5f)
		return -1;
	z.rad = normalize_Angle(z.rad);

	cout << "233:    " << landmark << endl;
	cout << "Xe.rows(): " << Xe.rows() << endl;

	dx = Xe(landmark, 0)     - Xe(0, 0);	// distance between existing landmark and x - pos of vehicle
	dy = Xe(landmark + 1, 0) - Xe(1, 0);	// distance between existing landmark and x - pos of vehicle
	r = sqrt(dx*dx + dy*dy);

	cout << "2333333" << endl;

	// Sensor Model - range and bearing
	H.resize(2, 1);
	H(0, 0) = r;
	H(1, 0) = atan2(dx, dy);							// atan2(y, x)是表示X - Y平面上所对应的(x, y)坐标的角度，它的值域范围是(-Pi, Pi)
	H(1, 0) = normalize_Angle(H(1, 0));

	cout << "H: " << endl << H << endl;

	// Calculate Jacobian Jh(dh / dx)
	Jh(0, 0) = dx / r;		Jh(0, 1) = dy / r;			Jh(0, 2) = 0;		// 这里的模型我们改了，因为角度锁了，而且x和y的方向有点调整
	Jh(1, 0) = dy / (r*r);	Jh(1, 1) = dx / (r*r);		Jh(1, 2) = 0;

	cout << "dx: " << dx << " " << "r: " << r << endl;

	if (numStates > 3)
	{
		// 如果第一次有东西过来
		MatrixXd temp;
		int i, j;

		// 扩展Jh
		temp = Jh;
		Jh.resize(2, numStates);
		Jh.setZero();
		for (i = 0; i < temp.rows(); i++)
			for (j = 0; j < temp.cols(); j++)	// 这边最好不要int j = 0, 容易被释放掉造成出错
				Jh(i, j) = temp(i, j);
	}

	Jh(0, landmark) = -dx / r;			Jh(0, landmark + 1) = -dy / r;			// this is for a new landmark, so the last two columns are modified
	Jh(0, landmark) = -dy / (r*r);     Jh(1, landmark + 1) = -dx / (r*r);		// 注意，这边模型也换了

	cout << "NumStates: " << endl << numStates << endl;
	cout << "Jh: " << endl << Jh << endl;

	V.resize(2, 2);
	V << 1, 0,
		 0, 1;

	R.resize(2, 2);
	R << 0.01, 0,
		 0,    (PI / 180.0f) * (PI / 180.0f);


	innov.resize(2, 1);							// 小心这个地方会爆炸
	innov(0, 0) = z.r - H(0, 0);
	innov(1, 0) = z.rad - H(1, 0);
	innov(1, 0) = normalize_Angle(innov(1, 0));

	cout << "innov: " << endl << innov << endl;

	MatrixXd Jh_T = Jh.transpose();
	MatrixXd V_T = V.transpose();
	MatrixXd inv = (Jh * Pe * Jh_T + V * R * V_T);

	inv = inv.inverse();
	K = Pe * Jh_T * inv;

	cout << "K: " << endl << K << endl;

	// 注意，EKF这边直接用Xe代替X了，好算
	Xe = Xe + K * innov;					// update state matrix
	Xe(2, 0) = normalize_Angle(Xe(2, 0));

	cout << "x_out:" << endl << Xe << endl;

	MatrixXd I;
	I.resize((K * Jh).rows(), (K * Jh).cols());
	I.setZero();
	for (int i = 0; i < I.rows(); i++)
		I(i, i) = 1;
	Pe = (I - K * Jh) * Pe;          // udpate covariance matrix

	cout << "p_out:" << endl << Pe << endl;

	current_Pos.X = Xe(0, 0);
	current_Pos.Y = Xe(1, 0);

	// 如果滤波器爆炸了则重置
	if (is_nan(Pe(0, 0)) == true)
	{
		init();
		return -5;
	}

	return 0;

}// int __ekf_slam::update_Existing(__point2p_rad z, int landmark)


double __ekf_slam::normalize_Angle(double in)
{
	// 输入任意角度，输出0~2*pi的角度

	while (in < 0.0f)
		in += 2 * PI;

	if (in > 2 * PI)
		in -= 2 * PI;

	return  in;

}// int __ekf_slam::normalize_Angle()

bool is_nan(double dval)
{
	// 判断一个数是不是NaN
	if (dval == dval)
		return false;
	else
		return true;
}// bool is_nan(double dval)
