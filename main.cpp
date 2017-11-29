#include "main.h"

/*

注意：
	1 http://blog.csdn.net/xuxuyoyo/article/details/53506711
	2 在windows环境下创建工程时，应使用较低的.net版本，如3.0，这个工程就是用3.0的.net才能编译的
	3 加入宏定义WIN32
		#define WIN32
	  这个定义要在Linux内去掉，不然会影响其工作

工程内的细节：
	4 关于vector的使用
		vector(向量): C++中的一种数据结构,确切的说是一个类.它相当于一个动态的数组,当程序员无法知道自己需要的数组的规模多大时,用其来解决问题可以达到最大节约空间的目的.
		http://blog.csdn.net/hancunai0017/article/details/7032383

	5 关于Mat的初始化
		Mat::Mat(int rows, int cols, int type, constScalar& s)
		type用CV_8UC3
		http://blog.csdn.net/zang141588761/article/details/50340709

	6 row和col
	  都是反的
		row == heigh == Point.y
		col == width == Point.x
		Mat::at(Point(x, y)) == Mat::at(y,x)

	7 putText
		http://blog.csdn.net/guduruyu/article/details/68491211

	8 Rplidar的获取频率函数
		drv->getFrequency(is_express, count, frequency, is_4k_mode);
		注意is_express和is_4k_mode都必须是变量，而不能是常量，否则不能工作

	9 OpenCV的霍夫变换详见这里
		http://blog.csdn.net/poem_qianmo/article/details/26977557/

	10 激光雷达对弧形障碍物的探测精度相当低，所以暂时都当直线障碍物处理
		增大每次作图的点半径，提高霍夫变换命中率，这样来实现更低的障碍物分类阈值，实现将曲线拟合成直线
*/

__vec3f A, G, H;

int main(int argc, char *argv[]) {

	int stat;
	int i;

	__e_angle E;

	// 初始化9250
	while (true) {

		stat = IMU.init();
		if (stat == 0)
			break;


		cout << "IMU Initiation is a failure, Code:" << stat << endl;
		delay(1000);		// 1s
	}

	// 初始化激光雷达
	const char * opt_com_path = NULL;
	_u32         opt_com_baudrate = 115200;
	u_result     op_result;
	int Yaw = 90;

	Mat kalman;

	// 处理类
	__lidar_img Lidar_Img;
	__obstacle_group OG;
	__positioning    P;
	//__tinyslam ts;


	// OpenCV测试
	Mat hk416 = imread("hk416.jpg");
	imshow("hk416", hk416);

	// read serial port from the command line...
	if (argc>1) opt_com_path = argv[1]; // or set to a fixed value: e.g. "com3"

										// read baud rate from the command line if specified...
	if (argc>2) opt_com_baudrate = strtoul(argv[2], NULL, 10);


	if (!opt_com_path) {
#ifdef _WIN32
		// use default com port
		opt_com_path = "\\\\.\\com3";
#else
		opt_com_path = "/dev/ttyUSB0";
#endif
	}

	// create the driver instance
	RPlidarDriver * drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);


	if (!opt_com_path) {
#ifdef _WIN32
		// use default com port
		opt_com_path = "\\\\.\\com3";
#else
		opt_com_path = "/dev/ttyUSB0";
#endif
	}

	if (!drv) {
		fprintf(stderr, "insufficent memory, exit\n");
		exit(-2);
	}


	// make connection...
	if (IS_FAIL(drv->connect(opt_com_path, opt_com_baudrate))) {
		fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
			, opt_com_path);

		waitKey(0);
		return -3;
	}



	// check health...
	if (!checkRPLIDARHealth(drv)) {

		waitKey(0);
		return -4;
	}


	// start scan...
	drv->startMotor();
	drv->startScan();


	i = 0;
	while (true) {

		update_IMUData();

		E.process_Data(A, G, H);

		rplidar_response_measurement_node_t nodes[360 * 2];
		size_t   count = _countof(nodes);
		bool is_express = false;
		bool is_4k_mode = false;

		op_result = drv->grabScanData(nodes, count);

		if (IS_OK(op_result)) {
			float frequency = 0;

			// 读取数据
			drv->getFrequency(is_express, count, frequency, is_4k_mode);
			Lidar_Img.scanData(nodes, count, frequency);
			//Lidar_Img.Draw(Lidar_Img.Img_Dst_Raw, Lidar_Img.Data, "Raw");

			// 数据角度归一化，预处理、滤波，计算速度
			Lidar_Img.Normalize_Data(Lidar_Img.Data);
			Lidar_Img.calc_Velocity();
			Lidar_Img.Kalman_Filter(0.0f, 0.0f);
			Lidar_Img.normalize_Orentation(Lidar_Img.Data_KArray, Yaw);

			// 预处理结果
			Lidar_Img.Draw(kalman, Lidar_Img.Data_KArray, "Dst_Kalman");

			// 建图
			OG.get_Array(Lidar_Img.Data_KArray);
			OG.draw();
			OG.calc_Lines();
			//OG.surf();

			// tinyslam测试
			//ts.Run(Lidar_Img.Data_KArray, Lidar_Img.Data, Lidar_Img.Vx, Lidar_Img.Vy);

			// 自制建图算法测试
			P.update_LineGroup(OG.OLines);
			P.calc_Grid_Velocity(Lidar_Img.Vx, Lidar_Img.Vy, frequency);

			i = 0;
			waitKey(30);
		}

		// 显示处理得到的角度数据
		if (i % 100 == 0) {

			//cout << "Ax: " << A.X << " Ay: " << A.Y << " Az: " << A.Z << endl;
			//cout << "Gx: " << G.X << " Gy: " << G.Y << " Gz: " << G.Z << endl;
			//cout << "Hx: " << H.X << " Hy: " << H.Y << " Hz: " << H.Z << endl;

			cout << "Pitch: " << E.Pitch << endl;
			cout << "Roll:  " << E.Roll  << endl;
			cout << "Yaw:   " << E.Yaw   << endl;
			cout << endl;
			cout << endl;

		}
		i++;
		delay(5);
	}


	waitKey(0);
    return 0;
}// main

void update_IMUData() {

		float ax, ay, az;
		float gx, gy, gz;
		int16_t hx, hy, hz;
		IMU.get_Accel(&ax, &ay, &az);
		IMU.get_Gyro(&gx, &gy, &gz);
		IMU.get_Mag_HT(&hx, &hy, &hz);

		A.X = ax;
		A.Y = ay;
		A.Z = az;
		G.X = gx;
		G.Y = gy;
		G.Z = gz;
		H.X = hx;
		H.Y = hy;
		H.Z = hz;

}// void update_IMUData()

bool checkRPLIDARHealth(RPlidarDriver * drv)
{
	u_result     op_result;
	rplidar_response_device_health_t healthinfo;

	op_result = drv->getHealth(healthinfo);
	if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
		printf("RPLidar health status : %d\n", healthinfo.status);
		if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
			fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
			// enable the following code if you want rplidar to be reboot by software
			// drv->reset();
			return false;
		}
		else {
			return true;
		}

	}
	else {
		fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
		return false;
	}
}// bool checkRPLIDARHealth(RPlidarDriver * drv)

