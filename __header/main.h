#ifndef __Main_H

#define __Main_H

#include <iostream>
#include <unistd.h>
#include <stdlib.h>
#include <math.h>
#include <sys/time.h>
#include <config.h>

//#include "mpu9250.h"
//#include "euler_angle.h"
#include "datalink.h"

#include "lidar_image.h"
#include "cv_match.h"

#include "ekf_slam.h"

using namespace std;
using namespace cv;

bool checkRPLIDARHealth(RPlidarDriver * drv);
void update_IMUData();

#endif // __Main_H


