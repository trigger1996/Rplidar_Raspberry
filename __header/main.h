#ifndef __Main_H

#define __Main_H

#include <iostream>
#include <math.h>
#include <sys/time.h>
#include <config.h>

#include "mpu9250.h"
#include "euler_angle.h"

#include "lidar_image.h"
#include "obstacle.h"
#include "position.h"

#include "tinyslam.h"

using namespace std;
using namespace cv;

bool checkRPLIDARHealth(RPlidarDriver * drv);
void update_IMUData();

#endif // __Main_H


