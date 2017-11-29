#ifndef __DMP_H

#define __DMP_H

#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "mpu9250.h"
#include "helper_3dmath.h"

#include <wiringPi.h>

using namespace std;

#define DIM 3			// 这个是自行脑补的
#define wrap_180(x) (x < -180 ? x+360 : (x > 180 ? x - 360: x))
#define delay_ms(a)    usleep(a*1000)

extern __mpu9250 IMU;

int writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data);
int8_t readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data);


int ms_open();
int ms_update();
int ms_close();

uint8_t GetGravity(VectorFloat *v, Quaternion *q);
uint8_t GetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity);
uint8_t GetGyro(int32_t *data, const uint8_t* packet);
#endif // __DMP_H
