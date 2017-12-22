#ifndef __PIXHAWK_DATALINK_H

#define __PIXHAWK_DATALINK_H


#include "config.h"
#include "wiringPi.h"
#include "wiringSerial.h"


#define PIX_RX_MAX 256
#define PIX_DATALINK_DEVICE "/dev/ttyS0"

class __pix_link
{
public:
	__pix_link();

	int init();

	int get_Data();

	__AHRS Eular_Angle;
	__AHRS Eular_Angle_Last;

	__Vec3f W;
	__Vec3f Acc;
	__Vec3f V;

private:

	int fd;
	int baudrate;

	char cache[256];
	int  cache_ptr;

	int receive();
	int refine(char str[]);

};

int find_Break(char str[], int len);

#endif	/* __PIXHAWK_DATALINK_H */
