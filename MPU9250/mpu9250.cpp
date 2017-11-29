#include "mpu9250.h"

// https://github.com/simondlevy/RPi_MPU9250/blob/master/MPU9250.cpp
// https://github.com/trigger1996/TauLabs/blob/next/flight/PiOS/inc/pios_mpu60x0.h



__mpu9250 IMU(1, 50000);		// CS1, 50KHz

// transformation matrix
/* transform the accel and gyro axes to match the magnetometer axes */

const int16_t tX[3] = {0,  1,  0};
const int16_t tY[3] = {1,  0,  0};
const int16_t tZ[3] = {0,  0, -1};

__mpu9250::__mpu9250(uint8_t i2c_address) {

	_i2c_address = i2c_address;

	_use_I2C = true;
	_use_SPI = false;
	_enable_Mag = true;

}// __mpu9250::__mpu9250(uint8_t i2c_address)


__mpu9250::__mpu9250(uint8_t spibus, int32_t speed) {

    _spi_bus = spibus;
    _spi_speed = speed;

	_use_I2C = false;
	_use_SPI = true;
	_enable_Mag = true;

}// __mpu9250::__mpu9250(uint8_t spibus, int32_t speed)

__mpu9250::~__mpu9250() {



}// __mpu9250::~__mpu9250()


int __mpu9250::init() {

	int stat;

    if( _use_SPI == true ) { // using SPI for communication
        // begin the SPI
        if (wiringPiSPISetup(_spi_bus, _spi_speed) < 0)
            return -1;
    }
    else if ( _use_I2C == true) { // using I2C for communication

        // starting the I2C bus
        _i2c_fd = wiringPiI2CSetup (_i2c_address);

        if (_i2c_fd < 0)
            return -1;
    }
    else {

		printf("No Communication Mode Selected! \n");
		return -1;
    }

	stat = config();
    if (stat != 0)
		return stat - 100;


	return 0;

}// int __mpu9250::init_SPI()


int __mpu9250::config() {

	int stat;

	// reset chip
	write_Reg(_MPU60X0_PWR_MGMT_REG, _MPU60X0_PWRMGMT_IMU_RST);

	// give chip some time to initialize
	delay(50);		// ms


	uint8_t id = read_Reg(_MPU60X0_WHOAMI);

	if (id != MPU9250_WHOAMI_ID)
		return -2;

	// power management config
	write_Reg(_MPU60X0_PWR_MGMT_REG, _MPU60X0_PWRMGMT_PLL_X_CLK);

	// user control
	write_Reg(_MPU60X0_USER_CTRL_REG, _MPU60X0_USERCTL_DIS_I2C |
									  _MPU60X0_USERCTL_I2C_MST_EN);



	if (_enable_Mag == true) {
		stat = config_Mag();

		if (stat != 0)
			return stat - 10;
	}



	// Digital low-pass filter and scale
	// set this before sample rate else sample rate calculation will fail
	set_Accel_DLPF(_MPU9250_ACCEL_LOWPASS_184_HZ);
	set_Gyro_DLPF(_MPU9250_GYRO_LOWPASS_184_HZ);

	// Sample rate
	stat = set_SampleRate(184);
	if (stat != 0)
		return stat - 200;

	// Set the gyro scale
	set_Gyro_Range(_MPU60X0_SCALE_500_DEG);

	// Set the accel scale
	set_Accel_Range(_MPU60X0_ACCEL_4G);

	// Interrupt configuration
	//PIOS_MPU9250_WriteReg(PIOS_MPU60X0_INT_CFG_REG, cfg->interrupt_cfg);

	// Interrupt enable
	//PIOS_MPU9250_WriteReg(PIOS_MPU60X0_INT_EN_REG, PIOS_MPU60X0_INTEN_DATA_RDY);


	return 0;

}// int __mpu9250::config_Gyro()

int __mpu9250::config_Mag() {



	uint8_t id = read_Reg_Mag(AK8963_WHOAMI_REG);
	if (id != AK8963_WHOAMI_ID)
		return -1;
	delay(1);

	// reset AK8963
	int stat = 1;
	stat = write_Reg_Mag(AK8963_CNTL2_REG, AK8963_CNTL2_SRST);
	if (stat != 0)
		return -2;


	// give chip some time to initialize
	delay(50);

	// set magnetometer sampling rate to 100Hz and 16-bit resolution
	write_Reg_Mag(AK8963_CNTL1_REG, AK8963_MODE_CONTINUOUS_FAST_16B);	// AK8963_MODE_CONTINUOUS_FAST_16B
																		// AK8963_MODE_SINGLE_MASUREMENT_16B

	// configure mpu9250 to read ak8963 data range from STATUS1 to STATUS2 at ODR
	write_Reg(_MPU60X0_SLV0_REG_REG, AK8963_ST1_REG);
	write_Reg(_MPU60X0_SLV0_ADDR_REG, _MPU9250_AK8963_ADDR | 0x80);
	write_Reg(_MPU60X0_SLV0_CTRL_REG, _MPU60X0_I2CSLV_EN | 8);

	return 0;

}// int __mpu9250::config_Mag()

int __mpu9250::set_Gyro_Range(enum _mpu60x0_range range) {

	write_Reg(_MPU60X0_GYRO_CFG_REG, range);
    dev.gyro_range = range;

	return 0;

}// int __mpu9250::set_Gyro_Range(enum _mpu60x0_range range)

int __mpu9250::set_Accel_Range(enum _mpu60x0_accel_range range) {

	write_Reg(_MPU60X0_ACCEL_CFG_REG, range);
	dev.accel_range = range;

	return 0;

}// int __mpu9250::set_Accel_Range(enum _mpu60x0_accel_range)


int __mpu9250::set_SampleRate(uint16_t samplerate_hz) {

	// mpu9250 ODR divider is unable to run from 8kHz clock like mpu60x0 :(
	// check if someone want to use 250Hz DLPF and don't want 8kHz sampling
	// and politely refuse him
	if ((dev.gyro_filter == _MPU9250_GYRO_LOWPASS_250_HZ) && (samplerate_hz != 8000))
		return -1;

	uint16_t filter_frequency = 1000;
	// limit samplerate to filter frequency
	if (samplerate_hz > filter_frequency)
		samplerate_hz = filter_frequency;

	// calculate divisor, round to nearest integeter
	int32_t divisor = (int32_t)(((float)filter_frequency / samplerate_hz) + 0.5f) - 1;

	// limit resulting divisor to register value range
	if (divisor < 0)
		divisor = 0;

	if (divisor > 0xff)
		divisor = 0xff;

	write_Reg(_MPU60X0_SMPLRT_DIV_REG, (uint8_t)divisor);

	return 0;

} // int __mpu9250::set_SampleRate(uint16_t samplerate_hz)


void __mpu9250::set_Gyro_DLPF(enum _mpu9250_gyro_filter filter) {

	write_Reg(_MPU60X0_DLPF_CFG_REG, filter);

	dev.gyro_filter = filter;

}// void __mpu9250::set_Gyro_DLPF(enum pios_mpu9250_gyro_filter filter)

void __mpu9250::set_Accel_DLPF(enum _mpu9250_accel_filter filter) {

	// Note this sets the a_fchoice_b to 0 which enables the DLPF
	// which is what is desired.

	write_Reg(_MPU60X0_ACCEL_CFG2_REG, filter);

	dev.accel_filter = filter;

}// void __mpu9250::set_Accel_DLPF(enum _mpu9250_accel_filter filter)


float __mpu9250::get_GyroScale() {

	switch (dev.gyro_range) {

		case _MPU60X0_SCALE_250_DEG:
			return 1.0f / 131.0f;

		case _MPU60X0_SCALE_500_DEG:
			return 1.0f / 65.5f;

		case _MPU60X0_SCALE_1000_DEG:
			return 1.0f / 32.8f;

		case _MPU60X0_SCALE_2000_DEG:
			return 1.0f / 16.4f;

	}

	return 0;

}// float __mpu9250::get_GyroScale()

float __mpu9250::get_AccelScale() {

	switch (dev.accel_range) {

		case _MPU60X0_ACCEL_2G:
			return GRAVITY / 16384.0f;

		case _MPU60X0_ACCEL_4G:
			return GRAVITY / 8192.0f;

		case _MPU60X0_ACCEL_8G:
			return GRAVITY / 4096.0f;

		case _MPU60X0_ACCEL_16G:
			return GRAVITY / 2048.0f;

	}

	return 0;

}// float __mpu9250::get_AccelScale()

/* get accelerometer data given pointers to store the three values */
void __mpu9250::get_AccelCounts(int16_t* ax, int16_t* ay, int16_t* az) {

    uint8_t buff[6] = { 0 };
    int16_t axx, ayy, azz;



	buff[0] = read_Reg(_MPU60X0_ACCEL_X_OUT_MSB);
    buff[1] = read_Reg(_MPU60X0_ACCEL_X_OUT_LSB);
    axx = (((int16_t)buff[0]) << 8) | buff[1];  // combine into 16 bit values
	buff[2] = read_Reg(_MPU60X0_ACCEL_Y_OUT_MSB);
    buff[3] = read_Reg(_MPU60X0_ACCEL_Y_OUT_LSB);
    ayy = (((int16_t)buff[2]) << 8) | buff[3];
	buff[4] = read_Reg(_MPU60X0_ACCEL_Z_OUT_MSB);
    buff[5] = read_Reg(_MPU60X0_ACCEL_Z_OUT_LSB);
    azz = (((int16_t)buff[4]) << 8) | buff[5];

    *ax = tX[0]*axx + tX[1]*ayy + tX[2]*azz; // transform axes
    *ay = tY[0]*axx + tY[1]*ayy + tY[2]*azz;
    *az = tZ[0]*axx + tZ[1]*ayy + tZ[2]*azz;

}// void __mpu9250::get_AccelCounts(int16_t* ax, int16_t* ay, int16_t* az)

void __mpu9250::get_Accel(float* ax, float* ay, float* az) {

    int16_t accel[3];
    float _accelScale;

    get_AccelCounts(&accel[0], &accel[1], &accel[2]);
    _accelScale = get_AccelScale();

    *ax = ((float) accel[0]) * _accelScale; // typecast and scale to values
    *ay = ((float) accel[1]) * _accelScale;
    *az = ((float) accel[2]) * _accelScale;

}// void __mpu9250::get_Accel(float* ax, float* ay, float* az)

void __mpu9250::get_GyroCounts(int16_t* gx, int16_t* gy, int16_t* gz) {

    uint8_t buff[6];
    int16_t gxx, gyy, gzz;

	buff[0] = read_Reg(_MPU60X0_GYRO_X_OUT_MSB);
    buff[1] = read_Reg(_MPU60X0_GYRO_X_OUT_LSB);
    gxx = (((int16_t)buff[0]) << 8) | buff[1];  // combine into 16 bit values
    buff[2] = read_Reg(_MPU60X0_GYRO_Y_OUT_MSB);
    buff[3] = read_Reg(_MPU60X0_GYRO_Y_OUT_LSB);
    gyy = (((int16_t)buff[2]) << 8) | buff[3];
    buff[4] = read_Reg(_MPU60X0_GYRO_Z_OUT_MSB);
    buff[5] = read_Reg(_MPU60X0_GYRO_Z_OUT_LSB);
    gzz = (((int16_t)buff[4]) << 8) | buff[5];

    *gx = tX[0]*gxx + tX[1]*gyy + tX[2]*gzz; // transform axes
    *gy = tY[0]*gxx + tY[1]*gyy + tY[2]*gzz;
    *gz = tZ[0]*gxx + tZ[1]*gyy + tZ[2]*gzz;

}// void __mpu9250::getGyroCounts(int16_t* gx, int16_t* gy, int16_t* gz)


/* get gyro data given pointers to store the three values */
void __mpu9250::get_Gyro(float* gx, float* gy, float* gz) {

    int16_t gyro[3];
    float _gyroScale;

    get_GyroCounts(&gyro[0], &gyro[1], &gyro[2]);
	_gyroScale = get_GyroScale();

    *gx = ((float) gyro[0]) * _gyroScale; // typecast and scale to values
    *gy = ((float) gyro[1]) * _gyroScale;
    *gz = ((float) gyro[2]) * _gyroScale;

}// void __mpu9250::getGyro(float* gx, float* gy, float* gz)

/* get magnetometer data given pointers to store the three values, return data as counts */
void __mpu9250::get_MagCounts(int16_t* hx, int16_t* hy, int16_t* hz) {

    uint8_t buff[6] = { 0 };

	buff[0] = read_Reg(EXT_SENS_DATA_00);	// L		// 注意这边高位和低位是反的
	buff[1] = read_Reg(EXT_SENS_DATA_01);	// H
	buff[2] = read_Reg(EXT_SENS_DATA_02);
	buff[3] = read_Reg(EXT_SENS_DATA_03);
	buff[4] = read_Reg(EXT_SENS_DATA_04);
	buff[5] = read_Reg(EXT_SENS_DATA_05);

	*hx = (((int16_t)buff[1]) << 8) | buff[0];  // combine into 16 bit values
	*hy = (((int16_t)buff[3]) << 8) | buff[2];
	*hz = (((int16_t)buff[5]) << 8) | buff[4];




}// void __mpu9250::get_MagCounts(int16_t* hx, int16_t* hy, int16_t* hz)



/* get magnetometer data given pointers to store the three values */

void __mpu9250::get_Mag(float* hx, float* hy, float* hz) {

    int16_t mag[3]  = { 0 };
	float _magScaleX, _magScaleY, _magScaleZ;

    get_MagCounts(&mag[0], &mag[1], &mag[2]);
	_magScaleX = _magScaleY = _magScaleZ = 1.0f;

    *hx = ((float) mag[0]) * _magScaleX; // typecast and scale to values
    *hy = ((float) mag[1]) * _magScaleY;
    *hz = ((float) mag[2]) * _magScaleZ;

}// void __mpu9250::get_Mag(float* hx, float* hy, float* hz)

void __mpu9250::get_Mag_HT(int16_t *hx, int16_t *hy, int16_t *hz) {

	// 十位深度滤波
	uint8_t buf[6];
	int16_t mag[3];
	static int32_t An[3] = {0,0,0};

	// 读取寄存器数据
	buf[0] = read_Reg(EXT_SENS_DATA_00);	// L		// 注意这边高位和低位是反的
	buf[1] = read_Reg(EXT_SENS_DATA_01);	// H
	buf[2] = read_Reg(EXT_SENS_DATA_02);
	buf[3] = read_Reg(EXT_SENS_DATA_03);
	buf[4] = read_Reg(EXT_SENS_DATA_04);
	buf[5] = read_Reg(EXT_SENS_DATA_05);


	// 十位深度滤波
	An[0] -= An[0]/10;
	An[0] += (int16_t)(buf[0] << 8 | buf[1]);
	mag[0] = An[0]/10;

	An[1] -= An[1]/10;
	An[1] += (int16_t)(buf[4] << 8 | buf[5]);
	mag[1] = An[1]/10;

	An[2] -= An[2]/10;
	An[2] += (int16_t)(buf[2] << 8 | buf[3]);
	mag[2] = An[2]/10;

	static int16_t mag_x_max, mag_x_min;
	static int16_t mag_y_max, mag_y_min;
	static int16_t mag_z_max, mag_z_min;

	mag_x_max = 360; mag_x_min = 180;
	mag_y_max = 135; mag_y_min = -150;
	mag_z_max = 250; mag_z_min = -240;

	*hx = mag[0] - (mag_x_max + mag_x_min) / 2;
	*hy = mag[1] - (mag_y_max + mag_y_min) / 2;
	*hz = mag[2] - (mag_z_max + mag_z_min) / 2;


}// void __mpu9250::get_Mag_HT(int16_t hx, int16_t hy, int16_t hz)


/*

void MPU9250::getTempCounts(int16_t* t){

    uint8_t buff[2];
    readRegisters(TEMP_OUT, sizeof(buff), &buff[0]); // grab the data from the MPU9250

    *t = (((int16_t)buff[0]) << 8) | buff[1];  // combine into 16 bit value and return

}

void MPU9250::getTemp(float* t){

    int16_t tempCount;
    getTempCounts(&tempCount);

    *t = (( ((float) tempCount) - _tempOffset )/_tempScale) + _tempOffset;

}
*/

int __mpu9250::write_Reg(uint8_t reg, uint8_t data) {

	int stat = 0;

    if(_use_SPI == true) {

        uint8_t buff2[2];
        buff2[0] = reg & 0x7f;
        buff2[1] = data;
        stat = wiringPiSPIDataRW(_spi_bus, buff2, 2);

    }

	return stat;

}// int __mpu9250::write_Reg(uint8_t reg, uint8_t data)

uint8_t __mpu9250::read_Reg(uint8_t reg) {

	uint8_t data = 0;

    if(_use_SPI == true) {

        uint8_t buff2[2];
		buff2[0] = reg | 0x80;
		buff2[1] = 0;

		wiringPiSPIDataRW(_spi_bus, buff2, 2);
		data = buff2[1];

    }

	return data;

}// int __mpu9250::read_Reg(uint8_t reg)

int __mpu9250::write_Reg_Mag(uint8_t reg, uint8_t data) {

	// we will use I2C SLV4 to manipulate with AK8963 control registers
    write_Reg(_MPU60X0_SLV4_REG_REG, reg);
    delay(1);		// ms
    write_Reg(_MPU60X0_SLV4_ADDR_REG, _MPU9250_AK8963_ADDR);
	delay(1);
	write_Reg(_MPU60X0_SLV4_DO_REG, data);
	delay(1);
	write_Reg(_MPU60X0_SLV4_CTRL_REG, _MPU60X0_I2CSLV_EN);
	delay(1);

	uint32_t timeout = 0;

	// wait for I2C transaction done, use simple safety
	// escape counter to prevent endless loop in case
	// MPU9250 is broken
	uint8_t stat = 0;

	do {

		if (timeout++ > 50)
			return -2;
		stat = read_Reg(_MPU60X0_I2C_MST_STATUS_REG);
		delay(1);

	} while ((stat & _MPU60X0_I2C_MST_SLV4_DONE) == 0);

	if (stat & _MPU60X0_I2C_MST_SLV4_NACK)
		return -3;

	return 0;


}// int __mpu9250::write_Reg_Mag(uint8_t reg, uint8_t data)

uint8_t __mpu9250::read_Reg_Mag(uint8_t reg) {


	// we will use I2C SLV4 to manipulate with AK8963 control registers
    write_Reg(_MPU60X0_SLV4_REG_REG, reg);
    delay(1);		// ms
    write_Reg(_MPU60X0_SLV4_ADDR_REG, _MPU9250_AK8963_ADDR | 0x80);
	delay(1);
	write_Reg(_MPU60X0_SLV4_CTRL_REG, _MPU60X0_I2CSLV_EN);
	delay(1);

	uint32_t timeout = 0;
	// wait for I2C transaction done, use simple safety
	// escape counter to prevent endless loop in case
	// MPU9250 is broken
	uint8_t stat = 0;

	do {

		if (timeout++ > 50)
			return 0;

		stat = read_Reg(_MPU60X0_I2C_MST_STATUS_REG);
		delay(1);

	} while ((stat & _MPU60X0_I2C_MST_SLV4_DONE) == 0);



	return read_Reg(_MPU60X0_SLV4_DI_REG);

}// uint8_t __mpu9250::read_Reg_Mag(uint8_t reg)


