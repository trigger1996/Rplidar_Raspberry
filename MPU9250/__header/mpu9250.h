/**
 ******************************************************************************
 * @addtogroup PIOS PIOS Core hardware abstraction layer
 * @{
 * @addtogroup PIOS_MPU9250 MPU9250 Functions
 * @brief Deals with the hardware interface to the 3-axis gyro
 * @{
 *
 * @file       pios_mpu9250.h
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2014
 * @brief      MPU9250 9-DOF chip function headers
 * @see        The GNU Public License (GPL) Version 3
 *
 ******************************************************************************
 */
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */


#ifndef __MPU9250_H
#define __MPU9250_H


#include "mpu60x0.h"
#include "physical_constant.h"
#include <stdio.h>
#include <stdlib.h>

#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <wiringPiSPI.h>

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"


#define MPU9250_WHOAMI_ID       0x71		// ??????? 为什么是0x71，真的对吗

#ifdef PIOS_MPU9250_SPI_HIGH_SPEED
	#define MPU9250_SPI_HIGH_SPEED              PIOS_MPU9250_SPI_HIGH_SPEED
#else
	#define MPU9250_SPI_HIGH_SPEED          20000000	// should result in 10.5MHz clock on F4 targets like Sparky2
#endif

#define MPU9250_SPI_LOW_SPEED               300000

#define _MPU9250_ACCEL_DLPF_CFG_REG         0x1D

#define _MPU9250_AK8963_ADDR                0x0C
#define AK8963_WHOAMI_REG                   0x00
#define AK8963_WHOAMI_ID                    0x48
#define AK8963_ST1_REG                      0x02
#define AK8963_ST2_REG                      0x09
#define AK8963_ST1_DOR                      0x02
#define AK8963_ST1_DRDY                     0x01
#define AK8963_ST2_BITM                     0x10
#define AK8963_ST2_HOFL                     0x08
#define AK8963_CNTL1_REG                    0x0A
#define AK8963_CNTL2_REG                    0x0B		// 这里官方写0x0A,应该是错了
#define AK8963_CNTL2_SRST                   0x01
#define AK8963_MODE_CONTINUOUS_FAST_16B     0x16
#define AK8963_MODE_SINGLE_MASUREMENT_16B	0x11		// 自增加

// 自增加
#define AK8963_ST1_REG						0x02
#define AK8963_HXL_REG						0x03
#define AK8963_HXH_REG						0x04
#define AK8963_HYL_REG						0x05
#define AK8963_HYH_REG						0x06
#define AK8963_HZL_REG						0x07
#define AK8963_HZH_REG						0x08

#define EXT_SENS_DATA_00					0x49
#define EXT_SENS_DATA_01					0x4A
#define EXT_SENS_DATA_02					0x4B
#define EXT_SENS_DATA_03					0x4C
#define EXT_SENS_DATA_04					0x4D
#define EXT_SENS_DATA_05					0x4E
#define EXT_SENS_DATA_06					0x4F

enum _mpu9250_gyro_filter {
	_MPU9250_GYRO_LOWPASS_250_HZ = 0x00,
	_MPU9250_GYRO_LOWPASS_184_HZ = 0x01,
	_MPU9250_GYRO_LOWPASS_92_HZ  = 0x02,
	_MPU9250_GYRO_LOWPASS_41_HZ  = 0x03,
	_MPU9250_GYRO_LOWPASS_20_HZ  = 0x04,
	_MPU9250_GYRO_LOWPASS_10_HZ  = 0x05,
	_MPU9250_GYRO_LOWPASS_5_HZ   = 0x06
};

enum _mpu9250_accel_filter {
	_MPU9250_ACCEL_LOWPASS_460_HZ = 0x00,
	_MPU9250_ACCEL_LOWPASS_184_HZ = 0x01,
	_MPU9250_ACCEL_LOWPASS_92_HZ  = 0x02,
	_MPU9250_ACCEL_LOWPASS_41_HZ  = 0x03,
	_MPU9250_ACCEL_LOWPASS_20_HZ  = 0x04,
	_MPU9250_ACCEL_LOWPASS_10_HZ  = 0x05,
	_MPU9250_ACCEL_LOWPASS_5_HZ   = 0x06
};

enum _mpu9250_orientation { // clockwise rotation from board forward
	_MPU9250_TOP_0DEG       = 0x00,
	_MPU9250_TOP_90DEG      = 0x01,
	_MPU9250_TOP_180DEG     = 0x02,
	_MPU9250_TOP_270DEG     = 0x03,
	_MPU9250_BOTTOM_0DEG    = 0x04,
	_MPU9250_BOTTOM_90DEG   = 0x05,
	_MPU9250_BOTTOM_180DEG  = 0x06,
	_MPU9250_BOTTOM_270DEG  = 0x07
};


struct _mpu9250_cfg {
	const struct _exti_cfg *exti_cfg; /* Pointer to the EXTI configuration */

	uint16_t default_samplerate;	/* Sample to use in Hz (See RM datasheet page 12 for more details) */
	uint8_t interrupt_cfg;			/* Interrupt configuration (See RM datasheet page 20 for more details) */
	bool use_magnetometer;			/* Use magnetometer or not - for example when external mag. is used */
	enum _mpu9250_gyro_filter default_gyro_filter;
	enum _mpu9250_accel_filter default_accel_filter;
	enum _mpu9250_orientation orientation;
};


struct mpu9250_dev {

	enum _mpu60x0_accel_range accel_range;
	enum _mpu60x0_range gyro_range;
	const struct _mpu9250_cfg *cfg;
	enum _mpu9250_gyro_filter gyro_filter;
	enum _mpu9250_accel_filter accel_filter;

};



class __mpu9250 {

public:

	__mpu9250(uint8_t i2c_address);
	__mpu9250(uint8_t spibus, int32_t speed);
	~__mpu9250();

	int init();
	int set_Gyro_Range(enum _mpu60x0_range range);
	int set_Accel_Range(enum _mpu60x0_accel_range);
	int set_SampleRate(uint16_t samplerate_hz);
	void set_Gyro_DLPF(enum _mpu9250_gyro_filter filter);
	void set_Accel_DLPF(enum _mpu9250_accel_filter filter);

	void get_Accel(float* ax, float* ay, float* az);
	void get_Gyro(float* gx, float* gy, float* gz);
	void get_Mag(float* hx, float* hy, float* hz);
	void get_Mag_HT(int16_t *hx, int16_t *hy, int16_t *hz);

	float get_GyroScale();
	float get_AccelScale();

	// 读写陀螺仪
    int write_Reg(uint8_t reg, uint8_t data);
	uint8_t read_Reg(uint8_t reg);

private:

	bool _use_I2C;
	bool _use_SPI;
	bool _enable_Mag;

	uint8_t _spi_bus;
    int32_t _spi_speed;

	uint8_t _i2c_address;
    uint8_t _i2c_fd;

    mpu9250_dev dev;	// 使用pios原装的一个东西，去掉了任务队列一些的，这个用起来挺爽的，所以留着

    int config();		// 初始化设定
    int config_Mag();

	// 读写磁力计
	int write_Reg_Mag(uint8_t reg, uint8_t data);
	uint8_t read_Reg_Mag(uint8_t reg);

	// 读数据，因为DMP要用所以这边改成public了
	void get_AccelCounts(int16_t* ax, int16_t* ay, int16_t* az);
	void get_GyroCounts(int16_t* gx, int16_t* gy, int16_t* gz);
	void get_MagCounts(int16_t* hx, int16_t* hy, int16_t* hz);
};

extern __mpu9250 IMU;

#endif /* __MPU9250_H */

