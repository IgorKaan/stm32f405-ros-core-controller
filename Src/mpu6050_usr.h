/*
 * vl53l0x.h
 *
 *  Created on: 30 ���. 2019 �.
 *      Author: Igor
 */

#ifndef MPU6050_USR_H_
#define MPU6050_USR_H_

#include "stm32f4xx_hal.h"

#define MPU6050_I2C_ADDR			0xD0

/* Who I am register value */
#define MPU6050_IM				0x68

/* MPU6050 registers */
#define MPU6050_AUX_VDDIO			0x01
#define MPU6050_SMPLRT_DIV			0x19
#define MPU6050_CONFIG				0x1A
#define MPU6050_GYRO_CONFIG			0x1B
#define MPU6050_ACCEL_CONFIG		0x1C
#define MPU6050_MOTION_THRESH		0x1F
#define MPU6050_INT_PIN_CFG			0x37
#define MPU6050_INT_ENABLE			0x38
#define MPU6050_INT_STATUS			0x3A
#define MPU6050_ACCEL_XOUT_H		0x3B
#define MPU6050_ACCEL_XOUT_L		0x3C
#define MPU6050_ACCEL_YOUT_H		0x3D
#define MPU6050_ACCEL_YOUT_L		0x3E
#define MPU6050_ACCEL_ZOUT_H		0x3F
#define MPU6050_ACCEL_ZOUT_L		0x40
#define MPU6050_TEMP_OUT_H			0x41
#define MPU6050_TEMP_OUT_L			0x42
#define MPU6050_GYRO_XOUT_H			0x43
#define MPU6050_GYRO_XOUT_L			0x44
#define MPU6050_GYRO_YOUT_H			0x45
#define MPU6050_GYRO_YOUT_L			0x46
#define MPU6050_GYRO_ZOUT_H			0x47
#define MPU6050_GYRO_ZOUT_L			0x48
#define MPU6050_MOT_DETECT_STATUS	0x61
#define MPU6050_SIGNAL_PATH_RESET	0x68
#define MPU6050_MOT_DETECT_CTRL		0x69
#define MPU6050_USER_CTRL			0x6A
#define MPU6050_PWR_MGMT_1			0x6B
#define MPU6050_PWR_MGMT_2			0x6C
#define MPU6050_FIFO_COUNTH			0x72
#define MPU6050_FIFO_COUNTL			0x73
#define MPU6050_FIFO_R_W			0x74
#define MPU6050_WHO_AM_I			0x75

void ledOn(void);
void ledOff(void);
void error(void);
void sensor_ini(void);
uint8_t read_id(void);

uint8_t I2Cx_ReadData(uint16_t Addr, uint8_t Reg);
void I2Cx_WriteData(uint16_t DeviceAddr, uint8_t RegisterAddr, uint8_t Value);
uint8_t sensor_io_read(uint16_t DeviceAddr, uint8_t RegisterAddr);
uint8_t sensor_io_write(uint16_t DeviceAddr, uint8_t RegisterAddr, uint8_t Value);

void MPU6050_init(void);
void MPU6050_calibrate(void);
void MPU6050_getAllData(int16_t *Data);
void MPU6050_writeReg(uint16_t Addr, uint8_t reg, uint8_t value);
void MPU6050_writeReg16Bit(uint16_t Addr, uint8_t reg, uint16_t value);
void MPU6050_writeReg32Bit(uint16_t Addr, uint8_t reg, uint32_t value);
void MPU6050_writeMulti(uint16_t Addr, uint8_t reg, uint8_t* src, uint8_t count);
uint8_t MPU6050_readReg(uint16_t Addr, uint8_t reg);
uint16_t MPU6050_readReg16Bit(uint16_t Addr, uint8_t reg);
uint32_t MPU6050_readReg32Bit(uint16_t Addr, uint8_t reg);
void MPU6050_readMulti(uint16_t Addr, uint8_t reg, uint8_t * dst, uint8_t count);
void I2C_ReadBuffer(uint8_t I2C_ADDRESS, uint8_t RegAddr, uint8_t *aRxBuffer, uint8_t RXBUFFERSIZE);

#endif /* VL53L0X_USR_H_ */
