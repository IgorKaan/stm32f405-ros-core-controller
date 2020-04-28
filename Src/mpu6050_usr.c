/*
 * vl53l0x.c
 *
 *  Created on: 30 ���. 2019 �.
 *      Author: Igor
 */
//#include "vl53l0x.h"
#include "mpu6050_usr.h"
#include "stm32f4xx_hal.h"

extern volatile uint32_t sysTick_Time;
extern I2C_HandleTypeDef hi2c1;
extern uint8_t ctrl;
extern float fGX_Cal;
extern float fGY_Cal;
extern float fGZ_Cal;
extern float gyroX;
extern float gyroY;
extern float gyroZ;
extern float accelX;
extern float accelY;
extern float accelZ;
extern int i;
extern int is_initialized;
//HAL_StatusTypeDef status = HAL_OK;
uint8_t test = 0x00;
extern uint8_t a;
uint8_t value = 0;

void delay(uint32_t delayTime){
	uint32_t startTime =  sysTick_Time;
	while ( (sysTick_Time - startTime) < delayTime );
}

void error(void) {
	HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_8);
			HAL_Delay(300);
			HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_8);
			HAL_Delay(300);
			HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_8);
			HAL_Delay(300);
			HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_8);
			HAL_Delay(300);
			HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_8);
			HAL_Delay(300);
			HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_8);
			HAL_Delay(300);
			HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_8);
}

uint8_t sensor_io_read(uint16_t DeviceAddr, uint8_t RegisterAddr) {
	return I2Cx_ReadData(DeviceAddr, RegisterAddr);
	return HAL_OK;
}

uint8_t sensor_io_write(uint16_t DeviceAddr, uint8_t RegisterAddr, uint8_t Value) {
	I2Cx_WriteData(DeviceAddr, RegisterAddr, Value);
	return HAL_OK;
}

uint8_t I2Cx_ReadData(uint16_t Addr, uint8_t Reg) {
	HAL_StatusTypeDef status = HAL_OK;
	value = 0;
	status = HAL_I2C_Mem_Read(&hi2c1, Addr<<1, Reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 0x10000);
	if(status != HAL_OK)
	{
		a = 1;
	    error();
	}
	return value;
}

void I2Cx_WriteData(uint16_t Addr, uint8_t Reg, uint8_t Value) {
	HAL_StatusTypeDef status = HAL_OK;
    status = HAL_I2C_Mem_Write(&hi2c1, Addr, (uint16_t)(Reg<<1), I2C_MEMADD_SIZE_8BIT, &Value, 1, 0x10000);
    /* Check the communication status */
    if(status != HAL_OK)
    {
    /* Execute user timeout callback */
    error();
    }
}

void ledOn(void) {
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_12,GPIO_PIN_SET);
}

void ledOff(void) {
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_12,GPIO_PIN_RESET);
}

void sensor_ini(void) {
	HAL_Delay(100);
	if ((read_id()==0xFF)||(read_id()==0x70)||(read_id()==0x48)) {
		a = 2;
	}
}

uint8_t read_id(void) {
	ctrl = 0x00;
	ctrl = sensor_io_read(0x68, 0x75);
	return ctrl;
}

void MPU6050_init(void)
{
	//power up gyro
	MPU6050_writeReg(MPU6050_IM, MPU6050_PWR_MGMT_1, 0x00);
	//gyro config 500
	MPU6050_writeReg(MPU6050_IM, MPU6050_GYRO_CONFIG, 0x08);
	//accel config 8g
	MPU6050_writeReg(MPU6050_IM, MPU6050_ACCEL_CONFIG, 0x10);

}

void MPU6050_calibrate(void)
{
//	 int16_t buffer[6];
//	 uint16_t iNumCM = 300;
//	 for (i = 0; i < iNumCM ; i ++){
//		 buffer[0] = MPU6050_readReg(MPU6050_IM, MPU6050_GYRO_XOUT_H);
//		 buffer[1] = MPU6050_readReg(MPU6050_IM, MPU6050_GYRO_XOUT_L);
//		 buffer[2] = MPU6050_readReg(MPU6050_IM, MPU6050_GYRO_YOUT_H);
//		 buffer[3] = MPU6050_readReg(MPU6050_IM, MPU6050_GYRO_YOUT_L);
//		 buffer[4] = MPU6050_readReg(MPU6050_IM, MPU6050_GYRO_ZOUT_H);
//		 buffer[5] = MPU6050_readReg(MPU6050_IM, MPU6050_GYRO_ZOUT_L);
//	     HAL_Delay(20);
//	 }
//	 is_initialized = 1;
}

void MPU6050_getAllData(int16_t *Data)
{
	uint8_t buffer[12];
	buffer[0] = MPU6050_readReg(MPU6050_IM, MPU6050_GYRO_XOUT_H);
	buffer[1] = MPU6050_readReg(MPU6050_IM, MPU6050_GYRO_XOUT_L);
	buffer[2] = MPU6050_readReg(MPU6050_IM, MPU6050_GYRO_YOUT_H);
	buffer[3] = MPU6050_readReg(MPU6050_IM, MPU6050_GYRO_YOUT_L);
	buffer[4] = MPU6050_readReg(MPU6050_IM, MPU6050_GYRO_ZOUT_H);
	buffer[5] = MPU6050_readReg(MPU6050_IM, MPU6050_GYRO_ZOUT_L);
	buffer[6] = MPU6050_readReg(MPU6050_IM, MPU6050_ACCEL_XOUT_H);
	buffer[7] = MPU6050_readReg(MPU6050_IM, MPU6050_ACCEL_XOUT_L);
	buffer[8] = MPU6050_readReg(MPU6050_IM, MPU6050_ACCEL_YOUT_H);
	buffer[9] = MPU6050_readReg(MPU6050_IM, MPU6050_ACCEL_YOUT_L);
	buffer[10] = MPU6050_readReg(MPU6050_IM, MPU6050_ACCEL_ZOUT_H);
	buffer[11] = MPU6050_readReg(MPU6050_IM, MPU6050_ACCEL_ZOUT_L);
	gyroX=(((int16_t)((uint16_t)buffer[0] << 8) + buffer[1]))/65.5f*3.14f/180.0f;
	gyroY=(((int16_t)((uint16_t)buffer[2] << 8) + buffer[3]))/65.5f*3.14f/180.0f;
	gyroZ=(((int16_t)((uint16_t)buffer[4] << 8) + buffer[5]))/65.5f*3.14f/180.0f;
	accelX=(((int16_t)((uint16_t)buffer[6] << 8) + buffer[7]))/4096.0f*9.8f;
	accelY=(((int16_t)((uint16_t)buffer[8] << 8) + buffer[9]))/4096.0f*9.8f;
	accelZ=(((int16_t)((uint16_t)buffer[10] << 8) + buffer[11]))/4096.0f*9.8f;
}

void MPU6050_writeReg(uint16_t Addr, uint8_t reg, uint8_t value)
{
	uint8_t buf[2];
	buf[0] = reg;
	buf[1] = value;
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(Addr << 1), buf, 2, 1000);
}

// Write a 16-bit register
void MPU6050_writeReg16Bit(uint16_t Addr, uint8_t reg, uint16_t value)
{
	uint8_t buf[3];
	buf[0] = reg;
	buf[1] = (uint8_t) (value >> 8);
	buf[2] = (uint8_t) (value & 0xFF);
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(Addr << 1), buf, 3, 1000);
}

// Write a 32-bit register
void MPU6050_writeReg32Bit(uint16_t Addr, uint8_t reg, uint32_t value)
{
	uint8_t buf[5];
	buf[0] = reg;
	buf[1] = (uint8_t) (value >> 24);
	buf[2] = (uint8_t) (value >> 16);
	buf[3] = (uint8_t) (value >> 8);
	buf[4] = (uint8_t) (value & 0xFF);
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(Addr << 1), buf, 5, 1000);
}

// Read an 8-bit register
uint8_t MPU6050_readReg(uint16_t Addr, uint8_t reg)
{
  uint8_t value;
  HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(Addr << 1), reg, 1, &value, 1, 1000);
  return value;
}

// Read a 16-bit register
uint16_t MPU6050_readReg16Bit(uint16_t Addr, uint8_t reg)
{
  uint16_t value;
  uint8_t buff[2];
  MPU6050_readMulti(Addr, reg, buff, 2);
  uint16_t tmp;
  tmp = buff[0];
  tmp <<= 8;
  tmp |= buff[1];
  value = tmp;
  return value;
}

// Read a 32-bit register
uint32_t MPU6050_readReg32Bit(uint16_t Addr, uint8_t reg)
{
  uint32_t value;
  uint8_t buff[4];
  MPU6050_readMulti(Addr, reg, buff, 4);
  uint32_t tmp;
  tmp = buff[0];
  tmp <<= 8;
  tmp |= buff[1];
  tmp <<= 8;
  tmp |= buff[2];
  tmp <<= 8;
  tmp |= buff[3];
  value = tmp;
  return value;
}

// Write an arbitrary number of bytes from the given array to the sensor,
// starting at the given register
void MPU6050_writeMulti(uint16_t Addr, uint8_t reg, uint8_t* src, uint8_t count)
{
	uint8_t data_index[1];
	data_index[0] = reg;
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(Addr << 1), data_index, 1, 1000);
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(Addr << 1), src, count, 1000);
}

// Read an arbitrary number of bytes from the sensor, starting at the given
// register, into the given array
void MPU6050_readMulti(uint16_t Addr, uint8_t reg, uint8_t * dst, uint8_t count)
{
	HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(Addr << 1), reg, 1, dst, count, 1000);
}

void I2C_ReadBuffer(uint8_t I2C_ADDRESS, uint8_t RegAddr, uint8_t *aRxBuffer, uint8_t RXBUFFERSIZE)
{
    MPU6050_writeReg(I2C_ADDRESS, RegAddr, 1);

    while(HAL_I2C_Master_Receive(&hi2c1, (uint16_t)I2C_ADDRESS<<1, aRxBuffer, (uint16_t)RXBUFFERSIZE, (uint32_t)1000) != HAL_OK){
        if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF){
            //_Error_Handler(__FILE__, __LINE__);
        }
    }

    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY){}
}


