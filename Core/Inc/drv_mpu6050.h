#ifndef DRV_MPU6050_H
#define DRV_MPU6050_H

#include "main.h"

// MPU6050 I2C 주소 (AD0가 GND에 연결되어 있을 경우 0x68)
#define MPU_ADDR            (0x68 << 1)

// MPU6050 주요 레지스터 주소
#define MPU_WHO_AM_I_REG    0x75
#define MPU_PWR_MGMT_1_REG  0x6B
#define MPU_ACCEL_XOUT_H_REG 0x3B

// 함수 프로토타입
uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c);
uint8_t MPU6050_Read_Raw(I2C_HandleTypeDef *hi2c, int16_t *acc, int16_t *gyro);

#endif
