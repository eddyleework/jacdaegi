#include "drv_mpu6050.h"

uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t check;

    // 센서 ID 체크
    if (HAL_I2C_Mem_Read(hi2c, MPU_ADDR, MPU_WHO_AM_I_REG, 1, &check, 1, 100) != HAL_OK || check != 0x68)
        return 1; // 실패

    // 센서 깨우기
    uint8_t data = 0;
    if (HAL_I2C_Mem_Write(hi2c, MPU_ADDR, MPU_PWR_MGMT_1_REG, 1, &data, 1, 100) != HAL_OK)
        return 1; // 실패

    HAL_Delay(10);
    return 0; // 성공
}

uint8_t MPU6050_Read_Raw(I2C_HandleTypeDef *hi2c, int16_t *acc, int16_t *gyro) {
    uint8_t buf[14];

    // 데이터 읽기 시도
    if (HAL_I2C_Mem_Read(hi2c, MPU_ADDR, MPU_ACCEL_XOUT_H_REG, 1, buf, 14, 100) != HAL_OK) {
        return 1; // 통신 실패 시 즉시 종료
    }

    // 가속도 X, Y, Z
    acc[0] = (int16_t)(buf[0] << 8 | buf[1]);
    acc[1] = (int16_t)(buf[2] << 8 | buf[3]);
    acc[2] = (int16_t)(buf[4] << 8 | buf[5]);

    // 자이로 X, Y, Z
    gyro[0] = (int16_t)(buf[8] << 8 | buf[9]);
    gyro[1] = (int16_t)(buf[10] << 8 | buf[11]);
    gyro[2] = (int16_t)(buf[12] << 8 | buf[13]);

    return 0; // 성공
}
