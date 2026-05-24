#ifndef DRV_MPU6050_FILTER_H
#define DRV_MPU6050_FILTER_H

#include <stdint.h>

// 필터 가중치 (나중에 튜닝할 때 여기만 수정)
#define FILTER_ALPHA 0.90f
#define FILTER_BETA  (1.0f - FILTER_ALPHA)

typedef struct {
    float pitch;
    float roll;
    float f_pitch;
    float f_roll;
} IMU_Angle_t;

void Apply_Complementary_Filter(int16_t *acc, int16_t *gyro, IMU_Angle_t *angle, float dt);

#endif
