#include "drv_mpu6050_filter.h"
#include <math.h>

void Apply_Complementary_Filter(int16_t *acc, int16_t *gyro, IMU_Angle_t *angle, float dt) {
    // 1. Raw 가속도 각도 계산
    float acc_pitch = atan2f((float)acc[0], (float)acc[2]) * 57.29577f;
    float acc_roll  = atan2f((float)acc[1], (float)acc[2]) * 57.29577f;

    angle->pitch = acc_pitch;
    angle->roll  = acc_roll;

    // 2. 상보 필터 적용 (헤더 파일의 매크로 사용)
    angle->f_pitch = FILTER_ALPHA * (angle->f_pitch + (gyro[1] / 131.0f) * dt) + FILTER_BETA * acc_pitch;
    angle->f_roll  = FILTER_ALPHA * (angle->f_roll  + (gyro[0] / 131.0f) * dt) + FILTER_BETA * acc_roll;
}
