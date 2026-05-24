#include "motor.h"
#include "tim.h"  // 타이머 핸들러 사용을 위해 필요

extern TIM_HandleTypeDef htim1; // main.c에서 정의된 핸들러 연결

void Motor_Init(void) {
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    // STBY 핀 제어
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
}

void Motor_Update(MotorControl *motor) {
    // 1. 방향 설정 (PA9, PA10)
    if (motor->direction == 1) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
    }
    // 2. PWM 설정
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, motor->speed);
}
