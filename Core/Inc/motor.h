#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"

// 모터 상태 구조체 정의
typedef struct {
    int direction;    // 1: 정, 0: 역
    uint32_t speed;   // 0~99 (Duty Cycle)
} MotorControl;

// 함수 프로토타입 (외부 공개)
void Motor_Init(void);
void Motor_Update(MotorControl *motor);

#endif
