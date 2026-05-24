# jacdaegi

STM32 기반 MPU6050 자세 제어 시스템 (PID Balance Controller)

본 프로젝트는 STM32 마이크로컨트롤러를 이용하여 MPU6050 IMU 센서 데이터를 처리하고,
PID 제어 알고리즘을 통해 DC 모터의 자세 안정화를 수행하는 임베디드 제어 시스템입니다.

---

## Project Overview

이 시스템은 Roll 각도를 기준으로 균형을 유지하는 제어 시스템이며,
실시간 센서 데이터 처리 + 제어 루프 + 모터 출력 + 통신 복구 기능을 포함합니다.

핵심 목표:
- IMU 기반 실시간 자세 추정
- PID 제어를 통한 안정화
- I2C 통신 장애 대응
- 실시간 디버깅 및 상태 모니터링

---

## System Architecture

### Sensor Layer
- MPU6050 (Accelerometer + Gyroscope)
- I2C 통신 기반 데이터 수집

### Processing Layer
- Complementary Filter 적용
- Roll Angle 계산

### Control Layer
- PID Controller (P / I / D)
- 목표 각도 기반 오차 계산

### Actuation Layer
- DC Motor PWM 제어
- 방향 + 속도 제어

---

## Control Algorithm

```c
error = target_angle - safe_angle;

Target
목표 각도: -20°

pid_output = (Kp * error)
           - (Kd * (gyro[0] / 131.0f));Kp = 45.0f;

Kp = 45.0f;
Ki = 5.0f;
Kd = 5.0f;
propeller_adj = 2.0f;Ki = 5.0f;
Kd = 5.0f;
propeller_adj = 2.0f;
