# jacdaegi

STM32 기반 MPU6050 자세 제어 시스템 (PID Balance Controller)

본 프로젝트는 STM32 마이크로컨트롤러를 이용하여 MPU6050 IMU 센서 데이터를 처리하고,
PID 제어 알고리즘을 통해 DC 모터의 자세 안정화를 수행하는 임베디드 제어 시스템입니다.

---

## Project Overview

이 시스템은 Roll 각도를 기준으로 균형을 유지하는 제어 시스템입니다.

실시간 센서 데이터 처리, PID 제어 루프, 모터 출력 제어, I2C 통신 복구 기능을 포함합니다.

---

## Project Structure

- Core/ : main source code & interrupt handlers  
- *.ioc : STM32CubeMX hardware configuration file  

---

## Control System

### Sensor
- MPU6050 IMU (Accelerometer + Gyroscope)
- I2C communication

### Filtering
- Complementary Filter
- Roll angle estimation

### Control Algorithm

```c
error = target_angle - safe_angle;

pid_output = (Kp * error)
           - (Kd * (gyro[0] / 131.0f));
