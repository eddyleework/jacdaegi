#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "tim.h"
#include "motor.h"
#include "drv_mpu6050.h"
#include "drv_mpu6050_filter.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>

/* Global Variables */
int16_t acc[3], gyro[3];
IMU_Angle_t my_angle = { 0 };
MotorControl myMotor = { 1, 0 };

float Kp = 45.0f, Ki = 5.0f, Kd = 5.0f, propeller_adj = 2.0f;
float error = 0.0f, integral = 0.0f, pid_output = 0.0f;
const float dt = 0.01f;
bool debug = true;

volatile uint32_t call_count = 0, hz_result = 0;
volatile uint8_t control_flag = 0;

/* Prototypes */
void SystemClock_Config(void);
void Control_Loop(void);
void I2C_Recovery(void);
void Handle_I2C_Error(uint32_t error_code, int speed_at_err);
void Print_Status_Log(void);

int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 10);
    return ch;
}

int main(void) {
    HAL_Init();
    HAL_Delay(500);
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART2_UART_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();

    MPU6050_Init(&hi2c1);
    Motor_Init();
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_Base_Start_IT(&htim2);

    uint32_t last_sec_tick = 0, last_print_tick = 0, last_fake_err_tick = 0;

    while (1) {
        if (control_flag) {
            control_flag = 0;
            Control_Loop();
        }

        uint32_t current_tick = HAL_GetTick();

        // 60초 주기 테스트 에러
        if (current_tick - last_fake_err_tick >= 60000) {
            printf("ERR_I2C:0xDEADBEEF [TEST]\r\n");
            last_fake_err_tick = current_tick;
        }

        // 1초 주기 Hz 모니터링
        if (current_tick - last_sec_tick >= 1000) {
            hz_result = call_count;
            call_count = 0;
            last_sec_tick = current_tick;
        }

        // 500ms 주기 상태 출력
        if (current_tick - last_print_tick >= 500) {
            Print_Status_Log();
            last_print_tick = current_tick;
        }
    }
}

void Control_Loop(void) {
    /* 1. 센서 데이터 읽기 및 에러 처리 */
    if(MPU6050_Read_Raw(&hi2c1, acc, gyro) != 0) {
        Handle_I2C_Error(HAL_I2C_GetError(&hi2c1), myMotor.speed);
        return;
    }

    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

    /* 2. 각도 필터링 */
    Apply_Complementary_Filter(acc, gyro, &my_angle, dt);

    /* 3. 각도 제한 (Saturation) */
    float safe_angle = (my_angle.f_roll > 90.0f) ? 90.0f : ((my_angle.f_roll < -90.0f) ? -90.0f : my_angle.f_roll);

    /* 4. 목표 각도 설정 및 에러 계산 */
    float target_angle = -20.0f; // 목표치를 -30도로 설정
    error = target_angle - safe_angle;

    /* 5. 적분기(Integral) 제어 */
    // 목표 각도 기준으로 30도 이상 벗어나면 초기화
//    if (fabs(safe_angle - target_angle) > 30.0f) {
//        integral = 0.0f;
//    }

    integral += error * dt;

    // 적분기 제한 (Wind-up 방지)
    if (integral > 20.0f) integral = 20.0f;
    else if (integral < -20.0f) integral = -20.0f;

    /* 6. PID 출력 계산 */
    pid_output = (Kp * error) - (Kd * (gyro[0] / 131.0f));

    if (myMotor.direction == 0) {
        pid_output *= propeller_adj;
    }

    /* 7. 모터 출력 업데이트 */
    int pwm_output = (int)pid_output;
    myMotor.direction = (pwm_output >= 0);

    // speed 값을 / 1로 변경하셨으므로 그대로 유지합니다.
    int abs_speed = abs(pwm_output);
    myMotor.speed = (abs_speed > 999 ? 999 : abs_speed) / 1;

    if (fabs(error) < 10.0f) {
        if (myMotor.speed < 200) {
            myMotor.speed = 200; // 최소한의 토크 유지
        }
    }
    Motor_Update(&myMotor);
    call_count++;
}

void Handle_I2C_Error(uint32_t error_code, int speed_at_err) {
    myMotor.speed = 0;
    Motor_Update(&myMotor);
    I2C_Recovery();
    printf("ERR_I2C:0x%08lX | SpeedAtErr:%d\r\n", error_code, speed_at_err);
}

void Print_Status_Log(void) {
    printf("Ang:%.2f | Err:%.2f | Int:%.2f | PID:%.2f | Spd:%d | Dir:%d | Hz:%lu\r\n",
           (double)my_angle.f_roll, (double)error, (double)integral,
           (double)pid_output, (int)myMotor.speed,
           (int)myMotor.direction, hz_result);
}

/* ... I2C_Recovery, SystemClock_Config, Error_Handler 등은 기존과 동일 ... */

/* ---------------- Timer ISR ---------------- */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {
		control_flag = 1;
	}
}
/**
 * @brief I2C 버스 강제 복구 (Bus Hang 대응)
 * SDA/SCL 라인을 GPIO로 강제 토글하여 슬레이브의 통신 잠김을 해제합니다.
 */
void I2C_Recovery(void)
{
    /* 1. I2C 하드웨어 정지 */
    HAL_I2C_DeInit(&hi2c1);

    /* 2. GPIO 설정을 위한 클록 활성화 */
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* 3. SDA(PB7) 및 SCL(PB6)을 오픈 드레인(OD) 출력으로 설정 */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin   = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* 4. 초기 상태: 모든 라인을 HIGH로 설정 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);

    /* 5. 16개의 클록 펄스 생성 (슬레이브의 SDA 고정 해제) */
    for (int i = 0; i < 16; i++)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); // SCL Low
        for (volatile int d = 0; d < 200; d++);              // Delay
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);   // SCL High
        for (volatile int d = 0; d < 200; d++);              // Delay
    }

    /* 6. I2C STOP 조건 생성 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);    // SDA Low
    for (volatile int d = 0; d < 200; d++);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);      // SCL High
    for (volatile int d = 0; d < 200; d++);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);      // SDA High (STOP)

    /* 7. GPIO 정리 */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6 | GPIO_PIN_7);

    /* 8. I2C 주변장치 하드웨어 리셋 */
    __HAL_RCC_I2C1_FORCE_RESET();
    for (volatile int d = 0; d < 10000; d++); // 안정화 대기
    __HAL_RCC_I2C1_RELEASE_RESET();

    /* 9. 주변장치 및 MPU6050 재초기화 */
    MX_I2C1_Init();
    hi2c1.State = HAL_I2C_STATE_READY;
    MPU6050_Init(&hi2c1);
}

/* ---------------- Clock ---------------- */

void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	__HAL_RCC_PWR_CLK_ENABLE();

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;

	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;

	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 180;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;

	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
		Error_Handler();
	}

	RCC_ClkInitStruct.ClockType =
	RCC_CLOCKTYPE_HCLK |
	RCC_CLOCKTYPE_SYSCLK |
	RCC_CLOCKTYPE_PCLK1 |
	RCC_CLOCKTYPE_PCLK2;

	RCC_ClkInitStruct.SYSCLKSource =
	RCC_SYSCLKSOURCE_PLLCLK;

	RCC_ClkInitStruct.AHBCLKDivider =
	RCC_SYSCLK_DIV1;

	RCC_ClkInitStruct.APB1CLKDivider =
	RCC_HCLK_DIV4;

	RCC_ClkInitStruct.APB2CLKDivider =
	RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct,
	FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
}

/* ---------------- Error ---------------- */

void Error_Handler(void) {
	__disable_irq();

	while (1) {
	}
}
