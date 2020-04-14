#include "motor.h"

extern TIM_HandleTypeDef htim4;

void MotorInit(void) {
    htim4.Instance->CCR1 = 0;
    htim4.Instance->CCR2 = 0;
    htim4.Instance->CCR3 = 0;
    htim4.Instance->CCR4 = 0;

    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
}

void SetMotor(uint16_t m1, uint16_t m2, u_int16_t m3, u_int16_t m4) {
    uint16_t period = htim4.Instance->ARR;

    htim4.Instance->CCR1 = (m1 > period)? period : m1;
    htim4.Instance->CCR2 = (m2 > period)? period : m2;
    htim4.Instance->CCR3 = (m3 > period)? period : m3;
    htim4.Instance->CCR4 = (m4 > period)? period : m4;
}