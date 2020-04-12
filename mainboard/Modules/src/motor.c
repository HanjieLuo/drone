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