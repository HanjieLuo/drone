#ifndef __utils_H
#define __utils_H

#include "stm32f4xx_hal.h"
#include "stdio.h"
// #include "usart.h"

extern UART_HandleTypeDef huart1;

char *itoa(int value, char *result, int base);

#endif /* __uart4_H */
