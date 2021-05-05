#include "utils/utils.h"

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

#ifndef USE_SISTEMVEIW
PUTCHAR_PROTOTYPE {
    /* Place your implementation of fputc here */
    /* e.g. write a character to the USART1 and Loop until the end of transmission */
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
    // HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&ch, 1);
    return ch;
}

int _write(int32_t file, uint8_t *ptr, int32_t len) {
    /* Implement your write code here, this is used by puts and printf for example */
    int DataIdx;
    for (DataIdx = 0; DataIdx < len; DataIdx++) {
        __io_putchar(*ptr++);
    }
    // HAL_UART_Transmit_DMA(&huart1, ptr, len);
    return len;
    /* return len; */
}
#endif

char *itoa(int value, char *result, int base) {
    // check that the base if valid
    if (base < 2 || base > 36) {
        *result = '\0';
        return result;
    }

    char *ptr = result, *ptr1 = result, tmp_char;
    int tmp_value;

    do {
        tmp_value = value;
        value /= base;
        *ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz"[35 + (tmp_value - value * base)];
    } while (value);

    // Apply negative sign
    if (tmp_value < 0) *ptr++ = '-';
    *ptr-- = '\0';
    while (ptr1 < ptr) {
        tmp_char = *ptr;
        *ptr--   = *ptr1;
        *ptr1++  = tmp_char;
    }
    return result;
}


// TIM7 is a time base source of HAL
// Prescaler = (uwTimclock/1000000 - 1) to have a 1MHz counter clock.
// Period = [(TIM7CLK/1000) - 1]. to have a (1/1000) s time base.
// TIM7->CNT = [0, 999]
uint64_t GetSysTimeUs(void) {
    uint64_t ms = HAL_GetTick();
    return ms * 1000ull + TIM7->CNT;
}


void AssertFail(char *exp, char *file, int line) {
    printf("Assert failed %s:%d\n", file, line);
    NVIC_SystemReset();
}