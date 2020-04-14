#ifndef __motor_H
#define __motor_H

#include "tim.h"

void MotorInit(void);
void SetMotor(uint16_t m1, uint16_t m2, u_int16_t m3, u_int16_t m4);

#endif /* __motor_H */