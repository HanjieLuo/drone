#ifndef __ESKF_H
#define __ESKF_H

#include "FreeRTOS.h"
#include "queue.h"
#include "utils.h"
#include "matrix.h"

bool EskfInit(const float32_t *_gravity,
              const float32_t *_acc_noise,
              const float32_t *_gyro_noise,
              const float32_t *_acc_bias_noise,
              const float32_t *_gyro_bias_noise);

void EskfPredict(const float *acc, const float *gyro);
#endif /* __ESKF_H */
