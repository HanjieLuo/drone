#include "eskf.h"

MAT_ALLOC(gravity, 3, 1);
float32_t std_acc_noise[3], std_gyro_noise[3], std_acc_bias_noise[3], std_gyro_bias_noise[3];

bool EskfInit(const float32_t *_gravity,
              const float32_t *_std_acc_noise,
              const float32_t *_std_gyro_noise,
              const float32_t *_std_acc_bias_noise,
              const float32_t *_std_gyro_bias_noise) {

    MAT_INIT(gravity, 3, 1);

    memcpy(MAT_DATA(gravity), _gravity, 3 * sizeof(float32_t));
    memcpy(std_acc_noise, _std_acc_noise, 3 * sizeof(float32_t));
    memcpy(std_gyro_noise, _std_gyro_noise, 3 * sizeof(float32_t));
    memcpy(std_acc_bias_noise, _std_acc_bias_noise, 3 * sizeof(float32_t));
    memcpy(std_gyro_bias_noise, _std_gyro_bias_noise, 3 * sizeof(float32_t));

    

    return true;
}
