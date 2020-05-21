#ifndef __SENSORS_H
#define __SENSORS_H

#include "FreeRTOS.h"
#include "hmc5883l.h"
#include "mpu6050.h"
#include "ms5611.h"
#include "queue.h"
#include "utils.h"

#define SensorsReadAccelRaw MPU6050ReadAccelRaw
#define SensorsReadGyroRaw MPU6050ReadGyroRaw
#define SensorsReadMagRaw HMC5883LReadMagRaw


// self.acc2real = 9.7833 / 16384.0 = 5.97125244140625e-4‬
// self.gyro2real = 0.00106422515
// self.mag2real = 0.00091743119


// Acc Calibration params
const static float Ta00 = 1.0, Ta01 = 9.0941e-05, Ta02 = -0.00458356;
const static float Ta10 = 0.0, Ta11 = 1.0, Ta12 = -0.00704908;
const static float Ta20 = 0.0, Ta21 = 0.0, Ta22 = 1.0;
const static float Sa_x = 0.000596561, Sa_y = 0.000594778, Sa_z = 0.000594165;
const static float Ba_x = -943.026, Ba_y = -100.32, Ba_z = -421.17;

// Gyro Calibration params
// bytearray(b'0.039483,-0.542183,9.724598,0.049353,0.005740,-0.016558,-0.152294,0.012844,-0.114679\n')
const static float Tg00 = 1.0, Tg01 = -0.0006364, Tg02 = 0.00508642;
const static float Tg10 = -0.00272323, Tg11 = 1.0, Tg12 = 0.00577732;
const static float Tg20 = 0.00824933, Tg21 = 0.0127456, Tg22 = 1.0;
const static float Sg_x = 0.00106156, Sg_y = 0.00105634, Sg_z = 0.00107057;
const static float Bg_x = 23.5762, Bg_y = 2.65462, Bg_z = -8.9182;

// const static float Tg00 = 1.0, Tg01 = 0.0032005, Tg02 = 0.00555608;
// const static float Tg10 = -0.00612083, Tg11 = 1.0, Tg12 = 0.00961652;
// const static float Tg20 = 0.0133572, Tg21 = 0.0172369, Tg22 = 1.0;
// const static float Sg_x = 0.00105975, Sg_y = 0.00106141, Sg_z = 0.0010787;
// const static float Bg_x = 23.8714, Bg_y = 3.58348, Bg_z = -8.76544;


const static float Sm_x = 0.00091743119, Sm_y = 0.00091743119, Sm_z = 0.00091743119;

void SensorsInit(void);
void SensorsReadTask(void *param);

void IMUCalibration(const int16_t *acc_raw, const int16_t *gyro_raw, const int16_t *mag_raw, float *acc, float *gyro, float *mag);

#endif /* __SENSORS_H */
