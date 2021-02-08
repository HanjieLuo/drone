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
const static float Sa_x = -0.000596561, Sa_y = -0.000594778, Sa_z = -0.000594165;
const static float Ba_x = -943.026, Ba_y = -100.32, Ba_z = -421.17;

// Gyro Calibration params
// bytearray(b'0.039483,-0.542183,9.724598,0.049353,0.005740,-0.016558,-0.152294,0.012844,-0.114679\n')
const static float Tg00 = 1.0, Tg01 = -0.0006364, Tg02 = 0.00508642;
const static float Tg10 = -0.00272323, Tg11 = 1.0, Tg12 = 0.00577732;
const static float Tg20 = 0.00824933, Tg21 = 0.0127456, Tg22 = 1.0;
const static float Sg_x = 0.00106156, Sg_y = 0.00105634, Sg_z = 0.00107057;
const static float Bg_x = 23.5762, Bg_y = 2.65462, Bg_z = -8.9182;


// Bm:
// [[-166.14858027]
//  [ 186.30477602]
//  [ 157.10313941]]
// Ainv:
// [[ 2.27519066e-03  1.88066651e-05  6.25771359e-05]
//  [ 1.88066651e-05  2.22321915e-03 -1.75871479e-06]
//  [ 6.25771359e-05 -1.75871479e-06  2.57840928e-03]]
// real_mag = Ainv * (raw_mag - Bm)

// R
// [[ 9.98554420e-01  1.59317702e-02 -5.13346769e-02]
//  [-1.59614459e-02  9.99872594e-01 -1.68150874e-04]
//  [ 5.13254576e-02  9.87283469e-04  9.98681492e-01]]
// mag_align = R * real_mag = R * Ainv * (raw_mag - Bm) = Tm * (raw_mag - Bm)
// Tm = R * Ainv
// const static float Tm00 = 2.26898894e-03, Tm01 = 5.42895782e-05, Tm02 = -6.99031511e-05;
// const static float Tm10 = -1.75215860e-05, Tm11 = 2.22263601e-03, Tm12 = -3.19087406e-06;
// const static float Tm20 = 1.79288397e-04, Tm21 = 1.40381230e-06, Tm22 = 2.57821969e-03;
const static float Tm00 = 2.27519066e-03, Tm01 = 1.88066651e-05, Tm02 = 6.25771359e-05;
const static float Tm10 = 1.88066651e-05, Tm11 = 2.22321915e-03, Tm12 = -1.75871479e-06;
const static float Tm20 = 6.25771359e-05, Tm21 = -1.75871479e-06, Tm22 = 2.57840928e-03;
const static float Bm_x = -166.14858027, Bm_y = 186.30477602, Bm_z = 157.10313941;

void SensorsInit(void);
void SensorsReadTask(void *param);

void IMUCalibration(const int16_t *acc_raw, const int16_t *gyro_raw, const int16_t *mag_raw, float *acc, float *gyro, float *mag);

#endif /* __SENSORS_H */
