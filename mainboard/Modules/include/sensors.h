#ifndef __SENSORS_H
#define __SENSORS_H

#define ARM_MATH_CM4
#include "arm_math.h"

#include "FreeRTOS.h"
#include "hmc5883l.h"
#include "mpu6050.h"
#include "ms5611.h"
#include "queue.h"
#include "utils.h"
#include "system_task.h"

#define SensorsReadAccelRaw MPU6050ReadAccelRaw
#define SensorsReadGyroRaw MPU6050ReadGyroRaw
#define SensorsReadMagRaw HMC5883LReadMagRaw


// self.acc2real = 9.7833 / 16384.0 = 5.97125244140625e-4‬
// self.gyro2real = 0.00106422515
// self.mag2real = 0.00091743119


// Acc Calibration params
// acc = Ta * diag(Sa) * (acc_raw - Ba)
// Let Ma = Ta * diag(Sa);
// const static float Ta00 = 1.0, Ta01 = 9.0941e-05, Ta02 = -0.00458356;
// const static float Ta10 = 0.0, Ta11 = 1.0, Ta12 = -0.00704908;
// const static float Ta20 = 0.0, Ta21 = 0.0, Ta22 = 1.0;
// const static float Sa_x = -0.000596561, Sa_y = -0.000594778, Sa_z = -0.000594165;
const static float Ma00 =-5.9656100000e-0,      Ma01 = -5.4089706098e-08,   Ma02 = 2.7233909274e-06,
                 /*Ma10 = 0.0000000000e+00,*/   Ma11 = -5.9477800000e-04,   Ma12 = 4.1883166182e-06,
                 /*Ma20 = 0.0000000000e+00,*/ /*Ma21 =  0.0000000000e+00,*/ Ma22 =-5.9416500000e-04;
const static float Ba_x = -943.026, Ba_y = -100.32, Ba_z = -421.17;

// Gyro Calibration params
// gyro = Tg * diag(Sg) * (gyro_raw - Bg)
// Let Mg = Tg * diag(Sg);
// bytearray(b'0.039483,-0.542183,9.724598,0.049353,0.005740,-0.016558,-0.152294,0.012844,-0.114679\n')
// const static float Tg00 = 1.0, Tg01 = -0.0006364, Tg02 = 0.00508642;
// const static float Tg10 = -0.00272323, Tg11 = 1.0, Tg12 = 0.00577732;
// const static float Tg20 = 0.00824933, Tg21 = 0.0127456, Tg22 = 1.0;
// const static float Sg_x = 0.00106156, Sg_y = 0.00105634, Sg_z = 0.00107057;
const static float Mg00 = 1.0615600000000000e-03, Mg01 =-6.7225477599999996e-07, Mg02 = 5.4453686594000006e-06,
                   Mg10 =-2.8908720388000003e-06, Mg11 = 1.0563400000000000e-03, Mg12 = 6.1850254724000004e-06,
                   Mg20 = 8.7571587547999988e-06, Mg21 = 1.3463053299999999e-05, Mg22 = 1.0705700000000001e-03;
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
// [[ 0.9996613759 -0.0145143345  0.0215978633]
//  [ 0.0144040375  0.9998824543  0.0052536868]
//  [-0.0216715783 -0.0049408113  0.999752935]]
// mag_align = R * real_mag = R * Ainv * (raw_mag - Bm) = Mm * (raw_mag - Bm), where Mm = R * Ainv
const static float Mm00 = 2.2754987917e-03, Mm01 =-1.3506234138e-05, Mm02 = 1.1826960352e-04;
const static float Mm10 = 5.1905146781e-05, Mm11 = 2.2232194724e-03, Mm12 = 1.2689010139e-05;
const static float Mm20 = 1.3161782481e-05, Mm21 =-1.3150356768e-05, Mm22 = 2.5764247895e-03;
const static float Bm_x = -166.14858027, Bm_y = 186.30477602, Bm_z = 157.10313941;

bool SensorsInit(void);
void SensorsTask(void *param);

void IMUCalibration(const int16_t *acc_raw, const int16_t *gyro_raw, const int16_t *mag_raw, float *acc, float *gyro, float *mag);

#endif /* __SENSORS_H */
