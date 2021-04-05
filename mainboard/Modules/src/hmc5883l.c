#include "hmc5883l.h"

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
const static float Mm00 = 2.2754987917e-03, Mm01 = -1.3506234138e-05, Mm02 = 1.1826960352e-04;
const static float Mm10 = 5.1905146781e-05, Mm11 = 2.2232194724e-03, Mm12 = 1.2689010139e-05;
const static float Mm20 = 1.3161782481e-05, Mm21 = -1.3150356768e-05, Mm22 = 2.5764247895e-03;
const static float Bm_x = -166.14858027, Bm_y = 186.30477602, Bm_z = 157.10313941;

static uint8_t i2c_data;
static uint8_t i2c_datas[6];
// static int16_t mx_raw, my_raw, mz_raw;

bool HMC5883LInit(void)
{
    if (HAL_I2C_IsDeviceReady(&hi2c1, HMC5883L_DEFAULT_ADDRESS, 10, 1000) != HAL_OK)
    {
        return false;
    }

    // printf("HMC5883LInit\r\n");

    i2c_data = (HMC5883L_AVERAGING_8 << (HMC5883L_CRA_AVERAGE_BIT - HMC5883L_CRA_AVERAGE_LENGTH + 1)) |
               (HMC5883L_RATE_75 << (HMC5883L_CRA_RATE_BIT - HMC5883L_CRA_RATE_LENGTH + 1)) |
               (HMC5883L_BIAS_NORMAL << (HMC5883L_CRA_BIAS_BIT - HMC5883L_CRA_BIAS_LENGTH + 1));
    I2C1MenWrite(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_CONFIG_A, 1, &i2c_data, 1);

    i2c_data = HMC5883L_GAIN_1090 << (HMC5883L_CRB_GAIN_BIT - HMC5883L_CRB_GAIN_LENGTH + 1);
    I2C1MenWrite(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_CONFIG_B, 1, &i2c_data, 1);

    i2c_data = HMC5883L_MODE_CONTINUOUS << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1);
    I2C1MenWrite(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_MODE, 1, &i2c_data, 1);

    return true;
}

bool HMC5883LReadMagRaw(int16_t *values)
{
    if (!I2C1MenRead(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_DATAX_H, 1, i2c_datas, 6))
    {
        return false;
    }

    values[0] = (int16_t)(i2c_datas[0] << 8 | i2c_datas[1]);
    values[1] = (int16_t)(i2c_datas[4] << 8 | i2c_datas[5]);
    values[2] = (int16_t)(i2c_datas[2] << 8 | i2c_datas[3]);
    return true;
}

bool HMC5883LReadMag(float *mx, float *my, float *mz)
{
    if (!I2C1MenRead(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_DATAX_H, 1, i2c_datas, 6))
    {
        return false;
    }

    int16_t mx_raw = (int16_t)(i2c_datas[0] << 8 | i2c_datas[1]);
    int16_t mz_raw = (int16_t)(i2c_datas[2] << 8 | i2c_datas[3]);
    int16_t my_raw = (int16_t)(i2c_datas[4] << 8 | i2c_datas[5]);

    *mx = mx_raw / 1090.f;
    *my = my_raw / 1090.f;
    *mz = mz_raw / 1090.f;

    return true;
}

void CalibrateMag(const int16_t *mag_raw, float *mag)
{
    float tmpx = mag_raw[0] - Bm_x;
    float tmpy = mag_raw[1] - Bm_y;
    float tmpz = mag_raw[2] - Bm_z;

    mag[0] = Mm00 * tmpx + Mm01 * tmpy + Mm02 * tmpz;
    mag[1] = Mm10 * tmpx + Mm11 * tmpy + Mm12 * tmpz;
    mag[2] = Mm20 * tmpx + Mm21 * tmpy + Mm22 * tmpz;
}
