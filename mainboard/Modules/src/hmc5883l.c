#include "hmc5883l.h"

static uint8_t i2c_data;
static uint8_t i2c_datas[6];
static int16_t mx_raw, my_raw, mz_raw;

bool HMC5883LInit(void) {
    if (HAL_I2C_IsDeviceReady(&hi2c1, HMC5883L_DEFAULT_ADDRESS, 10, 1000) != HAL_OK) {
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

bool HMC5883LReadMagRaw(int16_t *values) {
    if (!I2C1MenRead(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_DATAX_H, 1, i2c_datas, 6)) {
        return false;
    }

	values[0] = (int16_t)(i2c_datas[0] << 8 | i2c_datas [1]);
	values[1] = (int16_t)(i2c_datas[2] << 8 | i2c_datas [3]);
	values[2] = (int16_t)(i2c_datas[4] << 8 | i2c_datas [5]); 
    return true;  
}

bool HMC5883LReadMag(float *mx, float *my, float *mz) {
    if (!I2C1MenRead(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_DATAX_H, 1, i2c_datas, 6)) {
        return false;
    }

	mx_raw = (int16_t)(i2c_datas[0] << 8 | i2c_datas [1]);
	my_raw = (int16_t)(i2c_datas[2] << 8 | i2c_datas [3]);
	mz_raw = (int16_t)(i2c_datas[4] << 8 | i2c_datas [5]);

	*mx = mx_raw / 1090.f;
	*my = my_raw / 1090.f;
	*mz = mz_raw / 1090.f;

    return true;
}

