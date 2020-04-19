#include "gy_86.h"

static uint8_t i2c_data;
static uint8_t i2c_datas[6];

static int16_t accel_x_raw, accel_y_raw, accel_z_raw;

void Gy86Init(void) {
}

uint8_t MPU6050Init(void) {
    HAL_Delay(1000);

    HAL_I2C_Mem_Read(&hi2c1, MPU6050_DEV_ADDR, MPU6050_WHO_AM_I, 1, &i2c_data, 1, 1000);

    // 0x68 will be returned by the sensor if everything goes well
    if (i2c_data != 0x68) return 0;

    //复位 MPU6050
    i2c_data = 0X80;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_DEV_ADDR, MPU6050_PWR_MGMT_1, 1, &i2c_data, 1, 1000);
    HAL_Delay(10);

    // power management register 0X6B we should write all 0's to wake the sensor up
    i2c_data = 0;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_DEV_ADDR, MPU6050_PWR_MGMT_1, 1, &i2c_data, 1, 1000);
    HAL_Delay(10);

    //CLKSEL 3 (PLL with Z Gyro reference)
    //默认是使用内部 8M RC 晶振的，精度不高，所以我们一般选择 X/Y/Z 轴陀螺作为参考
    i2c_data = 0x03;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_DEV_ADDR, MPU6050_PWR_MGMT_1, 1, &i2c_data, 1, 1000);
    HAL_Delay(5);

    // Set Gyroscope DATA RATE of 1KHz by writing SMPLRT_DIV register
    i2c_data = 0x07;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_DEV_ADDR, MPU6050_SMPLRT_DIV, 1, &i2c_data, 1, 1000);
    HAL_Delay(5);

    // Set accelerometer configuration in ACCEL_CONFIG Register
    // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> ? 2g
    // 该寄存器我们只关心 AFS_SEL[1:0]这两个位，用于设置加速度传感器的满量程范围：0，
    // ±2g；1，±4g；2，±8g；3，±16g；我们一般设置为 0，即±2g，因为加速度传感器的
    // ADC 也是 16 位，所以得到灵敏度为：65536/4=16384LSB/g。
    i2c_data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_DEV_ADDR, MPU6050_ACCEL_CONFIG, 1, &i2c_data, 1, 1000);
    HAL_Delay(5);

    // Set Gyroscopic configuration in GYRO_CONFIG Register
    // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> ? 不自检，2000deg/s
    //     该寄存器我们只关心 FS_SEL[1:0]这两个位，用于设置陀螺仪的满量程范围：0，±250°
    // /S；1，±500°/S；2，±1000°/S；3，±2000°/S；我们一般设置为 3，即±2000°/S，因
    // 为陀螺仪的 ADC 为 16 位分辨率，所以得到灵敏度为：65536/4000=16.4LSB/(°/S)。
    i2c_data = 0x18;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_DEV_ADDR, MPU6050_GYRO_CONFIG, 1, &i2c_data, 1, 1000);
    HAL_Delay(5);

    //// enable I2C bypass for AUX I2C
    i2c_data = 0x02;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_DEV_ADDR, MPU6050_RA_INT_PIN_CFG, 1, &i2c_data, 1, 1000);
    // HAL_Delay(50);

    // PMU6050_WriteReg(MPU6050_RA_INT_PIN_CFG, 0x02);
    // PMU6050_WriteReg(MPU6050_RA_USER_CTRL, 0x00);

    return 1;
}

void MPU6050ReadAccel(float *accel_x, float *accel_y, float *accel_z) {
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_DEV_ADDR, MPU6050_ACCEL_XOUT_H, 1, i2c_datas, 6, 1000);

	accel_x_raw = (int16_t)(i2c_datas[0] << 8 | i2c_datas [1]);
	accel_y_raw = (int16_t)(i2c_datas[2] << 8 | i2c_datas [3]);
	accel_z_raw = (int16_t)(i2c_datas[4] << 8 | i2c_datas [5]);

	/*** convert the RAW values into acceleration in 'g'
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 16384.0
	     for more details check ACCEL_CONFIG Register              ****/

	*accel_x = accel_x_raw / 16384.f;
	*accel_y = accel_y_raw / 16384.f;
	*accel_z = accel_z_raw / 16384.f;    
}