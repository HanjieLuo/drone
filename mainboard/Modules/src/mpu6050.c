#include "mpu6050.h"

static uint8_t i2c_data;
static uint8_t i2c_datas[6];
static int16_t ax_raw, ay_raw, az_raw;
static int16_t gx_raw, gy_raw, gz_raw;
static int16_t celsius_raw;

bool MPU6050Init(void) {
    vTaskDelay(pdMS_TO_TICKS(200));

    // printf("MPU6050Init\r\n");
    if(HAL_I2C_IsDeviceReady(&hi2c1, MPU6050_DEV_ADDR, 10, 1000) != HAL_OK) {
        return false;
    }

    // printf("HAL_I2C_IsDeviceReady\r\n");

    // 0x68 will be returned by the sensor if everything goes well
    if (!I2C1MenRead(MPU6050_DEV_ADDR, MPU6050_WHO_AM_I, 1, &i2c_data, 1) || i2c_data != 0x68) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(5));

    // printf("MPU6050_WHO_AM_I\r\n");

    //复位 MPU6050
    i2c_data = 0X80;
    if (!I2C1MenWrite(MPU6050_DEV_ADDR, MPU6050_PWR_MGMT_1, 1, &i2c_data, 1)) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    // printf("MPU6050_PWR_MGMT_1\r\n");


    // power management register 0X6B we should write all 0's to wake the sensor up
    i2c_data = 0;
    if (!I2C1MenWrite(MPU6050_DEV_ADDR, MPU6050_PWR_MGMT_1, 1, &i2c_data, 1)) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    // printf("MPU6050_PWR_MGMT_1\r\n");

    //CLKSEL 3 (PLL with Z Gyro reference)
    //默认是使用内部 8M RC 晶振的，精度不高，所以我们一般选择 X/Y/Z 轴陀螺作为参考
    i2c_data = 0x03;
    if (!I2C1MenWrite(MPU6050_DEV_ADDR, MPU6050_PWR_MGMT_1, 1, &i2c_data, 1)) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(5));

    // printf("MPU6050_PWR_MGMT_1\r\n");

    // Set Gyroscope DATA RATE of 1KHz by writing SMPLRT_DIV register
    i2c_data = 0x07;
    if (!I2C1MenWrite(MPU6050_DEV_ADDR, MPU6050_SMPLRT_DIV, 1, &i2c_data, 1)) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(5));

    // printf("MPU6050_SMPLRT_DIV\r\n");

    // Set accelerometer configuration in ACCEL_CONFIG Register
    // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> ? 2g
    // 该寄存器我们只关心 AFS_SEL[1:0]这两个位，用于设置加速度传感器的满量程范围：0，
    // ±2g；1，±4g；2，±8g；3，±16g；我们一般设置为 0，即±2g，因为加速度传感器的
    // ADC 也是 16 位，所以得到灵敏度为：65536/4=16384LSB/g。
    i2c_data = 0x00;
    if (!I2C1MenWrite(MPU6050_DEV_ADDR, MPU6050_ACCEL_CONFIG, 1, &i2c_data, 1)) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(5));

    // printf("MPU6050_ACCEL_CONFIG\r\n");

    // Set Gyroscopic configuration in GYRO_CONFIG Register
    // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> ? 不自检，2000deg/s
    //     该寄存器我们只关心 FS_SEL[1:0]这两个位，用于设置陀螺仪的满量程范围：0，±250°
    // /S；1，±500°/S；2，±1000°/S；3，±2000°/S；我们一般设置为 3，即±2000°/S，因
    // 为陀螺仪的 ADC 为 16 位分辨率，所以得到灵敏度为：65536/4000=16.4LSB/(°/S)。
    i2c_data = 0x18;
    if (!I2C1MenWrite(MPU6050_DEV_ADDR, MPU6050_GYRO_CONFIG, 1, &i2c_data, 1)) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(5));

    // printf("MPU6050_GYRO_CONFIG\r\n");

    // enable I2C bypass
    i2c_data = 0x02;
    if (!I2C1MenWrite(MPU6050_DEV_ADDR, MPU6050_RA_INT_PIN_CFG, 1, &i2c_data, 1)) {
        return false;
    }

    // printf("MPU6050_RA_INT_PIN_CFG\r\n");

    // PMU6050_WriteReg(MPU6050_RA_INT_PIN_CFG, 0x02);
    // PMU6050_WriteReg(MPU6050_RA_USER_CTRL, 0x00);

    return true;
}

bool MPU6050ReadAccelRaw(int16_t *values) {
    if (!I2C1MenRead(MPU6050_DEV_ADDR, MPU6050_ACCEL_XOUT_H, 1, i2c_datas, 6)) {
        return false;
    }

	values[0] = (int16_t)(i2c_datas[0] << 8 | i2c_datas [1]);
	values[1] = (int16_t)(i2c_datas[2] << 8 | i2c_datas [3]);
	values[2] = (int16_t)(i2c_datas[4] << 8 | i2c_datas [5]);

    return true;  
}

bool MPU6050ReadGyroRaw(int16_t *values){
    if (!I2C1MenRead(MPU6050_DEV_ADDR, MPU6050_GYRO_XOUT_H, 1, i2c_datas, 6)) {
        return false;
    }

	values[0] = (int16_t)(i2c_datas[0] << 8 | i2c_datas [1]);
	values[1] = (int16_t)(i2c_datas[2] << 8 | i2c_datas [3]);
	values[2] = (int16_t)(i2c_datas[4] << 8 | i2c_datas [5]);

    return true;  
}

bool MPU6050ReadAccel(float *accel_x, float *accel_y, float *accel_z) {
    if (!I2C1MenRead(MPU6050_DEV_ADDR, MPU6050_ACCEL_XOUT_H, 1, i2c_datas, 6)) {
        return false;
    }

	ax_raw = (int16_t)(i2c_datas[0] << 8 | i2c_datas [1]);
	ay_raw = (int16_t)(i2c_datas[2] << 8 | i2c_datas [3]);
	az_raw = (int16_t)(i2c_datas[4] << 8 | i2c_datas [5]);

	/*** convert the RAW values into acceleration in 'g'
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 16384.0
	     for more details check ACCEL_CONFIG Register              ****/

	*accel_x = ax_raw / 16384.f;
	*accel_y = ay_raw / 16384.f;
	*accel_z = az_raw / 16384.f;

    return true;  
}

bool MPU6050ReadGyro(float *gx, float *gy, float *gz){
    if (!I2C1MenRead(MPU6050_DEV_ADDR, MPU6050_GYRO_XOUT_H, 1, i2c_datas, 6)) {
        return false;
    }

	gx_raw = (int16_t)(i2c_datas[0] << 8 | i2c_datas [1]);
	gy_raw = (int16_t)(i2c_datas[2] << 8 | i2c_datas [3]);
	gz_raw = (int16_t)(i2c_datas[4] << 8 | i2c_datas [5]);

	/*** convert the RAW values into dps (?/s)
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 131.0
	     for more details check GYRO_CONFIG Register              ****/

	*gx = gx_raw / 16.4f;
	*gy = gy_raw / 16.4f;
	*gz = gz_raw / 16.4f;

    return true;  
}

bool MPU6050ReadTemp(float *celsius) {
    if (!I2C1MenRead(MPU6050_DEV_ADDR, MPU6050_TEMP_OUT_H, 1, i2c_datas, 2)) {
        return false;
    }

	celsius_raw = (int16_t)(i2c_datas[0] << 8 | i2c_datas [1]);
	*celsius =  36.53 + (float)celsius_raw / 340.f;

    return true;      
}