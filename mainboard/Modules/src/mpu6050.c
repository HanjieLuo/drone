#include "mpu6050.h"

//-----------------------------------------
// Calibration
//-----------------------------------------
// Dataset: 1589818352, gravity: 9.803
// Acc Calibration params
// acc = Ta * diag(Sa) * (acc_raw - Ba)
// Let Ma = Ta * diag(Sa);
// const static float Ta00 = 1.0, Ta01 = 9.0941e-05, Ta02 = -0.00458356;
// const static float Ta10 = 0.0, Ta11 = 1.0, Ta12 = -0.00704908;
// const static float Ta20 = 0.0, Ta21 = 0.0, Ta22 = 1.0;
// const static float Sa_x = -0.000596561, Sa_y = -0.000594778, Sa_z = -0.000594165;
const static float Ma00 =-5.9656100000e-04,     Ma01 = -5.4089706098e-08,   Ma02 = 2.7233909274e-06,
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


static uint8_t i2c_data;
static uint8_t i2c_datas[6];
// static int16_t ax_raw, ay_raw, az_raw;
// static int16_t gx_raw, gy_raw, gz_raw;
// static int16_t celsius_raw;

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

    // DEVICE_RESET  resets all internal registers to their default values. No acknowledge!
    i2c_data = 0X80;
    if(HAL_I2C_Mem_Write_DMA(&hi2c1, MPU6050_DEV_ADDR, MPU6050_PWR_MGMT_1, 1, &i2c_data, 1) != HAL_OK) {
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

	int16_t ax_raw = (int16_t)(i2c_datas[0] << 8 | i2c_datas [1]);
	int16_t ay_raw = (int16_t)(i2c_datas[2] << 8 | i2c_datas [3]);
	int16_t az_raw = (int16_t)(i2c_datas[4] << 8 | i2c_datas [5]);

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

	int16_t gx_raw = (int16_t)(i2c_datas[0] << 8 | i2c_datas [1]);
	int16_t gy_raw = (int16_t)(i2c_datas[2] << 8 | i2c_datas [3]);
	int16_t gz_raw = (int16_t)(i2c_datas[4] << 8 | i2c_datas [5]);

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

	int16_t celsius_raw = (int16_t)(i2c_datas[0] << 8 | i2c_datas [1]);
	*celsius =  36.53 + (float)celsius_raw / 340.f;

    return true;      
}

void CalibrateIMU(const int16_t *acc_raw, const int16_t *gyro_raw, float *acc, float *gyro) {
    // 1.9us
    float tmpz = acc_raw[2] - Ba_z;
    float tmpy = acc_raw[1] - Ba_y;
    float tmpx = acc_raw[0] - Ba_x;

    acc[2] = Ma22 * tmpz;
    acc[1] = Ma11 * tmpy + Ma12 * tmpz;
    acc[0] = Ma00 * tmpx + Ma01 * tmpy + Ma02 * tmpz;

    tmpx = gyro_raw[0] - Bg_x;
    tmpy = gyro_raw[1] - Bg_y;
    tmpz = gyro_raw[2] - Bg_z;

    gyro[0] = Mg00 * tmpx + Mg01 * tmpy + Mg02 * tmpz;
    gyro[1] = Mg10 * tmpx + Mg11 * tmpy + Mg12 * tmpz;
    gyro[2] = Mg20 * tmpx + Mg21 * tmpy + Mg22 * tmpz;


    /* DSP version 1.39us
    arm_matrix_instance_f32 m_imu_data, m_B, m_unbias, m_M;
    float32_t imu_data[9], B_data[9], unbias_data[9];

    B_data[0] = Ba_x; B_data[1] = Ba_y; B_data[2] = Ba_z;
    B_data[3] = Bg_x; B_data[4] = Bg_y; B_data[5] = Bg_z;
    B_data[6] = Bm_x; B_data[7] = Bm_y; B_data[8] = Bm_z;

    float32_t M_data[81] = {0};
    M_data[0] = Ma00; M_data[1] = Ma01; M_data[2] = Ma02;
    M_data[10] = Ma11; M_data[11] = Ma12;
    M_data[20] = Ma22;
    M_data[30] = Mg00; M_data[31] = Mg01; M_data[32] = Mg02;
    M_data[39] = Mg10; M_data[40] = Mg11; M_data[41] = Mg12;
    M_data[48] = Mg20; M_data[49] = Mg21; M_data[50] = Mg22;
    M_data[60] = Mm00; M_data[61] = Mm01; M_data[62] = Mm02;
    M_data[69] = Mm10; M_data[70] = Mm11; M_data[71] = Mm12;
    M_data[78] = Mm20; M_data[79] = Mm21; M_data[80] = Mm22;


    acc[0] = Ma00 * tmpx + Ma01 * tmpy + Ma02 * tmpz;
    acc[1] = Ma11 * tmpy + Ma12 * tmpz;
    acc[2] = Ma22 * tmpz;

    arm_mat_init_f32(&m_B, 9, 1, B_data);
    arm_mat_init_f32(&m_unbias, 9, 1, unbias_data);
    arm_mat_init_f32(&m_M, 9, 9, M_data);

    imu_data[0] = (float)(acc_raw[0]);
    imu_data[1] = (float)(acc_raw[1]);
    imu_data[2] = (float)(acc_raw[2]);
    imu_data[3] = (float)(gyro_raw[0]);
    imu_data[4] = (float)(gyro_raw[1]);
    imu_data[5] = (float)(gyro_raw[2]);
    imu_data[6] = (float)(mag_raw[0]);
    imu_data[7] = (float)(mag_raw[1]);
    imu_data[8] = (float)(mag_raw[2]);

    arm_mat_init_f32(&m_imu_data, 9, 1, imu_data);
    arm_mat_sub_f32(&m_imu_data, &m_B, &m_unbias);
    arm_mat_mult_f32(&m_M, &m_unbias, &imu_data);

    */

}