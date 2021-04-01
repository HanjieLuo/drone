#include "sensors.h"

// self.acc2real = 9.7833 / 16384.0 = 5.97125244140625e-4‬
// self.gyro2real = 0.00106422515
// self.mag2real = 0.00091743119

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

static Lowpass2Data acc_lpf[3];
static Lowpass2Data gyro_lpf[3];

xQueueHandle acc_queue;
STATIC_MEM_QUEUE_ALLOC(acc_queue, 1, sizeof(float) * 3);
xQueueHandle gyro_queue;
STATIC_MEM_QUEUE_ALLOC(gyro_queue, 1, sizeof(float) * 3);
xQueueHandle mag_queue;
STATIC_MEM_QUEUE_ALLOC(mag_queue, 1, sizeof(float) * 3);
xQueueHandle alt_queue;
STATIC_MEM_QUEUE_ALLOC(alt_queue, 1, sizeof(float));

static TaskHandle_t sensors_task_handle;
STATIC_MEM_TASK_ALLOC(sensors_task, SENSORS_TASK_STACKSIZE);

const TickType_t sensors_task_wait = pdMS_TO_TICKS(1000.0 / IMU_SAMPLE_FREQ);
const static uint8_t mag_meas_delay = IMU_SAMPLE_FREQ / MAG_SAMPLE_FREQ;
const static uint8_t alt_meas_delay = IMU_SAMPLE_FREQ / ALT_SAMPLE_FREQ;

bool SensorsInit(void) {
    printf("Init Sensors...\r\n");

    bool IS_MPU6050_OK = MPU6050Init();
    printf("MPU6050Init:  %u\r\n", IS_MPU6050_OK);

    bool IS_HMC5883_OK = HMC5883LInit();
    printf("HMC5883LInit: %u\r\n", IS_HMC5883_OK);

    bool IS_MS5611_OK = MS5611Init();
    printf("MS5611Init:   %u\r\n", IS_MS5611_OK);

    for(unsigned int i = 0; i < 3; i++) {
        Lowpass2Init(&acc_lpf[i], IMU_SAMPLE_FREQ, ACC_LPF_CUTOFF_FREQ);
        Lowpass2Init(&gyro_lpf[i], IMU_SAMPLE_FREQ, GYRO_LPF_CUTOFF_FREQ);
    }

    // printf("acc_lpf: b0:%f, b1:%f, b2:%f, a1:%f, a2:%f\r\n", acc_lpf[0].b0, acc_lpf[0].b1, acc_lpf[0].b2, acc_lpf[0].a1, acc_lpf[0].a2);
    // printf("gyro_lpf: b0:%f, b1:%f, b2:%f, a1:%f, a2:%f\r\n\r\n", gyro_lpf[0].b0, gyro_lpf[0].b1, gyro_lpf[0].b2, gyro_lpf[0].a1, gyro_lpf[0].a2);

    acc_queue = STATIC_MEM_QUEUE_CREATE(acc_queue);
    gyro_queue = STATIC_MEM_QUEUE_CREATE(gyro_queue);
    mag_queue = STATIC_MEM_QUEUE_CREATE(mag_queue);
    alt_queue = STATIC_MEM_QUEUE_CREATE(alt_queue);

    sensors_task_handle = STATIC_MEM_TASK_CREATE(sensors_task, SensorsTask, "SensorsTask", NULL, SENSORS_TASK_PRI);

    bool IS_SENSORS_OK = IS_MPU6050_OK & IS_HMC5883_OK & IS_MS5611_OK;
    if (IS_SENSORS_OK) {
        printf("Success to Init Sensors!\r\n\r\n");
    } else {
        printf("Fail to Init Sensors!\r\n\r\n"); 
    }

    return IS_SENSORS_OK;
}

void SensorsTask(void *param) {
    SystemWaitStart();

    int16_t acc_raw[3] = {0}, gyro_raw[3] = {0}, mag_raw[3] = {0};
    float alt = 0.f, acc[3] = {0}, gyro[3] = {0}, mag[3] = {0};

    uint8_t mag_meas_count = mag_meas_delay;
    uint8_t alt_meas_count = alt_meas_delay;

    portTickType last_wait_time = xTaskGetTickCount();

    while(1) {
        MPU6050ReadAccelRaw(acc_raw);
        MPU6050ReadGyroRaw(gyro_raw);
      
        CalibrateIMU(acc_raw, gyro_raw, acc, gyro);

        Lowpass2Apply3Axis((Lowpass2Data*)(&acc_lpf), acc);
        Lowpass2Apply3Axis((Lowpass2Data*)(&gyro_lpf), gyro);

        xQueueOverwrite(acc_queue, acc);
        xQueueOverwrite(gyro_queue, gyro);

        if (--mag_meas_count == 0) {
            HMC5883LReadMagRaw(mag_raw);
            CalibrateMag(mag_raw, mag);
            xQueueOverwrite(mag_queue, mag);
            mag_meas_count = mag_meas_delay;
        }

        if (--alt_meas_count == 0) {
            MS5611ReadAlt(&alt);
            xQueueOverwrite(alt_queue, &alt);
            alt_meas_count = alt_meas_delay;
        } 

        // printf("%lu,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", (unsigned long)pdTICKS_TO_MS(xTaskGetTickCount()), acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2], mag[0], mag[1], mag[2], alt); 
        // printf("%lu\n", (unsigned long)pdTICKS_TO_MS(xTaskGetTickCount())); 
        vTaskDelayUntil(&last_wait_time, sensors_task_wait);

        // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
        // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);

        // vTaskDelay(pdMS_TO_TICKS(5));

        // TickType_t xTimeBefore, xTotalTimeSuspended;
        // xTimeBefore = xTaskGetTickCount();
        // xTotalTimeSuspended = xTaskGetTickCount() - xTimeBefore;
        // printf("%lu\r\n", xTotalTimeSuspended);


        // printf("Accel: %d, %d, %d\r\n", acc_raw[0], acc_raw[1], acc_raw[2]);
        // printf("Gyro: %d, %d, %d\r\n", gyro_raw[0], gyro_raw[1], gyro_raw[2]);
        // printf("Mag: %d, %d, %d\r\n", mag_raw[0], mag_raw[1], mag_raw[2]);

        // printf("Accel: %f %f %f\r\n", acc[0], acc[1], acc[2]);
        // printf("Gyro : %f %f %f\r\n", gyro[0], gyro[1], gyro[2]);
        // printf("Mag  : %f %f %f\r\n\r\n", mag[0], mag[1], mag[2]);

        // printf("%u,%d,%d,%d,%d,%d,%d\n", pdTICKS_TO_MS(xTaskGetTickCount()), acc_raw[0], acc_raw[1], acc_raw[2], gyro_raw[0], gyro_raw[1], gyro_raw[2]);

        // printf("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", (float)pdTICKS_TO_MS(xTaskGetTickCount()), acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2], mag[0], mag[1], mag[2]); 

        // printf("%f,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", (float)pdTICKS_TO_MS(xTaskGetTickCount()), acc_raw[0], acc_raw[1], acc_raw[2], gyro_raw[0], gyro_raw[1], gyro_raw[2], mag_raw[0], mag_raw[1], mag_raw[2]);

    }

    // int16_t* acc_raw  = pvPortMalloc(sizeof(int16_t) * 3);
    // int16_t* gyr_raw = pvPortMalloc(sizeof(int16_t) * 3);
    // int16_t* mag_raw = pvPortMalloc(sizeof(int16_t) * 3);
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

void CalibrateMag(const int16_t *mag_raw, float *mag) {
    float tmpx = mag_raw[0] - Bm_x;
    float tmpy = mag_raw[1] - Bm_y;
    float tmpz = mag_raw[2] - Bm_z;

    mag[0] = Mm00 * tmpx + Mm01 * tmpy + Mm02 * tmpz;
    mag[1] = Mm10 * tmpx + Mm11 * tmpy + Mm12 * tmpz;
    mag[2] = Mm20 * tmpx + Mm21 * tmpy + Mm22 * tmpz;
}

