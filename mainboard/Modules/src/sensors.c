#include "sensors.h"

xQueueHandle acc_raw_queue;
STATIC_MEM_QUEUE_ALLOC(acc_raw_queue, 1, sizeof(float) * 3);
xQueueHandle gyro_raw_queue;
STATIC_MEM_QUEUE_ALLOC(gyro_raw_queue, 1, sizeof(float) * 3);
xQueueHandle mag_raw_queue;
STATIC_MEM_QUEUE_ALLOC(mag_raw_queue, 1, sizeof(float) * 3);

static TaskHandle_t sensors_task_handle;
STATIC_MEM_TASK_ALLOC(sensors_task, SENSORS_TASK_STACKSIZE);
const TickType_t sensors_task_wait = pdMS_TO_TICKS(1000);

float tmpx, tmpy, tmpz;

bool SensorsInit(void) {
    printf("Init Sensors...\r\n");

    bool IS_MPU6050_OK = MPU6050Init();
    printf("MPU6050Init:  %u\r\n", IS_MPU6050_OK);

    bool IS_HMC5883_OK = HMC5883LInit();
    printf("HMC5883LInit: %u\r\n", IS_HMC5883_OK);

    bool IS_MS5611_OK = MS5611Init();
    printf("MS5611Init:   %u\r\n", IS_MS5611_OK);

    // acc_raw_queue = STATIC_MEM_QUEUE_CREATE(acc_raw_queue);
    // gyro_raw_queue = STATIC_MEM_QUEUE_CREATE(gyro_raw_queue);
    // mag_raw_queue = STATIC_MEM_QUEUE_CREATE(mag_raw_queue);

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

    int16_t acc_raw[3], gyro_raw[3], mag_raw[3];
    float acc[3], gyro[3], mag[3];

    TickType_t xTimeBefore, xTotalTimeSuspended;

    portTickType last_wait_time = xTaskGetTickCount();
    for(;;) {
        MPU6050ReadAccelRaw(acc_raw);
        MPU6050ReadGyroRaw(gyro_raw);
        HMC5883LReadMagRaw(mag_raw);

        // printf("%u,%d,%d,%d,%d,%d,%d\n", pdTICKS_TO_MS(xTaskGetTickCount()), acc_raw[0], acc_raw[1], acc_raw[2], gyro_raw[0], gyro_raw[1], gyro_raw[2]);
        
        // xTimeBefore = xTaskGetTickCount();
        // vTaskDelay(pdMS_TO_TICKS(5));
        IMUCalibration(acc_raw, gyro_raw, mag_raw, acc, gyro, mag);
        // IMUCalibration(acc_raw, gyro_raw, mag_raw, acc, gyro, mag);
        // IMUCalibration(acc_raw, gyro_raw, mag_raw, acc, gyro, mag);
        // IMUCalibration(acc_raw, gyro_raw, mag_raw, acc, gyro, mag);
        // IMUCalibration(acc_raw, gyro_raw, mag_raw, acc, gyro, mag);
        // IMUCalibration(acc_raw, gyro_raw, mag_raw, acc, gyro, mag);
        // IMUCalibration(acc_raw, gyro_raw, mag_raw, acc, gyro, mag);
        // IMUCalibration(acc_raw, gyro_raw, mag_raw, acc, gyro, mag);
        // IMUCalibration(acc_raw, gyro_raw, mag_raw, acc, gyro, mag);
        // IMUCalibration(acc_raw, gyro_raw, mag_raw, acc, gyro, mag);
        // xTotalTimeSuspended = xTaskGetTickCount() - xTimeBefore;
        // printf("%lu\r\n", xTotalTimeSuspended);

        // xQueueOverwrite(acc_raw_queue, acc_raw);
        // xQueueOverwrite(gyro_raw_queue, gyro_raw);
        // xQueueOverwrite(mag_raw_queue, mag_raw);

        // printf("Accel: %d, %d, %d\r\n", acc_raw[0], acc_raw[1], acc_raw[2]);
        // printf("Gyro: %d, %d, %d\r\n", gyro_raw[0], gyro_raw[1], gyro_raw[2]);
        // printf("Mag: %d, %d, %d\r\n", mag_raw[0], mag_raw[1], mag_raw[2]);

        // printf("Accel: %f %f %f\r\n", acc[0], acc[1], acc[2]);
        // printf("Gyro : %f %f %f\r\n", gyro[0], gyro[1], gyro[2]);
        // printf("Mag  : %f %f %f\r\n\r\n", mag[0], mag[1], mag[2]);

        // printf("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", (float)pdTICKS_TO_MS(xTaskGetTickCount()), acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2], mag[0], mag[1], mag[2]);

        // printf("%f,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", (float)pdTICKS_TO_MS(xTaskGetTickCount()), acc_raw[0], acc_raw[1], acc_raw[2], gyro_raw[0], gyro_raw[1], gyro_raw[2], mag_raw[0], mag_raw[1], mag_raw[2]);

        vTaskDelayUntil(&last_wait_time, sensors_task_wait);
    }

    // int16_t* acc_raw  = pvPortMalloc(sizeof(int16_t) * 3);
    // int16_t* gyr_raw = pvPortMalloc(sizeof(int16_t) * 3);
    // int16_t* mag_raw = pvPortMalloc(sizeof(int16_t) * 3);
}

void IMUCalibration(const int16_t *acc_raw, const int16_t *gyro_raw, const int16_t *mag_raw, float *acc, float *gyro, float *mag) {
    tmpz = acc_raw[2] - Ba_z;
    tmpy = acc_raw[1] - Ba_y;
    tmpx = acc_raw[0] - Ba_x;

    acc[2] = tmpz;
    acc[1] = tmpy + Ma12 * tmpz;
    acc[0] = tmpx + Ma01 * tmpy + Ma02 * tmpz;

    tmpx = gyro_raw[0] - Bg_x;
    tmpy = gyro_raw[1] - Bg_y;
    tmpz = gyro_raw[2] - Bg_z;

    gyro[0] = Mg00 * tmpx + Mg01 * tmpy + Mg02 * tmpz;
    gyro[1] = Mg10 * tmpx + Mg11 * tmpy + Mg12 * tmpz;
    gyro[2] = Mg20 * tmpx + Mg21 * tmpy + Mg22 * tmpz;

    tmpx = mag_raw[0] - Bm_x;
    tmpy = mag_raw[1] - Bm_y;
    tmpz = mag_raw[2] - Bm_z;

    mag[0] = Mm00 * tmpx + Mm01 * tmpy + Mm02 * tmpz;
    mag[1] = Mm10 * tmpx + Mm11 * tmpy + Mm12 * tmpz;
    mag[2] = Mm20 * tmpx + Mm21 * tmpy + Mm22 * tmpz;
}
