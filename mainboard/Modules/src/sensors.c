#include "sensors.h"

xQueueHandle acc_queue;
STATIC_MEM_QUEUE_ALLOC(acc_queue, 1, sizeof(float) * 3);
xQueueHandle gyro_queue;
STATIC_MEM_QUEUE_ALLOC(gyro_queue, 1, sizeof(float) * 3);
xQueueHandle mag_queue;
STATIC_MEM_QUEUE_ALLOC(mag_queue, 1, sizeof(float) * 3);
xQueueHandle alt_queue;
STATIC_MEM_QUEUE_ALLOC(alt_queue, 1, sizeof(float));


bool SensorsInit(void) {
    printf("Init Sensors...\r\n");

    bool IS_MPU6050_OK = MPU6050Init();
    printf("MPU6050Init:  %u\r\n", IS_MPU6050_OK);

    bool IS_HMC5883_OK = HMC5883LInit();
    printf("HMC5883LInit: %u\r\n", IS_HMC5883_OK);

    bool IS_MS5611_OK = MS5611Init();
    printf("MS5611Init:   %u\r\n", IS_MS5611_OK);

    // printf("acc_lpf: b0:%f, b1:%f, b2:%f, a1:%f, a2:%f\r\n", acc_lpf[0].b0, acc_lpf[0].b1, acc_lpf[0].b2, acc_lpf[0].a1, acc_lpf[0].a2);
    // printf("gyro_lpf: b0:%f, b1:%f, b2:%f, a1:%f, a2:%f\r\n\r\n", gyro_lpf[0].b0, gyro_lpf[0].b1, gyro_lpf[0].b2, gyro_lpf[0].a1, gyro_lpf[0].a2);

    bool IS_SENSORS_OK = IS_MPU6050_OK & IS_HMC5883_OK & IS_MS5611_OK;
    if (IS_SENSORS_OK) {
        printf("Success to Init Sensors!\r\n\r\n");
    } else {
        printf("Fail to Init Sensors!\r\n\r\n"); 
    }

    acc_queue = STATIC_MEM_QUEUE_CREATE(acc_queue);
    gyro_queue = STATIC_MEM_QUEUE_CREATE(gyro_queue);
    mag_queue = STATIC_MEM_QUEUE_CREATE(mag_queue);
    alt_queue = STATIC_MEM_QUEUE_CREATE(alt_queue);

    return IS_SENSORS_OK;
}

void SensorsReadIMU(uint64_t *timestamp, float *acc, float *gyro) {
    xQueuePeek(acc_queue, acc, portMAX_DELAY);
    xQueuePeek(gyro_queue, gyro, portMAX_DELAY);
    *timestamp = GetSysTimeUs();
}