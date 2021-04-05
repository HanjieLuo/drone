#ifndef __CONFIG_H
#define __CONFIG_H

// #define USE_SISTEMVEIW 1

// Task priorities. Higher number higher priority
// Max: configMAX_PRIORITIES
#define SYSTEMVIEW_TASK_PRI         6
#define SENSORS_TASK_PRI            5
#define STATE_ESTIMATOR_TASK_PRI    4
#define SYSTEM_TASK_PRI             2

//Task stack sizes
#define SYSTEM_TASK_STACKSIZE           (2 * configMINIMAL_STACK_SIZE)
#define SENSORS_TASK_STACKSIZE          (2 * configMINIMAL_STACK_SIZE)
#define STATE_ESTIMATOR_TASK_STACKSIZE  (2 * configMINIMAL_STACK_SIZE)
#define SYSTEMVIEW_TASK_STACKSIZE       (2 * configMINIMAL_STACK_SIZE)

// HZ
#define IMU_SAMPLE_FREQ 1000    //MPU6050
#define MAG_SAMPLE_FREQ 75      //HMC5883L
#define ALT_SAMPLE_FREQ 100     //MS5611

//陀螺仪低通滤波截止频率
//二次低通滤波的陀螺仪数据用于角速度环控制
//频率过低，信号延迟大，会导致控制发散
//频率过高，信号噪声大，会增加高频抖动
//要达到最佳效果，需经调试选取最佳的截止频率
//飞行器越小，系统惯性越小，所需的控制频率越高，对信号实时性的要求也就越高
//450轴距左右的飞机，截止频率不低于50就好，要根据实际情况选取，比如有时过软的硬件减震措施也会增加实际信号延迟
//Crazyfile: 80, BlueSkyFight: 88
#define GYRO_LPF_CUTOFF_FREQ  80
#define ACC_LPF_CUTOFF_FREQ 30

#endif /* __CONFIG_H */

