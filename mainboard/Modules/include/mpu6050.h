#ifndef __MPU6050_H
#define __MPU6050_H

#include "i2c1.h"
#include "utils.h"

#define MPU6050_DEV_ADDR 0xD0  // 6050 器件地址
//-----------------------------------------
// 定义MPU6050内部地址
//-----------------------------------------
#define MPU6050_SMPLRT_DIV 0x19    //陀螺仪采样率，典型值：0x07(125Hz)
#define MPU6050_CONFIG 0x1A        //低通滤波频率，典型值：0x06(5Hz)
#define MPU6050_GYRO_CONFIG 0x1B   //陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define MPU6050_ACCEL_CONFIG 0x1C  //加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)

/* 加速度相关寄存器地址 */
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_XOUT_L 0x3C
#define MPU6050_ACCEL_YOUT_H 0x3D
#define MPU6050_ACCEL_YOUT_L 0x3E
#define MPU6050_ACCEL_ZOUT_H 0x3F
#define MPU6050_ACCEL_ZOUT_L 0x40

/* 温度相关寄存器地址 */
#define MPU6050_TEMP_OUT_H 0x41
#define MPU6050_TEMP_OUT_L 0x42

/* 陀螺仪相关寄存器地址 */
#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_GYRO_XOUT_L 0x44
#define MPU6050_GYRO_YOUT_H 0x45
#define MPU6050_GYRO_YOUT_L 0x46
#define MPU6050_GYRO_ZOUT_H 0x47
#define MPU6050_GYRO_ZOUT_L 0x48

#define MPU6050_PWR_MGMT_1 0x6B    //电源管理，典型值：0x00(正常启用)
#define MPU6050_WHO_AM_I 0x75      //IIC地址寄存器(默认数值0x68，只读)
#define MPU6050_SlaveAddress 0xD0  //IIC写入时的地址字节数据，+1为读取
#define MPU6050_RA_INT_PIN_CFG 0x37

bool MPU6050Init(void);
bool MPU6050ReadAccel(float *ax, float *ay, float *az);

#endif /* __MPU6050_H */
