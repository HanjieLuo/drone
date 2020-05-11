#ifndef __GY_86_H
#define __GY_86_H

#include "utils.h"
#include "mpu6050.h"
#include "ms5611.h"
#include "hmc5883l.h"

bool SensorsInit(void);
void SnsorsReadTask(void);


#define SensorsReadAccelRaw MPU6050ReadAccelRaw
#define SensorsReadGyroRaw MPU6050ReadGyroRaw
#define SensorsReadMagRaw HMC5883LReadMagRaw



#endif /* __GY_86_H */
