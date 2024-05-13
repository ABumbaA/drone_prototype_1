/*mpu6050.h*/
#ifndef DRONE_MPU6050_H_GUARD
#define DRONE_MPU6050_H_GUARD

#include "..\..\include\common.h"

/*Function Prototypes*/
void init_mpu_gyro();
void get_mpu_data(double *mpu_current_data);

#endif