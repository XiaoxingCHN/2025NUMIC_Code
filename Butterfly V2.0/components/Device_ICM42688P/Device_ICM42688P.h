#ifndef __DEVICE_ICM42688P_H__
#define __DEVICE_ICM42688P_H__

#include "Device_ICM42688P_Register.h"
#include "stdint.h"

typedef struct{

    int16_t accX;
    int16_t accY;
    int16_t accZ;

}ICM42688P_ACCELDATA_Def;//存储加速度数据的结构体

typedef struct{

    int16_t gyroX;
    int16_t gyroY;
    int16_t gyroZ;
    
}ICM42688P_GYRODATA_Def;//存储陀螺仪数据的结构体

typedef struct{

    ICM42688P_ACCELDATA_Def accData;
    ICM42688P_GYRODATA_Def gyroData;

}ICM42688P_DATA_Def;//存储传感器数据的结构体

#endif // __DEVICE_ICM42688P_H__
