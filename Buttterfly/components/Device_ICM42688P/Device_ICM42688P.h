#ifndef __DEVICE_ICM42688P_H__
#define __DEVICE_ICM42688P_H__

#include "driver/spi_master.h"
#include "stdint.h"
#include "string.h"

#define ICM42688_INT1 5 // 中断引脚

#define SPI_SCLK_FREQ 10 * 1000 * 1000 // SPI 时钟频率 10MHz

#define SPI_CS_PIN      4
#define SPI_SCLK_PIN    3
#define SPI_MOSI_PIN    2
#define SPI_MISO_PIN    1
#define SPI_HOST_ID     SPI2_HOST
// #define ICM42688P_ADDR 0x68 // I2C 地址 (SDO悬空/接高时地址为0x19，左移1位后为0x32) SPI下无用可注释掉

/* === LSB 系数定义 === */
/* 加速度计量程对应的灵敏度系数 (单位: mg/LSB) */
#define LSB_ACC_2G      0.061f    /* ±2g 量程下的灵敏度 (mg/LSB) */
#define LSB_ACC_4G      0.122f    /* ±4g 量程下的灵敏度 (mg/LSB) */
#define LSB_ACC_8G      0.244f    /* ±8g 量程下的灵敏度 (mg/LSB) */
#define LSB_ACC_16G     0.488f    /* ±16g 量程下的灵敏度 (mg/LSB) */

/* 陀螺仪量程对应的灵敏度系数 (单位: mdps/LSB) */
#define LSB_GYRO_125DPS     3.8125f    /* ±125dps 量程下的灵敏度 (mdps/LSB) */
#define LSB_GYRO_250DPS     7.625f     /* ±250dps 量程下的灵敏度 (mdps/LSB) */
#define LSB_GYRO_500DPS     15.25f     /* ±500dps 量程下的灵敏度 (mdps/LSB) */
#define LSB_GYRO_1000DPS    30.5f      /* ±1000dps 量程下的灵敏度 (mdps/LSB) */
#define LSB_GYRO_2000DPS    61.0f      /* ±2000dps 量程下的灵敏度 (mdps/LSB) */

/* === 通用寄存器 (Bank 0) === */
#define ICM42688_WHO_AM_I               0x75    /* 设备ID寄存器，固定返回0x47 */
#define ICM42688_REG_BANK_SEL           0x76    /* 寄存器段选择 */

/* 接口/复位相关 */
#define ICM42688_DEVICE_CONFIG          0x11    /* 写0x01软复位 */
#define ICM42688_INTF_CONFIG0           0x4C    /* 接口配置：SPI 4/3 线等 */

/* 电源与配置 */
#define ICM42688_PWR_MGMT0              0x4E    /* 电源管理，开关陀螺/加速度计 */
#define ICM42688_GYRO_CONFIG0           0x4F    /* 陀螺仪 ODR/量程 */
#define ICM42688_ACCEL_CONFIG0          0x50    /* 加速度计 ODR/量程 */

/* 数据寄存器（Bank0） */
#define ICM42688_TEMP_DATA1             0x1D
#define ICM42688_TEMP_DATA0             0x1E
#define ICM42688_ACCEL_DATA_X1          0x1F
#define ICM42688_ACCEL_DATA_X0          0x20
#define ICM42688_ACCEL_DATA_Y1          0x21
#define ICM42688_ACCEL_DATA_Y0          0x22
#define ICM42688_ACCEL_DATA_Z1          0x23
#define ICM42688_ACCEL_DATA_Z0          0x24
#define ICM42688_GYRO_DATA_X1           0x25
#define ICM42688_GYRO_DATA_X0           0x26
#define ICM42688_GYRO_DATA_Y1           0x27
#define ICM42688_GYRO_DATA_Y0           0x28
#define ICM42688_GYRO_DATA_Z1           0x29
#define ICM42688_GYRO_DATA_Z0           0x2A

// 加速度计的量程范围
enum ICM42688P_AFS {
    ICM42688P_AFS_2G = 0,  // ±2g
    ICM42688P_AFS_4G = 1,  // ±4g  
    ICM42688P_AFS_8G = 2,  // ±8g
    ICM42688P_AFS_16G = 3, // ±16g (默认)
    NUM_ICM42688P_AFS
};

// 加速度计的输出数据速率
enum ICM42688P_AODR {
    ICM42688P_AODR_0_78HZ = 0x01,   // 低功耗模式 0.78Hz
    ICM42688P_AODR_1_5HZ = 0x02,    // 低功耗模式 1.5Hz
    ICM42688P_AODR_3_125HZ = 0x03,  // 低功耗模式 3.125Hz
    ICM42688P_AODR_6_25HZ = 0x04,   // 低功耗模式 6.25Hz
    ICM42688P_AODR_12_5HZ = 0x05,   // 高性能/低功耗模式 12.5Hz
    ICM42688P_AODR_25HZ = 0x06,     // 高性能/低功耗模式 25Hz
    ICM42688P_AODR_50HZ = 0x07,     // 高性能/低功耗模式 50Hz
    ICM42688P_AODR_100HZ = 0x08,    // 高性能/低功耗模式 100Hz
    ICM42688P_AODR_200HZ = 0x09,    // 高性能/低功耗模式 200Hz
    ICM42688P_AODR_400HZ = 0x0A,    // 高性能/低功耗模式 400Hz
    ICM42688P_AODR_800HZ = 0x0B,    // 高性能/低功耗模式 800Hz
    ICM42688P_AODR_1600HZ = 0x0C,   // 高性能模式 1600Hz
    NUM_ICM42688P_AODR
};

// 陀螺仪量程范围
enum ICM42688P_GFS {
    ICM42688P_GFS_125DPS = 0x04,   // ±125dps
    ICM42688P_GFS_250DPS = 0x03,   // ±250dps
    ICM42688P_GFS_500DPS = 0x02,   // ±500dps
    ICM42688P_GFS_1000DPS = 0x01,  // ±1000dps
    ICM42688P_GFS_2000DPS = 0x00,  // ±2000dps (默认)
    NUM_ICM42688P_GFS
};

// 陀螺仪输出速率
enum ICM42688P_GODR {
    ICM42688P_GODR_25HZ = 0x06,    // 高性能/正常/低功耗模式 25Hz
    ICM42688P_GODR_50HZ = 0x07,    // 高性能/正常/低功耗模式 50Hz
    ICM42688P_GODR_100HZ = 0x08,   // 高性能/正常/低功耗模式 100Hz
    ICM42688P_GODR_200HZ = 0x09,   // 高性能/正常模式 200Hz
    ICM42688P_GODR_400HZ = 0x0A,   // 高性能/正常模式 400Hz
    ICM42688P_GODR_800HZ = 0x0B,   // 高性能/正常模式 800Hz
    ICM42688P_GODR_1600HZ = 0x0C,  // 高性能/正常模式 1600Hz
    ICM42688P_GODR_3200HZ = 0x0D,  // 高性能/正常模式 3200Hz
    NUM_ICM42688P_GODR
};

/* 工作模式定义 */
enum ICM42688P_MODE {
    ICM42688P_MODE_STANDBY,     // 待机模式
    ICM42688P_MODE_LOW_POWER,   // 低功耗模式
    ICM42688P_MODE_NORMAL,      // 正常模式
    ICM42688P_MODE_HIGH_PERF,   // 高性能模式
    NUM_ICM42688P_MODE
};

typedef struct{

    float ax; // 加速度X轴数据 (单位: g)
    float ay; // 加速度Y轴数据 (单位: g)
    float az; // 加速度Z轴数据 (单位: g)

}ICM42688P_ACC_Def;

typedef struct{

    float gx; // 陀螺仪X轴数据 (单位: dps)
    float gy; // 陀螺仪Y轴数据 (单位: dps)
    float gz; // 陀螺仪Z轴数据 (单位: dps)

}ICM42688P_GYR_Def;

typedef struct{

    ICM42688P_ACC_Def Acc; // 加速度数据
    ICM42688P_GYR_Def Gyr; // 陀螺仪数据

}ICM42688P_Data_Def;

void Dev_ICM42688P_Init();
void Dev_ICM42688P_GetData(ICM42688P_Data_Def *data);
void Dev_ICM42688P_ZeroCalibrate(ICM42688P_Data_Def *dataZero);
#endif // __DEVICE_ICM42688P_H__
