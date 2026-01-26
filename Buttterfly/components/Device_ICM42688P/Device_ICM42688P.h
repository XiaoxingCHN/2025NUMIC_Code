#ifndef __DEVICE_ICM42688P_H__
#define __DEVICE_ICM42688P_H__

#define ICM42688_INT1 5 // 中断引脚

#define SPI_SCLK_FREQ 10 * 1000 * 1000 // SPI 时钟频率 10MHz

#define SPI_CS_PIN 4
#define SPI_SCLK_PIN 3
#define SPI_MOSI_PIN 2
#define SPI_MISO_PIN 1

#define ICM42688P_ADDR 0x68 // I2C 地址

/* === LSB 系数定义 === */
/* 加速度计量程对应的灵敏度系数 (单位: g/LSB, 即 重力加速度/最低有效位) 
   公式: 量程 / 2^(ADC位数)。ICM42688加速度计ADC为16位。 
   例如: ±16g 量程，总范围为32g，则 LSB = 32g / 65536 = 0.0047856934 g/LSB */
#define LSB_ACC_16G		0.0047856934f	/* ±16g 量程下的灵敏度 (g/LSB) */
#define LSB_ACC_8G		0.0023928467f	/* ±8g 量程下的灵敏度 (g/LSB) */
#define LSB_ACC_4G		0.0011964233f	/* ±4g 量程下的灵敏度 (g/LSB) */
#define LSB_ACC_2G		0.00059821167f	/* ±2g 量程下的灵敏度 (g/LSB) */

/* 陀螺仪量程对应的灵敏度系数 (单位: rad/s/LSB, 即 弧度/秒/最低有效位)
   公式: (量程*π)/(180 * 2^(ADC位数-1))。ICM42688陀螺仪ADC为16位。
   例如: ±2000dps 量程，转换为弧度: 2000*π/180 ≈ 34.906585 rad/s，
   则 LSB = 34.906585 / 32768 = 0.0010652644 rad/s/LSB */
#define LSB_GYRO_2000_R	0.0010652644f	/* ±2000dps 量程下的灵敏度 (rad/s/LSB) */
#define LSB_GYRO_1000_R	0.00053263222f	/* ±1000dps 量程下的灵敏度 (rad/s/LSB) */
#define LSB_GYRO_500_R	0.00026631611f	/* ±500dps 量程下的灵敏度 (rad/s/LSB) */
#define LSB_GYRO_250_R	0.00013315805f	/* ±250dps 量程下的灵敏度 (rad/s/LSB) */
#define LSB_GYRO_125D_R	0.000066579027f	/* ±125dps 量程下的灵敏度 (rad/s/LSB) */


/* Bank 0 寄存器 */
#define ICM42688_DEVICE_CONFIG             0x11	/* 设备配置寄存器 */
#define ICM42688_DRIVE_CONFIG              0x13	/* 驱动配置寄存器 */
#define ICM42688_INT_CONFIG                0x14	/* 中断配置寄存器 */
#define ICM42688_FIFO_CONFIG               0x16	/* FIFO配置寄存器 */
#define ICM42688_TEMP_DATA1                0x1D	/* 温度数据高8位 */
#define ICM42688_TEMP_DATA0                0x1E	/* 温度数据低8位 */
#define ICM42688_ACCEL_DATA_X1             0x1F	/* 加速度计X轴数据高8位 */
#define ICM42688_ACCEL_DATA_X0             0x20	/* 加速度计X轴数据低8位 */
#define ICM42688_ACCEL_DATA_Y1             0x21	/* 加速度计Y轴数据高8位 */
#define ICM42688_ACCEL_DATA_Y0             0x22	/* 加速度计Y轴数据低8位 */
#define ICM42688_ACCEL_DATA_Z1             0x23	/* 加速度计Z轴数据高8位 */
#define ICM42688_ACCEL_DATA_Z0             0x24	/* 加速度计Z轴数据低8位 */
#define ICM42688_GYRO_DATA_X1              0x25	/* 陀螺仪X轴数据高8位 */
#define ICM42688_GYRO_DATA_X0              0x26	/* 陀螺仪X轴数据低8位 */
#define ICM42688_GYRO_DATA_Y1              0x27	/* 陀螺仪Y轴数据高8位 */
#define ICM42688_GYRO_DATA_Y0              0x28	/* 陀螺仪Y轴数据低8位 */
#define ICM42688_GYRO_DATA_Z1              0x29	/* 陀螺仪Z轴数据高8位 */
#define ICM42688_GYRO_DATA_Z0              0x2A	/* 陀螺仪Z轴数据低8位 */
#define ICM42688_TMST_FSYNCH               0x2B	/* 时间戳高8位 */
#define ICM42688_TMST_FSYNCL               0x2C	/* 时间戳低8位 */
#define ICM42688_INT_STATUS                0x2D	/* 中断状态寄存器 */
#define ICM42688_FIFO_COUNTH               0x2E	/* FIFO计数器高8位 */
#define ICM42688_FIFO_COUNTL               0x2F	/* FIFO计数器低8位 */
#define ICM42688_FIFO_DATA                 0x30	/* FIFO数据读取寄存器 */
#define ICM42688_APEX_DATA0                0x31	/* APEX数据0 */
#define ICM42688_APEX_DATA1                0x32	/* APEX数据1 */
#define ICM42688_APEX_DATA2                0x33	/* APEX数据2 */
#define ICM42688_APEX_DATA3                0x34	/* APEX数据3 */
#define ICM42688_APEX_DATA4                0x35	/* APEX数据4 */
#define ICM42688_APEX_DATA5                0x36	/* APEX数据5 */
#define ICM42688_INT_STATUS2               0x37	/* 中断状态寄存器2 */
#define ICM42688_INT_STATUS3               0x38	/* 中断状态寄存器3 */
#define ICM42688_SIGNAL_PATH_RESET         0x4B	/* 信号路径复位寄存器 */
#define ICM42688_INTF_CONFIG0              0x4C	/* 接口配置寄存器0 */
#define ICM42688_INTF_CONFIG1              0x4D	/* 接口配置寄存器1 */
#define ICM42688_PWR_MGMT0                 0x4E	/* 电源管理寄存器0 (关键: 启动传感器) */
#define ICM42688_GYRO_CONFIG0              0x4F	/* 陀螺仪配置寄存器0 (GFS和GODR设置) */
#define ICM42688_ACCEL_CONFIG0             0x50	/* 加速度计配置寄存器0 (AFS和AODR设置) */
#define ICM42688_GYRO_CONFIG1              0x51	/* 陀螺仪配置寄存器1 (滤波设置) */
#define ICM42688_GYRO_ACCEL_CONFIG0        0x52	/* 陀螺仪和加速度计配置寄存器0 */
#define ICM42688_ACCEL_CONFIG1             0x53	/* 加速度计配置寄存器1 (滤波设置) */
#define ICM42688_TMST_CONFIG               0x54	/* 时间戳配置寄存器 */
#define ICM42688_APEX_CONFIG0              0x56	/* APEX配置寄存器0 */
#define ICM42688_SMD_CONFIG                0x57	/* 运动检测配置寄存器 */
#define ICM42688_FIFO_CONFIG1              0x5F	/* FIFO配置寄存器1 */
#define ICM42688_FIFO_CONFIG2              0x60	/* FIFO配置寄存器2 */
#define ICM42688_FIFO_CONFIG3              0x61	/* FIFO配置寄存器3 */
#define ICM42688_FSYNC_CONFIG              0x62	/* 帧同步配置寄存器 */
#define ICM42688_INT_CONFIG0               0x63	/* 中断配置寄存器0 */
#define ICM42688_INT_CONFIG1               0x64	/* 中断配置寄存器1 */
#define ICM42688_INT_SOURCE0               0x65	/* 中断源寄存器0 */
#define ICM42688_INT_SOURCE1               0x66	/* 中断源寄存器1 */
#define ICM42688_INT_SOURCE3               0x68	/* 中断源寄存器3 */
#define ICM42688_INT_SOURCE4               0x69	/* 中断源寄存器4 */
#define ICM42688_FIFO_LOST_PKT0            0x6C	/* FIFO丢失包计数高8位 */
#define ICM42688_FIFO_LOST_PKT1            0x6D	/* FIFO丢失包计数低8位 */
#define ICM42688_SELF_TEST_CONFIG          0x70	/* 自测试配置寄存器 */
#define ICM42688_WHO_AM_I                  0x75	/* 设备ID读取寄存器 */
#define ICM42688_REG_BANK_SEL              0x76	/* 寄存器Bank选择寄存器 */

/* Bank 1 寄存器 */
#define ICM42688_SENSOR_CONFIG0            0x03	/* 传感器配置寄存器0 */
#define ICM42688_GYRO_CONFIG_STATIC2       0x0B	/* 陀螺仪静态配置寄存器2 */
#define ICM42688_GYRO_CONFIG_STATIC3       0x0C	/* 陀螺仪静态配置寄存器3 */
#define ICM42688_GYRO_CONFIG_STATIC4       0x0D	/* 陀螺仪静态配置寄存器4 */
#define ICM42688_GYRO_CONFIG_STATIC5       0x0E	/* 陀螺仪静态配置寄存器5 */
#define ICM42688_GYRO_CONFIG_STATIC6       0x0F	/* 陀螺仪静态配置寄存器6 */
#define ICM42688_GYRO_CONFIG_STATIC7       0x10	/* 陀螺仪静态配置寄存器7 */
#define ICM42688_GYRO_CONFIG_STATIC8       0x11	/* 陀螺仪静态配置寄存器8 */
#define ICM42688_GYRO_CONFIG_STATIC9       0x12	/* 陀螺仪静态配置寄存器9 */
#define ICM42688_GYRO_CONFIG_STATIC10      0x13	/* 陀螺仪静态配置寄存器10 */
#define ICM42688_XG_ST_DATA                0x5F	/* 陀螺仪X轴自测试数据 */
#define ICM42688_YG_ST_DATA                0x60	/* 陀螺仪Y轴自测试数据 */
#define ICM42688_ZG_ST_DATA                0x61	/* 陀螺仪Z轴自测试数据 */
#define ICM42688_TMSTVAL0                  0x62	/* 时间戳值0 */
#define ICM42688_TMSTVAL1                  0x63	/* 时间戳值1 */
#define ICM42688_TMSTVAL2                  0x64	/* 时间戳值2 */
#define ICM42688_INTF_CONFIG4              0x7A	/* 接口配置寄存器4 */
#define ICM42688_INTF_CONFIG5              0x7B	/* 接口配置寄存器5 */
#define ICM42688_INTF_CONFIG6              0x7C	/* 接口配置寄存器6 */

// 加速度计的量程范围
enum ICM42688P_AFS
{
    ICM42688P_AFS_16G, // DEFAULT
    ICM42688P_AFS_8G,
    ICM42688P_AFS_4G,
    ICM42688P_AFS_2G,
    NUM_ICM42688P_AFS
};

// 加速度计的输出数据速率
enum ICM42688P_AODR
{
    ICM42688P_AODR_32000HZ,
    ICM42688P_AODR_16000HZ,
    ICM42688P_AODR_8000HZ,
    ICM42688P_AODR_4000HZ,
    ICM42688P_AODR_2000HZ,
    ICM42688P_AODR_1000HZ,
    ICM42688P_AODR_200HZ,
    ICM42688P_AODR_100HZ,
    ICM42688P_AODR_50HZ,
    ICM42688P_AODR_25HZ,
    ICM42688P_AODR_12_5HZ,
    ICM42688P_AODR_6_25HZ,
    ICM42688P_AODR_3_125HZ,
    ICM42688P_AODR_1_5625HZ,
    ICM42688P_AODR_500HZ,
    NUM_ICM42688P_AODR
};

// 陀螺仪量程
enum ICM42688P_GFS
{
    ICM42688P_GFS_2000DPS, // DEFAULT
    ICM42688P_GFS_1000DPS,
    ICM42688P_GFS_500DPS,
    ICM42688P_GFS_250DPS,
    ICM42688P_GFS_125DPS,
    ICM42688P_GFS_62_5DPS,
    ICM42688P_GFS_31_25DPS,
    ICM42688P_GFS_15_625DPS,
    NUM_ICM42688P_GFS
};

// 陀螺仪输出速率
enum ICM42688P_GODR
{
    ICM42688_GODR_32000HZ,
    ICM42688_GODR_16000HZ,
    ICM42688_GODR_8000HZ,
    ICM42688_GODR_4000HZ,
    ICM42688_GODR_2000HZ,
    ICM42688_GODR_1000HZ, // DEFAULT
    ICM42688_GODR_200HZ,
    ICM42688_GODR_100HZ,
    ICM42688_GODR_50HZ,
    ICM42688_GODR_25HZ,
    ICM42688_GODR_12_5HZ,
    ICM42688_GODR_X0HZ,
    ICM42688_GODR_X1HZ,
    ICM42688_GODR_X2HZ,
    ICM42688_GODR_500HZ,
    NUM_ICM42688_GODR
};

#endif // __DEVICE_ICM42688P_H__
