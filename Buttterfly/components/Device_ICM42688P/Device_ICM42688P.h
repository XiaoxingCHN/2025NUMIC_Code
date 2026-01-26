#ifndef __DEVICE_ICM42688P_H__
#define __DEVICE_ICM42688P_H__

#define ICM42688_INT1   5  // 中断引脚

#define SPI_SCLK_FREQ  10*1000*1000  // SPI 时钟频率 10MHz


#define SPI_CS_PIN      4
#define SPI_SCLK_PIN    3
#define SPI_MOSI_PIN    2
#define SPI_MISO_PIN    1


#define ICM42688P_ADDR         0x68  // I2C 地址

#endif // __DEVICE_ICM42688P_H__
