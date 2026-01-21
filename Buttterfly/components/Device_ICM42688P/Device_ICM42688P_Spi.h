#ifndef __DEVICE_ICM42688P_SPI_H__
#define __DEVICE_ICM42688P_SPI_H__

#include "driver/spi_master.h"
#include "driver/gpio.h"

#define SPI_HOST_ID     SPI2_HOST
#define SPI_CLOCK_HZ    10*1000*1000  // 10 MHz

#define SPI_MOSI_PIN    1
#define SPI_MISO_PIN    2
#define SPI_SCLK_PIN    3
#define SPI_CS_PIN      4

extern spi_device_handle_t spi_device_handle;

#endif // __DEVICE_ICM42688P_SPI_H__
