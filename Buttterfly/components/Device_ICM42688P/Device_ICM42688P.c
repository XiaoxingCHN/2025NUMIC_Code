#include "Device_ICM42688P.h"
#include "driver/spi_master.h"

spi_device_handle_t hspi2; // SPI 设备句柄used for ICM42688P

//向ICM42688P写入一个字节数据
void Dev_ICM42688P_WriteByte(uint8_t reg, uint8_t data)
{
    uint8_t tx_data[2];
    tx_data[0] = reg & 0x7F; // 写操作，最高位为0
    tx_data[1] = data;

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 16; // 16 bits
    t.tx_buffer = tx_data;

    spi_device_transmit(hspi2, &t);
}

//从ICM42688P读取一个字节数据
uint8_t Dev_ICM42688P_ReadByte(uint8_t reg)
{
    uint8_t tx_data[2];
    uint8_t rx_data[2];
    tx_data[0] = reg | 0x80; // 读操作，最高位为1
    tx_data[1] = 0x00;       // 占位符

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 16; // 16 bits
    t.tx_buffer = tx_data;
    t.rx_buffer = rx_data;

    spi_device_transmit(hspi2, &t);

    return rx_data[1]; // 返回读取的数据
}

//初始化设备ICM42688P
void Dev_ICM42688P_Init()
{

}
