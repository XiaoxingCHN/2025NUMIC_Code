#include "Device_ICM42688P.h"

static float accSensitivity = 0.244f; // 加速度灵敏度（LSB/g）
static float gyroSensitivity = 32.8f; // 陀螺仪的最小分辨率

/*******************************************************************************
* 名    称： Icm_Spi_ReadWriteNbytes
* 功    能： 使用SPI读写n个字节
* 入口参数： pBuffer: 写入的数组  len:写入数组的长度
* 出口参数： 无
* 作　　者： Baxiange
* 创建日期： 2024-07-25
* 修    改：
* 修改日期：
* 备    注：
*******************************************************************************/
// static void Icm_Spi_ReadWriteNbytes(uint8_t* pBuffer, uint8_t len)
// {
//     uint8_t i = 0;

//     for(i = 0; i < len; i ++)
//     {
// 		*pBuffer = SPI2_ReadWriteByte(*pBuffer);
//         pBuffer++;
//     }

// }


