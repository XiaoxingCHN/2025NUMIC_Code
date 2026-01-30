#include "Device_ICM42688P.h"



/*SPI总线配置*/
spi_bus_config_t spi2_bus_cfg = {
    .miso_io_num = SPI_MISO_PIN,
    .mosi_io_num = SPI_MOSI_PIN,
    .sclk_io_num = SPI_SCLK_PIN,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 32, // 最大传输大小，根据需要调整
};

//设备配置
spi_device_interface_config_t spi2_dev_cfg = {
    .mode = 0, // ICM42688P 支持 SPI 模式 0/3，推荐 0
    .clock_speed_hz = SPI_SCLK_FREQ,
    .spics_io_num = SPI_CS_PIN,
    .queue_size = 7,                    
    .flags = 0, // 默认全双工模式
    .pre_cb = NULL
};

spi_device_handle_t hspi2; // SPI 设备句柄used for ICM42688P

// 零偏存储（上电后通过 Dev_ICM42688P_ZeroCalibrate 计算）
static ICM42688P_Data_Def s_offset = {0};

//初始化ICM42688P所使用的SPI接口
void Dev_ICM42688P_SPI_Init(void)
{

    //初始化SPI总线
    esp_err_t ret = spi_bus_initialize(SPI_HOST_ID, &spi2_bus_cfg, SPI_DMA_CH_AUTO);
    if(ret!=ESP_OK)
    {
       printf("Failed to initialize SPI bus\n");
    }

    //添加SPI设备
    ret = spi_bus_add_device(SPI_HOST_ID, &spi2_dev_cfg, &hspi2);
    if(ret!=ESP_OK)
    {
       printf("Failed to add SPI device\n");
    }

}

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

//从ICM42688P读取多个字节数据
void Dev_ICM42688P_ReadBytes(uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t tx_data[len + 1]; // gnu17 变长数组，len 最大 12 足够栈放
    uint8_t rx_data[len + 1];

    tx_data[0] = reg | 0x80; // 读操作，最高位为1
    memset(tx_data + 1, 0x00, len); // dummy bytes

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = (len + 1) * 8; // 总长度，包含寄存器地址
    t.tx_buffer = tx_data;
    t.rx_buffer = rx_data;

    spi_device_transmit(hspi2, &t);

    // 复制读取的数据到用户缓冲区，跳过第一个字节（寄存器地址）
    memcpy(buf, rx_data + 1, len);
}

//初始化设备ICM42688P
void Dev_ICM42688P_Init()
{

    Dev_ICM42688P_SPI_Init(); 

    /*检查是否SPI通信成功*/
    uint8_t who_am_i = Dev_ICM42688P_ReadByte(ICM42688_WHO_AM_I);
    if(who_am_i != 0x47)
    {
        printf("ICM42688P not found! WHO_AM_I=0x%02X\n", who_am_i);
        return;
    }
    else
    {
        printf("ICM42688P found! WHO_AM_I=0x%02X\n", who_am_i);
    }

    // 软复位
    Dev_ICM42688P_WriteByte(ICM42688_DEVICE_CONFIG, 0x01);
    vTaskDelay(pdMS_TO_TICKS(100));

    // 接口配置：SPI 4 线，自动增址
    Dev_ICM42688P_WriteByte(ICM42688_INTF_CONFIG0, 0x00);

    // 电源管理：陀螺 LN、加速度 LN、温度开（0x0F）
    Dev_ICM42688P_WriteByte(ICM42688_PWR_MGMT0, 0x0F);
    vTaskDelay(pdMS_TO_TICKS(10));

    // 陀螺仪配置：ODR=1kHz，±2000dps（0x06 -> FS=2000dps, ODR=1kHz）
    Dev_ICM42688P_WriteByte(ICM42688_GYRO_CONFIG0, 0x06);

    // 加速度配置：ODR=1kHz，±16g（0x06 -> FS=16g, ODR=1kHz）
    Dev_ICM42688P_WriteByte(ICM42688_ACCEL_CONFIG0, 0x06);
    uint8_t filter_config = 0x9A;
    Dev_ICM42688P_WriteByte(0x08, filter_config);// 设置低通滤波器配置

}

//读取加速度和陀螺仪数据
void Dev_ICM42688P_GetData(ICM42688P_Data_Def *data)
{
    uint8_t buf[12];
    Dev_ICM42688P_ReadBytes(ICM42688_ACCEL_DATA_X1, buf, 12);

    int16_t ax_raw = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t ay_raw = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t az_raw = (int16_t)((buf[4] << 8) | buf[5]);
    int16_t gx_raw = (int16_t)((buf[6] << 8) | buf[7]);
    int16_t gy_raw = (int16_t)((buf[8] << 8) | buf[9]);
    int16_t gz_raw = (int16_t)((buf[10] << 8) | buf[11]);

    // 量程对应灵敏度：±16g -> 0.488 mg/LSB；±2000dps -> 61 mdps/LSB
    // float ax = ax_raw * (LSB_ACC_16G / 1000.0f); // 转换为 g
    // float ay = ay_raw * (LSB_ACC_16G / 1000.0f);
    // float az = az_raw * (LSB_ACC_16G / 1000.0f);
    // float gx = gx_raw * (LSB_GYRO_2000DPS / 1000.0f); // 转换为 dps
    // float gy = gy_raw * (LSB_GYRO_2000DPS / 1000.0f);
    // float gz = gz_raw * (LSB_GYRO_2000DPS / 1000.0f);
    //转换为m/s²
    float ax = ax_raw * (LSB_ACC_16G / 1000.0f) * 9.80665f; // 转换为 m/s²
    float ay = ay_raw * (LSB_ACC_16G / 1000.0f) * 9.80665f;
    float az = az_raw * (LSB_ACC_16G / 1000.0f) * 9.80665f;
    //转换为rad/s
    float gx = gx_raw * (LSB_GYRO_2000DPS / 1000.0f) * (3.14159265f / 180.0f); // 转换为 rad/s
    float gy = gy_raw * (LSB_GYRO_2000DPS / 1000.0f) * (3.14159265f / 180.0f); // 转换为 rad/s
    float gz = gz_raw * (LSB_GYRO_2000DPS / 1000.0f) * (3.14159265f / 180.0f); // 转换为 rad/s

    // 扣除零偏（需先调用 Dev_ICM42688P_ZeroCalibrate）
    data->Acc.ax = ax - s_offset.Acc.ax;
    data->Acc.ay = ay - s_offset.Acc.ay;
    data->Acc.az = az - s_offset.Acc.az;
    data->Gyr.gx = gx - s_offset.Gyr.gx;
    data->Gyr.gy = gy - s_offset.Gyr.gy;
    data->Gyr.gz = gz - s_offset.Gyr.gz;
}

//零偏校准，当角速度和加速度均接近0时，取前100此数据的平均值作为零偏，取样间隔为20ms
void Dev_ICM42688P_ZeroCalibrate(ICM42688P_Data_Def *dataZero)
{
    
    float gx_offset = 0.0f;
    float gy_offset = 0.0f;
    float gz_offset = 0.0f;
    float ax_offset = 0.0f;
    float ay_offset = 0.0f;
    float az_offset = 0.0f;

    ICM42688P_Data_Def tempData;
    const int samples = 200;
    for(int i=0; i<samples; i++)
    {
        Dev_ICM42688P_GetData(&tempData);
        gx_offset += tempData.Gyr.gx;
        gy_offset += tempData.Gyr.gy;
        gz_offset += tempData.Gyr.gz;
        ax_offset += tempData.Acc.ax;
        ay_offset += tempData.Acc.ay;
        az_offset += tempData.Acc.az;
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    dataZero->Gyr.gx = gx_offset / samples;
    dataZero->Gyr.gy = gy_offset / samples;
    dataZero->Gyr.gz = gz_offset / samples;
    dataZero->Acc.ax = ax_offset / samples;
    dataZero->Acc.ay = ay_offset / samples;
    dataZero->Acc.az = az_offset / samples;

        // 保存全局零偏供后续扣除
        s_offset = *dataZero;

        printf("Gyro Offsets(avg): gx=%.3f dps, gy=%.3f dps, gz=%.3f dps\n",
            s_offset.Gyr.gx, s_offset.Gyr.gy, s_offset.Gyr.gz);
        printf("Accel Offsets(avg): ax=%.3f g, ay=%.3f g, az=%.3f g\n",
            s_offset.Acc.ax, s_offset.Acc.ay, s_offset.Acc.az);
}