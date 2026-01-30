#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "led_strip.h"
#include "WS2812BS_Signal.h"
#include "Device_ICM42688P.h"



void app_main(void)
{

    configure_led(); // 配置 LED
    Dev_ICM42688P_Init(); // 初始化ICM42688P传感器
    ICM42688P_Data_Def dataZero={0};
    Dev_ICM42688P_ZeroCalibrate(&dataZero);
    ICM42688P_Data_Def sensor_data;
    while (1)
    {
        
        Dev_ICM42688P_GetData(&sensor_data); // 获取传感器数据
        // GetData 已经扣除了零偏，直接打印
        printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n", sensor_data.Acc.ax, sensor_data.Acc.ay, sensor_data.Acc.az, sensor_data.Gyr.gx, sensor_data.Gyr.gy, sensor_data.Gyr.gz);
        // ESP_LOGI(TAG, "Gyro: gx=%.2f dps, gy=%.2f dps, gz=%.2f dps", sensor_data.Gyr.gx, sensor_data.Gyr.gy, sensor_data.Gyr.gz);
        // ESP_LOGI(TAG, "Set LED color to RED");
        vTaskDelay(pdMS_TO_TICKS(10)); // 延时 1 秒
        // WS2812_Color_Set(255, 0, 255);              
        // vTaskDelay(pdMS_TO_TICKS(500));           // 延时 500 毫秒
        // WS2812_Color_Set(0, 0, 0);
        // vTaskDelay(pdMS_TO_TICKS(500));           // 延时 500

    }

}