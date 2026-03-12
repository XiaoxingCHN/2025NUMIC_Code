#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "Device_WS2812B.h"
#include "ESP_Log.h"

void app_main(void)
{
    configure_WS2812B();

    // 上电自检：先用慢速纯色确认接线和供电是否正常
    WS2812B_Color_Set(255, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(600));
    WS2812B_Color_Set(0, 255, 0);
    vTaskDelay(pdMS_TO_TICKS(600));
    WS2812B_Color_Set(0, 0, 255);
    vTaskDelay(pdMS_TO_TICKS(600));
    WS2812B_Color_Set(0, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(200));

    while (1)
    {
        ESP_LOGI("WS2812B", "Starting color cycle...");
        // 红 -> 黄
        for (int g = 0; g <= 255; g += 5) {
            WS2812B_Color_Set(255, g, 0);
            vTaskDelay(pdMS_TO_TICKS(40));
        }

        // 黄 -> 绿
        for (int r = 255; r >= 0; r -= 5) {
            WS2812B_Color_Set(r, 255, 0);
            vTaskDelay(pdMS_TO_TICKS(40));
        }

        // 绿 -> 青
        for (int b = 0; b <= 255; b += 5) {
            WS2812B_Color_Set(0, 255, b);
            vTaskDelay(pdMS_TO_TICKS(40));
        }

        // 青 -> 蓝
        for (int g = 255; g >= 0; g -= 5) {
            WS2812B_Color_Set(0, g, 255);
            vTaskDelay(pdMS_TO_TICKS(40));
        }

        // 蓝 -> 紫
        for (int r = 0; r <= 255; r += 5) {
            WS2812B_Color_Set(r, 0, 255);
            vTaskDelay(pdMS_TO_TICKS(40));
        }

        // 紫 -> 红
        for (int b = 255; b >= 0; b -= 5) {
            WS2812B_Color_Set(255, 0, b);
            vTaskDelay(pdMS_TO_TICKS(40));
        }
    }
}
