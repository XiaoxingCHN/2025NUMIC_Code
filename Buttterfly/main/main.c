#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "led_strip.h"
#include "WS2812BS_Signal.h"


static const char *TAG = "example";





void app_main(void)
{
    configure_led(); // 配置 LED

    while (1)
    {

        ESP_LOGI(TAG, "Set LED color to RED");
        WS2812_Color_Set(0, 0, 255);              // 设置为红色
        vTaskDelay(pdMS_TO_TICKS(20));              // 延时 500 毫秒

        // ESP_LOGI(TAG, "Clear LED color");
        // WS2812_Color_Set(0, 0, 0);              // 清空灯带，熄灭 LED
        // vTaskDelay(pdMS_TO_TICKS(1000)); // 延时 1000 毫秒

        // ESP_LOGI(TAG, "Set LED color to GREEN");
        // WS2812_Color_Set(0, 0, 0);              // 刷新灯带使颜色生效
        // vTaskDelay(pdMS_TO_TICKS(20));              // 延时 500 毫秒

        // ESP_LOGI(TAG, "Clear LED color");
        // WS2812_Color_Set(0, 0, 0);              // 清空灯带，熄灭 LED
        // vTaskDelay(pdMS_TO_TICKS(1000)); // 延时 1000 毫秒
    }
}