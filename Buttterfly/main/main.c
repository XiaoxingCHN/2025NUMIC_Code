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
        WS2812_Color_Set(255, 0, 255);              
        vTaskDelay(pdMS_TO_TICKS(500));           // 延时 500 毫秒
        WS2812_Color_Set(0, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(500));           // 延时 500

    }

}