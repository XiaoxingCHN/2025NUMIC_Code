#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "led_strip.h"

static const char *TAG = "example";

#define BLINK_GPIO 21 // LED 连接的 GPIO 引脚

static led_strip_handle_t led_strip; // LED 灯带句柄

static void configure_led(void)
{
    ESP_LOGI(TAG, "Configuring addressable LED on GPIO %d", BLINK_GPIO);

    // LED 灯带通用配置
    led_strip_config_t strip_config = {
        .strip_gpio_num = BLINK_GPIO,                                // 设置 GPIO 引脚
        .max_leds = 1,                                               // 设置 LED 数量
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_RGB, // 设置颜色格式
    };

    // RMT 后端特定配置
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // RMT 分辨率，10MHz
        .flags.with_dma = false,           // 禁用 DMA
    };

    // 创建 LED 灯带对象
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));

    led_strip_clear(led_strip); // 初始状态下清空灯带
}

void app_main(void)
{
    configure_led(); // 配置 LED

    while (1)
    {

        ESP_LOGI(TAG, "Set LED color to RED");
        led_strip_set_pixel(led_strip, 0, 255, 0, 0); // 设置为红色
        led_strip_refresh(led_strip);                 // 刷新灯带使颜色生效
        vTaskDelay(pdMS_TO_TICKS(1000));              // 延时 1000 毫秒

        ESP_LOGI(TAG, "Clear LED color");
        led_strip_clear(led_strip);      // 清空灯带，熄灭 LED
        vTaskDelay(pdMS_TO_TICKS(1000)); // 延时 1000 毫秒

        ESP_LOGI(TAG, "Set LED color to GREEN");
        led_strip_set_pixel(led_strip, 0, 0, 255, 0); // 设置为绿色
        led_strip_refresh(led_strip);                 // 刷新灯带使颜色生效
        vTaskDelay(pdMS_TO_TICKS(1000));              // 延时 1000 毫秒

        ESP_LOGI(TAG, "Clear LED color");
        led_strip_clear(led_strip);      // 清空灯带，熄灭 LED
        vTaskDelay(pdMS_TO_TICKS(1000)); // 延时 1000 毫秒
    }
}