#include "WS2812BS_Signal.h"

led_strip_handle_t led_strip; // WS2812B的控制句柄

/**
 * @brief 配置 WS2812B LED 灯带
 * @author XMX
 * **/
void configure_led(void)
{
   // ESP_LOGI(TAG, "Configuring addressable LED on GPIO %d", BLINK_GPIO);

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
/**
 * @brief 设置 WS2812B LED 颜色
 * @param red 红色分量 (0-255)
 * @param green 绿色分量 (0-255)
 * @param blue 蓝色分量 (0-255)
 * @author XMX
 * **/
void WS2812_Color_Set(uint8_t red, uint8_t green, uint8_t blue)
{
    led_strip_set_pixel(led_strip, 0, red, green, blue);//设置WS2812B的颜色
    led_strip_refresh(led_strip); // 刷新灯带使颜色生效
}
