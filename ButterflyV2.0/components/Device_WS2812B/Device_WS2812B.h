#ifndef __DEVICE_WS2812B_H__
#define __DEVICE_WS2812B_H__

#include "led_strip.h" // 包含 LED 灯带控制库
#include "stdint.h"   // 包含标准整数类型定义
//使用RMT驱动WS2812B,不占用额外的SPI资源

#define BLINK_GPIO 21 // LED 连接的 GPIO 引脚
#define NUM_LEDS 1   // LED 数量，根据实际连接的 LED 数量进行调整
extern led_strip_handle_t led_strip; //WS2812B的控制句柄

void configure_WS2812B(void);//初始化WS2812B的配置
void WS2812B_Color_Set(uint8_t red, uint8_t green, uint8_t blue);//设置WS2812B的颜色 255 255 255


#endif // __DEVICE_WS2812B_H__

