#ifndef __WS2812BS_SIGNAL_H__
#define __WS2812BS_SIGNAL_H__
#include "led_strip.h"
//使用RMT驱动WS2812B,不占用额外的SPI资源

#define BLINK_GPIO 21 // LED 连接的 GPIO 引脚

extern led_strip_handle_t led_strip; //WS2812B的控制句柄

void configure_led(void);//初始化WS2812B的配置
void WS2812_Color_Set(uint8_t red, uint8_t green, uint8_t blue);//设置WS2812B的颜色 255 255 255

#endif // __WS2812BS_SIGNAL_H__
