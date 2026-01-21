#ifndef __WS2812BS_SIGNAL_H__
#define __WS2812BS_SIGNAL_H__
#include "led_strip.h"
#define BLINK_GPIO 21 // LED 连接的 GPIO 引脚

extern led_strip_handle_t led_strip; // LED 灯带句柄
void configure_led(void);
void WS2812_Color_Set(uint8_t red, uint8_t green, uint8_t blue);
#endif // __WS2812BS_SIGNAL_H__
