#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "Device_WS2812B.h"
#include "Device_AprilTag.h"
#include "esp_log.h"

static const char *TAG = "main";

/* AprilTag 检测结果回调：检测到标签时将 LED 设置为绿色 */
static void apriltag_callback(const AprilTag_Result_t *result)
{
    if (result->detected)
    {
        ESP_LOGI(TAG, "AprilTag %u 已检测到: x=%.1fmm y=%.1fmm z=%.1fmm yaw=%.1f°",
                 result->tag_id, result->x, result->y, result->z, result->yaw);
        WS2812B_Color_Set(0, 255, 0); /* 绿色：检测到标签 */
    }
    else
    {
        WS2812B_Color_Set(255, 0, 0); /* 红色：未检测到标签 */
    }
}

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

    /* 初始化 AprilTag 接收器（使用默认 UART 配置） */
    AprilTag_Config_t at_cfg = {
        .uart_port = APRILTAG_UART_PORT,
        .baud_rate = APRILTAG_UART_BAUD,
        .tx_pin    = APRILTAG_UART_TX_PIN,
        .rx_pin    = APRILTAG_UART_RX_PIN,
        .callback  = apriltag_callback,
    };
    AprilTagInstance *at_ins = AprilTag_Init(&at_cfg);
    if (at_ins == NULL)
    {
        ESP_LOGE(TAG, "AprilTag 初始化失败");
    }

    AprilTag_Result_t at_result;

    while (1)
    {
        /* 轮询最新 AprilTag 结果 */
        if (at_ins != NULL && AprilTag_GetResult(at_ins, &at_result))
        {
            ESP_LOGI(TAG, "[轮询] Tag %u %s: x=%.1f y=%.1f z=%.1f yaw=%.1f",
                     at_result.tag_id,
                     at_result.detected ? "检测到" : "未检测到",
                     at_result.x, at_result.y, at_result.z, at_result.yaw);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
