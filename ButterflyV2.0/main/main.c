#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "Device_WS2812B.h"
#include "BSP_Camera.h"
#include "Algorithm_AprilTag.h"

static const char *TAG = "main";

/* 相机内参（使用 OV2640 QVGA 的典型值，按实际标定结果修改） */
static const AprilTag_Camera_t s_cam_params = {
    .fx       = 307.0f,   /* 水平焦距（像素） */
    .fy       = 307.0f,   /* 垂直焦距（像素） */
    .cx       = 160.0f,   /* 主点 X（QVGA 宽 320 的一半）*/
    .cy       = 120.0f,   /* 主点 Y（QVGA 高 240 的一半）*/
    .tag_size = 0.166f,   /* 标签实际边长（米），按现场标签修改 */
};

/* AprilTag 检测任务 */
static void apriltag_task(void *pvParameters)
{
    Camera_Frame_t  frame  = {0};
    AprilTag_Result_t result = {0};

    /* 初始化摄像头 */
    if (BSP_Camera_Init() != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed, task exiting");
        vTaskDelete(NULL);
        return;
    }

    /* 初始化 AprilTag 检测器（QVGA 分辨率）*/
    if (AprilTag_Init(CAM_DEFAULT_WIDTH, CAM_DEFAULT_HEIGHT) != ESP_OK) {
        ESP_LOGE(TAG, "AprilTag init failed, task exiting");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "AprilTag detection started");

    while (1) {
        /* 捕获一帧灰度图 */
        if (BSP_Camera_CaptureFrame(&frame) != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        /* 运行检测 */
        AprilTag_Detect(frame.buf, frame.width, frame.height, &result);
        BSP_Camera_ReleaseFrame(&frame);

        if (result.count > 0) {
            for (int i = 0; i < result.count; i++) {
                AprilTag_Detection_t *det = &result.tags[i];

                /* 估计位姿 */
                AprilTag_EstimatePose(det, &s_cam_params);

                ESP_LOGI(TAG,
                    "Tag %d | ham=%d | center=(%.1f,%.1f) | "
                    "t=[%.3f, %.3f, %.3f]m | r=[%.3f, %.3f, %.3f]",
                    det->id, det->hamming,
                    det->cx, det->cy,
                    det->pose_t[0], det->pose_t[1], det->pose_t[2],
                    det->pose_r[0], det->pose_r[1], det->pose_r[2]);

                /* 检测到标签时 LED 亮绿色（ID 越大颜色越深，防止溢出）*/
                int raw = 255 - det->id * 4;
                uint8_t brightness = (uint8_t)(raw > 0 ? raw : 0);
                WS2812B_Color_Set(0, brightness, 0);
            }
        } else {
            /* 未检测到标签：LED 熄灭 */
            WS2812B_Color_Set(0, 0, 0);
        }

        vTaskDelay(pdMS_TO_TICKS(33)); /* ~30 FPS */
    }
}

void app_main(void)
{
    configure_WS2812B();

    /* 上电自检：红 -> 绿 -> 蓝 */
    WS2812B_Color_Set(255, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(600));
    WS2812B_Color_Set(0, 255, 0);
    vTaskDelay(pdMS_TO_TICKS(600));
    WS2812B_Color_Set(0, 0, 255);
    vTaskDelay(pdMS_TO_TICKS(600));
    WS2812B_Color_Set(0, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(200));

    /* 在独立任务中运行 AprilTag 检测（需要约 32 KB 栈） */
    xTaskCreate(apriltag_task, "apriltag", 32 * 1024, NULL, 5, NULL);
}
