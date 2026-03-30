#ifndef __BSP_CAMERA_H__
#define __BSP_CAMERA_H__

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================= 用户可按工程修改的宏 ========================= */

/* OV2640 引脚定义（适用于 ESP32-S3 开发板，按需修改） */
#ifndef CAM_PIN_PWDN
#define CAM_PIN_PWDN    -1   /* 无下电引脚时置 -1 */
#endif

#ifndef CAM_PIN_RESET
#define CAM_PIN_RESET   -1   /* 无硬复位引脚时置 -1 */
#endif

#ifndef CAM_PIN_XCLK
#define CAM_PIN_XCLK    10
#endif

#ifndef CAM_PIN_SIOD
#define CAM_PIN_SIOD    40
#endif

#ifndef CAM_PIN_SIOC
#define CAM_PIN_SIOC    39
#endif

#ifndef CAM_PIN_D7
#define CAM_PIN_D7      48
#endif

#ifndef CAM_PIN_D6
#define CAM_PIN_D6      11
#endif

#ifndef CAM_PIN_D5
#define CAM_PIN_D5      12
#endif

#ifndef CAM_PIN_D4
#define CAM_PIN_D4      14
#endif

#ifndef CAM_PIN_D3
#define CAM_PIN_D3      16
#endif

#ifndef CAM_PIN_D2
#define CAM_PIN_D2      18
#endif

#ifndef CAM_PIN_D1
#define CAM_PIN_D1      17
#endif

#ifndef CAM_PIN_D0
#define CAM_PIN_D0      15
#endif

#ifndef CAM_PIN_VSYNC
#define CAM_PIN_VSYNC   38
#endif

#ifndef CAM_PIN_HREF
#define CAM_PIN_HREF    47
#endif

#ifndef CAM_PIN_PCLK
#define CAM_PIN_PCLK    13
#endif

/* 默认分辨率：QVGA 320×240，灰度图，AprilTag 检测的最佳平衡点 */
#ifndef CAM_DEFAULT_WIDTH
#define CAM_DEFAULT_WIDTH  320
#endif

#ifndef CAM_DEFAULT_HEIGHT
#define CAM_DEFAULT_HEIGHT 240
#endif

/* ====================================================================== */

/**
 * @brief 摄像头帧描述符
 */
typedef struct {
    uint8_t *buf;   /*!< 图像数据缓冲区（灰度，row-major，左上为原点） */
    size_t   len;   /*!< 缓冲区字节数，等于 width * height */
    int      width;
    int      height;
} Camera_Frame_t;

/**
 * @brief 初始化摄像头（OV2640，GRAYSCALE，QVGA）
 *
 * @return ESP_OK 成功，否则返回错误码
 */
esp_err_t BSP_Camera_Init(void);

/**
 * @brief 反初始化摄像头，释放底层资源
 */
esp_err_t BSP_Camera_Deinit(void);

/**
 * @brief 捕获一帧灰度图像
 *
 * 调用者使用完毕后 **必须** 调用 BSP_Camera_ReleaseFrame() 归还帧缓冲。
 *
 * @param[out] frame  输出帧描述符（buf 指针由驱动管理，请勿 free）
 * @return ESP_OK 成功，ESP_ERR_TIMEOUT 超时，其他为驱动错误
 */
esp_err_t BSP_Camera_CaptureFrame(Camera_Frame_t *frame);

/**
 * @brief 释放由 BSP_Camera_CaptureFrame() 获取的帧
 *
 * @param frame  待释放的帧描述符
 */
void BSP_Camera_ReleaseFrame(Camera_Frame_t *frame);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_CAMERA_H__ */
