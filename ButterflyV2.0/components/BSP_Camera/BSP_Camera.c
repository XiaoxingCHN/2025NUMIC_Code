#include "BSP_Camera.h"

#include "esp_camera.h"
#include "esp_log.h"

static const char *TAG = "BSP_Camera";

esp_err_t BSP_Camera_Init(void)
{
    camera_config_t config = {
        .pin_pwdn     = CAM_PIN_PWDN,
        .pin_reset    = CAM_PIN_RESET,
        .pin_xclk     = CAM_PIN_XCLK,
        .pin_sccb_sda = CAM_PIN_SIOD,
        .pin_sccb_scl = CAM_PIN_SIOC,

        .pin_d7    = CAM_PIN_D7,
        .pin_d6    = CAM_PIN_D6,
        .pin_d5    = CAM_PIN_D5,
        .pin_d4    = CAM_PIN_D4,
        .pin_d3    = CAM_PIN_D3,
        .pin_d2    = CAM_PIN_D2,
        .pin_d1    = CAM_PIN_D1,
        .pin_d0    = CAM_PIN_D0,
        .pin_vsync = CAM_PIN_VSYNC,
        .pin_href  = CAM_PIN_HREF,
        .pin_pclk  = CAM_PIN_PCLK,

        .xclk_freq_hz = 20000000,             /* 20 MHz XCLK */
        .ledc_timer   = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,

        .pixel_format = PIXFORMAT_GRAYSCALE,  /* 灰度，AprilTag 直接使用 */
        .frame_size   = FRAMESIZE_QVGA,       /* 320 × 240 */
        .jpeg_quality = 12,
        .fb_count     = 2,                    /* 双缓冲，避免掉帧 */
        .fb_location  = CAMERA_FB_IN_PSRAM,   /* 使用 PSRAM 存储帧缓冲 */
        .grab_mode    = CAMERA_GRAB_LATEST,   /* 始终返回最新帧 */
    };

    esp_err_t ret = esp_camera_init(&config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Camera initialized (%dx%d grayscale)", CAM_DEFAULT_WIDTH, CAM_DEFAULT_HEIGHT);
    return ESP_OK;
}

esp_err_t BSP_Camera_Deinit(void)
{
    esp_err_t ret = esp_camera_deinit();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Camera deinit failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t BSP_Camera_CaptureFrame(Camera_Frame_t *frame)
{
    if (frame == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    camera_fb_t *fb = esp_camera_fb_get();
    if (fb == NULL) {
        ESP_LOGE(TAG, "Camera capture failed");
        return ESP_ERR_TIMEOUT;
    }

    /* 每次只保留一个活跃帧（竞赛场景足够用；多帧并发请改用队列）。*/
    static camera_fb_t *s_active_fb = NULL;
    if (s_active_fb != NULL) {
        esp_camera_fb_return(s_active_fb);
    }
    s_active_fb = fb;

    frame->buf    = fb->buf;
    frame->len    = fb->len;
    frame->width  = (int)fb->width;
    frame->height = (int)fb->height;

    return ESP_OK;
}

void BSP_Camera_ReleaseFrame(Camera_Frame_t *frame)
{
    /* 当前实现中帧在下一次 CaptureFrame 时自动归还，
     * 此函数保留以供将来改为双/多帧并发时使用。
     */
    (void)frame;
}
