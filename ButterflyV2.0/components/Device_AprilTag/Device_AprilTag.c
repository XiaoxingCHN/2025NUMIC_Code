#include "Device_AprilTag.h"

#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "AprilTag";

/* -------------------- 内部结构体 -------------------- */

struct AprilTagInstance
{
    uart_port_t          uart_port;
    AprilTag_Callback_t  callback;

    AprilTag_Result_t    latest;        /* 最新检测结果 */
    SemaphoreHandle_t    mutex;         /* 保护 latest 字段 */
    volatile bool        new_data;      /* 是否有新数据未被取走 */

    TaskHandle_t         task_handle;   /* 后台解析任务句柄 */
    volatile bool        running;       /* 任务运行标志 */
    TaskHandle_t         caller_handle; /* 调用 Deinit 的任务句柄，用于任务退出通知 */
};

/* -------------------- 内部工具函数 -------------------- */

/* 将 4 个字节（little-endian）转换为 float */
static float bytes_to_float(const uint8_t *buf)
{
    float val;
    memcpy(&val, buf, sizeof(float));
    return val;
}

/* 校验和：对 [2]..[19] 进行字节累加，取低 8 位 */
static uint8_t calc_checksum(const uint8_t *frame)
{
    uint8_t sum = 0;
    for (int i = 2; i < APRILTAG_FRAME_LEN - 1; i++)
    {
        sum += frame[i];
    }
    return sum;
}

/* -------------------- 后台解析任务 -------------------- */

static void apriltag_rx_task(void *arg)
{
    AprilTagInstance *ins = (AprilTagInstance *)arg;

    uint8_t  buf[APRILTAG_FRAME_LEN * 2]; /* 接收缓冲区 */
    uint8_t  frame[APRILTAG_FRAME_LEN];
    int      frame_idx = 0;              /* 已累积的帧字节数 */
    bool     synced    = false;          /* 是否已同步到帧头 */

    while (ins->running)
    {
        int rx_len = uart_read_bytes(ins->uart_port, buf,
                                     sizeof(buf), pdMS_TO_TICKS(20));
        if (rx_len <= 0)
        {
            continue;
        }

        for (int i = 0; i < rx_len; i++)
        {
            uint8_t byte = buf[i];

            if (!synced)
            {
                /* 等待帧头第一个字节 */
                if (byte == APRILTAG_FRAME_HEADER_0)
                {
                    frame[0] = byte;
                    frame_idx = 1;
                    synced = true;
                }
            }
            else
            {
                frame[frame_idx++] = byte;

                /* 帧头第二个字节校验 */
                if (frame_idx == 2 && frame[1] != APRILTAG_FRAME_HEADER_1)
                {
                    /* 帧头不匹配，重新同步 */
                    frame_idx = 0;
                    synced = false;
                    continue;
                }

                if (frame_idx == APRILTAG_FRAME_LEN)
                {
                    /* 收到完整帧，校验 */
                    uint8_t expected = calc_checksum(frame);
                    if (frame[APRILTAG_FRAME_LEN - 1] == expected)
                    {
                        AprilTag_Result_t result;
                        result.tag_id   = frame[2];
                        result.detected = (frame[3] == 0x01);
                        result.x        = bytes_to_float(&frame[4]);
                        result.y        = bytes_to_float(&frame[8]);
                        result.z        = bytes_to_float(&frame[12]);
                        result.yaw      = bytes_to_float(&frame[16]);

                        if (xSemaphoreTake(ins->mutex, pdMS_TO_TICKS(5)) == pdTRUE)
                        {
                            ins->latest   = result;
                            ins->new_data = true;
                            xSemaphoreGive(ins->mutex);
                        }

                        if (ins->callback)
                        {
                            ins->callback(&result);
                        }

                        ESP_LOGD(TAG, "Tag %u %s x=%.1f y=%.1f z=%.1f yaw=%.1f",
                                 result.tag_id,
                                 result.detected ? "detected" : "lost",
                                 result.x, result.y, result.z, result.yaw);
                    }
                    else
                    {
                        ESP_LOGW(TAG, "Checksum error: got 0x%02X, expected 0x%02X",
                                 frame[APRILTAG_FRAME_LEN - 1], expected);
                    }

                    /* 准备接收下一帧 */
                    frame_idx = 0;
                    synced = false;
                }
            }
        }
    }

    /* 通知调用方任务已退出 */
    if (ins->caller_handle != NULL)
    {
        xTaskNotifyGive(ins->caller_handle);
    }

    vTaskDelete(NULL);
}

/* -------------------- 对外 API -------------------- */

AprilTagInstance *AprilTag_Init(const AprilTag_Config_t *cfg)
{
    /* 使用传入配置或默认值 */
    uart_port_t uart_port = cfg ? cfg->uart_port : APRILTAG_UART_PORT;
    int         baud_rate = cfg ? cfg->baud_rate : APRILTAG_UART_BAUD;
    int         tx_pin    = cfg ? cfg->tx_pin    : APRILTAG_UART_TX_PIN;
    int         rx_pin    = cfg ? cfg->rx_pin    : APRILTAG_UART_RX_PIN;

    /* 配置 UART */
    uart_config_t uart_cfg = {
        .baud_rate  = baud_rate,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t ret = uart_param_config(uart_port, &uart_cfg);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "uart_param_config failed: %s", esp_err_to_name(ret));
        return NULL;
    }

    ret = uart_set_pin(uart_port, tx_pin, rx_pin,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "uart_set_pin failed: %s", esp_err_to_name(ret));
        return NULL;
    }

    ret = uart_driver_install(uart_port, APRILTAG_RX_BUF_SIZE, 0, 0, NULL, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(ret));
        return NULL;
    }

    AprilTagInstance *ins = (AprilTagInstance *)calloc(1, sizeof(AprilTagInstance));
    if (ins == NULL)
    {
        ESP_LOGE(TAG, "calloc failed");
        uart_driver_delete(uart_port);
        return NULL;
    }

    ins->uart_port     = uart_port;
    ins->callback      = cfg ? cfg->callback : NULL;
    ins->new_data      = false;
    ins->running       = true;
    ins->caller_handle = NULL;

    ins->mutex = xSemaphoreCreateMutex();
    if (ins->mutex == NULL)
    {
        ESP_LOGE(TAG, "xSemaphoreCreateMutex failed");
        free(ins);
        uart_driver_delete(uart_port);
        return NULL;
    }

    memset(&ins->latest, 0, sizeof(AprilTag_Result_t));

    /* 启动后台解析任务 */
    BaseType_t task_ret = xTaskCreate(apriltag_rx_task, "apriltag_rx",
                                      APRILTAG_TASK_STACK_SIZE, ins,
                                      APRILTAG_TASK_PRIORITY, &ins->task_handle);
    if (task_ret != pdPASS)
    {
        ESP_LOGE(TAG, "xTaskCreate failed");
        vSemaphoreDelete(ins->mutex);
        free(ins);
        uart_driver_delete(uart_port);
        return NULL;
    }

    ESP_LOGI(TAG, "AprilTag receiver initialized (uart=%d, baud=%d, tx=%d, rx=%d)",
             uart_port, baud_rate, tx_pin, rx_pin);
    return ins;
}

bool AprilTag_GetResult(AprilTagInstance *ins, AprilTag_Result_t *result)
{
    if (ins == NULL || result == NULL)
    {
        return false;
    }

    bool has_new = false;

    if (xSemaphoreTake(ins->mutex, pdMS_TO_TICKS(5)) == pdTRUE)
    {
        *result       = ins->latest;
        has_new       = ins->new_data;
        ins->new_data = false;
        xSemaphoreGive(ins->mutex);
    }

    return has_new;
}

void AprilTag_Deinit(AprilTagInstance *ins)
{
    if (ins == NULL)
    {
        return;
    }

    /* 通知后台任务退出，等待其完成 */
    ins->caller_handle = xTaskGetCurrentTaskHandle();
    ins->running = false;

    /* 等待后台任务发送退出通知（最多等待 200ms） */
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(200));

    ins->task_handle = NULL;

    vSemaphoreDelete(ins->mutex);
    uart_driver_delete(ins->uart_port);
    free(ins);

    ESP_LOGI(TAG, "AprilTag receiver deinitialized");
}
