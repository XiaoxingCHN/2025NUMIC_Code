#ifndef __DEVICE_APRILTAG_H__
#define __DEVICE_APRILTAG_H__

#include <stdint.h>
#include <stdbool.h>
#include "driver/uart.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================= 用户可按工程修改的宏 ========================= */
#ifndef APRILTAG_UART_PORT
#define APRILTAG_UART_PORT       UART_NUM_1
#endif

#ifndef APRILTAG_UART_BAUD
#define APRILTAG_UART_BAUD       115200
#endif

#ifndef APRILTAG_UART_TX_PIN
#define APRILTAG_UART_TX_PIN     17
#endif

#ifndef APRILTAG_UART_RX_PIN
#define APRILTAG_UART_RX_PIN     18
#endif

#ifndef APRILTAG_RX_BUF_SIZE
#define APRILTAG_RX_BUF_SIZE     512
#endif

#ifndef APRILTAG_TASK_STACK_SIZE
#define APRILTAG_TASK_STACK_SIZE 4096
#endif

#ifndef APRILTAG_TASK_PRIORITY
#define APRILTAG_TASK_PRIORITY   5
#endif

/* 通信帧格式：帧头 + 数据 + 校验
 *  [0]    帧头高字节  0xAA
 *  [1]    帧头低字节  0xFF
 *  [2]    标签 ID (uint8_t)
 *  [3]    有效标志 (0x01 = 检测到；0x00 = 未检测到)
 *  [4-7]  X 位移 (float, little-endian, 单位 mm)
 *  [8-11] Y 位移 (float, little-endian, 单位 mm)
 *  [12-15]Z 位移 (float, little-endian, 单位 mm)
 *  [16-19]偏航角 Yaw (float, little-endian, 单位 °)
 *  [20]   校验和（[2]..[19] 的字节累加，取低 8 位）
 */
#define APRILTAG_FRAME_LEN       21
#define APRILTAG_FRAME_HEADER_0  0xAAU
#define APRILTAG_FRAME_HEADER_1  0xFFU
/* ====================================================================== */

/* AprilTag 检测结果 */
typedef struct
{
    uint8_t tag_id;     /* 检测到的标签 ID */
    bool    detected;   /* 是否检测到标签 */
    float   x;          /* 相机坐标系下的 X 偏移量 (mm) */
    float   y;          /* 相机坐标系下的 Y 偏移量 (mm) */
    float   z;          /* 距标签的距离 (mm) */
    float   yaw;        /* 绕相机 Z 轴偏航角 (°) */
} AprilTag_Result_t;

/* 检测结果更新回调；在任务上下文中调用 */
typedef void (*AprilTag_Callback_t)(const AprilTag_Result_t *result);

/* 初始化配置 */
typedef struct
{
    uart_port_t          uart_port;     /* UART 端口号 */
    int                  baud_rate;     /* 波特率 */
    int                  tx_pin;        /* TX GPIO */
    int                  rx_pin;        /* RX GPIO */
    AprilTag_Callback_t  callback;      /* 新数据到达时的回调（可为 NULL） */
} AprilTag_Config_t;

/* 不透明句柄 */
typedef struct AprilTagInstance AprilTagInstance;

/* ----------------------------- API ------------------------------------ */

/**
 * @brief  初始化 AprilTag UART 接收器并启动后台解析任务
 *
 * @param  cfg  初始化配置；若为 NULL 则使用默认宏定义
 * @return 成功返回句柄，失败返回 NULL
 */
AprilTagInstance *AprilTag_Init(const AprilTag_Config_t *cfg);

/**
 * @brief  获取最新的检测结果（线程安全）
 *
 * @param  ins     由 AprilTag_Init 返回的句柄
 * @param  result  输出参数，存放最新结果
 * @return true 表示自上次调用以来有新数据；false 表示无更新
 */
bool AprilTag_GetResult(AprilTagInstance *ins, AprilTag_Result_t *result);

/**
 * @brief  反初始化并释放资源
 *
 * @param  ins  由 AprilTag_Init 返回的句柄
 */
void AprilTag_Deinit(AprilTagInstance *ins);

#ifdef __cplusplus
}
#endif

#endif /* __DEVICE_APRILTAG_H__ */
