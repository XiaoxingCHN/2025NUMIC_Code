#ifndef __BSP_SPI_H__
#define __BSP_SPI_H__

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================= 用户可按工程修改的宏 ========================= */
#ifndef SPI_DEVICE_CNT
#define SPI_DEVICE_CNT            8
#endif

#ifndef BSP_SPI_DEFAULT_HOST
#define BSP_SPI_DEFAULT_HOST      SPI2_HOST
#endif

#ifndef SPI_MISO_PIN
#define SPI_MISO_PIN              13
#endif

#ifndef SPI_MOSI_PIN
#define SPI_MOSI_PIN              11
#endif

#ifndef SPI_CLK_PIN
#define SPI_CLK_PIN               12
#endif

#ifndef BSP_SPI_QUEUE_SIZE
#define BSP_SPI_QUEUE_SIZE        4
#endif

#ifndef BSP_SPI_MAX_TRANSFER_SZ
#define BSP_SPI_MAX_TRANSFER_SZ   4096
#endif

#ifndef SPI_TRANS_POOL_SIZE
#define SPI_TRANS_POOL_SIZE       BSP_SPI_QUEUE_SIZE
#endif
/* ====================================================================== */

typedef enum
{
    SPI_DMA_MODE   = 0,
    SPI_IT_MODE    = 1,
    SPI_BLOCK_MODE = 2,
} SPI_TXRX_MODE_e;

struct SPIInstance;
typedef struct SPIInstance SPIInstance;

typedef void (*SPI_Callback_t)(SPIInstance *ins);

/* 注册配置 */
typedef struct
{
    spi_host_device_t host;       /* SPI1/SPI2/SPI3 (推荐 SPI2/SPI3) */
    int cs_pin;                   /* 片选脚 */
    uint8_t spi_mode;             /* 0~3 */
    int clock_speed_hz;           /* <=0 则默认 10MHz */
    SPI_TXRX_MODE_e spi_work_mode;
    SPI_Callback_t callback;
    uint32_t id;
} SPI_Init_Config_s;

/* 事务槽位：用于异步模式持久化事务对象 */
typedef struct
{
    spi_transaction_t trans;
    uint8_t in_use;
} SPI_TransSlot_s;

struct SPIInstance
{
    spi_device_handle_t spi_handle;
    spi_host_device_t host;
    int cs_pin;
    int CS_State;
    SPI_TXRX_MODE_e spi_work_mode;
    SPI_Callback_t callback;
    uint32_t id;

    uint8_t *rx_buffer;
    size_t rx_size;

    volatile uint8_t *cs_pin_state;   /* 0忙/1空闲 */

    /* 异步完成标记（在 ISR 回调置位） */
    volatile uint8_t done_isr_flag;

    /* 固定事务池，避免异步使用栈变量 */
    SPI_TransSlot_s trans_pool[SPI_TRANS_POOL_SIZE];
};

/* API */
esp_err_t SPI_Init(void);
SPIInstance *SPIRegister(const SPI_Init_Config_s *conf);
esp_err_t SPIUnregister(SPIInstance *spi_ins);
esp_err_t SPIDeinit(void);

esp_err_t SPITransmit(SPIInstance *spi_ins, const uint8_t *ptr_data, size_t len);
esp_err_t SPIRecv(SPIInstance *spi_ins, uint8_t *ptr_data, size_t len);
esp_err_t SPITransRecv(SPIInstance *spi_ins, uint8_t *ptr_data_rx, const uint8_t *ptr_data_tx, size_t len);

esp_err_t SPISetMode(SPIInstance *spi_ins, SPI_TXRX_MODE_e spi_mode);

/* 异步完成回收：内部调用 spi_device_get_trans_result */
esp_err_t SPIProcessDone(SPIInstance *spi_ins, TickType_t wait_ticks);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_SPI_H__ */