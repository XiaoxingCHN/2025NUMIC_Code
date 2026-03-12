#include "BSP_spi.h"

#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "freertos/task.h"

static const char *TAG = "BSP_SPI";

/* 已注册实例表 */
static SPIInstance *spi_instance[SPI_DEVICE_CNT] = {0};
/* 忙标志数组：0忙，1空闲 */
static volatile uint8_t SPIDeviceOnGoing[SPI_DEVICE_CNT] = {0};
/* 已注册设备数量 */
static size_t instance_count = 0;
/* 总线状态 */
static bool bus_initialized = false;
static spi_host_device_t initialized_host = BSP_SPI_DEFAULT_HOST;

/* -------------------- 内部工具函数 -------------------- */

static int find_instance_index(SPIInstance *ins)
{
    for (size_t i = 0; i < instance_count; i++)
    {
        if (spi_instance[i] == ins)
        {
            return (int)i;
        }
    }
    return -1;
}

static void compact_instance_table(size_t idx_remove)
{
    if (idx_remove >= instance_count) return;

    for (size_t i = idx_remove; i + 1 < instance_count; i++)
    {
        spi_instance[i] = spi_instance[i + 1];
        /* 重新绑定忙标志指针，保持数组连续 */
        if (spi_instance[i] != NULL)
        {
            spi_instance[i]->cs_pin_state = &SPIDeviceOnGoing[i];
        }
        SPIDeviceOnGoing[i] = SPIDeviceOnGoing[i + 1];
    }

    spi_instance[instance_count - 1] = NULL;
    SPIDeviceOnGoing[instance_count - 1] = 1;
    instance_count--;
}

static SPI_TransSlot_s *alloc_trans_slot(SPIInstance *ins)
{
    if (!ins) return NULL;
    for (size_t i = 0; i < SPI_TRANS_POOL_SIZE; i++)
    {
        if (ins->trans_pool[i].in_use == 0)
        {
            ins->trans_pool[i].in_use = 1;
            memset(&ins->trans_pool[i].trans, 0, sizeof(spi_transaction_t));
            return &ins->trans_pool[i];
        }
    }
    return NULL;
}

static void free_trans_slot_by_trans(SPIInstance *ins, spi_transaction_t *t)
{
    if (!ins || !t) return;
    for (size_t i = 0; i < SPI_TRANS_POOL_SIZE; i++)
    {
        if (&ins->trans_pool[i].trans == t)
        {
            ins->trans_pool[i].in_use = 0;
            return;
        }
    }
}

/* 事务完成回调（ISR上下文） */
static void IRAM_ATTR spi_post_cb(spi_transaction_t *trans)
{
    SPIInstance *ins = (SPIInstance *)trans->user;
    if (ins == NULL)
    {
        return;
    }

    if (ins->cs_pin_state)
    {
        *ins->cs_pin_state = 1;
    }

    ins->done_isr_flag = 1;
}

/* 确保总线初始化 */
static esp_err_t ensure_bus_initialized(spi_host_device_t host)
{
    if (bus_initialized)
    {
        if (host == initialized_host)
        {
            return ESP_OK;
        }
        ESP_LOGE(TAG, "SPI bus already initialized on host %d; requested host %d",
                 initialized_host, host);
        return ESP_ERR_INVALID_STATE;
    }

    spi_bus_config_t buscfg = {
        .miso_io_num = SPI_MISO_PIN,
        .mosi_io_num = SPI_MOSI_PIN,
        .sclk_io_num = SPI_CLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = BSP_SPI_MAX_TRANSFER_SZ,
        .intr_flags = 0,
    };

    esp_err_t ret = spi_bus_initialize(host, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(ret));
        return ret;
    }

    bus_initialized = true;
    initialized_host = host;
    ESP_LOGI(TAG, "SPI bus initialized (host=%d)", host);
    return ESP_OK;
}

/* -------------------- 对外 API -------------------- */

esp_err_t SPI_Init(void)
{
    return ensure_bus_initialized(BSP_SPI_DEFAULT_HOST);
}

SPIInstance *SPIRegister(const SPI_Init_Config_s *conf)
{
    if (conf == NULL)
    {
        ESP_LOGE(TAG, "SPIRegister: conf is NULL");
        return NULL;
    }

    if (instance_count >= SPI_DEVICE_CNT)
    {
        ESP_LOGE(TAG, "SPIRegister: exceed max device count (%d)", SPI_DEVICE_CNT);
        return NULL;
    }

    spi_host_device_t host = conf->host;
    if (host != SPI2_HOST && host != SPI3_HOST && host != SPI1_HOST)
    {
        host = BSP_SPI_DEFAULT_HOST;
    }

    esp_err_t ret = ensure_bus_initialized(host);
    if (ret != ESP_OK)
    {
        return NULL;
    }

    spi_device_interface_config_t devcfg = {
        .mode = conf->spi_mode & 0x3,
        .clock_speed_hz = (conf->clock_speed_hz > 0) ? conf->clock_speed_hz : (10 * 1000 * 1000),
        .spics_io_num = conf->cs_pin,
        .queue_size = BSP_SPI_QUEUE_SIZE,
        .flags = 0,
        .pre_cb = NULL,
        .post_cb = spi_post_cb,
    };

    spi_device_handle_t dev_handle = NULL;
    ret = spi_bus_add_device(host, &devcfg, &dev_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "spi_bus_add_device failed: %s", esp_err_to_name(ret));
        return NULL;
    }

    SPIInstance *instance = (SPIInstance *)calloc(1, sizeof(SPIInstance));
    if (instance == NULL)
    {
        ESP_LOGE(TAG, "SPIRegister: calloc failed");
        spi_bus_remove_device(dev_handle);
        return NULL;
    }

    instance->spi_handle = dev_handle;
    instance->host = host;
    instance->cs_pin = conf->cs_pin;
    instance->spi_work_mode = conf->spi_work_mode;
    instance->callback = conf->callback;
    instance->id = conf->id;
    instance->rx_buffer = NULL;
    instance->rx_size = 0;
    instance->done_isr_flag = 0;

    for (size_t i = 0; i < SPI_TRANS_POOL_SIZE; i++)
    {
        instance->trans_pool[i].in_use = 0;
        memset(&instance->trans_pool[i].trans, 0, sizeof(spi_transaction_t));
    }

    instance->cs_pin_state = &SPIDeviceOnGoing[instance_count];
    *instance->cs_pin_state = 1;

    spi_instance[instance_count++] = instance;

    ESP_LOGI(TAG, "SPI device registered on host %d, CS=%d", host, conf->cs_pin);
    return instance;
}

esp_err_t SPIUnregister(SPIInstance *spi_ins)
{
    if (spi_ins == NULL)
    {
        ESP_LOGE(TAG, "SPIUnregister: spi_ins is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    int idx = find_instance_index(spi_ins);
    if (idx < 0)
    {
        ESP_LOGE(TAG, "SPIUnregister: instance not found");
        return ESP_ERR_NOT_FOUND;
    }

    /* 尝试回收队列中已完成事务，避免残留 */
    while (SPIProcessDone(spi_ins, 0) == ESP_OK)
    {
        /* drain */
    }

    esp_err_t ret = spi_bus_remove_device(spi_ins->spi_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "spi_bus_remove_device failed: %s", esp_err_to_name(ret));
        return ret;
    }

    free(spi_ins);
    compact_instance_table((size_t)idx);

    ESP_LOGI(TAG, "SPI device unregistered");
    return ESP_OK;
}

esp_err_t SPIDeinit(void)
{
    if (!bus_initialized)
    {
        return ESP_OK;
    }

    if (instance_count != 0)
    {
        ESP_LOGE(TAG, "SPIDeinit: still has %u device(s), please unregister first",
                 (unsigned)instance_count);
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = spi_bus_free(initialized_host);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "spi_bus_free failed: %s", esp_err_to_name(ret));
        return ret;
    }

    bus_initialized = false;
    initialized_host = BSP_SPI_DEFAULT_HOST;
    ESP_LOGI(TAG, "SPI bus deinitialized");
    return ESP_OK;
}

/* 内部：按模式提交事务 */
static esp_err_t submit_trans(SPIInstance *spi_ins, spi_transaction_t *trans)
{
    if (!spi_ins || !trans)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (spi_ins->cs_pin_state)
    {
        *spi_ins->cs_pin_state = 0;
    }

    esp_err_t ret = ESP_OK;

    switch (spi_ins->spi_work_mode)
    {
    case SPI_DMA_MODE:
    case SPI_IT_MODE:
        ret = spi_device_queue_trans(spi_ins->spi_handle, trans, portMAX_DELAY);
        if (ret != ESP_OK)
        {
            if (spi_ins->cs_pin_state) *spi_ins->cs_pin_state = 1;
        }
        break;

    case SPI_BLOCK_MODE:
        ret = spi_device_polling_transmit(spi_ins->spi_handle, trans);
        if (spi_ins->cs_pin_state) *spi_ins->cs_pin_state = 1;
        spi_ins->CS_State = gpio_get_level(spi_ins->cs_pin);
        if (spi_ins->callback)
        {
            /* 阻塞模式下已完成，直接任务上下文回调 */
            spi_ins->callback(spi_ins);
        }
        break;

    default:
        ret = ESP_ERR_INVALID_ARG;
        if (spi_ins->cs_pin_state) *spi_ins->cs_pin_state = 1;
        break;
    }

    return ret;
}

esp_err_t SPITransmit(SPIInstance *spi_ins, const uint8_t *ptr_data, size_t len)
{
    if (spi_ins == NULL || ptr_data == NULL || len == 0)
    {
        ESP_LOGE(TAG, "SPITransmit: invalid args");
        return ESP_ERR_INVALID_ARG;
    }

    if (spi_ins->spi_work_mode == SPI_BLOCK_MODE)
    {
        spi_transaction_t trans = {0};
        trans.length = len * 8;
        trans.tx_buffer = ptr_data;
        trans.user = spi_ins;
        esp_err_t ret = submit_trans(spi_ins, &trans);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "SPITransmit polling failed: %s", esp_err_to_name(ret));
        }
        return ret;
    }
    else
    {
        SPI_TransSlot_s *slot = alloc_trans_slot(spi_ins);
        if (!slot)
        {
            ESP_LOGE(TAG, "SPITransmit: no free trans slot");
            return ESP_ERR_NO_MEM;
        }

        slot->trans.length = len * 8;
        slot->trans.tx_buffer = ptr_data;
        slot->trans.user = spi_ins;

        esp_err_t ret = submit_trans(spi_ins, &slot->trans);
        if (ret != ESP_OK)
        {
            free_trans_slot_by_trans(spi_ins, &slot->trans);
            ESP_LOGE(TAG, "SPITransmit queue failed: %s", esp_err_to_name(ret));
        }
        return ret;
    }
}

esp_err_t SPIRecv(SPIInstance *spi_ins, uint8_t *ptr_data, size_t len)
{
    if (spi_ins == NULL || ptr_data == NULL || len == 0)
    {
        ESP_LOGE(TAG, "SPIRecv: invalid args");
        return ESP_ERR_INVALID_ARG;
    }

    spi_ins->rx_size = len;
    spi_ins->rx_buffer = ptr_data;

    if (spi_ins->spi_work_mode == SPI_BLOCK_MODE)
    {
        spi_transaction_t trans = {0};
        trans.length = len * 8;
        trans.rxlength = len * 8;
        trans.rx_buffer = ptr_data;
        trans.user = spi_ins;

        esp_err_t ret = submit_trans(spi_ins, &trans);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "SPIRecv polling failed: %s", esp_err_to_name(ret));
        }
        return ret;
    }
    else
    {
        SPI_TransSlot_s *slot = alloc_trans_slot(spi_ins);
        if (!slot)
        {
            ESP_LOGE(TAG, "SPIRecv: no free trans slot");
            return ESP_ERR_NO_MEM;
        }

        slot->trans.length = len * 8;
        slot->trans.rxlength = len * 8;
        slot->trans.rx_buffer = ptr_data;
        slot->trans.user = spi_ins;

        esp_err_t ret = submit_trans(spi_ins, &slot->trans);
        if (ret != ESP_OK)
        {
            free_trans_slot_by_trans(spi_ins, &slot->trans);
            ESP_LOGE(TAG, "SPIRecv queue failed: %s", esp_err_to_name(ret));
        }
        return ret;
    }
}

esp_err_t SPITransRecv(SPIInstance *spi_ins, uint8_t *ptr_data_rx, const uint8_t *ptr_data_tx, size_t len)
{
    if (spi_ins == NULL || ptr_data_rx == NULL || ptr_data_tx == NULL || len == 0)
    {
        ESP_LOGE(TAG, "SPITransRecv: invalid args");
        return ESP_ERR_INVALID_ARG;
    }

    spi_ins->rx_size = len;
    spi_ins->rx_buffer = ptr_data_rx;

    if (spi_ins->spi_work_mode == SPI_BLOCK_MODE)
    {
        spi_transaction_t trans = {0};
        trans.length = len * 8;
        trans.rxlength = len * 8;
        trans.tx_buffer = ptr_data_tx;
        trans.rx_buffer = ptr_data_rx;
        trans.user = spi_ins;

        esp_err_t ret = submit_trans(spi_ins, &trans);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "SPITransRecv polling failed: %s", esp_err_to_name(ret));
        }
        return ret;
    }
    else
    {
        SPI_TransSlot_s *slot = alloc_trans_slot(spi_ins);
        if (!slot)
        {
            ESP_LOGE(TAG, "SPITransRecv: no free trans slot");
            return ESP_ERR_NO_MEM;
        }

        slot->trans.length = len * 8;
        slot->trans.rxlength = len * 8;
        slot->trans.tx_buffer = ptr_data_tx;
        slot->trans.rx_buffer = ptr_data_rx;
        slot->trans.user = spi_ins;

        esp_err_t ret = submit_trans(spi_ins, &slot->trans);
        if (ret != ESP_OK)
        {
            free_trans_slot_by_trans(spi_ins, &slot->trans);
            ESP_LOGE(TAG, "SPITransRecv queue failed: %s", esp_err_to_name(ret));
        }
        return ret;
    }
}

esp_err_t SPISetMode(SPIInstance *spi_ins, SPI_TXRX_MODE_e spi_mode)
{
    if (spi_ins == NULL)
    {
        ESP_LOGE(TAG, "SPISetMode: spi_ins is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    if (spi_mode != SPI_DMA_MODE &&
        spi_mode != SPI_IT_MODE &&
        spi_mode != SPI_BLOCK_MODE)
    {
        ESP_LOGE(TAG, "SPISetMode: invalid mode %d", spi_mode);
        return ESP_ERR_INVALID_ARG;
    }

    spi_ins->spi_work_mode = spi_mode;
    return ESP_OK;
}

esp_err_t SPIProcessDone(SPIInstance *spi_ins, TickType_t wait_ticks)
{
    if (spi_ins == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (spi_ins->spi_work_mode == SPI_BLOCK_MODE)
    {
        /* 阻塞模式不需要 get_trans_result */
        return ESP_ERR_INVALID_STATE;
    }

    spi_transaction_t *rtrans = NULL;
    esp_err_t ret = spi_device_get_trans_result(spi_ins->spi_handle, &rtrans, wait_ticks);
    if (ret != ESP_OK)
    {
        return ret; /* ESP_ERR_TIMEOUT 等 */
    }

    free_trans_slot_by_trans(spi_ins, rtrans);

    spi_ins->CS_State = gpio_get_level(spi_ins->cs_pin);
    spi_ins->done_isr_flag = 0;

    /* 在任务上下文执行用户回调 */
    if (spi_ins->callback)
    {
        spi_ins->callback(spi_ins);
    }

    return ESP_OK;
}