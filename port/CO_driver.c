/*
 * CAN module object for generic microcontroller.
 *
 * This file is a template for other microcontrollers.
 *
 * @file        CO_driver.c
 * @ingroup     CO_driver
 * @author      Janez Paternoster / Sicris Embay
 * @copyright   2004 - 2023 Janez Paternoster / Sicris Embay
 *
 * This file is part of CANopenNode, an opensource CANopen Stack.
 * Project home page is <https://github.com/CANopenNode/CANopenNode>.
 * For more information on CANopen see <http://www.can-cia.org/>.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <string.h>
#include "301/CO_driver.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "freertos/queue.h"

static const char *TAG = "CO_driver";

#define CO_RX_QUEUE_LEN CONFIG_CO_TWAI_RX_QUEUE_LEN
#define CO_TX_QUEUE_LEN CONFIG_CO_TWAI_TX_QUEUE_LEN
/* The driver queues pointers to frames, so each frame must stay valid until its
 * on_tx_done callback. One frame can be in the controller while CO_TX_QUEUE_LEN
 * more wait in the driver queue. */
#define CO_TX_POOL_LEN (CO_TX_QUEUE_LEN + 1)
/* How long the Tx task waits for a free TX slot before giving up on a frame */
#define CO_TX_TASK_SLOT_TIMEOUT_MS 1000

typedef struct
{
    twai_frame_t frame;
    uint8_t data[TWAI_FRAME_MAX_LEN];
} tx_slot_t;

static twai_node_handle_t twaiNode = NULL;

static tx_slot_t txPool[CO_TX_POOL_LEN];
static StaticQueue_t txFreeQueueBuf;
static uint8_t txFreeQueueStorage[CO_TX_POOL_LEN * sizeof(tx_slot_t *)];
static QueueHandle_t txFreeQueue = NULL;

static StaticQueue_t rxQueueBuf;
static uint8_t rxQueueStorage[CO_RX_QUEUE_LEN * sizeof(CO_CANrxMsg_t)];
static QueueHandle_t rxQueue = NULL;

/* Counters written from the TWAI ISR */
static volatile uint32_t rxQueueOverflowCount = 0;
static volatile uint32_t txFailedCount = 0;
static volatile uint32_t busOffCount = 0;

static StaticTask_t xCoTxTaskBuffer;
static StackType_t xCoTxStack[CONFIG_CO_TX_TASK_STACK_SIZE];
static TaskHandle_t xCoTxTaskHandle = NULL;
static void CO_txTask(void *pxParam);

static StaticTask_t xCoRxTaskBuffer;
static StackType_t xCoRxStack[CONFIG_CO_RX_TASK_STACK_SIZE];
static TaskHandle_t xCoRxTaskHandle = NULL;
static void CO_rxTask(void *pxParam);

static bool bInstalled = false;

/******************************************************************************/
static bool CO_twaiOnRxDone(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx)
{
    CO_CANrxMsg_t msg;
    twai_frame_t frame = {
        .buffer = msg.data,
        .buffer_len = sizeof(msg.data),
    };
    BaseType_t woken = pdFALSE;

    if (twai_node_receive_from_isr(handle, &frame) != ESP_OK)
    {
        return false;
    }
    if (frame.header.ide || frame.header.fdf)
    {
        /* CANopen uses 11-bit classic frames only */
        return false;
    }
    msg.ident = (uint16_t)(frame.header.id & TWAI_STD_ID_MASK);
    msg.rtr = frame.header.rtr;
    msg.DLC = (uint8_t)(frame.header.dlc > TWAI_FRAME_MAX_DLC ? TWAI_FRAME_MAX_DLC : frame.header.dlc);

    if (xQueueSendFromISR(rxQueue, &msg, &woken) != pdTRUE)
    {
        rxQueueOverflowCount++;
    }
    return woken == pdTRUE;
}

static bool CO_twaiOnTxDone(twai_node_handle_t handle, const twai_tx_done_event_data_t *edata, void *user_ctx)
{
    BaseType_t woken = pdFALSE;

    if (!edata->is_tx_success)
    {
        txFailedCount++;
    }
    if (edata->done_tx_frame != NULL)
    {
        /* frame is the first member of tx_slot_t */
        tx_slot_t *slot = (tx_slot_t *)edata->done_tx_frame;
        xQueueSendFromISR(txFreeQueue, &slot, &woken);
    }
    return woken == pdTRUE;
}

static bool CO_twaiOnStateChange(twai_node_handle_t handle, const twai_state_change_event_data_t *edata, void *user_ctx)
{
    if (edata->new_sta == TWAI_ERROR_BUS_OFF && edata->old_sta != TWAI_ERROR_BUS_OFF)
    {
        busOffCount++;
    }
    return false;
}

/* Queue one CANopen tx buffer on the TWAI node. slotTimeoutMs is how long to wait
 * for a free TX slot; the driver queue itself never blocks since the pool is sized
 * to fit in it. */
static esp_err_t CO_twaiTransmit(const CO_CANtx_t *buffer, uint32_t slotTimeoutMs)
{
    tx_slot_t *slot;

    if (xQueueReceive(txFreeQueue, &slot, pdMS_TO_TICKS(slotTimeoutMs)) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }

    bool rtr = (buffer->ident & 0x0800U) != 0U;
    uint8_t dlc = buffer->DLC > TWAI_FRAME_MAX_LEN ? TWAI_FRAME_MAX_LEN : buffer->DLC;

    memset(&slot->frame, 0, sizeof(slot->frame));
    slot->frame.header.id = buffer->ident & TWAI_STD_ID_MASK;
    slot->frame.header.rtr = rtr;
    slot->frame.header.dlc = dlc;
    slot->frame.buffer = slot->data;
    slot->frame.buffer_len = rtr ? 0 : dlc;
    memcpy(slot->data, buffer->data, dlc);

    esp_err_t espRet = twai_node_transmit(twaiNode, &slot->frame, 0);
    if (espRet != ESP_OK)
    {
        xQueueSend(txFreeQueue, &slot, 0);
    }
    return espRet;
}

/******************************************************************************/
void CO_CANsetConfigurationMode(void *CANptr)
{
    /* Put CAN module in configuration mode */
}

/******************************************************************************/
void CO_CANsetNormalMode(CO_CANmodule_t *CANmodule)
{
    /* Put CAN module in normal mode */

    CANmodule->CANnormal = true;
}

/******************************************************************************/
CO_ReturnError_t CO_CANmodule_init(
    CO_CANmodule_t *CANmodule,
    void *CANptr,
    CO_CANrx_t rxArray[],
    uint16_t rxSize,
    CO_CANtx_t txArray[],
    uint16_t txSize,
    uint16_t CANbitRate)
{
    uint16_t i;

    /* verify arguments */
    if (CANmodule == NULL || rxArray == NULL || txArray == NULL)
    {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

#if CONFIG_CO_LED_ENABLE
    gpio_config_t io_conf;
    uint64_t pinMask = 0ULL;
#if (CONFIG_CO_LED_RED_GPIO >= 0)
    pinMask |= (1ULL << CONFIG_CO_LED_RED_GPIO);
#endif
#if (CONFIG_CO_LED_GREEN_GPIO >= 0)
    pinMask |= (1ULL << CONFIG_CO_LED_GREEN_GPIO);
#endif
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = pinMask;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    /* Set LED off */
#if (CONFIG_CO_LED_RED_GPIO >= 0)
    gpio_set_level(CONFIG_CO_LED_RED_GPIO,
#if CONFIG_CO_LED_RED_ACTIVE_HIGH
                   0);
#else
                   1);
#endif /* CONFIG_CO_LED_RED_ACTIVE_HIGH */
#endif

#if (CONFIG_CO_LED_GREEN_GPIO >= 0)
    gpio_set_level(CONFIG_CO_LED_GREEN_GPIO,
#if CONFIG_CO_LED_GREEN_ACTIVE_HIGH
                   0);
#else
                   1);
#endif
#endif
#endif /* CONFIG_CO_LED_ENABLE */

    /* Configure object variables */
    CANmodule->CANptr = CANptr;
    CANmodule->rxArray = rxArray;
    CANmodule->rxSize = rxSize;
    CANmodule->txArray = txArray;
    CANmodule->txSize = txSize;
    CANmodule->CANerrorStatus = 0;
    CANmodule->CANnormal = false;
    CANmodule->useCANrxFilters = false;
    CANmodule->bufferInhibitFlag = false;
    CANmodule->firstCANtxMessage = true;
    CANmodule->CANtxCount = 0U;
    CANmodule->errOld = 0U;

    for (i = 0U; i < rxSize; i++)
    {
        rxArray[i].ident = 0U;
        rxArray[i].mask = 0xFFFFU;
        rxArray[i].object = NULL;
        rxArray[i].CANrx_callback = NULL;
    }
    for (i = 0U; i < txSize; i++)
    {
        txArray[i].bufferFull = false;
    }

    /* Install TWAI driver */
    if (bInstalled != true)
    {
        twai_onchip_node_config_t node_config = {
            .io_cfg = {
                .tx = CONFIG_CO_TWAI_TX_GPIO,
                .rx = CONFIG_CO_TWAI_RX_GPIO,
                .quanta_clk_out = GPIO_NUM_NC,
                .bus_off_indicator = GPIO_NUM_NC,
            },
            /* sample point left at the driver default (80% at 500k, 87.5% below) */
            .bit_timing = {
                .bitrate = (uint32_t)CANbitRate * 1000U,
            },
            .fail_retry_cnt = -1, /* retransmit until success, as the legacy driver did */
            .tx_queue_depth = CO_TX_QUEUE_LEN,
        };
        const twai_event_callbacks_t cbs = {
            .on_rx_done = CO_twaiOnRxDone,
            .on_tx_done = CO_twaiOnTxDone,
            .on_state_change = CO_twaiOnStateChange,
        };

        esp_err_t espRet = twai_new_node_onchip(&node_config, &twaiNode);
        if (espRet != ESP_OK)
        {
            ESP_LOGE(TAG, "twai_new_node_onchip(%u kbps) failed: %s", CANbitRate, esp_err_to_name(espRet));
            twaiNode = NULL;
            return (espRet == ESP_ERR_INVALID_ARG) ? CO_ERROR_ILLEGAL_BAUDRATE : CO_ERROR_OUT_OF_MEMORY;
        }

        rxQueue = xQueueCreateStatic(CO_RX_QUEUE_LEN, sizeof(CO_CANrxMsg_t), rxQueueStorage, &rxQueueBuf);
        txFreeQueue = xQueueCreateStatic(CO_TX_POOL_LEN, sizeof(tx_slot_t *), txFreeQueueStorage, &txFreeQueueBuf);
        for (i = 0; i < CO_TX_POOL_LEN; i++)
        {
            tx_slot_t *slot = &txPool[i];
            xQueueSend(txFreeQueue, &slot, 0);
        }
        rxQueueOverflowCount = 0;
        txFailedCount = 0;
        busOffCount = 0;

        /* create Mutex */
        CANmodule->xMutexCanSendHdl = xSemaphoreCreateRecursiveMutexStatic(&(CANmodule->xMutexCanSendBuf));
        CANmodule->xMutexEmcyHdl = xSemaphoreCreateRecursiveMutexStatic(&(CANmodule->xMutexEmcyBuf));
        CANmodule->xMutexODHdl = xSemaphoreCreateRecursiveMutexStatic(&(CANmodule->xMutexODBuf));

        /* Start TWAI */
        ESP_ERROR_CHECK(twai_node_register_event_callbacks(twaiNode, &cbs, CANmodule));
        ESP_ERROR_CHECK(twai_node_enable(twaiNode));
        ESP_LOGI(TAG, "Driver started, %u kbps, rx queue %d, tx queue %d", CANbitRate, CO_RX_QUEUE_LEN, CO_TX_QUEUE_LEN);

        bInstalled = true;

        /* Create Tx tasks */
        ESP_LOGI(TAG, "Creating Tx Task");
        xCoTxTaskHandle = xTaskCreateStaticPinnedToCore(
            CO_txTask,
            "CO_tx",
            CONFIG_CO_TX_TASK_STACK_SIZE,
            (void *)CANmodule,
            CONFIG_CO_TX_TASK_PRIORITY,
            &xCoTxStack[0],
            &xCoTxTaskBuffer,
            CONFIG_CO_TASK_CORE);
        if (xCoTxTaskHandle == NULL)
        {
            ESP_LOGE(TAG, "txTask creation failed");
            return CO_ERROR_OUT_OF_MEMORY;
        }
        /* Create Rx tasks */
        ESP_LOGI(TAG, "Creating Rx Task");
        xCoRxTaskHandle = xTaskCreateStaticPinnedToCore(
            CO_rxTask,
            "CO_rx",
            CONFIG_CO_RX_TASK_STACK_SIZE,
            (void *)CANmodule,
            CONFIG_CO_RX_TASK_PRIORITY,
            &xCoRxStack[0],
            &xCoRxTaskBuffer,
            CONFIG_CO_TASK_CORE);
        if (xCoRxTaskHandle == NULL)
        {
            ESP_LOGE(TAG, "rxTask creation failed");
            return CO_ERROR_OUT_OF_MEMORY;
        }
    }
    else
    {
        ESP_LOGI(TAG, "Driver already installed");
    }

    return CO_ERROR_NO;
}

/******************************************************************************/
void CO_CANmodule_disable(CO_CANmodule_t *CANmodule)
{
    if (CANmodule != NULL)
    {
        /* Take all mutex before deleting it */
        xSemaphoreTakeRecursive(CANmodule->xMutexCanSendHdl, portMAX_DELAY);
        xSemaphoreTakeRecursive(CANmodule->xMutexEmcyHdl, portMAX_DELAY);
        xSemaphoreTakeRecursive(CANmodule->xMutexODHdl, portMAX_DELAY);

        /* Delete Tx and Rx Tasks */
        vTaskDelete(xCoTxTaskHandle);
        xCoTxTaskHandle = NULL;
        vTaskDelete(xCoRxTaskHandle);
        xCoRxTaskHandle = NULL;
        ESP_LOGI(TAG, "tx and rx tasks deleted");

        /* As holder of mutex, it is safe to delete it */
        vSemaphoreDelete(CANmodule->xMutexCanSendHdl);
        vSemaphoreDelete(CANmodule->xMutexEmcyHdl);
        vSemaphoreDelete(CANmodule->xMutexODHdl);
        CANmodule->xMutexCanSendHdl = NULL;
        CANmodule->xMutexEmcyHdl = NULL;
        CANmodule->xMutexODHdl = NULL;
        ESP_LOGI(TAG, "mutex deleted");

        /* Uninstall TWAI. Disabling the node stops its ISR, so the queues can go after it. */
        ESP_ERROR_CHECK(twai_node_disable(twaiNode));
        ESP_LOGI(TAG, "Driver stopped");
        ESP_ERROR_CHECK(twai_node_delete(twaiNode));
        twaiNode = NULL;
        ESP_LOGI(TAG, "Driver uninstalled");

        vQueueDelete(rxQueue);
        rxQueue = NULL;
        vQueueDelete(txFreeQueue);
        txFreeQueue = NULL;

        bInstalled = false;
    }
}

/******************************************************************************/
CO_ReturnError_t CO_CANrxBufferInit(
    CO_CANmodule_t *CANmodule,
    uint16_t index,
    uint16_t ident,
    uint16_t mask,
    bool_t rtr,
    void *object,
    void (*CANrx_callback)(void *object, void *message))
{
    CO_ReturnError_t ret = CO_ERROR_NO;

    if ((CANmodule != NULL) && (object != NULL) && (CANrx_callback != NULL) && (index < CANmodule->rxSize))
    {
        /* buffer, which will be configured */
        CO_CANrx_t *buffer = &CANmodule->rxArray[index];

        /* Configure object variables */
        buffer->object = object;
        buffer->CANrx_callback = CANrx_callback;

        /* CAN identifier and CAN mask, bit aligned with CAN module. Different on different microcontrollers. */
        buffer->ident = ident & 0x07FFU;
        if (rtr)
        {
            buffer->ident |= 0x0800U;
        }
        buffer->mask = (mask & 0x07FFU) | 0x0800U;

        /* Set CAN hardware module filter and mask. */
        if (CANmodule->useCANrxFilters)
        {
        }
    }
    else
    {
        ret = CO_ERROR_ILLEGAL_ARGUMENT;
    }

    return ret;
}

/******************************************************************************/
CO_CANtx_t *CO_CANtxBufferInit(
    CO_CANmodule_t *CANmodule,
    uint16_t index,
    uint16_t ident,
    bool_t rtr,
    uint8_t noOfBytes,
    bool_t syncFlag)
{
    CO_CANtx_t *buffer = NULL;

    if ((CANmodule != NULL) && (index < CANmodule->txSize))
    {
        /* get specific buffer */
        buffer = &CANmodule->txArray[index];
        buffer->ident = (uint32_t)ident & 0x07FFU;
        if (rtr)
        {
            buffer->ident |= 0x0800U;
        }
        buffer->DLC = noOfBytes;
        buffer->bufferFull = false;
        buffer->syncFlag = syncFlag;
    }

    return buffer;
}

/******************************************************************************/
CO_ReturnError_t CO_CANsend(CO_CANmodule_t *CANmodule, CO_CANtx_t *buffer)
{
    CO_ReturnError_t err = CO_ERROR_NO;

    /* Verify overflow */
    if (buffer->bufferFull)
    {
        /* don't set error, if bootup message is still on buffers */
        if (!CANmodule->firstCANtxMessage)
        {
            ESP_LOGE(TAG, "Overflow CANTX CANtxCount: %d send data: id: %#03lx, dlc: %d, data: [%02x %02x %02x %02x %02x %02x %02x %02x]",
                     CANmodule->CANtxCount++,
                     buffer->ident,
                     buffer->DLC,
                     buffer->data[0],
                     buffer->data[1],
                     buffer->data[2],
                     buffer->data[3],
                     buffer->data[4],
                     buffer->data[5],
                     buffer->data[6],
                     buffer->data[7]);
            CANmodule->CANerrorStatus |= CO_CAN_ERRTX_OVERFLOW;
        }
        err = CO_ERROR_TX_OVERFLOW;
    }

#if CONFIG_CO_DEBUG_DRIVER_CAN_SEND
    ESP_LOGI(TAG, "CANTX id: %#03lx, dlc: %d, data: [%02x %02x %02x %02x %02x %02x %02x %02x]",
             buffer->ident,
             buffer->DLC,
             buffer->data[0],
             buffer->data[1],
             buffer->data[2],
             buffer->data[3],
             buffer->data[4],
             buffer->data[5],
             buffer->data[6],
             buffer->data[7]);
#endif

    CO_LOCK_CAN_SEND(CANmodule);

    /* Don't block the caller: if every TX slot is in flight, leave the frame
     * to the Tx task, which waits for a slot. */
    esp_err_t espRet = CO_twaiTransmit(buffer, 0);
    if (ESP_OK != espRet)
    {
        ESP_LOGE(TAG, "Failed Tx. ident:%#2lx err:0x%x", buffer->ident, espRet);

        buffer->bufferFull = true;
        CANmodule->CANtxCount++;
        xTaskNotify(xCoTxTaskHandle, 0, eNoAction);
    }
    else if (buffer->bufferFull)
    {
        // buffer already overwritten anyway
        ESP_LOGW(TAG, "Cleared bufferFull ident:%#2lx", buffer->ident);
        buffer->bufferFull = false;
    }

    CO_UNLOCK_CAN_SEND(CANmodule);

    return err;
}

/******************************************************************************/
void CO_CANclearPendingSyncPDOs(CO_CANmodule_t *CANmodule)
{
    uint32_t tpdoDeleted = 0U;

    CO_LOCK_CAN_SEND(CANmodule);
    /* Abort message from CAN module, if there is synchronous TPDO.
     * Take special care with this functionality. */
    if (/*messageIsOnCanBuffer && */ CANmodule->bufferInhibitFlag)
    {
        /* clear TXREQ */
        CANmodule->bufferInhibitFlag = false;
        tpdoDeleted = 1U;
    }
    /* delete also pending synchronous TPDOs in TX buffers */
    if (CANmodule->CANtxCount != 0U)
    {
        uint16_t i;
        CO_CANtx_t *buffer = &CANmodule->txArray[0];
        for (i = CANmodule->txSize; i > 0U; i--)
        {
            if (buffer->bufferFull)
            {
                if (buffer->syncFlag)
                {
                    buffer->bufferFull = false;
                    CANmodule->CANtxCount--;
                    tpdoDeleted = 2U;
                }
            }
            buffer++;
        }
    }
    CO_UNLOCK_CAN_SEND(CANmodule);

    if (tpdoDeleted != 0U)
    {
        CANmodule->CANerrorStatus |= CO_CAN_ERRTX_PDO_LATE;
    }
}

/******************************************************************************/
void CO_CANgetStats(CO_CANstats_t *stats)
{
    memset(stats, 0, sizeof(*stats));
    if (!bInstalled || twaiNode == NULL)
    {
        return;
    }

    twai_node_status_t status;
    twai_node_record_t record;
    if (twai_node_get_info(twaiNode, &status, &record) == ESP_OK)
    {
        stats->state = status.state;
        stats->tx_error_count = status.tx_error_count;
        stats->rx_error_count = status.rx_error_count;
        stats->tx_queue_remaining = status.tx_queue_remaining;
        stats->bus_err_num = record.bus_err_num;
    }
    stats->installed = true;
    stats->rx_queue_waiting = uxQueueMessagesWaiting(rxQueue);
    stats->rx_queue_overflow = rxQueueOverflowCount;
    stats->tx_failed = txFailedCount;
    stats->bus_off_count = busOffCount;
}

void CO_CANmodule_process(CO_CANmodule_t *CANmodule)
{
    uint32_t err;
    twai_node_status_t statusInfo;
    esp_err_t espRet = ESP_OK;

    if (!bInstalled || twaiNode == NULL)
    {
        return;
    }

    espRet = twai_node_get_info(twaiNode, &statusInfo, NULL);
    if (espRet != ESP_OK)
    {
        ESP_LOGW(TAG, "twai_node_get_info returns %d", espRet);
        return;
    }

    /* The new driver does not recover from bus-off on its own. Start recovery once per
     * bus-off entry; the node reports BUS_OFF until 128 x 11 recessive bits are seen. */
    static bool recovering = false;
    if (statusInfo.state == TWAI_ERROR_BUS_OFF)
    {
        if (!recovering)
        {
            ESP_LOGE(TAG, "Bus-off (tx_err %u), starting recovery", statusInfo.tx_error_count);
            recovering = (twai_node_recover(twaiNode) == ESP_OK);
        }
    }
    else if (recovering)
    {
        ESP_LOGW(TAG, "Recovered from bus-off");
        recovering = false;
    }

    /******************************************************************************/
    /* Get error counters from the module. If necessary, function may use
     * different way to determine errors. */
    uint16_t rxErrors = statusInfo.rx_error_count;
    uint16_t txErrors = statusInfo.tx_error_count;
    bool busOff = statusInfo.state == TWAI_ERROR_BUS_OFF;
    /* Software RX queue drops; the new driver doesn't expose hardware FIFO overruns */
    uint8_t overflow = (uint8_t)rxQueueOverflowCount;

    err = ((uint32_t)busOff << 24) | ((uint32_t)(txErrors & 0xFFU) << 16) | ((uint32_t)(rxErrors & 0xFFU) << 8) | overflow;

    if (CANmodule->errOld != err)
    {
        uint16_t status = CANmodule->CANerrorStatus;

        CANmodule->errOld = err;

        if (busOff)
        {
            /* bus off */
            status |= CO_CAN_ERRTX_BUS_OFF;
        }
        else
        {
            /* recalculate CANerrorStatus, first clear some flags */
            status &= 0xFFFF ^ (CO_CAN_ERRTX_BUS_OFF |
                                CO_CAN_ERRRX_WARNING | CO_CAN_ERRRX_PASSIVE |
                                CO_CAN_ERRTX_WARNING | CO_CAN_ERRTX_PASSIVE);

            /* rx bus warning or passive */
            if (rxErrors >= 128)
            {
                status |= CO_CAN_ERRRX_WARNING | CO_CAN_ERRRX_PASSIVE;
            }
            else if (rxErrors >= 96)
            {
                status |= CO_CAN_ERRRX_WARNING;
            }

            /* tx bus warning or passive */
            if (txErrors >= 128)
            {
                status |= CO_CAN_ERRTX_WARNING | CO_CAN_ERRTX_PASSIVE;
            }
            else if (txErrors >= 96)
            {
                status |= CO_CAN_ERRTX_WARNING;
            }

            /* if not tx passive clear also overflow */
            if ((status & CO_CAN_ERRTX_PASSIVE) == 0)
            {
                status &= 0xFFFF ^ CO_CAN_ERRTX_OVERFLOW;
            }
        }

        static uint8_t overflow_last = 0;
        if (overflow != overflow_last)
        {
            /* CAN RX bus overflow */
            if (overflow != 0)
            {
                ESP_LOGE(TAG, "Set overflow error %d", overflow);
                status |= CO_CAN_ERRRX_OVERFLOW;
            }
            overflow_last = overflow;
        }

        CANmodule->CANerrorStatus = status;
    }
}

/******************************************************************************/
static void CO_txTask(void *pxParam)
{
    uint32_t notificationValue;
    CO_CANtx_t *pCanTx;
    esp_err_t espRet;
    CO_CANmodule_t *CANmodule = (CO_CANmodule_t *)pxParam;
    ESP_LOGI(TAG, "tx task running");

    int err_invalid_state_count = 0;
    while (1)
    {
        xTaskNotifyWait(ULONG_MAX, ULONG_MAX, &notificationValue, portMAX_DELAY);

        CO_LOCK_CAN_SEND(CANmodule);
        /* First CAN message (bootup) was sent successfully */
        CANmodule->firstCANtxMessage = false;
        /* clear flag from previous message */
        CANmodule->bufferInhibitFlag = false;
        /* Are there any new messages waiting to be send */
        while (CANmodule->CANtxCount > 0U)
        {
            /* search through whole array of pointers to transmit message buffers. */
            for (int i = 0; i < CANmodule->txSize; i++)
            {
                pCanTx = &(CANmodule->txArray[i]);
                if (pCanTx->bufferFull)
                {
                    espRet = CO_twaiTransmit(pCanTx, CO_TX_TASK_SLOT_TIMEOUT_MS);
                    if (ESP_OK == espRet)
                    {
                        pCanTx->bufferFull = false;
                        err_invalid_state_count = 0;
                    }
                    else
                    {
                        ESP_LOGE(TAG, "Failed Tx. id:%d err:0x%x", i, espRet);
                        if (espRet == ESP_ERR_INVALID_STATE)
                        {
                            /* node is bus-off, CO_CANmodule_process() starts the recovery */
                            err_invalid_state_count++;

                            ESP_LOGE(TAG, "Err Invalid state count is %d, will restart at 10", err_invalid_state_count);
                            if (err_invalid_state_count == 10)
                                esp_restart();
                        }
                    }
                    CANmodule->CANtxCount--;
                    CANmodule->bufferInhibitFlag = pCanTx->syncFlag;
                    break; /* exit for loop */
                }
            }
        }
        CO_UNLOCK_CAN_SEND(CANmodule);
    }
}

static void CO_rxTask(void *pxParam)
{
    CO_CANrxMsg_t rx_msg;
    CO_CANmodule_t *CANmodule = (CO_CANmodule_t *)pxParam;
    ESP_LOGI(TAG, "rx task running");

    while (1)
    {
        uint16_t index;            /* index of received message */
        uint32_t rcvMsgIdent;      /* identifier of the received message */
        CO_CANrx_t *buffer = NULL; /* receive message buffer from CO_CANmodule_t object. */
        bool_t msgMatched = false;

        if (xQueueReceive(rxQueue, &rx_msg, portMAX_DELAY) != pdTRUE)
        {
            continue;
        }

#if CONFIG_CO_DEBUG_DRIVER_CAN_RECEIVE
        ESP_LOGI(TAG, "CANRX id: 0x%x%s, dlc: %d, data: [%d %d %d %d %d %d %d %d]",
                 rx_msg.ident,
                 rx_msg.rtr ? " RTR" : "",
                 rx_msg.DLC,
                 rx_msg.data[0],
                 rx_msg.data[1],
                 rx_msg.data[2],
                 rx_msg.data[3],
                 rx_msg.data[4],
                 rx_msg.data[5],
                 rx_msg.data[6],
                 rx_msg.data[7]);
#endif /* CONFIG_CO_DEBUG_DRIVER_CAN_RECEIVE */

        /* bit 11 carries RTR, aligned with CO_CANrx_t ident/mask */
        rcvMsgIdent = rx_msg.ident | (rx_msg.rtr ? 0x0800U : 0U);
        /* CAN module filters are not used, message with any standard 11-bit identifier */
        /* has been received. Search rxArray form CANmodule for the same CAN-ID. */
        buffer = &CANmodule->rxArray[0];
        for (index = CANmodule->rxSize; index > 0U; index--)
        {
            if (((rcvMsgIdent ^ buffer->ident) & buffer->mask) == 0U)
            {
                msgMatched = true;
                break;
            }
            buffer++;
        }

        /* Call specific function, which will process the message */
        if (msgMatched && (buffer != NULL) && (buffer->CANrx_callback != NULL))
        {
            buffer->CANrx_callback(buffer->object, (void *)&rx_msg);
        }
    }
}
