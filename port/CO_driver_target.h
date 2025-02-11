/*
 * Device and application specific definitions for CANopenNode.
 *
 * @file        CO_driver_target.h
 * @author      Janez Paternoster
 * @copyright   2021 Janez Paternoster
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

#ifndef CO_DRIVER_TARGET_H
#define CO_DRIVER_TARGET_H

/* This file contains device and application specific definitions.
 * It is included from CO_driver.h, which contains documentation
 * for common definitions below. */

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/twai.h"

#ifdef CO_DRIVER_CUSTOM
#include "CO_driver_custom.h"
#endif

#ifdef __cplusplus
extern "C"
{
#endif

/* Stack configuration override default values.
 * For more information see file CO_config.h. */

/* Basic definitions. If big endian, CO_SWAP_xx macros must swap bytes. */
#define CO_LITTLE_ENDIAN
#define CO_SWAP_16(x) x
#define CO_SWAP_32(x) x
#define CO_SWAP_64(x) x
    /* NULL is defined in stddef.h */
    /* true and false are defined in stdbool.h */
    /* int8_t to uint64_t are defined in stdint.h */
    typedef uint_fast8_t bool_t;
    typedef float float32_t;
    typedef double float64_t;

/* Access to received CAN message */
#define CO_CANrxMsg_readIdent(msg) ((uint16_t)(((twai_message_t *)msg)->identifier))
#define CO_CANrxMsg_readDLC(msg) ((uint8_t)(((twai_message_t *)msg)->data_length_code))
#define CO_CANrxMsg_readData(msg) ((uint8_t *)&(((twai_message_t *)msg)->data[0]))

    /* Received message object */
    typedef struct
    {
        uint16_t ident;
        uint16_t mask;
        void *object;
        void (*CANrx_callback)(void *object, void *message);
    } CO_CANrx_t;

    /**
     * Configuration object for CAN transmit message for specific \ref CO_obj
     * "CANopenNode Object".
     *
     * Must be defined in the **CO_driver_target.h** file.
     *
     * Data fields of this structure are used exclusively by the driver. Usually it
     * has the following data fields, but they may differ for different
     * microcontrollers. Array of multiple CO_CANtx_t objects is included inside
     * CO_CANmodule_t.
     */
    typedef struct
    {
        uint32_t ident;             /**< CAN identifier as aligned in CAN module */
        uint8_t DLC;                /**< Length of CAN message */
        uint8_t data[8];            /**< 8 data bytes */
        volatile bool_t bufferFull; /**< True if previous message is still in the
                                         buffer */
        volatile bool_t syncFlag;   /**< Synchronous PDO messages has this flag set.
                      It prevents them to be sent outside the synchronous window */
    } CO_CANtx_t;
    /** @} */

    /* CAN module object */
    typedef struct
    {
        void *CANptr;
        CO_CANrx_t *rxArray;
        uint16_t rxSize;
        CO_CANtx_t *txArray;
        uint16_t txSize;
        uint16_t CANerrorStatus;
        volatile bool_t CANnormal;
        volatile bool_t useCANrxFilters;
        volatile bool_t bufferInhibitFlag;
        volatile bool_t firstCANtxMessage;
        volatile uint16_t CANtxCount;
        uint32_t errOld;
        StaticSemaphore_t xMutexCanSendBuf;
        SemaphoreHandle_t xMutexCanSendHdl;
        StaticSemaphore_t xMutexEmcyBuf;
        SemaphoreHandle_t xMutexEmcyHdl;
        StaticSemaphore_t xMutexODBuf;
        SemaphoreHandle_t xMutexODHdl;
    } CO_CANmodule_t;

    /* Data storage object for one entry */
    typedef struct
    {
        void *addr;
        size_t len;
        uint8_t subIndexOD;
        uint8_t attr;
        /* Additional variables (target specific) */
        void *addrNV;
    } CO_storage_entry_t;

/* (un)lock critical section in CO_CANsend() */
#define CO_LOCK_CAN_SEND(CAN_MODULE) (xSemaphoreTakeRecursive(CAN_MODULE->xMutexCanSendHdl, portMAX_DELAY))
#define CO_UNLOCK_CAN_SEND(CAN_MODULE) (xSemaphoreGiveRecursive(CAN_MODULE->xMutexCanSendHdl))

/* (un)lock critical section in CO_errorReport() or CO_errorReset() */
#define CO_LOCK_EMCY(CAN_MODULE) (xSemaphoreTakeRecursive(CAN_MODULE->xMutexEmcyHdl, portMAX_DELAY))
#define CO_UNLOCK_EMCY(CAN_MODULE) (xSemaphoreGiveRecursive(CAN_MODULE->xMutexEmcyHdl))

/* (un)lock critical section when accessing Object Dictionary */
#define CO_LOCK_OD(CAN_MODULE) (xSemaphoreTakeRecursive(CAN_MODULE->xMutexODHdl, portMAX_DELAY))
#define CO_UNLOCK_OD(CAN_MODULE) (xSemaphoreGiveRecursive(CAN_MODULE->xMutexODHdl))

/* Synchronization between CAN receive and message processing threads. */
#define CO_MemoryBarrier()
#define CO_FLAG_READ(rxNew) ((rxNew) != NULL)
#define CO_FLAG_SET(rxNew)  \
    {                       \
        CO_MemoryBarrier(); \
        rxNew = (void *)1L; \
    }
#define CO_FLAG_CLEAR(rxNew) \
    {                        \
        CO_MemoryBarrier();  \
        rxNew = NULL;        \
    }

#if CONFIG_CO_SDO_CLIENT_ENABLE
#define CO_CONFIG_FIFO CO_CONFIG_FIFO_ENABLE
#if CONFIG_CO_SDO_CLI_SEGMENTED
#define CO_CONFIG_SDO_CLI (CO_CONFIG_SDO_CLI_ENABLE | CO_CONFIG_SDO_CLI_SEGMENTED)
#else
#define CO_CONFIG_SDO_CLI CO_CONFIG_SDO_CLI_ENABLE
#endif

#endif

#if CONFIG_CO_CONFIG_NMT_MASTER
#define CO_CONFIG_NMT CO_CONFIG_NMT_MASTER
#endif

#define CO_CONFIG_TIME_BASE (CO_CONFIG_TIME_ENABLE |              \
                             CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE | \
                             CO_CONFIG_GLOBAL_FLAG_OD_DYNAMIC)

#ifdef CONFIG_CO_TIME_PRODUCER
#define CO_CONFIG_TIME (CO_CONFIG_TIME_BASE | CO_CONFIG_TIME_PRODUCER)
#else
#define CO_CONFIG_TIME CO_CONFIG_TIME_BASE
#endif

/* Stack configuration override default values.
 * For more information see file CO_config.h. */
#if CONFIG_CO_LED_ENABLE
#define CO_CONFIG_LEDS CO_CONFIG_LEDS_ENABLE
#else
#define CO_CONFIG_LEDS 0
#endif

#if CONFIG_CO_DEBUG_SDO
#define CO_CONFIG_DEBUG (CO_CONFIG_DEBUG_SDO_CLIENT | CO_CONFIG_DEBUG_SDO_SERVER)
#define CO_DEBUG_COMMON(msg) ESP_LOGI("CO_SDO", "%s", msg)
#endif /* CONFIG_CO_DEBUG_SDO */

#define CONFIG_CO_PRE 1
#if CONFIG_CO_PRE
#define CO_CONFIG_GLOBAL_RT_FLAG_CALLBACK_PRE CO_CONFIG_FLAG_CALLBACK_PRE
#endif

#if CONFIG_CO_SDO_SERVER_BLOCK_TRANSFER
#define CO_CONFIG_SDO_SRV_BUFFER_SIZE 900
#define CO_CONFIG_CRC16 CO_CONFIG_CRC16_ENABLE
#define CO_CONFIG_SDO_SRV (CO_CONFIG_SDO_SRV_SEGMENTED |        \
                           CO_CONFIG_SDO_SRV_BLOCK |            \
                           CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE | \
                           CO_CONFIG_GLOBAL_FLAG_TIMERNEXT |    \
                           CO_CONFIG_GLOBAL_FLAG_OD_DYNAMIC |   \
                           CO_CONFIG_FLAG_CALLBACK_PRE)
#endif
#if CONFIG_CO_CONFIG_EM_CONSUMER
// Default CO_CONFIG_EM + CO_CONFIG_EM_CONSUMER
#define CO_CONFIG_EM (CO_CONFIG_EM_PRODUCER |              \
                      CO_CONFIG_EM_HISTORY |               \
                      CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE | \
                      CO_CONFIG_GLOBAL_FLAG_TIMERNEXT |    \
                      CO_CONFIG_EM_CONSUMER)
#endif

#if CONFIG_CO_STORAGE
#define CO_CONFIG_STORAGE (CO_CONFIG_STORAGE_ENABLE)
#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* CO_DRIVER_TARGET_H */
