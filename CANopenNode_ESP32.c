#include "sdkconfig.h"

#if CONFIG_USE_CANOPENNODE

#include "esp_log.h"
#include "CANopenNode_ESP32.h"
#include "OD.h"
#include "OD_extensions.h"
#include <time.h>

#if (CONFIG_FREERTOS_HZ != 1000)
#error "FreeRTOS tick interrupt frequency must be 1000Hz"
#endif
#define CO_PERIODIC_TASK_INTERVAL_US (CONFIG_CO_PERIODIC_TASK_INTERVAL_MS * 1000)
#define CO_MAIN_TASK_INTERVAL_US (CONFIG_CO_MAIN_TASK_INTERVAL_MS * 1000)

static const char *TAG = "CO_ESP32";

CO_t *CO = NULL;
static void *CANptr = NULL;

static StaticTask_t xCoMainTaskBuffer;
static StackType_t xCoMainStack[CONFIG_CO_MAIN_TASK_STACK_SIZE];
static TaskHandle_t xCoMainTaskHandle = NULL;
static void CO_mainTask(void *pxParam);

static StaticTask_t xCoPeriodicTaskBuffer;
static StackType_t xCoPeriodicStack[CONFIG_CO_PERIODIC_TASK_STACK_SIZE];
static TaskHandle_t xCoPeriodicTaskHandle = NULL;
static void CO_periodicTask(void *pxParam);

static SemaphoreHandle_t xPeriodicTaskSemaphore, xProcessTaskSemaphore;
static uint8_t active_node_id = CONFIG_CO_DEFAULT_NODE_ID;

/* Days between CANopen epoch (1984-01-01) and Unix epoch (1970-01-01) */
#define CO_TIME_EPOCH_OFFSET_DAYS 5113U

static volatile bool co_time_received = false;

static void on_co_time_received(void *arg)
{
    (void)arg;
    co_time_received = true;
}

#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
static CO_storage_t storage;
#endif

/* -------------------------------------------------------------------------- */

void CO_ESP32_alloc(uint8_t node_id)
{
    ESP_LOGI(TAG, "Initializing Node %d", node_id);
    od_extensions_init_extensions();

    active_node_id = node_id;
    uint32_t heapMemoryUsed;
    xPeriodicTaskSemaphore = xSemaphoreCreateBinary();
    xProcessTaskSemaphore = xSemaphoreCreateBinary();

    CO = CO_new(NULL, &heapMemoryUsed);
    if (CO == NULL)
        ESP_LOGW(TAG, "Can't allocate memory");
    else
        ESP_LOGI(TAG, "Allocated %d bytes for CANopen objects", (int)heapMemoryUsed);
}

void CO_ESP32_storage_init(CO_storage_entry_t *entries, uint8_t entry_count)
{
#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
    uint32_t storageInitError = 0;
    CO_ReturnError_t err = CO_storageNvs_init(&storage, CO->CANmodule,
                                              OD_ENTRY_H1010_storeParameters,
                                              OD_ENTRY_H1011_restoreDefaultParameters,
                                              entries, entry_count, &storageInitError);
    if (err != CO_ERROR_NO && err != CO_ERROR_DATA_CORRUPT)
        ESP_LOGE(TAG, "Error: Storage %lu", storageInitError);
#endif
}

__attribute__((weak)) void CO_ESP32_post_canopen_init(void) {}

void CO_ESP32_start_task(void)
{
    xCoMainTaskHandle = xTaskCreateStaticPinnedToCore(
        CO_mainTask, "CO_main",
        CONFIG_CO_MAIN_TASK_STACK_SIZE, (void *)0,
        CONFIG_CO_MAIN_TASK_PRIORITY, &xCoMainStack[0],
        &xCoMainTaskBuffer, CONFIG_CO_TASK_CORE);
}

bool CO_ESP32_run(uint8_t node_id, CO_storage_entry_t *entries, uint8_t entry_count)
{
    CO_ESP32_alloc(node_id);
    CO_ESP32_storage_init(entries, entry_count);
    CO_ESP32_start_task();
    return true;
}

/* -------------------------------------------------------------------------- */

static void SignalPeriodicTask(void *)
{
    xSemaphoreGive(xPeriodicTaskSemaphore);
}

#if (CO_CONFIG_SDO_SRV) & CO_CONFIG_FLAG_CALLBACK_PRE
static void SignalProcessTask(void *)
{
    xSemaphoreGive(xProcessTaskSemaphore);
}
#endif

static void CO_mainTask(void *pxParam)
{
    CO_ReturnError_t err;
    uint32_t errInfo = 0;
    CO_NMT_reset_cmd_t reset = CO_RESET_NOT;
    TickType_t xLastWakeTime;

    ESP_LOGI(TAG, "main task running.");

    while (reset != CO_RESET_APP)
    {
        ESP_LOGI(TAG, "CANopenNode - Reset communication");

        CO->CANmodule->CANnormal = false;
        CO_CANsetConfigurationMode(CANptr);

        err = CO_CANinit(CO, CANptr, CONFIG_CO_DEFAULT_BPS);
        if (err != CO_ERROR_NO)
            ESP_LOGE(TAG, "CAN initialization failed: %d", err);

        err = CO_CANopenInit(CO,
                             NULL, NULL,
                             OD, NULL,
                             CO_NMT_STARTUP_TO_OPERATIONAL,
                             CONFIG_CO_FIRST_HB_TIME,
                             CONFIG_CO_SDO_SERVER_TIMEOUT,
                             CONFIG_CO_SDO_CLIENT_TIMEOUT,
#if CONFIG_CO_SDO_CLIENT_BLOCK_TRANSFER
                             true,
#else
                             false,
#endif
                             active_node_id, &errInfo);
        if ((err != CO_ERROR_NO) && (err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS))
        {
            if (err == CO_ERROR_OD_PARAMETERS)
                ESP_LOGE(TAG, "Object Dictionary entry 0x%lx", errInfo);
            else
                ESP_LOGE(TAG, "CANopen initialization failed: %d", err);
        }
        CO_ESP32_post_canopen_init();
        CO_SYNC_initCallbackPre(CO->SYNC, NULL, SignalPeriodicTask);
#if ((CO_CONFIG_TIME) & CO_CONFIG_FLAG_CALLBACK_PRE) != 0
        CO_TIME_initCallbackPre(CO->TIME, NULL, on_co_time_received);
#endif

        err = CO_CANopenInitPDO(CO, CO->em, OD, active_node_id, &errInfo);
        if (err != CO_ERROR_NO)
        {
            if (err == CO_ERROR_OD_PARAMETERS)
                ESP_LOGE(TAG, "Object Dictionary entry 0x%lx", errInfo);
            else
                ESP_LOGE(TAG, "PDO initialization failed: %d", err);
        }
        CO_RPDO_initCallbackPre(&CO->RPDO[0], NULL, SignalPeriodicTask);

#if (CO_CONFIG_SDO_SRV) & CO_CONFIG_FLAG_CALLBACK_PRE
        CO_SDOserver_initCallbackPre(CO->SDOserver, NULL, SignalProcessTask);
#endif

#if (CONFIG_CO_PERIODIC_TASK_PRIORITY <= CONFIG_CO_MAIN_TASK_PRIORITY)
/* Note: taskCO_timer priority must be higher than taskCO_main priority */
#error "Invalid CANopenNode task priority"
#endif
        if (xCoPeriodicTaskHandle == NULL)
        {
            ESP_LOGI(TAG, "creating periodic task");
            xCoPeriodicTaskHandle = xTaskCreateStaticPinnedToCore(
                CO_periodicTask, "CO_timer",
                CONFIG_CO_PERIODIC_TASK_STACK_SIZE, (void *)0,
                CONFIG_CO_PERIODIC_TASK_PRIORITY, &xCoPeriodicStack[0],
                &xCoPeriodicTaskBuffer, CONFIG_CO_TASK_CORE);
            if (xCoPeriodicTaskHandle == NULL)
                ESP_LOGE(TAG, "Failed to create periodic task");
        }
        ESP_LOGI(TAG, "periodic task created");

#if CO_CONFIG_LEDS
        CO_LEDs_init(CO->LEDs);
#endif
        CO_CANsetNormalMode(CO->CANmodule);
        reset = CO_RESET_NOT;
        ESP_LOGI(TAG, "CANopenNode is running");

        xLastWakeTime = xTaskGetTickCount();
        while (reset == CO_RESET_NOT)
        {
#if (CO_CONFIG_SDO_SRV) & CO_CONFIG_FLAG_CALLBACK_PRE
            xSemaphoreTake(xProcessTaskSemaphore, pdMS_TO_TICKS(CONFIG_CO_MAIN_TASK_INTERVAL_MS));
#else
            vTaskDelayUntil(&xLastWakeTime, CONFIG_CO_MAIN_TASK_INTERVAL_MS);
#endif
            reset = CO_process(CO, false, CO_MAIN_TASK_INTERVAL_US, NULL);
            if (co_time_received)
            {
                co_time_received = false;
                time_t unix_time = ((time_t)CO->TIME->days + CO_TIME_EPOCH_OFFSET_DAYS) * 86400 + CO->TIME->ms / 1000U;
                struct timespec ts = {.tv_sec = unix_time, .tv_nsec = (long)(CO->TIME->ms % 1000U) * 1000000L};
                clock_settime(CLOCK_REALTIME, &ts);
                //  ESP_LOGI(TAG, "System time set from CANopen TIME: days=%u ms=%lu",
                //           CO->TIME->days, (unsigned long)CO->TIME->ms);
            }
#if CO_CONFIG_LEDS
            uint32_t ledState;
#if (CONFIG_CO_LED_RED_GPIO >= 0)
            ledState = CO_LED_RED(CO->LEDs, CO_LED_CANopen);
#if CONFIG_CO_LED_RED_ACTIVE_HIGH
            gpio_set_level(CONFIG_CO_LED_RED_GPIO, ledState);
#else
            gpio_set_level(CONFIG_CO_LED_RED_GPIO, (ledState == 0) ? 1 : 0);
#endif
#endif
#if (CONFIG_CO_LED_GREEN_GPIO >= 0)
            ledState = CO_LED_GREEN(CO->LEDs, CO_LED_CANopen);
#if CONFIG_CO_LED_GREEN_ACTIVE_HIGH
            gpio_set_level(CONFIG_CO_LED_GREEN_GPIO, ledState);
#else
            gpio_set_level(CONFIG_CO_LED_GREEN_GPIO, (ledState == 0) ? 1 : 0);
#endif
#endif
#endif /* CO_CONFIG_LEDS */
        }
    }

    CO_delete(CO);
    ESP_LOGI(TAG, "resetting");
    vTaskDelay(100);
    esp_restart();
    vTaskDelete(NULL);
}

static void CO_periodicTask(void *pxParam)
{
    ESP_LOGI(TAG, "Periodic task running");

    while (1)
    {
        xSemaphoreTake(xPeriodicTaskSemaphore, pdMS_TO_TICKS(1));
        if ((!CO->nodeIdUnconfigured) && (CO->CANmodule->CANnormal))
        {
            bool syncWas = false;
#if (CO_CONFIG_SYNC) & CO_CONFIG_SYNC_ENABLE
            syncWas = CO_process_SYNC(CO, CO_PERIODIC_TASK_INTERVAL_US, NULL);
#endif
#if (CO_CONFIG_PDO) & CO_CONFIG_RPDO_ENABLE
            CO_process_RPDO(CO, syncWas, CO_PERIODIC_TASK_INTERVAL_US, NULL);
#endif
#if (CO_CONFIG_PDO) & CO_CONFIG_TPDO_ENABLE
            CO_process_TPDO(CO, syncWas, CO_PERIODIC_TASK_INTERVAL_US, NULL);
#endif
        }
    }
}

/* -------------------------------------------------------------------------- */

void CO_consumer_heartbeat_init(uint8_t node_id, uint16_t timeout_ms)
{
    uint32_t value = ((uint32_t)node_id << 16) | timeout_ms;
    OD_PERSIST_COMM.x1016_consumerHeartbeatTime[OD_PERSIST_COMM.x1016_consumerHeartbeatTime_sub0++] = value;
}

CO_HBconsNode_t *CO_consumer_find(uint8_t node_id)
{
    for (uint8_t i = 0; i < CO->HBcons->numberOfMonitoredNodes; i++)
    {
        CO_HBconsNode_t *node = &CO->HBcons->monitoredNodes[i];
        if (node->nodeId == node_id)
            return node;
    }
    ESP_LOGE(TAG, "Cannot find hb consumer for node id %d", node_id);
    ESP_ERROR_CHECK(ESP_ERR_INVALID_STATE);
    return NULL;
}

const char *hb_state_str(CO_HBconsumer_state_t state)
{
    switch (state)
    {
    case CO_HBconsumer_UNCONFIGURED:
        return "Unconfigured";
    case CO_HBconsumer_ACTIVE:
        return "Active";
    case CO_HBconsumer_TIMEOUT:
        return "Timeout";
    case CO_HBconsumer_UNKNOWN:
    default:
        return "Unknown";
    }
}

const char *nmt_state_str(CO_NMT_internalState_t state)
{
    switch (state)
    {
    case CO_NMT_INITIALIZING:
        return "Initializing";
    case CO_NMT_PRE_OPERATIONAL:
        return "Pre_operational";
    case CO_NMT_OPERATIONAL:
        return "Operational";
    case CO_NMT_STOPPED:
        return "Stopped";
    case CO_NMT_UNKNOWN:
    default:
        return "Unknown";
    }
}

#endif /* CONFIG_USE_CANOPENNODE */
