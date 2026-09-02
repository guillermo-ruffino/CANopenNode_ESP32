#include "can_ota.h"
#include "esp_log.h"

#include "esp_ota_ops.h"
#include "esp_timer.h"
#include "esp_task.h"
#include "CANopenNode_ESP32.h"

static const char *TAG = "can_ota";

/* esp_ota_begin()'s full-partition erase has been observed to take ~3.5s,
 * well past CONFIG_CO_SDO_SERVER_TIMEOUT (2000ms default). */
#define CAN_OTA_ERASE_SDO_TIMEOUT_MS 8000U

/* *
 * For more information see file CO_ODinterface.h, OD_IO_t.
 */

static esp_partition_t *update_partition;
static esp_ota_handle_t ota_handle;

/* Consuming projects that need to quiesce something (e.g. WiFi) before the
 * OTA partition erase can override this weak hook in their own main/. */
__attribute__((weak)) void can_ota_pre_erase_hook(void)
{
}

static void finish_task()
{
    ESP_ERROR_CHECK(esp_ota_end(ota_handle));
    esp_err_t err = esp_ota_set_boot_partition(update_partition);
    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "Set boot partition OK");
    }
    else
    {
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        ESP_LOGE(TAG, "Invalid boot partition. Error: %s ", esp_err_to_name(err));
    }

    const esp_partition_t *boot_partition = esp_ota_get_boot_partition();
    ESP_LOGI(TAG, "Next boot partition subtype %d at offset 0x%lx", boot_partition->subtype, boot_partition->address);

    ESP_LOGI(TAG, "Restarting ESP in 1000msec");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    esp_restart();
}

static OD_size_t data_index;
static ODR_t ota_write(OD_stream_t *stream, const void *buf,
                       OD_size_t count, OD_size_t *countWritten)
{
    if (stream == NULL || buf == NULL || countWritten == NULL)
    {
        return ODR_DEV_INCOMPAT;
    }

    ESP_LOGI(TAG, "Chunk size: %lu, offset: %lu length: %lu ", count, stream->dataOffset, stream->dataLength);

    /* Data will be just verified for correct sequence in this example
     * (repeating sequence of values 0..255). Data can be much longer than
     * count (current size of data in the buffer). So OD_write_domainDemo() may
     * be called multiple times. */
    if (stream->dataOffset == 0)
    {

        /* Data offset is 0, so this is the first call of this function
         * in current SDO communication. Initialize variables. */

        // dataSimulated = 0;
        data_index = 0;

        ESP_LOGI(TAG, "Writing to partition subtype %d at offset 0x%lx", update_partition->subtype,
                 update_partition->address);

        int64_t now = esp_timer_get_time();

        /* esp_ota_begin() erases the whole OTA partition, which can block this
         * task (and CO_process()) for several seconds -- well past the SDO
         * server's normal CONFIG_CO_SDO_SERVER_TIMEOUT. Stretch the timeout for
         * this server channel before erasing, and leave it stretched for the
         * rest of the transfer -- it gets set again (idempotently) on every
         * retry, and reverts to the sdkconfig default on the next comm reset,
         * so there's no need to restore it manually. */
        CO_SDOserver_t *sdoServer = &CO->SDOserver[0];
        sdoServer->SDOtimeoutTime_us = CAN_OTA_ERASE_SDO_TIMEOUT_MS * 1000U;
        sdoServer->block_SDOtimeoutTime_us = CAN_OTA_ERASE_SDO_TIMEOUT_MS * 700U;

        can_ota_pre_erase_hook();

        ESP_ERROR_CHECK(esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle));
        ESP_LOGI(TAG, "begin took: %lldus", esp_timer_get_time() - now);
    }

    esp_ota_write(ota_handle, buf, count);
    *countWritten = count;
    data_index += count;
    stream->dataOffset = data_index;

    /* determine, if file write finished or not (dataLength may not yet be
     * indicated) */
    if (stream->dataLength > 0 && stream->dataOffset >= stream->dataLength)
    {
        stream->dataOffset = 0;

        ESP_LOGI(TAG, "Binary transferred finished: %lu bytes", stream->dataLength);

        xTaskCreate(&finish_task, "ota_end", 4096, NULL, 10, NULL);

        return ODR_OK;
    }

    /* indicate partial write, this function will be called again. */
    return ODR_PARTIAL;
}

static OD_extension_t OD_ENTRY_H1F50_extension = {
    .object = NULL,
    .read = OD_readOriginal,
    .write = ota_write};
void can_ota_init()
{
    ESP_LOGI(TAG, "Can OTA setup");
    OD_extension_init(OD_ENTRY_H1F50_firmware, &OD_ENTRY_H1F50_extension);
    update_partition = esp_ota_get_next_update_partition(NULL);
}
