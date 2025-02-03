#include "CO_nvs.h"
#include "nvs.h"

#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE

static const char *CANOPEN_NS = "canopen";

/*
 * Function for writing data on "Store parameters" command - OD object 1010
 *
 * For more information see file CO_storage.h, CO_storage_entry_t.
 */
static ODR_t
storeNvs(CO_storage_entry_t *entry, CO_CANmodule_t *CANmodule)
{
    (void)CANmodule;
    ODR_t ret = ODR_OK;

    nvs_handle_t nvs_h;
    ESP_ERROR_CHECK(nvs_open(CANOPEN_NS, NVS_READWRITE, &nvs_h));
    char key[10];
    sprintf(key, "s%u", entry->subIndexOD);
    ESP_ERROR_CHECK(nvs_set_blob(nvs_h, key, entry->addr, entry->len));
    ESP_ERROR_CHECK(nvs_commit(nvs_h));
    nvs_close(nvs_h);

    return ret;
}

/*
 * Function for restoring data on "Restore default parameters" command - OD 1011
 *
 * For more information see file CO_storage.h, CO_storage_entry_t.
 */
static ODR_t
restoreNvs(CO_storage_entry_t *entry, CO_CANmodule_t *CANmodule)
{
    (void)CANmodule;
    ODR_t ret = ODR_OK;

    nvs_handle_t nvs_h;
    ESP_ERROR_CHECK(nvs_open(CANOPEN_NS, NVS_READWRITE, &nvs_h));
    char key[10];
    sprintf(key, "s%u", entry->subIndexOD);
    esp_err_t nvs_ret = nvs_erase_key(nvs_h, key);
    if (nvs_ret != ESP_ERR_NVS_NOT_FOUND)
        ESP_ERROR_CHECK(nvs_commit(nvs_h));

    nvs_close(nvs_h);

    return ret;
}

CO_ReturnError_t CO_storageNvs_init(CO_storage_t *storage,
                                    CO_CANmodule_t *CANmodule,
                                    OD_entry_t *OD_1010_StoreParameters,
                                    OD_entry_t *OD_1011_RestoreDefaultParam,
                                    CO_storage_entry_t *entries,
                                    uint8_t entriesCount,
                                    uint32_t *storageInitError)
{
    CO_ReturnError_t ret = ODR_OK;

    /* verify arguments */
    if (storage == NULL || entries == NULL || entriesCount == 0 || storageInitError == NULL)
    {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    storage->enabled = false;

    /* initialize storage and OD extensions */
    ret = CO_storage_init(storage, CANmodule, OD_1010_StoreParameters, OD_1011_RestoreDefaultParam, storeNvs,
                          restoreNvs, entries, entriesCount);
    if (ret != CO_ERROR_NO)
    {
        return ret;
    }

    nvs_handle_t nvs_h;
    esp_err_t nvs_ret;
    nvs_ret = nvs_open(CANOPEN_NS, NVS_READONLY, &nvs_h);
    if (nvs_ret == ESP_ERR_NVS_NOT_FOUND)
    {
        // if the namespace does not exist and NVS_READONLY is set, this error is given
        // use default values.
        storage->enabled = true;
        return ODR_OK;
    }
    ESP_ERROR_CHECK(nvs_ret);

    /* initialize entries */
    *storageInitError = 0;
    for (uint8_t i = 0; i < entriesCount; i++)
    {
        CO_storage_entry_t *entry = &entries[i];
        bool_t dataCorrupt = false;

        /* verify arguments */
        if (entry->addr == NULL || entry->len == 0 || entry->subIndexOD < 2)
        {
            *storageInitError = i;
            return CO_ERROR_ILLEGAL_ARGUMENT;
        }

        char key[10];
        sprintf(key, "s%u", entry->subIndexOD);

        size_t length;
        nvs_ret = nvs_get_blob(nvs_h, key, NULL, &length);
        if (nvs_ret == ESP_OK && length == entry->len)
        {
            nvs_get_blob(nvs_h, key, entry->addr, &entry->len);
        }
        else
        {
            /* additional info in case of error */

            uint32_t errorBit = entry->subIndexOD;
            if (errorBit > 31)
            {
                errorBit = 31;
            }
            *storageInitError |= ((uint32_t)1) << errorBit;
        }
    } /* for (entries) */

    storage->enabled = true;

    nvs_close(nvs_h);
    return ret;
}

#endif