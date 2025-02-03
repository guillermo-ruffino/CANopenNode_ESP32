#include "storage/CO_storage.h"

#if ((CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE) || defined CO_DOXYGEN

CO_ReturnError_t CO_storageNvs_init(CO_storage_t *storage,
                                    CO_CANmodule_t *CANmodule,
                                    OD_entry_t *OD_1010_StoreParameters,
                                    OD_entry_t *OD_1011_RestoreDefaultParam,
                                    CO_storage_entry_t *entries,
                                    uint8_t entriesCount,
                                    uint32_t *storageInitError);

#endif