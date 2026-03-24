#ifndef CANOPENNODE_ESP32_H
#define CANOPENNODE_ESP32_H

#include "CANopen.h"
#include "CO_nvs.h"

extern CO_t *CO;

/* All-in-one init (allocate + storage + start task) */
bool CO_ESP32_run(uint8_t node_id, CO_storage_entry_t *entries, uint8_t entry_count);

/* Split init steps (for projects that need to read OD values between storage and task start) */
void CO_ESP32_alloc(uint8_t node_id);
void CO_ESP32_storage_init(CO_storage_entry_t *entries, uint8_t entry_count);
void CO_ESP32_start_task(void);

/* Heartbeat consumer utilities */
void CO_consumer_heartbeat_init(uint8_t node_id, uint16_t timeout_ms);
CO_HBconsNode_t *CO_consumer_find(uint8_t node_id);
const char *hb_state_str(CO_HBconsumer_state_t state);
const char *nmt_state_str(CO_NMT_internalState_t state);

#endif /* CANOPENNODE_ESP32_H */
