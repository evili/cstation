/* Minimal OpenLCB core layer (transport-agnostic)
 * - Represents Node ID
 * - Provides simple message send/receive API for upper layers
 * - Delegates transport to an OpenLCB-CAN transport implementation
 */
#ifndef OPENLCB_CORE_H
#define OPENLCB_CORE_H

#include <zephyr/kernel.h>
#include <zephyr/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 6-byte Node ID */
int openlcb_core_set_node_id(const uint8_t node_id[6]);
const uint8_t *openlcb_core_get_node_id(void);

/* Message callback signature (transport-agnostic). Called from worker context. */
typedef void (*openlcb_message_cb_t)(const uint8_t *data, size_t len, void *user_data);
void openlcb_core_register_message_handler(openlcb_message_cb_t cb, void *user_data);

/* Called by transport when a complete OpenLCB message is received (worker context) */
void openlcb_core_incoming_message(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* OPENLCB_CORE_H */
