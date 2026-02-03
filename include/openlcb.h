/**
 * Minimal OpenLCB (LCC) helper over Zephyr CAN
 *
 * This module provides a thin transport abstraction to send/receive CAN frames
 * that are intended to carry OpenLCB payloads.
 *
 * NOTE: This is a minimal, pragmatic helper for testing and integration. It
 * does NOT implement the full OpenLCB specification. Adapt identifiers,
 * extended/standard ID usage and payload formats to match the OpenLCB/CAN
 * mapping required by your network.
 */

#ifndef OPENLCB_H
#define OPENLCB_H

/* Compatibility facade: includes OpenLCB core and CAN transport headers
 * Prefer using openlcb_core.h and openlcb_can.h directly.
 */
#include "openlcb_core.h"
#include "openlcb_can.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Backwards-compatible wrappers */
static inline int openlcb_init(const struct device *can_dev)
{
    return openlcb_can_init_transport(can_dev);
}

static inline void openlcb_register_rx_handler(openlcb_message_cb_t cb, void *user_data)
{
    openlcb_core_register_message_handler(cb, user_data);
}

static inline int openlcb_set_node_id(const uint8_t node_id[6])
{
    openlcb_core_set_node_id(node_id);
    return openlcb_can_set_node_id(node_id);
}

static inline int openlcb_send_hello(void)
{
    /* Start alias negotiation (hello publishes Node ID via AMD when alias reserved) */
    return openlcb_can_start_alias_negotiation();
}

#ifdef __cplusplus
}
#endif

#endif /* OPENLCB_H */
