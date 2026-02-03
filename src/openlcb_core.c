#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <string.h>
#include "../include/openlcb_core.h"

LOG_MODULE_REGISTER(openlcb_core, LOG_LEVEL_DBG);

static uint8_t g_node_id[6] = { 0x05, 0x01, 0x01, 0x01, 0x1A, 0x01 };
static openlcb_message_cb_t g_msg_cb;
static void *g_msg_ud;

int openlcb_core_set_node_id(const uint8_t node_id[6])
{
    if (!node_id) return -EINVAL;
    memcpy(g_node_id, node_id, sizeof(g_node_id));
    LOG_INF("openlcb_core: Node ID set to %02x:%02x:%02x:%02x:%02x:%02x",
            g_node_id[0], g_node_id[1], g_node_id[2], g_node_id[3], g_node_id[4], g_node_id[5]);
    return 0;
}

const uint8_t *openlcb_core_get_node_id(void)
{
    return g_node_id;
}

void openlcb_core_register_message_handler(openlcb_message_cb_t cb, void *user_data)
{
    g_msg_cb = cb;
    g_msg_ud = user_data;
}

/* Called by transport when an OpenLCB message is received (worker context) */
void openlcb_core_incoming_message(const uint8_t *data, size_t len)
{
    if (g_msg_cb) {
        g_msg_cb(data, len, g_msg_ud);
    } else {
        LOG_DBG("openlcb_core: no handler for incoming message (len=%zu)", len);
    }
}
