#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/logging/log.h>
#include <openlcb.h>

LOG_MODULE_REGISTER(cstation, LOG_LEVEL_INF);

int main(void)
{
    const struct device *can_dev = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(can1));

    if (!can_dev) {
        LOG_ERR("CAN1 device node not found");
        return -ENODEV;
    }

    if (!device_is_ready(can_dev)) {
        LOG_ERR("CAN1 device not ready");
    } else {
        LOG_INF("CAN1 device ready");
    }

    /* Initialize OpenLCB helper and register a simple RX handler */
    if (openlcb_init(can_dev) == 0) {
        /* set the Node ID (already defaulted, but explicit) */
        const uint8_t my_node_id[6] = { 0x05, 0x01, 0x01, 0x01, 0x1A, 0x01 };
        openlcb_set_node_id(my_node_id);

        openlcb_register_rx_handler(openlcb_app_rx_handler, NULL);
        LOG_INF("OpenLCB helper initialized - starting alias negotiation");

        /* start deterministic alias negotiation */
        if (openlcb_can_start_alias_negotiation() != 0) {
            LOG_WRN("Failed to start alias negotiation");
        }
    } else {
        LOG_WRN("OpenLCB helper failed to initialize");
    }

    while (1) {
        LOG_INF("Heartbeat - CAN1 %s alias=0x%03x state=%d", device_is_ready(can_dev) ? "ready" : "not ready", openlcb_can_get_alias(), openlcb_can_get_state());
        k_sleep(K_SECONDS(5));
    }

    return 0;
}
