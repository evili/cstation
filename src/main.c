#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/logging/log.h>
#include <lcc.h>
#include <lcc_can.h>

LOG_MODULE_REGISTER(cstation, LOG_LEVEL_INF);

const lcc_node_id_t CONFIG_LCC_NODE_ID = {0x05, 0x01, 0x01, 0x01, 0x1A, 0x01};
const struct device *can_dev = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(can1));

int main(void) {
    int ret;

    if (!can_dev) {
        LOG_ERR("CAN1 device node not found");
        return -ENODEV;
    }

    if (!device_is_ready(can_dev)) {
        LOG_ERR("CAN1 device not ready");
    } else {
        LOG_INF("CAN1 device ready");
    }

    /* Define LCC node */
    lcc_node_t node;

    ret = lcc_init(&node,
        CONFIG_LCC_NODE_ID,
        (void *) can_dev,
        lcc_can_attach,
        lcc_can_send_message,
        lcc_can_receive_message
    );
    if (ret != 0) {
        LOG_ERR("Failed to initialize LCC node");
        return -EIO;
    }

    while(1) {
        ret = lcc_send_message(&node, &LCC_MESSAGE_VERIFY_NODE_GLOBAL);
        if (ret != 0) {
            LOG_ERR("Failed to send LCC message");
        }
        k_sleep(K_MSEC(1000));
    }

    return 0;
}