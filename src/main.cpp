#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/logging/log.h>
#include <lcc.hxx>
#include <lcc_can.hxx>

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
    LCC_Node node;
    LCC_Dev lcc_can(can_dev);

    ret = node.init(CONFIG_LCC_NODE_ID, lcc_can);
    if (ret != 0) {
        LOG_ERR("Failed to initialize LCC node");
        return -EIO;
    }

    while(1) {
        ret = node.send(&node, &LCC_MESSAGE_VERIFY_NODE_GLOBAL);
        if (ret != 0) {
            LOG_ERR("Failed to send LCC message");
        }
        k_sleep(K_MSEC(1000));
    }

    return 0;
}
