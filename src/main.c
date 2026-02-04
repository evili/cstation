#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/logging/log.h>
#include <lcc.h>
#include <lcc_can.h>

LOG_MODULE_REGISTER(cstation, LOG_LEVEL_INF);

#define CONFIG_LCC_NODE_ID (0x050101011A01uLL)

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
    lcc_node_t node = {.id = CONFIG_LCC_NODE_ID, .state = LCC_STATE_UNINITIALIZED, .device = NULL};

    /* Define LCC_CAN */
    lcc_can_dev_t lcc_can_dev = {.can_dev = can_dev, .state = LCC_CAN_STATE_INHIBITED, .lcc_node = &node};

    /* Define LCC Device */
    lcc_device_t lcc_can_device = {
        .device_obj = &lcc_can_dev,
        .attach = lcc_can_attach,
        .send = lcc_can_send_message,
        .receive = lcc_can_receive_message
    };

    int ret = lcc_init(&node, (const uint8_t *)&CONFIG_LCC_NODE_ID);
    if (ret != 0) {
        LOG_ERR("Failed to initialize LCC node");
        return -EIO;
    }

    while(1) {
        ret = lcc_send_message(&node, LCC_MESSAGE_VERIFY_NODE_GLOBAL);
        if (ret != 0) {
            LOG_ERR("Failed to send LCC message");
        }
        k_sleep(K_MSEC(1000));
    }

    return 0;
}
