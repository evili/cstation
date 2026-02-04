#include <lcc_can.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/can.h>
#include <zephyr/device.h>

int lcc_can_attach(void *can_device, const lcc_node_t *node) {
    int ret;
    if (!can_device || !node) {
        LOG_ERR("lcc_can_attach: invalid arguments");
        return -EINVAL;
    }
    lcc_can_dev_t *lcc_can = (lcc_can_dev_t *) can_device;

    if(!lcc_can->can_dev || !device_is_ready(lcc_can->can_dev)) { {
        LOG_ERR("lcc_can_attach: CAN device not ready");
        return -EINVAL;
    }

    device *can_dev = lcc_can->can_dev;

    if(can_stop(can_dev) == -EIO) {
        LOG_ERR("lcc_can_attach: failed to stop CAN device");
        return -EIO;
    }

    struct can_timing timing;
    ret = can_set_bitrate(can_dev, LCC_CAN_BITRATE);
    if (ret != 0) {
        LOG_ERR("lcc_can_attach: failed to set CAN bitrate");
        return ret;
    }

    can_add_rx_filter(can_dev, lcc_can_rx_control_frame, (void *) node, LCC_CAN_CONTROL_FRAME_FILTER);
    can_add_rx_filter(can_dev, lcc_can_rx_lcc_frame, (void *) node, LCC_CAN_LCC_FRAME_FILTER);

    if(can_start(can_dev) == -EIO) {
        LOG_ERR("lcc_can_attach: failed to start CAN device");
        return -EIO;
    }

    return 0;
}