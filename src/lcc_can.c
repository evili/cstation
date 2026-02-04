#include <lcc_can.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/can.h>


int lcc_can_attach(void *can_device, const lcc_node_t *node) {
    if (!can_device || !node) {
        LOG_ERR("lcc_can_attach: invalid arguments");
        return -EINVAL;
    }
    lcc_can_dev_t *lcc_can = (lcc_can_dev_t *) can_device;

    if(!lcc_can->can_dev) {
        LOG_ERR("lcc_can_attach: CAN device not ready");
        return -EINVAL;
    }

    lcc_can->can_dev 



    return 0;
}