#include <lcc_can.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/can.h>
#include <zephyr/sys/hash_map.h>
#include <zephyr/device.h>

SYS_HASH_MAP_DEFINE_STATIC(can_alias_map);


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
    lcc_can->state = LCC_CAN_STATE_INHIBITED;
    device *can_dev = lcc_can->can_dev;

    lcc_can->node = node;

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
    ret = lcc_can_enter_permitted_state(lcc_can);
    return ESUCCESS;
}

int lcc_can_enter_permitted_state(lcc_can_dev_t *lcc_can) {
    int ret;
    if (!lcc_can || !lcc_can->node || !lcc_can->can_dev) {
        LOG_ERR("lcc_can_enter_permitted_state: invalid argument");
        return -EINVAL;
    }
    if (lcc_can->state >= LCC_CAN_STATE_PERMITTED) {
        LOG_WRN("lcc_can_enter_permitted_state: already in PERMITTED state");
        return ESUCCESS;
    }

    while (lcc_can->state != LCC_CAN_STATE_PERMITTED) {
        lcc_can_get_next_alias(lcc_can);
        ret = lcc_send_check_id(lcc_can);
        if(ret < 0) {
            LOG_ERR("lcc_can_enter_permitted_state: failed to send CHECK_ID message");
            lcc_can->state = LCC_CAN_STATE_INHIBITED;
            continue;
        }
        k_sleep(K_MSEC(LCC_CAN_CHECK_ID_WAIT));
        ret = lcc_send_reserve_id(lcc_can);
        if(ret < 0) {
            LOG_ERR("lcc_can_enter_permitted_state: failed to send RESERVE_ID message");
            lcc_can->state = LCC_CAN_STATE_INHIBITED;
            continue;
        }  
    }
    lcc_can->state = LCC_CAN_STATE_PERMITTED;
    uint64_t node_id = (uint64_t *) lcc_can->node->node_id;
    sys_hashmap_insert(lcc_can->alias_map, lcc_can->alias, node_id, NULL);
    LOG_INF("Node [0x%064x] entered PERMITTED state with alias %d", node_id, lcc_can->alias);
    return ESUCCESS;
}

void lcc_can_rx_control_frame(const struct device *dev, struct can_frame *frame, void *user_data) {
    if (!dev || !frame || !user_data) {
        LOG_ERR("lcc_can_rx_control_frame: invalid arguments");
        return;
    }
    lcc_node_t *node = (lcc_node_t *) user_data;
    lcc_can_frame_t *lcc_frame = (lcc_can_frame_t *) frame;
    LOG_DBG("Received control frame from [0x%012x]: type=0x%01x, content=0x%015x, dlc=%d",
         lcc_frame->source_id,
         lcc_frame->lcc_frame,
         lcc_frame->content,
         lcc_frame->dlc);
}

int lcc_can_rx_lcc_frame(const struct device *dev, struct can_frame *frame, void *user_data) {
    if (!dev || !frame || !user_data) {
        LOG_ERR("lcc_can_rx_lcc_frame: invalid arguments");
        return -EINVAL;
    }
    lcc_node_t *node = (lcc_node_t *) user_data;
    lcc_can_frame_t *lcc_frame = (lcc_can_frame_t *) frame;
    if (lcc_frame->lcc_frame != 1) {
        LOG_WRN("Received non-LCC frame in LCC filter");
        return -EINVAL;
    }
    LOG_DBG("Received LCC frame from [0x%012x]: type=0x%01x, content=0x%015x, dlc=%d",
         lcc_frame->source_id,
         lcc_frame->lcc_frame,
         lcc_frame->content,
         lcc_frame->dlc
    );
    // TODO: implement receiving LCC as part of many CAN Frames.
    LOG_ERR("LCC Receiving not implemented yet");
    free(lcc_message);
    free(data);
    return -ENOTSUP;
/*
    lcc_message_t *lcc_message = malloc(sizeof(lcc_message_t));
    uint8_t *data = malloc(lcc_frame->dlc);

    if (!lcc_message || !data) {
        LOG_ERR("Failed to allocate memory for LCC message");
        free(lcc_message);
        free(data);
        return -ENOMEM;
    }
*/
}   