#include <lcc_can.hxx>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/can.h>
#include <zephyr/sys/hash_map.h>
#include <zephyr/device.h>
#include <errno.h>

LOG_MODULE_REGISTER(lcc_can, LOG_LEVEL_INF);

SYS_HASHMAP_DEFAULT_DEFINE(LCC_CAN_Alias_Map);

LCC_Can::LCC_Can(const device *dev) {
  if(!dev) {
    LOG_ERR("lcc_can_attach: invalid arguments");
  }
  can_dev = dev;
}

int LCC_Can::attach(LCC_Node *node) {
    int ret;
    if (!node) {
        LOG_ERR("lcc_can_attach: invalid arguments");
        return -EINVAL;
    }
    
    if(!can_dev || !device_is_ready(can_dev)) {
        LOG_ERR("lcc_can_attach: CAN device not ready");
        return -EINVAL;
    }
    state = LCC_CAN_STATE_INHIBITED;

    this->node = node;

    if(can_stop(can_dev) == -EIO) {
        LOG_ERR("lcc_can_attach: failed to stop CAN device");
        return -EIO;
    }

    ret = can_set_bitrate(can_dev, LCC_CAN_BIT_RATE);
    if (ret != 0) {
        LOG_ERR("lcc_can_attach: failed to set CAN bitrate");
        return ret;
    }

    // can_add_rx_filter(can_dev, lcc_can_rx_control_frame, (void *) node, LCC_CAN_CONTROL_FRAME_FILTER);
    // can_add_rx_filter(can_dev, lcc_can_rx_lcc_frame, (void *) node, LCC_CAN_LCC_FRAME_FILTER);

    if(can_start(can_dev) == -EIO) {
        LOG_ERR("lcc_can_attach: failed to start CAN device");
        return -EIO;
    }
    return lcc_enter_permitted_state();
}

int LCC_Can::lcc_enter_permitted_state() {
    int ret;
    if (!node || !can_dev) {
        LOG_ERR("lcc_can_enter_permitted_state: invalid argument");
        return -EINVAL;
    }
    if (state >= LCC_CAN_STATE_PERMITTED) {
        LOG_WRN("lcc_can_enter_permitted_state: already in PERMITTED state");
        return 0;
    }

    while (state != LCC_CAN_STATE_PERMITTED) {
        lcc_can_get_next_alias();
        ret = lcc_send_check_id();
        if(ret < 0) {
            LOG_ERR("lcc_can_enter_permitted_state: failed to send CHECK_ID message");
            state = LCC_CAN_STATE_INHIBITED;
            continue;
        }
        k_sleep(K_MSEC(LCC_CAN_CHECK_ID_WAIT));
        ret = lcc_send_reserve_id();
        if(ret < 0) {
            LOG_ERR("lcc_can_enter_permitted_state: failed to send RESERVE_ID message");
            state = LCC_CAN_STATE_INHIBITED;
            continue;
        }  
    }
    state = LCC_CAN_STATE_PERMITTED;

    uint64_t *node_id = (uint64_t *) node->get_id;

    sys_hashmap_insert(alias_map, alias.alias16, node_id, NULL);
    LOG_INF("Node [0x%064x] entered PERMITTED state with alias %d", node_id, alias);
    return 0;
}

int LCC_Can::lcc_can_rx_control_frame(struct can_frame *frame) {
    if (!frame) {
        LOG_ERR("lcc_can_rx_control_frame: invalid arguments");
        return -EINVAL;
    }
    lcc_can_frame_t *lcc_frame = (lcc_can_frame_t *) frame;
    LOG_DBG("Received control frame from [0x%012x]: type=0x%01x, content=0x%015x, dlc=%d",
         lcc_frame->source_id,
         lcc_frame->lcc_frame,
         lcc_frame->content,
         lcc_frame->dlc);

    LOG_ERR("LCC Receiving Control Frame not implemented yet");
    return -ENOTSUP;

}

int LCC_Can::lcc_can_rx_lcc_frame(struct can_frame *frame) {
    if (!frame) {
        LOG_ERR("lcc_can_rx_lcc_frame: invalid arguments");
        return -EINVAL;
    }
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
    LOG_ERR("LCC Receiving LCC Frame not implemented yet");
    free(lcc_message);
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
