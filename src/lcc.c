#include <lcc.h>
#include <errno.h>

int lcc_init(const lcc_node_t *node, const uint8_t *node_id, lcc_device_t *device) {
    if (!node || !node_id || !device) {
        LOG_ERR("lcc_init: invalid arguments");
        return -EINVAL;
    }

    memcpy(node_id, node->node_id, LCC_NODE_ID_SIZE);
    node->device = *device;
    int ret = lcc_attach_device(node, device);
    if (ret < 0) {
        LOG_ERR("lcc_init: failed to attach device");
        return ret;
    }
    node->state = LCC_STATE_ATTACHED;
    return 0;
}

int lcc_send_message(const lcc_node_t *node, const lcc_message_t *message) {
    if (!node || !message || !message->data || message->len == 0) {
        LOG_ERR("lcc_send_message: invalid arguments");
        return -EINVAL;
    }
    if (node->state != LCC_STATE_INITIALIZED) {
        LOG_ERR("lcc_send_message: node not initialized");
        return -EACCES;
    }
    return lcc_device_send(node, message);
}

int lcc_device_send(const lcc_node_t *node, const lcc_message_t *message) {
    // Placeholder for device-specific send implementation
    LOG_DEBUG("lcc_device_send: sending message MTI=0x%04x len=%zu", message->mti, message->len);
    return -ENOSYS; // Not implemented
}