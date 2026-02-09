#include <lcc.hxx>
#include <errno.h>

int LCC::Init(const uint8_t *node_id, LCC_Device *device) { 
	if (!node_id || !device) {
        LOG_ERR("lcc_init: invalid arguments");
        return -EINVAL;
    }

    memcpy(node_id, this->node_id, LCC_NODE_ID_SIZE);
    this->device = *device;

    state = LCC_STATE_ATTACHED;

    int ret = this->device->attach(this);
    if (ret < 0) {
        LOG_ERR("lcc_init: failed to attach device");
        return ret;
    }
    state = LCC_STATE_INITIALIZED;
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
    return node->send(node->device , message);
}

int lcc_receive_message(lcc_node_t *node, lcc_message_t *message) {
    if (!node || !message) {
        LOG_ERR("lcc_receive_message: invalid arguments");
        return -EINVAL;
    }
    if (node->state != LCC_STATE_INITIALIZED) {
        LOG_ERR("lcc_receive_message: node not initialized");
        return -EACCES;
    }
    return node->receive(node->device , message);
}
