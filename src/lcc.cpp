#include <zephyr/logging/log.h>
#include <errno.h>

#include <lcc.hxx>


LOG_MODULE_REGISTER(lcc, LOG_LEVEL_INF);

int LCC_Node::Init(lcc_node_id_t *node_id, LCC_Device *device) {
	if (!node_id || !device) {
        LOG_ERR("lcc_init: invalid arguments");
        return -EINVAL;
    }

    memcpy(node_id, this->id, LCC_NODE_ID_LEN);
    this->device = device;

    state = LCC_STATE_ATTACHED;

    int ret = this->device->attach(this);
    if (ret < 0) {
        LOG_ERR("lcc_init: failed to attach device");
        return ret;
    }
    state = LCC_STATE_INITIALIZED;
    return 0;
}

int LCC_Node::Send(lcc_message_t *message) {
    if (!message || !message->data || message->len == 0) {
        LOG_ERR("lcc_send_message: invalid arguments");
        return -EINVAL;
    }
    if (state != LCC_STATE_INITIALIZED) {
        LOG_ERR("lcc_send_message: node not initialized");
        return -EACCES;
    }
    memcpy(message->source, this->id, LCC_NODE_ID_LEN);
    return device->send(message);
}

int LCC_Node::Receive(lcc_message_t *message) {
    if (!message) {
        LOG_ERR("lcc_receive_message: invalid arguments");
        return -EINVAL;
    }
    if (state != LCC_STATE_INITIALIZED) {
        LOG_ERR("lcc_receive_message: node not initialized");
        return -EACCES;
    }
    return device->recv(message);
}
