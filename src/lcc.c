_#include <lcc.h>
#include <errno.h>

int lcc_init(
    const lcc_node_t *node,
    const uint8_t *node_id,
    void *device,
    void *attach,
    void *send,
    void *receive
) {
    if (!node || !node_id || !device || !attach || !send || !receive) {
        LOG_ERR("lcc_init: invalid arguments");
        return -EINVAL;
    }

    memcpy(node_id, node->node_id, LCC_NODE_ID_SIZE);
    node->device = *device;
    node->attach =   (int (*)(void *, lcc_node_t *)) attach;
    node->send =     (int (*)(void *, const lcc_message_t *)) send;
    node->receive =  (int (*)(void *, lcc_message_t *)) receive;

    node->state = LCC_STATE_ATTACHED;

    int ret = node->attach(device, node);
    if (ret < 0) {
        LOG_ERR("lcc_init: failed to attach device");
        return ret;
    }
    node->state = LCC_STATE_INITIALIZED;
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
