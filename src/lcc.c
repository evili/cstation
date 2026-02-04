_#include <lcc.h>
#include <errno.h>

int lcc_init(
    const lcc_node_t *node,
    const uint8_t *node_id,
    void *device,
    int (*attach)  (void *device, lcc_node_t *node);
    int (*send)    (void *device, const struct lcc_message_t *message);
    int (*receive) (void *device, struct lcc_message_t *message);
) {
    if (!node || !node_id || !device || !attach || !send || !receive) {
        LOG_ERR("lcc_init: invalid arguments");
        return -EINVAL;
    }

    memcpy(node_id, node->node_id, LCC_NODE_ID_SIZE);
    node->device = *device;
    node->attach = attach;
    node->send = send;
    node->receive = receive;

    int ret = node->attach(device, node);
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
