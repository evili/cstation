#ifndef LCC_H
#define LCC_H

#define LCC_NODE_ID_SIZE 6
typedef enum {
    LCC_STATE_UNINITIALIZED = 0,
    LCC_STATE_ATTACHED,
    LCC_STATE_INITIALIZED
} lcc_state_t;

typedef struct {
    void *attach(lcc_node_t *node);
    void *send(const struct lcc_message_t *message);
    void *receive(struct lcc_message_t *message);
} lcc_device_t;

typedef struct {
    uint8_t node_id[6];
    uint16_t state;
    lcc_device_t device;
} lcc_node_t;

typedef struct {
    uint16_t mti;
    uint8_t *data;
    size_t len;
} lcc_message_t;

int lcc_init(const lcc_node_t *node, const uint8_t *node_id);
int lcc_send_message(const lcc_node_t *node, const lcc_message_t *message);
int lcc_register_rx_handler(const lcc_node_t *node, 
    void (*handler)(const lcc_message_t *, void *),
    void *user_data);

#endif /* LCC_H */