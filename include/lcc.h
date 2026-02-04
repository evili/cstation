#ifndef LCC_H
#define LCC_H

#define LCC_NODE_ID_LEN 6

typedef uint8_t lcc_node_id_t[LCC_NODE_ID_LEN];

typedef enum {
    LCC_STATE_UNINITIALIZED = 0,
    LCC_STATE_ATTACHED,
    LCC_STATE_INITIALIZED
} lcc_state_t;

typedef struct {
    lcc_node_id_t node_id;
    lcc_state_t state;
    void *device;
    int (*attach)  (void *device, lcc_node_t *node);
    int (*send)    (void *device, const struct lcc_message_t *message);
    int (*receive) (void *device, struct lcc_message_t *message);
} lcc_node_t;

typedef union {
    struct {
        uint8_t reserved  : 2;
        uint8_t special   : 1;
        uint8_t stream    : 1;
        uint8_t priority  : 2;
        uint8_t type      : 5;
        uint8_t simple    : 1;
        uint8_t addressed : 1;
        uint8_t event     : 1;
        uint8_t modifier  : 2;
    }
    uint16_t mti;
} lcc_mti_t;

typedef struct {
    uint16_t mti;
    uint8_t *data;
    size_t len;
} lcc_message_t;

const lcc_message_t LCC_MESSAGE_VERIFY_NODE_GLOBAL = {
    .mti = 0x0490,
    .data = NULL,
    .len = 0
};

const lcc_message_t LCC_MESSAGE_VERIFY_NODE_ADDRESSED = {
    .mti = 0x0488,
    .data = NULL,
    .len = LCC_NODE_ID_LEN
};

int lcc_node_init(
    lcc_node_t *node,
    const lcc_node_id_t node_id,,
    void *device,
    int (*attach)  (void *, lcc_node_t *),
    int (*send)    (void *, const struct lcc_message_t *),
    int (*receive) (void *, struct lcc_message_t *),
);
int lcc_send_message(const lcc_node_t *node, const lcc_message_t *message);
int lcc_receive_message(const lcc_node_t *node, lcc_message_t *message);

#endif /* LCC_H */