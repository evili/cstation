#ifndef LCC_HXX
#define LCC_HXX

#include <cstddef>
#include <cstdint>

#define LCC_NODE_ID_LEN 6


typedef uint8_t lcc_node_id_t[LCC_NODE_ID_LEN];

typedef enum {
    LCC_STATE_UNINITIALIZED = 0,
    LCC_STATE_ATTACHED,
    LCC_STATE_INITIALIZED
} lcc_state_t;

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
    };
    uint16_t mti;
} lcc_mti_t;

typedef struct {
    uint16_t mti;
    lcc_node_id_t source;
    size_t len;
    uint8_t *data;
} lcc_message_t;

const lcc_message_t LCC_MESSAGE_VERIFY_NODE_ADDRESSED = {
    .mti = 0x0488,
    .source = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    .len = LCC_NODE_ID_LEN,
    .data = nullptr
};

const lcc_message_t LCC_MESSAGE_VERIFY_NODE_GLOBAL = {
    .mti = 0x0490,
    .source = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    .len = LCC_NODE_ID_LEN,
    .data = nullptr
};



class LCC_Node;

class LCC_Device {
public:
  virtual int attach(LCC_Node *) = 0;
  virtual int send(lcc_message_t *) = 0;
  virtual int recv(lcc_message_t *) = 0;
};

class LCC_Node {
public:
  LCC_Node();
  int Init(lcc_node_id_t *, LCC_Device *);
  int Send(lcc_message_t *);
  int Receive(lcc_message_t *);
  lcc_node_id_t *get_id() { return &id; };
private:
  lcc_node_id_t id;
  lcc_state_t state = LCC_STATE_UNINITIALIZED;
  LCC_Device *device;
};


#endif /* LCC_HXX */
