#ifndef   LCC_CAN_HXX
#define   LCC_CAN_HXX

#include <zephyr/drivers/can.h>
#include "lcc.hxx"

// Timing Constants (in milliseconds)
#define LCC_CAN_CHECK_ID_WAIT 200
// CAN Bit Rate
#define LCC_CAN_BIT_RATE 125000
// Filter Masks
#define LCC_CAN_FILTER_CONTROL_FRAME_MASK 0x08000000
#define LCC_CAN_FILTER_CONTENT_FIELD_MASK 0x07FFF000
#define LCC_CAN_FILTER_SOURCE_ID_MASK     0x00000FFF

#define LCC_CAN_CONTROL_FRAME 0x00000000
#define LCC_CAN_LCC_FRAME     0x08000000


typedef union {
    can_frame frame;
    struct {
        uint32_t padding  :  3;
        uint32_t reserved  : 1;
        uint32_t lcc_frame : 1;
        uint32_t content   :15;
        uint32_t source_id :12;
        uint8_t  dlc;
        uint16_t timestamp;
        uint8_t  data[CAN_MAX_DLEN];
    };
} lcc_can_frame_t;

const can_filter LCC_CAN_CONTROL_FRAME_FILTER = {
    .id    = LCC_CAN_CONTROL_FRAME,
    .mask  = LCC_CAN_FILTER_CONTROL_FRAME_MASK,
    .flags = CAN_FILTER_IDE
};

const can_filter LCC_CAN_LCC_FRAME_FILTER = {
    .id    = LCC_CAN_LCC_FRAME,
    .mask  = LCC_CAN_FILTER_CONTROL_FRAME_MASK,
    .flags = CAN_FILTER_IDE
};

typedef enum {
    LCC_CAN_STATE_INHIBITED = 0,
    LCC_CAN_STATE_CHECK_ID1,
    LCC_CAN_STATE_CHECK_ID2,
    LCC_CAN_STATE_CHECK_ID3,
    LCC_CAN_STATE_CHECK_ID4,
    LCC_CAN_STATE_WAIT_ID,
    LCC_CAN_STATE_RESERVE_ID,
    LCC_CAN_STATE_PERMITTED
} lcc_can_state_t;

typedef union {
    uint16_t alias16;
    struct {
        uint16_t pad    : 4;
        uint16_t alias : 12;
    };
} can_alias_id_t;


class LCC_Can : public LCC_Device {
public:
  LCC_Can(const device *);
  int attach(LCC_Node *);
  int send(lcc_message_t *);
  int recv(lcc_message_t *);
private:
  struct device const *can_dev;
  LCC_Node *node;
  lcc_can_state_t state = LCC_CAN_STATE_INHIBITED;
  can_alias_id_t alias = {.pad = 0, .alias = 0};
  int lcc_enter_permitted_state();
  void lcc_can_get_next_alias();
  int lcc_send_check_id();
  int lcc_send_reserve_id();
  int lcc_can_rx_lcc_frame(struct can_frame *);
  int lcc_can_rx_control_frame(struct can_frame *);
};

#endif // LCC_CAN_H

