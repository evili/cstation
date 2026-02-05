#ifndef   LCC_CAN_H
#define   LCC_CAN_H

#include <zephyr/drivers/can.h>
#include "lcc.h"

#define LCC_CAN_CHECK_ID_WAIT 200 // milliseconds

#define LCC_CAN_BIT_RATE 125000 // 125 kbps


typedef union {
    can_frame_id_t can_frame;
    struct {
        uint32_t padding  :  3;
        uint32_t reserved  : 1;
        uint32_t lcc_frame : 1;
        uint32_t content   :15;
        uint32_t source_id :12;
        uint8_t  dlc;
        uint16_t timestamp;
        uint8_t  data[CAN_MAX_DLEN];
    }
} lcc_can_frame_t;


// Filter Masks 
LCC_CAN_FILTER_CONTROL_FRAME_MASK 0x08000000
LCC_CAN_FILTER_CONTENT_FIELD_MASK 0x07FFF000
LCC_CAN_FILTER_SOURCE_ID_MASK     0x00000FFF

LCC_CAN_CONTROL_FRAME 0x00000000
LCC_CAN_LCC_FRAME     0x08000000

const can_filter_t LCC_CAN_CONTROL_FRAME_FILTER = {
    .id    = LCC_CAN_CONTROL_FRAME,
    .mask  = LCC_CAN_FILTER_CONTROL_FRAME_MASK,
    .flags = CAN_FILTER_IDE
};

const can_filter_t LCC_CAN_LCC_FRAME_FILTER = {
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


typedef stuct {
    lcc_can_state_t state;
    const struct device *can_dev;
    const lcc_node_t *node;
    can_alias_id_t alias_id;
} lcc_can_dev_t;


int lcc_can_attach(void *can_device, const lcc_node_t *node);
int lcc_can_send_message(void *can_device, const lcc_message_t *message);
int lcc_can_receive_message(void *can_device, lcc_message_t *message);

void lcc_can_rx_lcc_frame(const lcc_can_dev_t *dev, struct can_frame *frame, void *user_data);
void lcc_can_rx_control_frame(const lcc_can_dev_t *dev, struct can_frame *frame, void *user_data);

#endif // LCC_CAN_H

