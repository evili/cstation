#ifndef   LCC_CAN_H
#define   LCC_CAN_H

#include <zephyr/drivers/can.h>
#include "lcc.h"

#define LCC_CAN_CHECK_ID_WAIT 200 // milliseconds

#define LCC_CAN_BIT_RATE 125000 // 125 kbps


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

typedef stuct {
    lcc_can_state_t state;
    const struct device *can_dev;
    const lcc_node_t *node;
} lcc_can_dev_t;


int lcc_can_attach(void *can_device, const lcc_node_t *node);
int lcc_can_send_message(void *can_device, const lcc_message_t *message);
int lcc_can_receive_message(void *can_device, lcc_message_t *message);

#endif // LCC_CAN_H

