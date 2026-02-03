/* OpenLCB-CAN transport layer
 * - Implements CAN-specific encoding/decoding per S-9.7.2.1 and TN-9.7.2.1
 * - Manages alias negotiation / state machine (Inhibited / Permitted)
 */
#ifndef OPENLCB_CAN_H
#define OPENLCB_CAN_H

#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    OLCAN_STATE_INHIBITED = 0,
    OLCAN_STATE_REQUESTING_ALIAS,
    OLCAN_STATE_PERMITTED,
} olcan_state_t;

int openlcb_can_init_transport(const struct device *can_dev);

/* Start alias negotiation (deterministic generator based on Node ID) */
int openlcb_can_start_alias_negotiation(void);

/* Get current alias (0 if none) */
uint16_t openlcb_can_get_alias(void);

/* Set Node ID (6 bytes) */
int openlcb_can_set_node_id(const uint8_t node_id[6]);

/* Send an OpenLCB message (transport will map to CAN frames) */
int openlcb_can_send_message(uint16_t mti, uint16_t dest_alias, const uint8_t *data, size_t len, k_timeout_t timeout);

/* For diagnostics */
olcan_state_t openlcb_can_get_state(void);

/* -------- Test hooks (unit tests) -------- */
typedef int (*openlcb_can_send_hook_t)(const struct can_frame *frame, k_timeout_t timeout);

/* Replace the internal send function with a test hook (captures frames) */
int openlcb_can_set_send_hook(openlcb_can_send_hook_t hook);

/* Initialize transport in test mode (no real device required) */
int openlcb_can_test_init(void);

/* Inject a raw CAN frame (id + data) into the RX path (test helper) */
int openlcb_can_inject_frame(uint32_t id, const uint8_t *data, uint8_t dlc);

/* Inject a simulated CAN controller state change (for unit tests).
 * @param state One of Zephyr's CAN controller states (enum can_state).
 */
int openlcb_can_inject_state_change(int state);

/* Test hook: get current LED indicator mode (0=normal, 1=Error Active, 2=Error Passive, 3=Bus Off) */
int openlcb_can_get_led_mode(void);

/* Test helper: get current blink half-period in ms (0 when not blinking) */
int openlcb_can_get_blink_ms(void);

/* Test helper: get LED0 (green) busy state (1 when flashing) */
int openlcb_can_get_led0_busy(void);

/* Test helper: get LED1 (blue) busy state (1 when flashing) */
int openlcb_can_get_led1_busy(void);

/* Test helper: get configured LED1 flash duration in ms */
int openlcb_can_get_led1_flash_ms(void);

/* Test helper: get configured LED0 flash duration in ms */
int openlcb_can_get_led0_flash_ms(void);

#ifdef __cplusplus
}
#endif

#endif /* OPENLCB_CAN_H */
