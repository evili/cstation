#include "../include/openlcb_can.h"
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>
#include <stdio.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>
#include "../include/openlcb_core.h"

LOG_MODULE_REGISTER(openlcb_can, LOG_LEVEL_DBG);

static const struct device *can_dev;
static olcan_state_t state = OLCAN_STATE_INHIBITED;
static uint16_t current_alias = 0;
static uint8_t node_id[6] = { 0x05, 0x01, 0x01, 0x01, 0x1A, 0x01 };

/* Candidate alias generation state */
static uint16_t alias_candidate = 0;
static uint16_t alias_probe_count = 0;

/* Timing/work items */
static struct k_work_delayable alias_rid_work;
static int last_tx_ms = 0;

/* CAN filter id */
static int rx_filter_id = -1;

/* LED indicator for CAN bus error states */
#if DT_NODE_HAS_STATUS(DT_ALIAS(led2), okay)
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);
#define HAVE_LED2 1
#else
#define HAVE_LED2 0
#endif

/* Green LED for CAN TX success (short flash) */
#if DT_NODE_HAS_STATUS(DT_ALIAS(led0), okay)
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
#define HAVE_LED0 1
#else
#define HAVE_LED0 0
#endif

/* Blue LED for CAN RX activity (short flash) */
#if DT_NODE_HAS_STATUS(DT_ALIAS(led1), okay)
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
#define HAVE_LED1 1
#else
#define HAVE_LED1 0
#endif

/* LED modes (mapped to tests via openlcb_can_get_led_mode()) */
enum olcan_led_mode {
    OLCAN_LED_NORMAL = 0,
    OLCAN_LED_ERROR_ACTIVE,
    OLCAN_LED_ERROR_PASSIVE,
    OLCAN_LED_BUS_OFF,
};

static atomic_t led_mode = ATOMIC_INIT(OLCAN_LED_NORMAL);
static struct k_work led_state_work;
static struct k_work_delayable led_blink_work;
static uint32_t led_blink_ms = 0;
static bool led_inverted = false; /* if board drives LED active low */

/* LED0 (green) short-flash state */
static atomic_t led0_flashing = ATOMIC_INIT(0);
static struct k_work led0_on_work;
static struct k_work_delayable led0_off_work;

/* LED1 (blue) short-flash state for RX */
static atomic_t led1_flashing = ATOMIC_INIT(0);
static struct k_work led1_on_work;
static struct k_work_delayable led1_off_work;


/* Forward declarations for LED/state handlers */
static void led_blink_handler(struct k_work *work);
static void led_state_apply(struct k_work *work);
static void can_state_change_cb(const struct device *dev, enum can_state st, struct can_bus_err_cnt err_cnt, void *user_data);

/* Forward declarations for LED0 handlers and CAN TX completion */
static void led0_on_handler(struct k_work *work);
static void led0_off_handler(struct k_work *work);
static void led0_trigger(void);
#if !defined(CONFIG_ZTEST)
static void can_tx_done(const struct device *dev, int error, void *user_data);
#endif

/* Forward declarations for LED1 handlers (RX LED) */
static void led1_on_handler(struct k_work *work);
static void led1_off_handler(struct k_work *work);
static void led1_trigger(void);



/* Helpers: build extended ID
 * Layout (bit numbers as in spec):
 * MSB (bit 28): reserved (1)
 * bit 27: frame type (0 control, 1 message)
 * bits 26-12: content field (15 bits)
 * bits 11-0: source Node ID alias (12 bits)
 */
static inline uint32_t build_extended_id(bool frame_type, uint32_t content_field, uint16_t source_alias)
{
    return (1U << 28) | ((frame_type ? 1U : 0U) << 27) | ((content_field & 0x7FFFU) << 12) | (source_alias & 0x0FFFU);
}

/* Send a control frame (no data by default) */
/* Test hook pointer: when set, all outgoing frames go to this hook instead
 * of the real CAN driver (useful for unit tests). */
static int (*test_send_hook)(const struct can_frame *frame, k_timeout_t timeout) = NULL;

static int send_frame_wrapper(const struct can_frame *frame, k_timeout_t timeout)
{
    if (test_send_hook) {
        int rc = test_send_hook(frame, timeout);
        /* Consider return 0 as successful transmit for test harness and flash LED0 */
        if (rc == 0) {
            led0_trigger();
        }
        return rc;
    }
#if defined(CONFIG_ZTEST)
    /* In test mode, avoid calling the real CAN syscall (not linked in test)
     * and return ENODEV so tests can set hooks explicitly.
     */
    return -ENODEV;
#else
    if (!can_dev) return -ENODEV;
    /* Provide a TX callback so we can flash LED0 on successful send */
    return can_send(can_dev, frame, timeout, can_tx_done, NULL);
#endif
}

static int send_control_frame(uint32_t content_field, uint16_t source_alias, const uint8_t *data, uint8_t dlc, k_timeout_t timeout)
{
    uint32_t id = build_extended_id(false, content_field, source_alias);
    struct can_frame frame = {0};
    frame.id = id;
    frame.dlc = dlc;
    frame.flags = CAN_FRAME_IDE;
    if (data && dlc) memcpy(frame.data, data, dlc);

    last_tx_ms = k_uptime_get_32();
    return send_frame_wrapper(&frame, timeout);
}

/* CID: MMM (3 bits) + 12-bit node_id chunk in content field; source_alias in id LSB */
static int send_cid(uint8_t mmm, uint16_t node_chunk_12, uint16_t source_alias)
{
    uint32_t content = ((uint32_t)(mmm & 0x7) << 12) | (node_chunk_12 & 0x0FFF);
    return send_control_frame(content, source_alias, NULL, 0, K_NO_WAIT);
}

/* RID (Reserve ID) uses content=0x0700 per spec */
#define CONTENT_RID 0x0700U
#define CONTENT_AMD 0x0701U
#define CONTENT_AME 0x0702U
#define CONTENT_AMR 0x0703U

static uint16_t hash_node_id_to_alias(const uint8_t nid[6], uint16_t salt)
{
    /* Deterministic alias generator: FNV-1a 32-bit then fold to 12 bits with salt */
    uint32_t h = 2166136261u;
    for (int i = 0; i < 6; i++) {
        h ^= nid[i];
        h *= 16777619u;
    }
    h ^= (uint32_t)salt;
    h *= 16777619u;
    return (uint16_t)(h & 0x0FFFu);
}

static void schedule_rid_send(struct k_work *work)
{
    ARG_UNUSED(work);
    LOG_DBG("alias: sending RID for candidate=0x%03x", alias_candidate);
    /* Send RID with candidate alias in source field */
    int rc = send_control_frame(CONTENT_RID, alias_candidate, NULL, 0, K_MSEC(100));
    if (rc < 0) {
        LOG_ERR("Failed to send RID: %d", rc);
        /* restart with next candidate */
        alias_probe_count++;
        alias_candidate = hash_node_id_to_alias(node_id, alias_probe_count);
        k_work_schedule(&alias_rid_work, K_MSEC(100 + alias_probe_count * 10));
        return;
    }

    /* After RID is sent, consider alias reserved and publish AMD */
    current_alias = alias_candidate;
    state = OLCAN_STATE_PERMITTED;
    LOG_INF("alias reserved: 0x%03x - publishing AMD", current_alias);

    /* send AMD with Node ID as data */
    send_control_frame(CONTENT_AMD, current_alias, node_id, sizeof(node_id), K_MSEC(100));
}

/* Fragment descriptor queued from ISR for processing in worker */
struct can_fragment {
    uint16_t src_alias;
    uint16_t can_mti;
    uint8_t ff; /* fragmentation flags: 0=only,1=first,3=middle,2=last */
    uint8_t dlc;
    uint8_t data[8];
};

K_MSGQ_DEFINE(can_frag_q, sizeof(struct can_fragment), 16, 4);

/* Worker thread to process fragments and reassemble messages */
#define REASM_MAX 8
struct reasm_entry {
    bool active;
    uint16_t src_alias;
    uint16_t can_mti;
    size_t len;
    uint8_t buf[256];
};
static struct reasm_entry reasm[REASM_MAX];

static void process_fragment(const struct can_fragment *frag)
{
    uint16_t src = frag->src_alias;
    uint16_t mti = frag->can_mti;
    uint8_t ff = frag->ff;
    size_t payload_offset = 0;
    uint8_t payload[8];

    if (ff == 0) {
        /* Only frame: payload is entire data (might include dest alias header) */
        payload_offset = 0;
        memcpy(payload, frag->data, frag->dlc);
        /* Deliver directly, stripping addressing flags if present according to MTI Address Present bit */
        openlcb_core_incoming_message(payload, frag->dlc);
        return;
    }

    /* For First/Middle/Last frames: append to reassembly buffer */
    struct reasm_entry *e = NULL;
    for (int i = 0; i < REASM_MAX; i++) {
        if (reasm[i].active && reasm[i].src_alias == src && reasm[i].can_mti == mti) {
            e = &reasm[i];
            break;
        }
    }
    if (!e && ff == 1) {
        /* allocate new entry */
        for (int i = 0; i < REASM_MAX; i++) {
            if (!reasm[i].active) {
                e = &reasm[i];
                e->active = true;
                e->src_alias = src;
                e->can_mti = mti;
                e->len = 0;
                break;
            }
        }
    }
    if (!e) {
        LOG_WRN("No reassembly buffer available for fragment src=0x%03x mti=0x%03x ff=%u", src, mti, ff);
        return;
    }

    /* Append fragment payload */
    size_t copy_len = frag->dlc;
    if (e->len + copy_len > sizeof(e->buf)) {
        LOG_ERR("Reassembly overflow");
        e->active = false;
        return;
    }
    memcpy(e->buf + e->len, frag->data, copy_len);
    e->len += copy_len;

    if (ff == 2) {
        /* last frame, deliver */
        openlcb_core_incoming_message(e->buf, e->len);
        e->active = false;
    }
}

/* Worker thread function */
static void can_frag_worker(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);
    struct can_fragment frag;

    while (1) {
        if (k_msgq_get(&can_frag_q, &frag, K_FOREVER) == 0) {
            process_fragment(&frag);
        }
    }
}

K_THREAD_STACK_DEFINE(can_frag_stack, 1024);
K_THREAD_DEFINE(can_frag_thread_id, 1024, can_frag_worker, NULL, NULL, NULL, 7, 0, 0);

/* RX callback called in ISR context by CAN driver */
static void can_rx_cb(const struct device *dev, struct can_frame *frame, void *user_data)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(user_data);

    /* Flash RX LED for any received frame (ignore if already flashing) */
    led1_trigger();

    /* Ignore non-extended frames */
    if ((frame->flags & CAN_FRAME_IDE) == 0) {
        return;
    }

    uint32_t id = frame->id;
    bool reserved_bit = (id >> 28) & 0x1;
    bool frame_type = (id >> 27) & 0x1; /* 0 control, 1 message */
    uint32_t content = (id >> 12) & 0x7FFFU;
    uint16_t src_alias = id & 0x0FFFU;

    LOG_DBG("CAN RX id=0x%08x reserved=%u type=%u content=0x%04x alias=0x%03x dlc=%u",
            id, reserved_bit, frame_type, content, src_alias, frame->dlc);

    /* Ignore frames without frame type = 0 (control) */
    if (frame_type == 0) {
        /* Handle control frames as before */
        if ((content & 0x0F00U) == 0x0700U) {
            uint16_t cmd = content & 0x0FFFU;
            switch (cmd) {
            case 0x0700: /* RID */
                LOG_DBG("Received RID from alias 0x%03x", src_alias);
                if (state == OLCAN_STATE_REQUESTING_ALIAS && src_alias == alias_candidate) {
                    LOG_INF("Collision detected for alias 0x%03x", alias_candidate);
                    alias_probe_count++;
                    alias_candidate = hash_node_id_to_alias(node_id, alias_probe_count);
                    k_work_schedule(&alias_rid_work, K_MSEC(100));
                }
                break;
            case 0x0701: /* AMD */
                LOG_DBG("Received AMD (Alias Map Definition) from alias 0x%03x", src_alias);
                if (state == OLCAN_STATE_PERMITTED && src_alias == current_alias) {
                    LOG_WRN("Alias conflict: another node advertised same alias 0x%03x - relinquishing alias", current_alias);
                    send_control_frame(CONTENT_AMR, current_alias, node_id, sizeof(node_id), K_MSEC(100));
                    current_alias = 0;
                    state = OLCAN_STATE_INHIBITED;
                    alias_probe_count = 0;
                    alias_candidate = hash_node_id_to_alias(node_id, alias_probe_count);
                    k_work_schedule(&alias_rid_work, K_MSEC(100));
                }
                break;
            case 0x0703: /* AMR */
                LOG_DBG("Received AMR referencing alias 0x%03x", src_alias);
                if (state == OLCAN_STATE_PERMITTED && src_alias == current_alias) {
                    LOG_WRN("AMR for our alias - relinquishing 0x%03x", current_alias);
                    current_alias = 0;
                    state = OLCAN_STATE_INHIBITED;
                    alias_probe_count = 0;
                    alias_candidate = hash_node_id_to_alias(node_id, alias_probe_count);
                    k_work_schedule(&alias_rid_work, K_MSEC(100));
                }
                break;
            default:
                LOG_DBG("Unknown control content: 0x%04x", content);
            }
        }
        return;
    }

    /* Message frame: prepare fragment descriptor and enqueue for worker */
    struct can_fragment frag = {0};
    frag.src_alias = src_alias;
    frag.can_mti = content & 0x0FFFU; /* lower 12 bits = CAN MTI */
    uint8_t ff = 0;
    uint8_t off = 0;
    if (frame->dlc == 0) return;

    /* Determine if this MTI is a Stream/Datagram (bit 12) */
    bool is_stream_or_datagram = (frag.can_mti & 0x1000U) != 0;
    if (is_stream_or_datagram) {
        /* For datagram/stream frames: complete frames have no header; fragmented datagrams
         * use a 1-byte control header with ff in the top nibble.
         */
        if (frame->dlc == 0) return;
        uint8_t b0 = frame->data[0];
        uint8_t possible_ff = (b0 >> 4) & 0x3U;
        /* If top nibble matches a known ff encoding, treat this as having a 1-byte control header */
        if (possible_ff == 0 || possible_ff == 1 || possible_ff == 2 || possible_ff == 3) {
            ff = possible_ff;
            off = 1;
        } else {
            /* No control header - entire payload is application data */
            ff = 0;
            off = 0;
        }
    } else {
        bool addr_present = (frag.can_mti & 0x0010U) != 0;
        if (addr_present) {
            if (frame->dlc < 2) return;
            uint8_t b0 = frame->data[0];
            ff = (b0 >> 4) & 0x3U;
            off = 2; /* payload starts after dest alias */
        } else {
            if (frame->dlc < 1) return;
            uint8_t b0 = frame->data[0];
            ff = (b0 >> 4) & 0x3U;
            off = 1; /* payload starts after control flags byte */
        }
    }

    frag.ff = (ff == 0x0) ? 0 : (ff == 0x1 ? 1 : (ff == 0x3 ? 3 : 2));
    frag.dlc = frame->dlc - off;
    if (frag.dlc > 8) frag.dlc = 8;
    memcpy(frag.data, frame->data + off, frag.dlc);

    int ret = k_msgq_put(&can_frag_q, &frag, K_NO_WAIT);
    if (ret != 0) {
        LOG_ERR("Fragment queue full - dropped frag from 0x%03x", src_alias);
    }
}

int openlcb_can_init_transport(const struct device *can_dev_in)
{
    if (!can_dev_in) return -EINVAL;
    can_dev = can_dev_in;

    if (!device_is_ready(can_dev)) {
        LOG_ERR("CAN device not ready");
        return -ENODEV;
    }

    /* Add permissive extended RX filter */
    struct can_filter fext = {
        .id = 0,
        .mask = 0,
        .flags = CAN_FILTER_IDE
    };
    rx_filter_id = can_add_rx_filter(can_dev, can_rx_cb, NULL, &fext);
    if (rx_filter_id < 0) {
        LOG_ERR("Failed to add CAN RX filter: %d", rx_filter_id);
        return rx_filter_id;
    }

    k_work_init_delayable(&alias_rid_work, schedule_rid_send);

    /* prepare initial alias candidate */
    alias_probe_count = 0;
    alias_candidate = hash_node_id_to_alias(node_id, alias_probe_count);

    /* Initialize LED work and register CAN state change callback */
    k_work_init(&led_state_work, led_state_apply);
    k_work_init_delayable(&led_blink_work, led_blink_handler);

    /* Configure LED (if available) and ensure it is off initially */
#if HAVE_LED2
    if (device_is_ready(led2.port)) {
        gpio_pin_configure_dt(&led2, GPIO_OUTPUT_INACTIVE);
    } else {
        LOG_WRN("LED2 device not ready");
    }
#endif

    /* Initialize LED0 handlers and configure LED0 if present */
    k_work_init(&led0_on_work, led0_on_handler);
    k_work_init_delayable(&led0_off_work, led0_off_handler);
#if HAVE_LED0
    if (device_is_ready(led0.port)) {
        gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);
    } else {
        LOG_WRN("LED0 device not ready");
    }
#endif

    /* Initialize LED1 handlers and configure LED1 if present */
    k_work_init(&led1_on_work, led1_on_handler);
    k_work_init_delayable(&led1_off_work, led1_off_handler);
#if HAVE_LED1
    if (device_is_ready(led1.port)) {
        gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);
    } else {
        LOG_WRN("LED1 device not ready");
    }
#endif

    /* Register callback with driver (optional, not all drivers require it) */
    can_set_state_change_callback(can_dev, can_state_change_cb, NULL);

    LOG_INF("openlcb_can: initialized - node id %02x:%02x:%02x:%02x:%02x:%02x",
            node_id[0], node_id[1], node_id[2], node_id[3], node_id[4], node_id[5]);

    return 0;
}

int openlcb_can_start_alias_negotiation(void)
{
    if (!can_dev) return -ENODEV;
    if (state == OLCAN_STATE_PERMITTED) return -EALREADY;

    state = OLCAN_STATE_REQUESTING_ALIAS;
    alias_probe_count = 0;
    alias_candidate = hash_node_id_to_alias(node_id, alias_probe_count);

    LOG_INF("Starting alias negotiation with initial candidate 0x%03x", alias_candidate);

    /* Send CID frames with MMM=7,6,5,4 and respective node id chunks */
    uint64_t nidbits = 0;
    for (int i = 0; i < 6; i++) nidbits = (nidbits << 8) | node_id[i];
    /* bits 36-47, 24-35, 12-23, 0-11 */
    uint16_t chunks[4];
    chunks[0] = (nidbits >> 36) & 0x0FFF;
    chunks[1] = (nidbits >> 24) & 0x0FFF;
    chunks[2] = (nidbits >> 12) & 0x0FFF;
    chunks[3] = (nidbits >> 0) & 0x0FFF;

    send_cid(0x7, chunks[0], alias_candidate);
    k_sleep(K_MSEC(5));
    send_cid(0x6, chunks[1], alias_candidate);
    k_sleep(K_MSEC(5));
    send_cid(0x5, chunks[2], alias_candidate);
    k_sleep(K_MSEC(5));
    send_cid(0x4, chunks[3], alias_candidate);

    /* Wait at least 200ms then send RID - schedule work */
    k_work_schedule(&alias_rid_work, K_MSEC(250));

    return 0;
}

uint16_t openlcb_can_get_alias(void)
{
    return current_alias;
}

/* Test API implementations */
int openlcb_can_set_send_hook(openlcb_can_send_hook_t hook)
{
    test_send_hook = (int (*)(const struct can_frame *, k_timeout_t))hook;
    return 0;
}

int openlcb_can_test_init(void)
{
    /* Minimal init for tests: set a non-null can_dev and prepare work and alias */
    can_dev = (const struct device *)1; /* sentinel */
    k_work_init_delayable(&alias_rid_work, schedule_rid_send);
    alias_probe_count = 0;
    alias_candidate = hash_node_id_to_alias(node_id, alias_probe_count);
    state = OLCAN_STATE_INHIBITED;
    current_alias = 0;
    return 0;
}

int openlcb_can_inject_frame(uint32_t id, const uint8_t *data, uint8_t dlc)
{
    struct can_frame frame = {0};
    frame.id = id;
    frame.flags = CAN_FRAME_IDE;
    frame.dlc = dlc;
    if (dlc && data) memcpy(frame.data, data, dlc);
    /* Call the RX handler as the real driver would */
    can_rx_cb(can_dev, &frame, NULL);
    return 0;
}

int openlcb_can_set_node_id(const uint8_t node_id_in[6])
{
    if (!node_id_in) return -EINVAL;
    memcpy(node_id, node_id_in, sizeof(node_id));
    /* Update initial candidate */
    alias_probe_count = 0;
    alias_candidate = hash_node_id_to_alias(node_id, alias_probe_count);
    return 0;
}

int openlcb_can_send_message(uint16_t mti, uint16_t dest_alias, const uint8_t *data, size_t len, k_timeout_t timeout)
{
    if (!can_dev) return -ENODEV;
    if (current_alias == 0) return -EPERM; /* not connected/permitted */

    /* Determine if message is Stream/Datagram based on MTI bit (bit 12) */
    bool is_stream_or_datagram = (mti & 0x1000U) != 0;

    uint16_t can_mti = mti & 0x0FFFU; /* low 12 bits */
    /* Per S-9.7.3: set Address-Present bit (bit 4 / mask 0x0010) when a destination alias is specified */
    if (dest_alias != 0x0FFFU) {
        can_mti |= 0x0010U;
    }

    if (!is_stream_or_datagram) {
        /* Map to CAN Frame Format 1 (Global & Addressed) - allow fragmentation */
        size_t offset = 0;
        bool first = true;
        while (offset < len) {
            uint8_t ff = 0; /* 00 only, 01 first, 11 middle, 10 last */
            uint8_t header[2] = {0};
            size_t payload_pos = 0;

            if (first && len - offset <= 8) {
                ff = 0; /* only frame */
                /* payload starts at byte 0 */
                payload_pos = 0;
            } else if (first) {
                ff = 1; /* first */
                payload_pos = 0;
            } else if (len - offset <= 8) {
                ff = 2; /* last */
                payload_pos = 0;
            } else {
                ff = 3; /* middle */
                payload_pos = 0;
            }

            struct can_frame frame = {0};
            frame.flags = CAN_FRAME_IDE;
            uint32_t id = build_extended_id(true, can_mti, current_alias);

            /* If address present (we use dest_alias != 0x0FFF as indicator), include first 2 bytes */
            if (dest_alias != 0x0FFF) {
                /* pack: first byte = rr(2)=0 | ff(2) <<4 | top nibble of dest alias
                 * second byte = low 8 bits of dest alias
                 */
                header[0] = (uint8_t)((ff & 0x3) << 4) | (uint8_t)((dest_alias >> 8) & 0x0F);
                header[1] = (uint8_t)(dest_alias & 0xFF);
                /* payload per frame limited to 6 bytes when dest present */
                size_t avail = 8 - 2;
                size_t take = MIN(avail, len - offset);
                frame.dlc = 2 + take;
                memcpy(frame.data, header, 2);
                if (take)
                    memcpy(frame.data + 2, data + offset, take);
                offset += take;
            } else {
                /* no dest alias, place ff in top nibble of first data byte */
                header[0] = (uint8_t)((ff & 0x3) << 4);
                size_t take = MIN((size_t)8, len - offset);
                frame.dlc = 1 + take;
                memcpy(frame.data, header, 1);
                if (take)
                    memcpy(frame.data + 1, data + offset, take);
                offset += take;
            }

            frame.id = id;
            int rc = send_frame_wrapper(&frame, timeout);
            if (rc < 0) return rc;
            first = false;
        }
        return 0;
    } else {
        /* Datagram/Stream mapping - use formats 2..5 or 7
         * We'll use Datagram formats (2=complete, 3=first,4=middle,5=final) for datagrams.
         */
        size_t remaining = len;
        size_t offset = 0;
        bool first = true;
        while (remaining > 0) {
            struct can_frame frame = {0};
            frame.flags = CAN_FRAME_IDE;

            size_t payload_space = 8;
            size_t take = MIN(payload_space, remaining);
            uint8_t ff_val = 0;

            if (first && remaining == take) {
                /* Datagram fits in single frame -> no control header, payload only */
                frame.dlc = take;
                memcpy(frame.data, data + offset, take);
                uint32_t id = build_extended_id(true, can_mti, current_alias);
                frame.id = id;
                int rc = send_frame_wrapper(&frame, timeout);
                if (rc < 0) return rc;
                return 0;
            }

            /* Fragmented datagram: send a 1-byte control header with ff in top nibble */
            if (first) {
                ff_val = 1; /* first */
            } else if (remaining == take) {
                ff_val = 2; /* last */
            } else {
                ff_val = 3; /* middle */
            }

            /* One-byte header consumes 1 octet of payload space */
            size_t use = MIN((size_t)(payload_space - 1), remaining);
            frame.dlc = 1 + use;
            frame.data[0] = (uint8_t)((ff_val & 0x3) << 4);
            if (use)
                memcpy(frame.data + 1, data + offset, use);

            uint32_t id = build_extended_id(true, can_mti, current_alias);
            frame.id = id;
            int rc = send_frame_wrapper(&frame, timeout);
            if (rc < 0) return rc;

            offset += use;
            remaining -= use;
            first = false;
        }
        return 0; 
    }
}

olcan_state_t openlcb_can_get_state(void)
{
    return state;
}

/* LED blink handler: toggles LED and reschedules itself */
static void led_blink_handler(struct k_work *work)
{
    ARG_UNUSED(work);
#if HAVE_LED2
    if (!device_is_ready(led2.port)) {
        return;
    }
    gpio_pin_toggle_dt(&led2);
#endif
    /* reschedule if period is non-zero */
    if (led_blink_ms > 0) {
        k_work_schedule(&led_blink_work, K_MSEC(led_blink_ms));
    }
}

/* Apply a LED mode change from work context */
static void led_state_apply(struct k_work *work)
{
    ARG_UNUSED(work);
    int m = atomic_get(&led_mode);

    switch (m) {
    case OLCAN_LED_ERROR_ACTIVE:
        /* slow flash - half-period configurable (default 1000ms) */
#ifdef CONFIG_OLCAN_LED_ERROR_ACTIVE_MS
        led_blink_ms = CONFIG_OLCAN_LED_ERROR_ACTIVE_MS;
#else
        led_blink_ms = 1000;
#endif
#if HAVE_LED2
        if (device_is_ready(led2.port)) {
            gpio_pin_configure_dt(&led2, GPIO_OUTPUT_INACTIVE);
        }
#endif
        k_work_schedule(&led_blink_work, K_NO_WAIT);
        break;
    case OLCAN_LED_ERROR_PASSIVE:
        /* rapid flash - half-period configurable (default 100ms) */
#ifdef CONFIG_OLCAN_LED_ERROR_PASSIVE_MS
        led_blink_ms = CONFIG_OLCAN_LED_ERROR_PASSIVE_MS;
#else
        led_blink_ms = 100;
#endif
#if HAVE_LED2
        if (device_is_ready(led2.port)) {
            gpio_pin_configure_dt(&led2, GPIO_OUTPUT_INACTIVE);
        }
#endif
        k_work_schedule(&led_blink_work, K_NO_WAIT);
        break;
    case OLCAN_LED_BUS_OFF:
#if HAVE_LED2
        if (device_is_ready(led2.port)) {
            gpio_pin_configure_dt(&led2, GPIO_OUTPUT_ACTIVE);
        }
#endif
        /* stop blinking */
        k_work_cancel_delayable(&led_blink_work);
        led_blink_ms = 0;
        break;
    case OLCAN_LED_NORMAL:
    default:
#if HAVE_LED2
        if (device_is_ready(led2.port)) {
            gpio_pin_configure_dt(&led2, GPIO_OUTPUT_INACTIVE);
        }
#endif
        k_work_cancel_delayable(&led_blink_work);
        led_blink_ms = 0;
        break;
    }
}

/* CAN state change callback (registered with the CAN driver). Called from driver context.
 * We set the new LED mode atomically and schedule a work item to apply it in thread context. */
static void can_state_change_cb(const struct device *dev, enum can_state st, struct can_bus_err_cnt err_cnt, void *user_data)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(err_cnt);
    ARG_UNUSED(user_data);

    int new_mode = OLCAN_LED_NORMAL;
    switch (st) {
    case CAN_STATE_ERROR_ACTIVE:
        new_mode = OLCAN_LED_ERROR_ACTIVE;
        break;
    case CAN_STATE_ERROR_PASSIVE:
        new_mode = OLCAN_LED_ERROR_PASSIVE;
        break;
    case CAN_STATE_BUS_OFF:
        new_mode = OLCAN_LED_BUS_OFF;
        break;
    default:
        new_mode = OLCAN_LED_NORMAL;
        break;
    }

    atomic_set(&led_mode, new_mode);
    k_work_submit(&led_state_work);
}

/* Test helper: inject a simulated CAN controller state change */
int openlcb_can_inject_state_change(int state)
{
    struct can_bus_err_cnt cnt = { .tx_err_cnt = 0, .rx_err_cnt = 0 };
    can_state_change_cb(can_dev, (enum can_state)state, cnt, NULL);
    return 0;
}

/* Test helper: return current LED mode */
int openlcb_can_get_led_mode(void)
{
    return atomic_get(&led_mode);
}

/* Test helper: get current blink half-period in ms (0 when not blinking) */
int openlcb_can_get_blink_ms(void)
{
    return (int)led_blink_ms;
}

/* LED0 on: set pin active and schedule off handler */
static void led0_on_handler(struct k_work *work)
{
    ARG_UNUSED(work);
#if HAVE_LED0
    if (device_is_ready(led0.port)) {
        gpio_pin_set_dt(&led0, 1);
    }
#endif
    /* schedule off after configured duration (default 1ms) */
#ifdef CONFIG_OLCAN_LED_TX_MS
    k_work_schedule(&led0_off_work, K_MSEC(CONFIG_OLCAN_LED_TX_MS));
#else
    k_work_schedule(&led0_off_work, K_MSEC(1));
#endif
}

/* LED0 off: clear pin and mark not flashing */
static void led0_off_handler(struct k_work *work)
{
    ARG_UNUSED(work);
#if HAVE_LED0
    if (device_is_ready(led0.port)) {
        gpio_pin_set_dt(&led0, 0);
    }
#endif
    atomic_set(&led0_flashing, 0);
}

/* Trigger a short flash: ensure we don't restart if already flashing */
static void led0_trigger(void)
{
    /* Attempt to set flashing flag; if already set, do nothing */
    if (!atomic_cas(&led0_flashing, 0, 1)) {
        /* already flashing - ignore */
        return;
    }
    k_work_submit(&led0_on_work);
}

/* CAN TX completion callback: only compiled in non-test builds where can_send is used */
#if !defined(CONFIG_ZTEST)
static void can_tx_done(const struct device *dev, int error, void *user_data)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(user_data);
    if (error == 0) {
        led0_trigger();
    }
}
#endif

/* Test helper to get LED0 busy state */
int openlcb_can_get_led0_busy(void)
{
    return atomic_get(&led0_flashing);
}

/* Test helper: get configured LED0 flash duration in ms (0 if not configured) */
int openlcb_can_get_led0_flash_ms(void)
{
#ifdef CONFIG_OLCAN_LED_TX_MS
    return CONFIG_OLCAN_LED_TX_MS;
#else
    return 1;
#endif
}

/* LED1 on: set pin active and schedule off handler */
static void led1_on_handler(struct k_work *work)
{
    ARG_UNUSED(work);
#if HAVE_LED1
    if (device_is_ready(led1.port)) {
        gpio_pin_set_dt(&led1, 1);
    }
#endif
    /* schedule off after configured duration (default 1ms) */
#ifdef CONFIG_OLCAN_LED_RX_MS
    k_work_schedule(&led1_off_work, K_MSEC(CONFIG_OLCAN_LED_RX_MS));
#else
    k_work_schedule(&led1_off_work, K_MSEC(1));
#endif
}

/* LED1 off: clear pin and mark not flashing */
static void led1_off_handler(struct k_work *work)
{
    ARG_UNUSED(work);
#if HAVE_LED1
    if (device_is_ready(led1.port)) {
        gpio_pin_set_dt(&led1, 0);
    }
#endif
    atomic_set(&led1_flashing, 0);
}

/* Trigger a short flash for LED1: ensure we don't restart if already flashing */
static void led1_trigger(void)
{
    /* Attempt to set flashing flag; if already set, do nothing */
    if (!atomic_cas(&led1_flashing, 0, 1)) {
        /* already flashing - ignore */
        return;
    }
    k_work_submit(&led1_on_work);
}

/* Test helper to get LED1 busy state */
int openlcb_can_get_led1_busy(void)
{
    return atomic_get(&led1_flashing);
}

/* Test helper: get configured LED1 flash duration in ms (0 if not configured) */
int openlcb_can_get_led1_flash_ms(void)
{
#ifdef CONFIG_OLCAN_LED_RX_MS
    return CONFIG_OLCAN_LED_RX_MS;
#else
    return 1;
#endif
}

 
