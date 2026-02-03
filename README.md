# cstation

Minimal Zephyr application that enables `can1` on `nucleo_f767zi` and verifies the device is ready.

## Build

1. Configure your Zephyr environment (west, ZEPHYR_BASE, etc.).
2. Build:

    west build -b nucleo_f767zi -d build

3. Flash:

    west flash

## Notes

- Ensure an external CAN transceiver is present and wired to the Nucleo CAN pins before connecting to the LCC/OpenLCB network.

- Implemented a minimal OpenLCB helper that: enforces 29-bit
  (extended) CAN frames only, registers an ISR RX handler, and
  provides APIs to set and send the Node ID. The default Node ID is
  **05:01:01:01:1A:01** (developer reserved range). See
  `include/openlcb.h` and `src/openlcb.c` for details. Note: payload
  format and extended CAN ID used in `openlcb_send_node_id()` are
  placeholders — update to the exact S-9.7.2.1 mapping if you need
  strict interoperability.
