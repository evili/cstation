# cstation

Minimal Zephyr application that implements partially LCC protocol.

##WARNING##: It is incomplete, non compliant, and non usable. Just an excercise.

## Build

1. Configure your Zephyr environment (west, ZEPHYR_BASE, etc.).
2. Build:

    west build -b nucleo_f767zi -d build

3. Flash:

    west flash

## Notes

- Ensure an external CAN transceiver is present and wired to the Nucleo CAN pins before connecting to the LCC/OpenLCB network.
