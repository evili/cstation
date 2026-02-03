#ifndef BOARD_LEDS_H
#define BOARD_LEDS_H

#include <zephyr/drivers/gpio.h>

/* Board LED definitions for OpenLCB CAN transport */
#if DT_NODE_HAS_STATUS(DT_ALIAS(led0), okay)
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
#define HAVE_LED0 1
#else
#define HAVE_LED0 0
#endif

#if DT_NODE_HAS_STATUS(DT_ALIAS(led1), okay)
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
#define HAVE_LED1 1
#else
#define HAVE_LED1 0
#endif

#if DT_NODE_HAS_STATUS(DT_ALIAS(led2), okay)
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);
#define HAVE_LED2 1
#else
#define HAVE_LED2 0
#endif

#endif /* BOARD_LEDS_H */
