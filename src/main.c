/*
 * Copyright (c) 2021-2022 Henrik Brix Andersen <henrik@brixandersen.dk>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <version.h>

#if KERNEL_VERSION_NUMBER == ZEPHYR_VERSION(3, 0, 0)
#include <device.h>
#include <devicetree.h>
#include <drivers/can.h>
#include <drivers/gpio.h>
#include <kernel.h>

#include <logging/log.h>

typedef struct zcan_frame can_frame_t;
typedef struct zcan_filter can_filter_t;
#else
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>

typedef struct can_frame can_frame_t;
typedef struct can_filter can_filter_t;
#endif

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

/* Devicetree */
#define CANBUS_NODE DT_NODELABEL(can1)
#define BUTTON_NODE DT_ALIAS(sw0)
#define BUTTON_NAME DT_PROP_OR(BUTTON_NODE, label, "sw0")

/* CAN frame to be sent */
static const can_frame_t frame = {
	.id_type = CAN_STANDARD_IDENTIFIER,
	.id = 0x7c9,
	.rtr = CAN_DATAFRAME,
	.fd = 0,
	.brs = 0,
	.dlc = 0,
	.data = { },
};

#if DT_NODE_EXISTS(BUTTON_NODE)
struct button_callback_context {
	struct gpio_callback callback;
	struct k_sem sem;
};

static void button_callback(const struct device *port, struct gpio_callback *cb,
			    gpio_port_pins_t pins)
{
	struct button_callback_context *ctx =
		CONTAINER_OF(cb, struct button_callback_context, callback);

	k_sem_give(&ctx->sem);
}
#endif /* DT_NODE_EXISTS(BUTTON_NODE) */

#if KERNEL_VERSION_NUMBER == ZEPHYR_VERSION(3, 0, 0)
static void can_tx_callback(int error, void *user_data)
#else
static void can_tx_callback(const struct device *dev, int error, void *user_data)
#endif
{
	LOG_INF("CAN frame sent");
	struct k_sem *tx_queue_sem = user_data;

	k_sem_give(tx_queue_sem);
}

CAN_MSGQ_DEFINE(rxq, 100u);

void main(void)
{
#if DT_NODE_EXISTS(BUTTON_NODE)
	const struct gpio_dt_spec btn = GPIO_DT_SPEC_GET(BUTTON_NODE, gpios);
	struct button_callback_context btn_cb_ctx;
#endif /* DT_NODE_EXISTS(BUTTON_NODE) */
	const struct device *dev = DEVICE_DT_GET(CANBUS_NODE);
	struct k_sem tx_queue_sem;
	int err;

	k_sem_init(&tx_queue_sem, CONFIG_SAMPLE_CAN_BABBLING_TX_QUEUE_SIZE,
		   CONFIG_SAMPLE_CAN_BABBLING_TX_QUEUE_SIZE);

	if (!device_is_ready(dev)) {
		printk("CAN device not ready");
		return;
	}

	can_filter_t filter = {
		.id_type = CAN_STANDARD_IDENTIFIER,
		.rtr_mask = 0,
		.id = 0x7cd,
		.id_mask = 0x7cd,
	};

	err = can_add_rx_filter_msgq(dev, &rxq, &filter);
	if (err < 0) {
		printk("Failed to add filter: %d", err);
		return;
	}

#if KERNEL_VERSION_NUMBER == ZEPHYR_VERSION(3, 2, 0)
	err = can_start(dev);
	if (err != 0) {
		printk("Error starting CAN controller [%d]", err);
		return;
	}
#endif

#if DT_NODE_EXISTS(BUTTON_NODE)
	k_sem_init(&btn_cb_ctx.sem, 0, 1);

	if (!device_is_ready(btn.port)) {
		printk("button device not ready\n");
		return;
	}

	err = gpio_pin_configure_dt(&btn, GPIO_INPUT);
	if (err != 0) {
		printk("failed to configure button GPIO (err %d)\n", err);
		return;
	}

	err = gpio_pin_interrupt_configure_dt(&btn, GPIO_INT_EDGE_TO_ACTIVE);
	if (err != 0) {
		printk("failed to configure button interrupt (err %d)\n", err);
		return;
	}

	gpio_init_callback(&btn_cb_ctx.callback, button_callback, BIT(btn.pin));
	gpio_add_callback(btn.port, &btn_cb_ctx.callback);
#endif /* DT_NODE_EXISTS(BUTTON_NODE) */

	printk("babbling on %s with %s (%d-bit) CAN ID 0x%0*x, RTR %d, CAN-FD %d\n",
	       dev->name,
	       frame.id_type == CAN_STANDARD_IDENTIFIER ? "standard" : "extended",
	       frame.id_type == CAN_STANDARD_IDENTIFIER ? 11 : 29,
	       frame.id_type == CAN_STANDARD_IDENTIFIER ? 3 : 8, frame.id,
	       frame.rtr, frame.fd);

#if DT_NODE_EXISTS(BUTTON_NODE)
	printk("Send by pressing %s button\n", BUTTON_NAME);
#endif /* DT_NODE_EXISTS(BUTTON_NODE) */

	struct k_poll_event events[] = {
		K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_SEM_AVAILABLE,
			K_POLL_MODE_NOTIFY_ONLY,
			&btn_cb_ctx.sem, 0),
		K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_MSGQ_DATA_AVAILABLE,
			K_POLL_MODE_NOTIFY_ONLY,
			&rxq, 0),
	};

	while (true)
	{
		err = k_poll(events, ARRAY_SIZE(events), K_FOREVER);
		if (err == 0) {
			if (events[0].state == K_POLL_STATE_SEM_AVAILABLE) {
				if (k_sem_take(&btn_cb_ctx.sem, K_NO_WAIT) == 0) {
					err = can_send(dev, &frame, K_NO_WAIT, can_tx_callback, &tx_queue_sem);
					if (err != 0) {
						LOG_INF("failed to enqueue CAN frame (err %d)", err);
					} else {
						LOG_INF("CAN frame enqueued");
					}
				}
				events[0].state = K_POLL_STATE_NOT_READY;
			}

			if (events[1].state == K_POLL_STATE_MSGQ_DATA_AVAILABLE) {
				can_frame_t rx_frame;
				err = k_msgq_get(&rxq, &rx_frame, K_NO_WAIT);
				if (err == 0) {
					LOG_INF("received CAN frame ID 0x%0*x, RTR %d, DLC %d, data %02x %02x %02x %02x %02x %02x %02x %02x",
					       frame.id_type == CAN_STANDARD_IDENTIFIER ? 3 : 8, rx_frame.id,
					       rx_frame.rtr, rx_frame.dlc,
					       rx_frame.data[0], rx_frame.data[1], rx_frame.data[2], rx_frame.data[3],
					       rx_frame.data[4], rx_frame.data[5], rx_frame.data[6], rx_frame.data[7]);
				}
			}
		}
	}
}
