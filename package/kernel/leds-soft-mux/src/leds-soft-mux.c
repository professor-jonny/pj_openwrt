// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * LED soft multiplexer
 *
 * Lets several logical LEDs share the same physical LEDs. When more than one
 * logical LED is on, only the one with the highest priority drives the
 * physical LEDs and the others are held off, so states never mix.
 *
 * Copyright (C) 2024 Radoslav Tsvetkov <rtsvetkov@gradotech.eu>
 * Copyright (C) 2025 Jonathan Brophy <professor_jonny@hotmail.com>
 */

#include <linux/cleanup.h>
#include <linux/container_of.h>
#include <linux/leds.h>
#include <linux/math.h>
#include <linux/minmax.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/types.h>

/**
 * struct soft_mux_led - one logical LED
 * @cdev: LED class device exposed to userspace
 * @mux: controller this LED belongs to
 * @phys: physical LEDs driven by this LED
 * @num_phys: number of entries in @phys
 * @priority: higher values win over lower ones
 * @level: requested brightness, 0 when the LED is off
 * @seq: order in which the LED was last turned on, larger is newer
 */
struct soft_mux_led {
	struct led_classdev cdev;
	struct soft_mux *mux;
	struct led_classdev **phys;
	unsigned int num_phys;
	u32 priority;
	unsigned int level;
	u64 seq;
};

/**
 * struct soft_mux - one controller
 * @dev: platform device of the controller
 * @lock: protects @seq and the state of every logical LED
 * @leds: logical LEDs of this controller
 * @num_leds: number of entries in @leds
 * @phys: every physical LED used by any logical LED, without duplicates
 * @num_phys: number of entries in @phys
 * @seq: counter used to order the logical LEDs that are turned on
 */
struct soft_mux {
	struct device *dev;
	struct mutex lock;	/* protects seq and every logical LED */
	struct soft_mux_led *leds;
	unsigned int num_leds;
	struct led_classdev **phys;
	unsigned int num_phys;
	u64 seq;
};

static bool soft_mux_uses(const struct soft_mux_led *sled,
			  const struct led_classdev *phys)
{
	unsigned int i;

	for (i = 0; i < sled->num_phys; i++)
		if (sled->phys[i] == phys)
			return true;

	return false;
}

/*
 * The winner is the logical LED that is on and has the highest priority.
 * Among equal priorities the one that was turned on last wins.
 */
static const struct soft_mux_led *soft_mux_pick_winner(struct soft_mux *mux)
{
	const struct soft_mux_led *winner = NULL;
	unsigned int i;

	for (i = 0; i < mux->num_leds; i++) {
		const struct soft_mux_led *sled = &mux->leds[i];

		if (!sled->level)
			continue;

		if (!winner || sled->priority > winner->priority ||
		    (sled->priority == winner->priority && sled->seq > winner->seq))
			winner = sled;
	}

	return winner;
}

/* The winner drives its physical LEDs, every other physical LED is turned off. */
static void soft_mux_update(struct soft_mux *mux)
{
	const struct soft_mux_led *winner;
	unsigned int i;

	lockdep_assert_held(&mux->lock);

	winner = soft_mux_pick_winner(mux);

	for (i = 0; i < mux->num_phys; i++) {
		struct led_classdev *phys = mux->phys[i];
		unsigned int value = 0;

		if (winner && soft_mux_uses(winner, phys)) {
			value = DIV_ROUND_CLOSEST(winner->level * phys->max_brightness,
						  LED_FULL);
			/* Any request that is on must turn an on/off LED on. */
			value = max(value, 1U);
		}

		led_set_brightness(phys, value);
	}
}

static int soft_mux_brightness_set(struct led_classdev *cdev,
				   enum led_brightness brightness)
{
	struct soft_mux_led *sled = container_of(cdev, struct soft_mux_led, cdev);
	struct soft_mux *mux = sled->mux;

	guard(mutex)(&mux->lock);

	if (brightness && !sled->level)
		sled->seq = ++mux->seq;

	sled->level = brightness;
	soft_mux_update(mux);

	return 0;
}

static void soft_mux_put_led(void *data)
{
	led_put(data);
}

static void soft_mux_track_phys(struct soft_mux *mux, struct led_classdev *phys)
{
	unsigned int i;

	for (i = 0; i < mux->num_phys; i++)
		if (mux->phys[i] == phys)
			return;

	mux->phys[mux->num_phys++] = phys;
}

static int soft_mux_add_led(struct soft_mux *mux, struct fwnode_handle *child,
			    unsigned int index)
{
	struct led_init_data init_data = { .fwnode = child };
	struct soft_mux_led *sled = &mux->leds[index];
	struct device *dev = mux->dev;
	int count, i, ret;

	count = fwnode_property_count_u32(child, "leds");
	sled->phys = devm_kcalloc(dev, count, sizeof(*sled->phys), GFP_KERNEL);
	if (!sled->phys)
		return -ENOMEM;

	for (i = 0; i < count; i++) {
		struct led_classdev *phys;

		phys = fwnode_led_get(child, i, NULL);
		if (IS_ERR(phys))
			return dev_err_probe(dev, PTR_ERR(phys),
					     "%pfwP: cannot get LED %d\n", child, i);

		ret = devm_add_action_or_reset(dev, soft_mux_put_led, phys);
		if (ret)
			return ret;

		sled->phys[sled->num_phys++] = phys;
		soft_mux_track_phys(mux, phys);
	}

	fwnode_property_read_u32(child, "priority", &sled->priority);

	sled->mux = mux;
	sled->cdev.max_brightness = LED_FULL;
	sled->cdev.brightness_set_blocking = soft_mux_brightness_set;

	return devm_led_classdev_register_ext(dev, &sled->cdev, &init_data);
}

static int soft_mux_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	unsigned int total = 0, n = 0;
	struct soft_mux *mux;
	int ret;

	device_for_each_child_node_scoped(dev, child) {
		int count = fwnode_property_count_u32(child, "leds");

		if (count <= 0)
			return dev_err_probe(dev, count ?: -EINVAL,
					     "%pfwP: missing leds property\n", child);

		total += count;
		n++;
	}

	if (!n)
		return dev_err_probe(dev, -EINVAL, "no LEDs defined\n");

	mux = devm_kzalloc(dev, sizeof(*mux), GFP_KERNEL);
	if (!mux)
		return -ENOMEM;

	mux->dev = dev;
	mux->num_leds = n;
	mux->leds = devm_kcalloc(dev, n, sizeof(*mux->leds), GFP_KERNEL);
	mux->phys = devm_kcalloc(dev, total, sizeof(*mux->phys), GFP_KERNEL);
	if (!mux->leds || !mux->phys)
		return -ENOMEM;

	ret = devm_mutex_init(dev, &mux->lock);
	if (ret)
		return ret;

	n = 0;
	device_for_each_child_node_scoped(dev, child) {
		ret = soft_mux_add_led(mux, child, n++);
		if (ret)
			return ret;
	}

	return 0;
}

static const struct of_device_id soft_mux_of_match[] = {
	{ .compatible = "leds-soft-mux" },
	{ }
};
MODULE_DEVICE_TABLE(of, soft_mux_of_match);

static struct platform_driver soft_mux_driver = {
	.probe = soft_mux_probe,
	.driver = {
		.name = "leds-soft-mux",
		.of_match_table = soft_mux_of_match,
	},
};
module_platform_driver(soft_mux_driver);

MODULE_AUTHOR("Radoslav Tsvetkov <rtsvetkov@gradotech.eu>");
MODULE_AUTHOR("Jonathan Brophy <professor_jonny@hotmail.com>");
MODULE_DESCRIPTION("LED soft multiplexer");
MODULE_LICENSE("GPL");
