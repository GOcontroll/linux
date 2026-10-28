// SPDX-License-Identifier: GPL-2.0-only
/*
 * Backlight driver for Maxim MAX25014
 *
 * Copyright (C) 2024 GOcontroll B.V.
 *      Maud Spierings <maudspierings@gocontroll.com>
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_data/max25014.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#define MAX25014_ISET_DEFAULT_100 11U
#define DEFAULT_BL_NAME "lcd-backlight"
#define MAX_BRIGHTNESS (100U)
#define MIN_BRIGHTNESS (0U)
#define TON_MAX (130720U) /* @153Hz */
#define TON_STEP (1307U) /* @153Hz */
#define TON_MIN (0U)

#define MAX25014_DEV_ID         (0x00U)
#define MAX25014_REV_ID         (0x01U)
#define MAX25014_ISET           (0x02U)
#define MAX25014_IMODE          (0x03U)
#define MAX25014_TON1H          (0x04U)
#define MAX25014_TON1L          (0x05U)
#define MAX25014_TON2H          (0x06U)
#define MAX25014_TON2L          (0x07U)
#define MAX25014_TON3H          (0x08U)
#define MAX25014_TON3L          (0x09U)
#define MAX25014_TON4H          (0x0AU)
#define MAX25014_TON4L          (0x0BU)
#define MAX25014_TON_1_4_LSB    (0x0CU)
#define MAX25014_SETTING        (0x12U)
#define MAX25014_DISABLE        (0x13U)
#define MAX25014_BSTMON         (0x14U)
#define MAX25014_IOUT1          (0x15U)
#define MAX25014_IOUT2          (0x16U)
#define MAX25014_IOUT3          (0x17U)
#define MAX25014_IOUT4          (0x18U)
#define MAX25014_OPEN           (0x1BU)
#define MAX25014_SHORT_GND      (0x1CU)
#define MAX25014_SHORT_LED      (0x1DU)
#define MAX25014_MASK           (0x1EU)
#define MAX25014_DIAG           (0x1FU)

#define MAX25014_REGISTERS_MAX  (0x1FU)

#define MAX25014_DIM_MODE       (0x04U)
#define MAX25014_ENABLE         BIT(5)
#define MAX25014_PSEN           BIT(4)
#define MAX25014_HW_RST         BIT(2)
#define MAX25014_FPWM           GENMASK(6,4)

/*
 * The chip sits on the display-side supply. After a short power interruption
 * that rail can still be rising, or the bus can be held, when the driver
 * probes; both clear within seconds. Keep retrying for this long before
 * giving up, instead of failing the whole display chain on the first error.
 */
#define MAX25014_PROBE_TIMEOUT_MS	10000U
#define MAX25014_PROBE_RETRY_MS		100U

/*
 * A dip on the display-side supply can reset the chip to its power-on
 * defaults while the SoC keeps running, which turns the backlight off for
 * good. Poll ISET to notice and reprogram it. 0 disables the poll.
 */
static unsigned int watchdog_ms = 1000;
module_param(watchdog_ms, uint, 0644);
MODULE_PARM_DESC(watchdog_ms,
		 "Interval in ms to check the chip for a power-on reset (0 = off)");

struct max25014;

struct max25014 {
	const char *chipname;
	uint32_t chip_id;
	struct i2c_client *client;
	struct backlight_device *bl;
	struct device *dev;
	struct regmap *regmap;
	struct max25014_platform_data *pdata;
	struct gpio_desc *enable;
	struct regulator *vin; /* regulator for boost converter Vin rail */
	struct mutex lock; /* serialises chip programming */
	struct delayed_work watchdog;
};

static const struct regmap_config max25014_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = MAX25014_REGISTERS_MAX,
};

/**
 * @brief get the bit mask for the DISABLE register.
 * 
 * @param strings the led string configuration array.
 * @return uint8_t bits to set in the register.
 */
static uint8_t strings_mask(uint32_t *strings)
{
	uint8_t res = 0, i;
	for (i = 0; i < 4; i++) {
		if (strings[i] == 0)
			res |= 1 << i;
	}
	return res;
}

/**
 * @brief control the brightness with i2c registers
 * 
 * @param regmap trivial
 * @param brt brightness
 * @return int 
 */
static int max25014_register_control(struct regmap *regmap, uint32_t brt)
{
	uint32_t reg = TON_STEP * brt;
	int ret;
	/*
	 * 18 bit number lowest, 2 bits in first register,
	 * next lowest 8 in the L register, next 8 in the H register
	 */
	ret = regmap_write(regmap, MAX25014_TON_1_4_LSB, reg & 0b00000011);
	if (ret != 0)
		return ret;
	ret = regmap_write(regmap, MAX25014_TON1L, (reg >> 2) & 0b11111111);
	if (ret != 0)
		return ret;
	return regmap_write(regmap, MAX25014_TON1H, (reg >> 10) & 0b11111111);
}

static int max25014_check_errors(struct max25014 *maxim)
{
	uint8_t i;
	int ret;
	uint32_t val;

	ret = regmap_read(maxim->regmap, MAX25014_OPEN, &val);
	if (ret != 0)
		goto i2c_err;
	if (val > 0) {
		dev_err(maxim->dev, "Open led strings detected on:\n");
		for (i = 0; i < 4; i++) {
			if (val & 1 << i) {
				dev_err(maxim->dev, "string %d\n", i + 1);
			}
		}
		return -EIO;
	}

	ret = regmap_read(maxim->regmap, MAX25014_SHORT_GND, &val);
	if (ret != 0)
		goto i2c_err;
	if (val > 0) {
		dev_err(maxim->dev, "Short to ground detected on:\n");
		for (i = 0; i < 4; i++) {
			if (val & 1 << i) {
				dev_err(maxim->dev, "string %d\n", i + 1);
			}
		}
		return -EIO;
	}

	ret = regmap_read(maxim->regmap, MAX25014_SHORT_LED, &val);
	if (ret != 0)
		goto i2c_err;
	if (val > 0) {
		dev_err(maxim->dev, "Shorted led detected on:\n");
		for (i = 0; i < 4; i++) {
			if (val & 1 << i) {
				dev_err(maxim->dev, "string %d\n", i + 1);
			}
		}
		return -EIO;
	}

	ret = regmap_read(maxim->regmap, MAX25014_DIAG, &val);
	if (ret != 0)
		goto i2c_err;
	/* 
	 * The HW_RST bit always starts at 1 after power up.
	 * It is reset on first read, does not indicate an error.
	 */
	if (val > 0 && val != MAX25014_HW_RST) {
		if (val & 0b1)
			dev_err(maxim->dev, "Overtemperature shutdown\n");
		if (val & 0b10)
			dev_warn(maxim->dev,
				 "Chip is getting too hot (>125C)\n");
		if (val & 0b1000)
			dev_err(maxim->dev, "Boost converter overvoltage\n");
		if (val & 0b10000)
			dev_err(maxim->dev, "Boost converter undervoltage\n");
		if (val & 0b100000)
			dev_err(maxim->dev, "IREF out of range\n");
		return -EIO;
	}
	return 0;
i2c_err:
	/* Reported by the caller, which may still retry. */
	return ret;
}

static uint32_t max25014_iset(struct max25014 *maxim)
{
	return maxim->pdata->iset | MAX25014_ENABLE | MAX25014_PSEN;
}

/*
 * Errors that mean "the bus or the chip is not ready yet" rather than "this
 * chip is broken": arbitration lost or a held bus, and a NAK from a chip whose
 * supply is still rising.
 */
static bool max25014_is_transient(int err)
{
	return err == -EAGAIN || err == -ETIMEDOUT || err == -EBUSY ||
	       err == -ENXIO || err == -EREMOTEIO;
}

/*
 * 1. disable unused strings
 * 2. set dim mode
 * 3. set initial brightness
 * 4. set setting register
 * 5. enable the backlight
 *
 * Returns the raw I2C error so the caller can decide whether to retry.
 */
static int max25014_configure(struct max25014 *maxim, uint32_t brightness)
{
	int ret;
	uint32_t val;

	ret = regmap_write(maxim->regmap, MAX25014_DISABLE,
			   strings_mask(maxim->pdata->strings));
	if (ret != 0)
		return ret;

	ret = regmap_write(maxim->regmap, MAX25014_IMODE, MAX25014_DIM_MODE);
	if (ret != 0)
		return ret;

	ret = max25014_register_control(maxim->regmap, brightness);
	if (ret != 0)
		return ret;

	ret = regmap_read(maxim->regmap, MAX25014_SETTING, &val);
	if (ret != 0)
		return ret;

	ret = regmap_write(
		maxim->regmap, MAX25014_SETTING,
		val & ~MAX25014_FPWM);
	if (ret != 0)
		return ret;

	return regmap_write(maxim->regmap, MAX25014_ISET, max25014_iset(maxim));
}

/*
 * Writes issued while the chip's supply is still rising can be lost while a
 * later read succeeds, which leaves a probed but unprogrammed chip. Read the
 * configuration back and report a mismatch as transient so it is retried.
 */
static int max25014_verify(struct max25014 *maxim)
{
	uint32_t val;
	int ret;

	ret = regmap_read(maxim->regmap, MAX25014_DISABLE, &val);
	if (ret != 0)
		return ret;
	if ((val & 0x0f) != strings_mask(maxim->pdata->strings))
		return -EAGAIN;

	ret = regmap_read(maxim->regmap, MAX25014_IMODE, &val);
	if (ret != 0)
		return ret;
	if (val != MAX25014_DIM_MODE)
		return -EAGAIN;

	ret = regmap_read(maxim->regmap, MAX25014_SETTING, &val);
	if (ret != 0)
		return ret;
	if (val & MAX25014_FPWM)
		return -EAGAIN;

	ret = regmap_read(maxim->regmap, MAX25014_ISET, &val);
	if (ret != 0)
		return ret;
	if (val != max25014_iset(maxim))
		return -EAGAIN;

	return 0;
}

static int max25014_program(struct max25014 *maxim, uint32_t brightness)
{
	int ret;

	ret = max25014_configure(maxim, brightness);
	if (ret != 0)
		return ret;
	return max25014_verify(maxim);
}

/*
 * Bring the chip up at probe time, retrying transient failures for up to
 * MAX25014_PROBE_TIMEOUT_MS. A failure here is terminal for the driver (and,
 * through the panel's backlight link, for the whole display pipeline), so a
 * bus that is busy for a moment must not count as one.
 */
static int max25014_init_chip(struct max25014 *maxim)
{
	unsigned long deadline = jiffies +
		msecs_to_jiffies(MAX25014_PROBE_TIMEOUT_MS);
	unsigned int attempts = 0;
	int ret;

	for (;;) {
		attempts++;
		ret = max25014_check_errors(maxim);
		if (ret == 0)
			ret = max25014_program(maxim,
					       maxim->pdata->initial_brightness);
		if (ret == 0 || !max25014_is_transient(ret) ||
		    time_after(jiffies, deadline))
			break;
		msleep(MAX25014_PROBE_RETRY_MS);
	}

	if (ret == 0 && attempts > 1)
		dev_info(maxim->dev, "configured after %u attempts\n", attempts);
	else if (ret != 0)
		dev_err(maxim->dev, "could not configure after %u attempts: %d\n",
			attempts, ret);
	return ret;
}

static int max25014_update_status(struct backlight_device *bl_dev)
{
	struct max25014 *maxim = bl_get_data(bl_dev);
	int ret;

	if (bl_dev->props.state & BL_CORE_SUSPENDED)
		bl_dev->props.brightness = 0;

	mutex_lock(&maxim->lock);
	/* Honours bl_power and fb blanking, so panel disable turns it off. */
	ret = max25014_register_control(maxim->regmap,
					backlight_get_brightness(bl_dev));
	mutex_unlock(&maxim->lock);
	return ret;
}

static void max25014_schedule_watchdog(struct max25014 *maxim)
{
	unsigned int ms = READ_ONCE(watchdog_ms);

	if (ms)
		schedule_delayed_work(&maxim->watchdog, msecs_to_jiffies(ms));
}

/*
 * The chip has no interrupt, so a power-on reset while the SoC keeps running
 * is only visible in its registers: ISET drops back to its default and the
 * ENABLE bit with it. Reprogram the chip with the current brightness. A read
 * error is not acted on; the bus may simply be busy, and the next poll looks
 * again.
 */
static void max25014_watchdog_work(struct work_struct *work)
{
	struct max25014 *maxim = container_of(to_delayed_work(work),
					      struct max25014, watchdog);
	uint32_t val;
	int ret;

	if (maxim->bl->props.state & BL_CORE_SUSPENDED)
		goto out;

	mutex_lock(&maxim->lock);
	ret = regmap_read(maxim->regmap, MAX25014_ISET, &val);
	if (ret == 0 && val != max25014_iset(maxim)) {
		dev_warn(maxim->dev,
			 "chip lost its configuration (ISET 0x%02x, expected 0x%02x), reprogramming\n",
			 val, max25014_iset(maxim));
		ret = max25014_program(maxim,
				       backlight_get_brightness(maxim->bl));
		if (ret)
			dev_err(maxim->dev, "reprogramming failed: %d\n", ret);
		else
			dev_info(maxim->dev, "backlight restored\n");
	}
	mutex_unlock(&maxim->lock);
out:
	max25014_schedule_watchdog(maxim);
}

static const struct backlight_ops max25014_bl_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status = max25014_update_status,
};

static int max25014_backlight_register(struct max25014 *maxim)
{
	struct backlight_device *bl;
	struct backlight_properties props;
	struct max25014_platform_data *pdata = maxim->pdata;
	const char *name = pdata->name ?: DEFAULT_BL_NAME;

	memset(&props, 0, sizeof(props));
	props.type = BACKLIGHT_PLATFORM;
	props.max_brightness = MAX_BRIGHTNESS;

	if (pdata->initial_brightness > props.max_brightness)
		pdata->initial_brightness = props.max_brightness;

	props.brightness = pdata->initial_brightness;

	bl = devm_backlight_device_register(maxim->dev, name, maxim->dev, maxim,
					    &max25014_bl_ops, &props);
	if (IS_ERR(bl))
		return PTR_ERR(bl);

	maxim->bl = bl;

	return 0;
}

static ssize_t max25014_get_chip_id(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct max25014 *maxim = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%s\n", maxim->chipname);
}

static DEVICE_ATTR(chip_id, S_IRUGO, max25014_get_chip_id, NULL);

static struct attribute *max25014_attributes[] = {
	&dev_attr_chip_id.attr,
	NULL,
};

static const struct attribute_group max25014_attr_group = {
	.attrs = max25014_attributes,
};

#ifdef CONFIG_OF
static int max25014_parse_dt(struct max25014 *maxim)
{
	struct device *dev = maxim->dev;
	struct device_node *node = dev->of_node;
	struct max25014_platform_data *pdata;

	int res;

	if (!node) {
		return dev_err_probe(dev, -EINVAL, "no platform data\n");
	}

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	res = of_property_count_u32_elems(node, "maxim,strings");
	if (res == 4) {
		of_property_read_u32_array(node, "maxim,strings", pdata->strings, 4);
	} else {
		dev_err_probe(dev, -EINVAL, "strings property not correctly defined\n");
	}

	of_property_read_string(node, "bl-name", &pdata->name);
	pdata->initial_brightness = 50U;
	of_property_read_u32(node, "default-brightness", &pdata->initial_brightness);
	pdata->iset = MAX25014_ISET_DEFAULT_100;
	of_property_read_u32(node, "maxim,iset", &pdata->iset);

	if (pdata->iset < 0 || pdata->iset > 15) {
		return dev_err_probe(dev, -EINVAL,
			"Invalid iset, should be a value from 0-15, entered was %d\n",
			pdata->iset);
	}

	if (pdata->initial_brightness < 0 || pdata->initial_brightness > 100) {
		return dev_err_probe(dev, -EINVAL,
			"Invalid initial brightness, should be a value from 0-100, entered was %d\n",
			pdata->initial_brightness);
	}

	maxim->pdata = pdata;

	return 0;
}
#else
static int max25014_parse_dt(struct max25014 *maxim)
{
	dev_err(maxim->dev,
		"CONFIG_OF not configured, unable to parse devicetree");
	return -EINVAL;
}
#endif

static int max25014_probe(struct i2c_client *cl)
{
	const struct i2c_device_id *id = i2c_client_get_device_id(cl);
	struct max25014 *maxim;
	int ret;

	maxim = devm_kzalloc(&cl->dev, sizeof(struct max25014), GFP_KERNEL);
	if (!maxim)
		return -ENOMEM;
	
	maxim->client = cl;
	maxim->dev = &cl->dev;
	maxim->chipname = id->name;
	maxim->chip_id = id->driver_data;
	maxim->pdata = dev_get_platdata(&cl->dev);

	if (!maxim->pdata) {
		ret = max25014_parse_dt(maxim);
		if (ret < 0)
			return ret;
	}

	maxim->vin = devm_regulator_get(maxim->dev, "power");
	if (IS_ERR(maxim->vin)) {
		if (PTR_ERR(maxim->vin) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		maxim->vin = NULL;
	}

	if (maxim->vin) {
		ret = regulator_enable(maxim->vin);
		if (ret < 0) {
			return dev_err_probe(maxim->dev, ret,
					     "failed to enable Vin\n");
		}
	}

	maxim->enable =
		devm_gpiod_get_optional(maxim->dev, "enable", GPIOD_ASIS);
	if (IS_ERR(maxim->enable)) {
		ret = PTR_ERR(maxim->enable);
		dev_err(maxim->dev, "failed to get enable gpio: %d\n", ret);
		goto disable_vin;
	}

	if (maxim->enable) {
		gpiod_set_value_cansleep(maxim->enable, 1);

		/* Datasheet states startup time is max 2 ms */
		usleep_range(2000, 2500);
	}

	maxim->regmap = devm_regmap_init_i2c(cl, &max25014_regmap_config);
	if (IS_ERR(maxim->regmap)) {
		ret = PTR_ERR(maxim->regmap);
		dev_err(maxim->dev, "failed to initialize the i2c regmap: %d\n", ret);
		goto disable_full;
	}

	i2c_set_clientdata(cl, maxim);
	mutex_init(&maxim->lock);
	INIT_DELAYED_WORK(&maxim->watchdog, max25014_watchdog_work);

	ret = max25014_init_chip(maxim);
	if (ret) /* already reported */
		goto disable_full;

	ret = max25014_backlight_register(maxim);
	if (ret) {
		dev_err(maxim->dev, "failed to register backlight. err: %d\n",
			ret);
		goto disable_full;
	}

	ret = sysfs_create_group(&maxim->dev->kobj, &max25014_attr_group);
	if (ret) {
		dev_err(maxim->dev, "failed to register sysfs. err: %d\n", ret);
		goto disable_full;
	}

	dev_info(maxim->dev, "max25014 probed.\n");
	max25014_schedule_watchdog(maxim);

	return 0;

disable_full:
	if (maxim->enable)
		gpiod_set_value_cansleep(maxim->enable, 0);
disable_vin:
	if (maxim->vin)
		regulator_disable(maxim->vin);
	return ret;
}

static void max25014_remove(struct i2c_client *cl)
{
	struct max25014 *maxim = i2c_get_clientdata(cl);

	cancel_delayed_work_sync(&maxim->watchdog);
	maxim->bl->props.brightness = 0;
	max25014_update_status(maxim->bl);
	if (maxim->enable)
		gpiod_set_value_cansleep(maxim->enable, 0);
	if (maxim->vin)
		regulator_disable(maxim->vin);
	sysfs_remove_group(&maxim->dev->kobj, &max25014_attr_group);
}

static const struct of_device_id max25014_dt_ids[] = {
	{
		.compatible = "maxim,max25014",
	},
	{}
};
MODULE_DEVICE_TABLE(of, max25014_dt_ids);

static const struct i2c_device_id max25014_ids[] = { { "max25014", 0U }, {} };
MODULE_DEVICE_TABLE(i2c, max25014_ids);

static struct i2c_driver max25014_driver = {
	.driver = {
		.name = "max25014",
		.of_match_table = of_match_ptr(max25014_dt_ids),
		/* Probe may wait out a busy bus; do not hold up the boot. */
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe = max25014_probe,
	.remove = max25014_remove,
	.id_table = max25014_ids,
};
module_i2c_driver(max25014_driver);

MODULE_DESCRIPTION("Maxim max25014 backlight driver");
MODULE_AUTHOR("Maud Spierings <maudspierings@gocontroll.com");
MODULE_LICENSE("GPL");
