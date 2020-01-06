/*
 * max30100.c - Support for MAX30100 heart rate and pulse oximeter sensor
 *
 * Copyright (C) 2015 Matt Ranostay <mranostay@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/kfifo_buf.h>

#define MAX30100_REGMAP_NAME	"max30100_regmap"
#define MAX30102_DRV_NAME	"max30102"

#define MAX30102_REG_MODE_CONFIG				0x09
#define MAX30102_REG_MODE_CONFIG_MODE_MASK		0x7
#define MAX30102_REG_MODE_CONFIG_MODE_HR_EN		BIT(1)
#define MAX30102_REG_MODE_CONFIG_MODE_SPO2_EN	(BIT(0) | BIT(1))
#define MAX30102_REG_MODE_CONFIG_MULTI_LED_EN	(BIT(0) | BIT(1) |BIT(2))
#define MAX30102_REG_MODE_CONFIG_RESET			BIT(6)
#define MAX30102_REG_MODE_CONFIG_PWR			BIT(7)

#define MAX30102_REG_SPO2_CONFIG		0x0A
#define MAX30102_REG_SPO2_CONFIG_50HZ		0x00
#define MAX30102_REG_SPO2_CONFIG_100HZ		BIT(2)
#define MAX30102_REG_SPO2_CONFIG_HI_RES_EN	BIT(6)
#define MAX30102_REG_SPO2_CONFIG_411		0x3

#define MAX30102_REG_TEMP_INTEGER		0x1F
#define MAX30102_REG_TEMP_FRACTION		0x20
#define MAX30102_REG_INT_1_STATUS		0x00
#define MAX30102_REG_INT_2_STATUS		0x01

#define  MAX30102_REG_FIFO_CONFIG			0x08
#define	 MAX30102_REG_FIFO_CONFIG_SMP_AVE   0x00
#define  MAX30102_REG_FIFO_ROLLOVER_EN		BIT(4)
#define  MAX30102_REG_FIFO_A_FULL			0xF

#define MAX30102_REG_FIFO_WR_PTR		0x04
#define MAX30102_REG_FIFO_OVR_CTR		0x05
#define MAX30102_REG_FIFO_RD_PTR		0x06
#define MAX30102_REG_FIFO_DATA			0x07

#define MAX30102_REG_LED_PULSE_AMP_1	0x0C
#define MAX30102_REG_LED_PULSE_AMP_2	0x0D
#define MAX30102_REG_PROXIMITY_MODE_LED_PULSE	0x10

#define AMP_12_5 0x3F
#define AMP_6_4 0x1F

#define MAX30102_REG_INT_1 0x02
#define MAX30102_REG_INT_1_ENABLE_MASK (0xF0 | BIT(0))
#define MAX30102_REG_INT_1_NEW_DATA_EN BIT(6)
#define MAX30102_REG_INT_1_FIFO_FULL_EN BIT(7)

#define MAX30102_REG_TEMP_CONFIG 0x21
#define MAX30102_REG_MODE_CONFIG_TEMP_EN  BIT(0)


#define  MAX30102_REG_PART_ID			0xFF

struct max30100_data {
	struct i2c_client *client;
	struct iio_dev *indio_dev;
	struct mutex lock;
	struct regmap *regmap;

	u32 red;
	u32 ir;
	unsigned char buff[6]; /* Two 24-bit samples */
};

static bool max30100_is_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MAX30102_REG_INT_1_STATUS:
	case MAX30102_REG_INT_2_STATUS:
	case MAX30102_REG_MODE_CONFIG:
	case MAX30102_REG_FIFO_WR_PTR:
	case MAX30102_REG_FIFO_OVR_CTR:
	case MAX30102_REG_FIFO_RD_PTR:
	case MAX30102_REG_FIFO_DATA:
	case MAX30102_REG_TEMP_INTEGER:
	case MAX30102_REG_TEMP_FRACTION:
		return true;
	default:
		return false;
	}
}

static bool max30102_is_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MAX30102_REG_MODE_CONFIG:
	case MAX30102_REG_LED_PULSE_AMP_1:
	case MAX30102_REG_LED_PULSE_AMP_2:
	case MAX30102_REG_PROXIMITY_MODE_LED_PULSE:
	case MAX30102_REG_SPO2_CONFIG:
	case MAX30102_REG_FIFO_CONFIG:
	case MAX30102_REG_INT_1:
	case MAX30102_REG_TEMP_CONFIG:
		return true;
	default:
		return false;
	};
}

static bool max30102_is_readable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MAX30102_REG_PART_ID:
		return true;
	default:
		return false;
	};
}



static const struct regmap_config max30100_regmap_config = {
//	.name = MAX30100_REGMAP_NAME,

	.reg_bits = 8,
	.val_bits = 8,

	.max_register = MAX30102_REG_PART_ID,
	.cache_type = REGCACHE_RBTREE,

	.writeable_reg = max30102_is_writeable_reg,
//	.readable_reg = max30102_is_readable_reg,
	.volatile_reg = max30100_is_volatile_reg,
};

static const unsigned int max30100_led_current_mapping[] = {
	4400, 7600, 11000, 14200, 17400,
	20800, 24000, 27100, 30600, 33800,
	37000, 40200, 43600, 46800, 50000
};

static const unsigned long max30100_scan_masks[] = {0x3, 0};

static const struct iio_chan_spec max30100_channels[] = {
	{
		.type = IIO_INTENSITY,
		.channel2 = IIO_MOD_LIGHT_IR,
		.modified = 1,

		.scan_index = 0,
		.scan_type = {
			.sign = 'u',
			.realbits = 24,
			.storagebits = 32,
			.endianness = IIO_BE,
		},
	},
	{
		.type = IIO_INTENSITY,
		.channel2 = IIO_MOD_LIGHT_RED,
		.modified = 1,

		.scan_index = 1,
		.scan_type = {
			.sign = 'u',
			.realbits = 24,
			.storagebits = 32,
			.endianness = IIO_BE,
		},
	},
	{
		.type = IIO_TEMP,
		.info_mask_separate =
			BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE),
		.scan_index = -1,
	},
};

static int max30100_set_powermode(struct max30100_data *data, bool state)
{
	return regmap_update_bits(data->regmap, MAX30102_REG_MODE_CONFIG,
				  MAX30102_REG_MODE_CONFIG_PWR,
				  state ? 0 : MAX30102_REG_MODE_CONFIG_PWR);
}

static int max30100_clear_fifo(struct max30100_data *data)
{
	int ret;

	ret = regmap_write(data->regmap, MAX30102_REG_FIFO_WR_PTR, 0);
	if (ret)
		return ret;

	ret = regmap_write(data->regmap, MAX30102_REG_FIFO_OVR_CTR, 0);
	if (ret)
		return ret;

	return regmap_write(data->regmap, MAX30102_REG_FIFO_RD_PTR, 0);
}

static int max30100_buffer_postenable(struct iio_dev *indio_dev)
{
	struct max30100_data *data = iio_priv(indio_dev);
	int ret;

	ret = max30100_set_powermode(data, true);
	if (ret)
		return ret;

	return max30100_clear_fifo(data);
}

static int max30100_buffer_predisable(struct iio_dev *indio_dev)
{
	struct max30100_data *data = iio_priv(indio_dev);

	return max30100_set_powermode(data, false);
}

static const struct iio_buffer_setup_ops max30100_buffer_setup_ops = {
	.postenable = max30100_buffer_postenable,
	.predisable = max30100_buffer_predisable,
};

#define	MAX30102_REG_INT_1_STATUS 0x00
#define	MAX30102_REG_FIFO_WRITE_PTR 0x04
#define	MAX30102_REG_FIFO_READ_PTR 0x06

//#if 0
static inline int max30100_fifo_count(struct max30100_data *data)
{
	unsigned int val;
	unsigned int write, read;
	int ret;

	ret = regmap_read(data->regmap, MAX30102_REG_INT_1_STATUS, &val);
	if (ret)
		return ret;
	printk(KERN_NOTICE "int status = %x ", val);
	ret = regmap_read(data->regmap, MAX30102_REG_FIFO_WRITE_PTR, &write);
	if (ret)
		return ret;

	ret = regmap_read(data->regmap, MAX30102_REG_FIFO_READ_PTR, &read);
	if (ret)
		return ret;

	/* FIFO is almost full */
//	if (val & MAX30100_REG_INT_STATUS_FIFO_RDY)
//		return MAX30100_REG_FIFO_DATA_ENTRY_COUNT - 1;

	return (write - read);
}
//#endif

#define MAX30102_REG_FIFO_DATA_ENTRY_LEN 6

static int max30100_read_measurement(struct max30100_data *data)
{
	int ret;

	ret = i2c_smbus_read_i2c_block_data(data->client,
					    MAX30102_REG_FIFO_DATA,
					    MAX30102_REG_FIFO_DATA_ENTRY_LEN,
					    (u8 *) data->buff);
	return (ret == MAX30102_REG_FIFO_DATA_ENTRY_LEN) ? 0 : ret;
}

static irqreturn_t max30100_interrupt_handler(int irq, void *private)
{
	struct iio_dev *indio_dev = private;
	struct max30100_data *data = iio_priv(indio_dev);
	int ret, cnt = 1;

	mutex_lock(&data->lock);
	cnt = max30100_fifo_count(data);

//	while (cnt || (cnt = max30100_fifo_count(data) > 0)) {
	while (cnt) {
		ret = max30100_read_measurement(data);
//		if (ret){
//		printk(KERN_NOTICE "Failed to read data\n");
//			break;
//		}

		data->red = data->buff[2] | ((data->buff[1])<<8) | ((data->buff[0])<<16);
		data->ir = data->buff[5] | ((data->buff[4])<<8) | ((data->buff[3])<<16);

		printk(KERN_NOTICE "RED [%d] IR [%d] \n", data->red, data->ir);

		iio_push_to_buffers(data->indio_dev, (u8*) data->red);
		iio_push_to_buffers(data->indio_dev, (u8*) data->ir);
		cnt--;
		if (cnt < 0)
			break;
	}

	mutex_unlock(&data->lock);

	return IRQ_HANDLED;
}

static int max30100_get_current_idx(unsigned int val, int *reg)
{
	int idx;

	/* LED turned off */
	if (val == 0) {
		*reg = 0;
		return 0;
	}

	for (idx = 0; idx < ARRAY_SIZE(max30100_led_current_mapping); idx++) {
		if (max30100_led_current_mapping[idx] == val) {
			*reg = idx + 1;
			return 0;
		}
	}

	return -EINVAL;
}


static int max30100_led_init(struct max30100_data *data)
{
	struct device *dev = &data->client->dev;
	int ret;

	ret = regmap_write(data->regmap, MAX30102_REG_LED_PULSE_AMP_1,
			AMP_12_5);
	if (ret){
		dev_err(dev, "Failed to enable LED 1 \n");
		return ret;
	}

	ret = regmap_write(data->regmap, MAX30102_REG_LED_PULSE_AMP_2,
			AMP_12_5);
	if (ret){
		dev_err(dev, "Failed to enable LED 2 \n");
		return ret;
	}

	ret = regmap_write(data->regmap, MAX30102_REG_PROXIMITY_MODE_LED_PULSE,
			AMP_6_4);
	if (ret){
		dev_err(dev, "Failed to enable proximity LED \n");
		return ret;
	}

	return ret;
}

static int max30100_chip_init(struct max30100_data *data)
{
	int ret;

	/* setup LED current settings */
	ret = max30100_led_init(data);
	if (ret){
		dev_err(&data->client->dev, "LED init failed \n");
		return ret;
	}

	/* enable hi-res SPO2 readings at 100Hz */
	ret = regmap_write(data->regmap, MAX30102_REG_SPO2_CONFIG,
				 MAX30102_REG_SPO2_CONFIG_HI_RES_EN |
				 MAX30102_REG_SPO2_CONFIG_411 |
				 MAX30102_REG_SPO2_CONFIG_100HZ);
//				 MAX30102_REG_SPO2_CONFIG_50HZ);
	if (ret){
		dev_err(&data->client->dev, "SPO2 config failed \n");
		return ret;
	}

	/* enable SPO2 mode */
	ret = regmap_update_bits(data->regmap, MAX30102_REG_MODE_CONFIG,
				 MAX30102_REG_MODE_CONFIG_MODE_MASK,
				 MAX30102_REG_MODE_CONFIG_MODE_SPO2_EN);
	if (ret){
		dev_err(&data->client->dev, "Failed to enable SPO2 \n");
		return ret;
	}


	/* config FIFO  */

	ret = regmap_write(data->regmap, MAX30102_REG_FIFO_CONFIG,
				 MAX30102_REG_FIFO_CONFIG_SMP_AVE|
				 MAX30102_REG_FIFO_ROLLOVER_EN|
				 MAX30102_REG_FIFO_A_FULL);
	if (ret){
		dev_err(&data->client->dev, "Failed to configure FIFO \n");
		return ret;
	}

	/* enable FIFO interrupt */
//	ret = regmap_update_bits(data->regmap, MAX30102_REG_INT_1,
//				 MAX30102_REG_INT_1_ENABLE_MASK,
//				 MAX30102_REG_INT_1_NEW_DATA_EN);
	regmap_update_bits(data->regmap, MAX30102_REG_INT_1,
				 MAX30102_REG_INT_1_ENABLE_MASK,
				 MAX30102_REG_INT_1_FIFO_FULL_EN);
	if (ret){
		dev_err(&data->client->dev, "Failed to enable interrupt \n");
		return ret;
	}

	return 0;
}

static int max30102_read_temp(struct max30100_data *data, int *val)
{
	int ret;
	unsigned int reg;

	ret = regmap_read(data->regmap, MAX30102_REG_TEMP_INTEGER, &reg);
	if (ret < 0)
		return ret;

	*val = reg << 4;

	ret = regmap_read(data->regmap, MAX30102_REG_TEMP_FRACTION, &reg);
	if (ret < 0)
		return ret;

	*val |= reg & 0xf;
	*val = sign_extend32(*val, 11);

	return 0;

}

static int max30102_get_temp(struct max30100_data *data, int *val)
{
	int ret;

	/* start acquisition */
	ret = regmap_update_bits(data->regmap, MAX30102_REG_TEMP_CONFIG,
				 MAX30102_REG_MODE_CONFIG_TEMP_EN,
				 MAX30102_REG_MODE_CONFIG_TEMP_EN);
	if (ret)
		return ret;

	usleep_range(35000, 50000);

	return max30102_read_temp(data, val);
}

static int max30100_read_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int *val, int *val2, long mask)
{
	struct max30100_data *data = iio_priv(indio_dev);
	int ret = -EINVAL;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		/*
		 * Temperature reading can only be acquired while engine
		 * is running
		 */
		mutex_lock(&indio_dev->mlock);

		if (iio_buffer_enabled(indio_dev))
			ret = -EAGAIN;
		else {
			ret = max30102_get_temp(data, val);
			if (!ret)
				ret = IIO_VAL_INT;

		}

		mutex_unlock(&indio_dev->mlock);
		break;
	case IIO_CHAN_INFO_SCALE:
		*val = 1;  /* 0.0625 */
		*val2 = 16;
		ret = IIO_VAL_FRACTIONAL;
		break;
	}

	return ret;
}

static const struct iio_info max30100_info = {
	.driver_module = THIS_MODULE,
	.read_raw = max30100_read_raw,
};

/*
static inline void iio_device_attach_buffer(struct iio_dev *indio_dev,
		struct iio_buffer *buffer)
{
	indio_dev->buffer = buffer;
}
*/

#define GPIO_ANY_GPIO_DESC           "Some gpio pin description"
#define IRQ_GPIO        17
static short int irq_gpio = 0;

static int max30102_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct max30100_data *data;
	struct iio_buffer *buffer;
	struct iio_dev *indio_dev;
	int ret;
	unsigned int chip_id;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
//	indio_dev = iio_device_alloc( sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	buffer = devm_iio_kfifo_allocate(&client->dev);
//	buffer = iio_kfifo_allocate(indio_dev);
//	buffer = iio_kfifo_allocate();
	if (!buffer)
		return -ENOMEM;

	iio_device_attach_buffer(indio_dev, buffer);

	indio_dev->name = MAX30102_DRV_NAME;
	indio_dev->channels = max30100_channels;
	indio_dev->info = &max30100_info;
	indio_dev->num_channels = ARRAY_SIZE(max30100_channels);
	indio_dev->available_scan_masks = max30100_scan_masks;
//	indio_dev->modes = (INDIO_BUFFER_SOFTWARE | INDIO_DIRECT_MODE);
	indio_dev->modes = (INDIO_DIRECT_MODE);
	indio_dev->setup_ops = &max30100_buffer_setup_ops;

	data = iio_priv(indio_dev);
	data->indio_dev = indio_dev;
	data->client = client;

	mutex_init(&data->lock);
	i2c_set_clientdata(client, indio_dev);

	printk("Client address: %x\n", client->addr);
	data->regmap = devm_regmap_init_i2c(client, &max30100_regmap_config);
	if (IS_ERR(data->regmap)) {
		dev_err(&client->dev, "regmap initialization failed.\n");
		return PTR_ERR(data->regmap);
	}

	ret = regmap_read(data->regmap, MAX30102_REG_PART_ID, &chip_id);
	if (ret)
		return ret;
	printk("MAX30102_REG_PART_ID: %x\n", chip_id);

//	ret = max30100_set_powermode(data, false);
	ret = max30100_set_powermode(data, true);
	if (ret){
		return ret;
	}

	if (gpio_request(IRQ_GPIO, GPIO_ANY_GPIO_DESC)) {
		printk("GPIO request faiure: %s\n", GPIO_ANY_GPIO_DESC);
		return -EINVAL;
	}

	if ( (irq_gpio = gpio_to_irq(IRQ_GPIO)) < 0 ) {
		gpio_free(IRQ_GPIO);
		printk("GPIO to IRQ mapping faiure %s\n", GPIO_ANY_GPIO_DESC);
		return -EINVAL;
	}

	printk(KERN_NOTICE "Mapped int %d\n", irq_gpio);

	client->irq = irq_gpio;
	ret = max30100_chip_init(data);
	if (ret){
		gpio_free(IRQ_GPIO);
		return ret;
	}

	if (client->irq <= 0) {
		dev_err(&client->dev, "no valid irq defined\n");
		return -EINVAL;
	}


	ret = devm_request_threaded_irq(&client->dev, client->irq,
					NULL, max30100_interrupt_handler,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					"max30102_irq", indio_dev);

/*
	ret = request_threaded_irq(client->irq, NULL,
					max30100_interrupt_handler,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					"max30102_irq", indio_dev);
*/
//	ret = request_irq(client->irq, max30100_interrupt_handler,
//				IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
//				GPIO_ANY_GPIO_DESC,"max30102_irq");
	if (ret) {
		gpio_free(IRQ_GPIO);
		dev_err(&client->dev, "request irq (%d) failed\n", client->irq);
		return ret;
	}

	return iio_device_register(indio_dev);
}

static int max30102_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct max30100_data *data = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);
	max30100_set_powermode(data, false);
//	free_irq(irq_gpio, indio_dev);
	gpio_free(IRQ_GPIO);

	return 0;
}

static const struct i2c_device_id max30102_id[] = {
	{ "max30102", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, max30102_id);

static const struct of_device_id max30102_dt_ids[] = {
	{ .compatible = "maxim,max30102" },
	{ }
};
MODULE_DEVICE_TABLE(of, max30102_dt_ids);

static struct i2c_driver max30102_driver = {
	.driver = {
		.name	= MAX30102_DRV_NAME,
		.of_match_table	= of_match_ptr(max30102_dt_ids),
	},
	.probe		= max30102_probe,
	.remove		= max30102_remove,
	.id_table	= max30102_id,
};
module_i2c_driver(max30102_driver);

MODULE_AUTHOR("navin patidar <navinx.patidar@intel.com>");
MODULE_DESCRIPTION("MAX30102 heart rate and pulse oximeter sensor");
MODULE_LICENSE("GPL");
