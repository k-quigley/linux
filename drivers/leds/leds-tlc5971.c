/*
 * Copyright 2017 K Quigley Ltd,  Kevin Quigley <kevin@kquigley.co.uk>
 *
 * Based on:
 *  - leds-dac124s085.c by Guennadi Liakhovetski <lg@denx.de>
 * and leds-tlc5940.c by Jordan Yelloz <jordan@yelloz.me>
 *
 * This file is subject to the terms and conditions of version 2 of
 * the GNU General Public License. See the file LICENSE in the main
 * directory of this archive for more details.
 *
 * LED driver for the TLC5971 SPI LED Controller
 */

#include <linux/leds.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include <linux/spi/spi.h>
#include <linux/of.h>
#include <linux/of_device.h>

#define DRIVER_NAME "leds-tlc5971"
//#define DEBUG


#define TLC5971_BITS_PER_WORD 8
#define TLC5971_MAX_SPEED_HZ 1000000

#define TLC5971_GSCLK_SPEED_HZ  250000
#define TLC5971_GSCLK_PERIOD_NS (1000000000 / TLC5971_GSCLK_SPEED_HZ)

#define TLC5971_BLANK_PERIOD_NS (4096 * TLC5971_GSCLK_PERIOD_NS)


//Command Bits
#define TLC5971_CMD_SIZE_BITS 6

// Functional Control Bits
#define TLC5971_FC_SIZE_BITS 5

// Group Brightness Control
#define TLC5971_BC_CHANNEL_WIDTH 7
#define TLC5971_BC_CHANNELS	 3
#define TLC5971_BC_SIZE_BITS  ((TLC5971_BC_CHANNELS) * (TLC5971_BC_CHANNEL_WIDTH))
#define TLC5971_BC_SIZE (TLC5971_BC_SIZE_BITS >> 3)

#define TLC5971_BC_DEFAULT 0x7F

//<--- 32bit boundary -->>

// GreyScale Control
#define TLC5971_MAX_LEDS   12
#define TLC5971_GS_CHANNEL_WIDTH 16
#define TLC5971_FB_SIZE_BITS ((TLC5971_MAX_LEDS) * TLC5971_GS_CHANNEL_WIDTH)
#define TLC5971_FB_SIZE (TLC5971_FB_SIZE_BITS >> 3)

#define FB_OFFSET_BITS(__led)   ( \
	  (TLC5971_FB_SIZE_BITS) - \
	  (TLC5971_GS_CHANNEL_WIDTH * ((__led) + 1)) \
								)
#define FB_OFFSET(__led)        (FB_OFFSET_BITS(__led) >> 3)

#define TLC5971_SPI_BITS ((TLC5971_CMD_SIZE_BITS) + (TLC5971_FC_SIZE_BITS) \
			  + (TLC5971_BC_SIZE_BITS) + ( TLC5971_FB_SIZE_BITS) )

#define TLC5971_SPI_SIZE (TLC5971_SPI_BITS >> 3)

#define TLC5971_TXLEN	(1*(TLC5971_SPI_SIZE))

#define TLC5971_HEADER_BITS ((TLC5971_CMD_SIZE_BITS) + (TLC5971_FC_SIZE_BITS) \
			  + (TLC5971_BC_SIZE_BITS) )

#define TLC5971_HEADER_SIZE (TLC5971_HEADER_BITS >> 3)

// Write command 
#define TLC5971_WRTCMD  (0x25)

// function control defines (function control is the 5 MSb's of the data package)
#define TLC5971_BLANK   (1<<0)
#define TLC5971_DSPRPT  (1<<1)
#define TLC5971_TMGRST  (1<<2)
#define TLC5971_EXTGCK  (1<<3)
#define TLC5971_OUTTMG  (1<<4)


struct TLC5971_led {
	struct led_classdev ldev;
	int                 id;
	int                 brightness;
	const char         *name;
	struct TLC5971     *tlc;

	spinlock_t          lock;
};

struct TLC5971_bc {
//	struct led_classdev ldev;
	int                 id;
	u8                  brightness;
	const char         *name;
	struct TLC5971     *tlc;
};

struct TLC5971 {
	struct TLC5971_led  leds[TLC5971_MAX_LEDS];
	u8		    bc[	TLC5971_BC_CHANNELS];
	u8                  fb[	TLC5971_FB_SIZE];
	u8		    fc;		// Function Control (5bits)
	bool                new_gs_data;

	struct mutex	    mlock;
	struct hrtimer      timer;

	struct work_struct  work;
	struct spi_device  *spi;
};

static enum hrtimer_restart
TLC5971_timer_func(struct hrtimer *const timer)
{

	struct TLC5971 *const tlc = container_of(timer, struct TLC5971, timer);

	hrtimer_forward_now(timer, ktime_set(0, TLC5971_BLANK_PERIOD_NS));

	if (tlc->new_gs_data) {
		schedule_work(&tlc->work);
	}

	return HRTIMER_RESTART;

}

static void
TLC5971_update_bc(struct TLC5971 *const tlc)
{

	u8 *const bc = &(tlc->bc[0]);
	int id;

	for (id = 0; id < TLC5971_BC_CHANNELS; id++) {
//		struct TLC5971_led *const led = &(tlc->leds[id]);
//		ToDo Add some control handles for BC compensation
		const u8 bright_control = TLC5971_BC_DEFAULT;
		const u8 brightness = bright_control & 0x7f;
		bc[id] = brightness & 0x7f;
	}

}
static void
TLC5971_work(struct work_struct *const work)
{

	struct TLC5971 *const tlc = container_of(work, struct TLC5971, work);
	struct spi_device *const spi = tlc->spi;
	struct device *const dev = &spi->dev;

	struct spi_transfer tx;
	struct spi_message msg;
	__u8 *message_tx;
	__u8 *message_rx;
	int ret = 0;
	int id = 0;

	message_tx = kmalloc(TLC5971_SPI_SIZE, GFP_KERNEL);
	message_rx = kmalloc(TLC5971_SPI_SIZE, GFP_KERNEL);

	memset(message_tx, 0x00, TLC5971_SPI_SIZE);
	memset(message_rx, 0x00, TLC5971_SPI_SIZE);
	memset(&tx, 0x00, sizeof(tx));
	spi_message_init(&msg);
	tx.rx_buf = message_rx;
	tx.tx_buf = message_tx;
	tx.len = TLC5971_TXLEN;
	spi_message_add_tail(&tx, &msg);

	if (mutex_lock_interruptible(&tlc->mlock)) {
		dev_err(dev, "unable to set lock");
		return;
	}

	TLC5971_update_bc(tlc);

	// Build SPI Stream (for one device only)

	tlc->fc = TLC5971_BLANK;	// To auto-power off

        for (id = 0; id < TLC5971_MAX_LEDS; id++) {

                struct TLC5971_led *const led = &(tlc->leds[id]);

                const u16 brightness = led->brightness & 0xffff;
                const u8 offset = (2*id+4);
		if (led->brightness > 0) {
			tlc->fc = TLC5971_DSPRPT; // If one led's on - enable device
		}
                message_tx[offset] = brightness >> 8;
                message_tx[offset + 1] = brightness & 0xff;
        }


	// Knife & Fork header as we're bit-shifting specifically for this device
	// We do this last - so we knwo what to set for fc

	// Command into MSB of Header[0]
	message_tx[0] = (TLC5971_WRTCMD << 2) & 0xFC;

	// 2 MSB bits of FC into remaining bits of of Header[0]
	message_tx[0] |= (( tlc->fc & 0x1f) >> 3);

	// Remaining 3 bits of FC into MSB of Header[1]
	message_tx[1] = (tlc->fc << 5) & 0xE0;

	// 5 BC2 MSBits into remaining bits of Header[1]
	message_tx[1] |= (tlc->bc[2] >> 2) & 0x1F;

	// Remaining 2 BC2 bits intto MSB of Header[2]
	message_tx[2] = (tlc->bc[2] << 6 ) & 0xC0;

	// 6 BC1 MSBits into remaining bits of Header[2]
	message_tx[2] |= ((tlc->bc[1] >> 1) & 0x3f);

	// Remaining 1 BC1 bits intto MSB of Header[3]
	message_tx[3] = (tlc->bc[1] << 7 ) & 0x80;

	// 7 BC0 MSBits into remaining bits of Header[3]
	message_tx[3] |= ((tlc->bc[0]) & 0x7f);

	ret = spi_sync(spi, &msg);
	if (ret) {
		dev_err(dev, "spi sync error");
		// Jump out of one device write fails
//		break;
	}

#ifdef DEBUG
	printk("\ntx->");
	for (ret = 0; ret < TLC5971_SPI_SIZE; ret++) {
		printk("%02d:0x%02x ", ret,message_tx[ret]);
	}
	printk("\nrx->");
	for (ret = 0; ret < TLC5971_SPI_SIZE; ret++) {
		printk("0x%02x ", message_rx[ret]);
	}
#endif /* DEBUG */


	tlc->new_gs_data = 0;
	kfree(message_tx);
	kfree(message_rx);

	mutex_unlock(&tlc->mlock);

}

static void
TLC5971_set_brightness(struct led_classdev *const ldev,
					   const enum led_brightness brightness)
{

	struct TLC5971_led *const led = container_of(
	  ldev,
	  struct TLC5971_led,
	  ldev
	);

	led->tlc->new_gs_data = 1;

	spin_lock(&led->lock);
	{
		led->brightness = brightness;
	}
	spin_unlock(&led->lock);

}

static int TLC5971_probe(struct spi_device *const spi)
{
	struct device *const dev = &(spi->dev);
	struct device_node *const np = dev->of_node;
	struct TLC5971 *const tlc = devm_kzalloc(
	  &spi->dev,
	  sizeof(struct TLC5971),
	  GFP_KERNEL
	);
	struct hrtimer *const timer = &tlc->timer;
	struct work_struct *const work = &tlc->work;
	struct TLC5971_led *led;
	struct device_node *child;
	int i, ret;

	if (!tlc) {
		return -ENOMEM;
	}

	mutex_init(&tlc->mlock);

	spi->bits_per_word = TLC5971_BITS_PER_WORD;
	spi->max_speed_hz = TLC5971_MAX_SPEED_HZ;

	hrtimer_init(timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	timer->function = TLC5971_timer_func;
	hrtimer_start(timer, ktime_set(1, 0), HRTIMER_MODE_REL);

	INIT_WORK(work, TLC5971_work);

	tlc->new_gs_data = 1;

	tlc->spi = spi;
	// Disable outputs (initially)
	tlc->fc = TLC5971_BLANK;

	i = 0;
	for_each_child_of_node(np, child) {
		led = &(tlc->leds[i]);
		led->name = of_get_property(child, "label", NULL) ? : child->name;
		led->id = i;
		led->tlc = tlc;
		led->brightness = LED_OFF;
		spin_lock_init(&led->lock);
		led->ldev.name = led->name;
		led->ldev.brightness = LED_OFF;
		led->ldev.max_brightness = 0xffff;
		led->ldev.brightness_set = TLC5971_set_brightness;
		ret = led_classdev_register(dev, &led->ldev);
		if (ret < 0)
			goto eledcr;
		i++;
	}

	// Initialize BC channel
	for (i = 0; i < TLC5971_BC_CHANNELS; i++) {
		tlc->bc[i] = TLC5971_BC_DEFAULT;
	}

	spi_set_drvdata(spi, tlc);

	return 0;

eledcr:
	dev_err(dev, "failed to set up child LED #%d: %d\n", i, ret);
	while (i--)
		led_classdev_unregister(&tlc->leds[i].ldev);

	return ret;
}

static int
TLC5971_remove(struct spi_device *const spi)
{
	struct TLC5971 *const tlc = spi_get_drvdata(spi);
	struct hrtimer *const timer = &tlc->timer;
	struct work_struct *const work = &tlc->work;
	struct TLC5971_led *led;
	int i;

	hrtimer_cancel(timer);
	cancel_work_sync(work);

	for (i = 0; i < TLC5971_MAX_LEDS; i++) {
		led = &tlc->leds[i];
		led_classdev_unregister(&led->ldev);
	}

	return 0;
}

static const struct of_device_id TLC5971_dt_ids[] = {
	{
		.compatible = "linux,TLC5971",
	},
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, TLC5971_dt_ids);

static struct spi_driver TLC5971_driver = {
	.probe = TLC5971_probe,
	.remove = TLC5971_remove,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = of_match_ptr(TLC5971_dt_ids),
	},
};

module_spi_driver(TLC5971_driver);

MODULE_AUTHOR("Kevin Quigley <kevin@kquigley.co.uk");
MODULE_DESCRIPTION("TLC5971 SPI LED driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("spi:" DRIVER_NAME);

