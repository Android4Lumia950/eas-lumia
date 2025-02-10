// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2016 - 2018 Intel Corporation

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/leds.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include "../../../include/media/as3638.h"

#define REG_DESIGN_INFO			0x00
#define REG_VERSION_CONTROL		0x01
#define REG_CURRENT_SET1		0x02
#define REG_CURRENT_SET2		0x03
#define REG_CURRENT_SET3		0x04
#define REG_CONFIG			0x05
#define REG_LOW_VOLTAGE			0x06
#define REG_TIMER			0x07
#define REG_CONTROL			0x08
#define REG_FAULT_AND_INFO		0x09
#define REG_FLASH_CURRENT_REACHED	0x20
#define REG_PROTECT			0x21
#define REG_HIG_OVERVOLTAGE_PROTECTION	0x22

/* Register fields */
#define VERSION_SHIFT			0
#define VERSION_MASK			0x0F

#define FLASH_CURRENT_SHIFT		0
#define FLASH_CURRENT_MASK		0x1F
#define TORCH_CURRENT_SHIFT		5
#define TORCH_CURRENT_MASK		0xE0

#define CONFIG_ASSIST_STROBE_SHIFT	1
#define CONFIG_ASSIST_STROBE_MASK	0x02
#define CONFIG_EXT_TORCH_ON_SHIFT	2
#define CONFIG_EXT_TORCH_ON_MASK	0x04
#define CONFIG_COIL_PEAK_SHIFT		3
#define CONFIG_COIL_PEAK_MASK		0x18
#define CONFIG_MUTE_POLARITY_SHIFT	5
#define CONFIG_MUTE_POLARITY_MASK	0x20
#define CONFIG_STROBE_TYPE_SHIFT	6
#define CONFIG_STROBE_TYPE_MASK		0x40
#define CONFIG_STROBE_ON_SHIFT		7
#define CONFIG_STROBE_ON_MASK		0x80

#define LOW_VOLTAGE_RESET_SHIFT		0
#define LOW_VOLTAGE_RESET_MASK		0x01
#define LOW_VOLTAGE_LOWV_RED_CURR_SHIFT	1
#define LOW_VOLTAGE_LOWV_RED_CURR_MASK	0x06
#define LOW_VOLTAGE_LOWV_SEL_SHIFT	3
#define LOW_VOLTAGE_LOWV_SEL_MASK	0x38
#define LOW_VOLTAGE_LOWV_ON_SHIFT	6
#define LOW_VOLTAGE_LOWV_ON_MASK	0x40
#define LOW_VOLTAGE_BOOST_SHIFT		7
#define LOW_VOLTAGE_BOOST_MASK		0x80

#define TIMER_FLASH_TIMEOUT_SHIFT	0
#define TIMER_FLASH_TIMEOUT_MASK	0x1F
#define TIMER_MUTE_THRESHOLD_SHIFT	5
#define TIMER_MUTE_THRESHOLD_MASK	0xE0

#define CONTROL_MODE_SETTING_SHIFT	0
#define CONTROL_MODE_SETTING_MASK	0x03
#define CONTROL_OUT_ON_SHIFT		2
#define CONTROL_OUT_ON_MASK		0x04
#define CONTROL_DCDC_SKIP_ENABLE_SHIFT	4
#define CONTROL_DCDC_SKIP_ENABLE_MASK	0x10
#define CONTROL_TXMASK_RED_CURR_SHIFT	5
#define CONTROL_TXMASK_RED_CURR_MASK	0x60
#define CONTROL_TXMASK_EN_SHIFT		7
#define CONTROL_TXMASK_EN_MASK		0x80

#define FAULT_UNDER_VOLTAGE_LO_SHIFT	0
#define FAULT_UNDER_VOLTAGE_LO_MASK	0x01
#define FAULT_LOW_VOLTAGE_SHIFT		1
#define FAULT_LOW_VOLTAGE_MASK		0x02
#define INFO_TORCH_DETECTED_SHIFT	2
#define INFO_TORCH_DETECTED_MASK	0x04
#define INFO_TXMASK_EVENT_SHIFT		3
#define INFO_TXMASK_EVENT_MASK		0x08
#define FAULT_TIMEOUT_SHIFT		4
#define FAULT_TIMEOUT_MASK		0x10
#define FAULT_OVERTEMP_SHIFT		5
#define FAULT_OVERTEMP_MASK		0x20
#define FAULT_LED_SHORT_SHIFT		6
#define FAULT_LED_SHORT_MASK		0x40
#define FAULT_LED_OPEN_SHIFT		7
#define FAULT_LED_OPEN_MASK		0x80

#define HOVP_SEL_HIGH_OVP_SHIFT		0
#define HOVP_SEL_HIGH_OVP_MASK		0x01
#define HOVP_LED_STATUS_ON_SHIFT	1
#define HOVP_LED_STATUS_ON_MASK		0x02
#define HOVP_LED_STATUS_1_SHIFT		2
#define HOVP_LED_STATUS_1_MASK		0x04
#define HOVP_LED_STATUS_2_SHIFT		3
#define HOVP_LED_STATUS_2_MASK		0x08
#define HOVP_LED_STATUS_3_SHIFT		4
#define HOVP_LED_STATUS_3_MASK		0x10

#define DESIGN_INFO_FIXED_ID		0x16
#define CONFIG_COIL_PEAK_1_3_AMP	0x0
#define CONFIG_COIL_PEAK_1_6_AMP	0x1
#define CONFIG_COIL_PEAK_1_95_AMP	0x2
#define CONFIG_COIL_PEAK_2_4_AMP	0x3
#define CONTROL_RED_CURR_100_MILLI_AMP	0x0
#define CONTROL_RED_CURR_200_MILLI_AMP	0x1
#define CONTROL_RED_CURR_300_MILLI_AMP	0x2
#define CONTROL_RED_CURR_400_MILLI_AMP	0x3
#define ALL_BITS_MASK			0xFF
#define AS3638_FLASH_TOUT_MIN		0
#define AS3638_FLASH_TOUT_STEP		1    /* Each step is 4ms */
#define AS3638_FLASH_TOUT_MAX		31   /* Max timeout is 128 ms */
#define AS3638_FLASH_TOUT_DEF		0x0F /* 64 ms timeout default */

#define AS3638_FLASH_INT_STEP		50000 /* uA */
#define AS3638_FLASH_INT_MILLI_A_TO_REG(a) \
	(((a) * 1000) / AS3638_FLASH_INT_STEP)

#define AS3638_TORCH_INT_STEP		12500 /* uA */
#define AS3638_TORCH_INT_MILLI_A_TO_REG(a) \
	(((a) * 1000) / AS3638_TORCH_INT_STEP)

const int torch_led1_intensity_table[] = {    0,  9400, 14100, 18100,
					  23500, 32900, 51800, 98800}; /* uA */

enum mode_setting {
	MODE_EXT_TORCH		= 0x0,
	MODE_MEM_INTERFACE	= 0x1, /* Not supported */
	MODE_TORCH		= 0x2,
	MODE_FLASH		= 0x3,
};


enum as3638_led_mode {
    AS3638_MODE_SHUTDOWN,
    AS3638_MODE_TORCH,
    AS3638_MODE_FLASH
};

struct as3638_led {
    struct led_classdev cdev;
    struct as3638_flash *flash;
    enum as3638_led_id id;
    enum as3638_led_mode mode;
    unsigned int brightness;
};

struct as3638_flash {
    struct device *dev;
    struct regmap *regmap;
    struct mutex lock;
    struct as3638_platform_data *pdata;
    struct as3638_led leds[AS3638_LED_MAX];
    struct work_struct work;
    int current_led;
};

static const int torch_led1_intensity_table[] = { 
    0, 9400, 14100, 18100, 23500, 32900, 51800, 98800 
};

/* Helper functions that don't change (as3638_dump_registers, 
   as3638_disable_outputs, as3638_force_reduction_current, etc) */

static int as3638_set_led_mode(struct as3638_flash *flash,
                  enum as3638_led_id led_no,
                  enum as3638_led_mode mode)
{
    int rval;
    u8 mode_reg;

    switch (mode) {
    case AS3638_MODE_SHUTDOWN:
        mode_reg = MODE_EXT_TORCH;
        break;
    case AS3638_MODE_TORCH:
        mode_reg = MODE_TORCH;
        break;
    case AS3638_MODE_FLASH:
        mode_reg = MODE_FLASH;
        break;
    default:
        return -EINVAL;
    }

    rval = regmap_update_bits(flash->regmap, REG_CONTROL,
                  CONTROL_MODE_SETTING_MASK,
                  mode_reg << CONTROL_MODE_SETTING_SHIFT);
    if (rval < 0)
        return rval;

    flash->leds[led_no].mode = mode;
    return 0;
}

static void as3638_work_handler(struct work_struct *work)
{
    struct as3638_flash *flash = container_of(work, struct as3638_flash, work);
    struct as3638_led *led;
    int i, rval;

    mutex_lock(&flash->lock);

    for (i = 0; i < AS3638_LED_MAX; i++) {
        led = &flash->leds[i];
        
        if (flash->current_led != led->id) {
            rval = as3638_init_device(flash, led->id);
            if (rval < 0)
                continue;
        }

        switch (led->mode) {
        case AS3638_MODE_TORCH:
            rval = as3638_set_intensity(flash, led->id);
            if (rval < 0)
                break;
            
            rval = as3638_set_led_mode(flash, led->id, AS3638_MODE_TORCH);
            if (rval < 0)
                break;
            
            rval = regmap_update_bits(flash->regmap, REG_CONTROL,
                          CONTROL_OUT_ON_MASK,
                          1 << CONTROL_OUT_ON_SHIFT);
            break;

        case AS3638_MODE_FLASH:
            rval = as3638_set_intensity(flash, led->id);
            if (rval < 0)
                break;
            
            rval = as3638_set_led_mode(flash, led->id, AS3638_MODE_FLASH);
            if (rval < 0)
                break;
            
            rval = regmap_update_bits(flash->regmap, REG_CONTROL,
                          CONTROL_OUT_ON_MASK,
                          1 << CONTROL_OUT_ON_SHIFT);
            break;

        case AS3638_MODE_SHUTDOWN:
        default:
            rval = as3638_disable_outputs(flash);
            break;
        }
    }

    mutex_unlock(&flash->lock);
}

static void as3638_brightness_set(struct led_classdev *led_cdev,
                  enum led_brightness brightness)
{
    struct as3638_led *led = container_of(led_cdev, struct as3638_led, cdev);
    struct as3638_flash *flash = led->flash;

    led->brightness = brightness;
    
    if (brightness == LED_OFF)
        led->mode = AS3638_MODE_SHUTDOWN;
    else
        led->mode = AS3638_MODE_TORCH;

    schedule_work(&flash->work);
}

static ssize_t flash_strobe_store(struct device *dev,
                  struct device_attribute *attr,
                  const char *buf, size_t count)
{
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
    struct as3638_led *led = container_of(led_cdev, struct as3638_led, cdev);
    struct as3638_flash *flash = led->flash;
    unsigned long state;
    int ret;

    ret = kstrtoul(buf, 10, &state);
    if (ret)
        return ret;

    mutex_lock(&flash->lock);
    
    if (state) {
        led->mode = AS3638_MODE_FLASH;
        schedule_work(&flash->work);
    } else {
        led->mode = AS3638_MODE_SHUTDOWN;
        schedule_work(&flash->work);
    }

    mutex_unlock(&flash->lock);
    return count;
}

static DEVICE_ATTR(flash_strobe, 0644, NULL, flash_strobe_store);

static int as3638_init_led(struct as3638_flash *flash,
               enum as3638_led_id led_id,
               struct device_node *np)
{
    struct as3638_led *led = &flash->leds[led_id];
    const char *name;
    int ret;

    if (of_property_read_string(np, "label", &name))
        name = kasprintf(GFP_KERNEL, "as3638-led%d", led_id + 1);

    led->cdev.name = name;
    led->cdev.brightness_set = as3638_brightness_set;
    led->cdev.max_brightness = LED_FULL;
    led->cdev.flags |= LED_CORE_SUSPENDRESUME;
    led->id = led_id;
    led->flash = flash;

    ret = led_classdev_register(flash->dev, &led->cdev);
    if (ret < 0)
        return ret;

    ret = device_create_file(led->cdev.dev, &dev_attr_flash_strobe);
    if (ret < 0)
        led_classdev_unregister(&led->cdev);

    return ret;
}

static int as3638_probe(struct i2c_client *client,
            const struct i2c_device_id *id)
{
    struct as3638_platform_data *pdata = dev_get_platdata(&client->dev);
    struct device_node *np = client->dev.of_node;
    struct as3638_flash *flash;
    int i, ret;

    flash = devm_kzalloc(&client->dev, sizeof(*flash), GFP_KERNEL);
    if (!flash)
        return -ENOMEM;

    flash->dev = &client->dev;
    flash->pdata = pdata;
    mutex_init(&flash->lock);
    INIT_WORK(&flash->work, as3638_work_handler);

    flash->regmap = devm_regmap_init_i2c(client, &as3638_regmap);
    if (IS_ERR(flash->regmap))
        return PTR_ERR(flash->regmap);

    /* Hardware initialization */
    ret = as3638_hw_init(flash);  // Implement based on original init code
    if (ret)
        return ret;

    /* Initialize LEDs */
    for (i = 0; i < AS3638_LED_MAX; i++) {
        ret = as3638_init_led(flash, i, np);
        if (ret < 0)
            goto err_leds;
    }

    i2c_set_clientdata(client, flash);
    return 0;

err_leds:
    while (i--)
        led_classdev_unregister(&flash->leds[i].cdev);
    return ret;
}

static int as3638_remove(struct i2c_client *client)
{
    struct as3638_flash *flash = i2c_get_clientdata(client);
    int i;

    cancel_work_sync(&flash->work);
    
    for (i = 0; i < AS3638_LED_MAX; i++) {
        led_classdev_unregister(&flash->leds[i].cdev);
        device_remove_file(flash->leds[i].cdev.dev, &dev_attr_flash_strobe);
    }

    return 0;
}

static const struct i2c_device_id as3638_id[] = {
    { "as3638", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, as3638_id);

static struct i2c_driver as3638_i2c_driver = {
    .driver = {
        .name = "as3638",
        .owner = THIS_MODULE,
    },
    .probe = as3638_probe,
    .remove = as3638_remove,
    .id_table = as3638_id,
};

module_i2c_driver(as3638_i2c_driver);

MODULE_AUTHOR("Your Name <your.email@example.com>");
MODULE_DESCRIPTION("AS3638 LED driver");
MODULE_LICENSE("GPL v2");
