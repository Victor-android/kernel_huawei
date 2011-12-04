/*BU5D08276 sibingsong 20100420 begin*/
/*add led driver*/
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/device.h>
#include <mach/board.h>
#include <linux/uaccess.h>
	
#include <linux/workqueue.h>
#include <linux/string.h>
#include <linux/leds.h>
#include <mach/vreg.h>
#include <mach/mpp.h>
#include <linux/platform_device.h>
#include <mach/pmic.h>

#define MPP_RED "mpp7"
#define MPP_BLUE "mpp20"
#define MAX_BACKLIGHT_BRIGHTNESS 255
#define LED_OFF 0
/*<BU5D10119 sibingsong 20100518 begin*/
#define LEVEL 2
/*BU5D10119 sibingsong 20100518 end>*/
#ifdef RGB_DEBUG
#define RGB_PRINT(x...) do{ \
		printk(KERN_INFO "[RGB_LED] "x); \
	}while(0)
#else
#define RGB_PRINT(x...) do{}while(0)
#endif
static void set_red_brightness(struct led_classdev *led_cdev,
					enum led_brightness value)
{
	int ret = 0;

	RGB_PRINT("%s: value = %d\n",__func__, value);
	ret = pmic_set_low_current_led_intensity(PM_LOW_CURRENT_LED_DRV0, (!!value)? LEVEL : 0);

	if(ret)
	{
		RGB_PRINT("%s: failed\n",__func__);
		return;
	}

	RGB_PRINT("%s: success\n",__func__);
}

static void set_green_brightness(struct led_classdev *led_cdev,
					enum led_brightness value)
{
	int ret = 0;
	
	RGB_PRINT("%s: value = %d\n",__func__, value);	
	ret = pmic_set_low_current_led_intensity(PM_LOW_CURRENT_LED_DRV1, (!!value)? LEVEL : 0);
	if(ret)
	{
		RGB_PRINT("%s: failed\n",__func__);
		return;
	}

	RGB_PRINT("%s: success\n",__func__);

}

static void set_blue_brightness(struct led_classdev *led_cdev,
					enum led_brightness value)
{
	int ret = 0;
	
	RGB_PRINT("%s: value = %d\n",__func__, value);	
	ret = pmic_set_low_current_led_intensity(PM_LOW_CURRENT_LED_DRV2, (!!value)? LEVEL : 0);
	if(ret)
	{
		RGB_PRINT("%s: failed\n",__func__);
		return;
	}

	RGB_PRINT("%s: success\n",__func__);
}

static int rgb_leds_probe(struct platform_device *pdev)
{
	int rc = -ENODEV;
	
	struct led_classdev *p_rgb_data = NULL;

	p_rgb_data = kzalloc(sizeof(struct led_classdev)*3, GFP_KERNEL);
	
	if (p_rgb_data == NULL) {
		rc = -ENOMEM;
		goto err_alloc_failed;
	}

	platform_set_drvdata(pdev, p_rgb_data);
	
	p_rgb_data[0].name = "red";
	p_rgb_data[0].brightness = LED_OFF;
	p_rgb_data[0].brightness_set = set_red_brightness;

	p_rgb_data[1].name = "green";
	p_rgb_data[1].brightness = LED_OFF;
	p_rgb_data[1].brightness_set = set_green_brightness;

	p_rgb_data[2].name = "blue";
	p_rgb_data[2].brightness = LED_OFF;
	p_rgb_data[2].brightness_set = set_blue_brightness;


	/* red */
	rc = led_classdev_register(&pdev->dev, &p_rgb_data[0]);
	if (rc < 0) {
		printk(KERN_ERR "rbg red: led_classdev_register failed\n");
		goto err_led0_classdev_register_failed;
	}
	/* green */
	rc = led_classdev_register(&pdev->dev, &p_rgb_data[1]);
	if (rc < 0) {
		printk(KERN_ERR "rbg green: led_classdev_register failed\n");
		goto err_led1_classdev_register_failed;
	}
	/* blue */
	rc = led_classdev_register(&pdev->dev, &p_rgb_data[2]);
	if (rc < 0) {
		printk(KERN_ERR "rbg blue: led_classdev_register failed\n");
		goto err_led2_classdev_register_failed;
	}

	RGB_PRINT("led_classdev_register sucess\n");
	
	return 0;
err_led2_classdev_register_failed:
	led_classdev_unregister(&p_rgb_data[1]);
err_led1_classdev_register_failed:
	led_classdev_unregister(&p_rgb_data[0]);
err_led0_classdev_register_failed:
err_alloc_failed:
	kfree(p_rgb_data);
	return rc;

}
static struct platform_driver rgb_leds_driver = {
	.probe = rgb_leds_probe,
	.driver = {
		   .name = "rgb-leds",
		   },
};

int __init rgb_leds_init(void)
{
	int rc = -ENODEV;
	if (platform_driver_register(&rgb_leds_driver))
			return rc;
	return 0;
}

static void __exit rgb_leds_exit(void)
{
	platform_driver_unregister(&rgb_leds_driver);
}

module_init(rgb_leds_init);
module_exit(rgb_leds_exit);
/*BU5D08276 sibingsong 20100420 end>*/

