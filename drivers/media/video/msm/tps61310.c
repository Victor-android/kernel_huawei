/*< DTS2011091402372   yuguangcai 20110914 begin */
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/earlysuspend.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <mach/camera.h>
/* < DTS2011102201769 zhangyu 20111025 begin */
#include <mach/pmic.h>
#include <linux/gpio.h>
#include <linux/mfd/pmic8058.h>
/* DTS2011102201769 zhangyu 20111025 end > */

#include <mach/gpio.h>
#include <asm/mach-types.h>

#define TAG "TPS61310"
#define TPS61310_I2C_NAME "tps61310"
#define CMD_RD_REG 1
#define CMD_WR_REG 3
/* < DTS2011102201769 zhangyu 20111025 begin */
#define PMIC_GPIO_FLASH_EN 22    /*  PMIC GPIO NUMBER 23   */
#define PMIC_GPIO_TORCH_FLASH 23    /*  PMIC GPIO NUMBER 24   */
/* DTS2011102201769 zhangyu 20111025 end > */

struct tps61310_info_t
{
    char *name;
};

static struct i2c_client *g_client;

/*i2c write func for tps61310*/
static int tps61310_i2c_write(unsigned char reg, unsigned char value )
{
    unsigned char buf[2];
    struct i2c_msg msg[] =
    {
        {
            .addr  = g_client->addr,
            .flags = 0,
            .len = 2,
            .buf = buf,
        },
    };

    buf[0] = reg;
    buf[1] = value;

    if (i2c_transfer(g_client->adapter, msg, 1) < 0)
    {
        printk("%s : transfer error\n", __FUNCTION__ );
        return -EIO;
    }

    return 0;
}

#if 0
/*i2c read func for tps61310*/
static int tps61310_i2c_read(unsigned char reg, unsigned char *value)
{
    unsigned char buf;

    struct i2c_msg msgs[] =
    {
        {
            .addr  = g_client->addr,
            .flags = 0,
            .len = 1,
            .buf = &buf,
        },
        {
            .addr  = g_client->addr,
            .flags = I2C_M_RD,
            .len = 1,
            .buf = &buf,
        },
    };

    buf = reg;

    if (i2c_transfer(g_client->adapter, msgs, 2) < 0)
    {
        printk("%s : transfer error\n", __FUNCTION__ );
        return -EIO;
    }

    *value = buf;

    return 0;
}

#endif

int tps61310_set_flash(unsigned led_state)
{
    int rc = 0;

    printk("tps61310_set_flash: led_state = %d\n", led_state);
    /* < DTS2011102201769 zhangyu 20111025 begin */
    /* Change tps61310 work mode. */
    switch (led_state)
    {
    case MSM_CAMERA_LED_LOW:
        pm8058_gpio_set_value(PMIC_GPIO_FLASH_EN, 1);
        pm8058_gpio_set_value(PMIC_GPIO_TORCH_FLASH, 1);
        mdelay(1);

        tps61310_i2c_write( 0x00, 0x0A );
        tps61310_i2c_write( 0x05, 0x6F );
        tps61310_i2c_write( 0x01, 0x40 );
        tps61310_i2c_write( 0x02, 0x40 );

        break;

    case MSM_CAMERA_LED_HIGH:
        pm8058_gpio_set_value(PMIC_GPIO_FLASH_EN, 1);
        pm8058_gpio_set_value(PMIC_GPIO_TORCH_FLASH, 1);
        mdelay(1);

        tps61310_i2c_write( 0x03, 0xE7 );
        tps61310_i2c_write( 0x05, 0x6F );
        tps61310_i2c_write( 0x01, 0x84 );
        tps61310_i2c_write( 0x02, 0x84 );

        break;

    case MSM_CAMERA_LED_TORCH:
        pm8058_gpio_set_value(PMIC_GPIO_FLASH_EN, 1);
        pm8058_gpio_set_value(PMIC_GPIO_TORCH_FLASH, 1);
        mdelay(1);

        tps61310_i2c_write( 0x00, 0x01 );
        tps61310_i2c_write( 0x05, 0x6F );
        tps61310_i2c_write( 0x01, 0x40 );
        tps61310_i2c_write( 0x02, 0x40 );

        break;

    case MSM_CAMERA_LED_OFF:
        tps61310_i2c_write( 0x01, 0x00 );
        tps61310_i2c_write( 0x02, 0x00 );
        tps61310_i2c_write( 0x00, 0x80 );

        mdelay(1);
        pm8058_gpio_set_value(PMIC_GPIO_FLASH_EN, 0);
        pm8058_gpio_set_value(PMIC_GPIO_TORCH_FLASH, 0);

        break;

    default:
        tps61310_i2c_write( 0x00, 0x80 );

        mdelay(1);
        pm8058_gpio_set_value(PMIC_GPIO_FLASH_EN, 0);
        pm8058_gpio_set_value(PMIC_GPIO_TORCH_FLASH, 0);
        break;
    }
    /* DTS2011102201769 zhangyu 20111025 end > */

    return rc;
}

/*probe func for tps61310*/
static int tps61310_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int rc;

    printk("Enter %s()\n", __FUNCTION__ );

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
    {
        printk("Enter %s() error !!!\n", __FUNCTION__ );
        rc = -ENOTSUPP;
    }

    g_client = client;

    return 0;
}

/*exit func for tps61310*/
static int tps61310_i2c_remove(struct i2c_client *client)
{
    /*nothing to do*/
    g_client = NULL;
    return 0;
}

static const struct i2c_device_id tps61310_id[] =
{
    {TPS61310_I2C_NAME, 0},
    { }
};

static struct i2c_driver tps61310_i2c_driver =
{
    .probe    = tps61310_i2c_probe,
    .remove   = tps61310_i2c_remove,
    .id_table = tps61310_id,
    .driver   = {
        .name = TPS61310_I2C_NAME,
    },
};

static int __devinit tps61310_init(void)
{
    printk("%s %s :\n", TAG, __FUNCTION__);
    return i2c_add_driver(&tps61310_i2c_driver);
}

static void __exit tps61310_exit(void)
{
    return i2c_del_driver(&tps61310_i2c_driver);
}

module_init(tps61310_init);
module_exit(tps61310_exit);

MODULE_DESCRIPTION("TPS61310 Driver");
MODULE_LICENSE("GPL");
/* DTS2011091402372   yuguangcai 20110914 end > */
