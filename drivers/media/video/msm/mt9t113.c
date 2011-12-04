/*< DTS2011102805854 yuguangcai 20111031 begin */

/*
 * Copyright (c) 2008-2009 QUALCOMM USA, INC.
 *
 * All source code in this file is licensed under the following license
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can find it at http://www.fsf.org
 */

#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <media/msm_camera.h>
#include <mach/gpio.h>
#include <mach/camera.h>
#include "mt9t113.h"
#include "linux/hardware_self_adapt.h"

#ifdef CONFIG_HUAWEI_HW_DEV_DCT
 #include <linux/hw_dev_dec.h>
#endif

#ifdef CONFIG_HUAWEI_SENSOR_MT9T113
 #undef CDBG
 #define CDBG(fmt, args...) printk(KERN_INFO "mt9t113.c: " fmt, ## args)
#endif

/*=============================================================
    SENSOR REGISTER DEFINES
==============================================================*/
#define MT9T113_CHIP_ID 0x4680

enum mt9t113_test_mode_t
{
    TEST_OFF,
    TEST_1,
    TEST_2,
    TEST_3
};

enum mt9t113_resolution_t
{
    QTR_SIZE,
    FULL_SIZE,
    INVALID_SIZE
};

enum mt9t113_reg_update_t
{
    /* Sensor egisters that need to be updated during initialization */
    REG_INIT,

    /* Sensor egisters that needs periodic I2C writes */
    UPDATE_PERIODIC,

    /* All the sensor Registers will be updated */
    UPDATE_ALL,

    /* Not valid update */
    UPDATE_INVALID
};

enum mt9t113_setting_t
{
    RES_PREVIEW,
    RES_CAPTURE
};

/*
 * Time in milisecs for waiting for the sensor to reset.
 */
#define MT9T113_RESET_DELAY_MSECS 66

/* for 30 fps preview */
#define MT9T113_DEFAULT_CLOCK_RATE 24000000

#define MT9T113_ARRAY_SIZE(x) (sizeof(x) / sizeof(x[0]))

/* FIXME: Changes from here */
struct mt9t113_work_t
{
    struct work_struct work;
};

struct mt9t113_ctrl_t
{
    const struct  msm_camera_sensor_info *sensordata;

    int      sensormode;
    uint32_t fps_divider; /* init to 1 * 0x00000400 */
    uint32_t pict_fps_divider; /* init to 1 * 0x00000400 */

    uint16_t curr_lens_pos;
    uint16_t init_curr_lens_pos;
    uint16_t my_reg_gain;
    uint32_t my_reg_line_count;

    enum mt9t113_resolution_t prev_res;
    enum mt9t113_resolution_t pict_res;
    enum mt9t113_resolution_t curr_res;
    enum mt9t113_test_mode_t  set_test;

    unsigned short imgaddr;
};

typedef enum
{
    CAMERA_WB_MIN_MINUS_1,
    CAMERA_WB_AUTO = 1,/* This list must match aeecamera.h */
    CAMERA_WB_CUSTOM,
    CAMERA_WB_INCANDESCENT,
    CAMERA_WB_FLUORESCENT,
    CAMERA_WB_DAYLIGHT,
    CAMERA_WB_CLOUDY_DAYLIGHT,
    CAMERA_WB_TWILIGHT,
    CAMERA_WB_SHADE,
    CAMERA_WB_MAX_PLUS_1
} config3a_wb_t;

typedef enum
{
    CAMERA_ANTIBANDING_OFF,
    CAMERA_ANTIBANDING_60HZ,
    CAMERA_ANTIBANDING_50HZ,
    CAMERA_ANTIBANDING_AUTO,
    CAMERA_MAX_ANTIBANDING,
} camera_antibanding_type;

const static char mt9t113_supported_effect[] = "none,mono,negative,sepia,aqua";
static bool CSI_CONFIG;

static struct  mt9t113_work_t *mt9t113sensorw = NULL;

static struct  i2c_client *mt9t113_client  = NULL;
static struct mt9t113_ctrl_t *mt9t113_ctrl = NULL;
static enum mt9t113_reg_update_t last_rupdate = -1;
static enum mt9t113_setting_t last_rt = -1;
static DECLARE_WAIT_QUEUE_HEAD(mt9t113_wait_queue);
DEFINE_MUTEX(mt9t113_sem);

static int mt9t113_i2c_rxdata(unsigned short saddr,
                              unsigned char *rxdata, int length)
{
    struct i2c_msg msgs[] =
    {
        {
            .addr  = saddr,
            .flags = 0,
            .len = 2,
            .buf = rxdata,
        },
        {
            .addr  = saddr,
            .flags = I2C_M_RD,
            .len = length,
            .buf = rxdata,
        },
    };

    if (i2c_transfer(mt9t113_client->adapter, msgs, 2) < 0)
    {
        CDBG("mt9t113_i2c_rxdata failed!\n");
        return -EIO;
    }

    return 0;
}

static int32_t mt9t113_i2c_read_w(unsigned short raddr, unsigned short *rdata)
{
    int32_t rc = 0;
    unsigned char buf[4];

    if (!rdata)
    {
        return -EIO;
    }

    memset(buf, 0, sizeof(buf));

    buf[0] = (raddr & 0xFF00) >> 8;
    buf[1] = (raddr & 0x00FF);

    rc = mt9t113_i2c_rxdata(mt9t113_client->addr << 2, buf, 2);
    if (rc < 0)
    {
        return rc;
    }

    *rdata = buf[0] << 8 | buf[1];

    if (rc < 0)
    {
        CDBG("mt9t113_i2c_read failed!\n");
    }

    return rc;
}

static int32_t mt9t113_i2c_txdata(unsigned short saddr,
                                  unsigned char *txdata, int length)
{
    int32_t i  = 0;
    int32_t rc = -EFAULT;
    struct i2c_msg msg[] =
    {
        {
            .addr  = saddr,
            .flags = 0,
            .len = length,
            .buf = txdata,
        },
    };

    for (i = 0; i < 3; i++)
    {
        rc = i2c_transfer(mt9t113_client->adapter, msg, 1);
        if (0 <= rc)
        {
            return 0;
        }
    }

    if (3 == i)
    {
        CDBG("mt9t113_i2c_txdata faild\n");
        return -EIO;
    }

    return 0;
}

static int32_t mt9t113_i2c_write_w(unsigned short waddr, unsigned short wdata)
{
    int32_t rc = -EFAULT;
    unsigned char buf[4];

    memset(buf, 0, sizeof(buf));
    buf[0] = (waddr & 0xFF00) >> 8;
    buf[1] = (waddr & 0x00FF);
    buf[2] = (wdata & 0xFF00) >> 8;
    buf[3] = (wdata & 0x00FF);

    rc = mt9t113_i2c_txdata(mt9t113_client->addr << 2, buf, 4);

    if (rc < 0)
    {
        CDBG("i2c_write_w failed, addr = 0x%x, val = 0x%x!\n",
             waddr, wdata);
    }

    return rc;
}

static int32_t mt9t113_i2c_write_w_table(struct mt9t113_i2c_reg_conf const *reg_conf_tbl,
                                         int                                num_of_items_in_table)
{
    int i;
    int32_t rc = -EFAULT;

    for (i = 0; i < num_of_items_in_table; i++)
    {
        rc = mt9t113_i2c_write_w(reg_conf_tbl->waddr, reg_conf_tbl->wdata);
        if (rc < 0)
        {
            break;
        }

        reg_conf_tbl++;
    }

    return rc;
}

int32_t mt9t113_set_default_focus(uint8_t af_step)
{
    CDBG("s5k4cdgx_set_default_focus:\n");

    return 0;
}

int32_t mt9t113_set_fps(struct fps_cfg    *fps)
{
    /* input is new fps in Q8 format */
    int32_t rc = 0;

    CDBG("mt9t113_set_fps\n");
    return rc;
}

int32_t mt9t113_write_exp_gain(uint16_t gain, uint32_t line)
{
    CDBG("mt9t113_write_exp_gain\n");
    return 0;
}

int32_t mt9t113_set_pict_exp_gain(uint16_t gain, uint32_t line)
{
    int32_t rc = 0;

    CDBG("mt9t113_set_pict_exp_gain\n");

    mdelay(10);

    /* camera_timed_wait(snapshot_wait*exposure_ratio); */
    return rc;
}

int32_t mt9t113_setting(enum mt9t113_reg_update_t rupdate,
                        enum mt9t113_setting_t    rt)
{
    struct msm_camera_csi_params mt9t113_csi_params;
    int32_t rc = 0;
    unsigned short chipid, capret, i = 100;

    //  mutex_lock(&mt9t113_sem);
    if ((rupdate == last_rupdate) && (rt == last_rt))
    {
        CDBG("mt9t113_setting exit\n");

        //     mutex_unlock(&mt9t113_sem);
        return rc;
    }

    CDBG("mt9t113_setting in rupdate=%d,rt=%d\n", rupdate, rt);
    switch (rupdate)
    {
    case UPDATE_PERIODIC:

        /*preview setting*/
        if (rt == RES_PREVIEW)
        {
            CDBG("mt9t113:  sensor: init preview reg.\n");
            rc = mt9t113_i2c_write_w_table(mt9t113_regs.mt9t113_preview_reg_config,
                                           mt9t113_regs.mt9t113_preview_reg_config_size);
            mdelay(200);
            if (rc)
            {
                CDBG("       write mt9t113_preview_reg_config error!!!!!!");
            }

            if (!CSI_CONFIG)
            {
                CDBG("mt9t113: init CSI  config!\n");

                //   msm_camio_vfe_clk_rate_set(192000000);
                mt9t113_csi_params.data_format = CSI_8BIT;
                mt9t113_csi_params.lane_cnt = 1;
                mt9t113_csi_params.lane_assign = 0xe4;
                mt9t113_csi_params.dpcm_scheme = 0;
                mt9t113_csi_params.settle_cnt = 0x18;
                rc = msm_camio_csi_config(&mt9t113_csi_params);
                CSI_CONFIG = 1;
            }
        }
        /*snapshot setting*/
        else
        {
            CDBG("mt9t113:  sensor: init snapshot reg.\n");
            rc = mt9t113_i2c_write_w_table(mt9t113_regs.mt9t113_snapshot_reg_config,
                                           mt9t113_regs.mt9t113_snapshot_reg_config_size);
            mdelay(100);

            while (i)
            {
                mt9t113_i2c_write_w(0x098E, 0x8401);
                rc = mt9t113_i2c_read_w(0x0990, &capret);

                CDBG("%s: capret is %d\n", __func__, capret);
                if (0x07 == capret)
                {
                    break;
                }

                i--;
                mdelay(10);
            }

            if (0 == i)
            {
                CDBG("mt9t113:  init snapshot reg failed.\n");
            }
        }

        mdelay(100);
        break;

    case REG_INIT:

        CSI_CONFIG = 0;

        /* Write init sensor register */
        rc = mt9t113_i2c_write_w_table(mt9t113_regs.mt9t113_init_reg_sensor_start,
                                       mt9t113_regs.mt9t113_init_reg_sensor_start_size);
        mdelay(100);
        if (rc)
        {
            CDBG("       write mt9t113_init_reg_sensor_start error!!!!!!");
        }

        rc = mt9t113_i2c_write_w_table(mt9t113_regs.mt9t113_init_reg_config_byd,
                                       mt9t113_regs.mt9t113_init_reg_config_byd_size);
        mdelay(200);
        if (rc)
        {
            CDBG("       write mt9t113_init_reg_config_byd error!!!!!!");
        }

        rc = mt9t113_i2c_write_w_table(mt9t113_regs.mt9t113_init_reg_config_byd_2,
                                       mt9t113_regs.mt9t113_init_reg_config_byd_2_size);
        mdelay(100);
        if (rc)
        {
            CDBG("       write mt9t113_init_reg_config_byd_2 error!!!!!!");
        }

        rc = mt9t113_i2c_write_w_table(mt9t113_regs.mt9t113_init_reg_config_byd_3,
                                       mt9t113_regs.mt9t113_init_reg_config_byd_3_size);
        mdelay(300);
        if (rc)
        {
            CDBG("       write mt9t113_init_reg_config_byd_3 error!!!!!!");
        }

        rc = mt9t113_i2c_read_w(0x0010, &chipid);
        CDBG("~~~~~~~~~~~~~~~ 0x0010 = 0x%x \n", chipid);

        rc = mt9t113_i2c_read_w(0x001E, &chipid);
        CDBG("~~~~~~~~~~~~~~~ 0x001E = 0x%x \n", chipid);
        break;

    default:
        rc = -EFAULT;
        break;
    } /* switch (rupdate) */
    if (rc == 0)
    {
        last_rupdate = rupdate;
        last_rt = rt;
    }

    //  mutex_unlock(&mt9t113_sem);
    return rc;
}

int32_t mt9t113_video_config(int mode, int res)
{
    int32_t rc;

    switch (res)
    {
    case QTR_SIZE:
        rc = mt9t113_setting(UPDATE_PERIODIC, RES_PREVIEW);
        if (rc < 0)
        {
            return rc;
        }

        CDBG("sensor configuration done!\n");
        break;

    case FULL_SIZE:
        rc = mt9t113_setting(UPDATE_PERIODIC, RES_CAPTURE);
        if (rc < 0)
        {
            return rc;
        }

        break;

    default:
        return 0;
    } /* switch */

    mt9t113_ctrl->prev_res   = res;
    mt9t113_ctrl->curr_res   = res;
    mt9t113_ctrl->sensormode = mode;

    return rc;
}

int32_t mt9t113_snapshot_config(int mode)
{
    int32_t rc = 0;

    CDBG("mt9t113_snapshot_config in\n");
    rc = mt9t113_setting(UPDATE_PERIODIC, RES_CAPTURE);
    mdelay(50);
    if (rc < 0)
    {
        return rc;
    }

    mt9t113_ctrl->curr_res = mt9t113_ctrl->pict_res;

    mt9t113_ctrl->sensormode = mode;

    return rc;
}

int32_t mt9t113_power_down(void)
{
    int32_t rc = 0;

    mdelay(5);

    return rc;
}

int32_t mt9t113_move_focus(int direction, int32_t num_steps)
{
    return 0;
}

static int mt9t113_sensor_init_done(const struct msm_camera_sensor_info *data)
{
    /* Set the sensor reset when camera is not initialization. */
    gpio_direction_output(data->sensor_reset, 0);
    gpio_free(data->sensor_reset);

    gpio_direction_output(data->sensor_pwd, 1);
    gpio_free(data->sensor_pwd);

    if (data->vreg_disable_func)
    {
        data->vreg_disable_func(data->sensor_vreg, data->vreg_num);
    }

    last_rupdate = -1;
    last_rt = -1;
    return 0;
}

static int mt9t113_probe_init_sensor(const struct msm_camera_sensor_info *data)
{
    int rc;
    unsigned short chipid;

    /* pull down power down */
    rc = gpio_request(data->sensor_pwd, "mt9t113");
    if (!rc || (rc == -EBUSY))
    {
        gpio_direction_output(data->sensor_pwd, 1);
    }
    else
    {
        goto init_probe_fail;
    }

    /* Set the sensor reset when camera is not initialization. */
    rc = gpio_request(data->sensor_reset, "mt9t113");
    if (!rc)
    {
        rc = gpio_direction_output(data->sensor_reset, 0);
    }
    else
    {
        goto init_probe_fail;
    }

    mdelay(20);
    if (data->vreg_enable_func)
    {
        rc = data->vreg_enable_func(data->sensor_vreg, data->vreg_num);
        if (rc < 0)
        {
            CDBG("%s : vreg_enable_func failed! ", __func__);
            goto init_probe_fail;
        }
    }

    {
        rc = gpio_direction_output(data->sensor_pwd, 0);
        if (rc < 0)
        {
            goto init_probe_fail;
        }

        mdelay(50);

        /*hardware reset*/
        /* Set the sensor reset when camera is not initialization. */
        rc = gpio_direction_output(data->sensor_reset, 1);
        if (rc < 0)
        {
            goto init_probe_fail;
        }

        mdelay(100);
    }

    /* Set the soft reset to reset the chip and read the chip ID when camera is not initialization. */
    /* 3. Read sensor Model ID: */
    rc = mt9t113_i2c_read_w(0x0000, &chipid);
    if (rc < 0)
    {
        CDBG("mt9t113_i2c_read_w Model_ID failed!! rc=%d", rc);
        goto init_probe_fail;
    }

    CDBG("mt9t113 chipid = 0x%x\n", chipid);

    /* 4. Compare sensor ID to MT9T113 ID: */
    if (chipid != MT9T113_CHIP_ID)
    {
        CDBG("mt9t113 Model_ID error!!");
        rc = -ENODEV;
        goto init_probe_fail;
    }

    CDBG("sensor name is %s.", data->sensor_name);

    goto init_probe_done;

init_probe_fail:
    mt9t113_sensor_init_done(data);
init_probe_done:
    return rc;
}

int mt9t113_sensor_open_init(const struct msm_camera_sensor_info *data)
{
    int32_t rc;

    mt9t113_ctrl = kzalloc(sizeof(struct mt9t113_ctrl_t), GFP_KERNEL);
    if (!mt9t113_ctrl)
    {
        CDBG("mt9t113_sensor_open_init failed!\n");
        rc = -ENOMEM;
        goto init_done;
    }

    CDBG("mt9t113_sensor_open_*****************!\n");
    mt9t113_ctrl->fps_divider = 1 * 0x00000400;
    mt9t113_ctrl->pict_fps_divider = 1 * 0x00000400;
    mt9t113_ctrl->set_test = TEST_OFF;
    mt9t113_ctrl->prev_res = QTR_SIZE;
    mt9t113_ctrl->pict_res = FULL_SIZE;

    if (data)
    {
        mt9t113_ctrl->sensordata = data;
    }

    /* enable mclk first */
    msm_camio_clk_rate_set(MT9T113_DEFAULT_CLOCK_RATE);
    mdelay(20);

    rc = mt9t113_probe_init_sensor(data);
    if (rc < 0)
    {
        CDBG("mt9t113 init failed!!!!!\n");
        goto init_fail;
    }
    else
    {
        rc = mt9t113_setting(REG_INIT, RES_PREVIEW);
        CDBG("mt9t113 init succeed!!!!! rc = %d \n", rc);
        goto init_done;
    }

    /* Don't write sensor init register at open camera. */

init_fail:
    kfree(mt9t113_ctrl);
init_done:
    return rc;
}

int mt9t113_init_client(struct i2c_client *client)
{
    /* Initialize the MSM_CAMI2C Chip */
    init_waitqueue_head(&mt9t113_wait_queue);
    return 0;
}

int32_t mt9t113_set_sensor_mode(int mode, int res)
{
    int32_t rc = 0;

    CDBG("%s : start to set sensor mode ", __func__);
    switch (mode)
    {
    case SENSOR_PREVIEW_MODE:
        CDBG("SENSOR_PREVIEW_MODE,res=%d\n", res);
        rc = mt9t113_video_config(mode, res);
        break;

    case SENSOR_SNAPSHOT_MODE:
    case SENSOR_RAW_SNAPSHOT_MODE:
        CDBG("SENSOR_SNAPSHOT_MODE\n");
        rc = mt9t113_snapshot_config(mode);
        break;

    default:
        rc = -EINVAL;
        break;
    }

    return rc;
}

static long mt9t113_set_effect(int mode, int effect)
{
    struct mt9t113_i2c_reg_conf const *reg_conf_tbl = NULL;
    int num_of_items_in_table = 0;
    long rc = 0;

    printk("mt9t113_set_effect \n ");
    switch (effect)
    {
    case CAMERA_EFFECT_OFF:
        printk("~~~~~~~~~~~CAMERA_EFFECT_OFF \n ");
        reg_conf_tbl = mt9t113_regs.mt9t113_effect_off_reg_config;
        num_of_items_in_table = mt9t113_regs.mt9t113_effect_off_reg_config_size;
        break;

    case CAMERA_EFFECT_MONO:
        printk("~~~~~~~~~~~CAMERA_EFFECT_MONO \n ");
        reg_conf_tbl = mt9t113_regs.mt9t113_effect_mono_reg_config;
        num_of_items_in_table = mt9t113_regs.mt9t113_effect_mono_reg_config_size;
        break;

    case CAMERA_EFFECT_NEGATIVE:
        reg_conf_tbl = mt9t113_regs.mt9t113_effect_negative_reg_config;
        num_of_items_in_table = mt9t113_regs.mt9t113_effect_negative_reg_config_size;
        break;

    case CAMERA_EFFECT_SOLARIZE:
        reg_conf_tbl = mt9t113_regs.mt9t113_effect_solarize_reg_config;
        num_of_items_in_table = mt9t113_regs.mt9t113_effect_solarize_reg_config_size;
        break;

    case CAMERA_EFFECT_POSTERIZE:
        reg_conf_tbl = mt9t113_regs.mt9t113_effect_posterize_reg_config;
        num_of_items_in_table = mt9t113_regs.mt9t113_effect_posterize_reg_config_size;
        break;

    case CAMERA_EFFECT_SEPIA:
        reg_conf_tbl = mt9t113_regs.mt9t113_effect_sepia_reg_config;
        num_of_items_in_table = mt9t113_regs.mt9t113_effect_sepia_reg_config_size;
        break;

    case CAMERA_EFFECT_AQUA:
        reg_conf_tbl = mt9t113_regs.mt9t113_effect_aqua_reg_config;
        num_of_items_in_table = mt9t113_regs.mt9t113_effect_aqua_reg_config_size;
        break;

    case CAMERA_EFFECT_WHITEBOARD:
        reg_conf_tbl = mt9t113_regs.mt9t113_effect_whiteboard_reg_config;
        num_of_items_in_table = mt9t113_regs.mt9t113_effect_whiteboard_reg_config_size;
        break;

    case CAMERA_EFFECT_BLACKBOARD:
        reg_conf_tbl = mt9t113_regs.mt9t113_effect_blackboard_reg_config;
        num_of_items_in_table = mt9t113_regs.mt9t113_effect_blackboard_reg_config_size;
        break;

    default:
        return 0;
    }

    rc = mt9t113_i2c_write_w_table(reg_conf_tbl, num_of_items_in_table);
    return rc;
}

static long mt9t113_set_wb(int wb)
{
    struct mt9t113_i2c_reg_conf const *reg_conf_tbl = NULL;
    int num_of_items_in_table = 0;
    long rc = 0;

    CDBG("mt9t113_set_wb FF wb:%d", wb);
    switch (wb)
    {
    case CAMERA_WB_AUTO:
        reg_conf_tbl = mt9t113_regs.mt9t113_wb_auto_reg_config;
        num_of_items_in_table = mt9t113_regs.mt9t113_wb_auto_reg_config_size;
        break;

    case CAMERA_WB_INCANDESCENT:
        reg_conf_tbl = mt9t113_regs.mt9t113_wb_a_reg_config;
        num_of_items_in_table = mt9t113_regs.mt9t113_wb_a_reg_config_size;
        break;

    case CAMERA_WB_FLUORESCENT:
        reg_conf_tbl = mt9t113_regs.mt9t113_wb_tl84_reg_config;
        num_of_items_in_table = mt9t113_regs.mt9t113_wb_tl84_reg_config_size;
        break;

    case CAMERA_WB_DAYLIGHT:
        reg_conf_tbl = mt9t113_regs.mt9t113_wb_d65_reg_config;
        num_of_items_in_table = mt9t113_regs.mt9t113_wb_d65_reg_config_size;
        break;

    case CAMERA_WB_CLOUDY_DAYLIGHT:
        reg_conf_tbl = mt9t113_regs.mt9t113_wb_d50_reg_config;
        num_of_items_in_table = mt9t113_regs.mt9t113_wb_d50_reg_config_size;
        break;

    case CAMERA_WB_CUSTOM:
        reg_conf_tbl = mt9t113_regs.mt9t113_wb_f_reg_config;
        num_of_items_in_table = mt9t113_regs.mt9t113_wb_f_reg_config_size;
        break;

    case CAMERA_WB_TWILIGHT:
        return 0;
        break;

    case CAMERA_WB_SHADE:
        return 0;
        break;

    default:
        return 0;
    }

    rc = mt9t113_i2c_write_w_table(reg_conf_tbl, num_of_items_in_table);
    return rc;
}

static long mt9t113_set_antibanding(int antibanding)
{
    long rc = 0;

    /*FF*/
    CDBG("mt9t113_set_antibanding FF antibanding:%d", antibanding);

    return rc;
}

int mt9t113_sensor_config(void __user *argp)
{
    struct sensor_cfg_data cdata;
    long rc = 0;

    if (copy_from_user(&cdata,
                       (void *)argp,
                       sizeof(struct sensor_cfg_data)))
    {
        return -EFAULT;
    }

    mutex_lock(&mt9t113_sem);
    CDBG("mt9t113_sensor_config: cfgtype = %d\n",
         cdata.cfgtype);
    switch (cdata.cfgtype)
    {
    case CFG_GET_PICT_FPS:
        break;

    case CFG_GET_PREV_L_PF:
        break;

    case CFG_GET_PREV_P_PL:
        break;

    case CFG_GET_PICT_L_PF:
        break;

    case CFG_GET_PICT_P_PL:
        break;

    case CFG_GET_PICT_MAX_EXP_LC:
        break;

    case CFG_SET_FPS:
    case CFG_SET_PICT_FPS:
        rc = mt9t113_set_fps(&(cdata.cfg.fps));
        break;

    case CFG_SET_EXP_GAIN:
        rc =
            mt9t113_write_exp_gain(
            cdata.cfg.exp_gain.gain,
            cdata.cfg.exp_gain.line);
        break;

    case CFG_SET_PICT_EXP_GAIN:
        rc =
            mt9t113_set_pict_exp_gain(
            cdata.cfg.exp_gain.gain,
            cdata.cfg.exp_gain.line);
        break;

    case CFG_SET_MODE:
        CDBG(">>>>>>>>>>>>.CFG_SET_MODE type is %d\n", cdata.cfgtype);
        rc = mt9t113_set_sensor_mode(cdata.mode,
                                     cdata.rs);
        break;

    case CFG_PWR_DOWN:
        rc = mt9t113_power_down();
        break;

    case CFG_MOVE_FOCUS:
        rc =
            mt9t113_move_focus(
            cdata.cfg.focus.dir,
            cdata.cfg.focus.steps);
        break;

    case CFG_SET_DEFAULT_FOCUS:
        rc =
            mt9t113_set_default_focus(
            cdata.cfg.focus.steps);
        break;
    case CFG_SET_EFFECT:
        CDBG(">>>>>>>>>>>>>>>>CFG_SET_EFFECT type is %d\n", cdata.cfgtype);
        rc = mt9t113_set_effect(cdata.mode,
                                cdata.cfg.effect);
        break;

    case CFG_SET_WB:
        rc = mt9t113_set_wb(cdata.cfg.effect);
        break;

    case CFG_SET_ANTIBANDING:
        if (0)
        {
            rc = mt9t113_set_antibanding(cdata.cfg.effect);
        }

        break;

    case CFG_MAX:

        break;

    default:
        rc = -EFAULT;
        break;
    }

    mutex_unlock(&mt9t113_sem);

    return rc;
}

int mt9t113_sensor_release(void)
{
    int rc = -EBADF;

    mutex_lock(&mt9t113_sem);

    mt9t113_power_down();

    mt9t113_sensor_init_done(mt9t113_ctrl->sensordata);

    msleep(150);
    kfree(mt9t113_ctrl);

    mutex_unlock(&mt9t113_sem);
    CDBG("mt9t113_release completed!\n");
    return rc;
}

static int mt9t113_i2c_probe(struct i2c_client *         client,
                             const struct i2c_device_id *id)
{
    int rc = 0;

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
    {
        rc = -ENOTSUPP;
        goto probe_failure;
    }

    mt9t113sensorw =
        kzalloc(sizeof(struct mt9t113_work_t), GFP_KERNEL);

    if (!mt9t113sensorw)
    {
        rc = -ENOMEM;
        goto probe_failure;
    }

    i2c_set_clientdata(client, mt9t113sensorw);
    mt9t113_init_client(client);
    mt9t113_client = client;

    //mt9t113_client->addr = mt9t113_client->addr >> 1;
    mdelay(50);

    CDBG("i2c probe ok\n");
    return 0;

probe_failure:
    kfree(mt9t113sensorw);
    mt9t113sensorw = NULL;
    pr_err("i2c probe failure %d\n", rc);
    return rc;
}

static const struct i2c_device_id mt9t113_i2c_id[] =
{
    { "mt9t113", 0},
    { }
};

static struct i2c_driver mt9t113_i2c_driver =
{
    .id_table = mt9t113_i2c_id,
    .probe    = mt9t113_i2c_probe,
    .remove   = __exit_p(mt9t113_i2c_remove),
    .driver   = {
        .name = "mt9t113",
    },
};

static int mt9t113_sensor_probe(const struct msm_camera_sensor_info *info,
                                struct msm_sensor_ctrl *             s)
{
    /* We expect this driver to match with the i2c device registered
     * in the board file immediately. */
    int rc = i2c_add_driver(&mt9t113_i2c_driver);

    if ((rc < 0) || (mt9t113_client == NULL))
    {
        rc = -ENOTSUPP;
        goto probe_done;
    }

    /* enable mclk first */
    msm_camio_clk_rate_set(MT9T113_DEFAULT_CLOCK_RATE);
    mdelay(20);

    rc = mt9t113_probe_init_sensor(info);
    if (rc < 0)
    {
        CDBG("mt9t113 probe failed!!!!\n");
        i2c_del_driver(&mt9t113_i2c_driver);
        goto probe_done;
    }
    else
    {
        CDBG("mt9t113 probe succeed!!!!\n");
    }

#ifdef CONFIG_HUAWEI_HW_DEV_DCT
    /* detect current device successful, set the flag as present */
    set_hw_dev_flag(DEV_I2C_CAMERA_MAIN);
#endif

    s->s_init = mt9t113_sensor_open_init;
    s->s_release = mt9t113_sensor_release;
    s->s_config = mt9t113_sensor_config;
    s->s_mount_angle = 0;
    mt9t113_sensor_init_done(info);

    /* For go to sleep mode, follow the datasheet */
    msleep(150);

    //set_camera_support(true);
probe_done:
    return rc;
}

static int __mt9t113_probe(struct platform_device *pdev)
{
    return msm_camera_drv_start(pdev, mt9t113_sensor_probe);
}

static struct platform_driver msm_camera_driver =
{
    .probe     = __mt9t113_probe,
    .driver    = {
        .name  = "msm_camera_mt9t113",
        .owner = THIS_MODULE,
    },
};

static int __init mt9t113_init(void)
{
    return platform_driver_register(&msm_camera_driver);
}

module_init(mt9t113_init);

/* DTS2011102805854 yuguangcai 20111031 end > */
