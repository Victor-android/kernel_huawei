/* kernel/power/earlysuspend.c
 *
 * Copyright (C) 2005-2008 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/earlysuspend.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/rtc.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>

#include "power.h"

enum {
	DEBUG_USER_STATE = 1U << 0,
	DEBUG_SUSPEND = 1U << 2,
};
/* <DTS2010092703937 hufeng 20100927 begin */
#ifdef CONFIG_HUAWEI_KERNEL
/*<BU5D00025, jialin, 20091223, add log information, begin */
static int debug_mask = DEBUG_USER_STATE | DEBUG_SUSPEND;
/*BU5D00025, jialin, 20091223, add log information, end> */
#else
static int debug_mask = DEBUG_USER_STATE;
/* DTS2010092703937 hufeng 20100927 end> */
#endif
module_param_named(debug_mask, debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

/*< DTS2011092606663 pengyu 20110926 begin */
#ifdef CONFIG_HUAWEI_KERNEL
/* Global variables definition */
/*< DTS2011110802942 pengyu 20111108 begin */
/* Cancel DTS2011092606663, because it will affect the performance of music with wma format */
/* DTS2011110802942 pengyu 20111108 end >*/
/*< DTS2011102601746 pengyu 20111026 begin */
#define UP_THRESHOLD_MAX_SIZE           20
#define DEF_FREQUENCY_UP_THRESHOLD      (80)
#define MICRO_FREQUENCY_UP_THRESHOLD    (95)
/* DTS2011102601746 pengyu 20111026 end >*/

/*< DTS2011110802942 pengyu 20111108 begin */
/* Cancel DTS2011092606663, because it will affect the performance of music with wma format */
/* DTS2011110802942 pengyu 20111108 end >*/
/*< DTS2011102601746 pengyu 20111026 begin */
static const char up_threshold_path[] = "/sys/devices/system/cpu/cpu0/cpufreq/ondemand/up_threshold";
/* DTS2011102601746 pengyu 20111026 end >*/
/*< DTS2011110802942 pengyu 20111108 begin */
/* Cancel DTS2011092606663, because it will affect the performance of music with wma format */
/* DTS2011110802942 pengyu 20111108 end >*/
#endif
/* DTS2011092606663 pengyu 20110926 end >*/

static DEFINE_MUTEX(early_suspend_lock);
static LIST_HEAD(early_suspend_handlers);
static void early_suspend(struct work_struct *work);
static void late_resume(struct work_struct *work);
static DECLARE_WORK(early_suspend_work, early_suspend);
static DECLARE_WORK(late_resume_work, late_resume);
static DEFINE_SPINLOCK(state_lock);
enum {
	SUSPEND_REQUESTED = 0x1,
	SUSPENDED = 0x2,
	SUSPEND_REQUESTED_AND_SUSPENDED = SUSPEND_REQUESTED | SUSPENDED,
};
static int state;

void register_early_suspend(struct early_suspend *handler)
{
	struct list_head *pos;

	mutex_lock(&early_suspend_lock);
	list_for_each(pos, &early_suspend_handlers) {
		struct early_suspend *e;
		e = list_entry(pos, struct early_suspend, link);
		if (e->level > handler->level)
			break;
	}
	list_add_tail(&handler->link, pos);
	if ((state & SUSPENDED) && handler->suspend)
		handler->suspend(handler);
	mutex_unlock(&early_suspend_lock);
}
EXPORT_SYMBOL(register_early_suspend);

void unregister_early_suspend(struct early_suspend *handler)
{
	mutex_lock(&early_suspend_lock);
	list_del(&handler->link);
	mutex_unlock(&early_suspend_lock);
}
EXPORT_SYMBOL(unregister_early_suspend);

/*< DTS2011092606663 pengyu 20110926 begin */
#ifdef CONFIG_HUAWEI_KERNEL
/*< DTS2011110802942 pengyu 20111108 begin */
/* Cancel DTS2011092606663, because it will affect the performance of music with wma format */
/* DTS2011110802942 pengyu 20111108 end >*/

/*< DTS2011102601746 pengyu 20111026 begin */
/* Set up_threshold of ondemand governor */
void set_up_threshold(int screen_on)
{
    int ret = 0;
    struct file *filp = (struct file *)-ENOENT;
    char buf[UP_THRESHOLD_MAX_SIZE];

    filp = filp_open(up_threshold_path, O_RDWR, S_IRUSR);
    if (IS_ERR(filp) || !filp->f_op) {
        printk(KERN_ERR "%s: file %s filp_open error\n", __FUNCTION__, up_threshold_path);
        return;
    }
    sprintf(buf, "%d\n", screen_on ? DEF_FREQUENCY_UP_THRESHOLD : MICRO_FREQUENCY_UP_THRESHOLD);
    ret = filp->f_op->write(filp, buf, UP_THRESHOLD_MAX_SIZE, &filp->f_pos);
    if (ret < 0) {
        printk(KERN_ERR "%s: write error: %d\n", __FUNCTION__, ret);
    }
    if (!IS_ERR(filp)) {
        filp_close(filp, NULL);
    }
}
EXPORT_SYMBOL(set_up_threshold);
/* DTS2011102601746 pengyu 20111026 end >*/
#endif
/* DTS2011092606663 pengyu 20110926 end >*/

static void early_suspend(struct work_struct *work)
{
	struct early_suspend *pos;
	unsigned long irqflags;
	int abort = 0;

	mutex_lock(&early_suspend_lock);
	spin_lock_irqsave(&state_lock, irqflags);
	if (state == SUSPEND_REQUESTED)
		state |= SUSPENDED;
	else
		abort = 1;
	spin_unlock_irqrestore(&state_lock, irqflags);

	if (abort) {
		if (debug_mask & DEBUG_SUSPEND)
			pr_info("early_suspend: abort, state %d\n", state);
		mutex_unlock(&early_suspend_lock);
		goto abort;
	}

	if (debug_mask & DEBUG_SUSPEND)
		pr_info("early_suspend: call handlers\n");
	list_for_each_entry(pos, &early_suspend_handlers, link) {
/* <DTS2010092703937 hufeng 20100927 begin */
#ifdef CONFIG_HUAWEI_KERNEL
        /*<BU5D00025, jialin, 20091223, add log information, begin */
        if (pos->suspend != NULL) {
            pos->suspend(pos);
            printk("s: %x\n",(unsigned int)pos->suspend);
        }
        /*BU5D00025, jialin, 20091223, add log information, end> */
#else
/* DTS2010092703937 hufeng 20100927 end> */
		if (pos->suspend != NULL) {
			pos->suspend(pos);
        }
/* <DTS2010092703937 hufeng 20100927 begin */
#endif
/* DTS2010092703937 hufeng 20100927 end> */
	}
/*< DTS2011092606663 pengyu 20110926 begin */
#ifdef CONFIG_HUAWEI_KERNEL
/*< DTS2011110802942 pengyu 20111108 begin */
/* Cancel DTS2011092606663, because it will affect the performance of music with wma format */
/* DTS2011110802942 pengyu 20111108 end >*/
/*< DTS2011102601746 pengyu 20111026 begin */
    /* Set up_threshold to MICRO_FREQUENCY_UP_THRESHOLD when screen is off */
    set_up_threshold(false);
/* DTS2011102601746 pengyu 20111026 end >*/
#endif
/* DTS2011092606663 pengyu 20110926 end >*/
	mutex_unlock(&early_suspend_lock);

	suspend_sys_sync_queue();
abort:
	spin_lock_irqsave(&state_lock, irqflags);
	if (state == SUSPEND_REQUESTED_AND_SUSPENDED)
		wake_unlock(&main_wake_lock);
	spin_unlock_irqrestore(&state_lock, irqflags);
}

static void late_resume(struct work_struct *work)
{
	struct early_suspend *pos;
	unsigned long irqflags;
	int abort = 0;

	mutex_lock(&early_suspend_lock);
/*< DTS2011092606663 pengyu 20110926 begin */
#ifdef CONFIG_HUAWEI_KERNEL
/*< DTS2011110802942 pengyu 20111108 begin */
/* Cancel DTS2011092606663, because it will affect the performance of music with wma format */
/* DTS2011110802942 pengyu 20111108 end >*/
/*< DTS2011102601746 pengyu 20111026 begin */
    /* Set up_threshold to DEF_FREQUENCY_UP_THRESHOLD when screen is ready to on */
    set_up_threshold(true);
/* DTS2011102601746 pengyu 20111026 end >*/
#endif
/* DTS2011092606663 pengyu 20110926 end >*/
	spin_lock_irqsave(&state_lock, irqflags);
	if (state == SUSPENDED)
		state &= ~SUSPENDED;
	else
		abort = 1;
	spin_unlock_irqrestore(&state_lock, irqflags);

	if (abort) {
		if (debug_mask & DEBUG_SUSPEND)
			pr_info("late_resume: abort, state %d\n", state);
		goto abort;
	}
	if (debug_mask & DEBUG_SUSPEND)
		pr_info("late_resume: call handlers\n");
	list_for_each_entry_reverse(pos, &early_suspend_handlers, link)
/* <DTS2010092703937 hufeng 20100927 begin */
#ifdef CONFIG_HUAWEI_KERNEL
        /*<BU5D00025, jialin, 20091223, add log information, begin */
        if (pos->resume != NULL) {
            pos->resume(pos);
            printk("r: %x\n",(unsigned int)pos->resume);
        }
        /*BU5D00025, jialin, 20091223, add log information, end> */
#else
/* DTS2010092703937 hufeng 20100927 end> */
		if (pos->resume != NULL) {
			pos->resume(pos);
        }
/* <DTS2010092703937 hufeng 20100927 begin */
#endif
/* DTS2010092703937 hufeng 20100927 end> */
	if (debug_mask & DEBUG_SUSPEND)
		pr_info("late_resume: done\n");
abort:
	mutex_unlock(&early_suspend_lock);
}

void request_suspend_state(suspend_state_t new_state)
{
	unsigned long irqflags;
	int old_sleep;

	spin_lock_irqsave(&state_lock, irqflags);
	old_sleep = state & SUSPEND_REQUESTED;
	if (debug_mask & DEBUG_USER_STATE) {
		struct timespec ts;
		struct rtc_time tm;
		getnstimeofday(&ts);
		rtc_time_to_tm(ts.tv_sec, &tm);
		pr_info("request_suspend_state: %s (%d->%d) at %lld "
			"(%d-%02d-%02d %02d:%02d:%02d.%09lu UTC)\n",
			new_state != PM_SUSPEND_ON ? "sleep" : "wakeup",
			requested_suspend_state, new_state,
			ktime_to_ns(ktime_get()),
			tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
			tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec);
	}
	if (!old_sleep && new_state != PM_SUSPEND_ON) {
		state |= SUSPEND_REQUESTED;
		queue_work(suspend_work_queue, &early_suspend_work);
	} else if (old_sleep && new_state == PM_SUSPEND_ON) {
		state &= ~SUSPEND_REQUESTED;
		wake_lock(&main_wake_lock);
		queue_work(suspend_work_queue, &late_resume_work);
	}
	requested_suspend_state = new_state;
	spin_unlock_irqrestore(&state_lock, irqflags);
}

suspend_state_t get_suspend_state(void)
{
	return requested_suspend_state;
}
