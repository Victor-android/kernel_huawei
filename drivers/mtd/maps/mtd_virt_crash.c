/* <DTS2010080901139 hufeng 20100821 begin */
/*
 * Huawei virtual MTD used on crash debug
 *
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <asm/io.h>
#include <mach/hardware.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
/* <DTS2010091301566 hufeng 20100913 begin */
#include <linux/proc_fs.h>
#include <linux/mmc/mmc_panic.h>
/* DTS2010091301566 hufeng 20100913 end> */

// MTD device: name CRASH

/* dynamic ioremap() areas */

/* <DTS2010091301566 hufeng 20100913 begin */
/* removed several lines. */
/* DTS2010091301566 hufeng 20100913 end> */
u8 crash_log[CRASH_LOG_SIZE];

static struct mtd_info *crash_mtd;

struct map_info crash_sram_map = {
	.name = "MTD-Crash",
	.size = CRASH_LOG_SIZE,
	.bankwidth = 4,
	.phys = 0,
};

static int init_crash_virt_flash (void)
{
	int err;

/* <DTS2010091301566 hufeng 20100913 begin */
    /** add the read part of the apanic-on-mmc function to make it complete **/
    /* read the mmc panic partition into the mtd virtual flash */
    mmc_panic_read(crash_log, CRASH_LOG_SIZE);
/* DTS2010091301566 hufeng 20100913 end> */

	crash_sram_map.virt = crash_log;
	
	simple_map_init(&crash_sram_map);
	
	crash_mtd = do_map_probe("map_ram", &crash_sram_map);
	if (!crash_mtd) {
		printk("HW crash SRAM probe failed\n");
		err = -ENXIO;
		goto out;
	}

	crash_mtd->owner = THIS_MODULE;
	//crash_mtd->erasesize = 16;

	if (add_mtd_device(crash_mtd)) {
		printk("hw Crash SRAM device addition failed\n");
		err = -ENOMEM;
		goto out_probe;
	}

	printk("crash MTD virtual device add sucess! log size = 0x%x\n", CRASH_LOG_SIZE);
	return 0;

out_probe:
	map_destroy(crash_mtd);
	crash_mtd = 0;
out:
	return err;
}

/* <DTS2010091301566 hufeng 20100913 begin */
static int trigger_init_flash(struct file *file, const char __user *buffer,
        unsigned long count, void *data)
{
    static int init_flag = 0;
    /* init the virtual flash only once */
    if (init_flag == 0) { 
        init_crash_virt_flash();
        init_flag = 1;
    }

    return count;
}

/*
 * we do not call init_crash_virt_flash() in module initialization, since the
 * mmc is not ready yet. Instead, we call it only after the mmc by writing to
 * the proc entry later.
 */
/* DTS2010091301566 hufeng 20100913 end> */
static int __init init_hw_crash_virt_mtd(void)
{
/* <DTS2010091301566 hufeng 20100913 begin */
	struct proc_dir_entry *init;

    init = create_proc_entry("init_panic_virt_flash", S_IWUSR, NULL);
    if (init == NULL) {
		printk(KERN_ERR "%s: failed creating proc file\n", __func__);
        return -1;
    }

    init->write_proc = trigger_init_flash;
/* DTS2010091301566 hufeng 20100913 end> */

	return 0;
}


static void __exit exit_hw_crash_virt_mtd(void)
{
}



module_init(init_hw_crash_virt_mtd);
module_exit(exit_hw_crash_virt_mtd);

MODULE_DESCRIPTION("CRASH map driver");
MODULE_LICENSE("GPL");

/* DTS2010080901139 hufeng 20100821 end> */
