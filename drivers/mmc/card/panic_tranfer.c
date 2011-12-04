/* <DTS2010080901139 hufeng 20100821 begin */
/*
	panic transfer.c

*/
/* <DTS2010091301566 hufeng 20100913 begin */
#include <linux/mmc/card.h>
#include <linux/mmc/core.h>
#include <linux/scatterlist.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/module.h>
#include <linux/mmc/mmc_panic.h>

/* PANIC_MAGIC definition from kernel/drivers/misc/apanic.c */
#define PANIC_MAGIC 0xdeadf00d
/* DTS2010091301566 hufeng 20100913 end> */
/*<DTS2010122400624 liyuping 20101224 begin */ 
//panic info start from 1048582,avoid overriding flags info.
#define SECTOR_INDEX 1048582	/* the starting sector of the MISC partition(mmcblk0p7) */
/* DTS2010122400624 liyuping 20101224 end >*/


static struct mmc_card *panic_card;
/* <DTS2010091301566 hufeng 20100913 begin */

int mmc_panic_save_card(void *card)
{
	if(card != NULL) {
        panic_card = (struct mmc_card *)card;
        printk("mmc_panic_save_card card = %s, card type = %d\n",
                panic_card->dev.bus->name, panic_card->type);
    }
	
	return 0;
}
EXPORT_SYMBOL(mmc_panic_save_card);
/* DTS2010091301566 hufeng 20100913 end> */

/*
 * Fill in the mmc_request structure given a set of transfer parameters.
 */
static void mmc_panic_prepare_mrq(struct mmc_card *card,
	struct mmc_request *mrq, struct scatterlist *sg, unsigned sg_len,
	unsigned dev_addr, unsigned blocks, unsigned blksz, int write)
{
	BUG_ON(!mrq || !mrq->cmd || !mrq->data || !mrq->stop);

	if (blocks > 1) {
		mrq->cmd->opcode = write ?
			MMC_WRITE_MULTIPLE_BLOCK : MMC_READ_MULTIPLE_BLOCK;
	} else {
		mrq->cmd->opcode = write ?
			MMC_WRITE_BLOCK : MMC_READ_SINGLE_BLOCK;
	}

	mrq->cmd->arg = dev_addr;
	mrq->cmd->flags = MMC_RSP_R1 | MMC_CMD_ADTC;

	if (blocks == 1)
		mrq->stop = NULL;
	else {
		mrq->stop->opcode = MMC_STOP_TRANSMISSION;
		mrq->stop->arg = 0;
		mrq->stop->flags = MMC_RSP_R1B | MMC_CMD_AC;
	}

	mrq->data->blksz = blksz;
	mrq->data->blocks = blocks;
	mrq->data->flags = write ? MMC_DATA_WRITE : MMC_DATA_READ;
	mrq->data->sg = sg;
	mrq->data->sg_len = sg_len;

	mmc_set_data_timeout(mrq->data, card);
}

/*
 * Wait for the card to finish the busy state
 */
static int mmc_panic_wait_busy(struct mmc_card *card)
{
	int ret, busy;
	struct mmc_command cmd;

	busy = 0;
	do {
		memset(&cmd, 0, sizeof(struct mmc_command));

		cmd.opcode = MMC_SEND_STATUS;
		cmd.arg = card->rca << 16;
		cmd.flags = MMC_RSP_R1 | MMC_CMD_AC;

		ret = mmc_panic_wait_for_cmd(card->host, &cmd, 0);
		printk("#### panic send command resp = %d\n", cmd.resp[0]);
		
		if (ret)
			break;

		if (!busy && !(cmd.resp[0] & R1_READY_FOR_DATA)) {
			busy = 1;
			printk(KERN_INFO "%s: Warning: Host did not "
				"wait for busy state to end.\n",
				mmc_hostname(card->host));
		}
	} while (!(cmd.resp[0] & R1_READY_FOR_DATA) ||
				(R1_CURRENT_STATE(cmd.resp[0]) == 7));

	return ret;
}


/* <DTS2010091301566 hufeng 20100913 begin */
static int mmc_panic_buffer_transfer(struct mmc_card *card,
	u8 *buffer, unsigned addr, unsigned blksz, int write, int in_panic)
{
	int ret;

	struct mmc_request mrq;
	struct mmc_command cmd;
	struct mmc_command stop;
	struct mmc_data data;

	struct scatterlist sg;
	unsigned blocks;
	
	memset(&mrq, 0, sizeof(struct mmc_request));
	memset(&cmd, 0, sizeof(struct mmc_command));
	memset(&data, 0, sizeof(struct mmc_data));
	memset(&stop, 0, sizeof(struct mmc_command));

	mrq.cmd = &cmd;
	mrq.data = &data;
	mrq.stop = &stop;
	blocks = CRASH_LOG_SIZE/blksz;
		
    if (in_panic) {
        card->host->claimed = 1;
    } else {
        mmc_claim_host(card->host);
    }
	
	sg_init_one(&sg, buffer, blksz*blocks);

	mmc_panic_prepare_mrq(card, &mrq, &sg, 1, addr, blocks, blksz, write);

	mmc_panic_start_req(card->host, &mrq);
/* DTS2010091301566 hufeng 20100913 end> */
	if (cmd.error){
		ret = cmd.error;
		goto error_out;
	}
	if (data.error){
		ret = data.error;
		goto error_out;
	}

	ret = mmc_panic_wait_busy(card);

error_out:
/* <DTS2010091301566 hufeng 20100913 begin */
    if (in_panic) {
        card->host->claimed = 0;
    } else {
        mmc_release_host(card->host);
    }
/* DTS2010091301566 hufeng 20100913 end> */

	return ret;
}

/*
 * Configure correct block size in card
 */
static int mmc_panic_set_blksize(struct mmc_card *card, unsigned size)
{
	struct mmc_command cmd;
	int ret;

	printk("mmc_panic_set_blksize ENTRY\n");
	cmd.opcode = MMC_SET_BLOCKLEN;
	cmd.arg = size;
	cmd.flags = MMC_RSP_R1 | MMC_CMD_AC;
	ret = mmc_panic_wait_for_cmd(card->host, &cmd, 0);
	if (ret)
		return ret;

	return 0;
}

/**
mmc_panic_log_transfer
description: transfer buffer to logging partion

*/
/* <DTS2010091301566 hufeng 20100913 begin */
static int mmc_panic_log_transfer(u8 *buffer, int write, int in_panic)
{
	int ret;
	unsigned sector = SECTOR_INDEX;

    if (in_panic) {
        panic_card->host->claimed = 1;
    } else {
        mmc_claim_host(panic_card->host);
    }
	
    ret = mmc_panic_set_blksize(panic_card, 512);

    if (in_panic) {
        panic_card->host->claimed = 0;
    } else {
        mmc_release_host(panic_card->host);
    }
/* DTS2010091301566 hufeng 20100913 end> */
	
	printk("mmc_panic_set_blksize\n");
	
	if (ret)
		return ret;

/* <DTS2010091301566 hufeng 20100913 begin */
	ret = mmc_panic_buffer_transfer(panic_card, buffer, sector, 512, write, in_panic);
	
/* DTS2010091301566 hufeng 20100913 end> */
	return 0;
}

/**
mmc_panic_write
description: write panic log to logging partition

PARAM:
unsigned log_len:  this length of buffer must equal to logging partion len
u8* buffer: log buffer

*/
/* <DTS2010091301566 hufeng 20100913 begin */
int mmc_panic_write(u8* buffer, unsigned int length)
{
    u32 panic_magic = *(u32*)buffer;
    int in_panic;

    BUG_ON(length != CRASH_LOG_SIZE);

    if (panic_magic == PANIC_MAGIC) {
        in_panic = 1;
    } else {
        in_panic = 0;
    }

    mmc_panic_log_transfer(buffer, 1, in_panic);
	
	return 0;
}
EXPORT_SYMBOL(mmc_panic_write);

/**
mmc_panic_read
description: read panic log from logging partition

PARAM:
u8* buffer: log buffer, the length of buffer must equal to BUFFER_SIZE
unsigned log_len:  this length of buffer must equal to logging partion len

*/
int mmc_panic_read(u8* buffer, unsigned int length)
{
    BUG_ON(length != CRASH_LOG_SIZE);

    mmc_panic_log_transfer(buffer, 0, 0);
	
	return 0;
}
EXPORT_SYMBOL(mmc_panic_read);
/* DTS2010091301566 hufeng 20100913 end> */


/* DTS2010080901139 hufeng 20100821 end> */
