/*
 * AMC-Pico8 Linux Driver
 *
 *  Copyright (C) 2015 CAEN ELS d.o.o.
 *
 *  Jan Marjanovic <j.marjanovic@caenels.com>
 *
 *  Copyright 2016 Board of Trustees of Michigan State University
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License v2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#include "amc_pico_char.h"

static
int char_open(struct inode *inode, struct file *file)
{
    struct file_data *fdata;
	struct board_data *board = container_of(inode->i_cdev, struct board_data, cdev);

	dev_dbg(&board->pci_dev->dev, "char_open()\n");

	if (!try_module_get(THIS_MODULE))
		return -ENODEV;

    fdata = kzalloc(sizeof(*fdata), GFP_KERNEL);
    if(!fdata) {
        module_put(THIS_MODULE);
        return -ENOMEM;
    }
    fdata->board = board;

    file->private_data = fdata;

	return 0;
}

static
int char_release(struct inode *inode, struct file *file)
{
    struct file_data *fdata = (struct file_data *)file->private_data;
	struct board_data *board = container_of(inode->i_cdev, struct board_data, cdev);

	dev_dbg(&board->pci_dev->dev, "char_release()\n");

    kfree(fdata);

	module_put(THIS_MODULE);
	return 0;
}

#ifdef CONFIG_AMC_PICO_FRIB
static ssize_t frib_write_reg(struct board_data *board,
                             const char __user *buf,
                             size_t count,
                             loff_t *pos);
static ssize_t frib_read_reg(struct board_data *board,
                             char __user *buf,
                             size_t count,
                             loff_t *pos);
static ssize_t frib_read_capture(struct board_data *board,
                                 char __user *buf,
                                 size_t count,
                                 loff_t *pos);
#endif

static
ssize_t char_read(
	struct file *filp,
	char __user *buf,
	size_t count,
	loff_t *pos
)
{
    struct file_data *fdata = (struct file_data *)filp->private_data;
    struct board_data *board = fdata->board;
	int rc, cond;
	size_t tmp_count;
	int i;

    dev_dbg(&board->pci_dev->dev, "  read(), site_mode=%u count %zd\n", fdata->site_mode, count);
    if(0) {}
#ifdef CONFIG_AMC_PICO_FRIB
    else if(dmac_site==USER_SITE_FRIB) {
        switch(fdata->site_mode) {
        case 0:  break;
        case 1:  return frib_read_reg(board, buf, count, pos);
        case 2:  return frib_read_capture(board, buf, count, pos);
        default: return -EINVAL;
        }
    }
#endif
    else if(fdata->site_mode!=0)
        return -EINVAL;

    if (count > DMA_BUF_COUNT*DMA_BUF_SIZE) return -EINVAL;

    spin_lock_irq(&board->dma_queue.lock);

	if(board->read_in_progress) {
        spin_unlock_irq(&board->dma_queue.lock);
        dev_dbg(&board->pci_dev->dev, "  read(), concurrent read()s not allowed\n");
		return -EIO;
	}
	board->read_in_progress = 1;

	/* start dma transfer */
	i = 0;
	tmp_count = count;
	dma_enable(board, 0);
	while (tmp_count > DMA_BUF_SIZE) {
		dma_push(board, (uint32_t)board->dma_buf[i++], DMA_BUF_SIZE, 0);
		tmp_count -= DMA_BUF_SIZE;
	}
	dma_push(board, (uint32_t)board->dma_buf[i], tmp_count, 1);
	mb();
	dma_enable(board, 1);

    if (likely(board->irqmode!=dmac_irq_poll)) {
        rc = wait_event_interruptible_locked_irq(board->dma_queue, board->dma_irq_flag!=0);

    } else {
        const unsigned long twait = msecs_to_jiffies(1);
        do {
            spin_unlock_irq(&board->dma_queue.lock);
            /* must unlock for call to amc_isr() as spin locks aren't recursive */
            if(amc_isr(board->pci_dev->irq, board)==IRQ_NONE)
                rc = wait_event_interruptible_timeout(board->dma_queue, board->dma_irq_flag!=0, twait);
            else
                rc = 0;
            spin_lock_irq(&board->dma_queue.lock);
            /* continue while no "IRQ" signaled, and wait not interrupted */
        } while(board->dma_irq_flag==0 && rc>=0);
        if(rc>0) rc=0;
    }
    /*
     * dma_irq_flag==1 && rc==0 is normal completion
     * rc==-ERESTARTSYS is user abort
     * other cases not to happen, but are treated as -ECANCELED
     */

    cond = board->dma_irq_flag;
    board->dma_irq_flag = 0;
	board->read_in_progress = 0;
    dev_dbg(&board->pci_dev->dev, "read() wait complete w/ rc=%d cond=%d\n", rc, cond);

	if (rc != 0 || cond!=1) { /* interrupted or aborted */
		if(cond!=1) rc = -ECANCELED;
		/* reset DMA engine */
		dma_reset(board);
        board->dma_bytes_trans = 0;
        spin_unlock_irq(&board->dma_queue.lock);

        dev_dbg(&board->pci_dev->dev, "  read(): interrupt failed: %d\n", rc);
		return rc;
	} else {
        spin_unlock_irq(&board->dma_queue.lock);

		i = 0;
		tmp_count = count;
        dev_dbg(&board->pci_dev->dev, "  read(): returned from sleep\n");
		rc = 0;
		while (tmp_count > DMA_BUF_SIZE && rc==0) {
			rc = copy_to_user(buf + DMA_BUF_SIZE*i,
					board->kernel_mem_buf[i], DMA_BUF_SIZE);
			tmp_count -= DMA_BUF_SIZE;
			/* sometimes the DMA done interrupt comes even though nothing has been
			 * transfered.  Fill our buffer with a test pattern so that this is more
			 * obvious.
			 */
			memset(board->kernel_mem_buf[i], 0xf0, DMA_BUF_SIZE);
			i++;
		}
		if(rc==0) {
			rc = copy_to_user(buf + DMA_BUF_SIZE*i,
							board->kernel_mem_buf[i], tmp_count);
			memset(board->kernel_mem_buf[i], 0xf0, tmp_count);
		}

		if(rc) return rc;
	}

	*pos += count;

	return count;
}

/* all possible ioctl() value types */
union ioctl_value {
    uint8_t u8;
    uint32_t u32;
    struct trg_ctrl trg;
};

static
long char_ioctl(
	struct file *filp,
	unsigned int cmd,
	unsigned long arg
)
{
    union ioctl_value uval;
	long ret;
    struct file_data *fdata = (struct file_data *)filp->private_data;
    struct board_data *board = fdata->board;
    size_t tocpy = sizeof(uval);

    /* copy some bytes to/from user space, others are zero'd */
    memset(&uval, 0, sizeof(uval));
    if(_IOC_SIZE(cmd)<sizeof(uval))
        tocpy = _IOC_SIZE(cmd);

    dev_dbg(&board->pci_dev->dev, "%s: 0x%08x size=%u\n", __PRETTY_FUNCTION__, cmd, (unsigned)tocpy);

    if(_IOC_DIR(cmd)&_IOC_WRITE) {
        /* copy in all provided bytes. based on IOCTL code. */
        ret = copy_from_user(&uval, (const void*)arg, tocpy);
        if(ret) return ret;
    }

	/* validate cmd and copy in values from user before locking
	 * can't access board->
	 */
	switch (cmd) {
    case SET_RANGE:
        ret = 0;
		break;
    case SET_FSAMP:	{
		uint32_t scaler = PICO_CLK_FREQ / uval.u32 - 1;
		if ((uval.u32 > PICO_ADC_MAX_FREQ) || (scaler > (PICO_CONV_MAX-1))){
			return -EINVAL;
		} else {
			ret = 0;
		}
		break;
	}
    case SET_TRG:
    case SET_RING_BUF:
	case SET_GATE_MUX:
	case SET_CONV_MUX:
	case GET_RANGE:
	case GET_FSAMP:
	case GET_B_TRANS:
	case ABORT_READ:
		ret = 0;
		break;
	case GET_VERSION:
        /* Versions:
         *  0 - implied by errno==EINVAL
         *  1 - Added GET_VERSION and ABORT_READ
         *  2 - Added GET_SITE_ID, GET_SITE_VERSION, SET_SITE_MODE.
         *      Changed all others.
         *  3 - Changed GET_FSAMP and SET_FSAMP to use frequency as
         *      a parameter
         */
        return put_user(GET_VERSION_CURRENT, (uint32_t*)arg);
    case GET_SITE_ID:
        return put_user(dmac_site, (uint32_t*)arg);
    case GET_SITE_VERSION:
    {
        uint32_t sver;
        switch(dmac_site) {
#ifdef CONFIG_AMC_PICO_FRIB
        case USER_SITE_FRIB: sver = 0; break;
#endif
        default: sver = 0;
        }
        return put_user(sver, (uint32_t*)arg);
    }
    case SET_SITE_MODE:
        if(0) {}
#ifdef CONFIG_AMC_PICO_FRIB
        else if(dmac_site==USER_SITE_FRIB) {
            if(uval.u32>2) return -EINVAL;
        }
#endif
        else if(uval.u32!=0) return -EINVAL;

        fdata->site_mode = uval.u32;

        return 0;
	default:
        ret = -EINVAL;
	}

    if(ret) return ret;

    /* locking here to protect RMW register operations.
     * Use dma_queue.lock for convinience
     */
    spin_lock_irq(&board->dma_queue.lock); /* enter critical section, can't sleep */

	switch (cmd) {
    case SET_RANGE: {
        uint32_t control;
		control = ioread32(board->bar0);
		control &= ~0xFFUL;
        control |= uval.u8;
		iowrite32(control, board->bar0);
		control = ioread32(board->bar0);
		break;
    }
	case GET_RANGE:
        uval.u8 = ioread32(board->bar0) & 0xFF;
		break;

	case SET_FSAMP: {
        uint32_t scaler = PICO_CLK_FREQ / uval.u32 - 1;
        iowrite32(scaler, board->bar0 + PICO_CONV_GEN);
        uval.u32 = ioread32(board->bar0 + PICO_CONV_GEN);
		break;
	}

	case GET_FSAMP: {
        uint32_t scaler = ioread32(board->bar0 + PICO_CONV_GEN);
        uval.u32 = PICO_CLK_FREQ / (scaler + 1);
		break;
	}

	case GET_B_TRANS:
        uval.u32 = board->dma_bytes_trans;
		break;

    case SET_TRG: {
        uint32_t ctrl_tmp;
        iowrite32(*(uint32_t *)&uval.trg.limit,
			board->bar0 + PICO_ADDR + TRG_OFFS_LIMIT);
        iowrite32(uval.trg.nr_samp,
			board->bar0 + PICO_ADDR + TRG_OFFS_NRSAMP);

		ctrl_tmp = ioread32(board->bar0 + PICO_ADDR + TRG_OFFS_CTRL);

		/* change the trigger edge */
		ctrl_tmp &= ~(0x3);
        ctrl_tmp |= uval.trg.mode;

		/* change the channel bits */
		ctrl_tmp &= ~(0x7 << TRG_CTRL_CH_SHIFT);
        ctrl_tmp |= uval.trg.ch_sel << TRG_CTRL_CH_SHIFT;

		iowrite32(ctrl_tmp, board->bar0 + PICO_ADDR + TRG_OFFS_CTRL);
        uval.u32 = ioread32(board->bar0 + PICO_ADDR + TRG_OFFS_CTRL);
		break;
    }
	case SET_RING_BUF:
        iowrite32(uval.u32,
			board->bar0 + PICO_ADDR + RING_BUFF_OFFS_DELAY);
		break;

    case SET_GATE_MUX: {
        uint32_t ctrl_tmp;
        uval.u32 &= MUX_TRG_MASK;
        uval.u32 <<= MUX_TRG_SHIFT;

		ctrl_tmp = ioread32(board->bar0 + PICO_ADDR + PICO_CONV_TRG);
		ctrl_tmp &= ~(MUX_TRG_MASK << MUX_TRG_SHIFT);
        ctrl_tmp |= uval.u32;

		iowrite32(ctrl_tmp, board->bar0 + PICO_ADDR + PICO_CONV_TRG);
        uval.u32 = ioread32(board->bar0 + PICO_ADDR + PICO_CONV_TRG);
		break;
    }
    case SET_CONV_MUX: {
        uint32_t ctrl_tmp;
        uval.u32 &= MUX_CONV_MASK;
        uval.u32 <<= MUX_CONV_SHIFT;

		ctrl_tmp = ioread32(board->bar0 + PICO_ADDR + PICO_CONV_TRG);
		ctrl_tmp &= ~(MUX_CONV_MASK << MUX_CONV_SHIFT);
        ctrl_tmp |= uval.u32;

		iowrite32(ctrl_tmp, board->bar0 + PICO_ADDR + PICO_CONV_TRG);
        uval.u32 = ioread32(board->bar0 + PICO_ADDR + PICO_CONV_TRG);
		break;
    }
	case ABORT_READ:
        /* abort in progress DMA waiter */
        board->dma_irq_flag = 2;
        wake_up_locked(&board->dma_queue);

	default:
        dev_dbg(&board->pci_dev->dev, "%s:   unknown ioctl\n", __PRETTY_FUNCTION__);
		break;
	}

    spin_unlock_irq(&board->dma_queue.lock); /* end of critical section, can't access board-> */

#ifdef CONFIG_AMC_PICO_FRIB
    if(cmd==ABORT_READ) {
        /* abort any waiting for capture buffer */
        spin_lock_irq(&board->capture_queue.lock);
        board->capture_ready = 2;
        wake_up_locked(&board->capture_queue);
        spin_unlock_irq(&board->capture_queue.lock);
    }
#endif

    if(_IOC_DIR(cmd)&_IOC_READ) {
        ret = copy_to_user((void*)arg, &uval, tocpy);
        if(ret) return ret;
    }

	return ret;
}

static
ssize_t char_write(struct file *filp, const char __user *buf, size_t count, loff_t *pos)
{
    struct file_data *fdata = (struct file_data *)filp->private_data;
    struct board_data *board = fdata->board;
    (void)board;
    if(0) {}
#ifdef CONFIG_AMC_PICO_FRIB
    else if(dmac_site==USER_SITE_FRIB) {
        switch(fdata->site_mode) {
        case 1:  return frib_write_reg(board, buf, count, pos);
        default: return -EINVAL;
        }
    }
#endif
    else
        return -EINVAL;
}

static
loff_t char_llseek(struct file *filp, loff_t pos, int whence)
{
    struct file_data *fdata = (struct file_data *)filp->private_data;
    //struct board_data *board = fdata->board;
    loff_t npos;

    if(dmac_site!=USER_SITE_FRIB || fdata->site_mode==0)
        return -EINVAL;
#ifdef CONFIG_AMC_PICO_FRIB

    switch(whence) {
    case 0: npos = pos; break;
    case 1: npos = filp->f_pos + pos; break;
    case 2: npos = 0x40000 + pos; break;
    default: return -EINVAL;
    }

    if(npos<0)
        return -EINVAL;
    else if(npos>0x40000)
        npos = 0x40000;

    filp->f_pos = npos;
    return npos;

#else
    //(void)board;
    (void)npos;
    return -EINVAL;
#endif
}

const struct file_operations amc_pico_fops = {
	.owner		= THIS_MODULE,
	.open		= char_open,
	.release	= char_release,
	.read		= char_read,
    .write      = char_write,
    .llseek     = char_llseek,
	.unlocked_ioctl = char_ioctl
};

#ifdef CONFIG_AMC_PICO_FRIB

static ssize_t frib_write_reg(struct board_data *board,
                             const char __user *buf,
                             size_t count,
                             loff_t *pos)
{
    ssize_t ret=0;
    unsigned offset, end;
    const uint32_t __user *ibuf = (const uint32_t __user *)buf;

    if(count%4) return -EINVAL;
    if(!pos || *pos<USER_ADDR) return -EINVAL;

    offset = *pos-USER_ADDR;

    if(offset>=0x10000) return -EINVAL;

    if(count>0x10000-offset)
        count = 0x10000-offset;

    end = offset+count;

    for(; offset<count; offset+=4, ibuf+=4)
    {
        uint32_t val;
        ret = get_user(val, ibuf);
        if(unlikely(ret)) break;
        iowrite32(val, board->bar0 + USER_ADDR + offset);
    }

    if(!ret) ret=count;
    return ret;
}

static ssize_t frib_read_reg(struct board_data *board,
                             char __user *buf,
                             size_t count,
                             loff_t *pos)
{
    ssize_t ret=0;
    unsigned offset, end;
    uint32_t __user *ibuf = (uint32_t __user *)buf;

    if(count%4) return -EINVAL;
    if(!pos || *pos<USER_ADDR) return -EINVAL;

    offset = *pos-USER_ADDR;

    if(offset>=0x10000) return 0;

    if(count>0x10000-offset)
        count = 0x10000-offset;

    end = offset+count;

    for(; ret && offset<count; offset+=4, ibuf+=4)
    {
        uint32_t val = ioread32(board->bar0 + USER_ADDR + offset);
        ret = put_user(val, ibuf);
    }

    if(!ret) ret=count;
    return ret;
    /* *pos not updated */
}

static ssize_t frib_read_capture(struct board_data *board,
                                 char __user *buf,
                                 size_t count,
                                 loff_t *pos)
{
    ssize_t ret;
    unsigned offset;

    if(count%4) return -EINVAL;
    if(!pos || *pos<USER_ADDR+0x100) return -EINVAL;

    offset = *pos-(USER_ADDR+0x100);

    if(offset>=board->capture_length) {
        dev_dbg(&board->pci_dev->dev, "Capture read starts past end of buffer %u %u\n",
                offset, board->capture_length);
        return 0;
    }

    if(count > board->capture_length-offset)
        count = board->capture_length-offset;

    spin_lock_irq(&board->capture_queue.lock);

    ret = wait_event_interruptible_locked_irq(board->capture_queue, board->capture_ready!=0);

    if(!ret && board->capture_ready!=1)
        ret = -ECANCELED;
    board->capture_ready = 0;

    spin_unlock_irq(&board->capture_queue.lock);

    if(ret) return ret;

    ret = copy_to_user(buf, board->capture_buf, count);
    if(!ret) ret=count;
    return ret;
    /* *pos not updated */
}

#endif
