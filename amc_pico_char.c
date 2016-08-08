/*
 * AMC-Pico8 Linux Driver
 *
 *  Copyright (C) 2015 CAEN ELS d.o.o.
 *
 *  Jan Marjanovic <j.marjanovic@caenels.com>
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
	struct board_data *board = container_of(inode->i_cdev, struct board_data, cdev);

	dev_dbg(&board->pci_dev->dev, "char_open()\n");

	if (!try_module_get(THIS_MODULE))
		return -ENODEV;

	file->private_data = board;

	return 0;
}

static
int char_release(struct inode *inode, struct file *file)
{
	struct board_data *board = container_of(inode->i_cdev, struct board_data, cdev);

	dev_dbg(&board->pci_dev->dev, "char_release()\n");

	module_put(THIS_MODULE);
	return 0;
}

static
ssize_t char_read(
	struct file *filp,
	char __user *buf,
	size_t count,
	loff_t *pos
)
{
	struct board_data *board;
	int rc, cond;
	size_t tmp_count;
	int i;

	board = (struct board_data *) filp->private_data;

    dev_dbg(&board->pci_dev->dev, "  read(), count %zd\n", count);

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

static
long char_ioctl(
	struct file *filp,
	unsigned int cmd,
	unsigned long arg
)
{
	long ret;
    struct board_data *board = (struct board_data *) filp->private_data;
	struct trg_ctrl trg;
	uint32_t control = 0, scaler = 0;
	uint8_t range = 0;
	uint32_t rng_buf_delay = 0;
	uint32_t ctrl_tmp;

    dev_dbg(&board->pci_dev->dev, "%s\n", __PRETTY_FUNCTION__);
    dev_dbg(&board->pci_dev->dev, "cmd: 0x%08x\n", cmd);

	/* validate cmd and copy in values from user before locking
	 * can't access board->
	 */
	switch (cmd) {
	case SET_RANGE:
		ret = get_user(range, (uint8_t*)arg);
		range &= 0xFF;
        dev_dbg(&board->pci_dev->dev, "setting range: %01x\n", range);
		break;
	case SET_FSAMP:
		ret = get_user(scaler, (uint32_t*)arg);
		if(!ret) {
            dev_dbg(&board->pci_dev->dev, "clock scaler: %d\n", scaler);

			if ((scaler == 0) || scaler > PICO_CONV_MAX){
				printk(KERN_DEBUG MOD_NAME
					": prescaler out of range (%d > %d)\n",
					scaler, PICO_CONV_MAX);

				ret = -EINVAL;
			}
		}
		break;
	case SET_TRG:
		ret = copy_from_user(&trg, (void*)arg, sizeof(trg));
        dev_dbg(&board->pci_dev->dev, "    trg.limit: %08x\n",
			*(uint32_t *)&trg.limit);
        dev_dbg(&board->pci_dev->dev, "    trg.nrsamp: %d\n", trg.nr_samp);
        dev_dbg(&board->pci_dev->dev, "    trg.ch_sel: %d\n", trg.ch_sel);
        dev_dbg(&board->pci_dev->dev, "    trg.mode: %s\n",
			(trg.mode == DISABLED ? "DISABLED" :
			trg.mode == POS_EDGE ?  "POS_EDGE" :
			trg.mode == NEG_EDGE ?  "NEG_EDGE" : "BOTH_EDGE"));

		break;
	case SET_RING_BUF:
		ret = get_user(rng_buf_delay, (uint32_t*)arg);
        dev_dbg(&board->pci_dev->dev, "rng_buf_delay: %d\n", rng_buf_delay);
		break;
	case SET_GATE_MUX:
        dev_dbg(&board->pci_dev->dev, "set gate mux: %d\n", control);
		ret = get_user(control, (uint32_t*)arg);
		break;
	case SET_CONV_MUX:
        dev_dbg(&board->pci_dev->dev, "set conv mux: %d\n", control);
		ret = get_user(control, (uint32_t*)arg);
		break;
	case GET_RANGE:
	case GET_FSAMP:
	case GET_B_TRANS:
	case ABORT_READ:
		ret = 0;
		break;
	case GET_VERSION:
		return put_user(1, (uint32_t*)arg);
	default:
		ret = -EINVAL;
	}

    if(ret) return ret;

    spin_lock_irq(&board->dma_queue.lock); /* enter critical section, can't sleep */

	switch (cmd) {
	case SET_RANGE:
		control = ioread32(board->bar0);
		control &= ~0xFFUL;
		control |= range;
		iowrite32(control, board->bar0);
		control = ioread32(board->bar0);
		break;

	case GET_RANGE:
		range = ioread32(board->bar0) & 0xFF;
		break;

	case SET_FSAMP:
		iowrite32(scaler, board->bar0 + PICO_CONV_GEN);
		scaler = ioread32(board->bar0 + PICO_CONV_GEN);
		break;

	case GET_FSAMP:
		scaler = ioread32(board->bar0 + PICO_CONV_GEN);
		break;

	case GET_B_TRANS:
        control = board->dma_bytes_trans;
		break;

	case SET_TRG:
		iowrite32(*(uint32_t *)&trg.limit,
			board->bar0 + PICO_ADDR + TRG_OFFS_LIMIT);
		iowrite32(trg.nr_samp,
			board->bar0 + PICO_ADDR + TRG_OFFS_NRSAMP);

		ctrl_tmp = ioread32(board->bar0 + PICO_ADDR + TRG_OFFS_CTRL);

		/* change the trigger edge */
		ctrl_tmp &= ~(0x3);
		ctrl_tmp |= trg.mode;

		/* change the channel bits */
		ctrl_tmp &= ~(0x7 << TRG_CTRL_CH_SHIFT);
		ctrl_tmp |= trg.ch_sel << TRG_CTRL_CH_SHIFT;

		iowrite32(ctrl_tmp, board->bar0 + PICO_ADDR + TRG_OFFS_CTRL);
		control = ioread32(board->bar0 + PICO_ADDR + TRG_OFFS_CTRL);
		break;

	case SET_RING_BUF:
		iowrite32(rng_buf_delay,
			board->bar0 + PICO_ADDR + RING_BUFF_OFFS_DELAY);
		break;

	case SET_GATE_MUX:
		control &= MUX_TRG_MASK;
		control <<= MUX_TRG_SHIFT;

		ctrl_tmp = ioread32(board->bar0 + PICO_ADDR + PICO_CONV_TRG);
		ctrl_tmp &= ~(MUX_TRG_MASK << MUX_TRG_SHIFT);
		ctrl_tmp |= control;

		iowrite32(ctrl_tmp, board->bar0 + PICO_ADDR + PICO_CONV_TRG);
		control = ioread32(board->bar0 + PICO_ADDR + PICO_CONV_TRG);
		break;

	case SET_CONV_MUX:
		control &= MUX_CONV_MASK;
		control <<= MUX_CONV_SHIFT;

		ctrl_tmp = ioread32(board->bar0 + PICO_ADDR + PICO_CONV_TRG);
		ctrl_tmp &= ~(MUX_CONV_MASK << MUX_CONV_SHIFT);
		ctrl_tmp |= control;

		iowrite32(ctrl_tmp, board->bar0 + PICO_ADDR + PICO_CONV_TRG);
		control = ioread32(board->bar0 + PICO_ADDR + PICO_CONV_TRG);
		break;

	case ABORT_READ:
        board->dma_irq_flag = 2;
        wake_up_locked(&board->dma_queue);

	default:
		break;
	}

    spin_unlock_irq(&board->dma_queue.lock); /* end of critical section, can't access board-> */

	/* copy to user */
	switch (cmd) {
	case SET_RANGE:
        dev_dbg(&board->pci_dev->dev, "   control reg readback: %08x\n",
			range);
		break;
	case GET_RANGE:
        dev_dbg(&board->pci_dev->dev, "   range: %x\n", range);
		ret = put_user(range, (uint8_t*)arg);
		break;
	case SET_FSAMP:
        dev_dbg(&board->pci_dev->dev, "   scaler readback: %08x\n",
			scaler);
		break;
	case GET_FSAMP:
        dev_dbg(&board->pci_dev->dev, "   scaler: %d", scaler);
		ret = put_user(scaler, (uint32_t*)arg);
		break;
	case SET_TRG:
        dev_dbg(&board->pci_dev->dev, "   trigger control: %08x\n",
			(unsigned)control);
		break;
	case SET_RING_BUF:
		break;
	case SET_GATE_MUX:
        dev_dbg(&board->pci_dev->dev, "cont_trg reg: %08x\n",
			(unsigned)control);
		break;
	case SET_CONV_MUX:
        dev_dbg(&board->pci_dev->dev, "cont_trg reg: %08x\n",
			(unsigned)control);
		break;
	case GET_B_TRANS:
		ret = put_user(control, (uint32_t*)arg);
		break;
	case ABORT_READ:
		break;
	}

	return ret;
}

const struct file_operations amc_pico_fops = {
	.owner		= THIS_MODULE,
	.open		= char_open,
	.release	= char_release,
	.read		= char_read,
	.unlocked_ioctl = char_ioctl
};
