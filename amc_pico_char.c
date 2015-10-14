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

int char_open(struct inode *inode, struct file *file)
{
	struct board_data *board;

	debug_print(DEBUG_CHAR, "char_open()\n");

	board = container_of(inode->i_cdev, struct board_data, cdev);
	file->private_data = board;

	return 0;
}


ssize_t char_read(
	struct file *filp,
	char __user *buf,
	size_t count,
	loff_t *pos
)
{
	struct board_data *board;
	int rc;
	size_t tmp_count;
	int i;

	board = (struct board_data *) filp->private_data;

	mutex_lock(&board->mutex);

	debug_print(DEBUG_CHAR, "  read(), count %zd\n", count);

	if (count > DMA_BUF_COUNT*DMA_BUF_SIZE) return -EINVAL;

	if(board->read_in_progress) {
		debug_print(DEBUG_CHAR, "  read(), concurrent read()s not allowed\n");
		mutex_unlock(&board->mutex);
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

	mutex_unlock(&board->mutex);

	rc = wait_event_interruptible_timeout(queue, irq_flag != 0,
		msecs_to_jiffies(500*(i+1)));

	mutex_lock(&board->mutex);
	irq_flag = 0;
	board->read_in_progress = 0;

	if (rc == 0) {
		debug_print(DEBUG_CHAR, "  read(): interrupt failed: %d\n", rc);
		/* reset DMA engine */
		dma_reset(board);
		bytes_trans = 0;
		count = 0;
	} else {
		debug_print(DEBUG_CHAR, "  read(): returned from sleep\n");
		i = 0;
		tmp_count = count;
		while (tmp_count > DMA_BUF_SIZE) {
			copy_to_user(buf + DMA_BUF_SIZE*i,
				board->kernel_mem_buf[i], DMA_BUF_SIZE);
			i++;
			tmp_count -= DMA_BUF_SIZE;
		}
		copy_to_user(buf + DMA_BUF_SIZE*i,
			board->kernel_mem_buf[i], tmp_count);
	}

	*pos += count;

	mutex_unlock(&board->mutex);
	return count;
}


long char_ioctl(
	struct file *filp,
	unsigned int cmd,
	unsigned long arg
)
{
	struct board_data *board;
	struct trg_ctrl trg;
	uint32_t control, scaler;
	uint8_t range;
	uint32_t rng_buf_delay;
	uint32_t ctrl_tmp;

	debug_print(DEBUG_CHAR, "%s\n", __PRETTY_FUNCTION__);
	debug_print(DEBUG_CHAR, "cmd: 0x%08x\n", cmd);

	board = (struct board_data *) filp->private_data;

	mutex_lock(&board->mutex);

	switch (cmd) {
	/* ================================================================== */
	case SET_RANGE:
		copy_from_user(&range, (void *)arg, 1);
		range &= 0xFF;
		debug_print(DEBUG_CHAR, "setting range: %01x\n", range);
		control = (ioread32(board->bar[0]));
		control &= ~0xFFUL; control |= range;
		iowrite32(control, board->bar[0]);
		debug_print(DEBUG_CHAR, "   control reg readback: %08x\n",
			ioread32(board->bar[0]));
		break;

	/* ================================================================== */
	case GET_RANGE:
		range = (uint8_t)((ioread32(board->bar[0])) & 0xFF);
		debug_print(DEBUG_CHAR, "   range: %x\n", range);
		copy_to_user((void *)arg, &range, sizeof(uint8_t));
		break;

	/* ================================================================== */
	case SET_FSAMP:
		copy_from_user(&scaler, (void *)arg, 4);

		/* convert freq to scaler (340MHz is clk freq of FPGA) */
		scaler = PICO_CLK_FREQ / scaler - 1;
		debug_print(DEBUG_CHAR, "clock scaler: %d\n", scaler);

		if ((scaler == 0) || scaler > PICO_CONV_MAX){
			printk(KERN_DEBUG MOD_NAME
				": prescaler out of range (%d > %d)\n",
				scaler, PICO_CONV_MAX);

			mutex_unlock(&board->mutex);
			return -1;
		}

		iowrite32((scaler), board->bar[0] + PICO_CONV_GEN);
		debug_print(DEBUG_CHAR, "   scaler readback: %08x\n",
			ioread32(board->bar[0] + PICO_CONV_GEN));
		break;

	/* ================================================================== */
	case GET_FSAMP:
		scaler = ioread32(board->bar[0] + PICO_CONV_GEN);
		scaler = PICO_CLK_FREQ / scaler;
		debug_print(DEBUG_CHAR, "   freq: %d", scaler);
		copy_to_user((void *)arg, &scaler, 4);
		break;

	/* ================================================================== */
	case GET_B_TRANS:
		copy_to_user((void *)arg, (void *)&bytes_trans,
			sizeof(uint32_t));
		break;

	/* ================================================================== */
	case SET_TRG:
		copy_from_user((void *)&trg, (void *)arg,
			sizeof(struct trg_ctrl));

		debug_print(DEBUG_FULL, "    trg.limit: %08x\n",
			*(uint32_t *)&trg.limit);
		debug_print(DEBUG_FULL, "    trg.nrsamp: %d\n", trg.nr_samp);
		debug_print(DEBUG_FULL, "    trg.ch_sel: %d\n", trg.ch_sel);
		debug_print(DEBUG_FULL, "    trg.mode: %s\n",
			(trg.mode == DISABLED ? "DISABLED" :
			trg.mode == POS_EDGE ?  "POS_EDGE" :
			trg.mode == NEG_EDGE ?  "NEG_EDGE" : "BOTH_EDGE"));

		iowrite32(*(uint32_t *)&trg.limit,
			board->bar[0] + PICO_ADDR + TRG_OFFS_LIMIT);
		iowrite32(trg.nr_samp,
			board->bar[0] + PICO_ADDR + TRG_OFFS_NRSAMP);

		ctrl_tmp = ioread32(board->bar[0] + PICO_ADDR + TRG_OFFS_CTRL);

		/* change the trigger edge */
		ctrl_tmp &= ~(0x3);
		ctrl_tmp |= trg.mode;

		/* change the channel bits */
		ctrl_tmp &= ~(0x7 << TRG_CTRL_CH_SHIFT);
		ctrl_tmp |= trg.ch_sel << TRG_CTRL_CH_SHIFT;

		iowrite32(ctrl_tmp, board->bar[0] + PICO_ADDR + TRG_OFFS_CTRL);
		debug_print(DEBUG_CHAR, "   trigger control: %08x\n",
			ioread32(board->bar[0] + PICO_ADDR + TRG_OFFS_CTRL));
		break;

	/* ================================================================== */
	case SET_RING_BUF:
		copy_from_user((void *)&rng_buf_delay, (void *)arg,
			sizeof(uint32_t));
		debug_print(DEBUG_FULL, "rng_buf_delay: %d\n", rng_buf_delay);
		iowrite32(rng_buf_delay,
			board->bar[0] + PICO_ADDR + RING_BUFF_OFFS_DELAY);
		break;

	/* ================================================================== */
	case SET_GATE_MUX:
		copy_from_user((void *)&control, (void *)arg, sizeof(uint32_t));
		debug_print(DEBUG_FULL, "set gate mux: %d\n", control);

		control &= MUX_TRG_MASK;
		control <<= MUX_TRG_SHIFT;

		ctrl_tmp = ioread32(board->bar[0] + PICO_ADDR + PICO_CONV_TRG);
		ctrl_tmp &= ~(MUX_TRG_MASK << MUX_TRG_SHIFT);
		ctrl_tmp |= control;

		iowrite32(ctrl_tmp, board->bar[0] + PICO_ADDR + PICO_CONV_TRG);

		debug_print(DEBUG_FULL, "cont_trg reg: %08x\n",
			ioread32(board->bar[0] + PICO_ADDR + PICO_CONV_TRG));
		break;

	/* ================================================================== */
	case SET_CONV_MUX:
		copy_from_user((void *)&control, (void *)arg, sizeof(uint32_t));
		debug_print(DEBUG_FULL, "set conv mux: %d\n", control);

		control &= MUX_CONV_MASK;
		control <<= MUX_CONV_SHIFT;

		ctrl_tmp = ioread32(board->bar[0] + PICO_ADDR + PICO_CONV_TRG);
		ctrl_tmp &= ~(MUX_CONV_MASK << MUX_CONV_SHIFT);
		ctrl_tmp |= control;

		iowrite32(ctrl_tmp, board->bar[0] + PICO_ADDR + PICO_CONV_TRG);

		debug_print(DEBUG_FULL, "cont_trg reg: %08x\n",
			ioread32(board->bar[0] + PICO_ADDR + PICO_CONV_TRG));
		break;

	/* ================================================================== */
	default:
		printk(KERN_ERR MOD_NAME ": unknown ioctl cmd: %08x\n", cmd);
		break;
	}

	mutex_unlock(&board->mutex);

	return 0;
}
