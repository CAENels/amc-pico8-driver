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


#include "amc_pico_bist.h"

/* time measurement */
static ktime_t t0;
static ktime_t t1;


int BIST(struct pci_dev *dev)
{
	const int buf_size = 4*1024*1024;
	struct board_data *board = NULL;
	void *tmp_buf = NULL;
	dma_addr_t tmp_buf_dma;
	uint32_t mux_tmp;
	uint64_t time_needed, throughput;
	int rc;

	board = dev_get_drvdata(&dev->dev);

	printk(KERN_DEBUG MOD_NAME ": Performing BIST routine...\n");

	/* allocate buffer */
	tmp_buf = pci_alloc_consistent(dev, buf_size, &tmp_buf_dma);

	/* read mux setting */
	mux_tmp = ioread32(board->bar[0] + MUX_ADDR);

	/* mux to PRBS */
	iowrite32(1, board->bar[0] + MUX_ADDR);

	/* DMA transfer */
	irq_flag = 0;
	t0 = ktime_get();
	dma_enable(board, 0);
	dma_push(board, tmp_buf_dma, buf_size, 1);
	dma_enable(board, 1);
	rc = wait_event_interruptible_timeout(queue, irq_flag != 0,
		msecs_to_jiffies(1000));
	irq_flag = 0;
	t1 = ktime_get();
	dma_enable(board, 0);

	if (rc == 0) {
		printk(KERN_ALERT MOD_NAME ": DMA was unable to finish\n");
		rc = -1;
	} else {
		time_needed = ktime_to_ns(ktime_sub(t1, t0));

		/* 953 = ns->s, b->Mb */
		throughput = (buf_size*8)/(time_needed/953);

		printk(KERN_DEBUG MOD_NAME ":  DMA trans size: %d bytes\n",
			buf_size);
		printk(KERN_DEBUG MOD_NAME ":  DMA trans time: %llu ns\n",
			time_needed);
		printk(KERN_DEBUG MOD_NAME ":  DMA throughput: %llu Mbps\n",
			throughput);
		rc = 0;
	}

	/* return mux to previous value */
	iowrite32(mux_tmp, board->bar[0] + MUX_ADDR);

	/* free buffer */
	pci_free_consistent(dev, buf_size, tmp_buf, tmp_buf_dma);

	return rc;
}
