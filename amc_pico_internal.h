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


#ifndef AMC_PICO_INTERNAL_H_
#define AMC_PICO_INTERNAL_H_

#include <linux/kobject.h>
#include <linux/spinlock.h>
#include <linux/cdev.h>
#include <linux/irqreturn.h>
#include <linux/device.h>
#include <linux/pci.h>
#include <linux/atomic.h>

#include "amc_pico.h"
#include "amc_pico_regs.h"


/** Driver name (shows in lsmod and dmesg) */
#define MOD_NAME "amc_pico"

/** Number of buffers allocated for DMA */
#define DMA_BUF_COUNT		(8)

/** Buffer size allocated (should be <= 4MB) */
#define DMA_BUF_SIZE		damc_dma_buf_len
extern unsigned long damc_dma_buf_len;

extern uint32_t dmac_site;

irqreturn_t amc_isr(int irq, void *dev_id);

enum dmac_irqmode_t {
    dmac_irq_poll,
    dmac_irq_level,
    dmac_irq_msi
};

/**
 * \struct board_data
 *
 * Keeps state of the PCIe core.
 */

struct board_data {
    /* our own kobj, so we may outlive cdev */
    struct kobject kobj;

	/** the kernel pci device data structure provided by probe() */
    struct pci_dev *pci_dev;

	/** kernel virtual address of the mapped BAR memory and IO regions of
	 *  the End Point. Used by map_bars()/unmap_bars().
	 */
	void * __iomem bar0;
	void * __iomem bar2;

    enum dmac_irqmode_t irqmode;

	/* number of interrupts */
    uint32_t irq_count;

	/** character device number */
	dev_t cdevno;

	/** character device */
	struct cdev cdev;

	/** pointer to DMA buffer for mSGDMA on FPGA */
	void *kernel_mem_buf[DMA_BUF_COUNT];

	/** physical address of buffers */
	dma_addr_t dma_buf[DMA_BUF_COUNT];

	unsigned read_in_progress;
    wait_queue_head_t dma_queue;
    unsigned dma_irq_flag;
    uint32_t dma_bytes_trans;

#ifdef CONFIG_AMC_PICO_FRIB
    unsigned capture_ready;
    unsigned capture_length;
    uint32_t *capture_buf;
    wait_queue_head_t capture_queue;
#endif

    atomic_t num_isr;
    cycles_t last_isr;
    cycles_t longest_isr;
};

#endif /* AMC_PICO_INTERNAL_H_ */
