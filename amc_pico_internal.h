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


#ifndef AMC_PICO_INTERNAL_H_
#define AMC_PICO_INTERNAL_H_

#include <linux/mutex.h>
#include <linux/cdev.h>

#include "amc_pico.h"


/** Driver name (shows in lsmod and dmesg) */
#define MOD_NAME "amc_pico"


/** Maximal number of BARs */
#define PCIE_NR_BARS	7


/**
 * \struct board_data
 *
 * Keeps state of the PCIe core.
 */

struct board_data {
	/** the kernel pci device data structure provided by probe() */
	struct pci_dev *pci_dev; /* TODO: do we really need this? */

	/** kernel virtual address of the mapped BAR memory and IO regions of
	 *  the End Point. Used by map_bars()/unmap_bars().
	 */
	void * __iomem bar[PCIE_NR_BARS];

	struct mutex mutex;

	/* MSI interrupt TODO do we need this */
	uint8_t msi_enabled;

	/* number of interrupts */
	uint32_t irq_count;

	/** irq line */
	int irq_line;

	/**  */
	struct class *damc_fmc25_class;

	/** character device number */
	dev_t cdevno;

	/** character device */
	struct cdev cdev;

	/** pointer to DMA buffer for mSGDMA on FPGA */
	void *kernel_mem_buf[DMA_BUF_COUNT];

	/** physical address of buffers */
	dma_addr_t dma_buf[DMA_BUF_COUNT];

};

extern uint32_t bytes_trans;
extern wait_queue_head_t queue;
extern int irq_flag;

#endif /* AMC_PICO_INTERNAL_H_ */