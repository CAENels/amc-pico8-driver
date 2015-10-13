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

/**
 * \file
 * \brief Register the module with PCIe subsytem
 */

#include <linux/kernel.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/dma-mapping.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <asm/io.h>
#include <asm/ioctl.h>

#include "amc_pico.h"
#include "amc_pico_regs.h"
#include "amc_pico_dma.h"
#include "amc_pico_debug.h"
#include "amc_pico_char.h"
#include "amc_pico_bist.h"


int version[3] = {1, 0, 7};
uint32_t bytes_trans;
DECLARE_WAIT_QUEUE_HEAD(queue);
int irq_flag;

/** List of devices this driver recognizes */
static const struct pci_device_id ids[] = {
	{ PCI_DEVICE(0x10ee, 0x0007), },
	{ 0, }
};

MODULE_DEVICE_TABLE(pci, ids);


static const struct file_operations fops = {
	.owner		= THIS_MODULE,
	.open		= char_open,
	.read		= char_read,
	.unlocked_ioctl = char_ioctl
};


static irqreturn_t amc_isr(int irq, void *dev_id)
{
	struct board_data *board;
	uint32_t count = 0;

	debug_print(DEBUG_IRQ, "ISR: irq: 0x%x\n", irq);

	board = (struct board_data *)dev_id;

	if (board == NULL) {
		/* interrupt was not from this device */
		return IRQ_NONE;
	}

	/* mb(); */
	bytes_trans = 0;

	do {
		count = ioread32(board->bar[0] + DMA_ADDR + DMA_OFFSET_STATUS);
		count = (count >> 16) & 0x7FF;

		if (count == 0) {
			debug_print(DEBUG_IRQ, "ISR: done with resp queue\n");
			break;
		}

		if (count == 0xFFFFFFFFUL) {
			printk(KERN_ERR MOD_NAME
				": something wrong when reading from DMA\n");
			break;
		}

		bytes_trans += ioread32(board->bar[0] + DMA_ADDR + DMA_OFFSET_RESP_LEN);
		debug_print(DEBUG_IRQ, "   ISR: resp count: %08x\n", count);
		debug_print(DEBUG_IRQ, "   ISR: resp len: %08x\n", bytes_trans);
		debug_print(DEBUG_IRQ, "   ISR: resp addr: %08x\n",
			ioread32(board->bar[0] + DMA_ADDR + DMA_OFFSET_RESP_ADDR));

		/* pop from resp fifo */
		iowrite32(0, board->bar[0] + DMA_ADDR + DMA_OFFSET_RESP_LEN);
		mb();
	} while (count > 0);

	irq_flag = 1;
	wake_up(&queue);
	debug_print(DEBUG_IRQ, "ISR: waked up queue\n");

	return IRQ_HANDLED;
}


/**
 * \brief Claims control of PCI device
 * \param dev   PCI device (bus, ...)
 * \param id    Device data (vendor, device, subvendor, subdevice...)
 * \return      0 on success, negative on fail
 */

static int probe(struct pci_dev *dev, const struct pci_device_id *id)
{
	int rc, i;
	struct board_data *board = NULL;
	int irq_line;
	struct device *cdev;

	printk(KERN_DEBUG MOD_NAME " probe()\n");
	printk(KERN_DEBUG MOD_NAME " subvendor id: %04x\n",
		dev->subsystem_vendor);
	printk(KERN_DEBUG MOD_NAME " subdevice id: %04x\n",
		dev->subsystem_device);

	if( (dev->subsystem_vendor != AMC_PICO_SUBVENDOR_ID)
			&& (dev->subsystem_device != AMC_PICO_SUBDEVICE_ID)){
		printk(KERN_DEBUG MOD_NAME " This device is not AMC-Pico8\n");
		printk(KERN_DEBUG MOD_NAME " we will not claim it.\n");
		return -1;
	}


	/* Allocate memory for board structure */
	board = kzalloc(sizeof(struct board_data), GFP_KERNEL);
	if (!board) {
		printk(KERN_DEBUG MOD_NAME " kzalloc() failed.\n");
		return -1;
	}

	/* store our data (like global variable) */
	dev_set_drvdata(&dev->dev, board);

	/* initialize mutex */
	mutex_init(&board->mutex);

	/* Enable pci device */
	rc = pci_enable_device(dev);
	if (rc) {
		printk(KERN_DEBUG MOD_NAME ": pci_enable_device() failed\n");
		goto probe_free_board;
	}

	/* Enable bus mastering on device */
	pci_set_master(dev);

	/* Enable MSI interrupts */
	rc = pci_enable_msi(dev);
	if (rc) {
		printk(KERN_DEBUG MOD_NAME ": unable to use MSI interrupts\n");
		board->msi_enabled = 0;
	} else {
		board->msi_enabled = 1;
	}

	irq_line = dev->irq;

	/* Register interrupts */
	rc = request_irq(irq_line, amc_isr, IRQF_SHARED, MOD_NAME,
		(void *) board);

	if (rc) {
		printk(KERN_DEBUG MOD_NAME ": Could not request IRQ #%d, error %d\n",
		       irq_line, rc);
		board->irq_line = -1;
		goto probe_disable_dev;
	}

	board->irq_line = irq_line;
	printk(KERN_DEBUG MOD_NAME ":Succesfully requested IRQ #%d\n", irq_line);

	/* Scan BARs */
	for (i = 0; i < PCIE_NR_BARS; i++) {
		unsigned long bar_start = pci_resource_start(dev, i);

		if (bar_start) {
			unsigned long bar_end = pci_resource_end(dev, i);
			unsigned long bar_flags = pci_resource_flags(dev, i);
			/* unsigned long bar_len = bar_end - bar_start + 1; */
			debug_print(DEBUG_SYS,
				"BAR%d 0x%08lx-0x%08lx flags 0x%08lx\n",
				i, bar_start, bar_end, bar_flags);

			board->bar[i] = pci_ioremap_bar(dev, i);
			debug_print(DEBUG_SYS, "BAR%d mapping: %p\n",
			        i, board->bar[i]);
		}
	}

	/* check if we got BAR0 (all FPGA logic is there) */
	if (!board->bar[0]) {
		printk(KERN_DEBUG MOD_NAME ": BAR0 not available, exiting\n");
		goto probe_unmap_bars;
	}

	/* Our DMA supports only 32-bit addressing */
	rc = pci_set_dma_mask(dev, DMA_BIT_MASK(32));
	if (rc < 0) {
		printk(KERN_DEBUG MOD_NAME
			": Problem setting DMA mask\n");
		goto probe_unmap_bars;
	}

	rc = pci_set_consistent_dma_mask(dev, DMA_BIT_MASK(32));
	if (rc < 0) {
		printk(KERN_DEBUG MOD_NAME
			": Problem setting consistent DMA mask\n");
		goto probe_unmap_bars;
	}

	for (i = 0; i < DMA_BUF_COUNT; i++) {
		board->kernel_mem_buf[i] =
		pci_alloc_consistent(dev, DMA_BUF_SIZE, &board->dma_buf[i]);
		debug_print(DEBUG_SYS, "pci_alloc() buf addr: %p",
			board->kernel_mem_buf[i]);
		debug_print(DEBUG_SYS, "\tsize: %d, bus_addr: 0x%08llx\n",
			DMA_BUF_SIZE, board->dma_buf[i]);

		if (!board->dma_buf[i]) {
			printk(KERN_DEBUG MOD_NAME
				": Problem allocating buffer %d, exiting\n", i);
			goto probe_free_bufs;
		}
	}

	/* Create master class */
	board->damc_fmc25_class = class_create(THIS_MODULE, MOD_NAME);

	debug_print(DEBUG_CHAR, "%s char_init()\n", MOD_NAME);

	/* Allocate a dynamically allocated character device node */
	rc = alloc_chrdev_region(&board->cdevno, 0, 1, MOD_NAME);

	/* check if allocation failed */
	if (rc < 0) {
		printk(KERN_ERR MOD_NAME ": alloc_chrdev_region() = %d\n", rc);
		goto probe_free_bufs;
	}

	/* Add file_opperations structure to device */
	cdev_init(&board->cdev, &fops);
	board->cdev.owner = THIS_MODULE;

	/* Add char device */
	rc = cdev_add(&board->cdev, board->cdevno, 1);
	if (rc < 0) {
		printk(KERN_ERR MOD_NAME ": cdev_add() = %d\n", rc);
		goto probe_ureg_chrdev;
	}

	/* Create char device */
	cdev = device_create(board->damc_fmc25_class, NULL, board->cdevno,
		NULL, MOD_NAME);

	/* output version and timestamp */
	printk(KERN_DEBUG MOD_NAME ": FPGA HW version = %08x\n",
		ioread32(board->bar[0] + PICO_ADDR + FPGA_VER_OFFSET));
	printk(KERN_DEBUG MOD_NAME ": FPGA HW timestamp = %d\n",
		ioread32(board->bar[0] + PICO_ADDR + FPGA_TS_OFFSET));

	/* perform build in self test (DMA transfer) */
	//BIST(dev);

	/* probe was successful */
	return 0;


/* various exit paths when a fail occurs during probing */
probe_ureg_chrdev:
	unregister_chrdev_region(board->cdevno, 1);

probe_free_bufs:
	for (i = 0; i < DMA_BUF_COUNT; i++) {
		if (board->kernel_mem_buf[i]) {
			debug_print(DEBUG_SYS, "Freeing DMA buffer at: %p\n",
				board->kernel_mem_buf[i]);

			pci_free_consistent(	dev,
						DMA_BUF_SIZE,
						board->kernel_mem_buf[i],
						board->dma_buf[i]);
		}
	}
probe_unmap_bars:
	for (i = 0; i < PCIE_NR_BARS; i++) {
		if (board->bar[i]) {
			pci_iounmap(dev, board->bar[i]);
			board->bar[i] = NULL;
		}
	}

probe_disable_dev:
	free_irq(board->irq_line, (void *)board);
	if (board->msi_enabled) {
		pci_disable_msi(dev);
		board->msi_enabled = 0;
	}
	pci_disable_device(dev);

probe_free_board:
	kfree(board);

	return -1;
}

/**
 * \brief  Cleans PCI device things
 * \param	dev	PCI device (bus, ...)
 */

static void remove(struct pci_dev *dev)
{
	int i;
	struct board_data *board = dev_get_drvdata(&dev->dev);

	printk(KERN_DEBUG MOD_NAME " remove()\n");

	if (board->irq_line >= 0) {
		printk(KERN_DEBUG "irq_count: %d\n",board->irq_count);

		printk(KERN_DEBUG "Freeing IRQ #%d for dev_id 0x%08lx.\n",
			board->irq_line, (unsigned long)board);
			free_irq(board->irq_line, (void *)board);
	}

	/* Disable MSI */
	if (board->msi_enabled) {
		pci_disable_msi(dev);
		board->msi_enabled = 0;
	}

	/* Remove char device */
	device_destroy(board->damc_fmc25_class, board->cdevno);
	cdev_del(&board->cdev);
	unregister_chrdev_region(board->cdevno, 1);

	/* Remove class */
	class_destroy(board->damc_fmc25_class);

	/* Remove buffer for DMA */
	for (i = 0; i < DMA_BUF_COUNT; i++) {
		debug_print(DEBUG_SYS, "Freeing DMA buffer at: %p\n",
			board->kernel_mem_buf[i]);

		pci_free_consistent(	dev,
					DMA_BUF_SIZE,
					board->kernel_mem_buf[i],
					board->dma_buf[i]);
	}


	/* Unmap BARs */
	for (i = 0; i < PCIE_NR_BARS; i++) {
		if (board->bar[i]) {
			pci_iounmap(dev, board->bar[i]);
			board->bar[i] = NULL;
		}
	}

	/* Disable device */
	pci_disable_device(dev);

	/* Free allocated memory */
	kfree(board);
}


/* PCI driver structure */
static struct pci_driver pci_driver = {
	.name = MOD_NAME,
	.id_table = ids,
	.probe = probe,
	.remove = remove,
};


void print_all_ioctls(void){
	printk(KERN_DEBUG MOD_NAME
		": supported IOCTL: SET_RANGE = 0x%08x\n", (unsigned int)SET_RANGE);
	printk(KERN_DEBUG MOD_NAME
		": supported IOCTL: GET_RANGE = 0x%08x\n", (unsigned int)GET_RANGE);
	printk(KERN_DEBUG MOD_NAME
		": supported IOCTL: SET_FSAMP = 0x%08x\n", (unsigned int)SET_FSAMP);
	printk(KERN_DEBUG MOD_NAME
		": supported IOCTL: GET_FSAMP = 0x%08x\n", (unsigned int)GET_FSAMP);
	printk(KERN_DEBUG MOD_NAME
		": supported IOCTL: GET_B_TRANS = 0x%08x\n", (unsigned int)GET_B_TRANS);
	printk(KERN_DEBUG MOD_NAME
		": supported IOCTL: SET_TRG   = 0x%08x\n", (unsigned int)SET_TRG);
	printk(KERN_DEBUG MOD_NAME
		": supported IOCTL: SET_RING_BUF = 0x%08x\n", (unsigned int)SET_RING_BUF);
	printk(KERN_DEBUG MOD_NAME
		": supported IOCTL: SET_GATE_MUX = 0x%08x\n", (unsigned int)SET_GATE_MUX);
	printk(KERN_DEBUG MOD_NAME
		": supported IOCTL: SET_CONV_MUX = 0x%08x\n", (unsigned int)SET_CONV_MUX);
}

/**
 * \brief Registers driver to kernel
 * \returns 0 on success, negative on fail
 */

static int __init damc_fmc25_pcie_init(void)
{
	int rc = 0;

	printk(KERN_DEBUG "===============================================\n");
	printk(KERN_DEBUG "              CAEN ELS AMC-PICO8               \n");
	printk(KERN_DEBUG "               version: %d.%d.%d               \n",
		version[0], version[1], version[2]);
	printk(KERN_DEBUG MOD_NAME " init(), built at "
		__DATE__ " " __TIME__ "\n");
	printk(KERN_DEBUG "===============================================\n");

	print_all_ioctls();

	rc = pci_register_driver(&pci_driver);
	return rc;
}

/**
 * \brief Removes driver from kernel
 */
static void __exit damc_fmc25_pcie_exit(void)
{
	printk(KERN_DEBUG MOD_NAME " exit()\n");
	pci_unregister_driver(&pci_driver);
}


module_init(damc_fmc25_pcie_init);
module_exit(damc_fmc25_pcie_exit);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jan Marjanovic <j.marjanovic@caenels.com>");
MODULE_DESCRIPTION("AMC-Pico8 Driver");
