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

#define DRV_NAME "AMC-Pico8 Driver"

static
int version[3] = {1, 0, 7};

static
struct class *damc_fmc25_class;

/* allow DMA buffer size to be selected at load time.
 * May be reduced for testing
 */
static
unsigned long damc_req_dma_buf_len = 4*1024*1024;
module_param_named(dma_buf_len, damc_req_dma_buf_len, ulong, 0444);

unsigned long damc_dma_buf_len;

/** List of devices this driver recognizes */
static const struct pci_device_id ids[] = {
	{ .vendor = PCI_VENDOR_ID_XILINX, .device = 0x0007,
	  .subvendor = AMC_PICO_SUBVENDOR_ID, .subdevice = AMC_PICO_SUBDEVICE_ID
	},
	{ 0, }
};

MODULE_DEVICE_TABLE(pci, ids);

static irqreturn_t amc_isr(int irq, void *dev_id)
{
	struct board_data *board;
	uint32_t count = 0;
	size_t nsent = 0;
	unsigned long flags;
	unsigned cycles = 0;
	int op = 1;

	board = (struct board_data *)dev_id;

	if (board == NULL) {
		/* interrupt was not from this device */
		return IRQ_NONE;
	}

    count = (ioread32(board->bar0 + DMA_ADDR + DMA_OFFSET_STATUS) >> 16) & 0x7FF;

	dev_dbg(&board->pci_dev->dev, "ISR: irq: 0x%x %u\n", irq, (unsigned)count);

	if(count==0)
		return IRQ_NONE;

	do {
		if (count == 0xFFFFFFFFUL) {
			dev_err(&board->pci_dev->dev,
				"something wrong when reading from DMA\n");
			break;

		} else if (cycles++>100) {
			dev_err(&board->pci_dev->dev, "FIFO ran away, stopping\n");
			op = 2;
			break;
		}

        nsent += ioread32(board->bar0 + DMA_ADDR + DMA_OFFSET_RESP_LEN);
		dev_dbg(&board->pci_dev->dev, "   ISR: resp count: %08x\n", count);
		dev_dbg(&board->pci_dev->dev, "   ISR: resp len: %08x\n", (unsigned)nsent);
		dev_dbg(&board->pci_dev->dev, "   ISR: resp addr: %08x\n",
            ioread32(board->bar0 + DMA_ADDR + DMA_OFFSET_RESP_ADDR));

		/* pop from resp fifo */
        iowrite32(0, board->bar0 + DMA_ADDR + DMA_OFFSET_RESP_LEN);
		mb();
        count = (ioread32(board->bar0 + DMA_ADDR + DMA_OFFSET_STATUS) >> 16) & 0x7FF;
	} while (count > 0);

	spin_lock_irqsave(&board->queue.lock, flags);
	board->irq_flag = op;
	board->bytes_trans = nsent;
	wake_up_locked(&board->queue);
	spin_unlock_irqrestore(&board->queue.lock, flags);

	dev_dbg(&board->pci_dev->dev, "ISR: waked up queue\n");

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

	dev_info(&dev->dev, "probe()\n");

	/* Allocate memory for board structure */
	board = kzalloc(sizeof(struct board_data), GFP_KERNEL);
	if (!board) {
		return -ENOMEM;
	}

	board->pci_dev = dev;

	/* store our data (like global variable) */
	dev_set_drvdata(&dev->dev, board);

	init_waitqueue_head(&board->queue);

	/* Enable pci device */
	rc = pci_enable_device(dev);
	if (rc) {
		dev_err(&dev->dev, "pci_enable_device() failed\n");
		goto probe_free_board;
	}

	if (pci_request_regions(dev, DRV_NAME)) {
		dev_err(&dev->dev, "pci_request_regions failis\n");
		goto probe_disable_dev;
	}

	/* Enable bus mastering on device */
	pci_set_master(dev);

    board->bar0 = pci_ioremap_bar(dev, 0);

	/* check if we got BAR0 (all FPGA logic is there) */
    if (!board->bar0) {
		dev_err(&dev->dev, "BAR0 not available\n");
		goto probe_unmap_bars;
	}

	/* Our DMA supports only 32-bit addressing */
	rc = pci_set_dma_mask(dev, DMA_BIT_MASK(32));
	if (rc < 0) {
		dev_err(&dev->dev, "Problem setting DMA mask\n");
		goto probe_unmap_bars;
	}

	rc = pci_set_consistent_dma_mask(dev, DMA_BIT_MASK(32));
	if (rc < 0) {
		dev_err(&dev->dev, "Problem setting consistent DMA mask\n");
		goto probe_unmap_bars;
	}

	for (i = 0; i < DMA_BUF_COUNT; i++) {
		board->kernel_mem_buf[i] =
		pci_alloc_consistent(dev, DMA_BUF_SIZE, &board->dma_buf[i]);
		dev_dbg(&dev->dev, "pci_alloc() buf addr: %p",
			board->kernel_mem_buf[i]);
		dev_dbg(&dev->dev, "\tsize: %u, bus_addr: 0x%08llx\n",
			(unsigned)DMA_BUF_SIZE, board->dma_buf[i]);

		if (!board->dma_buf[i]) {
			dev_err(&dev->dev, "Problem allocating buffer %d, exiting\n", i);
			goto probe_free_bufs;
		}
	}

	dma_reset(board);

	if(pci_enable_msi(dev))
		goto probe_free_bufs;

	irq_line = dev->irq;

	/* Register interrupts */
	rc = request_irq(irq_line, amc_isr, IRQF_SHARED, MOD_NAME,
		(void *) board);

	if (rc) {
		dev_err(&dev->dev, "Could not request IRQ #%d, error %d\n",
		       irq_line, rc);
		board->irq_line = -1;
		goto disable_msi;
	}

	board->irq_line = irq_line;
	dev_dbg(&dev->dev, "Succesfully requested IRQ #%d\n", irq_line);

	/* perform build in self test (DMA transfer)
	 * before making device available to user processes
	 * to avoid having to deal with concurrency issues
	 */
	//dev_info(&dev->dev, "BIST %d\n", BIST(dev));

	/* Allocate a dynamically allocated character device node */
	rc = alloc_chrdev_region(&board->cdevno, 0, 1, MOD_NAME);

	/* check if allocation failed */
	if (rc < 0) {
		dev_err(&dev->dev, "alloc_chrdev_region() = %d\n", rc);
		goto disable_irq;
	}

	/* Add file_opperations structure to device */
	cdev_init(&board->cdev, &amc_pico_fops);
	board->cdev.owner = THIS_MODULE;

	/* Add char device */
	rc = cdev_add(&board->cdev, board->cdevno, 1);
	if (rc < 0) {
		dev_err(&dev->dev, "cdev_add() = %d\n", rc);
		goto probe_ureg_chrdev;
	}

	/* Create char device */
	cdev = device_create(damc_fmc25_class, &dev->dev, board->cdevno,
			NULL, MOD_NAME "_%s", pci_name(dev));

	/* output version and timestamp */
	dev_info(&dev->dev, "FPGA HW version = %08x\n",
        ioread32(board->bar0 + PICO_ADDR + FPGA_VER_OFFSET));
	dev_info(&dev->dev, "FPGA HW timestamp = %d\n",
        ioread32(board->bar0 + PICO_ADDR + FPGA_TS_OFFSET));

	/* probe was successful */
	return 0;


/* various exit paths when a fail occurs during probing */
probe_ureg_chrdev:
	unregister_chrdev_region(board->cdevno, 1);

disable_irq:
	free_irq(irq_line, (void *) board);

disable_msi:
	pci_disable_msi(dev);

probe_free_bufs:
	for (i = 0; i < DMA_BUF_COUNT; i++) {
		if (board->kernel_mem_buf[i]) {
			dev_dbg(&dev->dev, "Freeing DMA buffer at: %p\n",
				board->kernel_mem_buf[i]);

			pci_free_consistent(	dev,
						DMA_BUF_SIZE,
						board->kernel_mem_buf[i],
						board->dma_buf[i]);
		}
	}
probe_unmap_bars:
    pci_iounmap(dev, board->bar0);

	pci_release_regions(dev);

probe_disable_dev:
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

	dev_info(&dev->dev, " remove()\n");

	if (board->irq_line >= 0) {
		dev_info(&dev->dev, "irq_count: %d\n",board->irq_count);

		dev_dbg(&dev->dev, "Freeing IRQ #%d for dev_id 0x%08lx.\n",
		board->irq_line, (unsigned long)board);
		free_irq(board->irq_line, (void *)board);
	}

	/* Disable MSI */
	pci_disable_msi(dev);

	/* Remove char device */
	device_destroy(damc_fmc25_class, board->cdevno);
	cdev_del(&board->cdev);

	spin_lock_irq(&board->queue.lock);
	if(board->read_in_progress) {
		board->irq_flag = 2;
		wake_up_locked(&board->queue);
	}
	spin_unlock_irq(&board->queue.lock);

	unregister_chrdev_region(board->cdevno, 1);

	/* Remove buffer for DMA */
	for (i = 0; i < DMA_BUF_COUNT; i++) {
		dev_dbg(&dev->dev, "Freeing DMA buffer at: %p\n",
			board->kernel_mem_buf[i]);

		pci_free_consistent(	dev,
					DMA_BUF_SIZE,
					board->kernel_mem_buf[i],
					board->dma_buf[i]);
	}


	/* Unmap BARs */
    pci_iounmap(dev, board->bar0);

	pci_release_regions(dev);

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

	damc_dma_buf_len = damc_req_dma_buf_len;

	printk(KERN_DEBUG "===============================================\n");
	printk(KERN_DEBUG "              CAEN ELS AMC-PICO8               \n");
	printk(KERN_DEBUG "               version: %d.%d.%d               \n",
		version[0], version[1], version[2]);
	printk(KERN_DEBUG MOD_NAME " init(), built at "
		__DATE__ " " __TIME__ "\n");
	printk(KERN_DEBUG "===============================================\n");

	print_all_ioctls();

	damc_fmc25_class = class_create(THIS_MODULE, MOD_NAME);
	if(!damc_fmc25_class) return -ENOMEM;

	rc = pci_register_driver(&pci_driver);
	if(rc)
		class_destroy(damc_fmc25_class);
	return rc;
}

/**
 * \brief Removes driver from kernel
 */
static void __exit damc_fmc25_pcie_exit(void)
{
	printk(KERN_DEBUG MOD_NAME " exit()\n");
	pci_unregister_driver(&pci_driver);
	class_destroy(damc_fmc25_class);
}


module_init(damc_fmc25_pcie_init);
module_exit(damc_fmc25_pcie_exit);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jan Marjanovic <j.marjanovic@caenels.com>");
MODULE_DESCRIPTION(DRV_NAME);
