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

#include <linux/version.h>
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
#include "amc_pico_char.h"
#include "amc_pico_bist.h"
#include "amc_pico_version.h"

#define DRV_NAME "AMC-Pico8 Driver"

#ifndef CONFIG_PCI_MSI
# error CONFIG_PCI_MSI is required
#endif

static
int version[3] = {1, 0, 7};

static
struct class *damc_fmc25_class;

/* allow DMA buffer size to be selected at load time.
 * May be reduced for testing.
 * Increasing this will at some point cause allocation failures
 * in probe().
 * The limit will be host specific.
 */
static
ulong damc_req_dma_buf_len = 4*1024*1024;
module_param_named(dma_buf_len, damc_req_dma_buf_len, ulong, 0444);

unsigned long damc_dma_buf_len;

/* 0 - polled  (debugging)
 * 1 - classic PCI level IRQ
 * 2 - PCI MSI
 */
uint dmac_irqmode = 2;
module_param_named(irqmode, dmac_irqmode, uint, 0444);

/* select source of site specific FW customization. */
static
char *dmac_site_name =
#ifdef CONFIG_AMC_PICO_SITE_DEFAULT
        CONFIG_AMC_PICO_SITE_DEFAULT
#else
        NULL
#endif
        ;
uint32_t dmac_site;
module_param_named(site, dmac_site_name, charp, 0444);


/** List of devices this driver recognizes */
static const struct pci_device_id ids[] = {
	{ .vendor = PCI_VENDOR_ID_XILINX, .device = 0x0007,
	  .subvendor = AMC_PICO_SUBVENDOR_ID, .subdevice = AMC_PICO_SUBDEVICE_ID
	},
	{ 0, }
};

MODULE_DEVICE_TABLE(pci, ids);

irqreturn_t amc_isr(int irq, void *dev_id)
{
    struct board_data *board;
    uint32_t active;

    board = (struct board_data *)dev_id;

    WARN_ONCE(board==NULL, "amc_pico ISR had board==NULL\n");
    if (board == NULL)
        return IRQ_NONE;

    active = ioread32(board->bar0+INT_READ);
    if(unlikely(active&~INT_MASK)) {
        /* Maybe some new FW feature has signaled an interrupt we don't know
         * how to handle, and can't mask out.
         * So clear it and hope for the best...
         */
        dev_warn(&board->pci_dev->dev, "Device signaling unknown IRQ %08x\n", (unsigned)active);
    }

    if(!active) {
        if (unlikely(board->irqmode==dmac_irq_msi)) {
            dev_warn(&board->pci_dev->dev, "Spurious IRQ in MSI mode %08x\n", (unsigned)active);
        }
        return IRQ_NONE;
    }

    if(active&INT_DMA_DONE) {
        size_t nsent = 0;
        unsigned long flags;
        unsigned cycles = 0;
        int op = 1;

        uint32_t count = (ioread32(board->bar0 + DMA_ADDR + DMA_OFFSET_STATUS) >> 16) & 0x7FF;

        dev_dbg(&board->pci_dev->dev, "ISR: irq: 0x%x %u\n", irq, (unsigned)count);

        if(count==0)
            return IRQ_NONE;

        do {
            if (unlikely(count == 0xFFFFFFFFUL)) {
                dev_err(&board->pci_dev->dev,
                        "something wrong when reading from DMA\n");
                break;

            } else if (unlikely(cycles++>100)) {
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

        spin_lock_irqsave(&board->dma_queue.lock, flags);
        board->dma_irq_flag = op;
        board->dma_bytes_trans = nsent;
        wake_up_locked(&board->dma_queue);
        spin_unlock_irqrestore(&board->dma_queue.lock, flags);

        dev_dbg(&board->pci_dev->dev, "ISR: waked up dma_queue\n");
    }
    if(active&INT_USER) {
        if(0) {}
#ifdef CONFIG_AMC_PICO_FRIB
        else if(dmac_site==USER_SITE_FRIB) {
            /* TODO: Note, being sloppy with locking here
             *  Not sure how to guard this since can't copy_to_user()
             *  with spinlock held.
             */
            uint32_t *buf = board->capture_buf;
            unsigned i;
            for(i=0; i<board->capture_length; i++) {
                *buf++ = ioread32(board->bar0 + USER_ADDR + FRIB_CAP_START + 4*i);
            }
            spin_lock_irq(&board->capture_queue.lock);
            board->capture_ready = 1;
            wake_up_locked(&board->capture_queue);
            spin_unlock_irq(&board->capture_queue.lock);
        }
#endif
    }

    iowrite32(active, board->bar0+INT_CLEAR);

    return IRQ_HANDLED;
}

static
int pico_pci_setup(struct pci_dev *dev, struct board_data *board)
{
#define ERR(COND, LBL, MSG, ...) if(COND) { dev_err(&dev->dev, MSG, ##__VA_ARGS__); goto LBL; }

    unsigned i;
    int ret;

    ret = pci_enable_device(dev);
    ERR(ret, done, "Failed to enable\n");

    ret = pci_request_regions(dev, DRV_NAME);
    ERR(ret, pcidisable, "Failed to configure BARs\n");

    ret = -EIO;

    board->bar0 = pci_ioremap_bar(dev, 0);
    ERR(!board->bar0, release, "Failed to map BAR0\n");

    board->bar2 = pci_ioremap_bar(dev, 2);
    ERR(!board->bar2, unmap0, "Failed to map BAR2\n");

    pci_set_master(dev);

    ret = pci_set_dma_mask(dev, DMA_BIT_MASK(32));
    if(!ret) ret = pci_set_consistent_dma_mask(dev, DMA_BIT_MASK(32));
    ERR(ret, unmap2, "Failed to set DMA masks\n");

    ret = -ENOMEM;
    for (i = 0; i < DMA_BUF_COUNT; i++) {
        board->kernel_mem_buf[i] = pci_alloc_consistent(dev, DMA_BUF_SIZE, &board->dma_buf[i]);
        ERR(!board->kernel_mem_buf[i], freebufs, "Failed to allocate DMA buffer %u\n", i);

        dev_dbg(&dev->dev, "pci_alloc() virt addr: %p\tsize: %u, phys addr: 0x%08llx\n",
            board->kernel_mem_buf[i], (unsigned)DMA_BUF_SIZE, board->dma_buf[i]);
    }

    if (board->irqmode==dmac_irq_msi) {
        ret = pci_enable_msi(dev);
        ERR(ret, freebufs, "Failed to enable any MSI interrupts\n");
    }

    if (board->irqmode!=dmac_irq_poll) {
        ret = request_irq(dev->irq, &amc_isr, 0, "pico_acq", board);
        ERR(ret, msidisable, "Failed to attach acquire ISR\n");
    }

    return 0;
//stopirq:
//    if (board->irqmode!=dmac_irq_poll) free_irq(dev->irq, &board);
msidisable:
    if (board->irqmode==dmac_irq_msi) pci_disable_msi(dev);
freebufs:
    for (i = 0; i < DMA_BUF_COUNT; i++) {
        if(!board->kernel_mem_buf[i]) continue;
        pci_free_consistent(	dev,
                    DMA_BUF_SIZE,
                    board->kernel_mem_buf[i],
                    board->dma_buf[i]);
    }
unmap2:
    pci_iounmap(dev, board->bar2);
unmap0:
    pci_iounmap(dev, board->bar0);
release:
    pci_release_regions(dev);
pcidisable:
    pci_disable_device(dev);
done:
    return ret;
#undef ERR
}

static
int pico_pci_cleanup(struct pci_dev *dev, struct board_data *board)
{
    unsigned i;
    if (board->irqmode!=dmac_irq_poll) {
        free_irq(dev->irq, board);
    }
    if (board->irqmode==dmac_irq_msi) {
        pci_disable_msi(dev);
    }
    for (i = 0; i < DMA_BUF_COUNT; i++) {
        if(!board->kernel_mem_buf[i]) continue;
        pci_free_consistent(	dev,
                    DMA_BUF_SIZE,
                    board->kernel_mem_buf[i],
                    board->dma_buf[i]);
    }

    pci_iounmap(dev, board->bar2);
    pci_iounmap(dev, board->bar0);

    pci_release_regions(dev);

    pci_disable_device(dev);
    return 0;
}

static
void pico_wait_for_op(struct board_data *board)
{
    spin_lock_irq(&board->dma_queue.lock);
    if(board->read_in_progress) {
        board->dma_irq_flag = 2;
        wake_up_locked(&board->dma_queue);
    }
    spin_unlock_irq(&board->dma_queue.lock);
}

static
int pico_cdev_setup(struct pci_dev *dev, struct board_data *board)
{
#define ERR(COND, LBL, MSG, ...) if(COND) { dev_err(&dev->dev, MSG, ##__VA_ARGS__); goto LBL; }

    struct device *cdev;
    int ret;

    ret = alloc_chrdev_region(&board->cdevno, 0, 1, MOD_NAME);
    ERR(ret, done, "Failed to allocate chrdev number\n");

    cdev_init(&board->cdev, &amc_pico_fops);
    board->cdev.owner = THIS_MODULE;

    ret = cdev_add(&board->cdev, board->cdevno, 1);
    ERR(ret, cfree, "Failed to add chrdev\n")

    cdev = device_create(damc_fmc25_class, &dev->dev, board->cdevno,
                         NULL, MOD_NAME "_%s", pci_name(dev));
    ret = -ENOMEM;
    ERR(IS_ERR(cdev), cdel, "Failed to allocate device\n");

    return 0;
//devdtor:
//    device_destroy(damc_fmc25_class, board->cdevno);
cdel:
    cdev_del(&board->cdev);
    pico_wait_for_op(board);
cfree:
    unregister_chrdev_region(board->cdevno, 1);
done:
    return ret;
#undef ERR
}

static
void pico_cdev_cleanup(struct pci_dev *dev, struct board_data *board)
{
    device_destroy(damc_fmc25_class, board->cdevno);
    cdev_del(&board->cdev);
    pico_wait_for_op(board);
    unregister_chrdev_region(board->cdevno, 1);
}

/**
 * \brief Claims control of PCI device
 * \param dev   PCI device (bus, ...)
 * \param id    Device data (vendor, device, subvendor, subdevice...)
 * \return      0 on success, negative on fail
 */

static int probe(struct pci_dev *dev, const struct pci_device_id *id)
{
    int ret;
    struct board_data *board = NULL;

	dev_info(&dev->dev, "probe()\n");

	/* Allocate memory for board structure */
	board = kzalloc(sizeof(struct board_data), GFP_KERNEL);
	if (!board) {
		return -ENOMEM;
	}

	board->pci_dev = dev;
    board->irqmode = dmac_irqmode;
    if (board->irqmode>2)
        board->irqmode = 2;

	/* store our data (like global variable) */
	dev_set_drvdata(&dev->dev, board);

    init_waitqueue_head(&board->dma_queue);

    ret = pico_pci_setup(dev, board);
    if(!ret) {
        dev_info(&dev->dev, "FPGA HW version = %08x\n",
            ioread32(board->bar0 + PICO_ADDR + FPGA_VER_OFFSET));
        dev_info(&dev->dev, "FPGA HW timestamp = %d\n",
            ioread32(board->bar0 + PICO_ADDR + FPGA_TS_OFFSET));

        dma_reset(board);

        ret = pico_cdev_setup(dev, board);
        if(ret) {
            pico_pci_cleanup(dev, board);
        }
    }
#ifdef CONFIG_AMC_PICO_FRIB
    if(dmac_site==USER_SITE_FRIB) {
        init_waitqueue_head(&board->capture_queue);

        board->capture_length = (0x6c-0x40)*4;
        board->capture_buf = kmalloc(4*board->capture_length, GFP_KERNEL);
        if(!board->capture_buf) {
            board->capture_length = 0;
            dev_err(&dev->dev, "FRIB capture buffer alloc fails.  Capture disabled.\n");
        }
    }
#endif
    return ret;
}

/**
 * \brief  Cleans PCI device things
 * \param	dev	PCI device (bus, ...)
 */

static void remove(struct pci_dev *dev)
{
	struct board_data *board = dev_get_drvdata(&dev->dev);

#ifdef CONFIG_AMC_PICO_FRIB
    kfree(board->capture_buf);
#endif
	dev_info(&dev->dev, " remove()\n");
    pico_cdev_cleanup(dev, board);
    pico_pci_cleanup(dev, board);

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

    if(!dmac_site_name || dmac_site_name[0]=='\0' || strcmp(dmac_site_name, "caen")==0) {
        dmac_site = USER_SITE_NONE;
#ifdef CONFIG_AMC_PICO_FRIB
    } else if(strcmp(dmac_site_name, "frib")==0) {
        dmac_site = USER_SITE_FRIB;
#endif
    } else {
        printk(KERN_ERR "amc_pico has not site '%s'\n", dmac_site_name);
        return -EINVAL;
    }

	damc_dma_buf_len = damc_req_dma_buf_len;

	printk(KERN_DEBUG "===============================================\n");
	printk(KERN_DEBUG "              CAEN ELS AMC-PICO8               \n");
	printk(KERN_DEBUG "               version: %d.%d.%d               \n",
		version[0], version[1], version[2]);
    printk(KERN_DEBUG MOD_NAME " init(), built " AMC_PICO_VERSION "\n");
#ifdef CONFIG_AMC_PICO_FRIB
    printk(KERN_DEBUG "Includes \"frib\" site FW support.\n");
#endif
#ifdef CONFIG_AMC_PICO_SITE_DEFAULT
    printk(KERN_DEBUG "Defaults to " CONFIG_AMC_PICO_SITE_DEFAULT " site\n");
#endif
    if(dmac_site!=USER_SITE_NONE)
        printk(KERN_DEBUG "Enabling site mods for \"%s\"\n", dmac_site_name);
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
