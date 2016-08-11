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

 #include "amc_pico_dma.h"

void dma_push(struct board_data *dev, uint32_t address, uint32_t length, int gen_irq)
{
	iowrite32(address, dev->bar0 + DMA_ADDR + DMA_OFFSET_ADDR);
    dev_dbg(&dev->pci_dev->dev,  "   dma_start(): DMA address readback: %08x\n",
		ioread32(dev->bar0 + DMA_ADDR + DMA_OFFSET_ADDR));

	iowrite32(length, dev->bar0 + DMA_ADDR + DMA_OFFSET_LEN);
    dev_dbg(&dev->pci_dev->dev,  "   dma_start(): DMA length readback: %08x\n",
		ioread32(dev->bar0 + DMA_ADDR + DMA_OFFSET_LEN));

	/* make sure that address and length have been written */
	mb();
    dev_dbg(&dev->pci_dev->dev,  "   dma_start(): DMA command go%s!\n",
		gen_irq ? ", gen irq" : "");
	iowrite32(DMA_CMD_MASK_DMA_GO  | (gen_irq ? DMA_CMD_MASK_GEN_IRQ : 0 ),
		dev->bar0 + DMA_ADDR + DMA_OFFSET_CMD);
}


void dma_enable(struct board_data *dev, int enable)
{
	uint32_t ctrl = enable ? DMA_CTRL_MASK_ENABLE : 0;


	iowrite32(ctrl, dev->bar0 + DMA_ADDR + DMA_OFFSET_CONTROL);

/** Register access during DMA sometimes trigger hard lockup of device.
 *  The following is a great way to trigger this.
 *  Resolved w/ DMA controller FW fix in Jan 2016
	ctrl = ioread32(dev->bar0 + DMA_ADDR + DMA_OFFSET_CONTROL);
    dev_dbg(&dev->pci_dev->dev, "   dma_enable(): ctrl read: 0x%08x\n", ctrl);
 */
}


void dma_reset(struct board_data *dev)
{
	iowrite32(DMA_CTRL_MASK_RESET,
		dev->bar0 + DMA_ADDR + DMA_OFFSET_CONTROL);

	/* force write before continuing */
	mb();
}
