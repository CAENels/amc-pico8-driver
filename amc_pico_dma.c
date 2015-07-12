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

 #include "amc_pico_dma.h"


void dma_push(struct board_data *dev, uint32_t address, uint32_t length)
{
	iowrite32(address, dev->bar[0] + DMA_ADDR + DMA_OFFSET_ADDR);
	debug_print(DEBUG_DMA,  "   dma_start(): DMA address readback: %08x\n",
		ioread32(dev->bar[0] + DMA_ADDR + DMA_OFFSET_ADDR));

	iowrite32(length, dev->bar[0] + DMA_ADDR + DMA_OFFSET_LEN);
	debug_print(DEBUG_DMA,  "   dma_start(): DMA length readback: %08x\n",
		ioread32(dev->bar[0] + DMA_ADDR + DMA_OFFSET_LEN));

	/* make sure that address and length have been written */
	mb();
	debug_print(DEBUG_DMA,  "   dma_start(): DMA command go!\n");
	iowrite32(DMA_CMD_MASK_DMA_GO | DMA_CMD_MASK_GEN_IRQ,
		dev->bar[0] + DMA_ADDR + DMA_OFFSET_CMD);
}


void dma_enable(struct board_data *dev, int enable)
{
	uint32_t ctrl;

	ctrl = ioread32(dev->bar[0] + DMA_ADDR + DMA_OFFSET_CONTROL);
	debug_print(DEBUG_DMA, "   dma_enable(): ctrl read: 0x%08x\n", ctrl);

	if (enable)
		ctrl |= DMA_CTRL_MASK_ENABLE;
	else
		ctrl &= ~DMA_CTRL_MASK_ENABLE;

	iowrite32(ctrl, dev->bar[0] + DMA_ADDR + DMA_OFFSET_CONTROL);
}


void dma_reset(struct board_data *dev)
{
	iowrite32(DMA_CTRL_MASK_RESET,
		dev->bar[0] + DMA_ADDR + DMA_OFFSET_CONTROL);

	/* force write before continuing */
	mb();
}