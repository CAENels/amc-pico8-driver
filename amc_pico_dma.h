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


/**
 * \file
 * \brief FPGA DMA functions
 *
 * The AMC-Pico8 Virtex-5 FPGA includes Scatter-Gather DMA, which should
 * be used to perform the transfers of measurement data.
 *
 * The DMA consists of:
 *     * DMA engine (performs data transfer),
 *     * command and response FIFOs (queue the commands and responses from DMA
 *       engine) and
 *     * register module, which enables PCIe subsystem to interface with DMA.
 *
 * If/when DMA engine is enabled and the command FIFO is not empty, it will
 * perform the transfer and signal the interrupt in two cases:
 *     * when the number of bytes has been successfully transfered or
 *     * if the DMA was stopped by hardware (e.g. at the end of the trigger
 *       window).
 *
 * The number of bytes transferred is saved in the response FIFO and it is made
 * available to the application.
 */

#ifndef AMC_PICO_DMA_H_
#define AMC_PICO_DMA_H_

#include <linux/kernel.h>
#include <linux/module.h>

#include "amc_pico.h"
#include "amc_pico_internal.h"
#include "amc_pico_regs.h"


/**
 * \brief Pushes new command to DMA command FIFO
 * \param board    amc_pico board_data
 * \param address  bus address (=physical addr on x86) of the memory buffer
 * \param length   length in bytes
 * \param gen_irq  selects if the transfer generates interrupt
 *
 * Pushes a new command (address, length and flag to generate irq when done)
 * to DMA FIFO.
 */

void dma_push(struct board_data *board, uint32_t address, uint32_t length, int gen_irq);

/**
 * \brief Enables or pauses DMA engine
 * \param board     amc_pico board_data
 * \param enable    1 to enable DMA machine, 0 to pause it
 *
 */

void dma_enable(struct board_data *board, int enable);

/**
 * \brief Resets the DMA engine
 * \param board    amc_pico board_data
 *
 */
void dma_reset(struct board_data *board);


#endif /* AMC_PICO_DMA_H_ */
