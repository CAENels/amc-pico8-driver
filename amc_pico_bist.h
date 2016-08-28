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
 * \brief Built-In Self Test
 */

#ifndef AMC_PICO_BIST_H_
#define AMC_PICO_BIST_H_

#include <linux/kernel.h>
#include <linux/jiffies.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/pci.h>
#include <linux/hrtimer.h>
#include <linux/sched.h>

#include "amc_pico_internal.h"
#include "amc_pico_regs.h"
#include "amc_pico_dma.h"


/**
 * \brief Built-In Self Test routine
 * \param dev pci device with amc_pico board_data as private data
 * \return 0 if the test was successful, -1 if the DMA did not work correctly
 *
 * Performs a DMA transfer of 4 megabytes using the DMA on the FPGA while
 * measuring the time it takes. The DMA signals the interrupt when the
 * transfer finishes. The throughput is calculated from the time needed
 * to perform the transfer.
 */
int BIST(struct pci_dev *dev);


#endif /* AMC_PICO_BIST_H_ */
