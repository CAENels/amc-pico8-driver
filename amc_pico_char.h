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
 * \brief Char device functions
 */

#ifndef AMC_PICO_CHAR_H_
#define AMC_PICO_CHAR_H_

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/pci.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <asm/uaccess.h>

#include "amc_pico_internal.h"
#include "amc_pico_regs.h"
#include "amc_pico_dma.h"
#include "amc_pico_debug.h"

/**
 * \brief Stores board struct in private data
 * \param inode   index node structure
 * \param file    file structure
 * \returns       0 on success, negative number on fail
 */
int char_open(struct inode *inode, struct file *file);


/**
 * \brief Reads the picoammeter measurement data
 * \param filp    file structure
 * \param buf     user space buffer
 * \param count   number of bytes to read
 * \param pos     position in file
 * \returns the number of bytes requested (parameter count), even if the
 * transfer was terminated early. The application should get the number
 * of bytes actually transfered using the ioctl(GET_B_TRANS).
 *
 * Reads the picoammeter measurement data using DMA in the FPGA. Each sample
 * is 32 bytes wide (8 channels x 4 bytes per channel (float)). This function
 * will put the calling thread to sleep until the DMA finishes the transfer.
 *
 */
ssize_t char_read(struct file *filp, char __user *buf, size_t count,
	loff_t *pos);

/**
 * \brief Performs various configuration tasks
 * \param filp    file structure
 * \param cmd     defines in amc_pico.h
 * \param arg     corresponding argument to define in amc_pico.h
 *
 * Performs various configuration tasks, such as setting the picoammeter range,
 * setting the trigger parameters and setting the sampling frequency.
 */
long char_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);


#endif /* AMC_PICO_CHAR_H_ */
