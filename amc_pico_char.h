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

extern const struct file_operations amc_pico_fops;

struct file_data {
    struct board_data *board;

    unsigned site_mode;
};

#endif /* AMC_PICO_CHAR_H_ */
