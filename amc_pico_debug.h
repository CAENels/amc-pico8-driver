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
 * \brief Debug printing control
 */

#ifndef AMC_PICO_DEBUG_H_
#define AMC_PICO_DEBUG_H_

#include "amc_pico_internal.h"

#ifndef DEBUG_IRQ
#define DEBUG_IRQ	0
#endif

#ifndef DEBUG_CHAR
#define DEBUG_CHAR	0
#endif

#ifndef DEBUG_SYS
#define DEBUG_SYS	0
#endif

#ifndef DEBUG_DMA
#define DEBUG_DMA	0
#endif

#ifndef DEBUG_FULL
#define DEBUG_FULL	0
#endif


#define debug_print(type, fmt, args...) \
	do { if (type) printk(KERN_DEBUG MOD_NAME ": " fmt, ##args); } \
	while (0);


#endif /* AMC_PICO_DEBUG_H_ */
