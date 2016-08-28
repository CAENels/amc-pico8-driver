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
 * \brief User-space interface
 */

#ifndef AMC_PICO_H_
#define AMC_PICO_H_

#include <asm/ioctl.h>
#include <linux/types.h>

#ifndef __KERNEL__
#include <stdint.h>
#endif


/** AMC-Pico magic number */
#define	AMC_PICO_MAGIC	'P'

/** Structure for trigger control */
struct __attribute__((__packed__)) trg_ctrl {
	float limit;
	uint32_t nr_samp;
	uint32_t ch_sel;
	enum mode_t {DISABLED, POS_EDGE, NEG_EDGE, BOTH_EDGE} mode;
};

/* Driver interface version number.
 * GET_VERSION_CURRENT is current version.
 * Initialize integer w/ zero to identify version zero interface
 * which does not fail unknown requests (errno==EINVAL implies version zero)
 @code
  int ver = 0;
  ioctl(fd, GET_VERSION, &ver);
  if(ver!=GET_VERSION_CURRENT) { oops ... }
 @endcode
 */
#define GET_VERSION	_IOR(AMC_PICO_MAGIC, 10, uint32_t)
#define GET_VERSION_CURRENT 3

/** Sets the picoammeter range, each bit sets the individual channel,
 * RNG0 is the higher current range
 */
#define SET_RANGE	_IOWR(AMC_PICO_MAGIC, 11, uint8_t)

/** Gets picoammeter range */
#define GET_RANGE	_IOR(AMC_PICO_MAGIC, 14, uint8_t)

/** Sets the picoammeter sampling frequency (in Hz) */
#define SET_FSAMP	_IOWR(AMC_PICO_MAGIC, 12, uint32_t)

/** Gets the picoammeter sampling frequency (in Hz) */
#define GET_FSAMP	_IOR(AMC_PICO_MAGIC, 13, uint32_t)

/** Gets number of bytes last DMA transfer succesfully transfered */
#define GET_B_TRANS	_IOR(AMC_PICO_MAGIC, 40, uint32_t)

/** Sets trigger parameters */
#define SET_TRG		_IOWR(AMC_PICO_MAGIC, 50, struct trg_ctrl)

/** Sets ring buffer (pre-trigger storage) parameters */
#define SET_RING_BUF	_IOW(AMC_PICO_MAGIC, 60,  uint32_t)

/** Sets gate mux */
#define SET_GATE_MUX	_IOWR(AMC_PICO_MAGIC, 70,  uint32_t)

/** Sets convert signal mux */
#define SET_CONV_MUX	_IOWR(AMC_PICO_MAGIC, 80,  uint32_t)

/** Abort in progress read() w/o closing FD */
#define ABORT_READ _IO(AMC_PICO_MAGIC, 81)

#define USER_SITE_NONE 0xcae4
#define USER_SITE_FRIB 0xf41B

/** Identify user/site firmware customization */
#define GET_SITE_ID _IOR(AMC_PICO_MAGIC, 91, uint32_t)
#define GET_SITE_VERSION _IOR(AMC_PICO_MAGIC, 92, uint32_t)
#define SET_SITE_MODE _IOW(AMC_PICO_MAGIC, 92, uint32_t)

#endif /* AMC_PICO_H_ */
