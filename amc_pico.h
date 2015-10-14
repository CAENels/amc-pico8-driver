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
 * \brief User-space interface
 */

#ifndef AMC_PICO_H_
#define AMC_PICO_H_

#include <asm/ioctl.h>
#include <linux/types.h>

#ifndef __KERNEL__
#include <stdint.h>
#endif


/** Number of buffers allocated for DMA */
#define DMA_BUF_COUNT		(8)

/** Buffer size allocated (should be <= 4MB) */
#define DMA_BUF_SIZE		(4*1024*1024)

/** AMC-Pico magic number */
#define	AMC_PICO_MAGIC	'P'

/** Structure for trigger control */
struct __attribute__((__packed__)) trg_ctrl {
	float limit;
	uint32_t nr_samp;
	uint32_t ch_sel;
	enum {DISABLED, POS_EDGE, NEG_EDGE, BOTH_EDGE} mode;
};


/** Sets the picoammeter range, each bit sets the individual channel,
 * RNG0 is the higher current range
 */
#define SET_RANGE	_IOW(AMC_PICO_MAGIC, 11, uint8_t*)

/** Gets picoammeter range */
#define GET_RANGE	_IOR(AMC_PICO_MAGIC, 11, uint8_t*)

/** Sets the picoammeter sampling frequency (in Hz) */
#define SET_FSAMP	_IOW(AMC_PICO_MAGIC, 12, uint32_t*)

/** Gets the picoammeter sampling frequency (in Hz) */
#define GET_FSAMP	_IOR(AMC_PICO_MAGIC, 12, uint32_t*)

/** Gets number of bytes last DMA transfer succesfully transfered */
#define GET_B_TRANS	_IOR(AMC_PICO_MAGIC, 40, uint32_t*)

/** Sets trigger parameters */
#define SET_TRG		_IOW(AMC_PICO_MAGIC, 50, struct trg_ctrl *)

/** Sets ring buffer (pre-trigger storage) parameters */
#define SET_RING_BUF	_IOW(AMC_PICO_MAGIC, 60,  uint32_t*)

/** Sets gate mux */
#define SET_GATE_MUX	_IOW(AMC_PICO_MAGIC, 70,  uint32_t*)

/** Sets convert signal mux */
#define SET_CONV_MUX	_IOW(AMC_PICO_MAGIC, 80,  uint32_t*)

/** Abort in progress read() w/o closing FD */
#define ABORT_READ _IOW(AMC_PICO_MAGIC, 81, int)

#endif /* AMC_PICO_H_ */
