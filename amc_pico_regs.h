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
 * \brief AMC-Pico8 address map
 */

#ifndef AMC_PICO_REGS_H_
#define AMC_PICO_REGS_H_

#define AMC_PICO_SUBVENDOR_ID	(0xCAE2)
#define AMC_PICO_SUBDEVICE_ID	(0x71C0)

/* module adresses */
#define PICO_ADDR	(0x0)
#define DMA_ADDR	(0x10000)
#define MUX_ADDR	(0x20000)
#define USER_ADDR   (0x30000)
#define INTR_ADDR	(0x40000)

/* on DMA_ADDR */
#define DMA_OFFSET_STATUS	(0x0)
#define DMA_OFFSET_CONTROL	(0x4)
#define DMA_OFFSET_CMD		(0x8)
#define DMA_OFFSET_ADDR		(0xC)
#define DMA_OFFSET_LEN		(0x10)
#define DMA_OFFSET_RESP_LEN	(0x14)
#define DMA_OFFSET_RESP_ADDR	(0x18)


/* on PICO_ADDR */
#define PICO_CONV_TRG		(0x4)
#define PICO_CONV_GEN		(0x8)
#define RING_BUFF_OFFS_DELAY	(0xC)
#define TRG_OFFS_CTRL		(0x10)
#define TRG_OFFS_LIMIT		(0x14)
#define TRG_OFFS_NRSAMP		(0x18)
#define FPGA_VER_OFFSET		(0x78)
#define FPGA_TS_OFFSET		(0x7C)

#define PICO_CLK_FREQ		(300000000)
#define PICO_CONV_MAX		(2048)
#define PICO_ADC_MAX_FREQ	(1000000)

#define TRG_CTRL_CH_SHIFT	(8)

/* */
#define MUX_TRG_MASK		(0x7)
#define MUX_TRG_SHIFT		(0)
#define MUX_CONV_MASK		(0x7)
#define MUX_CONV_SHIFT		(8)

/* DMA register masks */
#define DMA_CTRL_MASK_ENABLE	(0x00000100)
#define DMA_CTRL_MASK_RESET	(0x00000001)

#define DMA_CMD_MASK_DMA_GO	(0x80000000)
#define DMA_CMD_MASK_GEN_IRQ	(0x08000000)

/* on INTR */
/* Introduced in FW version 0x0001000b */
/* constant 0x157C5721 */
#define INTR_ID (INTR_ADDR+0x0)
#define INTR_STATUS (INTR_ADDR+0x4)
/* bit indicating whether device think MSI is enabld */
#define INTR_STATUS_MSI_EN (1<<8)
/* bit indicating whether an interrupt is latched (INTR_LATCH is non-zero) */
#define INTR_STATUS_ACT    (1<<0)
/* unused #define INTR_CONTROL (INTR_ADDR+0x8) */
/* bit mask of all interrupt sources */
#define INTR_LATCH (INTR_ADDR+0xc)
/* bit mask.  write to clear per interrupt source */
#define INTR_CLEAR (INTR_ADDR+0x10)
/* bit mask.  write to un-mask per interrupt source */
#define INTR_ENABLE (INTR_ADDR+0x14)

/* mask for both INTR_LATCH and INTR_CLEAR.
 * HW impliments 8 bits.
 */
#define INTR_DMA_DONE 0x1
#define INTR_USER 0x2
#define INTR_MASK (INTR_DMA_DONE|INTR_USER)

/** Registers specific to FRIB local firmware.
 */
#ifdef CONFIG_AMC_PICO_FRIB

#define FRIB_CAP_FIRST (USER_ADDR+0x100)
#define FRIB_CAP_LAST  (USER_ADDR+0x1AC)

#define USER_ID (USER_ADDR+0)
#define USER_STATUS (USER_ADDR+0x4)
#define USER_CONTROL (USER_ADDR+0x8)

#define FRIB_VERSION  (USER_ADDR+4*0x10)
#endif /* CONFIG_AMC_PICO_FRIB */

#endif /* AMC_PICO_REGS_H_ */
