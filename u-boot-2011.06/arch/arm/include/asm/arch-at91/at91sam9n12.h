/*
 * Chip-specific header file for the AT91SAM9N12
 *
 * (C) 2011 Atmel Corporation.
 *
 * Definitions for the SoC:
 * AT91SAM9N12 
 *
 * Note that those SoCs are mostly software and pin compatible,
 * therefore this file applies to all of them. Differences between
 * those SoCs are concentrated at the end of this file.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __AT91SAM9N12_H
#define __AT91SAM9N12_H

#define CONFIG_ARM926EJS	/* ARM926EJS Core */
#define CONFIG_AT91FAMILY	/* It's a member of AT91 */

/*
 * Peripheral identifiers/interrupts.
 */
#define ATMEL_ID_FIQ	0	/* Advanced Interrupt Controller (FIQ) */
#define ATMEL_ID_SYS	1	/* System Peripherals */
#define ATMEL_ID_PIOAB	2	/* Parallel IO Controller A and B */
#define ATMEL_ID_PIOCD	3	/* Parallel IO Controller C and D */
/* Reserved		4 */
#define ATMEL_ID_USART0	5	/* USART 0 */
#define ATMEL_ID_USART1	6	/* USART 1 */
#define ATMEL_ID_USART2	7	/* USART 2 */
#define ATMEL_ID_USART3	8	/* USART 3 */
#define ATMEL_ID_TWI0	9	/* Two-Wire Interface 0 */
#define ATMEL_ID_TWI1	10	/* Two-Wire Interface 1 */
/* Reserved		11 */
#define ATMEL_ID_MCI	12	/* Multimedia Card Interface */
#define ATMEL_ID_SPI0	13	/* Serial Peripheral Interface 0 */
#define ATMEL_ID_SPI1	14	/* Serial Peripheral Interface 1 */
#define ATMEL_ID_UART0	15	/* UART 0 */
#define ATMEL_ID_UART1	16	/* UART 1 */
#define ATMEL_ID_TCB	17	/* Timer Counter */
#define ATMEL_ID_PWMC	18	/* Pulse Width Modulation */
#define ATMEL_ID_TSADC	19	/* Analog-to-Digital Converter */
#define ATMEL_ID_DMAC	20	/* DMA Controller */
/* Reserved		21 */
#define ATMEL_ID_UHP	22	/* USB Host port */
#define ATMEL_ID_UDP	23	/* USB Device Port */
/* Reserved		24 */
#define ATMEL_ID_LCDC	25	/* LCD Controller */
/* Reserved:		26 */
/* Reserved:		27 */
#define ATMEL_ID_SSC	28	/* Serial Synchronous Controller */
/* Reserved:		29 */
#define ATMEL_ID_TRNG	30	/* True Random Number Generator */
#define ATMEL_ID_IRQ	31	/* Advanced Interrupt Controller (IRQ) */

/*
 * User Peripherals physical base addresses.
 */
#define ATMEL_BASE_SPI0		0xf0000000
#define ATMEL_BASE_SPI1		0xf0004000
#define ATMEL_BASE_MCI		0xf0008000
#define ATMEL_BASE_SSC0		0xf0010000
#define ATMEL_BASE_TC012	0xf8008000
#define ATMEL_BASE_TC345	0xf800c000
#define ATMEL_BASE_TWI0		0xf8010000
#define ATMEL_BASE_TWI1		0xf8014000
#define ATMEL_BASE_USART0	0xf801c000
#define ATMEL_BASE_USART1	0xf8020000
#define ATMEL_BASE_USART2	0xf8024000
#define ATMEL_BASE_USART3	0xf8028000
#define ATMEL_BASE_PWMC		0xf8034000
#define ATMEL_BASE_LCDC		0xf8038000
#define ATMEL_BASE_UDP		0xf803c000
#define ATMEL_BASE_UART0	0xf8040000
#define ATMEL_BASE_UART1	0xf8044000
#define ATMEL_BASE_TRNG		0xf8048000
#define ATMEL_BASE_TSADC	0xf804c000

/*
 * System Peripherals physical base addresses.
 */
#define ATMEL_BASE_SYS		0xffffc000
#define ATMEL_BASE_FUSE		0xffffdc00
#define ATMEL_BASE_MATRIX	0xffffde00
#define ATMEL_BASE_PMECC	0xffffe000
#define ATMEL_BASE_PMERRLOC	0xffffe600
#define ATMEL_BASE_SDRAMC	0xffffe800
#define ATMEL_BASE_SMC		0xffffea00
#define ATMEL_BASE_DMAC		0xffffec00
#define ATMEL_BASE_AIC		0xfffff000
#define ATMEL_BASE_DBGU		0xfffff200
#define ATMEL_BASE_PIOA		0xfffff400
#define ATMEL_BASE_PIOB		0xfffff600
#define ATMEL_BASE_PIOC		0xfffff800
#define ATMEL_BASE_PIOD		0xfffffa00
#define ATMEL_BASE_PMC		0xfffffc00
#define ATMEL_BASE_RSTC		0xfffffe00
#define ATMEL_BASE_SHDC		0xfffffe10
#define ATMEL_BASE_PIT		0xfffffe30
#define ATMEL_BASE_WDT		0xfffffe40
#define ATMEL_BASE_SCKCR	0xfffffe50
#define ATMEL_BASE_BSCR		0xfffffe54
#define ATMEL_BASE_GPBR		0xfffffe60
#define ATMEL_BASE_RTC		0xfffffeb0

/*
 * Internal Memory common on all these SoCs
 */
#define ATMEL_BASE_BOOT		0x00000000	/* Boot mapped area */
#define ATMEL_BASE_ROM		0x00100000	/* Internal ROM base address */
#define ATMEL_BASE_SRAM		0x00300000	/* Internal SRAM */
#define ATMEL_UHP_BASE		0x00500000	/* USB Host controller */

/*
 * External memory
 */
#define ATMEL_BASE_CS0		0x10000000	/* typically NOR */
#define ATMEL_BASE_CS1		0x20000000	/* DRAM */
#define ATMEL_BASE_CS2		0x30000000
#define ATMEL_BASE_CS3		0x40000000	/* typically NAND */
#define ATMEL_BASE_CS4		0x50000000
#define ATMEL_BASE_CS5		0x60000000

/*
 * Other misc defines
 */
#define ATMEL_PMC_UHP		AT91SAM926x_PMC_UHP
#define ATMEL_CPU_HAS_PIO3			/* CPU has PIO v3 */
#define ATMEL_PIO_PORTS		4		/* AT91SAM9N12 has 4 PIO */
#define ATMEL_BASE_PIO		ATMEL_BASE_PIOA

#define AT91_BASE_SYS		ATMEL_BASE_SYS
#define AT91_PMECC		(ATMEL_BASE_PMECC	- AT91_BASE_SYS)
#define AT91_PMERRLOC		(ATMEL_BASE_PMERRLOC	- AT91_BASE_SYS)
#define AT91_SMC		(ATMEL_BASE_SMC		- AT91_BASE_SYS)

/*
 * SoC specific defines
 */
# define ATMEL_CPU_NAME		"AT91SAM9N12"

#endif
