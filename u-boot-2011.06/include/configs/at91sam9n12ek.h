/*
 * (C) 2011 Atmel Corporation.
 *
 * Configuation settings for the AT91SAM9N12-EK boards.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef __AT91SAM9N12_CONFIG_H_
#define __AT91SAM9N12_CONFIG_H_

#define CONFIG_AT91_LEGACY
/*
 * SoC must be defined first, before hardware.h is included.
 * In this case SoC is defined in boards.cfg.
 */
#include <asm/hardware.h>

/*
 * Warning: changing CONFIG_SYS_TEXT_BASE requires
 * adapting the initial boot program.
 * Since the linker has to swallow that define, we must use a pure
 * hex number here!
 */
#define CONFIG_SYS_TEXT_BASE		0x26f00000

/* ARM asynchronous clock */
#define CONFIG_SYS_AT91_SLOW_CLOCK	32768		/* slow clock xtal */
#define CONFIG_SYS_AT91_MAIN_CLOCK	16000000	/* main clock xtal */
#define CONFIG_SYS_HZ			1000

# define CONFIG_AT91SAM9N12EK		/* It's an Atmel AT91SAM9N12 EK */

/* Misc CPU related */
#define CONFIG_ARCH_CPU_INIT
#undef	CONFIG_USE_IRQ			/* we don't need IRQ/FIQ stuff	*/
#define CONFIG_CMDLINE_TAG		/* enable passing of ATAGs */
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG
#define CONFIG_SKIP_LOWLEVEL_INIT
#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_DISPLAY_CPUINFO

/* general purpose I/O */
#define CONFIG_ATMEL_LEGACY		/* required until (g)pio is fixed */
#define CONFIG_AT91_GPIO
#define CONFIG_AT91_GPIO_PULLUP	1	/* keep pullups on peripheral pins */

/* serial console */
#define CONFIG_ATMEL_USART
#define CONFIG_USART_BASE		ATMEL_BASE_DBGU
#define	CONFIG_USART_ID			ATMEL_ID_SYS
#define CONFIG_BAUDRATE			115200
#define CONFIG_SYS_BAUDRATE_TABLE	{115200, 19200, 38400, 57600, 9600}

/* LCD */
#define CONFIG_LCD
#define LCD_BPP				LCD_COLOR16
#define LCD_OUTPUT_BPP			24
#define CONFIG_LCD_LOGO			1
#undef LCD_TEST_PATTERN
#define CONFIG_LCD_INFO			1
#define CONFIG_LCD_INFO_BELOW_LOGO	1
#define CONFIG_SYS_WHITE_ON_BLACK	1
#define CONFIG_ATMEL_LCD		1
#define CONFIG_ATMEL_LCD_RGB565		1
#define CONFIG_SYS_CONSOLE_IS_IN_ENV	1

/* LED */
#define CONFIG_AT91_LED
#define	CONFIG_RED_LED		AT91_PIN_PB6	/* this is the power led */
#define	CONFIG_GREEN_LED	AT91_PIN_PB5	/* this is the user led */
#define	CONFIG_BLUE_LED		AT91_PIN_PB4	/* this is the user led */

#define CONFIG_BOOTDELAY	3

/*
 * BOOTP options
 */
#define CONFIG_BOOTP_BOOTFILESIZE
#define CONFIG_BOOTP_BOOTPATH
#define CONFIG_BOOTP_GATEWAY
#define CONFIG_BOOTP_HOSTNAME

/*
 * Command line configuration.
 */
#include <config_cmd_default.h>
#undef CONFIG_CMD_BDI
#undef CONFIG_CMD_FPGA
#undef CONFIG_CMD_IMI
#undef CONFIG_CMD_IMLS
#undef CONFIG_CMD_LOADS
#undef CONFIG_CMD_SOURCE

#define CONFIG_CMD_PING
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_NAND
#define CONFIG_CMD_USB

#define CONFIG_NR_DRAM_BANKS		1
#define CONFIG_SYS_SDRAM_BASE		ATMEL_BASE_CS1
#define CONFIG_SYS_SDRAM_SIZE		0x08000000

/*
 * Initial stack pointer: 4k - GENERATED_GBL_DATA_SIZE in internal SRAM,
 * leaving the correct space for initial global data structure above
 * that address while providing maximum stack area below.
 */
# define CONFIG_SYS_INIT_SP_ADDR \
	(ATMEL_BASE_SRAM + 0x1000 - GENERATED_GBL_DATA_SIZE)

/* DataFlash */
#ifdef CONFIG_SYS_USE_DATAFLASH_CS0
#define CONFIG_ATMEL_SPI
#endif

#ifdef CONFIG_ATMEL_SPI
#define CONFIG_CMD_SF
#define CONFIG_CMD_SPI
#define CONFIG_SPI_FLASH		1
#define CONFIG_SPI_FLASH_ATMEL		1
#define CONFIG_SYS_USE_DATAFLASH	1
#define CONFIG_ENV_SPI_MAX_HZ		30000000
#define CONFIG_SF_DEFAULT_SPEED		30000000
#define CONFIG_ENV_SPI_MODE		SPI_MODE_3
#define CONFIG_SF_DEFAULT_MODE		SPI_MODE_3
#endif

/* NAND flash */
#ifdef CONFIG_CMD_NAND
#define CONFIG_NAND_ATMEL
#define CONFIG_SYS_MAX_NAND_DEVICE	1
#define CONFIG_SYS_NAND_BASE		ATMEL_BASE_CS3
#define CONFIG_SYS_NAND_DBW_8
#define CONFIG_SYS_NAND_MASK_ALE	(1 << 21)
#define CONFIG_SYS_NAND_MASK_CLE	(1 << 22)
#define CONFIG_SYS_NAND_ENABLE_PIN	AT91_PIN_PD4
#define CONFIG_SYS_NAND_READY_PIN	AT91_PIN_PD5
/* PMECC & PMERRLOC */
#define CONFIG_SYS_NAND_PMECC_BASE      AT91_PMECC
#define CONFIG_SYS_NAND_PMERRLOC_BASE   AT91_PMERRLOC
#define CONFIG_ATMEL_NAND_HWECC         1
#define CONFIG_ATMEL_NAND_HW_PMECC      1
#endif

/* NOR flash - no real flash on this board */
#define CONFIG_SYS_NO_FLASH

/* Ethernet */
#define CONFIG_DRIVER_KS885X
#define CONFIG_NET_MULTI
#define CONFIG_NET_RETRY_COUNT		20
#define CONFIG_RESET_PHY_R

/* USB */
#define CONFIG_USB_ATMEL
#define CONFIG_USB_OHCI_NEW
#define CONFIG_DOS_PARTITION
#define CONFIG_SYS_USB_OHCI_CPU_INIT
#define CONFIG_SYS_USB_OHCI_REGS_BASE		ATMEL_UHP_BASE
#define CONFIG_SYS_USB_OHCI_SLOT_NAME		"at91sam9n12"
#define CONFIG_SYS_USB_OHCI_MAX_ROOT_PORTS	2
#define CONFIG_USB_STORAGE
#define CONFIG_CMD_FAT

#define CONFIG_SYS_LOAD_ADDR			0x22000000 /* load address */

#define CONFIG_SYS_MEMTEST_START		CONFIG_SYS_SDRAM_BASE
#define CONFIG_SYS_MEMTEST_END			0x23e00000

#ifdef CONFIG_SYS_USE_DATAFLASH

/* bootstrap + u-boot + env + linux in dataflash on CS0 */
#define CONFIG_ENV_IS_IN_SPI_FLASH	1
#define CONFIG_ENV_OFFSET	0x5000
#define CONFIG_ENV_SIZE		0x3000
#define CONFIG_ENV_SECT_SIZE	0x1000
#define CONFIG_BOOTCOMMAND	"sf probe 0; " \
				"sf read 0x22000000 0x42000 0x250000; " \
				"bootm 0x22000000"
#define CONFIG_BOOTARGS		"mem=128M "				\
				"console=ttyS0,115200 "			\
				"root=/dev/mtdblock1 "			\
				"mtdparts=atmel_nand:4M(kernel)ro,"	\
				"60M(rootfs),-(data) noinitrd "		\
				"rw rootfstype=jffs2"

#else

/* bootstrap + u-boot + env + linux in nandflash */
#define CONFIG_ENV_IS_IN_NAND
#define CONFIG_ENV_OFFSET		0x60000
#define CONFIG_ENV_OFFSET_REDUND	0x80000
#define CONFIG_ENV_SIZE		0x20000		/* 1 sector = 128 kB */
#define CONFIG_BOOTCOMMAND	"nand read 0x22000000 0xA0000 0x200000; bootm"
#define CONFIG_BOOTARGS		"console=ttyS0,115200 "			\
				"root=/dev/mtdblock5 "			\
				"mtdparts=atmel_nand:128k(bootstrap)ro,"	\
				"256k(uboot)ro,128k(env1)ro,"		\
				"128k(env2)ro,2M(linux),-(root) "	\
				"rw rootfstype=jffs2"

#endif

#define CONFIG_SYS_PROMPT		"U-Boot> "
#define CONFIG_SYS_CBSIZE		256
#define CONFIG_SYS_MAXARGS		16
#define CONFIG_SYS_PBSIZE		(CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)
#define CONFIG_SYS_LONGHELP
#define CONFIG_CMDLINE_EDITING

/*
 * Size of malloc() pool
 */
#define CONFIG_SYS_MALLOC_LEN		ROUND(3 * CONFIG_ENV_SIZE + 128*1024, 0x1000)

#define CONFIG_STACKSIZE	(32*1024)	/* regular stack */

#ifdef CONFIG_USE_IRQ
#error CONFIG_USE_IRQ not supported
#endif

#endif
