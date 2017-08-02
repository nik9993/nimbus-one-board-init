/*
 * (C) 2011 Atmel Corporation.
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

#include <common.h>
#include <asm/io.h>
#include <asm/arch/at91sam9n12_matrix.h>
#include <asm/arch/at91sam9_smc.h>
#include <asm/arch/at91_common.h>
#include <asm/arch/at91_pmc.h>
#include <asm/arch/at91_rstc.h>
#include <asm/arch/gpio.h>
#include <lcd.h>
#include <atmel_lcdc.h>

#if defined(CONFIG_RESET_PHY_R) && defined(CONFIG_DRIVER_KS885X)
#include <net.h>
#include <netdev.h>
#endif

DECLARE_GLOBAL_DATA_PTR;

/* ------------------------------------------------------------------------- */
/*
 * Miscelaneous platform dependent initialisations
 */

#ifdef CONFIG_CMD_NAND
static void at91sam9n12ek_nand_hw_init(void)
{
	struct at91_smc *smc = (struct at91_smc *)ATMEL_BASE_SMC;
	struct at91_matrix *matrix = (struct at91_matrix *)ATMEL_BASE_MATRIX;
	unsigned long csa;

	/* Assign CS3 to NAND/SmartMedia Interface */
	csa = readl(&matrix->ebicsa);
	csa |= AT91_MATRIX_CS3A_SMC_SMARTMEDIA;
	/* Configure databus */
	csa &= ~AT91_MATRIX_NFD0_ON_D16; /* nandflash connect to D0~D15 */
	/* Configure IO drive */
	csa |= AT91_MATRIX_HIGH_DRIVE;
	writel(csa, &matrix->ebicsa);

	/* Configure SMC CS3 for NAND/SmartMedia */
	writel(AT91_SMC_SETUP_NWE(1) | AT91_SMC_SETUP_NCS_WR(0) |
		AT91_SMC_SETUP_NRD(2) | AT91_SMC_SETUP_NCS_RD(0),
		&smc->cs[3].setup);
	writel(AT91_SMC_PULSE_NWE(3) | AT91_SMC_PULSE_NCS_WR(5) |
		AT91_SMC_PULSE_NRD(4) | AT91_SMC_PULSE_NCS_RD(6),
		&smc->cs[3].pulse);
	writel(AT91_SMC_CYCLE_NWE(5) | AT91_SMC_CYCLE_NRD(7),
		&smc->cs[3].cycle);
	writel(AT91_SMC_MODE_RM_NRD | AT91_SMC_MODE_WM_NWE |
		AT91_SMC_MODE_EXNW_DISABLE |
#ifdef CONFIG_SYS_NAND_DBW_16
		AT91_SMC_MODE_DBW_16 |
#else /* CONFIG_SYS_NAND_DBW_8 */
		AT91_SMC_MODE_DBW_8 |
#endif
		AT91_SMC_MODE_TDF_CYCLE(1),
		&smc->cs[3].mode);

	/* Configure RDY/BSY */
	at91_set_gpio_input(CONFIG_SYS_NAND_READY_PIN, 1);

	/* Enable NandFlash */
	at91_set_gpio_output(CONFIG_SYS_NAND_ENABLE_PIN, 1);

	at91_set_a_periph(AT91_PIO_PORTD, 0, 1);    /* NAND OE */
	at91_set_a_periph(AT91_PIO_PORTD, 1, 1);    /* NAND WE */
	at91_set_a_periph(AT91_PIO_PORTD, 2, 1);    /* ALE */
	at91_set_a_periph(AT91_PIO_PORTD, 3, 1);    /* CLE */
}
#endif

#ifdef CONFIG_LCD
vidinfo_t panel_info = {
	vl_col:		480,
	vl_row:		272,
	vl_clk:		9000000,
	vl_bpix:	LCD_BPP,
	vl_tft:		1,
	vl_hsync_len:	5,
	vl_left_margin:	8,
	vl_right_margin:43,
	vl_vsync_len:	10,
	vl_upper_margin:4,
	vl_lower_margin:12,
	mmio:		ATMEL_BASE_LCDC,
};


void lcd_enable(void)
{
	at91_set_gpio_output(AT91_PIN_PC25, 0);	/* power up */
}

void lcd_disable(void)
{
	at91_set_gpio_output(AT91_PIN_PC25, 1);	/* power down */
}

static void at91sam9n12ek_lcd_hw_init(void)
{
	struct at91_pmc *pmc = (struct at91_pmc *)ATMEL_BASE_PMC;

	at91_set_A_periph(AT91_PIN_PC24, 0);	/* LCDDPWR */
	at91_set_A_periph(AT91_PIN_PC26, 0);	/* LCDVSYNC */
	at91_set_A_periph(AT91_PIN_PC27, 0);	/* LCDHSYNC */
	at91_set_A_periph(AT91_PIN_PC28, 0);	/* LCDDOTCK */
	at91_set_A_periph(AT91_PIN_PC29, 0);	/* LCDDEN */
	at91_set_A_periph(AT91_PIN_PC30, 0);	/* LCDDOTCK */

	at91_set_A_periph(AT91_PIN_PC0, 0);	/* LCDD0 */
	at91_set_A_periph(AT91_PIN_PC1, 0);	/* LCDD1 */
	at91_set_A_periph(AT91_PIN_PC2, 0);	/* LCDD2 */
	at91_set_A_periph(AT91_PIN_PC3, 0);	/* LCDD3 */
	at91_set_A_periph(AT91_PIN_PC4, 0);	/* LCDD4 */
	at91_set_A_periph(AT91_PIN_PC5, 0);	/* LCDD5 */
	at91_set_A_periph(AT91_PIN_PC6, 0);	/* LCDD6 */
	at91_set_A_periph(AT91_PIN_PC7, 0);	/* LCDD7 */
	at91_set_A_periph(AT91_PIN_PC8, 0);	/* LCDD8 */
	at91_set_A_periph(AT91_PIN_PC9, 0);	/* LCDD9 */
	at91_set_A_periph(AT91_PIN_PC10, 0);	/* LCDD10 */
	at91_set_A_periph(AT91_PIN_PC11, 0);	/* LCDD11 */
	at91_set_A_periph(AT91_PIN_PC12, 0);	/* LCDD12 */
	at91_set_B_periph(AT91_PIN_PC13, 0);	/* LCDD13 */
	at91_set_A_periph(AT91_PIN_PC14, 0);	/* LCDD14 */
	at91_set_A_periph(AT91_PIN_PC15, 0);	/* LCDD15 */
	at91_set_A_periph(AT91_PIN_PC16, 0);	/* LCDD16 */
	at91_set_A_periph(AT91_PIN_PC17, 0);	/* LCDD17 */
	at91_set_A_periph(AT91_PIN_PC18, 0);	/* LCDD18 */
	at91_set_A_periph(AT91_PIN_PC19, 0);	/* LCDD19 */
	at91_set_A_periph(AT91_PIN_PC20, 0);	/* LCDD20 */
	at91_set_B_periph(AT91_PIN_PC21, 0);	/* LCDD21 */
	at91_set_A_periph(AT91_PIN_PC22, 0);	/* LCDD22 */
	at91_set_A_periph(AT91_PIN_PC23, 0);	/* LCDD23 */

	writel(1 << ATMEL_ID_LCDC, &pmc->pcer);
}

#ifdef CONFIG_LCD_INFO
#include <nand.h>
#include <version.h>

void lcd_show_board_info(void)
{
	ulong dram_size, nand_size;
	int i;
	char temp[32];

	lcd_printf ("%s\n", U_BOOT_VERSION);
	lcd_printf ("(C) 2011 ATMEL Corp\n");
	lcd_printf ("at91support@atmel.com\n");
	lcd_printf ("%s CPU at %s MHz\n",
		ATMEL_CPU_NAME,
		strmhz(temp, get_cpu_clk_rate()));

	dram_size = 0;
	for (i = 0; i < CONFIG_NR_DRAM_BANKS; i++)
		dram_size += gd->bd->bi_dram[i].size;
	nand_size = 0;
	for (i = 0; i < CONFIG_SYS_MAX_NAND_DEVICE; i++)
		nand_size += nand_info[i].size;
	lcd_printf ("  %ld MB SDRAM, %ld MB NAND\n",
		dram_size >> 20,
		nand_size >> 20 );
}
#endif /* CONFIG_LCD_INFO */
#endif /* CONFIG_LCD */

#ifdef CONFIG_DRIVER_KS885X
static void at91sam9n12ek_ks885x_hw_init(void)
{
	struct at91_smc *smc = (struct at91_smc *)ATMEL_BASE_SMC;
	writel(AT91_SMC_SETUP_NWE(2) | AT91_SMC_SETUP_NCS_WR(0) |
		AT91_SMC_SETUP_NRD(1) | AT91_SMC_SETUP_NCS_RD(0),
		&smc->cs[2].setup);
	writel(AT91_SMC_PULSE_NWE(7) | AT91_SMC_PULSE_NCS_WR(7) |
		AT91_SMC_PULSE_NRD(7) | AT91_SMC_PULSE_NCS_RD(7),
		&smc->cs[2].pulse);
	writel(AT91_SMC_CYCLE_NWE(9) | AT91_SMC_CYCLE_NRD(9),
		&smc->cs[2].cycle);
	writel(AT91_SMC_READMODE | AT91_SMC_WRITEMODE |
		AT91_SMC_EXNWMODE_DISABLE |
		AT91_SMC_BAT_SELECT | AT91_SMC_DBW_16 |
		AT91_SMC_TDF_(1),
		&smc->cs[2].mode);

	/* Configure NCS2 PIN */
	at91_set_B_periph(AT91_PIN_PD19, 0);
	/* Configure Interrupt pin as input, no pull-up */
	at91_set_gpio_input(AT91_PIN_PD21, 0);
}
#endif /* CONFIG_DRIVER_KS885X */

/* SPI chip select control */
#ifdef CONFIG_ATMEL_SPI
#include <spi.h>
int spi_cs_is_valid(unsigned int bus, unsigned int cs)
{
	return bus == 0 && cs < 2;
}

void spi_cs_activate(struct spi_slave *slave)
{
	switch (slave->cs) {
	case 1:
		at91_set_gpio_output(AT91_PIN_PA7, 0);
		break;
	case 0:
	default:
		at91_set_gpio_output(AT91_PIN_PA14, 0);
		break;
	}
}

void spi_cs_deactivate(struct spi_slave *slave)
{
	switch (slave->cs) {
	case 1:
		at91_set_gpio_output(AT91_PIN_PA7, 1);
		break;
	case 0:
	default:
		at91_set_gpio_output(AT91_PIN_PA14, 1);
		break;
	}
}
#endif /* CONFIG_ATMEL_SPI */

int board_early_init_f(void)
{
	struct at91_pmc *pmc = (struct at91_pmc *)ATMEL_BASE_PMC;

	/* Enable clocks for all PIOs */
	writel((1 << ATMEL_ID_PIOAB) | (1 << ATMEL_ID_PIOCD), &pmc->pcer);

	return 0;
}

int board_init(void)
{
	gd->bd->bi_arch_number = MACH_TYPE_AT91SAM9N12EK;

	/* adress of boot parameters */
	gd->bd->bi_boot_params = CONFIG_SYS_SDRAM_BASE + 0x100;

	at91_seriald_hw_init();
#ifdef CONFIG_CMD_NAND
	at91sam9n12ek_nand_hw_init();
#endif
#if defined(CONFIG_HAS_DATAFLASH) || defined(CONFIG_ATMEL_SPI)
	at91_spi0_hw_init((1 << 0) | (1 << 1));
#endif

#ifdef CONFIG_DRIVER_KS885X
	at91sam9n12ek_ks885x_hw_init();
#endif

#ifdef CONFIG_LCD
	at91sam9n12ek_lcd_hw_init();
#endif

	return 0;
}

int dram_init(void)
{
	gd->ram_size = get_ram_size(
		(void *)CONFIG_SYS_SDRAM_BASE,
		CONFIG_SYS_SDRAM_SIZE);
	return 0;
}

#ifdef CONFIG_RESET_PHY_R
void reset_phy(void)
{
#ifdef CONFIG_DRIVER_KS885X
	/*
	 * Initialize ethernet HW addr prior to starting Linux,
	 * needed for nfsroot
	 */
	eth_init(gd->bd);
#endif
}
#endif

#ifdef CONFIG_DRIVER_KS885X
int board_eth_init(bd_t *bis)
{
	return ks8851_initialize(bis);
}
#endif
