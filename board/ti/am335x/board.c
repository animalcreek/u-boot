/*
 * board.c
 *
 * Board functions for TI AM335X based boards
 *
 * Copyright (C) 2011, Texas Instruments, Incorporated - http://www.ti.com/
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <errno.h>
#include <spl.h>
#include <asm/arch/cpu.h>
#include <asm/arch/hardware.h>
#include <asm/arch/omap.h>
#include <asm/arch/ddr_defs.h>
#include <asm/arch/clock.h>
#include <asm/arch/clk_synthesizer.h>
#include <asm/arch/gpio.h>
#include <asm/arch/mmc_host_def.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/mem.h>
#include <asm/io.h>
#include <asm/emif.h>
#include <asm/gpio.h>
#include <i2c.h>
#include <spi.h>
#include <miiphy.h>
#include <cpsw.h>
#include <power/tps65217.h>
#include <power/tps65910.h>
#include <environment.h>
#include <watchdog.h>
#include <environment.h>
#include <command.h>
#include <board-common/board_detect.h>
#include <lcd.h>
#include "../../../drivers/video/am335x-fb.h"
#include "board.h"

DECLARE_GLOBAL_DATA_PTR;

/* GPIO that controls power to DDR on EVM-SK */
#define GPIO_TO_PIN(bank, gpio)		(32 * (bank) + (gpio))
#define GPIO_DDR_VTT_EN		GPIO_TO_PIN(0, 7)
#define ICE_GPIO_DDR_VTT_EN	GPIO_TO_PIN(0, 18)
#define GPIO_PR1_MII_CTRL	GPIO_TO_PIN(3, 4)
#define GPIO_MUX_MII_CTRL	GPIO_TO_PIN(3, 10)
#define GPIO_FET_SWITCH_CTRL	GPIO_TO_PIN(0, 7)
#define GPIO_PHY_RESET		GPIO_TO_PIN(2, 5)
#define GPIO_LCD_RESET		GPIO_TO_PIN(1, 21)
#define GPIO_HW_REV0		GPIO_TO_PIN(1, 26)
#define GPIO_HW_REV1		GPIO_TO_PIN(1, 27)
#define GPIO_HW_REV2		GPIO_TO_PIN(1, 28)
#define GPIO_HW_REV3		GPIO_TO_PIN(1, 29)
#define GPIO_LCD_REV0		GPIO_TO_PIN(3, 0)
#define GPIO_LCD_REV1		GPIO_TO_PIN(3, 1)

static struct ctrl_dev *cdev = (struct ctrl_dev *)CTRL_DEVICE_BASE;

struct gpio_map {
	unsigned int	gpio;
	char		*name;
};

static struct gpio_map hw_rev_gpios[] = {
	{
		.gpio	= GPIO_HW_REV0,
		.name	= "hw_rev_0",
	},
	{
		.gpio	= GPIO_HW_REV1,
		.name	= "hw_rev_1",
	},
	{
		.gpio	= GPIO_HW_REV2,
		.name	= "hw_rev_2",
	},
	{
		.gpio	= GPIO_HW_REV3,
		.name	= "hw_rev_3",
	},
};

static struct gpio_map lcd_rev_gpios[] = {
	{
		.gpio	= GPIO_LCD_REV0,
		.name	= "lcd_rev_0",
	},
	{
		.gpio	= GPIO_LCD_REV1,
		.name	= "lcd_rev_1",
	},
};

int do_set_kv3_serial(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	int r;
	char * read;
	char * write;
	struct ti_am_eeprom am_ep;
	char serial[TI_EEPROM_HDR_SERIAL_LEN + 1];
	char serial_split[TI_EEPROM_HDR_SERIAL_LEN + 5];

	if (argc != 2) {
		printf("Invalid argument(s):  set_kv3_serial <serial-number>\n");
		return 1;
	}

    memset(serial, 0, TI_EEPROM_HDR_SERIAL_LEN + 1);

	// Strip dashes
	read = argv[1];
	write = serial;
	int i = 0;
	while (*read) {
		if (*read != '-') {
			*write = *read;
			write++;
			i++;
		}
		read++;

		if (i > TI_EEPROM_HDR_SERIAL_LEN) {
			printf("Failed to set KV3 serial, serial number too long\n");
			return 1;
		}
	}

	r = ti_kv3_update_eeprom("KV3", "00D1", serial);
	if (r) {
		printf("Failed to set KV3 serial\n");
		return r;
	}

	/* Read back directly from EEPROM to print confirmation */
	r = i2c_read(CONFIG_SYS_I2C_EEPROM_ADDR, 0x0, 2, (uint8_t *)&am_ep, sizeof(am_ep));
	if (r) {
		printf("Failed to read EEPROM\n");
		return r;
	}

	strlcpy(serial, am_ep.serial, TI_EEPROM_HDR_SERIAL_LEN + 1);
	serial[TI_EEPROM_HDR_SERIAL_LEN] = '\0';

	if (!strncmp(serial, "A1", 2)) {
		sprintf(serial_split, "%c%c-%c-%c%c-%c%c%c-%s",
				serial[0],
				serial[1],
				serial[2],
				serial[3],
				serial[4],
				serial[5],
				serial[6],
				serial[7],
				serial + 8);
	} else {
		sprintf(serial_split, serial);
	}

	setenv("board_serial", serial_split);
	printf("Set KV3 serial to %s\n", serial_split);
	return 0;
}
U_BOOT_CMD(set_kv3_serial, 2, 0, do_set_kv3_serial,
	"set kv3 serial number", "set kv3 serial number");

int do_zero_kv3_serial(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	if (argc != 1) {
		printf("zero_kv3_serial takes no arguments\n");
		return 1;
	}

	return ti_kv3_update_eeprom(NULL, NULL, NULL);
}
U_BOOT_CMD(zero_kv3_serial, 1, 1, do_zero_kv3_serial,
	"zero kv3 serial number", "zero kv3 serial number");

/*
 * Read header information from EEPROM into global structure.
 */
static int __maybe_unused read_eeprom(void)
{
	return ti_i2c_eeprom_am_get(-1, CONFIG_SYS_I2C_EEPROM_ADDR);
}

#ifndef CONFIG_SKIP_LOWLEVEL_INIT
static const struct ddr_data ddr2_data = {
	.datardsratio0 = MT47H128M16RT25E_RD_DQS,
	.datafwsratio0 = MT47H128M16RT25E_PHY_FIFO_WE,
	.datawrsratio0 = MT47H128M16RT25E_PHY_WR_DATA,
};

static const struct cmd_control ddr2_cmd_ctrl_data = {
	.cmd0csratio = MT47H128M16RT25E_RATIO,

	.cmd1csratio = MT47H128M16RT25E_RATIO,

	.cmd2csratio = MT47H128M16RT25E_RATIO,
};

static const struct emif_regs ddr2_emif_reg_data = {
	.sdram_config = MT47H128M16RT25E_EMIF_SDCFG,
	.ref_ctrl = MT47H128M16RT25E_EMIF_SDREF,
	.sdram_tim1 = MT47H128M16RT25E_EMIF_TIM1,
	.sdram_tim2 = MT47H128M16RT25E_EMIF_TIM2,
	.sdram_tim3 = MT47H128M16RT25E_EMIF_TIM3,
	.emif_ddr_phy_ctlr_1 = MT47H128M16RT25E_EMIF_READ_LATENCY,
};

static const struct ddr_data ddr3_data = {
	.datardsratio0 = MT41J128MJT125_RD_DQS,
	.datawdsratio0 = MT41J128MJT125_WR_DQS,
	.datafwsratio0 = MT41J128MJT125_PHY_FIFO_WE,
	.datawrsratio0 = MT41J128MJT125_PHY_WR_DATA,
};

static const struct ddr_data ddr3_beagleblack_data = {
	.datardsratio0 = MT41K256M16HA125E_RD_DQS,
	.datawdsratio0 = MT41K256M16HA125E_WR_DQS,
	.datafwsratio0 = MT41K256M16HA125E_PHY_FIFO_WE,
	.datawrsratio0 = MT41K256M16HA125E_PHY_WR_DATA,
};

static const struct ddr_data ddr3_kv3_data = {
	.datardsratio0 = MT41K64M16TW107J_RD_DQS,
	.datawdsratio0 = MT41K64M16TW107J_WR_DQS,
	.datafwsratio0 = MT41K64M16TW107J_PHY_FIFO_WE,
	.datawrsratio0 = MT41K64M16TW107J_PHY_WR_DATA,
};

static const struct ddr_data ddr3_evm_data = {
	.datardsratio0 = MT41J512M8RH125_RD_DQS,
	.datawdsratio0 = MT41J512M8RH125_WR_DQS,
	.datafwsratio0 = MT41J512M8RH125_PHY_FIFO_WE,
	.datawrsratio0 = MT41J512M8RH125_PHY_WR_DATA,
};

static const struct ddr_data ddr3_icev2_data = {
	.datardsratio0 = MT41J128MJT125_RD_DQS_400MHz,
	.datawdsratio0 = MT41J128MJT125_WR_DQS_400MHz,
	.datafwsratio0 = MT41J128MJT125_PHY_FIFO_WE_400MHz,
	.datawrsratio0 = MT41J128MJT125_PHY_WR_DATA_400MHz,
};

static const struct cmd_control ddr3_cmd_ctrl_data = {
	.cmd0csratio = MT41J128MJT125_RATIO,
	.cmd0iclkout = MT41J128MJT125_INVERT_CLKOUT,

	.cmd1csratio = MT41J128MJT125_RATIO,
	.cmd1iclkout = MT41J128MJT125_INVERT_CLKOUT,

	.cmd2csratio = MT41J128MJT125_RATIO,
	.cmd2iclkout = MT41J128MJT125_INVERT_CLKOUT,
};

static const struct cmd_control ddr3_beagleblack_cmd_ctrl_data = {
	.cmd0csratio = MT41K256M16HA125E_RATIO,
	.cmd0iclkout = MT41K256M16HA125E_INVERT_CLKOUT,

	.cmd1csratio = MT41K256M16HA125E_RATIO,
	.cmd1iclkout = MT41K256M16HA125E_INVERT_CLKOUT,

	.cmd2csratio = MT41K256M16HA125E_RATIO,
	.cmd2iclkout = MT41K256M16HA125E_INVERT_CLKOUT,
};

static const struct cmd_control ddr3_kv3_cmd_ctrl_data = {
	.cmd0csratio = MT41K64M16TW107J_RATIO,
	.cmd0iclkout = MT41K64M16TW107J_INVERT_CLKOUT,

	.cmd1csratio = MT41K64M16TW107J_RATIO,
	.cmd1iclkout = MT41K64M16TW107J_INVERT_CLKOUT,

	.cmd2csratio = MT41K64M16TW107J_RATIO,
	.cmd2iclkout = MT41K64M16TW107J_INVERT_CLKOUT,
};

static const struct cmd_control ddr3_evm_cmd_ctrl_data = {
	.cmd0csratio = MT41J512M8RH125_RATIO,
	.cmd0iclkout = MT41J512M8RH125_INVERT_CLKOUT,

	.cmd1csratio = MT41J512M8RH125_RATIO,
	.cmd1iclkout = MT41J512M8RH125_INVERT_CLKOUT,

	.cmd2csratio = MT41J512M8RH125_RATIO,
	.cmd2iclkout = MT41J512M8RH125_INVERT_CLKOUT,
};

static const struct cmd_control ddr3_icev2_cmd_ctrl_data = {
	.cmd0csratio = MT41J128MJT125_RATIO_400MHz,
	.cmd0iclkout = MT41J128MJT125_INVERT_CLKOUT_400MHz,

	.cmd1csratio = MT41J128MJT125_RATIO_400MHz,
	.cmd1iclkout = MT41J128MJT125_INVERT_CLKOUT_400MHz,

	.cmd2csratio = MT41J128MJT125_RATIO_400MHz,
	.cmd2iclkout = MT41J128MJT125_INVERT_CLKOUT_400MHz,
};

static struct emif_regs ddr3_emif_reg_data = {
	.sdram_config = MT41J128MJT125_EMIF_SDCFG,
	.ref_ctrl = MT41J128MJT125_EMIF_SDREF,
	.sdram_tim1 = MT41J128MJT125_EMIF_TIM1,
	.sdram_tim2 = MT41J128MJT125_EMIF_TIM2,
	.sdram_tim3 = MT41J128MJT125_EMIF_TIM3,
	.zq_config = MT41J128MJT125_ZQ_CFG,
	.emif_ddr_phy_ctlr_1 = MT41J128MJT125_EMIF_READ_LATENCY |
				PHY_EN_DYN_PWRDN,
};

static struct emif_regs ddr3_beagleblack_emif_reg_data = {
	.sdram_config = MT41K256M16HA125E_EMIF_SDCFG,
	.ref_ctrl = MT41K256M16HA125E_EMIF_SDREF,
	.sdram_tim1 = MT41K256M16HA125E_EMIF_TIM1,
	.sdram_tim2 = MT41K256M16HA125E_EMIF_TIM2,
	.sdram_tim3 = MT41K256M16HA125E_EMIF_TIM3,
	.zq_config = MT41K256M16HA125E_ZQ_CFG,
	.emif_ddr_phy_ctlr_1 = MT41K256M16HA125E_EMIF_READ_LATENCY,
};

static struct emif_regs ddr3_kv3_emif_reg_data = {
	.sdram_config = MT41K64M16TW107J_EMIF_SDCFG,
	.ref_ctrl = MT41K64M16TW107J_EMIF_SDREF,
	.sdram_tim1 = MT41K64M16TW107J_EMIF_TIM1,
	.sdram_tim2 = MT41K64M16TW107J_EMIF_TIM2,
	.sdram_tim3 = MT41K64M16TW107J_EMIF_TIM3,
	.zq_config = MT41K64M16TW107J_ZQ_CFG,
	.emif_ddr_phy_ctlr_1 = MT41K64M16TW107J_EMIF_READ_LATENCY,
};

static struct emif_regs ddr3_evm_emif_reg_data = {
	.sdram_config = MT41J512M8RH125_EMIF_SDCFG,
	.ref_ctrl = MT41J512M8RH125_EMIF_SDREF,
	.sdram_tim1 = MT41J512M8RH125_EMIF_TIM1,
	.sdram_tim2 = MT41J512M8RH125_EMIF_TIM2,
	.sdram_tim3 = MT41J512M8RH125_EMIF_TIM3,
	.zq_config = MT41J512M8RH125_ZQ_CFG,
	.emif_ddr_phy_ctlr_1 = MT41J512M8RH125_EMIF_READ_LATENCY |
				PHY_EN_DYN_PWRDN,
};

static struct emif_regs ddr3_icev2_emif_reg_data = {
	.sdram_config = MT41J128MJT125_EMIF_SDCFG_400MHz,
	.ref_ctrl = MT41J128MJT125_EMIF_SDREF_400MHz,
	.sdram_tim1 = MT41J128MJT125_EMIF_TIM1_400MHz,
	.sdram_tim2 = MT41J128MJT125_EMIF_TIM2_400MHz,
	.sdram_tim3 = MT41J128MJT125_EMIF_TIM3_400MHz,
	.zq_config = MT41J128MJT125_ZQ_CFG_400MHz,
	.emif_ddr_phy_ctlr_1 = MT41J128MJT125_EMIF_READ_LATENCY_400MHz |
				PHY_EN_DYN_PWRDN,
};

#ifdef CONFIG_SPL_OS_BOOT
int spl_start_uboot(void)
{
	/* break into full u-boot on 'c' */
	if (serial_tstc() && serial_getc() == 'c')
		return 1;

#ifdef CONFIG_SPL_ENV_SUPPORT
	env_init();
	env_relocate_spec();
	if (getenv_yesno("boot_os") != 1)
		return 1;
#endif

	return 0;
}
#endif

#define OSC	(V_OSCK/1000000)
const struct dpll_params dpll_ddr = {
		266, OSC-1, 1, -1, -1, -1, -1};
const struct dpll_params dpll_ddr_evm_sk = {
		303, OSC-1, 1, -1, -1, -1, -1};
const struct dpll_params dpll_ddr_bone_black = {
		400, OSC-1, 1, -1, -1, -1, -1};

void am33xx_spl_board_init(void)
{
	int mpu_vdd;

	if (read_eeprom() < 0)
		puts("Could not get board ID.\n");

	/* Get the frequency */
	dpll_mpu_opp100.m = am335x_get_efuse_mpu_max_freq(cdev);

	if (board_is_bone() || board_is_bone_lt() || board_is_kv3()) {
		/* BeagleBone PMIC Code */
		int usb_cur_lim;

		/*
		 * Only perform PMIC configurations if board rev > A1
		 * on Beaglebone White
		 */
		if (board_is_bone() && !strncmp(board_ti_get_rev(), "00A1", 4))
			return;

		if (i2c_probe(TPS65217_CHIP_PM))
			return;

		/*
		 * On Beaglebone White we need to ensure we have AC power
		 * before increasing the frequency.
		 */
		if (board_is_bone()) {
			uchar pmic_status_reg;
			if (tps65217_reg_read(TPS65217_STATUS,
					      &pmic_status_reg))
				return;
			if (!(pmic_status_reg & TPS65217_PWR_SRC_AC_BITMASK)) {
				puts("No AC power, disabling frequency switch\n");
				return;
			}
		}

		/*
		 * Override what we have detected since we know if we have
		 * a Beaglebone Black it supports 1GHz.
		 */
		if (board_is_bone_lt())
			dpll_mpu_opp100.m = MPUPLL_M_1000;

		/*
		 * Increase USB current limit to 1300mA or 1800mA and set
		 * the MPU voltage controller as needed.
		 */
		if (dpll_mpu_opp100.m == MPUPLL_M_1000) {
			usb_cur_lim = TPS65217_USB_INPUT_CUR_LIMIT_1800MA;
			mpu_vdd = TPS65217_DCDC_VOLT_SEL_1325MV;
		} else {
			usb_cur_lim = TPS65217_USB_INPUT_CUR_LIMIT_1300MA;
			mpu_vdd = TPS65217_DCDC_VOLT_SEL_1275MV;
		}

		if (tps65217_reg_write(TPS65217_PROT_LEVEL_NONE,
				       TPS65217_POWER_PATH,
				       usb_cur_lim,
				       TPS65217_USB_INPUT_CUR_LIMIT_MASK))
			puts("tps65217_reg_write failure\n");

		/* Set DCDC3 (CORE) voltage to 1.125V */
		if (tps65217_voltage_update(TPS65217_DEFDCDC3,
					    TPS65217_DCDC_VOLT_SEL_1125MV)) {
			puts("tps65217_voltage_update failure\n");
			return;
		}

		/* Set CORE Frequencies to OPP100 */
		do_setup_dpll(&dpll_core_regs, &dpll_core_opp100);

		/* Set DCDC2 (MPU) voltage */
		if (tps65217_voltage_update(TPS65217_DEFDCDC2, mpu_vdd)) {
			puts("tps65217_voltage_update failure\n");
			return;
		}

		/*
		 * Set LDO3, LDO4 output voltage to 3.3V for Beaglebone.
		 * Set LDO3 to 1.8V and LDO4 to 3.3V for Beaglebone Black.
		 */
		if (board_is_bone()) {
			if (tps65217_reg_write(TPS65217_PROT_LEVEL_2,
					       TPS65217_DEFLS1,
					       TPS65217_LDO_VOLTAGE_OUT_3_3,
					       TPS65217_LDO_MASK))
				puts("tps65217_reg_write failure\n");
		} else {
			if (tps65217_reg_write(TPS65217_PROT_LEVEL_2,
					       TPS65217_DEFLS1,
					       TPS65217_LDO_VOLTAGE_OUT_1_8,
					       TPS65217_LDO_MASK))
				puts("tps65217_reg_write failure\n");
		}

		if (tps65217_reg_write(TPS65217_PROT_LEVEL_2,
				       TPS65217_DEFLS2,
				       TPS65217_LDO_VOLTAGE_OUT_3_3,
				       TPS65217_LDO_MASK))
			puts("tps65217_reg_write failure\n");
	} else {
		int sil_rev;

		/*
		 * The GP EVM, IDK and EVM SK use a TPS65910 PMIC.  For all
		 * MPU frequencies we support we use a CORE voltage of
		 * 1.1375V.  For MPU voltage we need to switch based on
		 * the frequency we are running at.
		 */
		if (i2c_probe(TPS65910_CTRL_I2C_ADDR))
			return;

		/*
		 * Depending on MPU clock and PG we will need a different
		 * VDD to drive at that speed.
		 */
		sil_rev = readl(&cdev->deviceid) >> 28;
		mpu_vdd = am335x_get_tps65910_mpu_vdd(sil_rev,
						      dpll_mpu_opp100.m);

		/* Tell the TPS65910 to use i2c */
		tps65910_set_i2c_control();

		/* First update MPU voltage. */
		if (tps65910_voltage_update(MPU, mpu_vdd))
			return;

		/* Second, update the CORE voltage. */
		if (tps65910_voltage_update(CORE, TPS65910_OP_REG_SEL_1_1_3))
			return;

		/* Set CORE Frequencies to OPP100 */
		do_setup_dpll(&dpll_core_regs, &dpll_core_opp100);
	}

	/* Set MPU Frequency to what we detected now that voltages are set */
	do_setup_dpll(&dpll_mpu_regs, &dpll_mpu_opp100);
}

const struct dpll_params *get_dpll_ddr_params(void)
{
	enable_i2c0_pin_mux();
	i2c_init(CONFIG_SYS_OMAP24_I2C_SPEED, CONFIG_SYS_OMAP24_I2C_SLAVE);
	if (read_eeprom() < 0)
		puts("Could not get board ID.\n");

	if (board_is_evm_sk())
		return &dpll_ddr_evm_sk;
	else if (board_is_bone_lt() || board_is_icev2() || board_is_kv3())
		return &dpll_ddr_bone_black;
	else if (board_is_evm_15_or_later())
		return &dpll_ddr_evm_sk;
	else
		return &dpll_ddr;
}

void set_uart_mux_conf(void)
{
#if CONFIG_CONS_INDEX == 1
	enable_uart0_pin_mux();
#elif CONFIG_CONS_INDEX == 2
	enable_uart1_pin_mux();
#elif CONFIG_CONS_INDEX == 3
	enable_uart2_pin_mux();
#elif CONFIG_CONS_INDEX == 4
	enable_uart3_pin_mux();
#elif CONFIG_CONS_INDEX == 5
	enable_uart4_pin_mux();
#elif CONFIG_CONS_INDEX == 6
	enable_uart5_pin_mux();
#endif
}

void set_mux_conf_regs(void)
{
	if (read_eeprom() < 0)
		puts("Could not get board ID.\n");

	enable_board_pin_mux();
}

const struct ctrl_ioregs ioregs_evmsk = {
	.cm0ioctl		= MT41J128MJT125_IOCTRL_VALUE,
	.cm1ioctl		= MT41J128MJT125_IOCTRL_VALUE,
	.cm2ioctl		= MT41J128MJT125_IOCTRL_VALUE,
	.dt0ioctl		= MT41J128MJT125_IOCTRL_VALUE,
	.dt1ioctl		= MT41J128MJT125_IOCTRL_VALUE,
};

const struct ctrl_ioregs ioregs_bonelt = {
	.cm0ioctl		= MT41K256M16HA125E_IOCTRL_VALUE,
	.cm1ioctl		= MT41K256M16HA125E_IOCTRL_VALUE,
	.cm2ioctl		= MT41K256M16HA125E_IOCTRL_VALUE,
	.dt0ioctl		= MT41K256M16HA125E_IOCTRL_VALUE,
	.dt1ioctl		= MT41K256M16HA125E_IOCTRL_VALUE,
};

const struct ctrl_ioregs ioregs_kv3 = {
	.cm0ioctl		= MT41K64M16TW107J_IOCTRL_VALUE,
	.cm1ioctl		= MT41K64M16TW107J_IOCTRL_VALUE,
	.cm2ioctl		= MT41K64M16TW107J_IOCTRL_VALUE,
	.dt0ioctl		= MT41K64M16TW107J_IOCTRL_VALUE,
	.dt1ioctl		= MT41K64M16TW107J_IOCTRL_VALUE,
};

const struct ctrl_ioregs ioregs_evm15 = {
	.cm0ioctl		= MT41J512M8RH125_IOCTRL_VALUE,
	.cm1ioctl		= MT41J512M8RH125_IOCTRL_VALUE,
	.cm2ioctl		= MT41J512M8RH125_IOCTRL_VALUE,
	.dt0ioctl		= MT41J512M8RH125_IOCTRL_VALUE,
	.dt1ioctl		= MT41J512M8RH125_IOCTRL_VALUE,
};

const struct ctrl_ioregs ioregs = {
	.cm0ioctl		= MT47H128M16RT25E_IOCTRL_VALUE,
	.cm1ioctl		= MT47H128M16RT25E_IOCTRL_VALUE,
	.cm2ioctl		= MT47H128M16RT25E_IOCTRL_VALUE,
	.dt0ioctl		= MT47H128M16RT25E_IOCTRL_VALUE,
	.dt1ioctl		= MT47H128M16RT25E_IOCTRL_VALUE,
};

void sdram_init(void)
{
	if (read_eeprom() < 0)
		puts("Could not get board ID.\n");

	if (board_is_evm_sk()) {
		/*
		 * EVM SK 1.2A and later use gpio0_7 to enable DDR3.
		 * This is safe enough to do on older revs.
		 */
		gpio_request(GPIO_DDR_VTT_EN, "ddr_vtt_en");
		gpio_direction_output(GPIO_DDR_VTT_EN, 1);
	}

	if (board_is_icev2()) {
		gpio_request(ICE_GPIO_DDR_VTT_EN, "ddr_vtt_en");
		gpio_direction_output(ICE_GPIO_DDR_VTT_EN, 1);
	}

	if (board_is_evm_sk())
		config_ddr(303, &ioregs_evmsk, &ddr3_data,
			   &ddr3_cmd_ctrl_data, &ddr3_emif_reg_data, 0);
	else if (board_is_bone_lt())
		config_ddr(400, &ioregs_bonelt,
			   &ddr3_beagleblack_data,
			   &ddr3_beagleblack_cmd_ctrl_data,
			   &ddr3_beagleblack_emif_reg_data, 0);
	else if (board_is_kv3())
		config_ddr(400, &ioregs_kv3,
			  &ddr3_kv3_data,
			  &ddr3_kv3_cmd_ctrl_data,
			  &ddr3_kv3_emif_reg_data, 0);
	else if (board_is_evm_15_or_later())
		config_ddr(303, &ioregs_evm15, &ddr3_evm_data,
			   &ddr3_evm_cmd_ctrl_data, &ddr3_evm_emif_reg_data, 0);
	else if (board_is_icev2())
		config_ddr(400, &ioregs_evmsk, &ddr3_icev2_data,
			   &ddr3_icev2_cmd_ctrl_data, &ddr3_icev2_emif_reg_data,
			   0);
	else
		config_ddr(266, &ioregs, &ddr2_data,
			   &ddr2_cmd_ctrl_data, &ddr2_emif_reg_data, 0);
}
#endif

/*
 * Basic board specific setup.  Pinmux has been handled already.
 */
int board_init(void)
{
#if defined(CONFIG_HW_WATCHDOG)
	hw_watchdog_init();
#endif

	gd->bd->bi_boot_params = CONFIG_SYS_SDRAM_BASE + 0x100;
#if defined(CONFIG_NOR) || defined(CONFIG_NAND)
	gpmc_init();
#endif
	return 0;
}

#ifdef CONFIG_BOARD_LATE_INIT
int board_late_init(void)
{
#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
	set_board_info_env(NULL);

    char buf[4];
    sprintf(buf, "%d", get_sysboot_value());
    setenv("sys_boot_value", buf);
#endif

	return 0;
}
#endif

#if (defined(CONFIG_DRIVER_TI_CPSW) && !defined(CONFIG_SPL_BUILD)) || \
	(defined(CONFIG_SPL_ETH_SUPPORT) && defined(CONFIG_SPL_BUILD))
static void cpsw_control(int enabled)
{
	/* VTP can be added here */

	return;
}

static struct cpsw_slave_data cpsw_slaves[] = {
	{
		.slave_reg_ofs	= 0x208,
		.sliver_reg_ofs	= 0xd80,
		.phy_addr	= 0,
	},
	{
		.slave_reg_ofs	= 0x308,
		.sliver_reg_ofs	= 0xdc0,
		.phy_addr	= 1,
	},
};

static struct cpsw_platform_data cpsw_data = {
	.mdio_base		= CPSW_MDIO_BASE,
	.cpsw_base		= CPSW_BASE,
	.mdio_div		= 0xff,
	.channels		= 8,
	.cpdma_reg_ofs		= 0x800,
	.slaves			= 1,
	.slave_data		= cpsw_slaves,
	.ale_reg_ofs		= 0xd00,
	.ale_entries		= 1024,
	.host_port_reg_ofs	= 0x108,
	.hw_stats_reg_ofs	= 0x900,
	.bd_ram_ofs		= 0x2000,
	.mac_control		= (1 << 5),
	.control		= cpsw_control,
	.host_port_num		= 0,
	.version		= CPSW_CTRL_VERSION_2,
};

static void request_and_set_gpio(int gpio, char *name)
{
	int ret;

	ret = gpio_request(gpio, name);
	if (ret < 0) {
		printf("%s: Unable to request %s\n", __func__, name);
		return;
	}

	ret = gpio_direction_output(gpio, 0);
	if (ret < 0) {
		printf("%s: Unable to set %s  as output\n", __func__, name);
		goto err_free_gpio;
	}

	gpio_set_value(gpio, 1);

	return;

err_free_gpio:
	gpio_free(gpio);
}

#define REQUEST_AND_SET_GPIO(N)	request_and_set_gpio(N, #N);

/**
 * RMII mode on ICEv2 board needs 50MHz clock. Given the clock
 * synthesizer With a capacitor of 18pF, and 25MHz input clock cycle
 * PLL1 gives an output of 100MHz. So, configuring the div2/3 as 2 to
 * give 50MHz output for Eth0 and 1.
 */
static struct clk_synth cdce913_data = {
	.id = 0x81,
	.capacitor = 0x90,
	.mux = 0x6d,
	.pdiv2 = 0x2,
	.pdiv3 = 0x2,
};
#endif

#if ((defined(CONFIG_SPL_ETH_SUPPORT) || defined(CONFIG_SPL_USBETH_SUPPORT)) &&\
	defined(CONFIG_SPL_BUILD)) || \
	((defined(CONFIG_DRIVER_TI_CPSW) || \
	  defined(CONFIG_USB_ETHER) && defined(CONFIG_MUSB_GADGET)) && \
	 !defined(CONFIG_SPL_BUILD))

/*
 * This function will:
 * Read the eFuse for MAC addresses, and set ethaddr/eth1addr/usbnet_devaddr
 * in the environment
 * Perform fixups to the PHY present on certain boards.  We only need this
 * function in:
 * - SPL with either CPSW or USB ethernet support
 * - Full U-Boot, with either CPSW or USB ethernet
 * Build in only these cases to avoid warnings about unused variables
 * when we build an SPL that has neither option but full U-Boot will.
 */
int board_eth_init(bd_t *bis)
{
	int rv, n = 0;
	uint8_t mac_addr[6];
	uint32_t mac_hi, mac_lo;

	/* try reading mac address from efuse */
	mac_lo = readl(&cdev->macid0l);
	mac_hi = readl(&cdev->macid0h);
	mac_addr[0] = mac_hi & 0xFF;
	mac_addr[1] = (mac_hi & 0xFF00) >> 8;
	mac_addr[2] = (mac_hi & 0xFF0000) >> 16;
	mac_addr[3] = (mac_hi & 0xFF000000) >> 24;
	mac_addr[4] = mac_lo & 0xFF;
	mac_addr[5] = (mac_lo & 0xFF00) >> 8;

#if (defined(CONFIG_DRIVER_TI_CPSW) && !defined(CONFIG_SPL_BUILD)) || \
	(defined(CONFIG_SPL_ETH_SUPPORT) && defined(CONFIG_SPL_BUILD))
	if (!getenv("ethaddr")) {
		printf("<ethaddr> not set. Validating first E-fuse MAC\n");

		if (is_valid_ethaddr(mac_addr))
			eth_setenv_enetaddr("ethaddr", mac_addr);
	}

#ifdef CONFIG_DRIVER_TI_CPSW

	mac_lo = readl(&cdev->macid1l);
	mac_hi = readl(&cdev->macid1h);
	mac_addr[0] = mac_hi & 0xFF;
	mac_addr[1] = (mac_hi & 0xFF00) >> 8;
	mac_addr[2] = (mac_hi & 0xFF0000) >> 16;
	mac_addr[3] = (mac_hi & 0xFF000000) >> 24;
	mac_addr[4] = mac_lo & 0xFF;
	mac_addr[5] = (mac_lo & 0xFF00) >> 8;

	if (!getenv("eth1addr")) {
		if (is_valid_ethaddr(mac_addr))
			eth_setenv_enetaddr("eth1addr", mac_addr);
	}

	if (board_is_icev2()) {
		REQUEST_AND_SET_GPIO(GPIO_PR1_MII_CTRL);
		REQUEST_AND_SET_GPIO(GPIO_MUX_MII_CTRL);
		REQUEST_AND_SET_GPIO(GPIO_FET_SWITCH_CTRL);
		REQUEST_AND_SET_GPIO(GPIO_PHY_RESET);

		rv = setup_clock_synthesizer(&cdce913_data);
		if (rv) {
			printf("Clock synthesizer setup failed %d\n", rv);
			return rv;
		}
	}

	if (read_eeprom() < 0)
		puts("Could not get board ID.\n");

	if (board_is_bone() || board_is_bone_lt() || board_is_idk() || board_is_kv3()) {
		writel(MII_MODE_ENABLE, &cdev->miisel);
		cpsw_slaves[0].phy_if = cpsw_slaves[1].phy_if =
				PHY_INTERFACE_MODE_MII;
	} else if (board_is_icev2()) {
		writel(RMII_MODE_ENABLE | RMII_CHIPCKL_ENABLE, &cdev->miisel);
		cpsw_slaves[0].phy_if = PHY_INTERFACE_MODE_RMII;
		cpsw_slaves[1].phy_if = PHY_INTERFACE_MODE_RMII;
		cpsw_slaves[0].phy_addr = 1;
		cpsw_slaves[1].phy_addr = 3;
	} else {
		writel((RGMII_MODE_ENABLE | RGMII_INT_DELAY), &cdev->miisel);
		cpsw_slaves[0].phy_if = cpsw_slaves[1].phy_if =
				PHY_INTERFACE_MODE_RGMII;
	}

	rv = cpsw_register(&cpsw_data);
	if (rv < 0)
		printf("Error %d registering CPSW switch\n", rv);
	else
		n += rv;
#endif

	/*
	 *
	 * CPSW RGMII Internal Delay Mode is not supported in all PVT
	 * operating points.  So we must set the TX clock delay feature
	 * in the AR8051 PHY.  Since we only support a single ethernet
	 * device in U-Boot, we only do this for the first instance.
	 */
#define AR8051_PHY_DEBUG_ADDR_REG	0x1d
#define AR8051_PHY_DEBUG_DATA_REG	0x1e
#define AR8051_DEBUG_RGMII_CLK_DLY_REG	0x5
#define AR8051_RGMII_TX_CLK_DLY		0x100

	if (board_is_evm_sk() || board_is_gp_evm()) {
		const char *devname;
		devname = miiphy_get_current_dev();

		miiphy_write(devname, 0x0, AR8051_PHY_DEBUG_ADDR_REG,
				AR8051_DEBUG_RGMII_CLK_DLY_REG);
		miiphy_write(devname, 0x0, AR8051_PHY_DEBUG_DATA_REG,
				AR8051_RGMII_TX_CLK_DLY);
	}
#endif
#if defined(CONFIG_USB_ETHER) && \
	(!defined(CONFIG_SPL_BUILD) || defined(CONFIG_SPL_USBETH_SUPPORT))
	if (is_valid_ethaddr(mac_addr))
		eth_setenv_enetaddr("usbnet_devaddr", mac_addr);

	rv = usb_eth_initialize(bis);
	if (rv < 0)
		printf("Error %d registering USB_ETHER\n", rv);
	else
		n += rv;
#endif
	return n;
}
#endif

#ifndef CONFIG_SPL_BUILD
#define CMD 0x00
#define DAT 0x01
#define END 0x02

#define CORDIAL_REV1	2
#define CORDIAL_REV2	3

typedef struct {
    uint8_t type;
        uint8_t cmd_or_data;
    uint16_t delay;
} cordial_init_seq_t;

static const cordial_init_seq_t cordial_init_sequence_rev1[] = {
    {CMD, 0xFF, 0},
        {DAT, 0xFF, 0}, {DAT, 0x98, 0}, {DAT, 0x06, 0}, {DAT, 0x04, 0}, {DAT, 0x00, 0},
    {CMD, 0xFF, 0},
        {DAT, 0xFF, 0}, {DAT, 0x98, 0}, {DAT, 0x06, 0}, {DAT, 0x04, 0}, {DAT, 0x01, 0},
    {CMD, 0x08, 0},
        {DAT, 0x10, 0},
    {CMD, 0x21, 0},
        {DAT, 0x03, 0},
    {CMD, 0x30, 0},
        {DAT, 0x02, 0},
    {CMD, 0x31, 0},
        {DAT, 0x00, 0},
    {CMD, 0x60, 0},
        {DAT, 0x07, 0},
    {CMD, 0x61, 0},
        {DAT, 0x06, 0},
    {CMD, 0x62, 0},
        {DAT, 0x06, 0},
    {CMD, 0x63, 0},
        {DAT, 0x04, 0},
    {CMD, 0x40, 0},
        {DAT, 0x14, 0},
    {CMD, 0x41, 0},
        {DAT, 0x55, 0},
    {CMD, 0x42, 0},
        {DAT, 0x01, 0},
    {CMD, 0x43, 0},
        {DAT, 0x09, 0},
    {CMD, 0x44, 0},
        {DAT, 0x0C, 0},
    {CMD, 0x45, 0},
        {DAT, 0x14, 0},
    {CMD, 0x50, 0},
        {DAT, 0x50, 0},
    {CMD, 0x51, 0},
        {DAT, 0x50, 0},
    {CMD, 0x52, 0},
        {DAT, 0x00, 0},
    {CMD, 0x53, 0},
        {DAT, 0x42, 0},
    {CMD, 0xA0, 0},
        {DAT, 0x00, 0},
    {CMD, 0xA1, 0},
        {DAT, 0x09, 0},
    {CMD, 0xA2, 0},
        {DAT, 0x0C, 0},
    {CMD, 0xA3, 0},
        {DAT, 0x0F, 0},
    {CMD, 0xA4, 0},
        {DAT, 0x06, 0},
    {CMD, 0xA5, 0},
        {DAT, 0x09, 0},
    {CMD, 0xA6, 0},
        {DAT, 0x07, 0},
    {CMD, 0xA7, 0},
        {DAT, 0x01, 0},
    {CMD, 0xA8, 0},
        {DAT, 0x06, 0},
    {CMD, 0xA9, 0},
        {DAT, 0x09, 0},
    {CMD, 0xAA, 0},
        {DAT, 0x11, 0},
    {CMD, 0xAB, 0},
        {DAT, 0x06, 0},
    {CMD, 0xAC, 0},
        {DAT, 0x0E, 0},
    {CMD, 0xAD, 0},
        {DAT, 0x19, 0},
    {CMD, 0xAE, 0},
        {DAT, 0x0C, 0},
    {CMD, 0xAF, 0},
        {DAT, 0x00, 0},
    {CMD, 0xC0, 0},
        {DAT, 0x00, 0},
    {CMD, 0xC1, 0},
        {DAT, 0x09, 0},
    {CMD, 0xC2, 0},
        {DAT, 0x0C, 0},
    {CMD, 0xC3, 0},
        {DAT, 0x0F, 0},
    {CMD, 0xC4, 0},
        {DAT, 0x06, 0},
    {CMD, 0xC5, 0},
        {DAT, 0x09, 0},
    {CMD, 0xC6, 0},
        {DAT, 0x07, 0},
    {CMD, 0xC7, 0},
        {DAT, 0x16, 0},
    {CMD, 0xC8, 0},
        {DAT, 0x06, 0},
    {CMD, 0xC9, 0},
        {DAT, 0x09, 0},
    {CMD, 0xCA, 0},
        {DAT, 0x11, 0},
    {CMD, 0xCB, 0},
        {DAT, 0x06, 0},
    {CMD, 0xCC, 0},
        {DAT, 0x0E, 0},
    {CMD, 0xCD, 0},
        {DAT, 0x19, 0},
    {CMD, 0xCE, 0},
        {DAT, 0x0C, 0},
    {CMD, 0xCF, 0},
        {DAT, 0x00, 0},
    {CMD, 0xFF, 0},
        {DAT, 0xFF, 0}, {DAT, 0x98, 0}, {DAT, 0x06, 0}, {DAT, 0x04, 0}, {DAT, 0x07, 0},
    {CMD, 0x17, 0},
        {DAT, 0x32, 20},  /* stops here mysteriously*/
    {CMD, 0x06, 0},
        {DAT, 0x13, 20},
    {CMD, 0x02, 0},
        {DAT, 0x77, 20},
    {CMD, 0x18, 0},
        {DAT, 0x1D, 20},
    {CMD, 0xE1, 0},
        {DAT, 0x79, 20},
    {CMD, 0xFF, 0},
        {DAT, 0xFF, 0}, {DAT, 0x98, 0}, {DAT, 0x06, 0}, {DAT, 0x04, 0}, {DAT, 0x06, 0},
    {CMD, 0x00, 0},
        {DAT, 0xA0, 0},
    {CMD, 0x01, 0},
        {DAT, 0x05, 0},
    {CMD, 0x02, 0},
        {DAT, 0x00, 0},
    {CMD, 0x03, 0},
        {DAT, 0x00, 0},
    {CMD, 0x04, 0},
        {DAT, 0x01, 0},
    {CMD, 0x05, 0},
        {DAT, 0x01, 0},
    {CMD, 0x06, 0},
        {DAT, 0x88, 0},
    {CMD, 0x07, 0},
        {DAT, 0x04, 0},
    {CMD, 0x08, 0},
        {DAT, 0x01, 0},
    {CMD, 0x09, 0},
        {DAT, 0x90, 0},
    {CMD, 0x0A, 0},
        {DAT, 0x04, 0},
    {CMD, 0x0B, 0},
        {DAT, 0x01, 0},
    {CMD, 0x0C, 0},
        {DAT, 0x01, 0},
    {CMD, 0x0D, 0},
        {DAT, 0x01, 0},
    {CMD, 0x0E, 0},
        {DAT, 0x00, 0},
    {CMD, 0x0F, 0},
        {DAT, 0x00, 0},
    {CMD, 0x10, 0},
        {DAT, 0x55, 0},
    {CMD, 0x11, 0},
        {DAT, 0x50, 0},
    {CMD, 0x12, 0},
        {DAT, 0x01, 0},
    {CMD, 0x13, 0},
        {DAT, 0x85, 0},
    {CMD, 0x14, 0},
        {DAT, 0x85, 0},
    {CMD, 0x15, 0},
        {DAT, 0xC0, 0},
    {CMD, 0x16, 0},
        {DAT, 0x0B, 0},
    {CMD, 0x17, 0},
        {DAT, 0x00, 0},
    {CMD, 0x18, 0},
        {DAT, 0x00, 0},
    {CMD, 0x19, 0},
        {DAT, 0x00, 0},
    {CMD, 0x1A, 0},
        {DAT, 0x00, 0},
    {CMD, 0x1B, 0},
        {DAT, 0x00, 0},
    {CMD, 0x1C, 0},
        {DAT, 0x00, 0},
    {CMD, 0x1D, 0},
        {DAT, 0x00, 0},
    {CMD, 0x20, 0},
        {DAT, 0x01, 0},
    {CMD, 0x21, 0},
        {DAT, 0x23, 0},
    {CMD, 0x22, 0},
        {DAT, 0x45, 0},
    {CMD, 0x23, 0},
        {DAT, 0x67, 0},
    {CMD, 0x24, 0},
        {DAT, 0x01, 0},
    {CMD, 0x25, 0},
        {DAT, 0x23, 0},
    {CMD, 0x26, 0},
        {DAT, 0x45, 0},
    {CMD, 0x27, 0},
        {DAT, 0x67, 0},
    {CMD, 0x30, 0},
        {DAT, 0x02, 0},
    {CMD, 0x31, 0},
        {DAT, 0x22, 0},
    {CMD, 0x32, 0},
        {DAT, 0x11, 0},
    {CMD, 0x33, 0},
        {DAT, 0xAA, 0},
    {CMD, 0x34, 0},
        {DAT, 0xBB, 0},
    {CMD, 0x35, 0},
        {DAT, 0x66, 0},
    {CMD, 0x36, 0},
        {DAT, 0x00, 0},
    {CMD, 0x37, 0},
        {DAT, 0x22, 0},
    {CMD, 0x38, 0},
        {DAT, 0x22, 0},
    {CMD, 0x39, 0},
        {DAT, 0x22, 0},
    {CMD, 0x3A, 0},
        {DAT, 0x22, 0},
    {CMD, 0x3B, 0},
        {DAT, 0x22, 0},
    {CMD, 0x3C, 0},
        {DAT, 0x22, 0},
    {CMD, 0x3D, 0},
        {DAT, 0x22, 0},
    {CMD, 0x3E, 0},
        {DAT, 0x22, 0},
    {CMD, 0x3F, 0},
        {DAT, 0x22, 0},
    {CMD, 0x40, 0},
        {DAT, 0x22, 0},
    {CMD, 0x52, 0},
        {DAT, 0x12, 0},
    {CMD, 0x53, 0},
        {DAT, 0x12, 0},
    {CMD, 0xFF, 0},
        {DAT, 0xFF, 0}, {DAT, 0x98, 0}, {DAT, 0x06, 0}, {DAT, 0x04, 0}, {DAT, 0x00, 0},
    {CMD, 0x3A, 0},
        {DAT, 0x70, 0},

    {CMD, 0x11, 120}, /* Exit Sleep Mode */
    {CMD, 0x29, 120}, /* Set Display On */
    {END}
};

static const cordial_init_seq_t cordial_init_sequence_rev2[] = {
    {CMD, 0xFF, 0}, 
        {DAT, 0xFF, 0}, {DAT, 0x98, 0}, {DAT, 0x06, 0}, {DAT, 0x04, 0}, {DAT, 0x01, 0},
    {CMD, 0xFF, 0}, 
        {DAT, 0xFF, 0}, {DAT, 0x98, 0}, {DAT, 0x06, 0}, {DAT, 0x04, 0}, {DAT, 0x01, 0},
    {CMD, 0x08, 0}, 	//Output   SDA
        {DAT, 0x10, 0},
    {CMD, 0x021, 0},   	//DE = 1 Active
        {DAT, 0x01, 0},
    {CMD, 0x30, 0}, 	//Resolution setting 480 X 800
        {DAT, 0x02, 0},
    {CMD, 0x31, 0}, 	//Inversion setting for 2-dot
        {DAT, 0x02, 0},  // 02
    {CMD, 0x40, 0}, 	//
        {DAT, 0x16, 0},
    {CMD, 0x41, 0}, 	//
        {DAT, 0x33, 0},
    {CMD, 0x42, 0}, 	//
        {DAT, 0x02, 0},
    {CMD, 0x43, 0},
        {DAT, 0x09, 0},
    {CMD, 0x44, 0},
        {DAT, 0x09, 0},
    {CMD, 0x50, 0},
        {DAT, 0x78, 0},
    {CMD, 0x51, 0},
        {DAT, 0x78, 0},
    {CMD, 0x52, 0},
        {DAT, 0x00, 0},
    {CMD, 0x53, 0},
        {DAT, 0x5E, 0},
    {CMD, 0x60, 0},
        {DAT, 0x07, 0},
    {CMD, 0x61, 0},
        {DAT, 0x00, 0},
    {CMD, 0x62, 0},
        {DAT, 0x08, 0},
    {CMD, 0x63, 0},
        {DAT, 0x00, 0},
    {CMD, 0xFF, 0}, 
        {DAT, 0xFF, 0}, {DAT, 0x98, 0}, {DAT, 0x06, 0}, {DAT, 0x04, 0}, {DAT, 0x01, 0},
    {CMD, 0xA0, 0},
        {DAT, 0x00, 0},
    {CMD, 0xA1, 0},
        {DAT, 0x1B, 0},
    {CMD, 0xA2, 0},
        {DAT, 0x24, 0},
    {CMD, 0xA3, 0},
        {DAT, 0x11, 0},
    {CMD, 0xA4, 0},
 {DAT, 0x07, 0},
    {CMD, 0xA5, 0},
        {DAT, 0x0C, 0},
    {CMD, 0xA6, 0},
        {DAT, 0x08, 0},
    {CMD, 0xA7, 0},
        {DAT, 0x05, 0},
    {CMD, 0xA8, 0},
        {DAT, 0x06, 0},
    {CMD, 0xA9, 0},
        {DAT, 0x0B, 0},
    {CMD, 0xAA, 0},
        {DAT, 0x0E, 0},
    {CMD, 0xAB, 0},
        {DAT, 0x07, 0},
    {CMD, 0xAC, 0},
        {DAT, 0x0E, 0},
    {CMD, 0xAD, 0},
        {DAT, 0x12, 0},
    {CMD, 0xAE, 0},
        {DAT, 0x0C, 0},
    {CMD, 0xAF, 0},
        {DAT, 0x00, 0},
    {CMD, 0xC0, 0},
        {DAT, 0x00, 0},
    {CMD, 0xC1, 0},
        {DAT, 0x1C, 0},
    {CMD, 0xC2, 0},
        {DAT, 0x24, 0},
    {CMD, 0xC3, 0},
        {DAT, 0x11, 0},
    {CMD, 0xC4, 0},
        {DAT, 0x07, 0},
    {CMD, 0xC5, 0},
        {DAT, 0x0C, 0},
    {CMD, 0xC6, 0},
        {DAT, 0x08, 0},
    {CMD, 0xC7, 0},
        {DAT, 0x06, 0},
    {CMD, 0xC8, 0},
        {DAT, 0x07, 0},
    {CMD, 0xC9, 0},
        {DAT, 0x0A, 0},
    {CMD, 0xCA, 0},
        {DAT, 0x0E, 0},
    {CMD, 0xCB, 0},
        {DAT, 0x07, 0},
    {CMD, 0xCC, 0},
        {DAT, 0x0D, 0},
    {CMD, 0xCD, 0},
        {DAT, 0x11, 0},
    {CMD, 0xCE, 0},
        {DAT, 0x0C, 0},
    {CMD, 0xCF, 0},
        {DAT, 0x00, 0},
    {CMD, 0xFF, 0}, 
        {DAT, 0xFF, 0}, {DAT, 0x98, 0}, {DAT, 0x06, 0}, {DAT, 0x04, 0}, {DAT, 0x06, 0},
    {CMD, 0x00, 0},
        {DAT, 0x20, 0},
    {CMD, 0x01, 0},
        {DAT, 0x04, 0},
    {CMD, 0x02, 0},
        {DAT, 0x00, 0},
    {CMD, 0x03, 0},
        {DAT, 0x00, 0},
    {CMD, 0x04, 0},
        {DAT, 0x16, 0},
    {CMD, 0x05, 0},
        {DAT, 0x16, 0},
    {CMD, 0x06, 0},
        {DAT, 0x88, 0},
    {CMD, 0x07, 0},
        {DAT, 0x02, 0},
    {CMD, 0x08, 0},
        {DAT, 0x01, 0},
    {CMD, 0x09, 0},
        {DAT, 0x00, 0},
    {CMD, 0x0A, 0},
        {DAT, 0x00, 0},
    {CMD, 0x0B, 0},
        {DAT, 0x00, 0},
    {CMD, 0x0C, 0},
        {DAT, 0x16, 0},
    {CMD, 0x0D, 0},
        {DAT, 0x16, 0},
    {CMD, 0x0E, 0},
        {DAT, 0x00, 0},
    {CMD, 0x0F, 0},
        {DAT, 0x00, 0},
    {CMD, 0x10, 0},
        {DAT, 0x50, 0},
    {CMD, 0x11, 0},
        {DAT, 0x52, 0},
    {CMD, 0x12, 0},
        {DAT, 0x00, 0},
    {CMD, 0x13, 0},
        {DAT, 0x00, 0},
    {CMD, 0x14, 0},
        {DAT, 0x00, 0},
    {CMD, 0x15, 0},
        {DAT, 0x43, 0},
    {CMD, 0x16, 0},
        {DAT, 0x08, 0},
    {CMD, 0x17, 0},
        {DAT, 0x00, 0},
    {CMD, 0x18, 0},
        {DAT, 0x00, 0},
    {CMD, 0x19, 0},
        {DAT, 0x00, 0},
    {CMD, 0x1A, 0},
        {DAT, 0x00, 0},
    {CMD, 0x1B, 0},
        {DAT, 0x00, 0},
    {CMD, 0x1C, 0},
        {DAT, 0x00, 0},
    {CMD, 0x1D, 0},
        {DAT, 0x00, 0},
    {CMD, 0x20, 0},
        {DAT, 0x01, 0},
    {CMD, 0x21, 0},
        {DAT, 0x23, 0},
    {CMD, 0x22, 0},
        {DAT, 0x45, 0},
    {CMD, 0x23, 0},
        {DAT, 0x67, 0},
    {CMD, 0x24, 0},
        {DAT, 0x01, 0},
    {CMD, 0x25, 0},
        {DAT, 0x23, 0},
    {CMD, 0x26, 0},
        {DAT, 0x45, 0},
    {CMD, 0x27, 0},
        {DAT, 0x67, 0},
    {CMD, 0x30, 0},
        {DAT, 0x13, 0},
    {CMD, 0x31, 0},
        {DAT, 0x11, 0},
    {CMD, 0x32, 0},
        {DAT, 0x00, 0},
    {CMD, 0x33, 0},
        {DAT, 0x22, 0},
    {CMD, 0x34, 0},
        {DAT, 0x22, 0},
    {CMD, 0x35, 0},
        {DAT, 0x22, 0},
    {CMD, 0x36, 0},
        {DAT, 0x22, 0},
    {CMD, 0x37, 0},
        {DAT, 0xAA, 0},
    {CMD, 0x38, 0},
        {DAT, 0xBB, 0},
    {CMD, 0x39, 0},
        {DAT, 0x66, 0},
    {CMD, 0x3A, 0},
        {DAT, 0x22, 0},
    {CMD, 0x3B, 0},
        {DAT, 0x22, 0},
    {CMD, 0x3C, 0},
        {DAT, 0x22, 0},
    {CMD, 0x3D, 0},
        {DAT, 0x22, 0},
    {CMD, 0x3E, 0},
        {DAT, 0x22, 0},
    {CMD, 0x3F, 0},
        {DAT, 0x22, 0},
    {CMD, 0x40, 0},
        {DAT, 0x22, 0},
    {CMD, 0xFF, 0},
        {DAT, 0xFF, 0}, {DAT, 0x98, 0}, {DAT, 0x06, 0}, {DAT, 0x07, 0}, {DAT, 0x00, 0},
    {CMD, 0x17, 0},
        {DAT, 0x22, 0},
    {CMD, 0x02, 0},
        {DAT, 0x77, 0},
    {CMD, 0xFF, 0},
        {DAT, 0xFF, 0}, {DAT, 0x98, 0}, {DAT, 0x06, 0}, {DAT, 0x04, 0}, {DAT, 0x00, 0},

    {CMD, 0x11, 480}, /* Exit Sleep Mode */
    {CMD, 0x29, 40}, /* Set Display On */
    {END}
};

vidinfo_t	panel_info = {
		.vl_col = 480,
		.vl_row = 800,
		.vl_bpix = 4,
		.priv = 0
};

void lcd_bl_power(int on)
{
	/* Set up the TPS WLED backlight */
	if (tps65217_reg_write(TPS65217_PROT_LEVEL_NONE, TPS65217_WLEDCTRL1,
				on ? 0x8 : 0, 0xf))
		puts("bl: tps65217_reg_write failure1\n");
	if (tps65217_reg_write(TPS65217_PROT_LEVEL_NONE, TPS65217_WLEDCTRL2,
				on ? 50 : 0, 0x7f))
		puts("bl: tps65217_reg_write failure2\n");
}

void lcd_ctrl_init(void *lcdbase)
{
	struct am335x_lcdpanel lcd_panel;
#ifdef CONFIG_USE_FDT
	/* TODO: is there a better place to load the dtb ? */
	load_devicetree();
#endif
	memset(&lcd_panel, 0, sizeof(struct am335x_lcdpanel));

	lcd_panel.hactive = 480;
	lcd_panel.vactive = 800;
	lcd_panel.bpp = 16;
	lcd_panel.hfp = 5;
	lcd_panel.hbp = 5;
	lcd_panel.hsw = 5;
	lcd_panel.vfp = 5;
	lcd_panel.vbp = 5;
	lcd_panel.vsw = 5;
	lcd_panel.pxl_clk_div = 2; /* Taken from kernel register dump */
	lcd_panel.pol = 0x2300000; /* Inverse syncs */
	lcd_panel.pup_delay = 0;
	lcd_panel.pon_delay = 0;

	panel_info.vl_rot = 0;

	/*lcd_panel.panel_power_ctrl = &lcd_bl_power;*/

	if (0 != am335xfb_init(&lcd_panel))
		printf("ERROR: failed to initialize video!");
	/*
	 * modifiy panel info to 'real' resolution, to operate correct with
	 * lcd-framework.
	 */
	panel_info.vl_col = lcd_panel.hactive;
	panel_info.vl_row = lcd_panel.vactive;

	lcd_set_flush_dcache(1);
}

static int spi_send(struct spi_slave *slave, u16 data)
{
	int ret;

	ret = spi_xfer(slave, 9, &data, NULL, SPI_XFER_BEGIN | SPI_XFER_END);
	if (ret)
		return -EIO;

	return ret;
}

static int cordial_spi_send(struct spi_slave *slave, uint8_t type,
			    uint8_t cmd_or_data)
{
	int ret = 0;
	u16 spi_data = (type ? 0x100 : 0) | cmd_or_data;
	
	ret = spi_send(slave, spi_data);
	if (ret)
		puts("error in spi_write\n");

	return ret;
}

static int get_rev(const struct gpio_map *gmp, size_t size, bool invert,
		   unsigned int *revp)
{
	unsigned int i, rev = 0;
	int ret;

	for (i = 0; i < size; i++, gmp++) {
		gpio_request(gmp->gpio, gmp->name);
		gpio_direction_input(gmp->gpio);

		ret = gpio_get_value(gmp->gpio);
		if (ret < 0) {
			printf("Error reading GPIO %d\n", gmp->gpio);
			return -EIO;
		}

		gpio_free(gmp->gpio);

		if ((ret && !invert) || (!ret && invert))
			rev |= BIT(i);
	}

	*revp = rev;
	return 0;
}

static unsigned int get_cordial_rev(void)
{
	unsigned int hw_rev, lcd_rev, rev;
	int ret;

	ret = get_rev(hw_rev_gpios, ARRAY_SIZE(hw_rev_gpios), true,
		      &hw_rev);
	if (ret)
		hw_rev = UINT_MAX; /* Invalid value */

	switch (hw_rev) {
	case 3:
	case 4:
		rev = CORDIAL_REV1;
		break;
	case 5:
		rev = CORDIAL_REV2;
		break;
	default:
		ret = get_rev(lcd_rev_gpios, ARRAY_SIZE(lcd_rev_gpios), false,
			      &lcd_rev);
		if (ret)
			lcd_rev = UINT_MAX; /* Invalid value */

		rev = lcd_rev;
	}

	return rev;
}

void lcd_enable(void)
{
	struct spi_slave *slave;
	cordial_init_seq_t const *cordial_init_seq;
	unsigned int rev;
	int ret;

	/* Take LCD out of reset */
	gpio_request(GPIO_LCD_RESET, "lcd_reset");
	gpio_direction_output(GPIO_LCD_RESET, 0);
	mdelay(100);

	rev = get_cordial_rev();

	switch (rev) {
	default:
		printf("Invalid cordial revision %u, default to 1\n", rev);
		/* FALLTHROUGH */
	case CORDIAL_REV1:
		cordial_init_seq = cordial_init_sequence_rev1;
		break;
	case CORDIAL_REV2:
		cordial_init_seq = cordial_init_sequence_rev2;
		break;
	}

	/* Take the LCD out of reset */
	gpio_direction_output(GPIO_LCD_RESET, 1);

	/* Get SPI slave for all transfers */
	slave = spi_setup_slave(CONFIG_DEFAULT_SPI_BUS,
				CONFIG_DEFAULT_SPI_CHIP,
				CONFIG_DEFAULT_SPI_RATE,
				CONFIG_DEFAULT_SPI_MODE);
	if (!slave) {
		printf("Invalid device %d:%d\n",
			CONFIG_DEFAULT_SPI_BUS, CONFIG_DEFAULT_SPI_CHIP);
		return;
	}

	spi_set_wordlen(slave, 9);

	if (spi_claim_bus(slave))
		goto done;

	/* Send Cordial SPI initialization sequence */
	while (cordial_init_seq->type != END) {
		cordial_spi_send(slave, cordial_init_seq->type,
				 cordial_init_seq->cmd_or_data);
		if (cordial_init_seq->delay > 0)
		    mdelay(cordial_init_seq->delay);
		cordial_init_seq++;
	}

	spi_release_bus(slave);
done:
	spi_free_slave(slave);

	lcd_bl_power(1);
}
#endif
