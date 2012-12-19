/*
 * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Modified from mach-omap/omap2/board-generic.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/io.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/irqdomain.h>

#include <asm/mach/arch.h>

#include <mach/common.h>
#include <mach/cp_intc.h>
#include <mach/da8xx.h>

#define DA8XX_NUM_UARTS	3

void __init da8xx_uart_clk_enable(void)
{
	int i;
	for (i = 0; i < DA8XX_NUM_UARTS; i++)
		davinci_serial_setup_clk(i, NULL);
}

static struct of_device_id da8xx_irq_match[] __initdata = {
	{ .compatible = "ti,cp-intc", .data = cp_intc_of_init, },
	{ }
};

static void __init da8xx_init_irq(void)
{
	of_irq_init(da8xx_irq_match);
}

struct of_dev_auxdata da850_evm_auxdata_lookup[] __initdata = {
	OF_DEV_AUXDATA("ti,davinci-i2c", 0x01c22000, "i2c_davinci.1", NULL),
	OF_DEV_AUXDATA("ti,davinci-i2c", 0x01e28000, "i2c_davinci.2", NULL),
	OF_DEV_AUXDATA("ti,da830-mcasp-audio", 0x01d00000, "davinci-mcasp.0",
			NULL),
	{}
};

#ifdef CONFIG_ARCH_DAVINCI_DA850

/*
 * The following EDMA channels/slots are not being used by drivers (for
 * example: Timer, GPIO, UART events etc) on da850/omap-l138 EVM, hence
 * they are being reserved for codecs on the DSP side.
 */
static const s16 da850_dma0_rsv_chans[][2] = {
	/* (offset, number) */
	{ 8,  6},
	{24,  4},
	{30,  2},
	{-1, -1}
};

static const s16 da850_dma0_rsv_slots[][2] = {
	/* (offset, number) */
	{ 8,  6},
	{24,  4},
	{30, 50},
	{-1, -1}
};

static const s16 da850_dma1_rsv_chans[][2] = {
	/* (offset, number) */
	{ 0, 28},
	{30,  2},
	{-1, -1}
};

static const s16 da850_dma1_rsv_slots[][2] = {
	/* (offset, number) */
	{ 0, 28},
	{30, 90},
	{-1, -1}
};

static struct edma_rsv_info da850_edma_cc0_rsv = {
	.rsv_chans      = da850_dma0_rsv_chans,
	.rsv_slots      = da850_dma0_rsv_slots,
};

static struct edma_rsv_info da850_edma_cc1_rsv = {
	.rsv_chans      = da850_dma1_rsv_chans,
	.rsv_slots      = da850_dma1_rsv_slots,
};

static struct edma_rsv_info *da850_edma_rsv[2] = {
	&da850_edma_cc0_rsv,
	&da850_edma_cc1_rsv,
};

static void __init da850_init_machine(void)
{
	int ret;

	of_platform_populate(NULL, of_default_bus_match_table,
			da850_evm_auxdata_lookup, NULL);

	ret = da850_register_edma(da850_edma_rsv);
	if (ret)
		pr_warn("da850_evm_init: edma registration failed: %d\n", ret);


	da8xx_uart_clk_enable();
}

static const char *da850_boards_compat[] __initdata = {
	"enbw,cmc",
	"ti,da850-evm",
	"ti,da850",
	NULL,
};

DT_MACHINE_START(DA850_DT, "Generic DA850/OMAP-L138/AM18x")
	.map_io		= da850_init,
	.init_irq	= da8xx_init_irq,
	.init_time	= davinci_timer_init,
	.init_machine	= da850_init_machine,
	.dt_compat	= da850_boards_compat,
	.init_late	= davinci_init_late,
	.restart	= da8xx_restart,
MACHINE_END

#endif
