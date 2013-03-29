/*
 * Low level suspend code for AM33XX SoCs
 *
 * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/
 * Vaibhav Bedia <vaibhav.bedia@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/linkage.h>
#include <linux/ti_emif.h>
#include <linux/platform_data/gpio-omap.h>
#include <asm/memory.h>
#include <asm/assembler.h>

#include "cm33xx.h"
#include "pm33xx.h"
#include "prm33xx.h"
#include "control.h"

	.text
	.align 3

	.macro	pll_bypass, name, clk_mode_addr, idlest_addr, pll_mode
pll_bypass_\name:
	ldr	r0, \clk_mode_addr
	ldr	r1, [r0]
	str	r1, clk_mode_\pll_mode
	bic	r1, r1, #(7 << 0)
	orr	r1, r1, #0x5
	str	r1, [r0]
	ldr	r0, \idlest_addr
wait_pll_bypass_\name:
	ldr	r1, [r0]
	tst	r1, #0x0
	bne	wait_pll_bypass_\name
	.endm

	.macro	pll_lock, name, clk_mode_addr, idlest_addr, pll_mode
pll_lock_\name:
	ldr	r0, \clk_mode_addr
	ldr	r1, clk_mode_\pll_mode
	str	r1, [r0]
	and	r1, r1, #0x7
	cmp	r1, #0x7
	bne	pll_mode_restored_\name
	ldr	r0, \idlest_addr
wait_pll_lock_\name:
	ldr	r1, [r0]
	ands	r1, #0x1
	beq	wait_pll_lock_\name
pll_mode_restored_\name:
	nop
	.endm

	.macro	ddr_self_refresh, num
ddr_self_refresh_\num:
	add	r1, r0, #EMIF_POWER_MANAGEMENT_CONTROL
	ldr	r2, [r1]
	orr	r2, r2, #0xa0		@ a reasonable delay for entering SR
	str	r2, [r1, #0]
	str	r2, [r1, #4]		@ write to shadow register also

	ldr	r2, ddr_start		@ do a dummy access to DDR
	ldr	r3, [r2, #0]
	ldr	r3, [r1, #0]		@ now set the LP MODE to Self-Refresh
	orr	r3, r3, #0x200
	str	r3, [r1, #0]

	mov	r1, #0x1000		@ Give some time for system to enter SR
wait_sr_\num:
	subs	r1, r1, #1
	bne	wait_sr_\num
	.endm

	.macro	wait_sdram_config, num
wait_sdram_config_\num:
	mov	r0, #0x100
wait_sc_\num:
	subs	r0, r0 ,#1
	bne	wait_sc_\num
	.endm

/*
 * This routine is executed from internal RAM and expects the memory type and
 * other DDR configuration values which are different across different memory
 * types to be passed in r0 _strictly_ in following order:
 * 1) memory_type [r0]
 * 2) emif_addr_virt [r1]
 * 4) cpu_rev [r2]
 *
 * The code loads these values taking r0 value as reference to the array in
 * registers starting from r0, i,e memory_type goes to r0 and emif_addr_virt
 * goes to r1 and so on. These are then saved into memory locations before
 * proceeding with the sleep sequence and hence registers r0,r1 can still be
 * used in the rest of the sleep code.
 *
 * mem_type is used to decide different suspend-resume sequences for DDR2
 * and DDR3.
 * cpu_rev is used to update certain steps selectively for PG2.0 in the
 * suspend-resume sequence (like switching to LVCMOS mode is skipped for PG2.0)
 */


ENTRY(am33xx_do_wfi)
	stmfd	sp!, {r4 - r11, lr}	@ save registers on stack

	mov	r8, r0			@ same arg list passed to us

	/* This ensures isb */
	ldr	r0, dcache_flush
	blx	r0

	/* Same as v7_flush_icache_all - saving a branch */
	mov	r0, #0
	mcr	p15, 0, r0, c7, c5, 0	@ I+BTB cache invalidate

	ldm	r8, {r0-r5}		@ gather values passed

	/* Save the values passed */
	str	r0, mem_type
	str	r1, emif_addr_virt
	str	r2, cpu_rev
	str	r3, vtt_toggle_enabled
	str	r4, vtt_gpio_pin
	str	r5, gpio0_bank_virt_addr

	/* VTP Control value is different for DDR2 & DDR3 */
	ldr	r1, susp_vtp_ctrl_ddr2
	cmp	r0, #MEM_TYPE_DDR2
	beq	mem_is_ddr2
	ldr	r1, susp_vtp_ctrl_ddr3
mem_is_ddr2:
	str	r1, susp_vtp_ctrl_val

	ldr	r0, emif_addr_virt
	/* Save EMIF configuration */
	ldr	r1, [r0, #EMIF_SDRAM_CONFIG]
	str	r1, emif_sdcfg_val
	ldr	r1, [r0, #EMIF_SDRAM_REFRESH_CONTROL]
	str	r1, emif_ref_ctrl_val
	ldr	r1, [r0, #EMIF_SDRAM_TIMING_1]
	str	r1, emif_timing1_val
	ldr	r1, [r0, #EMIF_SDRAM_TIMING_2]
	str	r1, emif_timing2_val
	ldr	r1, [r0, #EMIF_SDRAM_TIMING_3]
	str	r1, emif_timing3_val
	ldr	r1, [r0, #EMIF_POWER_MANAGEMENT_CONTROL]
	str	r1, emif_pmcr_val
	ldr	r1, [r0, #EMIF_POWER_MANAGEMENT_CTRL_SHDW]
	str	r1, emif_pmcr_shdw_val
	ldr	r1, [r0, #EMIF_SDRAM_OUTPUT_IMPEDANCE_CALIBRATION_CONFIG]
	str	r1, emif_zqcfg_val
	ldr	r1, [r0, #EMIF_DDR_PHY_CTRL_1]
	str	r1, emif_rd_lat_val

	/* Ensure that all the writes to DDR leave the A8 */
	dsb
	dmb
	isb

	ddr_self_refresh	1

	/* Disable EMIF at this point */
	ldr	r1, virt_emif_clkctrl
	ldr	r2, [r1]
	bic	r2, r2, #0x03
	str	r2, [r1]

	ldr	r1, virt_emif_clkctrl
wait_emif_disable:
	ldr	r2, [r1]
	ldr	r3, module_disabled_val
	cmp	r2, r3
	bne	wait_emif_disable

	/*
	 * For the MPU WFI to be registered as an interrupt
	 * to WKUP_M3, MPU_CLKCTRL.MODULEMODE needs to be set
	 * to DISABLED
	 */
	ldr	r1, virt_mpu_clkctrl
	ldr	r2, [r1]
	bic	r2, r2, #0x03
	str	r2, [r1]

	/* Different sleep sequences for DDR2 and DDR3 */
	ldr	r6, mem_type
	cmp	r6, #MEM_TYPE_DDR2
	beq	no_ddr3_rst_for_ddr2

	/* For DDR3, hold DDR_RESET high via control module */
	ldr	r2, virt_ddr_io_ctrl
	ldr	r1, [r2]
	orr	r1,r1,#DDR3_RST_DEF_VAL		@ set ddr3_rst_def_val
	str	r1, [r2]

	/* mddr mode selection not required for PG2.0 */
	ldr	r6, cpu_rev
	cmp	r6, #CPU_REV_2
	beq	mddr_modesel_skip

	/* Put IO in mDDR (cmos) mode */
	ldr	r2, virt_ddr_io_ctrl
	ldr	r1, [r2]
	orr	r1,r1,#DDR_IO_MDDR_SEL
	str	r1, [r2]
mddr_modesel_skip:
no_ddr3_rst_for_ddr2:
	/* Weak pull down for DQ, DM */
	ldr	r1, virt_ddr_data0_ioctrl
	ldr	r2, susp_io_pull_data
	str	r2, [r1]

	ldr	r1, virt_ddr_data1_ioctrl
	ldr	r2, susp_io_pull_data
	str	r2, [r1]

	/* Different sleep sequences for DDR2 and DDR3 */
	ldr	r6, mem_type
	cmp	r6, #MEM_TYPE_DDR2
	beq	ddr2_no_cmd_operation

	/* Weak pull down for macro CMD0 */
	ldr	r1, virt_ddr_cmd0_ioctrl
	ldr	r2, susp_io_pull_cmd1
	str	r2, [r1]

	/* Weak pull down for macro CMD1 */
	ldr	r1, virt_ddr_cmd1_ioctrl
	ldr	r2, susp_io_pull_cmd1
	str	r2, [r1]

	/*
	 * Weak pull down for macro CMD2
	 * exception: keep DDR_RESET pullup
	 */
	ldr	r1, virt_ddr_cmd2_ioctrl
	ldr	r2, susp_io_pull_cmd2
	str	r2, [r1]

ddr2_no_cmd_operation:
	/*
	 * Disable VTT Regulator
	 *
	 * In order to keep the DDR in Self-Refresh, CKE pin needs to be Pulled
	 * down.
	 * The VTT regulator Pull-up is on the same CKE pin. The pull down
	 * enabled on CKE is a weak pull down.  This weak pull-down is not
	 * enough to pull the line below 0.75v.
	 * Because of this, the DDR never properly goes to self refresh and
	 * hence, low power state is not achieved.
	 *
	 * This issue is addressed as below on a few boards
	 * 1. Add an external pull down on CKE pin and
	 * 2. provide a method to disable VTT regulator (via gpio toggling)
	 *
	 * The 1st part of the solution is addressed in h/w. Handle the 2nd sw
	 * part (depending on whether this toggling is requested)
	 */

	ldr	r6, vtt_toggle_enabled
	cmp	r6, #1
	bne	no_vtt_toggle

	/* Drive GPIO0 VTT Pin LOW */
	mov	r2, #1
	ldr	r1, vtt_gpio_pin
	mov	r2, r2, lsl r1
	ldr	r0, gpio0_bank_virt_addr
	str	r2, [r0, #OMAP4_GPIO_CLEARDATAOUT]

	/* Disable GPIO0 for EVM_SK here */
	ldr	r1, gpio0_clkctrl_virt_addr
	ldr	r2, [r1]
	bic	r2, r2, #(3 << 0)
	str	r2, [r1]
	ldr	r3, module_disabled_val
wait_gpio0_disable:
	ldr	r2, [r1]
	cmp	r2, r3
	bne	wait_gpio0_disable

no_vtt_toggle:
	/* Disable VTP */
	ldr	r1, virt_ddr_vtp_ctrl
	ldr	r2, susp_vtp_ctrl_val
	str	r2, [r1]

	/* Enable SRAM LDO ret mode */
	ldr	r0, virt_sram_ldo_addr
	ldr	r1, [r0]
	orr	r1, #1
	str	r1, [r0]

	/* Put the PLLs in bypass mode */
	pll_bypass	core, virt_core_clk_mode, virt_core_idlest, core_val
	pll_bypass	ddr, virt_ddr_clk_mode, virt_ddr_idlest, ddr_val
	pll_bypass	disp, virt_disp_clk_mode, virt_disp_idlest, disp_val
	pll_bypass	per, virt_per_clk_mode, virt_per_idlest, per_val
	pll_bypass	mpu, virt_mpu_clk_mode, virt_mpu_idlest, mpu_val

	dsb
	dmb
	isb

	wfi

	/* NOPs to ensure the A8 pipeline is clean */
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop

	/* We come here in case of an abort due to a late interrupt */

	/* Set MPU_CLKCTRL.MODULEMODE back to ENABLE */
	ldr	r1, virt_mpu_clkctrl
	mov	r2, #0x02
	str	r2, [r1]

	/* Relock the PLLs */
	pll_lock	mpu_abt, virt_mpu_clk_mode, virt_mpu_idlest, mpu_val
	pll_lock	per_abt, virt_per_clk_mode, virt_per_idlest, per_val
	pll_lock	disp_abt, virt_disp_clk_mode, virt_disp_idlest, disp_val
	pll_lock	ddr_abt, virt_ddr_clk_mode, virt_ddr_idlest, ddr_val
	pll_lock	core_abt, virt_core_clk_mode, virt_core_idlest, core_val

	/* Disable SRAM LDO ret mode */
	ldr	r0, virt_sram_ldo_addr
	ldr	r1, [r0]
	bic	r1, #1
	str	r1, [r0]

	ldr	r6, mem_type
	cmp	r6, #MEM_TYPE_DDR2
	beq	ddr2_no_mddr_change1

	/* mddr mode selection not required for PG2.0 */
	ldr     r6, cpu_rev
	cmp     r6, #CPU_REV_2
	beq     mddr_modesel_skip1

	/* Take out IO of mDDR mode */
	ldr	r0, virt_ddr_io_ctrl
	ldr	r1, [r0]
	bic	r1, r1, #DDR_IO_MDDR_SEL
	str	r1, [r0]

mddr_modesel_skip1:
	/* Disable the pull for CMD2 */
	ldr	r1, virt_ddr_cmd2_ioctrl
	ldr	r2, resume_io_pull_cmd
	str	r2, [r1]
	/* Disable the pull for CMD1 */
	ldr	r1, virt_ddr_cmd1_ioctrl
	ldr	r2, resume_io_pull_cmd
	str	r2, [r1]
	/* Disable the pull for CMD0 */
	ldr	r1, virt_ddr_cmd0_ioctrl
	ldr	r2, resume_io_pull_cmd
	str	r2, [r1]

ddr2_no_mddr_change1:
	/* Disable the pull for DATA1 */
	ldr	r1, virt_ddr_data1_ioctrl
	ldr	r2, resume_io_pull_data
	str	r2, [r1]
	/* Disable the pull for DATA0 */
	ldr	r1, virt_ddr_data0_ioctrl
	ldr	r2, resume_io_pull_data
	str	r2, [r1]

	/* Enable EMIF */
	ldr	r1, virt_emif_clkctrl
	mov	r2, #0x02
	str	r2, [r1]
wait_emif_enable:
	ldr	r3, [r1]
	cmp	r2, r3
	bne	wait_emif_enable

	/* Enable VTP */
	ldr	r0, virt_ddr_vtp_ctrl
	ldr	r1, [r0]
	mov	r2, #0x0	@ clear the register
	str	r2, [r0]
	mov	r2, #0x6	@ write the filter value
	str	r2, [r0]

	ldr	r1, [r0]
	ldr	r2, vtp_enable	@ set the enable bit
	orr	r2, r2, r1
	str	r2, [r0]

	ldr	r1, [r0]	@ toggle the CLRZ bit
	bic	r1, #1
	str	r1, [r0]

	ldr	r1, [r0]
	orr	r1, #1
	str	r1, [r0]

poll_vtp_ready_abt:
	ldr	r1, [r0]	@ poll for VTP ready
	tst	r1, #(AM33XX_VTP_CTRL_READY)
	beq	poll_vtp_ready_abt

	wait_sdram_config	1

	/* Enable VTT_Regulator */
	ldr	r6, vtt_toggle_enabled
	cmp	r6, #1
	bne	no_vtt_toggle1

	/* Enable GPIO0 VTT Toggle */
	ldr	r1, gpio0_clkctrl_virt_addr
	mov	r2, #0x2
	str	r2, [r1]
wait_gpio0_enable1:
	ldr	r3, [r1]
	cmp	r2, r3
	bne	wait_gpio0_enable1

	/* Drive GPIO0 VTT pin HIGH */
	mov	r2, #1
	ldr	r1, vtt_gpio_pin
	mov	r2, r2, lsl r1
	ldr	r0, gpio0_bank_virt_addr
	str	r2, [r0, #OMAP4_GPIO_SETDATAOUT]

no_vtt_toggle1:
	/* Disable EMIF self-refresh */
	ldr	r0, emif_addr_virt
	add	r0, r0, #EMIF_POWER_MANAGEMENT_CONTROL
	ldr	r1, [r0]
	bic	r1, r1, #LP_MODE_MASK
	str	r1, [r0]
	str	r1, [r0, #4]

	ldr	r6, mem_type
	cmp	r6, #MEM_TYPE_DDR2
	beq	ddr2_no_ddr3_reset1

	/* For DDR3, make DDR_RESET low via control module */
	ldr	r0, virt_ddr_io_ctrl
	ldr	r1, [r0]
	bic	r1, r1, #DDR3_RST_DEF_VAL
	str	r1, [r0]

	b	return_to_ddr_abt

ddr2_no_ddr3_reset1:
	/*
	 * A write to SDRAM CONFIG register triggers
	 * an init sequence and hence it must be done
	 * at the end
	 */
	ldr r0, emif_addr_virt
	add r0, r0, #EMIF_SDRAM_CONFIG
	ldr r4, emif_sdcfg_val
	str r4, [r0]

return_to_ddr_abt:
	/* Kill some time for sanity to settle in */
	mov r0, #0x1000
wait_abt:
	subs   r0, r0, #1
	bne wait_abt

	/* Let the suspend code know about the abort */
	mov	r0, #1
	ldmfd	sp!, {r4 - r11, pc}	@ restore regs and return
ENDPROC(am33xx_do_wfi)

	.align
ENTRY(am33xx_resume_offset)
	.word . - am33xx_do_wfi

ENTRY(am33xx_resume_from_deep_sleep)
	/* Take the PLLs out of LP_BYPASS */
	pll_lock	mpu, phys_mpu_clk_mode, phys_mpu_idlest, mpu_val
	pll_lock	per, phys_per_clk_mode, phys_per_idlest, per_val
	pll_lock	disp, phys_disp_clk_mode, phys_disp_idlest, disp_val
	pll_lock	ddr, phys_ddr_clk_mode, phys_ddr_idlest, ddr_val
	pll_lock	core, phys_core_clk_mode, phys_core_idlest, core_val

	/* Disable SRAM LDO ret mode */
	ldr	r0, phys_sram_ldo_addr
	ldr	r1, [r0]
	bic	r1, #1
	str	r1, [r0]

	ldr	r6, mem_type
	cmp	r6, #MEM_TYPE_DDR2
	beq	ddr2_no_mddr_change2

	/* mddr mode selection not required for PG2.0 */
	ldr     r6, cpu_rev
	cmp     r6, #CPU_REV_2
	beq     mddr_modesel_skip2

	/* Take out IO of mDDR mode */
	ldr	r0, phys_ddr_io_ctrl
	ldr	r1, [r0]
	bic	r1, r1, #DDR_IO_MDDR_SEL
	str	r1, [r0]

mddr_modesel_skip2:
	/* Disable the pull for CMD2 */
	ldr	r1, phys_ddr_cmd2_ioctrl
	ldr	r2, resume_io_pull_cmd
	str	r2, [r1]
	/* Disable the pull for CMD1 */
	ldr	r1, phys_ddr_cmd1_ioctrl
	ldr	r2, resume_io_pull_cmd
	str	r2, [r1]
	/* Disable the pull for CMD0 */
	ldr	r1, phys_ddr_cmd0_ioctrl
	ldr	r2, resume_io_pull_cmd
	str	r2, [r1]

ddr2_no_mddr_change2:
	/* Disable the pull for DATA1 */
	ldr	r1, phys_ddr_data1_ioctrl
	ldr	r2, resume_io_pull_data
	str	r2, [r1]
	/* Disable the pull for DATA0 */
	ldr	r1, phys_ddr_data0_ioctrl
	ldr	r2, resume_io_pull_data
	str	r2, [r1]

	/* Config VTP */
	ldr	r0, phys_ddr_vtp_ctrl
	ldr	r1, [r0]
	mov	r2, #0x0	@ clear the register
	str	r2, [r0]
	mov	r2, #0x6	@ write the filter value
	str	r2, [r0]

	ldr	r1, [r0]
	ldr	r2, vtp_enable	@ set the enable bit
	orr	r2, r2, r1
	str	r2, [r0]

	ldr	r1, [r0]	@ toggle the CLRZ bit
	bic	r1, #1
	str	r1, [r0]

	ldr	r1, [r0]
	orr	r1, #1
	str	r1, [r0]

poll_vtp_ready:
	ldr	r1, [r0]	@ poll for VTP ready
	tst	r1, #AM33XX_VTP_CTRL_READY
	beq	poll_vtp_ready

	/* Enable EMIF */
	ldr	r1, phys_emif_clkctrl
	mov	r2, #0x02
	str	r2, [r1]
wait_emif_enable1:
	ldr	r3, [r1]
	cmp	r2, r3
	bne	wait_emif_enable1

	wait_sdram_config	2

	/* Enable VTT_Regulator */
	ldr	r6, vtt_toggle_enabled
	cmp	r6, #1
	bne	no_vtt_toggle2

	/* Enable GPIO0 for EVM-SK here */
	ldr	r1, gpio0_clkctrl_phy_addr
	mov	r2, #0x2
	str	r2, [r1]
wait_gpio0_phys_enable2:
	ldr	r3, [r1]
	cmp	r2, r3
	bne	wait_gpio0_phys_enable2

	/* Drive GPIO0 VTT Pin HIGH */
	mov	r2, #1
	ldr	r1, vtt_gpio_pin
	mov	r2, r2, lsl r1
	ldr	r0, gpio0_bank_phy_addr
	str	r2, [r0, #OMAP4_GPIO_SETDATAOUT]

no_vtt_toggle2:
	/* Config EMIF Timings */
	ldr	r3, emif_phys_addr
	ldr	r4, emif_rd_lat_val
	str	r4, [r3, #EMIF_DDR_PHY_CTRL_1]
	str	r4, [r3, #EMIF_DDR_PHY_CTRL_1_SHDW]
	ldr	r4, emif_timing1_val
	str	r4, [r3, #EMIF_SDRAM_TIMING_1]
	str	r4, [r3, #EMIF_SDRAM_TIMING_1_SHDW]
	ldr	r4, emif_timing2_val
	str	r4, [r3, #EMIF_SDRAM_TIMING_2]
	str	r4, [r3, #EMIF_SDRAM_TIMING_2_SHDW]
	ldr	r4, emif_timing3_val
	str	r4, [r3, #EMIF_SDRAM_TIMING_3]
	str	r4, [r3, #EMIF_SDRAM_TIMING_3_SHDW]
	ldr	r4, emif_ref_ctrl_val
	str	r4, [r3, #EMIF_SDRAM_REFRESH_CONTROL]
	str	r4, [r3, #EMIF_SDRAM_REFRESH_CTRL_SHDW]
	ldr	r4, emif_pmcr_val
	str	r4, [r3, #EMIF_POWER_MANAGEMENT_CONTROL]
	ldr	r4, emif_pmcr_shdw_val
	str	r4, [r3, #EMIF_POWER_MANAGEMENT_CTRL_SHDW]

	/*
	 * Output impedence calib needed only for DDR3
	 * but since the initial state of this will be
	 * disabled for DDR2 no harm in restoring the
	 * old configuration
	 */
	ldr	r4, emif_zqcfg_val
	str	r4, [r3, #EMIF_SDRAM_OUTPUT_IMPEDANCE_CALIBRATION_CONFIG]

	/* Write to SDRAM_CONFIG only for DDR2 */
	ldr	r6, mem_type
	cmp	r6, #MEM_TYPE_DDR2
	beq	ddr2_sdcfg

	/* For DDR3, make DDR_RESET low via control module */
	ldr	r0, phys_ddr_io_ctrl
	ldr	r1, [r0]
	bic	r1, r1, #DDR3_RST_DEF_VAL
	str	r1, [r0]

	b	return_to_ddr

	/*
	 * A write to SDRAM CONFIG register triggers
	 * an init sequence and hence it must be done
	 * at the end for DDR2
	 */
ddr2_sdcfg:
	ldr	r4, emif_sdcfg_val
	str	r4, [r3, #EMIF_SDRAM_CONFIG]

return_to_ddr:
	/* Back from la-la-land. Kill some time for sanity to settle in */
	mov	r0, #0x1000
wait_resume:
	subs	r0, r0, #1
	bne	wait_resume

	/* We are back. Branch to the common CPU resume routine */
	mov	r0, #0
	ldr	pc, resume_addr
ENDPROC(am33xx_resume_from_deep_sleep)


/*
 * Local variables
 */
	.align
resume_addr:
	.word	cpu_resume - PAGE_OFFSET + 0x80000000
dcache_flush:
	.word   v7_flush_dcache_all
ddr_start:
	.word	PAGE_OFFSET
emif_phys_addr:
	.word	AM33XX_EMIF_BASE
gpio0_bank_virt_addr:
	.word	0xDEADBEEF
gpio0_bank_phy_addr:
	.word	AM33XX_GPIO0_BASE
virt_mpu_idlest:
	.word	AM33XX_CM_IDLEST_DPLL_MPU
virt_mpu_clk_mode:
	.word	AM33XX_CM_CLKMODE_DPLL_MPU
phys_mpu_clk_mode:
	.word	(AM33XX_CM_BASE + AM33XX_CM_WKUP_MOD + \
		AM33XX_CM_CLKMODE_DPLL_MPU_OFFSET)
phys_mpu_idlest:
	.word	(AM33XX_CM_BASE + AM33XX_CM_WKUP_MOD + \
		AM33XX_CM_IDLEST_DPLL_MPU_OFFSET)
virt_core_idlest:
	.word	AM33XX_CM_IDLEST_DPLL_CORE
virt_core_clk_mode:
	.word	AM33XX_CM_CLKMODE_DPLL_CORE
phys_core_clk_mode:
	.word	(AM33XX_CM_BASE + AM33XX_CM_WKUP_MOD + \
		AM33XX_CM_CLKMODE_DPLL_CORE_OFFSET)
phys_core_idlest:
	.word	(AM33XX_CM_BASE + AM33XX_CM_WKUP_MOD + \
		AM33XX_CM_IDLEST_DPLL_CORE_OFFSET)
virt_per_idlest:
	.word	AM33XX_CM_IDLEST_DPLL_PER
virt_per_clk_mode:
	.word	AM33XX_CM_CLKMODE_DPLL_PER
phys_per_clk_mode:
	.word	(AM33XX_CM_BASE + AM33XX_CM_WKUP_MOD + \
		AM33XX_CM_CLKMODE_DPLL_PER_OFFSET)
phys_per_idlest:
	.word	(AM33XX_CM_BASE + AM33XX_CM_WKUP_MOD + \
		AM33XX_CM_IDLEST_DPLL_PER_OFFSET)
virt_disp_idlest:
	.word	AM33XX_CM_IDLEST_DPLL_DISP
virt_disp_clk_mode:
	.word	AM33XX_CM_CLKMODE_DPLL_DISP
phys_disp_clk_mode:
	.word	(AM33XX_CM_BASE + AM33XX_CM_WKUP_MOD + \
		AM33XX_CM_CLKMODE_DPLL_DISP_OFFSET)
phys_disp_idlest:
	.word	(AM33XX_CM_BASE + AM33XX_CM_WKUP_MOD + \
		AM33XX_CM_IDLEST_DPLL_DISP_OFFSET)
virt_ddr_idlest:
	.word	AM33XX_CM_IDLEST_DPLL_DDR
virt_ddr_clk_mode:
	.word	AM33XX_CM_CLKMODE_DPLL_DDR
phys_ddr_clk_mode:
	.word	(AM33XX_CM_BASE + AM33XX_CM_WKUP_MOD + \
		AM33XX_CM_CLKMODE_DPLL_DDR_OFFSET)
phys_ddr_idlest:
	.word	(AM33XX_CM_BASE + AM33XX_CM_WKUP_MOD + \
		AM33XX_CM_IDLEST_DPLL_DDR_OFFSET)
virt_sram_ldo_addr:
	.word	AM33XX_PRM_LDO_SRAM_MPU_CTRL
phys_sram_ldo_addr:
	.word	(AM33XX_PRM_BASE + AM33XX_PRM_DEVICE_MOD + \
		AM33XX_PRM_LDO_SRAM_MPU_CTRL_OFFSET)
virt_mpu_clkctrl:
	.word	AM33XX_CM_MPU_MPU_CLKCTRL
virt_emif_clkctrl:
	.word	AM33XX_CM_PER_EMIF_CLKCTRL
phys_emif_clkctrl:
	.word	(AM33XX_CM_BASE + AM33XX_CM_PER_MOD + \
		AM33XX_CM_PER_EMIF_CLKCTRL_OFFSET)
gpio0_clkctrl_virt_addr:
	.word   AM33XX_CM_WKUP_GPIO0_CLKCTRL
gpio0_clkctrl_phy_addr:
	.word   (AM33XX_CM_BASE + AM33XX_CM_WKUP_MOD + \
		AM33XX_CM_WKUP_GPIO0_CLKCTRL_OFFSET)
module_clk_mask:
	.word	0x03
module_clk_enable:
	.word	0x02
module_disabled_val:
	.word	0x30000
mem_type:
	.word	0xDEADBEEF
cpu_rev:
	.word	0xDEADBEEF
vtt_toggle_enabled:
	.word   0xDEADBEEF
vtt_gpio_pin:
	.word   0xDEADBEEF

/* DDR related defines */
virt_ddr_io_ctrl:
	.word	AM33XX_CTRL_REGADDR(AM33XX_DDR_IO_CTRL)
phys_ddr_io_ctrl:
	.word	AM33XX_CTRL_BASE + AM33XX_DDR_IO_CTRL
virt_ddr_vtp_ctrl:
	.word	AM33XX_CTRL_REGADDR(AM33XX_VTP0_CTRL_REG)
phys_ddr_vtp_ctrl:
	.word	AM33XX_CTRL_BASE + AM33XX_VTP0_CTRL_REG
virt_ddr_cmd0_ioctrl:
	.word	AM33XX_CTRL_REGADDR(AM33XX_DDR_CMD0_IOCTRL)
phys_ddr_cmd0_ioctrl:
	.word	AM33XX_CTRL_BASE + AM33XX_DDR_CMD0_IOCTRL
virt_ddr_cmd1_ioctrl:
	.word	AM33XX_CTRL_REGADDR(AM33XX_DDR_CMD1_IOCTRL)
phys_ddr_cmd1_ioctrl:
	.word	AM33XX_CTRL_BASE + AM33XX_DDR_CMD1_IOCTRL
virt_ddr_cmd2_ioctrl:
	.word	AM33XX_CTRL_REGADDR(AM33XX_DDR_CMD2_IOCTRL)
phys_ddr_cmd2_ioctrl:
	.word	AM33XX_CTRL_BASE + AM33XX_DDR_CMD2_IOCTRL
virt_ddr_data0_ioctrl:
	.word	AM33XX_CTRL_REGADDR(AM33XX_DDR_DATA0_IOCTRL)
phys_ddr_data0_ioctrl:
	.word	AM33XX_CTRL_BASE + AM33XX_DDR_DATA0_IOCTRL
virt_ddr_data1_ioctrl:
	.word	AM33XX_CTRL_REGADDR(AM33XX_DDR_DATA1_IOCTRL)
phys_ddr_data1_ioctrl:
	.word	AM33XX_CTRL_BASE + AM33XX_DDR_DATA1_IOCTRL
vtp_enable:
	.word	AM33XX_VTP_CTRL_ENABLE
mddr_sel:
	.word	DDR_IO_MDDR_SEL
ddr3_rst_def_val:
	.word	DDR3_RST_DEF_VAL

/*
 * Values recommended by the HW team. These change the pulls
 * on certain IOs of DATA and CMD macros
 */
susp_io_pull_data:
	.word	0x3FF00003
susp_io_pull_cmd1:
	.word   0xFFE0018B
susp_io_pull_cmd2:
	.word   0xFFA0098B
resume_io_pull_data:
	.word	0x18B
resume_io_pull_cmd:
	.word	0x18B
susp_vtp_ctrl_val:
	.word	0xDEADBEEF
susp_vtp_ctrl_ddr2:
	.word	SUSP_VTP_CTRL_DDR2
susp_vtp_ctrl_ddr3:
	.word	SUSP_VTP_CTRL_DDR3

/* Placeholder for storing EMIF configuration */
emif_addr_virt:
	.word	0xDEADBEEF
emif_rd_lat_val:
	.word	0xDEADBEEF
emif_timing1_val:
	.word	0xDEADBEEF
emif_timing2_val:
	.word	0xDEADBEEF
emif_timing3_val:
	.word	0xDEADBEEF
emif_sdcfg_val:
	.word	0xDEADBEEF
emif_ref_ctrl_val:
	.word	0xDEADBEEF
emif_zqcfg_val:
	.word	0xDEADBEEF
emif_pmcr_val:
	.word	0xDEADBEEF
emif_pmcr_shdw_val:
	.word	0xDEADBEEF

/* Placeholder for storing PLL mode */
clk_mode_mpu_val:
	.word	0xDEADBEEF
clk_mode_per_val:
	.word	0xDEADBEEF
clk_mode_disp_val:
	.word	0xDEADBEEF
clk_mode_ddr_val:
	.word	0xDEADBEEF
clk_mode_core_val:
	.word	0xDEADBEEF

	.align 3
ENTRY(am33xx_do_wfi_sz)
	.word	. - am33xx_do_wfi

