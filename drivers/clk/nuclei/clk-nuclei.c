// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/*
 * Copyright (C) 2026 Nucleisys.
 *
 */

// #define LOG_DEBUG
#include "clk-nuclei.h"
#include <clk-uclass.h>
#include <clk.h>
#include <common.h>
#include <dm.h>
#include <dm/device_compat.h>
#include <dt-bindings/clock/nuclei-clock.h>
#include <errno.h>
#include <linux/bitops.h>
#include <linux/math64.h>

static struct nuclei_gated_div_info clk_gate_divs[] = {
	/* UART */
	{"usart0_clk_i", "sys_clk", CLK_CTRL41_USART0_CLK_OFS, SUBM_CLK_CTRL0_OFS,
	 GATE_USART0, 100000000, CLK_USART0},
	{"usart0_intf_clk_i", "sys_intf_clk", CLK_CTRL42_USART0_INTF_CLK_I_OFS,
	 SUBM_CLK_CTRL0_OFS, GATE_USART0_INTF, 100000000, CLK_USART0_INTF},

	/* SDIO */
	{"sdio0_clk_i", "sys_clk", CLK_CTRL136_SDIO0_CLK_OFS, SUBM_CLK_CTRL2_OFS,
	 GATE_SDIO0, 200000000, CLK_SDIO0},
	{"sdio0_intf_clk_i", "sys_intf_clk", CLK_CTRL137_SDIO0_INTF_CLK_I_OFS,
	 SUBM_CLK_CTRL2_OFS, GATE_SDIO0_INTF, 200000000, CLK_SDIO0_INTF},
	{"hs_sdio0_clk_i", "sys_clk", CLK_CTRL138_HS_SDIO0_CLK_OFS,
	 SUBM_CLK_CTRL2_OFS, GATE_HS_SDIO0, 200000000, CLK_HS_SDIO0},
	{"hs_sdio0_intf_clk_i", "sys_intf_clk", CLK_CTRL139_HS_SDIO0_INTF_CLK_I_OFS,
	 SUBM_CLK_CTRL2_OFS, GATE_HS_SDIO0_INTF, 200000000, CLK_HS_SDIO0_INTF},

	/* XEC */
	{"xec_gen20_sys_clk_i", "mux_xec_sys", CLK_CTRL126_XEC_GEN20_SYS_CLK_OFS,
	 SUBM_CLK_CTRL2_OFS, GATE_XEC, 25000000, CLK_XEC_GEN20_SYS},
	{"xec_gen21_sys_clk_i", "mux_xec_sys", CLK_CTRL127_XEC_GEN21_SYS_CLK_OFS,
	 SUBM_CLK_CTRL2_OFS, GATE_XEC, 125000000, CLK_XEC_GEN21_SYS},
	{"rmii_clk_ref_i", "mux_xec_rmii", CLK_CTRL128_RMII_CLK_REF_OFS,
	 SUBM_CLK_CTRL2_OFS, GATE_XEC, 50000000, CLK_RMII_CLK_REF},
	{"ptp_ref_clk_i", "mux_xec_sys", CLK_CTRL129_PTP_REF_CLK_OFS,
	 SUBM_CLK_CTRL2_OFS, GATE_XEC, 125000000, CLK_PTP_REF},

	/* I2C */
	{"i2c0_clk_i", "sys_clk", CLK_CTRL68_I2C0_CLK_OFS, SUBM_CLK_CTRL0_OFS,
	 GATE_I2C0, 100000000, CLK_I2C0},
	{"i2c0_intf_clk_i", "sys_intf_clk", CLK_CTRL69_I2C0_INTF_CLK_I_OFS,
	 SUBM_CLK_CTRL0_OFS, GATE_I2C0_INTF, 100000000, CLK_I2C0_INTF},

};

static ulong nuclei_mux_get_rate(struct nuclei_clk_priv* priv, u32 mux_reg,
				 u32 shift, u32 width, u32* parent_idx)
{
	u32 val = nuclei_clk_readl(priv, mux_reg);
	u32 idx = (val >> shift) & ((1 << width) - 1);
	ulong rate = 0;

	if (parent_idx)
		*parent_idx = idx;

	switch (mux_reg) {
	case CLK_CTRL0_SYS_PLL_CLK_S_OFS:
	case CLK_CTRL2_CORE1_PLL_CLK_S_OFS:
	case CLK_CTRL3_XUC_PLL_CLK_S_OFS:
		switch (idx) {
		case 0:
			rate = clk_get_rate(priv->hsi);
			break;
		case 1:
			rate = clk_get_rate(priv->hse);
			break;
		case 2:
			rate = clk_get_rate(priv->clk_in1);
			break;
		case 3:
			rate = clk_get_rate(priv->clk_in2);
			break;
		default:
			break;
		}
		break;
	case CLK_CTRL4_SYS_CLK_OFS:
	case CLK_CTRL5_SYS_INTF_CLK_OFS:
		switch (idx) {
		case 0:
			rate = clk_get_rate(priv->hsi);
			break;
		case 1:
			rate = clk_get_rate(priv->hse);
			break;
		case 2:
			rate = priv->sys_pll_rate;
			break;
		case 3:
			rate = priv->sys_pll_div2_rate;
			break;
		case 4:
			rate = priv->core1_pll_rate;
			break;
		case 5:
			rate = priv->xuc_pll_rate;
			break;
		default:
			break;
		}
		break;
	case CLK_CTRL16_XEC_SYS_CLK_S_OFS:
	case CLK_CTRL17_XEC_RMII_CLK_S_OFS:
		switch (idx) {
		case 0:
			rate = clk_get_rate(priv->hsi);
			break;
		case 1:
			rate = priv->sys_pll_rate;
			break;
		case 2:
			rate = priv->sys_pll_div2_rate;
			break;
		case 3:
			rate = priv->core1_pll_rate;
			break;
		case 4:
			rate = priv->xuc_pll_rate;
			break;
		case 5:
			rate = clk_get_rate(priv->clk_in1);
			break;
		case 6:
			rate = clk_get_rate(priv->clk_in2);
			break;
		default:
			break;
		}
		break;
	}

	return rate;
}

static ulong nuclei_pll_calc_rate(struct nuclei_clk_priv* priv, u32 pll_reg,
				  ulong parent_rate)
{
	u32 val = nuclei_clk_readl(priv, pll_reg);

	if (val & BIT(PLL_BP_SHIFT))
		return parent_rate;

	unsigned int n = (val >> PLL_N_SHIFT) & PLL_N_MASK;
	unsigned int m = (val >> PLL_M_SHIFT) & PLL_M_MASK;
	unsigned int od = (val >> PLL_OD_SHIFT) & PLL_OD_MASK;

	if (n == 0 || m == 0)
		return 0;

	return div_u64((u64)parent_rate * m, n << od);
}

static unsigned int nuclei_get_div_value(struct nuclei_clk_priv* priv,
					 unsigned int div_reg)
{
	if (!div_reg)
		return 1;
	u32 val = nuclei_clk_readl(priv, div_reg);
	return (val & 0xFF) + 1;
}

static ulong nuclei_clk_get_rate(struct clk* clk)
{
	struct nuclei_clk_priv* priv = dev_get_priv(clk->dev);
	ulong parent_rate;
	unsigned int div_val;

	switch (clk->id) {
	case CLK_CORE1_PLL:
		return priv->core1_pll_rate;

	case CLK_XUC_PLL:
		return priv->xuc_pll_rate;

	case CLK_USART0:
		parent_rate = priv->sys_clk_rate;
		div_val = nuclei_get_div_value(priv, CLK_CTRL41_USART0_CLK_OFS);
		return parent_rate / div_val;

	case CLK_USART0_INTF:
		parent_rate = priv->sys_intf_clk_rate;
		div_val = nuclei_get_div_value(
		    priv, CLK_CTRL42_USART0_INTF_CLK_I_OFS);
		return parent_rate / div_val;

	case CLK_SDIO0:
		parent_rate = priv->sys_clk_rate;
		div_val = nuclei_get_div_value(priv, CLK_CTRL136_SDIO0_CLK_OFS);
		return parent_rate / div_val;

	case CLK_SDIO0_INTF:
		parent_rate = priv->sys_intf_clk_rate;
		div_val = nuclei_get_div_value(
		    priv, CLK_CTRL137_SDIO0_INTF_CLK_I_OFS);
		return parent_rate / div_val;

	case CLK_HS_SDIO0:
		parent_rate = priv->sys_clk_rate;
		div_val =
		    nuclei_get_div_value(priv, CLK_CTRL138_HS_SDIO0_CLK_OFS);
		return parent_rate / div_val;

	case CLK_HS_SDIO0_INTF:
		parent_rate = priv->sys_intf_clk_rate;
		div_val = nuclei_get_div_value(
		    priv, CLK_CTRL139_HS_SDIO0_INTF_CLK_I_OFS);
		return parent_rate / div_val;

	case CLK_XEC_GEN20_SYS:
	case CLK_XEC_GEN21_SYS:
	case CLK_PTP_REF:
		parent_rate = priv->xec_sys_rate;
		div_val = nuclei_get_div_value(
		    priv, (clk->id == CLK_XEC_GEN20_SYS)
			      ? CLK_CTRL126_XEC_GEN20_SYS_CLK_OFS
			  : (clk->id == CLK_XEC_GEN21_SYS)
			      ? CLK_CTRL127_XEC_GEN21_SYS_CLK_OFS
			      : CLK_CTRL129_PTP_REF_CLK_OFS);
		return parent_rate / div_val;

	case CLK_RMII_CLK_REF:
		parent_rate = priv->xec_rmii_rate;
		div_val =
		    nuclei_get_div_value(priv, CLK_CTRL128_RMII_CLK_REF_OFS);
		return parent_rate / div_val;

	case CLK_I2C0:
		parent_rate = priv->sys_clk_rate;
		div_val = nuclei_get_div_value(priv, CLK_CTRL68_I2C0_CLK_OFS);
		return parent_rate / div_val;

	case CLK_I2C0_INTF:
		parent_rate = priv->sys_intf_clk_rate;
		div_val =
		    nuclei_get_div_value(priv, CLK_CTRL69_I2C0_INTF_CLK_I_OFS);
		return parent_rate / div_val;

	default:
		return -ENOENT;
	}
}

static ulong nuclei_clk_set_rate(struct clk* clk, ulong rate)
{
	struct nuclei_clk_priv* priv = dev_get_priv(clk->dev);
	unsigned int div_reg;
	ulong parent_rate;
	unsigned int div_val;
	u32 val;

	switch (clk->id) {
	case CLK_USART0:
		parent_rate = priv->sys_clk_rate;
		div_reg = CLK_CTRL41_USART0_CLK_OFS;
		break;
	case CLK_USART0_INTF:
		parent_rate = priv->sys_intf_clk_rate;
		div_reg = CLK_CTRL42_USART0_INTF_CLK_I_OFS;
		break;
	case CLK_SDIO0:
		parent_rate = priv->sys_clk_rate;
		div_reg = CLK_CTRL136_SDIO0_CLK_OFS;
		break;
	case CLK_SDIO0_INTF:
		parent_rate = priv->sys_intf_clk_rate;
		div_reg = CLK_CTRL137_SDIO0_INTF_CLK_I_OFS;
		break;
	case CLK_HS_SDIO0:
		parent_rate = priv->sys_clk_rate;
		div_reg = CLK_CTRL138_HS_SDIO0_CLK_OFS;
		break;
	case CLK_HS_SDIO0_INTF:
		parent_rate = priv->sys_intf_clk_rate;
		div_reg = CLK_CTRL139_HS_SDIO0_INTF_CLK_I_OFS;
		break;
	case CLK_XEC_GEN20_SYS:
		parent_rate = priv->xec_sys_rate;
		div_reg = CLK_CTRL126_XEC_GEN20_SYS_CLK_OFS;
		break;
	case CLK_XEC_GEN21_SYS:
		parent_rate = priv->xec_sys_rate;
		div_reg = CLK_CTRL127_XEC_GEN21_SYS_CLK_OFS;
		break;
	case CLK_RMII_CLK_REF:
		parent_rate = priv->xec_rmii_rate;
		div_reg = CLK_CTRL128_RMII_CLK_REF_OFS;
		break;
	case CLK_PTP_REF:
		parent_rate = priv->xec_rmii_rate;
		div_reg = CLK_CTRL129_PTP_REF_CLK_OFS;
		break;
	case CLK_I2C0:
		parent_rate = priv->sys_clk_rate;
		div_reg = CLK_CTRL68_I2C0_CLK_OFS;
		break;
	case CLK_I2C0_INTF:
		parent_rate = priv->sys_intf_clk_rate;
		div_reg = CLK_CTRL69_I2C0_INTF_CLK_I_OFS;
		break;
	default:
		return -ENOENT;
	}

	if (!parent_rate)
		return -EINVAL;

	div_val = DIV_ROUND_UP(parent_rate, rate);
	if (div_val < 1)
		div_val = 1;
	if (div_val > DIV_MAX_DIV)
		div_val = DIV_MAX_DIV;

	val = nuclei_clk_readl(priv, div_reg);
	val &= ~0xFF;
	val |= (div_val - 1) & 0xFF;
	nuclei_clk_writel(priv, div_reg, val);

	return parent_rate / div_val;
}

static int nuclei_clk_enable(struct clk* clk)
{
	struct nuclei_clk_priv* priv = dev_get_priv(clk->dev);
	const struct nuclei_gated_div_info* info;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(clk_gate_divs); i++) {
		if (clk_gate_divs[i].id == clk->id) {
			info = &clk_gate_divs[i];
			if (info->gate_reg) {
				nuclei_clk_update_bits(priv, info->gate_reg,
						       BIT(info->gate_bit),
						       BIT(info->gate_bit));
			}
			return 0;
		}
	}

	return 0;
}

static int nuclei_clk_disable(struct clk* clk)
{
	struct nuclei_clk_priv* priv = dev_get_priv(clk->dev);
	const struct nuclei_gated_div_info* info;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(clk_gate_divs); i++) {
		if (clk_gate_divs[i].id == clk->id) {
			info = &clk_gate_divs[i];
			if (info->gate_reg) {
				nuclei_clk_update_bits(priv, info->gate_reg,
						       BIT(info->gate_bit), 0);
			}
			return 0;
		}
	}

	return 0;
}

static const struct clk_ops nuclei_clk_ops = {
	.enable = nuclei_clk_enable,
	.disable = nuclei_clk_disable,
	.get_rate = nuclei_clk_get_rate,
	.set_rate = nuclei_clk_set_rate,
};

static int nuclei_clk_probe(struct udevice* dev)
{
	struct nuclei_clk_priv* priv = dev_get_priv(dev);
	ulong rate;
	u32 div;

	priv->dev = dev;
	priv->base = dev_remap_addr(dev);
	if (!priv->base)
		return -ENOMEM;

	// Get fix clock
	priv->hsi = devm_clk_get(dev, "hsi");
	if (IS_ERR(priv->hsi)) {
		dev_warn(dev, "Failed to get hsi\n");
		priv->hsi = NULL;
	}

	priv->hse = devm_clk_get(dev, "hse");
	if (IS_ERR(priv->hse)) {
		dev_warn(dev, "Failed to get hse\n");
		priv->hse = NULL;
	}

	priv->clk_in1 = devm_clk_get(dev, "clk_in1");
	if (IS_ERR(priv->clk_in1))
		priv->clk_in1 = NULL;

	priv->clk_in2 = devm_clk_get(dev, "clk_in2");
	if (IS_ERR(priv->clk_in2))
		priv->clk_in2 = NULL;

	// get PLL rates
	rate =
	    nuclei_mux_get_rate(priv, CLK_CTRL0_SYS_PLL_CLK_S_OFS, 0, 2, NULL);
	priv->sys_pll_rate =
	    nuclei_pll_calc_rate(priv, PLL_CTRL0_SYS_PLL_CLK_OFS, rate);
	dev_dbg(dev, "SYS PLL: parent=%lu, out=%lu\n", rate,
		priv->sys_pll_rate);

	rate = nuclei_mux_get_rate(priv, CLK_CTRL2_CORE1_PLL_CLK_S_OFS, 0, 2,
				   NULL);
	priv->core1_pll_rate =
	    nuclei_pll_calc_rate(priv, PLL_CTRL1_CORE1_PLL_CLK_OFS, rate);
	dev_dbg(dev, "CORE1 PLL: parent=%lu, out=%lu\n", rate,
		priv->core1_pll_rate);

	rate =
	    nuclei_mux_get_rate(priv, CLK_CTRL3_XUC_PLL_CLK_S_OFS, 0, 2, NULL);
	priv->xuc_pll_rate =
	    nuclei_pll_calc_rate(priv, PLL_CTRL2_XUC_PLL_CLK_OFS, rate);
	dev_dbg(dev, "XUC PLL: parent=%lu, out=%lu\n", rate,
		priv->xuc_pll_rate);

	div = nuclei_get_div_value(priv, CLK_CTRL1_SYS_PLL_CLK_DIV2_OFS);
	priv->sys_pll_div2_rate = priv->sys_pll_rate / div;

	// get sys_clk sys_intf_clk
	priv->sys_clk_rate =
	    nuclei_mux_get_rate(priv, CLK_CTRL4_SYS_CLK_OFS, 0, 3, NULL);
	priv->sys_intf_clk_rate =
	    nuclei_mux_get_rate(priv, CLK_CTRL5_SYS_INTF_CLK_OFS, 0, 3, NULL);
	priv->xec_sys_rate =
	    nuclei_mux_get_rate(priv, CLK_CTRL16_XEC_SYS_CLK_S_OFS, 0, 3, NULL);
	priv->xec_rmii_rate = nuclei_mux_get_rate(
	    priv, CLK_CTRL17_XEC_RMII_CLK_S_OFS, 0, 3, NULL);
	dev_dbg(dev,
		"sys_clk=%lu, sys_intf_clk=%lu, xec_sys_clk=%lu, "
		"xec_rmii_clk=%lu\n",
		priv->sys_clk_rate, priv->sys_intf_clk_rate, priv->xec_sys_rate,
		priv->xec_rmii_rate);

	return 0;
}

static int nuclei_clk_bind(struct udevice* dev)
{
	return nuclei_reset_bind(dev, RST_MAX_NUM);
}

static const struct udevice_id nuclei_clk_ids[] = {
	{.compatible = "nuclei,ccu"},
	{}
};

U_BOOT_DRIVER(nuclei_clk) = {
	.name = "nuclei_clk",
	.id = UCLASS_CLK,
	.of_match = nuclei_clk_ids,
	.probe = nuclei_clk_probe,
	.ops = &nuclei_clk_ops,
	.priv_auto = sizeof(struct nuclei_clk_priv),
	.bind = nuclei_clk_bind,
};
