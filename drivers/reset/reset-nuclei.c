// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/*
 * Copyright (C) 2026 Nucleisys.
 *
 */

#include <common.h>
#include <dm.h>
#include <dm/device-internal.h>
#include <dm/device_compat.h>
#include <dm/lists.h>
#include <linux/bitops.h>
#include <linux/io.h>
#include <reset-uclass.h>

/* reset registers */
#define RESET_CTRL0_OFS 0x020
#define RESET_CTRL1_OFS 0x024
#define RESET_CTRL2_OFS 0x028
#define RESET_CTRL3_OFS 0x02C

struct nuclei_reset_priv {
	void __iomem* base;
	int nr_reset;
};

static void nuclei_reset_id_to_reg(unsigned long id, unsigned int* reg_off,
				   unsigned int* bit)
{
	unsigned int reg_index = id / 32;
	*bit = id % 32;
	*reg_off = RESET_CTRL0_OFS + reg_index * 4;
}

static int nuclei_reset_status(struct reset_ctl* reset)
{
	struct nuclei_reset_priv* priv = dev_get_priv(reset->dev);
	unsigned int reg_off, bit;
	u32 val;

	if (reset->id >= priv->nr_reset)
		return -EINVAL;
	nuclei_reset_id_to_reg(reset->id, &reg_off, &bit);

	val = readl(priv->base + reg_off);
	return !(val & BIT(bit)); /* 0=reset, 1=release */
}

static int nuclei_reset_assert(struct reset_ctl* reset)
{
	struct nuclei_reset_priv* priv = dev_get_priv(reset->dev);
	unsigned int reg_off, bit;
	u32 val;

	if (reset->id >= priv->nr_reset)
		return -EINVAL;
	nuclei_reset_id_to_reg(reset->id, &reg_off, &bit);

	val = readl(priv->base + reg_off);
	val &= ~BIT(bit);
	writel(val, priv->base + reg_off);

	return 0;
}

static int nuclei_reset_deassert(struct reset_ctl* reset)
{
	struct nuclei_reset_priv* priv = dev_get_priv(reset->dev);
	unsigned int reg_off, bit;
	u32 val;

	if (reset->id >= priv->nr_reset)
		return -EINVAL;
	nuclei_reset_id_to_reg(reset->id, &reg_off, &bit);

	val = readl(priv->base + reg_off);
	val |= BIT(bit);
	writel(val, priv->base + reg_off);

	return 0;
}

static const struct reset_ops nuclei_reset_ops = {
	.rst_status = nuclei_reset_status,
	.rst_assert = nuclei_reset_assert,
	.rst_deassert = nuclei_reset_deassert,
};

static int nuclei_reset_probe(struct udevice* dev)
{
	struct nuclei_reset_priv* priv = dev_get_priv(dev);

	priv->base = dev_remap_addr(dev);
	if (!priv->base)
		return -ENOMEM;

	return 0;
}

int nuclei_reset_bind(struct udevice* dev, ulong count)
{
	struct udevice* rst_dev;
	struct nuclei_reset_priv* priv;
	int ret;

	ret = device_bind_driver_to_node(dev, "nuclei_reset", "reset",
					 dev_ofnode(dev), &rst_dev);
	if (ret) {
		dev_err(dev, "failed to bind nuclei_reset driver (ret=%d)\n",
			ret);
		return ret;
	}
	priv = malloc(sizeof(struct nuclei_reset_priv));
	priv->nr_reset = count;
	dev_set_priv(rst_dev, priv);

	return 0;
}

U_BOOT_DRIVER(nuclei_reset) = {
	.name = "nuclei_reset",
	.id = UCLASS_RESET,
	.probe = nuclei_reset_probe,
	.ops = &nuclei_reset_ops,
	.priv_auto = sizeof(struct nuclei_reset_priv),
};
