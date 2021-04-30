// SPDX-License-Identifier: GPL-2.0+
/*
 * Nuclei GPIO driver
 *
 * Copyright (C) 2021  Nuclei System Technology
 * modified based on sifive_gpio.c
 */

#include <common.h>
#include <dm.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <errno.h>
#include <linux/bitops.h>

#define GPIO_INPUT_VAL	0x00
#define GPIO_INPUT_EN	0x04
#define GPIO_OUTPUT_EN	0x08
#define GPIO_OUTPUT_VAL	0x0C
#define GPIO_RISE_IE	0x18
#define GPIO_RISE_IP	0x1C
#define GPIO_FALL_IE	0x20
#define GPIO_FALL_IP	0x24
#define GPIO_HIGH_IE	0x28
#define GPIO_HIGH_IP	0x2C
#define GPIO_LOW_IE	0x30
#define GPIO_LOW_IP	0x34
#define GPIO_OUTPUT_XOR	0x40

#define NR_GPIOS	16

enum gpio_state {
	LOW,
	HIGH
};

/* Details about a GPIO bank */
struct nuclei_gpio_platdata {
	void *base;     /* address of registers in physical memory */
};

static int nuclei_gpio_probe(struct udevice *dev)
{
	struct nuclei_gpio_platdata *plat = dev_get_platdata(dev);
	struct gpio_dev_priv *uc_priv = dev_get_uclass_priv(dev);
	char name[18], *str;

	sprintf(name, "gpio@%4lx_", (uintptr_t)plat->base);
	str = strdup(name);
	if (!str)
		return -ENOMEM;
	uc_priv->bank_name = str;

	/*
	 * Use the gpio count mentioned in device tree,
	 * if not specified in dt, set NR_GPIOS as default
	 */
	uc_priv->gpio_count = dev_read_u32_default(dev, "ngpios", NR_GPIOS);

	return 0;
}

static void nuclei_update_gpio_reg(void *bptr, u32 offset, bool value)
{
	void __iomem *ptr = (void __iomem *)bptr;

	u32 bit = BIT(offset);
	u32 old = readl(ptr);

	if (value)
		writel(old | bit, ptr);
	else
		writel(old & ~bit, ptr);
}

static int nuclei_gpio_direction_input(struct udevice *dev, u32 offset)
{
	struct nuclei_gpio_platdata *plat = dev_get_platdata(dev);
	struct gpio_dev_priv *uc_priv = dev_get_uclass_priv(dev);

	if (offset > uc_priv->gpio_count)
		return -EINVAL;

	/* Configure gpio direction as input */
	nuclei_update_gpio_reg(plat->base + GPIO_INPUT_EN,  offset, true);
	nuclei_update_gpio_reg(plat->base + GPIO_OUTPUT_EN, offset, false);

	return 0;
}

static int nuclei_gpio_direction_output(struct udevice *dev, u32 offset,
					int value)
{
	struct nuclei_gpio_platdata *plat = dev_get_platdata(dev);
	struct gpio_dev_priv *uc_priv = dev_get_uclass_priv(dev);

	if (offset > uc_priv->gpio_count)
		return -EINVAL;

	/* Configure gpio direction as output */
	nuclei_update_gpio_reg(plat->base + GPIO_OUTPUT_EN, offset, true);
	nuclei_update_gpio_reg(plat->base + GPIO_INPUT_EN,  offset, false);

	/* Set the output state of the pin */
	nuclei_update_gpio_reg(plat->base + GPIO_OUTPUT_VAL, offset, value);

	return 0;
}

static int nuclei_gpio_get_value(struct udevice *dev, u32 offset)
{
	struct nuclei_gpio_platdata *plat = dev_get_platdata(dev);
	struct gpio_dev_priv *uc_priv = dev_get_uclass_priv(dev);
	int val;
	int dir;

	if (offset > uc_priv->gpio_count)
		return -EINVAL;

	/* Get direction of the pin */
	dir = !(readl(plat->base + GPIO_OUTPUT_EN) & BIT(offset));

	if (dir)
		val = readl(plat->base + GPIO_INPUT_VAL) & BIT(offset);
	else
		val = readl(plat->base + GPIO_OUTPUT_VAL) & BIT(offset);

	return val ? HIGH : LOW;
}

static int nuclei_gpio_set_value(struct udevice *dev, u32 offset, int value)
{
	struct nuclei_gpio_platdata *plat = dev_get_platdata(dev);
	struct gpio_dev_priv *uc_priv = dev_get_uclass_priv(dev);

	if (offset > uc_priv->gpio_count)
		return -EINVAL;

	nuclei_update_gpio_reg(plat->base + GPIO_OUTPUT_VAL, offset, value);

	return 0;
}

static int nuclei_gpio_get_function(struct udevice *dev, unsigned int offset)
{
	struct nuclei_gpio_platdata *plat = dev_get_platdata(dev);
	u32	outdir, indir, val;
	struct gpio_dev_priv *uc_priv = dev_get_uclass_priv(dev);

	if (offset > uc_priv->gpio_count)
		return -1;

	/* Get direction of the pin */
	outdir = readl(plat->base + GPIO_OUTPUT_EN) & BIT(offset);
	indir  = readl(plat->base + GPIO_INPUT_EN) & BIT(offset);

	if (outdir)
		/* Pin at specified offset is configured as output */
		val = GPIOF_OUTPUT;
	else if (indir)
		/* Pin at specified offset is configured as input */
		val = GPIOF_INPUT;
	else
		/*The requested GPIO is not set as input or output */
		val = GPIOF_UNUSED;

	return val;
}

static const struct udevice_id nuclei_gpio_match[] = {
	{ .compatible = "nuclei,gpio0" },
	{ }
};

static const struct dm_gpio_ops nuclei_gpio_ops = {
	.direction_input        = nuclei_gpio_direction_input,
	.direction_output       = nuclei_gpio_direction_output,
	.get_value              = nuclei_gpio_get_value,
	.set_value              = nuclei_gpio_set_value,
	.get_function		= nuclei_gpio_get_function,
};

static int nuclei_gpio_ofdata_to_platdata(struct udevice *dev)
{
	struct nuclei_gpio_platdata *plat = dev_get_platdata(dev);
	fdt_addr_t addr;

	addr = dev_read_addr(dev);
	if (addr == FDT_ADDR_T_NONE)
		return -EINVAL;

	plat->base = (void *)addr;
	return 0;
}

U_BOOT_DRIVER(gpio_nuclei) = {
	.name	= "gpio_nuclei",
	.id	= UCLASS_GPIO,
	.of_match = nuclei_gpio_match,
	.ofdata_to_platdata = of_match_ptr(nuclei_gpio_ofdata_to_platdata),
	.platdata_auto_alloc_size = sizeof(struct nuclei_gpio_platdata),
	.ops	= &nuclei_gpio_ops,
	.probe	= nuclei_gpio_probe,
};
