// SPDX-License-Identifier: GPL-2.0+

#include <dm.h>
#include <log.h>
#include <asm/csr.h>

int spl_soc_init(void)
{
	int ret;
	struct udevice *dev;

	/* DDR init */
	ret = uclass_get_device(UCLASS_RAM, 0, &dev);
	if (ret) {
		debug("DRAM init failed: %d\n", ret);
		return ret;
	}

	return 0;
}

extern void soc_clk_init(void);
extern int ddr_init(void);
void harts_early_init(void)
{
	soc_clk_init();
	if (ddr_init())
		asm volatile("j . ");
}
