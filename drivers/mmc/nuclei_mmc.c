// SPDX-License-Identifier: GPL-2.0+
/*
 * Nuclei MMC driver
 *
 * Copyright (c) 2024 Nucleisys Technologies
 */

#include <common.h>
#include <malloc.h>
#include <mmc.h>
#include <asm/global_data.h>
#include <asm/io.h>
#include <asm/unaligned.h>
#include <errno.h>
#include <dm/device_compat.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <wait_bit.h>
#include <dm.h>
DECLARE_GLOBAL_DATA_PTR;

/* Registers */
#define SDIO_RX_SADDR				0x0
#define SDIO_RX_SIZE				0x4
#define SDIO_RX_CFG					0x8
#define SDIO_CR						0xC
#define SDIO_TX_SADDR				0x10
#define SDIO_TX_SIZE				0x14
#define SDIO_TX_CFG					0x18
#define SDIO_CMD_OP					0x20
#define SDIO_CMD_ARG				0x24
#define SDIO_DATA_SETUP				0x28
#define SDIO_START					0x2C
#define SDIO_RSP0					0x30
#define SDIO_RSP1					0x34
#define SDIO_RSP2					0x38
#define SDIO_RSP3					0x3C
#define SDIO_CLK_DIV				0x40
#define SDIO_STATUS					0x44
#define SDIO_DATA_TIMEOUT_CNT		0x50
#define SDIO_CMD_POWERUP_CNT		0x54
#define SDIO_TX_DATA				0x60
#define SDIO_RX_DATA				0x64
#define	SDIO_TX_MARK				0x68
#define	SDIO_RX_MARK				0x6C
#define SDIO_IP						0x70
#define SDIO_IE						0x74
#define SDIO_SAMPLE_DDR				0x78
#define SDIO_CRC_VALUE				0x8C

#define SDIO_CR_DMA_EN				BIT(0)
#define SDIO_CR_DDR_EN				BIT(1)

#define SDIO_RXFIFO_EMPTY			BIT(2)
#define SDIO_TXFIFO_FULL			BIT(3)

#define SDIO_STATUS_EOT				BIT(0)
#define SDIO_STATUS_BUSY			BIT(6)

#define SDIO_STATUS_CMDERR_RSP_TO	BIT(16)
#define SDIO_STATUS_CMDERR_WrongDir	BIT(17)
#define SDIO_STATUS_CMDERR_BUSY_TO	BIT(18)
#define SDIO_STATUS_CMDERR_CRC		BIT(19)

#define SDIO_CMD_OP_CRC_EN 			BIT(2)
#define SDIO_CMD_OP_POWER_EN	 	BIT(4)
#define SDIO_CMD_OP_CRC_CHECK_EN 	BIT(5)
#define SDIO_DATA_SETUP_EN			BIT(0)
#define SDIO_DATA_SETUP_RD			BIT(1)
#define SDIO_DATA_SETUP_MODE		GENMASK(3,2)
#define SDIO_DATA_SETUP_BLK_NUM		GENMASK(19, 4)
#define SDIO_DATA_SETUP_BLK_SIZE	GENMASK(31, 20)

#define NUCLEI_MMC_MAX_TIMEOUT		0x10000

#define NUCLEI_MISC_BASE            0xf8b300000
#define NUCLEI_IOMUX_BASE			0xf8bc00000

#define NUCLEI_MMC_INPUT_CLK		100000000



struct nuclei_mmc_plat {
	struct mmc_config cfg;
	struct mmc mmc;
};

struct nuclei_mmc_priv {
	void __iomem		*regs;
	u32			flags;
/* priv flags */
#define NUCLEI_MMC_BUS_WIDTH_MASK	0x3
#define NUCLEI_MMC_BUS_WIDTH_1		0x0
#define NUCLEI_MMC_BUS_WIDTH_4		0x1
#define NUCLEI_MMC_BUS_WIDTH_8		0x2
};

static int nuclei_mmc_config_clock(struct nuclei_mmc_priv *priv, struct mmc *mmc)
{
	int clk_div = 0;

	clk_div = NUCLEI_MMC_INPUT_CLK / mmc->clock;
	if (NUCLEI_MMC_INPUT_CLK % mmc->clock)
		clk_div++;
	if (clk_div < 2)
		return -EINVAL;
	
	writel((clk_div >> 1) - 1, priv->regs + SDIO_CLK_DIV);

	return 0;
}

static int nuclei_mmc_wait_for_completion(struct nuclei_mmc_priv *priv)
{
	int i;
	u32 stat;
	
	/* wait for completion */
	for (i = 0; i < 0x1000; i++) {
		stat = readl(priv->regs + SDIO_STATUS);
		if (stat & SDIO_STATUS_EOT) {
			break;
		}
		udelay(100);
	}
	/* check err */
	if (stat & SDIO_STATUS_CMDERR_RSP_TO){
		printf("%s resp timeout\n", __func__);
		return -ETIMEDOUT;
	}
	if (stat & SDIO_STATUS_CMDERR_BUSY_TO) {
		printf("%s busy timeout\n", __func__);
		return -ETIMEDOUT;
	}	
	if (stat & SDIO_STATUS_CMDERR_CRC) {
		printf("%s crc err\n", __func__);
		return -ECOMM;
	}
	if (stat & SDIO_STATUS_CMDERR_WrongDir) {
		printf("%s dirction err\n", __func__);
		return -ECOMM;
	}
	/* clear stat */
	writel(stat, priv->regs + SDIO_STATUS);
	
	return 0;
}

static int nuclei_mmc_transfer_data(struct nuclei_mmc_priv *priv, struct mmc_data *data)
{
	int sz = data->blocks * data->blocksize;
	uint32_t *buf;
	int timeout = NUCLEI_MMC_MAX_TIMEOUT;
	uint32_t reg;

	if (data->flags & MMC_DATA_READ){
		buf = data->dest;
	    while (sz > 0 && --timeout > 0) {
	        reg = readl(priv->regs + SDIO_IP);
	        if (!(reg & SDIO_RXFIFO_EMPTY)) {
	            *buf++ = readl(priv->regs + SDIO_RX_DATA);
	            sz -= 4;
	            timeout = NUCLEI_MMC_MAX_TIMEOUT;
	        } else
	            udelay(100);
	    }
	} else if (data->flags & MMC_DATA_WRITE) {
		buf = data->src;
	    while (sz > 0 && --timeout > 0) {
	        reg = readl(priv->regs + SDIO_IP);
	        if (!(reg & SDIO_TXFIFO_FULL)) {
	            writel(*buf++, priv->regs + SDIO_TX_DATA);
	            sz -= 4;
	            timeout = NUCLEI_MMC_MAX_TIMEOUT;
	        } else
	            udelay(100);
	    }		
	}
	if (timeout == 0)
		return -ECOMM;
	return nuclei_mmc_wait_for_completion(priv);
}

static int nuclei_mmc_send_cmd(struct mmc *mmc, struct nuclei_mmc_priv *priv,
			   struct mmc_cmd *cmd, struct mmc_data *data)
{
	u32 stat, mask, cmdat = 0;
	int i, ret=0;
	ulong start = get_timer(0);
	ulong timeout = 5000;
	u32 val = 0;

	while (readl(priv->regs + SDIO_STATUS) & SDIO_STATUS_BUSY) {
		if (get_timer(start) > timeout) {
			printf("%s: Timeout on data busy\n", __func__);
			return -ETIMEDOUT;
		}
	}

	/* setup command and response*/
	val = cmd->cmdidx << 8;
	val |= cmd->resp_type & 0xf;
	writel(val, priv->regs + SDIO_CMD_OP);
	writel(cmd->cmdarg, priv->regs + SDIO_CMD_ARG);

	if (data) {
		/* setup data */
		val = SDIO_DATA_SETUP_EN;
		if (data->flags & MMC_DATA_READ)
			val |= SDIO_DATA_SETUP_RD;
		val |= (priv->flags & NUCLEI_MMC_BUS_WIDTH_MASK) << 2;
		val |= ((data->blocks-1) << 4) & SDIO_DATA_SETUP_BLK_NUM;
		val |= ((data->blocksize-1) << 20) & SDIO_DATA_SETUP_BLK_SIZE;
		writel(val, priv->regs + SDIO_DATA_SETUP);
	} else {
		val = 0;
	}
	/* write the data setup */
	writel(val, priv->regs + SDIO_DATA_SETUP);
	/* start transmission */
	writel(1, priv->regs + SDIO_START);

	ret = nuclei_mmc_wait_for_completion(priv);
	if (ret) {
		printf("%s: transmission err\n", __func__);
		return ret;
	}
	
	if (cmd->resp_type & MMC_RSP_PRESENT) {
		/* read the response */
		if (cmd->resp_type & MMC_RSP_136) {
			cmd->response[0] = readl(priv->regs + SDIO_RSP3);
			cmd->response[1] = readl(priv->regs + SDIO_RSP2);
			cmd->response[2] = readl(priv->regs + SDIO_RSP1);
			cmd->response[3] = readl(priv->regs + SDIO_RSP0);
		} else {
			cmd->response[0] = readl(priv->regs + SDIO_RSP0);
			cmd->response[1] = readl(priv->regs + SDIO_RSP1) & 0x3f;
		}
	}
	if (data) 
		ret = nuclei_mmc_transfer_data(priv, data);

	return ret;
}

static int nuclei_mmc_set_ios(struct mmc *mmc, struct nuclei_mmc_priv *priv)
{
	u32 val;
	
	/* change clock */
	if (mmc->clock && nuclei_mmc_config_clock(priv, mmc) != 0)
		return -EINVAL;

	/* set sdr/ddr mode */
	val = readl(priv->regs + SDIO_CR);
	val &= ~SDIO_CR_DDR_EN;
	if (mmc->ddr_mode)
		val |= SDIO_CR_DDR_EN;
	writel(val, priv->regs + SDIO_CR);
	
	/* set the bus width for the next command */
	priv->flags &= ~NUCLEI_MMC_BUS_WIDTH_MASK;
	if (mmc->bus_width == 8)
		priv->flags |= NUCLEI_MMC_BUS_WIDTH_8;
	else if (mmc->bus_width == 4)
		priv->flags |= NUCLEI_MMC_BUS_WIDTH_4;
	else
		priv->flags |= NUCLEI_MMC_BUS_WIDTH_1;

	return 0;
}

static void nuclei_mmc_config_iomux(void)
{
	u32 val;

	/* config sdio clk */
	__raw_writel(1, NUCLEI_IOMUX_BASE + 0x4000 + 0x4 * 20);
	val = __raw_readl(NUCLEI_IOMUX_BASE + 0x10000 + 0x4 * 20);
	val |= 32 | 8;
	__raw_writel(val, NUCLEI_IOMUX_BASE + 0x10000 + 0x4 * 20);
	/* config sdio cmd */
	__raw_writel(1, NUCLEI_IOMUX_BASE + 0x4000 + 0x4 * 21);
	val = __raw_readl(NUCLEI_IOMUX_BASE + 0x10000 + 0x4 * 21);
	val |= 32 | 8;
	__raw_writel(val, NUCLEI_IOMUX_BASE + 0x10000 + 0x4 * 21);
	
	/* config sdio data0 */
	__raw_writel(1, NUCLEI_IOMUX_BASE + 0x4000 + 0x4 * 22);
	val = __raw_readl(NUCLEI_IOMUX_BASE + 0x10000 + 0x4 * 22);
	val |= 32 | 8 | 1;
	__raw_writel(val, NUCLEI_IOMUX_BASE + 0x10000 + 0x4 * 22);
	/* config sdio data1 */
	__raw_writel(1, NUCLEI_IOMUX_BASE + 0x4000 + 0x4 * 23);
	val = __raw_readl(NUCLEI_IOMUX_BASE + 0x10000 + 0x4 * 23);
	val |= 32 | 8 | 1;
	__raw_writel(val, NUCLEI_IOMUX_BASE + 0x10000 + 0x4 * 23);
	/* config sdio data2 */
	__raw_writel(1, NUCLEI_IOMUX_BASE + 0x4000 + 0x4 * 24);
	val = __raw_readl(NUCLEI_IOMUX_BASE + 0x10000 + 0x4 * 24);
	val |= 32 | 8 | 1;
	__raw_writel(val, NUCLEI_IOMUX_BASE + 0x10000 + 0x4 * 24);
	/* config sdio data3 */
	__raw_writel(1, NUCLEI_IOMUX_BASE + 0x4000 + 0x4 * 25);
	val = __raw_readl(NUCLEI_IOMUX_BASE + 0x10000 + 0x4 * 25);
	val |= 32 | 8 | 1;
	__raw_writel(val, NUCLEI_IOMUX_BASE + 0x10000 + 0x4 * 25);

	mb();
}

static int nuclei_mmc_core_init(struct mmc *mmc)
{
	struct nuclei_mmc_priv *priv = mmc->priv;
	int ret;

	/* Reset */
	ret = readl(NUCLEI_MISC_BASE + 0x20);
	ret &= ~(1<<8);
	writel(ret, NUCLEI_MISC_BASE + 0x20);
	ret |= 1<<8;
	writel(ret, NUCLEI_MISC_BASE + 0x20);

	/* disable interrupt */
	writel(0, priv->regs + SDIO_IE);

	/* Maximum timeouts */
	writel(0xffffffff, priv->regs + SDIO_DATA_TIMEOUT_CNT);

	/* config sdio iomux */
	nuclei_mmc_config_iomux();

	return 0;
}

static int nuclei_mmc_dm_send_cmd(struct udevice *dev, struct mmc_cmd *cmd,
			      struct mmc_data *data)
{
	struct nuclei_mmc_priv *priv = dev_get_priv(dev);
	struct mmc *mmc = mmc_get_mmc_dev(dev);

	return nuclei_mmc_send_cmd(mmc, priv, cmd, data);
}

static int nuclei_mmc_dm_set_ios(struct udevice *dev)
{
	struct nuclei_mmc_priv *priv = dev_get_priv(dev);
	struct mmc *mmc = mmc_get_mmc_dev(dev);

	return nuclei_mmc_set_ios(mmc, priv);
};

static const struct dm_mmc_ops nuclei_mmc_ops = {
	.send_cmd	= nuclei_mmc_dm_send_cmd,
	.set_ios	= nuclei_mmc_dm_set_ios,
};

static int nuclei_mmc_of_to_plat(struct udevice *dev)
{
	struct nuclei_mmc_priv *priv = dev_get_priv(dev);
	struct nuclei_mmc_plat *plat = dev_get_plat(dev);
	struct mmc_config *cfg;
	int ret;

	priv->regs = map_physmem(dev_read_addr(dev), 0x1000, MAP_NOCACHE);
	cfg = &plat->cfg;

	cfg->name = "nuclei_mmc";
	cfg->host_caps = MMC_MODE_HS_52MHz | MMC_MODE_HS;

	ret = mmc_of_parse(dev, cfg);
	if (ret < 0) {
		dev_err(dev, "failed to parse host caps\n");
		return ret;
	}

	cfg->f_min = 400000;

	cfg->voltages = MMC_VDD_32_33 | MMC_VDD_33_34;
	cfg->b_max = CONFIG_SYS_MMC_MAX_BLK_COUNT;

	return 0;
}

static int nuclei_mmc_bind(struct udevice *dev)
{
	struct nuclei_mmc_plat *plat = dev_get_plat(dev);

	return mmc_bind(dev, &plat->mmc, &plat->cfg);
}

static int nuclei_mmc_probe(struct udevice *dev)
{
	struct mmc_uclass_priv *upriv = dev_get_uclass_priv(dev);
	struct nuclei_mmc_priv *priv = dev_get_priv(dev);
	struct nuclei_mmc_plat *plat = dev_get_plat(dev);

	plat->mmc.priv = priv;
	upriv->mmc = &plat->mmc;
	return nuclei_mmc_core_init(&plat->mmc);
}

static const struct udevice_id nuclei_mmc_ids[] = {
	{ .compatible = "nuclei,mmc" },
	{ }
};

U_BOOT_DRIVER(nuclei_mmc_drv) = {
	.name			= "nuclei_mmc",
	.id			= UCLASS_MMC,
	.of_match		= nuclei_mmc_ids,
	.of_to_plat	= nuclei_mmc_of_to_plat,
	.bind			= nuclei_mmc_bind,
	.probe			= nuclei_mmc_probe,
	.priv_auto	= sizeof(struct nuclei_mmc_priv),
	.plat_auto	= sizeof(struct nuclei_mmc_plat),
	.ops			= &nuclei_mmc_ops,
};
