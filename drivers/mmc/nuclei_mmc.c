// SPDX-License-Identifier: GPL-2.0+
/*
 * Nuclei MMC driver
 *
 * Copyright (c) 2024 Nucleisys Technologies
 */

#include <common.h>
#include <clk.h>
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
#include <linux/dma-mapping.h>

DECLARE_GLOBAL_DATA_PTR;

/* Registers */
#define SDIO_RX_SADDR				0x0
#define SDIO_RX_SIZE				0x4
#define SDIO_RX_CFG					0x8
#define SDIO_CR						0xC
#define SDIO_TX_SADDR				0x10
#define SDIO_TX_SIZE				0x14
#define SDIO_TX_CFG					0x18
#define SDIO_VERSION				0x1C
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
#define SDIO_CMD_WAIT_RSP_CNT		0x58
#define SDIO_TX_DATA				0x60
#define SDIO_RX_DATA				0x64
#define SDIO_TX_MARK				0x68
#define SDIO_RX_MARK				0x6C
#define SDIO_IP						0x70
#define SDIO_IE						0x74
#define SDIO_SAMPLE_DDR				0x78
#define SDIO_CRC_VALUE				0x8C
#define SDIO_DMA_INTR_EN			0x1C00
#define SDIO_DMA_INTR_STAT			0x1C04
#define SDIO_DMA_INTR_CLR			0x1C08


#define SDIO_DMA_INT_EN_RX_FTRANS	BIT(0)
#define SDIO_DMA_INT_EN_TX_FTRANS	BIT(3)
#define SDIO_DMA_INT_STAT_RX_FTRANS	BIT(0)
#define SDIO_DMA_INT_STAT_TX_FTRANS	BIT(3)
#define SDIO_DMA_INT_CLR_RX_FTRANS	BIT(0)
#define SDIO_DMA_INT_CLR_TX_FTRANS	BIT(3)


#define SDIO_DMA_TX_RX_EN			BIT(4)
#define SDIO_DMA_DATASIZE_MASK		GENMASK(2,1)
#define SDIO_DMA_DATASIZE_WORD		(2<<1)
#define SDIO_DMA_DATASIZE_HALFWORD	(1<<1)

#define SDIO_CR_DMA_EN				BIT(0)
#define SDIO_CR_DDR_EN				BIT(1)
#define SDIO_CR_CLK_STOP			BIT(5)
#define SDIO_CR_RST_MODE			GENMASK(22, 21)
#define SDIO_CR_RST_MODE_OFFSET		21
#define SDIO_CR_RST_MODE_PWR_ON		0x3

#define SDIO_RXFIFO_RX_IRQ			BIT(1)
#define SDIO_RXFIFO_EMPTY			BIT(2)
#define SDIO_TXFIFO_FULL			BIT(3)

#define SDIO_STATUS_EOT				BIT(0)
#define SDIO_STATUS_BUSY			BIT(6)

#define SDIO_STATUS_CMDERR_RSP_TO	BIT(16)
#define SDIO_STATUS_CMDERR_WrongDir	BIT(17)
#define SDIO_STATUS_CMDERR_BUSY_TO	BIT(18)
#define SDIO_STATUS_CMDERR_CRC		BIT(19)
#define SDIO_STATUS_DATAERR			GENMASK(29, 24)

#define SDIO_CMD_OP_CRC_EN			BIT(2)
#define SDIO_CMD_OP_POWER_EN		BIT(4)
#define SDIO_CMD_OP_CRC_CHECK_EN	BIT(5)
#define SDIO_CMD_OP_CMD_EN			BIT(7)
#define SDIO_CMD_RSP_MASK			GENMASK(3,0)
#define SDIO_CMD_OP_MASK			GENMASK(13,8)

#define SDIO_DATA_SETUP_EN			BIT(0)
#define SDIO_DATA_SETUP_RD			BIT(1)
#define SDIO_DATA_SETUP_MODE		GENMASK(3,2)
#define SDIO_DATA_SETUP_BLK_NUM		GENMASK(19, 4)
#define SDIO_DATA_SETUP_BLK_SIZE	GENMASK(31, 20)

#define NUCLEI_MMC_MAX_TIMEOUT		0xFFFFFFFF

#define NUCLEI_MISC_BASE			0xf9c880000ULL
#define NUCLEI_IOMUX_BASE			0xf9ca00000ULL

#define NUCLEI_MMC_RX_WMARK			56

#define CONFIG_NUCLEI_MMC_PIO

struct nuclei_mmc_plat {
	struct mmc_config cfg;
	struct mmc mmc;
	struct clk clk;
};

struct nuclei_mmc_priv {
	void __iomem *regs;
	u32 flags;
/* priv flags */
#define NUCLEI_MMC_BUS_WIDTH_MASK	0x3
#define NUCLEI_MMC_BUS_WIDTH_1		0x0
#define NUCLEI_MMC_BUS_WIDTH_4		0x1
#define NUCLEI_MMC_BUS_WIDTH_8		0x2
	u32 bus_clk_rate;
	u32 rx_watermark;
	u32 tx_watermark;
};

static int nuclei_mmc_config_clock(struct nuclei_mmc_priv *priv, struct mmc *mmc)
{
	int clk_div = 0;

	clk_div = priv->bus_clk_rate / mmc->clock;
	if (priv->bus_clk_rate % mmc->clock)
		clk_div++;
	if (clk_div < 2)
		return -EINVAL;
	printf("clk_div:%x-%x-%x\n", clk_div,priv->bus_clk_rate, mmc->clock);
	writel((clk_div >> 1) - 1, priv->regs + SDIO_CLK_DIV);

	return 0;
}

static int nuclei_mmc_wait_for_completion(struct nuclei_mmc_priv *priv)
{
	int i;
	u32 stat;

	/* wait for completion */
	for (i = 0; i < NUCLEI_MMC_MAX_TIMEOUT; i++) {
		stat = readl(priv->regs + SDIO_STATUS);
		if (stat & SDIO_STATUS_EOT) {
			break;
		}
		udelay(50);
	}

	if ((stat & SDIO_STATUS_EOT)==0) printf("timeout overflow\n");
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
	uint32_t timeout = NUCLEI_MMC_MAX_TIMEOUT;
	uint32_t reg;
	int i;

	if (data->flags & MMC_DATA_READ) {
		buf = (uint32_t *)data->dest;
		while (sz > 0 && timeout > 0) {
			reg = readl(priv->regs + SDIO_IP);
			if (sz > NUCLEI_MMC_RX_WMARK * 4){
				if (reg & SDIO_RXFIFO_RX_IRQ) {
					for(i = 0; i < NUCLEI_MMC_RX_WMARK; i++)
						*buf++ = readl(priv->regs + SDIO_RX_DATA);
					sz -= NUCLEI_MMC_RX_WMARK * 4;
				} else {
					timeout--;
					udelay(50);
				}
			} else {
				if (!(reg & SDIO_RXFIFO_EMPTY)) {
					*buf++ = readl(priv->regs + SDIO_RX_DATA);
					sz -= 4;
				} else {
					timeout--;
					udelay(50);
				}
			}
		}
	} else if (data->flags & MMC_DATA_WRITE) {
		buf = (uint32_t *)data->src;
		while (sz > 0 && timeout > 0) {
			reg = readl(priv->regs + SDIO_IP);
			if (!(reg & SDIO_TXFIFO_FULL)) {
				writel(*buf++, priv->regs + SDIO_TX_DATA);
				sz -= 4;
			} else {
				timeout--;
				udelay(50);
			}
		}
	}
	if (timeout == 0)
		return -ECOMM;

	return 0;
}

static int nuclei_mmc_send_cmd(struct mmc *mmc, struct nuclei_mmc_priv *priv,
			   struct mmc_cmd *cmd, struct mmc_data *data)
{
	int ret=0;
	ulong start = get_timer(0);
	ulong timeout = 100000;
	u32 val = 0;

	while ((readl(priv->regs + SDIO_STATUS) & SDIO_STATUS_BUSY) &&
		(cmd->cmdidx != MMC_CMD_STOP_TRANSMISSION)) {
		if (get_timer(start) > timeout) {
			printf("%s: Timeout on data busy\n", __func__);
			return -ETIMEDOUT;
		}
	}

	/* setup command and response*/
	val = readl(priv->regs + SDIO_CMD_OP);
	val &=~SDIO_CMD_OP_MASK;
	val &=~SDIO_CMD_RSP_MASK;
	val |= cmd->cmdidx << 8;
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
	//printf("data_setup:%x\n", val);
	/* write the data setup */
	writel(val, priv->regs + SDIO_DATA_SETUP);
	/* start transmission */
	writel(1, priv->regs + SDIO_START);

	if (data) {
		ret = nuclei_mmc_transfer_data(priv, data);
		if (ret) {
			printf("%s: data timeout err:%d\n", __func__, ret);
			return ret;
		}
	}
	ret = nuclei_mmc_wait_for_completion(priv);
	if (ret) {
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
		}
	}

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
}

static int nuclei_mmc_core_init(struct mmc *mmc)
{
	struct nuclei_mmc_priv *priv = mmc->priv;
	uint32_t val;

	/* sdio clk en */
	val = readl((void*)(NUCLEI_MISC_BASE + 0x48));
	val |= 1<<21 ;
	writel(val, (void*)(NUCLEI_MISC_BASE + 0x48));

	/* Reset sdio ip */
	val = readl((void*)(NUCLEI_MISC_BASE + 0x28));
	val &= ~(1<<21);
	writel(val, (void*)(NUCLEI_MISC_BASE + 0x28));
	val |= 1<<21 ;
	writel(val, (void*)(NUCLEI_MISC_BASE + 0x28));

	/* disable interrupt */
	writel(0, priv->regs + SDIO_IE);

	priv->rx_watermark = NUCLEI_MMC_RX_WMARK;
	writel(priv->rx_watermark, priv->regs + SDIO_RX_MARK);

	/* Maximum timeouts */
	writel(0xFFFFFFFF, priv->regs + SDIO_DATA_TIMEOUT_CNT);
	writel(0x1000, priv->regs + SDIO_CMD_WAIT_RSP_CNT);

	val = readl(priv->regs + SDIO_CR);
	val &= ~SDIO_CR_RST_MODE;
	val |= SDIO_CR_RST_MODE_PWR_ON << SDIO_CR_RST_MODE_OFFSET;
	writel(val, priv->regs + SDIO_CR);

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
	if (!priv->regs) {
		dev_err(dev, "can't get registers base address\n");
		return -ENOENT;
	}

	ret = clk_get_by_index(dev, 0, &plat->clk);
	if (ret < 0)
		return ret;

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
	int ret;
	unsigned long clk_rate;
	struct mmc_uclass_priv *upriv = dev_get_uclass_priv(dev);
	struct nuclei_mmc_priv *priv = dev_get_priv(dev);
	struct nuclei_mmc_plat *plat = dev_get_plat(dev);

	plat->mmc.priv = priv;
	upriv->mmc = &plat->mmc;

	/* enable clock */
	ret = clk_enable(&plat->clk);
	if (ret < 0)
		return ret;

	clk_rate = clk_get_rate(&plat->clk);
	if (!clk_rate)
		return -EINVAL;

	priv->bus_clk_rate = clk_rate;

	nuclei_mmc_core_init(&plat->mmc);
	printf("SDIO Version:%x ", readl(priv->regs + SDIO_VERSION));

	return 0;
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
