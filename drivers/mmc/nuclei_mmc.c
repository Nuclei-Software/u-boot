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
#define SDIO_DATA_SETUP_EN			BIT(0)
#define SDIO_DATA_SETUP_RD			BIT(1)
#define SDIO_DATA_SETUP_MODE		GENMASK(3,2)
#define SDIO_DATA_SETUP_BLK_NUM		GENMASK(19, 4)
#define SDIO_DATA_SETUP_BLK_SIZE	GENMASK(31, 20)

/*poll mode max timeout 1000 is work well */
#define NUCLEI_MMC_MAX_TIMEOUT		1000

#define NUCLEI_MISC_BASE			0xf8b300000
#define NUCLEI_IOMUX_BASE			0xf8bc00000

#define NUCLEI_MMC_INPUT_CLK		100000000

#define CONFIG_NUCLEI_MMC_PIO

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
/* Because hardware limit tranfersize to 1M,
 * so each segment maxlen fixed to 1Mb-512b
 * dma_segment_num indicate dma transfer times.
 */
	int			dma_cmdarg;
	int			dma_remain_blocks;
};

static int nuclei_mmc_config_clock(struct nuclei_mmc_priv *priv, struct mmc *mmc)
{
	int clk_div = 0;

	clk_div = NUCLEI_MMC_INPUT_CLK / mmc->clock;
	if (NUCLEI_MMC_INPUT_CLK % mmc->clock)
		clk_div++;
	if (clk_div < 2)
		return -EINVAL;
	printf("clk_div:%x\n", clk_div);
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
	if (stat & SDIO_STATUS_DATAERR) {
		printf("%s data transfer err:%x\n", __func__, stat);
		return -ECOMM;
	}
	if (stat & GENMASK(5, 1)) printf("trans err:%x\n", stat);
	/* clear stat */
	writel(stat, priv->regs + SDIO_STATUS);

	return 0;
}

void dump_data(char *buf, int len)
{
#ifdef DUMP_DATA
	int i;

	printf("\n==>dumping buf %x, len:%x\n", buf, len);
	for(i = 0; i < len; i++) {
		printf("%02x ", buf[i]);
		if ((i+1)%16 ==0)
			printf("\n");
	}
#endif
}

#if defined(CONFIG_NUCLEI_MMC_PIO)
static int nuclei_mmc_setup_pio(struct nuclei_mmc_priv *priv, struct mmc_data *data)
{
	u32 val = 0;

	/* setup data */
	val = SDIO_DATA_SETUP_EN;
	if (data->flags & MMC_DATA_READ)
		val |= SDIO_DATA_SETUP_RD;
	val |= (priv->flags & NUCLEI_MMC_BUS_WIDTH_MASK) << 2;
	val |= ((data->blocks-1) << 4) & SDIO_DATA_SETUP_BLK_NUM;
	val |= ((data->blocksize-1) << 20) & SDIO_DATA_SETUP_BLK_SIZE;
	//printf("data_setup:0x%x\n", val);
	writel(val, priv->regs + SDIO_DATA_SETUP);
}

static int nuclei_mmc_wait_pio(struct nuclei_mmc_priv *priv, struct mmc_data *data)
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
	}
	else if (data->flags & MMC_DATA_WRITE) {
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
	dump_data(data->dest, data->blocks * data->blocksize);
	return nuclei_mmc_wait_for_completion(priv);
}
#else
void dump_reg(struct nuclei_mmc_priv *priv)
{
#ifdef DUMP_REG
	printf("rx_addr:%x,rx_size:%x,cr:%x,cfg:%x\n",
		readl(priv->regs + SDIO_RX_SADDR),
		readl(priv->regs + SDIO_RX_SIZE),
		readl(priv->regs + SDIO_CR),
		readl(priv->regs + SDIO_RX_CFG));
	printf("cmd_op:%x,cmdargs:%x, datasetup:%x\n",
		readl(priv->regs + SDIO_CMD_OP),
		readl(priv->regs + SDIO_CMD_ARG),
		readl(priv->regs + SDIO_DATA_SETUP));
#endif
}

//#define NUCLEI_SDIO_DMA_MAXLEN		(0x100000-0x8000)
//#define NUCLEI_SDIO_DMA_MAXBLOCKS		1984
#define NUCLEI_SDIO_DMA_MAXLEN			0x80000
#define NUCLEI_SDIO_DMA_MAXBLOCKS		1024
//#define NUCLEI_SDIO_DMA_MAXLEN		0x10000
//#define NUCLEI_SDIO_DMA_MAXBLOCKS		128
//#define NUCLEI_SDIO_DMA_MAXLEN		0x8000
//#define NUCLEI_SDIO_DMA_MAXBLOCKS		64

static int nuclei_mmc_setup_dma(struct nuclei_mmc_priv *priv, struct mmc_data *data)
{
	int sz = data->blocks * data->blocksize;
	uint8_t *buf;
	uint32_t reg;
	dma_addr_t dma_addr;
	uint32_t val;

	/* setup data */
	val = SDIO_DATA_SETUP_EN;
	if (data->flags & MMC_DATA_READ)
		val |= SDIO_DATA_SETUP_RD;

	val |= (priv->flags & NUCLEI_MMC_BUS_WIDTH_MASK) << 2;
	if (sz > NUCLEI_SDIO_DMA_MAXLEN) {
		val |= ((NUCLEI_SDIO_DMA_MAXBLOCKS-1) << 4) & SDIO_DATA_SETUP_BLK_NUM;
	} else {
		val |= ((data->blocks-1) << 4) & SDIO_DATA_SETUP_BLK_NUM;
	}

	val |= ((data->blocksize-1) << 20) & SDIO_DATA_SETUP_BLK_SIZE;
	writel(val, priv->regs + SDIO_DATA_SETUP);

	/* save blocks number */
	priv->dma_remain_blocks = data->blocks;

	/* Enable DMA Mode*/
	reg = readl(priv->regs + SDIO_CR);
	reg |= SDIO_CR_DMA_EN;
	writel(reg, priv->regs + SDIO_CR);

	//printf("len:0x%x,ds:0x%x\n",sz, val);
	if (data->flags & MMC_DATA_READ){
		buf = data->dest;
		dma_addr = dma_map_single(buf, sz, DMA_FROM_DEVICE);

		/* Config RX DMA */
		writel(dma_addr, priv->regs + SDIO_RX_SADDR);
		writel((sz > NUCLEI_SDIO_DMA_MAXLEN) ? NUCLEI_SDIO_DMA_MAXLEN : sz,
			priv->regs + SDIO_RX_SIZE);

		/* Start RX DMA */
		reg = readl(priv->regs + SDIO_RX_CFG);
		reg &= ~SDIO_DMA_DATASIZE_MASK;
		reg |= SDIO_DMA_DATASIZE_WORD | SDIO_DMA_TX_RX_EN;
		writel(reg, priv->regs + SDIO_RX_CFG);
	} else if (data->flags & MMC_DATA_WRITE) {

	}
}

static int nuclei_mmc_wait_dma(struct nuclei_mmc_priv *priv, struct mmc_data *data)
{
	uint32_t reg;
	uint8_t *buf;
	uint32_t sz = data->blocks * data->blocksize;
	uint32_t val;

	if (data->flags & MMC_DATA_READ){
		buf = data->dest;
check_finished:
		while(!(readl(priv->regs + SDIO_DMA_INTR_STAT) & SDIO_DMA_INT_STAT_RX_FTRANS));
		writel(SDIO_DMA_INT_CLR_RX_FTRANS, priv->regs + SDIO_DMA_INTR_CLR);
		dump_data(buf, (sz > NUCLEI_SDIO_DMA_MAXLEN) ? NUCLEI_SDIO_DMA_MAXLEN : sz);
		if (sz > NUCLEI_SDIO_DMA_MAXLEN) {
		/* send cmd12 */
			{
				//printf("cmd12\n");
				reg = readl(priv->regs + SDIO_CMD_OP);
				val = MMC_CMD_STOP_TRANSMISSION << 8;
				val |= MMC_RSP_R1b & 0xf;
				writel(val, priv->regs + SDIO_CMD_OP);
				writel(0, priv->regs + SDIO_CMD_ARG);
				writel(0, priv->regs + SDIO_DATA_SETUP);
				writel(1, priv->regs + SDIO_START);
				nuclei_mmc_wait_for_completion(priv);
				//printf("sta:%x\n", readl(priv->regs + SDIO_STATUS));
				writel(reg, priv->regs + SDIO_CMD_OP);
			}

			sz -= NUCLEI_SDIO_DMA_MAXLEN;
			priv->dma_remain_blocks -= NUCLEI_SDIO_DMA_MAXBLOCKS;

			val = SDIO_DATA_SETUP_EN;
			if (data->flags & MMC_DATA_READ)
				val |= SDIO_DATA_SETUP_RD;

			val |= (priv->flags & NUCLEI_MMC_BUS_WIDTH_MASK) << 2;
			if (sz > NUCLEI_SDIO_DMA_MAXLEN)
				val |= ((NUCLEI_SDIO_DMA_MAXBLOCKS - 1) << 4) & SDIO_DATA_SETUP_BLK_NUM;
			else
				val |= ((priv->dma_remain_blocks - 1) << 4) & SDIO_DATA_SETUP_BLK_NUM;

			val |= ((data->blocksize-1) << 20) & SDIO_DATA_SETUP_BLK_SIZE;
			writel(val, priv->regs + SDIO_DATA_SETUP);

			buf += NUCLEI_SDIO_DMA_MAXLEN;
			/* Config RX DMA */
			writel(buf, priv->regs + SDIO_RX_SADDR);

			writel((sz > NUCLEI_SDIO_DMA_MAXLEN) ? NUCLEI_SDIO_DMA_MAXLEN : sz,
				priv->regs + SDIO_RX_SIZE);

			/* Start RX DMA */
			reg = readl(priv->regs + SDIO_RX_CFG);
			reg &= ~SDIO_DMA_DATASIZE_MASK;
			reg |= SDIO_DMA_DATASIZE_WORD | SDIO_DMA_TX_RX_EN;
			writel(reg, priv->regs + SDIO_RX_CFG);

			/* update block position */
			writel(priv->dma_cmdarg + NUCLEI_SDIO_DMA_MAXBLOCKS, priv->regs + SDIO_CMD_ARG);
			priv->dma_cmdarg += NUCLEI_SDIO_DMA_MAXBLOCKS;
			dump_reg(priv);
			/* start transmission */
			writel(1, priv->regs + SDIO_START);

			nuclei_mmc_wait_for_completion(priv);
			goto check_finished;
		} else {
			/* Disable DMA */
			reg = readl(priv->regs + SDIO_CR);
			reg &= ~SDIO_CR_DMA_EN;
			writel(reg, priv->regs + SDIO_CR);

			writel(0, priv->regs + SDIO_DATA_SETUP);
		}
	} else if (data->flags & MMC_DATA_WRITE) {

	}

	return 0;
}
#endif

static int nuclei_mmc_send_cmd(struct mmc *mmc, struct nuclei_mmc_priv *priv,
			   struct mmc_cmd *cmd, struct mmc_data *data)
{
	u32 stat, mask, cmdat = 0;
	int i, ret=0;
	ulong start = get_timer(0);
	ulong timeout = 50000;
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
	//printf("cmd op:%x,args:%x\n", val, cmd->cmdarg);
	if (data) {
#if defined(CONFIG_NUCLEI_MMC_PIO)
		nuclei_mmc_setup_pio(priv, data);
#else
		priv->dma_cmdarg = cmd->cmdarg;
		nuclei_mmc_setup_dma(priv, data);
		dump_reg(priv);
#endif
	} else {
		writel(0, priv->regs + SDIO_DATA_SETUP);
#if !defined(CONFIG_NUCLEI_MMC_PIO)
		/* Disable DMA Mode*/
		val = readl(priv->regs + SDIO_CR);
		val &= ~SDIO_CR_DMA_EN;
		writel(val, priv->regs + SDIO_CR);
#endif
	}

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
		}
	}

	if (data) 
#if defined(CONFIG_NUCLEI_MMC_PIO)
		ret = nuclei_mmc_wait_pio(priv, data);
#else
		ret = nuclei_mmc_wait_dma(priv, data);
#endif

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
	ret |= 1<<8 ;
	writel(ret, NUCLEI_MISC_BASE + 0x20);

	ret = readl(NUCLEI_MISC_BASE + 0x20);
	ret &= ~(1<<9);
	writel(ret, NUCLEI_MISC_BASE + 0x20);
	ret |= 1<<9 ;
	writel(ret, NUCLEI_MISC_BASE + 0x20);

	/* disable interrupt */
	writel(0, priv->regs + SDIO_IE);

	/* Maximum timeouts */
	writel(0xffffffff, priv->regs + SDIO_DATA_TIMEOUT_CNT);

#if !defined(CONFIG_NUCLEI_MMC_PIO)
	writel(SDIO_DMA_INT_EN_RX_FTRANS, priv->regs + SDIO_DMA_INTR_EN);
	//ret = readl(priv->regs + SDIO_CR);
	//ret &= ~BIT(5);
	//writel(ret, priv->regs + SDIO_CR);
#endif

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
