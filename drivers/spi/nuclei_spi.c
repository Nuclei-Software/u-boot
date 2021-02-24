// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2021 Nuclei, Inc.
 * Copyright 2018 SiFive, Inc.
 * Copyright 2019 Bhargav Shah <bhargavshah1988@gmail.com>
 *
 * Nuclei SPI controller driver (master mode only)
 */

#include <common.h>
#include <dm.h>
#include <dm/device_compat.h>
#include <malloc.h>
#include <spi.h>
#include <spi-mem.h>
#include <wait_bit.h>
#include <asm/io.h>
#include <linux/bitops.h>
#include <linux/log2.h>
#include <clk.h>

#define NUCLEI_SPI_MAX_CS		32

#define NUCLEI_SPI_DEFAULT_DEPTH	8
#define NUCLEI_SPI_DEFAULT_BITS		8

/* register offsets */
#define NUCLEI_SPI_REG_SCKDIV            0x00 /* Serial clock divisor */
#define NUCLEI_SPI_REG_SCKMODE           0x04 /* Serial clock mode */
#define NUCLEI_SPI_REG_CSID              0x10 /* Chip select ID */
#define NUCLEI_SPI_REG_CSDEF             0x14 /* Chip select default */
#define NUCLEI_SPI_REG_CSMODE            0x18 /* Chip select mode */
#define NUCLEI_SPI_REG_DELAY0            0x28 /* Delay control 0 */
#define NUCLEI_SPI_REG_DELAY1            0x2c /* Delay control 1 */
#define NUCLEI_SPI_REG_FMT               0x40 /* Frame format */
#define NUCLEI_SPI_REG_TXDATA            0x48 /* Tx FIFO data */
#define NUCLEI_SPI_REG_RXDATA            0x4c /* Rx FIFO data */
#define NUCLEI_SPI_REG_TXMARK            0x50 /* Tx FIFO watermark */
#define NUCLEI_SPI_REG_RXMARK            0x54 /* Rx FIFO watermark */
#define NUCLEI_SPI_REG_FCTRL             0x60 /* SPI flash interface control */
#define NUCLEI_SPI_REG_FFMT              0x64 /* SPI flash instruction format */
#define NUCLEI_SPI_REG_IE                0x70 /* Interrupt Enable Register */
#define NUCLEI_SPI_REG_IP                0x74 /* Interrupt Pendings Register */

/* sckdiv bits */
#define NUCLEI_SPI_SCKDIV_DIV_MASK       0xfffU

/* sckmode bits */
#define NUCLEI_SPI_SCKMODE_PHA           BIT(0)
#define NUCLEI_SPI_SCKMODE_POL           BIT(1)
#define NUCLEI_SPI_SCKMODE_MODE_MASK     (NUCLEI_SPI_SCKMODE_PHA | \
                                          NUCLEI_SPI_SCKMODE_POL)

/* csmode bits */
#define NUCLEI_SPI_CSMODE_MODE_AUTO      0U
#define NUCLEI_SPI_CSMODE_MODE_HOLD      2U
#define NUCLEI_SPI_CSMODE_MODE_OFF       3U

/* delay0 bits */
#define NUCLEI_SPI_DELAY0_CSSCK(x)       ((u32)(x))
#define NUCLEI_SPI_DELAY0_CSSCK_MASK     0xffU
#define NUCLEI_SPI_DELAY0_SCKCS(x)       ((u32)(x) << 16)
#define NUCLEI_SPI_DELAY0_SCKCS_MASK     (0xffU << 16)

/* delay1 bits */
#define NUCLEI_SPI_DELAY1_INTERCS(x)     ((u32)(x))
#define NUCLEI_SPI_DELAY1_INTERCS_MASK   0xffU
#define NUCLEI_SPI_DELAY1_INTERXFR(x)    ((u32)(x) << 16)
#define NUCLEI_SPI_DELAY1_INTERXFR_MASK  (0xffU << 16)

/* fmt bits */
#define NUCLEI_SPI_FMT_PROTO_SINGLE      0U
#define NUCLEI_SPI_FMT_PROTO_DUAL        1U
#define NUCLEI_SPI_FMT_PROTO_QUAD        2U
#define NUCLEI_SPI_FMT_PROTO_MASK        3U
#define NUCLEI_SPI_FMT_ENDIAN            BIT(2)
#define NUCLEI_SPI_FMT_DIR               BIT(3)
#define NUCLEI_SPI_FMT_LEN(x)            ((u32)(x) << 16)
#define NUCLEI_SPI_FMT_LEN_MASK          (0xfU << 16)

/* txdata bits */
#define NUCLEI_SPI_TXDATA_DATA_MASK      0xffU
#define NUCLEI_SPI_TXDATA_FULL           BIT(31)

/* rxdata bits */
#define NUCLEI_SPI_RXDATA_DATA_MASK      0xffU
#define NUCLEI_SPI_RXDATA_EMPTY          BIT(31)

/* ie and ip bits */
#define NUCLEI_SPI_IP_TXWM               BIT(0)
#define NUCLEI_SPI_IP_RXWM               BIT(1)

/* format protocol */
#define NUCLEI_SPI_PROTO_QUAD		4 /* 4 lines I/O protocol transfer */
#define NUCLEI_SPI_PROTO_DUAL		2 /* 2 lines I/O protocol transfer */
#define NUCLEI_SPI_PROTO_SINGLE		1 /* 1 line I/O protocol transfer */

struct nuclei_spi {
	void		*regs;		/* base address of the registers */
	u32		fifo_depth;
	u32		bits_per_word;
	u32		cs_inactive;	/* Level of the CS pins when inactive*/
	u32		freq;
	u32		num_cs;
	u8		fmt_proto;
};

static void nuclei_spi_prep_device(struct nuclei_spi *spi,
				   struct dm_spi_slave_platdata *slave_plat)
{
	/* Update the chip select polarity */
	if (slave_plat->mode & SPI_CS_HIGH)
		spi->cs_inactive &= ~BIT(slave_plat->cs);
	else
		spi->cs_inactive |= BIT(slave_plat->cs);
	writel(spi->cs_inactive, spi->regs + NUCLEI_SPI_REG_CSDEF);

	/* Select the correct device */
	writel(slave_plat->cs, spi->regs + NUCLEI_SPI_REG_CSID);
}

static int nuclei_spi_set_cs(struct nuclei_spi *spi,
			     struct dm_spi_slave_platdata *slave_plat)
{
	u32 cs_mode = NUCLEI_SPI_CSMODE_MODE_HOLD;

	if (slave_plat->mode & SPI_CS_HIGH)
		cs_mode = NUCLEI_SPI_CSMODE_MODE_AUTO;

	writel(cs_mode, spi->regs + NUCLEI_SPI_REG_CSMODE);

	return 0;
}

static void nuclei_spi_clear_cs(struct nuclei_spi *spi)
{
	writel(NUCLEI_SPI_CSMODE_MODE_AUTO, spi->regs + NUCLEI_SPI_REG_CSMODE);
}

static void nuclei_spi_prep_transfer(struct nuclei_spi *spi,
				     struct dm_spi_slave_platdata *slave_plat,
				     u8 *rx_ptr)
{
	u32 cr;

	/* Modify the SPI protocol mode */
	cr = readl(spi->regs + NUCLEI_SPI_REG_FMT);

	/* Bits per word ? */
	cr &= ~NUCLEI_SPI_FMT_LEN_MASK;
	cr |= NUCLEI_SPI_FMT_LEN(spi->bits_per_word);

	/* LSB first? */
	cr &= ~NUCLEI_SPI_FMT_ENDIAN;
	if (slave_plat->mode & SPI_LSB_FIRST)
		cr |= NUCLEI_SPI_FMT_ENDIAN;

	/* Number of wires ? */
	cr &= ~NUCLEI_SPI_FMT_PROTO_MASK;
	switch (spi->fmt_proto) {
	case NUCLEI_SPI_PROTO_QUAD:
		cr |= NUCLEI_SPI_FMT_PROTO_QUAD;
		break;
	case NUCLEI_SPI_PROTO_DUAL:
		cr |= NUCLEI_SPI_FMT_PROTO_DUAL;
		break;
	default:
		cr |= NUCLEI_SPI_FMT_PROTO_SINGLE;
		break;
	}

	/* SPI direction in/out ? */
	cr &= ~NUCLEI_SPI_FMT_DIR;
	if (!rx_ptr)
		cr |= NUCLEI_SPI_FMT_DIR;

	writel(cr, spi->regs + NUCLEI_SPI_REG_FMT);
}

static void nuclei_spi_rx(struct nuclei_spi *spi, u8 *rx_ptr)
{
	u32 data;

	do {
		data = readl(spi->regs + NUCLEI_SPI_REG_RXDATA);
	} while (data & NUCLEI_SPI_RXDATA_EMPTY);

	if (rx_ptr)
		*rx_ptr = data & NUCLEI_SPI_RXDATA_DATA_MASK;
}

static void nuclei_spi_tx(struct nuclei_spi *spi, const u8 *tx_ptr)
{
	u32 data;
	u8 tx_data = (tx_ptr) ? *tx_ptr & NUCLEI_SPI_TXDATA_DATA_MASK :
				NUCLEI_SPI_TXDATA_DATA_MASK;

	do {
		data = readl(spi->regs + NUCLEI_SPI_REG_TXDATA);
	} while (data & NUCLEI_SPI_TXDATA_FULL);

	writel(tx_data, spi->regs + NUCLEI_SPI_REG_TXDATA);
}

static int nuclei_spi_wait(struct nuclei_spi *spi, u32 bit)
{
	return wait_for_bit_le32(spi->regs + NUCLEI_SPI_REG_IP,
				 bit, true, 100, false);
}

static int nuclei_spi_xfer(struct udevice *dev, unsigned int bitlen,
			   const void *dout, void *din, unsigned long flags)
{
	struct udevice *bus = dev->parent;
	struct nuclei_spi *spi = dev_get_priv(bus);
	struct dm_spi_slave_platdata *slave_plat = dev_get_parent_platdata(dev);
	const u8 *tx_ptr = dout;
	u8 *rx_ptr = din;
	u32 remaining_len;
	int ret;

	if (flags & SPI_XFER_BEGIN) {
		nuclei_spi_prep_device(spi, slave_plat);

		ret = nuclei_spi_set_cs(spi, slave_plat);
		if (ret)
			return ret;
	}

	nuclei_spi_prep_transfer(spi, slave_plat, rx_ptr);

	remaining_len = bitlen / 8;

	while (remaining_len) {
		unsigned int n_words = min(remaining_len, spi->fifo_depth);
		unsigned int tx_words, rx_words;

		/* Enqueue n_words for transmission */
		for (tx_words = 0; tx_words < n_words; tx_words++) {
			if (!tx_ptr)
				nuclei_spi_tx(spi, NULL);
			else
				nuclei_spi_tx(spi, tx_ptr++);
		}

		if (rx_ptr) {
			/* Wait for transmission + reception to complete */
			writel(n_words - 1, spi->regs + NUCLEI_SPI_REG_RXMARK);
			ret = nuclei_spi_wait(spi, NUCLEI_SPI_IP_RXWM);
			if (ret)
				return ret;

			/* Read out all the data from the RX FIFO */
			for (rx_words = 0; rx_words < n_words; rx_words++)
				nuclei_spi_rx(spi, rx_ptr++);
		} else {
			/* Wait for transmission to complete */
			ret = nuclei_spi_wait(spi, NUCLEI_SPI_IP_TXWM);
			if (ret)
				return ret;
		}

		remaining_len -= n_words;
	}

	if (flags & SPI_XFER_END)
		nuclei_spi_clear_cs(spi);

	return 0;
}

static int nuclei_spi_exec_op(struct spi_slave *slave,
			      const struct spi_mem_op *op)
{
	struct udevice *dev = slave->dev;
	struct nuclei_spi *spi = dev_get_priv(dev->parent);
	unsigned long flags = SPI_XFER_BEGIN;
	u8 opcode = op->cmd.opcode;
	unsigned int pos = 0;
	const void *tx_buf = NULL;
	void *rx_buf = NULL;
	int op_len, i;
	int ret;

	if (!op->addr.nbytes && !op->dummy.nbytes && !op->data.nbytes)
		flags |= SPI_XFER_END;

	spi->fmt_proto = op->cmd.buswidth;

	/* send the opcode */
	ret = nuclei_spi_xfer(dev, 8, (void *)&opcode, NULL, flags);
	if (ret < 0) {
		dev_err(dev, "failed to xfer opcode\n");
		return ret;
	}

	op_len = op->addr.nbytes + op->dummy.nbytes;
	u8 op_buf[op_len];

	/* send the addr + dummy */
	if (op->addr.nbytes) {
		/* fill address */
		for (i = 0; i < op->addr.nbytes; i++)
			op_buf[pos + i] = op->addr.val >>
				(8 * (op->addr.nbytes - i - 1));

		pos += op->addr.nbytes;

		/* fill dummy */
		if (op->dummy.nbytes)
			memset(op_buf + pos, 0xff, op->dummy.nbytes);

		/* make sure to set end flag, if no data bytes */
		if (!op->data.nbytes)
			flags |= SPI_XFER_END;

		spi->fmt_proto = op->addr.buswidth;

		ret = nuclei_spi_xfer(dev, op_len * 8, op_buf, NULL, flags);
		if (ret < 0) {
			dev_err(dev, "failed to xfer addr + dummy\n");
			return ret;
		}
	}

	/* send/received the data */
	if (op->data.nbytes) {
		if (op->data.dir == SPI_MEM_DATA_IN)
			rx_buf = op->data.buf.in;
		else
			tx_buf = op->data.buf.out;

		spi->fmt_proto = op->data.buswidth;

		ret = nuclei_spi_xfer(dev, op->data.nbytes * 8,
				      tx_buf, rx_buf, SPI_XFER_END);
		if (ret) {
			dev_err(dev, "failed to xfer data\n");
			return ret;
		}
	}

	return 0;
}

static int nuclei_spi_set_speed(struct udevice *bus, uint speed)
{
	struct nuclei_spi *spi = dev_get_priv(bus);
	u32 scale;

	if (speed > spi->freq)
		speed = spi->freq;

	/* Cofigure max speed */
	scale = (DIV_ROUND_UP(spi->freq >> 1, speed) - 1)
					& NUCLEI_SPI_SCKDIV_DIV_MASK;
	writel(scale, spi->regs + NUCLEI_SPI_REG_SCKDIV);

	return 0;
}

static int nuclei_spi_set_mode(struct udevice *bus, uint mode)
{
	struct nuclei_spi *spi = dev_get_priv(bus);
	u32 cr;

	/* Switch clock mode bits */
	cr = readl(spi->regs + NUCLEI_SPI_REG_SCKMODE) &
				~NUCLEI_SPI_SCKMODE_MODE_MASK;
	if (mode & SPI_CPHA)
		cr |= NUCLEI_SPI_SCKMODE_PHA;
	if (mode & SPI_CPOL)
		cr |= NUCLEI_SPI_SCKMODE_POL;

	writel(cr, spi->regs + NUCLEI_SPI_REG_SCKMODE);

	return 0;
}

static int nuclei_spi_cs_info(struct udevice *bus, uint cs,
			      struct spi_cs_info *info)
{
	struct nuclei_spi *spi = dev_get_priv(bus);

	if (cs >= spi->num_cs)
		return -EINVAL;

	return 0;
}

static void nuclei_spi_init_hw(struct nuclei_spi *spi)
{
	u32 cs_bits;

	/* probe the number of CS lines */
	spi->cs_inactive = readl(spi->regs + NUCLEI_SPI_REG_CSDEF);
	writel(0xffffffffU, spi->regs + NUCLEI_SPI_REG_CSDEF);
	cs_bits = readl(spi->regs + NUCLEI_SPI_REG_CSDEF);
	writel(spi->cs_inactive, spi->regs + NUCLEI_SPI_REG_CSDEF);
	if (!cs_bits) {
		printf("Could not auto probe CS lines\n");
		return;
	}

	spi->num_cs = ilog2(cs_bits) + 1;
	if (spi->num_cs > NUCLEI_SPI_MAX_CS) {
		printf("Invalid number of spi slaves\n");
		return;
	}

	/* Watermark interrupts are disabled by default */
	writel(0, spi->regs + NUCLEI_SPI_REG_IE);

	/* Default watermark FIFO threshold values */
	writel(1, spi->regs + NUCLEI_SPI_REG_TXMARK);
	writel(0, spi->regs + NUCLEI_SPI_REG_RXMARK);

	/* Set CS/SCK Delays and Inactive Time to defaults */
	writel(NUCLEI_SPI_DELAY0_CSSCK(1) | NUCLEI_SPI_DELAY0_SCKCS(1),
	       spi->regs + NUCLEI_SPI_REG_DELAY0);
	writel(NUCLEI_SPI_DELAY1_INTERCS(1) | NUCLEI_SPI_DELAY1_INTERXFR(0),
	       spi->regs + NUCLEI_SPI_REG_DELAY1);

	/* Exit specialized memory-mapped SPI flash mode */
	writel(0, spi->regs + NUCLEI_SPI_REG_FCTRL);
}

static int nuclei_spi_probe(struct udevice *bus)
{
	struct nuclei_spi *spi = dev_get_priv(bus);
	struct clk clkdev;
	int ret;

	spi->regs = (void *)(ulong)dev_remap_addr(bus);
	if (!spi->regs)
		return -ENODEV;

	spi->fifo_depth = dev_read_u32_default(bus,
					       "nuclei,fifo-depth",
					       NUCLEI_SPI_DEFAULT_DEPTH);

	spi->bits_per_word = dev_read_u32_default(bus,
						  "nuclei,max-bits-per-word",
						  NUCLEI_SPI_DEFAULT_BITS);

	ret = clk_get_by_index(bus, 0, &clkdev);
	if (ret)
		return ret;
	spi->freq = clk_get_rate(&clkdev);

	/* init the nuclei spi hw */
	nuclei_spi_init_hw(spi);

	return 0;
}

static const struct spi_controller_mem_ops nuclei_spi_mem_ops = {
	.exec_op	= nuclei_spi_exec_op,
};

static const struct dm_spi_ops nuclei_spi_ops = {
	.xfer		= nuclei_spi_xfer,
	.set_speed	= nuclei_spi_set_speed,
	.set_mode	= nuclei_spi_set_mode,
	.cs_info	= nuclei_spi_cs_info,
	.mem_ops	= &nuclei_spi_mem_ops,
};

static const struct udevice_id nuclei_spi_ids[] = {
	{ .compatible = "nuclei,spi0" },
	{ }
};

U_BOOT_DRIVER(nuclei_spi) = {
	.name	= "nuclei_spi",
	.id	= UCLASS_SPI,
	.of_match = nuclei_spi_ids,
	.ops	= &nuclei_spi_ops,
	.priv_auto_alloc_size = sizeof(struct nuclei_spi),
	.probe	= nuclei_spi_probe,
};
