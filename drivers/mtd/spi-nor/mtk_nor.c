/*
 * Copyright (c) 2015 MediaTek Inc.
 * Author: Bayi.Cheng <bayi.cheng@mediatek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/ioport.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mtd/mtd.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/spi-nor.h>

#define MTK_NOR_CMD_REG			0x00
#define MTK_NOR_CNT_REG			0x04
#define MTK_NOR_RDSR_REG		0x08
#define MTK_NOR_RDATA_REG		0x0c
#define MTK_NOR_RADR0_REG		0x10
#define MTK_NOR_RADR1_REG		0x14
#define MTK_NOR_RADR2_REG		0x18
#define MTK_NOR_WDATA_REG		0x1c
#define MTK_NOR_PRGDATA0_REG		0x20
#define MTK_NOR_PRGDATA1_REG		0x24
#define MTK_NOR_PRGDATA2_REG		0x28
#define MTK_NOR_PRGDATA3_REG		0x2c
#define MTK_NOR_PRGDATA4_REG		0x30
#define MTK_NOR_PRGDATA5_REG		0x34
#define MTK_NOR_SHREG0_REG		0x38
#define MTK_NOR_SHREG1_REG		0x3c
#define MTK_NOR_SHREG2_REG		0x40
#define MTK_NOR_SHREG3_REG		0x44
#define MTK_NOR_SHREG4_REG		0x48
#define MTK_NOR_SHREG5_REG		0x4c
#define MTK_NOR_SHREG6_REG		0x50
#define MTK_NOR_SHREG7_REG		0x54
#define MTK_NOR_SHREG8_REG		0x58
#define MTK_NOR_SHREG9_REG		0x5c
#define MTK_NOR_FLHCFG_REG		0x84
#define MTK_NOR_PP_DATA_REG		0x98
#define MTK_NOR_PREBUF_STUS_REG		0x9c
#define MTK_NOR_INTRSTUS_REG		0xa8
#define MTK_NOR_INTREN_REG		0xac
#define MTK_NOR_TIME_REG		0x94
#define MTK_NOR_CHKSUM_CTL_REG		0xb8
#define MTK_NOR_CHKSUM_REG		0xbc
#define MTK_NOR_CMD2_REG		0xc0
#define MTK_NOR_WRPROT_REG		0xc4
#define MTK_NOR_RADR3_REG		0xc8
#define MTK_NOR_DUAL_REG		0xcc
#define MTK_NOR_DELSEL0_REG		0xa0
#define MTK_NOR_DELSEL1_REG		0xa4
#define MTK_NOR_DELSEL2_REG		0xd0
#define MTK_NOR_DELSEL3_REG		0xd4
#define MTK_NOR_DELSEL4_REG		0xd8
#define MTK_NOR_CFG1_REG		0x60
#define MTK_NOR_CFG2_REG		0x64
#define MTK_NOR_CFG3_REG		0x68
#define MTK_NOR_STATUS0_REG		0x70
#define MTK_NOR_STATUS1_REG		0x74
#define MTK_NOR_STATUS2_REG		0x78
#define MTK_NOR_STATUS3_REG		0x7c
/* commands for mtk nor controller */
#define MTK_NOR_READ_CMD		0x0
#define MTK_NOR_RDSR_CMD		0x2
#define MTK_NOR_PRG_CMD			0x4
#define MTK_NOR_WR_CMD			0x10
#define MTK_NOR_WRSR_CMD		0x20
#define MTK_NOR_PIO_READ_CMD		0x81
#define MTK_NOR_WR_BUF_ENABLE		0x1
#define MTK_NOR_WR_BUF_DISABLE		0x0
#define MTK_NOR_ENABLE_SF_CMD		0x30
#define MTK_NOR_DUAD_ADDR_EN		0x8
#define MTK_NOR_QUAD_READ_EN		0x4
#define MTK_NOR_DUAL_ADDR_EN		0x2
#define MTK_NOR_DUAL_READ_EN		0x1
#define MTK_NOR_DUAL_DISABLE		0x0
#define MTK_NOR_FAST_READ			0x1

#define SFLASH_WRBUF_SIZE		128
#define MAX_FLASHCOUNT			1
#define SFLASHHWNAME_LEN		12
#define SFLASH_MAX_DMA_SIZE		(1024 * 8)

#define LoWord(d)		((u16)(d & 0x0000ffffL))
#define HiWord(d)		((u16)((d >> 16) & 0x0000ffffL))
#define LoByte(w)		((u8)(w & 0x00ff))
#define HiByte(w)		((u8)(((w) >> 8) & 0x00ff))
#define LOCAL_BUF_SIZE		(SFLASH_MAX_DMA_SIZE * 20)

struct mt8173_nor {
	struct spi_nor nor;
	struct device *dev;
	void __iomem *base;	/* nor flash base address */
	struct clk *spi_clk;
	struct clk *src_axi_clk;
	struct clk *sf_mux_clk;
	struct clk *nor_clk;
};

static void mt8173_nor_set_read_mode(struct mt8173_nor *mt8173_nor)
{
	struct spi_nor *nor = &mt8173_nor->nor;

	switch (nor->flash_read) {
	case SPI_NOR_FAST:
		writeb(SPINOR_OP_READ_FAST, mt8173_nor->base +
		       MTK_NOR_PRGDATA3_REG);
		writeb(MTK_NOR_FAST_READ, mt8173_nor->base +
		       MTK_NOR_CFG1_REG);
		break;
	case SPI_NOR_DUAL:
		writeb(SPINOR_OP_READ_1_1_2, mt8173_nor->base +
		       MTK_NOR_PRGDATA3_REG);
		writeb(MTK_NOR_DUAL_READ_EN, mt8173_nor->base +
		       MTK_NOR_DUAL_REG);
		break;
	case SPI_NOR_QUAD:
		writeb(SPINOR_OP_READ_1_1_4, mt8173_nor->base +
		       MTK_NOR_PRGDATA3_REG);
		writeb(MTK_NOR_QUAD_READ_EN, mt8173_nor->base +
		       MTK_NOR_DUAL_REG);
		break;
	default:
		writeb(SPINOR_OP_READ, mt8173_nor->base +
		       MTK_NOR_PRGDATA3_REG);
		writeb(MTK_NOR_DUAL_DISABLE , mt8173_nor->base +
		       MTK_NOR_DUAL_REG);
		break;
	}
}

static int mt8173_nor_polling_reg(struct mt8173_nor *mt8173_nor, int compare)
{
	int reg, ret;

	ret =  readl_poll_timeout(mt8173_nor->base + MTK_NOR_CMD_REG, reg,
				  !(reg & compare), 100, 10000);
	if (ret)
		dev_err(mt8173_nor->dev, "compare val%02X timeout!\n", compare);
	return ret;
}

static int mt8173_nor_execute_cmd(struct mt8173_nor *mt8173_nor, u8 cmdval)
{
	u8 val = cmdval & 0x1f;

	writeb(cmdval, mt8173_nor->base + MTK_NOR_CMD_REG);
	return mt8173_nor_polling_reg(mt8173_nor, val);
}

static int mt8173_nor_set_cmd(struct mt8173_nor *mt8173_nor, int addr,
			      int len, int op)
{
	writeb(op , mt8173_nor->base + MTK_NOR_PRGDATA5_REG);
	/* reset the following register */
	writeb(LoByte(HiWord(addr)) , mt8173_nor->base + MTK_NOR_PRGDATA4_REG);
	writeb(HiByte(LoWord(addr)) , mt8173_nor->base + MTK_NOR_PRGDATA3_REG);
	writeb(LoByte(LoWord(addr)) , mt8173_nor->base + MTK_NOR_PRGDATA2_REG);
	writeb(len , mt8173_nor->base + MTK_NOR_CNT_REG);
	return mt8173_nor_execute_cmd(mt8173_nor, MTK_NOR_PRG_CMD);
}

static int mt8173_nor_get_para(struct mt8173_nor *mt8173_nor, u8 *buf, int len)
{
	if (len > 1) {
		/* read JEDEC ID need 4 bytes commands */
		mt8173_nor_set_cmd(mt8173_nor, 0, 32, SPINOR_OP_RDID);
		buf[2] = readb(mt8173_nor->base + MTK_NOR_SHREG0_REG);
		buf[1] = readb(mt8173_nor->base + MTK_NOR_SHREG1_REG);
		buf[0] = readb(mt8173_nor->base + MTK_NOR_SHREG2_REG);
	} else {
		if (mt8173_nor_execute_cmd(mt8173_nor, MTK_NOR_RDSR_CMD)) {
			dev_err(mt8173_nor->dev, "read status failed!\n");
			return -EINVAL;
		}
		*buf = readb(mt8173_nor->base + MTK_NOR_RDSR_REG);
	}
	return 0;
}

/* cmd1 sent to nor flash, cmd2 write to nor controller */
static int mt8173_nor_set_para(struct mt8173_nor *mt8173_nor, int cmd1,
			       int cmd2)
{
	if (mt8173_nor_set_cmd(mt8173_nor, 0, 8, SPINOR_OP_WREN)) {
		dev_err(mt8173_nor->dev,
			"write enable failed in write protect!\n");
		return -EINVAL;
	}
	writeb(cmd1, mt8173_nor->base + MTK_NOR_PRGDATA5_REG);
	writeb(8, mt8173_nor->base + MTK_NOR_CNT_REG);
	if (mt8173_nor_execute_cmd(mt8173_nor, cmd2)) {
		dev_err(mt8173_nor->dev, "execute cmd failed!\n");
		return -EINVAL;
	}

	return 0;
}

static int mt8173_nor_write_buffer_enable(struct mt8173_nor *mt8173_nor)
{
	u8 reg, ret;

	writel(MTK_NOR_WR_BUF_ENABLE , mt8173_nor->base + MTK_NOR_CFG2_REG);
	ret =  readb_poll_timeout(mt8173_nor->base + MTK_NOR_CFG2_REG, reg,
				  0x01 == (reg & 0x01), 100, 10000);
	if (ret)
		dev_err(mt8173_nor->dev, "timeout!\n");
	return ret;
}

static int mt8173_nor_write_buffer_disable(struct mt8173_nor *mt8173_nor)
{
	u8 reg, ret;

	writel(MTK_NOR_WR_BUF_DISABLE, mt8173_nor->base + MTK_NOR_CFG2_REG);
	ret =  readb_poll_timeout(mt8173_nor->base + MTK_NOR_CFG2_REG, reg,
				  MTK_NOR_WR_BUF_DISABLE == (reg & 0xf), 100,
				  10000);
	if (ret)
		dev_err(mt8173_nor->dev, "compare timeout!\n");
	return ret;
}

static int mt8173_nor_erase_sector(struct spi_nor *nor, loff_t offset)
{
	struct mt8173_nor *mt8173_nor = nor->priv;

	if (mt8173_nor_set_cmd(mt8173_nor, 0, 8, SPINOR_OP_WREN)) {
		dev_err(mt8173_nor->dev,
			"write enable failed in erase sector!\n");
		return -EINVAL;
	}

	mt8173_nor_set_cmd(mt8173_nor, (int)offset, 32, SPINOR_OP_BE_4K);
	return 0;
}

static int mt8173_nor_read_hw(struct mt8173_nor *mt8173_nor, int addr, int len,
			      int *retlen, u8 *buf)
{
	int ret = 0, i;

	mt8173_nor_set_read_mode(mt8173_nor);
	writeb(HiByte(HiWord(addr)) , mt8173_nor->base + MTK_NOR_RADR3_REG);
	writeb(LoByte(HiWord(addr)) , mt8173_nor->base + MTK_NOR_RADR2_REG);
	writeb(HiByte(LoWord(addr)) , mt8173_nor->base + MTK_NOR_RADR1_REG);
	writeb(LoByte(LoWord(addr)) , mt8173_nor->base + MTK_NOR_RADR0_REG);

	for (i = 0; i < len; i++, (*retlen)++) {
		if (mt8173_nor_execute_cmd(mt8173_nor, MTK_NOR_PIO_READ_CMD)) {
			dev_err(mt8173_nor->dev, "read flash failed!\n");
			return -EINVAL;
		}
		buf[i] = readb(mt8173_nor->base + MTK_NOR_RDATA_REG);
	}
	return ret;
}

static int mt8173_nor_read(struct spi_nor *nor, loff_t from, size_t len,
			   size_t *retlen, u_char *buf)
{
	struct mt8173_nor *mt8173_nor = nor->priv;

	return mt8173_nor_read_hw(mt8173_nor, (int)from, (int)len,
				 (int *)retlen, (u8 *)buf);
}

static int mt8173_nor_write_single_byte(struct mt8173_nor *mt8173_nor,
					int u4addr, u8 u1data)
{
	if (u4addr >= mt8173_nor->nor.mtd.size) {
		dev_err(mt8173_nor->dev, "invalid write address!\n");
		return -EINVAL;
	}

	writeb(u1data , mt8173_nor->base + MTK_NOR_WDATA_REG);
	writeb(LoByte(HiWord(u4addr)) , mt8173_nor->base + MTK_NOR_RADR2_REG);
	writeb(HiByte(LoWord(u4addr)) , mt8173_nor->base + MTK_NOR_RADR1_REG);
	writeb(LoByte(LoWord(u4addr)) , mt8173_nor->base + MTK_NOR_RADR0_REG);

	if (mt8173_nor_execute_cmd(mt8173_nor, MTK_NOR_WR_CMD)) {
		dev_err(mt8173_nor->dev, "write byte offset%08x\n", u4addr);
		return -EINVAL;
	}
	return 0;
}

static int mt8173_nor_write_buffer(struct mt8173_nor *mt8173_nor, int u4addr,
				   int u4len, const u8 *buf)
{
	int i, j, bufidx, data;

	if (buf == NULL)
		return -EINVAL;

	writeb(LoByte(HiWord(u4addr)) , mt8173_nor->base + MTK_NOR_RADR2_REG);
	writeb(HiByte(LoWord(u4addr)) , mt8173_nor->base + MTK_NOR_RADR1_REG);
	writeb(LoByte(LoWord(u4addr)) , mt8173_nor->base + MTK_NOR_RADR0_REG);

	bufidx = 0;
	for (i = 0; i < u4len; i += 4) {
		for (j = 0; j < 4; j++) {
			(*((u8 *)&data + j)) = buf[bufidx];
			bufidx++;
		}
		writel(data, mt8173_nor->base + MTK_NOR_PP_DATA_REG);
	}

	if (mt8173_nor_execute_cmd(mt8173_nor, MTK_NOR_WR_CMD)) {
		dev_err(mt8173_nor->dev, "write buffer offset:%08x\n", u4addr);
		return -EINVAL;
	}
	return 0;
}

static void mt8173_nor_write(struct spi_nor *nor, loff_t to, size_t len,
			     size_t *retlen, const u_char *buf)
{
	struct mt8173_nor *mt8173_nor = nor->priv;
	size_t i;

	if (nor->page_size == len) {
		if (mt8173_nor_write_buffer_enable(mt8173_nor))
			dev_err(mt8173_nor->dev, "write buffer enable failed !\n");
		while (len > 0) {
			mt8173_nor_write_buffer(mt8173_nor, to,
						SFLASH_WRBUF_SIZE, buf);
			len -= SFLASH_WRBUF_SIZE;
			to += SFLASH_WRBUF_SIZE;
			buf += SFLASH_WRBUF_SIZE;
			(*retlen) += SFLASH_WRBUF_SIZE;
		}
		if (mt8173_nor_write_buffer_disable(mt8173_nor))
			dev_err(mt8173_nor->dev, "write buffer disable failed !\n");
	} else {
		for (i = 0; i < len; i++,  (*retlen)++) {
			if (mt8173_nor_write_single_byte(mt8173_nor, to, *buf))
				dev_err(mt8173_nor->dev, "write byte failed !\n");
			to++;
			buf++;
		}
	}
}

static int mt8173_nor_read_reg(struct spi_nor *nor, u8 opcode, u8 *buf, int len)
{
	int ret = 0;
	struct mt8173_nor *mt8173_nor = nor->priv;
	/* mtk nor controller haven't supoort SPINOR_OP_RDCR */
	if (opcode == SPINOR_OP_RDID || opcode == SPINOR_OP_RDSR)
		ret = mt8173_nor_get_para(mt8173_nor, buf, len);
	else
		dev_warn(mt8173_nor->dev, "invalid cmd %d\n", opcode);

	return ret;
}

static int mt8173_nor_write_reg(struct spi_nor *nor, u8 opcode, u8 *buf,
				int len, int write_enable)
{
	int ret, cmd_to_nor, cmd_to_controller;
	struct mt8173_nor *mt8173_nor = nor->priv;

	if (opcode == SPINOR_OP_WRSR || opcode == SPINOR_OP_CHIP_ERASE) {
		if (len > 0) {
			cmd_to_nor = *buf;
			cmd_to_controller = MTK_NOR_WRSR_CMD;
		} else {
			cmd_to_nor = opcode;
			cmd_to_controller = MTK_NOR_PRG_CMD;
		}
		ret = mt8173_nor_set_para(mt8173_nor, cmd_to_nor,
					  cmd_to_controller);
	} else if (opcode == SPINOR_OP_WREN || opcode == SPINOR_OP_WRDI) {
		ret = mt8173_nor_set_cmd(mt8173_nor, 0, 8, opcode);
	} else {
		dev_warn(mt8173_nor->dev, "have not support cmd %d\n", opcode);
		ret = -EINVAL;
	}
	return ret;
}

static int __init mtk_nor_init(struct mt8173_nor *mt8173_nor,
			       struct mtd_part_parser_data ppdata)
{
	int ret = -ENODEV;
	struct spi_nor *nor;
	struct mtd_info *mtd;

	writel(MTK_NOR_ENABLE_SF_CMD, mt8173_nor->base + MTK_NOR_WRPROT_REG);
	nor = &mt8173_nor->nor;
	mtd = &nor->mtd;
	nor->dev = mt8173_nor->dev;
	nor->priv = mt8173_nor;

	/* fill the hooks to spi nor */
	nor->read = mt8173_nor_read;
	nor->read_reg = mt8173_nor_read_reg;
	nor->write = mt8173_nor_write;
	nor->write_reg = mt8173_nor_write_reg;
	nor->erase = mt8173_nor_erase_sector;
	mtd->owner = THIS_MODULE;
	mtd->name = "mtk_nor";
	/* initialized with NULL */
	ret = spi_nor_scan(nor, NULL, SPI_NOR_DUAL);
	if (ret) {
		dev_err(mt8173_nor->dev, "spi_nor_scan failed !\n");
		return -EINVAL;
	}
	dev_dbg(mt8173_nor->dev, "mtd->size :0x%llx!\n", mtd->size);
	ret = mtd_device_parse_register(mtd, NULL, &ppdata, NULL, 0);

	return ret;
}

static int mtk_nor_drv_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct mtd_part_parser_data ppdata;
	struct resource *res;
	int ret;
	struct mt8173_nor *mt8173_nor = devm_kzalloc(&pdev->dev,
		sizeof(*mt8173_nor), GFP_KERNEL);

	if (!pdev->dev.of_node) {
		dev_err(&pdev->dev, "No DT found\n");
		return -EINVAL;
	}

	if (!mt8173_nor)
		return -ENOMEM;
	platform_set_drvdata(pdev, mt8173_nor);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	mt8173_nor->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(mt8173_nor->base)) {
		ret = PTR_ERR(mt8173_nor->base);
		goto nor_free;
	}

	mt8173_nor->spi_clk = devm_clk_get(&pdev->dev, "spi_clk");
	if (IS_ERR(mt8173_nor->spi_clk)) {
		ret = PTR_ERR(mt8173_nor->spi_clk);
		goto nor_free;
	}

	mt8173_nor->src_axi_clk = devm_clk_get(&pdev->dev, "axi_clk");
	if (IS_ERR(mt8173_nor->src_axi_clk)) {
		ret = PTR_ERR(mt8173_nor->src_axi_clk);
		goto nor_free;
	}

	mt8173_nor->sf_mux_clk = devm_clk_get(&pdev->dev, "mux_clk");
	if (IS_ERR(mt8173_nor->sf_mux_clk)) {
		ret = PTR_ERR(mt8173_nor->sf_mux_clk);
		goto nor_free;
	}

	mt8173_nor->nor_clk = devm_clk_get(&pdev->dev, "sf_clk");
	if (IS_ERR(mt8173_nor->nor_clk)) {
		ret = PTR_ERR(mt8173_nor->nor_clk);
		goto nor_free;
	}

	mt8173_nor->dev = &pdev->dev;
	clk_prepare_enable(mt8173_nor->spi_clk);
	clk_prepare_enable(mt8173_nor->src_axi_clk);
	clk_prepare_enable(mt8173_nor->sf_mux_clk);
	clk_prepare_enable(mt8173_nor->nor_clk);
	clk_set_parent(mt8173_nor->nor_clk, mt8173_nor->sf_mux_clk);

	ppdata.of_node = np;
	ret = mtk_nor_init(mt8173_nor, ppdata);

nor_free:
	return ret;
}

static int mtk_nor_drv_remove(struct platform_device *pdev)
{
	struct mt8173_nor *mt8173_nor = platform_get_drvdata(pdev);
	struct spi_nor *nor = &mt8173_nor->nor;

	clk_disable_unprepare(mt8173_nor->spi_clk);
	clk_disable_unprepare(mt8173_nor->src_axi_clk);
	clk_disable_unprepare(mt8173_nor->sf_mux_clk);
	clk_disable_unprepare(mt8173_nor->nor_clk);
	mtd_device_unregister(&nor->mtd);
	return 0;
}

static const struct of_device_id mtk_nor_of_ids[] = {
	{ .compatible = "mediatek,mt8173-nor"},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mtk_nor_of_ids);

static struct platform_driver mtk_nor_driver = {
	.probe = mtk_nor_drv_probe,
	.remove = mtk_nor_drv_remove,
	.driver = {
		.name = "mtk-nor",
		.of_match_table = mtk_nor_of_ids,
	},
};

module_platform_driver(mtk_nor_driver);
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MediaTek SPI NOR Flash Driver");
