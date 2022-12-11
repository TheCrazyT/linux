/**
 * NAND flash driver for Broadcom Secondary Memory Interface
 *
 * Written by Luke Wren <luke@raspberrypi.org>
 * Copyright (c) 2015, Raspberry Pi (Trading) Ltd.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions, and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The names of the above-listed copyright holders may not be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2, as published by the Free
 * Software Foundation.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/mtd/rawnand.h>
#include <linux/mtd/partitions.h>

#include <linux/broadcom/bcm2835_smi.h>

#define DEVICE_NAME "bcm2835-smi-nand"
#define DRIVER_NAME "smi-nand-bcm2835"

struct bcm2835_smi_controller {
	struct platform_device *ofdev;
	void __iomem *base;
	struct nand_chip chip;
	int chip_select;
	struct nand_controller smi_control;
	struct mtd_info *mtd;
};
struct bcm2835_smi_nand_host {
	struct bcm2835_smi_instance *smi_inst;
	struct nand_chip nand_chip;
	struct mtd_info mtd;
	struct device *dev;
};
struct bcm2835_smi_controller smic;

/****************************************************************************
*
*   NAND functionality implementation
*
****************************************************************************/

#define SMI_NAND_CLE_PIN 0x01
#define SMI_NAND_ALE_PIN 0x02

static inline struct bcm2835_smi_instance* fetch_inst(struct nand_chip *chip) {
	struct bcm2835_smi_controller *psmic = nand_get_controller_data(chip);
	struct mtd_info *mtd = psmic->mtd;
	struct bcm2835_smi_nand_host *host;
	struct bcm2835_smi_instance *inst;
	if(!mtd) {
		pr_crit("mtd failed!");
		return NULL;
	}
	host = dev_get_drvdata(mtd->dev.parent);
	if(!host) {
		dev_err(mtd->dev.parent, "host failed!");
		return NULL;
	}
	inst = host->smi_inst;
	if(!inst) {
		dev_err(mtd->dev.parent, "inst failed!");
		return NULL;
	}
	return inst;
}

static inline void bcm2835_smi_nand_cmd_ctrl(struct nand_chip *chip, int cmd,
					     unsigned int ctrl)
{
	uint32_t cmd32 = cmd;
	uint32_t addr = ~(SMI_NAND_CLE_PIN | SMI_NAND_ALE_PIN);
	
	struct bcm2835_smi_instance *inst = fetch_inst(chip);

	if (ctrl & NAND_CLE)
		addr |= SMI_NAND_CLE_PIN;
	if (ctrl & NAND_ALE)
		addr |= SMI_NAND_ALE_PIN;
	/* Lower ALL the CS pins! */
	if (ctrl & NAND_NCE)
		addr &= (SMI_NAND_CLE_PIN | SMI_NAND_ALE_PIN);

	bcm2835_smi_set_address(inst, addr);

	if (cmd != NAND_CMD_NONE)
		bcm2835_smi_write_buf(inst, &cmd32, 1);
}

static inline uint8_t bcm2835_smi_nand_read_byte(struct nand_chip *chip)
{
	uint8_t byte;

	struct bcm2835_smi_instance *inst = fetch_inst(chip);

	bcm2835_smi_read_buf(inst, &byte, 1);
	return byte;
}

static inline void bcm2835_smi_nand_write_byte(struct nand_chip *chip,
					       uint8_t byte)
{
	struct bcm2835_smi_instance *inst = fetch_inst(chip);

	bcm2835_smi_write_buf(inst, &byte, 1);
}

static inline void bcm2835_smi_nand_write_buf(struct nand_chip *chip,
					      const uint8_t *buf, int len)
{
	struct bcm2835_smi_instance *inst = fetch_inst(chip);

	bcm2835_smi_write_buf(inst, buf, len);
}

static inline void bcm2835_smi_nand_read_buf(struct nand_chip *chip,
					     uint8_t *buf, int len)
{
	struct bcm2835_smi_instance *inst = fetch_inst(chip);

	bcm2835_smi_read_buf(inst, buf, len);
}

/****************************************************************************
*
*   Probe and remove functions
*
***************************************************************************/

static int bcm2835_smi_nand_probe(struct platform_device *pdev)
{
	struct bcm2835_smi_nand_host *host;
	struct nand_chip *this;
	struct mtd_info *mtd;
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node, *smi_node;
	struct smi_settings *smi_settings;
	struct bcm2835_smi_instance *smi_inst;

	pr_warn("bcm2835_smi_nand_probe");
	
	if (!dev) {
		pr_crit("dev not set!");
		return -EINVAL;
	}
	
	if (!node) {
		dev_err(dev, "No device tree node supplied!");
		return -EINVAL;
	}

	smi_node = of_parse_phandle(node, "smi_handle", 0);

	dev_warn(dev,"before bcm2835_smi_get");
	/* Request use of SMI peripheral: */
	smi_inst = bcm2835_smi_get(smi_node);

	if (!smi_inst) {
		dev_err(dev, "Could not register with SMI.");
		return -EPROBE_DEFER;
	}

	/* Set SMI timing and bus width */
	pr_warn("before bcm2835_smi_get_settings_from_regs");
	smi_settings = bcm2835_smi_get_settings_from_regs(smi_inst);

	smi_settings->data_width = SMI_WIDTH_8BIT;
	smi_settings->read_setup_time = 2;
	smi_settings->read_hold_time = 1;
	smi_settings->read_pace_time = 1;
	smi_settings->read_strobe_time = 3;

	smi_settings->write_setup_time = 2;
	smi_settings->write_hold_time = 1;
	smi_settings->write_pace_time = 1;
	smi_settings->write_strobe_time = 3;

	dev_warn(dev,"before bcm2835_smi_set_regs_from_settings");
	bcm2835_smi_set_regs_from_settings(smi_inst);

	dev_warn(dev,"before devm_kzalloc");
	host = devm_kzalloc(dev, sizeof(struct bcm2835_smi_nand_host),
		GFP_KERNEL);
	if (!host)
		return -ENOMEM;

	host->dev = dev;
	host->smi_inst = smi_inst;

	dev_warn(dev,"before platform_set_drvdata");
	platform_set_drvdata(pdev, host);

	/* Link the structures together */

	this = &host->nand_chip;
	
	mtd = &host->mtd;
	mtd->priv = this;
	mtd->owner = THIS_MODULE;
	mtd->dev.parent = dev;
	mtd->name = DRIVER_NAME;

	if(!mtd->dev.parent){
		dev_err(dev, "mtd->dev.parent failed! #1");
		return -EINVAL;
	}
	
	/* 20 us command delay time... */
	this->legacy.chip_delay = 20;

	this->priv = host;
	this->legacy.cmd_ctrl = bcm2835_smi_nand_cmd_ctrl;
	this->legacy.read_byte = bcm2835_smi_nand_read_byte;
	this->legacy.write_byte = bcm2835_smi_nand_write_byte;
	this->legacy.write_buf = bcm2835_smi_nand_write_buf;
	this->legacy.read_buf = bcm2835_smi_nand_read_buf;
	
	this->ecc.engine_type = NAND_ECC_ENGINE_TYPE_SOFT;
	this->ecc.algo = NAND_ECC_ALGO_BCH;

	/* Should never be accessed directly: */

	this->legacy.IO_ADDR_R = (void *)0xdeadbeef;
	this->legacy.IO_ADDR_W = (void *)0xdeadbeef;
	
	dev_warn(dev,"additional prechecks #1");
	dev_warn(dev,"mtd before: %p",mtd);
	//additional prechecks
	/*mtd = nand_to_mtd(this);
	dev_warn(dev,"mtd after: %p",mtd);
	if(!mtd) {
		dev_err(dev, "nand_to_mtd failed!");
		return -EINVAL;
	}*/
	dev_warn(dev,"additional prechecks #2");
	if(!mtd->dev.parent){
		dev_err(dev, "mtd->dev.parent failed! #2");
		return -EINVAL;
	}
	dev_warn(dev,"additional prechecks #3");
	host = dev_get_drvdata(mtd->dev.parent);
	dev_warn(dev,"additional prechecks #4");
	if(!host) {
		dev_err(dev, "dev_get_drvdata failed!");
		return -EINVAL;
	}
	dev_warn(dev,"additional prechecks #5");
	if(!host->smi_inst) {
		dev_err(dev, "host->smi_inst is null!");
		return -EINVAL;
	}
	
	dev_warn(dev,"before nand_set_controller_data");
	nand_set_controller_data(this, &smic);
	smic.mtd = mtd;

	/* Scan to find the device and get the page size */
	dev_warn(dev,"before nand_scan");

	if (nand_scan(this, 1))
		return -ENXIO;

	dev_warn(dev,"nand id: %02x %02x %02x %02x",
		this->id.data[0],this->id.data[1],this->id.data[2],this->id.data[3]);

	return 0;
}

static int bcm2835_smi_nand_remove(struct platform_device *pdev)
{
	struct bcm2835_smi_nand_host *host = platform_get_drvdata(pdev);
	pr_warn("bcm2835_smi_nand_remove");
	if (host) {
		struct nand_chip *chip = &host->nand_chip;
		int ret;

		ret = mtd_device_unregister(nand_to_mtd(chip));
		WARN_ON(ret);
		nand_cleanup(chip);
		//bcm2835_smi_nand_disable(host);

	}

	return 0;
}

/****************************************************************************
*
*   Register the driver with device tree
*
***************************************************************************/

static const struct of_device_id bcm2835_smi_nand_of_match[] = {
	{.compatible = "brcm,bcm2835-smi-nand",},
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, bcm2835_smi_nand_of_match);

static struct platform_driver bcm2835_smi_nand_driver = {
	.probe = bcm2835_smi_nand_probe,
	.remove = bcm2835_smi_nand_remove,
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = bcm2835_smi_nand_of_match,
	},
};

module_platform_driver(bcm2835_smi_nand_driver);

MODULE_ALIAS("platform:smi-nand-bcm2835");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION
	("Driver for NAND chips using Broadcom Secondary Memory Interface");
MODULE_AUTHOR("Luke Wren <luke@raspberrypi.org>");
