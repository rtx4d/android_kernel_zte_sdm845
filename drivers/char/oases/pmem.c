/*
 * Sysfs for OASES framework
 *
 * Copyright (C) 2016 Baidu, Inc. All Rights Reserved.
 *
 * You should have received a copy of license along with this program;
 * if not, ask for it from Baidu, Inc.
 *
 */
#include <linux/types.h>
#include <linux/compiler.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/kmsg_dump.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/version.h>
#include "pmem.h"
#include "util.h"

struct oases_pmem oases_pmem;

struct oases_pmem_data {
	phys_addr_t phys_addr;
	unsigned phys_size;
	void *virt_addr;
};

static struct oases_pmem_data *oases_pmem_data = NULL;

static void oases_dump(struct kmsg_dumper *dump,
	    enum kmsg_dump_reason reason)
{
	struct oases_pmem *p;

	if (reason != KMSG_DUMP_PANIC)
		return;
	if (oases_pmem_data) {
		p = oases_pmem_data->virt_addr;
		if (p) {
			memcpy(p->magic, OASES_PMEM_MAGIC, 16);
			p->boot |= OASES_KDUMP_PANIC;
		}
	}
}

static struct kmsg_dumper oases_kmsg_dump = {
	.dump = oases_dump,
};

static int oases_pmem_probe(struct platform_device *pdev)
{
	int rc;
	struct device *dev = &pdev->dev;
	struct device_node *of;
	struct oases_pmem_data *pdata;
	u32 val;
	struct oases_pmem tmp, *p;

	if (!pdev->dev.of_node) {
		oases_debug("OASES DT not configured\n");
		return -EINVAL;
	}
	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (pdata == NULL) {
		oases_error("devm_kzalloc failed\n");
		return -ENOMEM;
	}
	of = pdev->dev.of_node;
	rc = of_property_read_u32(of, "oases,pmem_phys", &val);
	if (rc) {
		oases_error("oases,pmem_phys not configured\n");
		goto fail_of;
	}
	pdata->phys_addr = val;
	rc = of_property_read_u32(of, "oases,pmem_size", &val);
	if (rc) {
		oases_error("oases,pmem_size not configured\n");
		goto fail_of;
	}
	pdata->phys_size = val;
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 16, 0)
	rc = dev_set_drvdata(dev, pdata);
	if (rc) {
		kfree(pdata);
		oases_error("dev_set_drvdata failed\n");
		return rc;
	}
#else
	dev_set_drvdata(dev, pdata);
#endif
	oases_pmem_data = pdata;
fail_of:
	pdata = dev_get_drvdata(dev);
	if (pdata == NULL) {
		return -EINVAL;
	}
	pdata->virt_addr = ioremap(pdata->phys_addr, pdata->phys_size);
	if (pdata->virt_addr) {
		p = pdata->virt_addr;
		memcpy_fromio(&tmp, p, sizeof(*p));
		if (!memcmp(tmp.magic, OASES_PMEM_MAGIC, 16))
			oases_pmem = tmp;
		memset(&tmp, 0, sizeof(tmp));
		memcpy(tmp.magic, OASES_PMEM_MAGIC, 16);
		memcpy_toio(p, &tmp, sizeof(*p));
	}
	rc = kmsg_dump_register(&oases_kmsg_dump);
	if (rc) {
		oases_error("kmsg_dump_register failed\n");
		return rc;
	}
	return 0;
}

static int oases_pmem_remove(struct platform_device *pdev)
{
	oases_debug("oases_pmem_remove\n");
	return 0;
}

static const struct of_device_id oases_pmem_dt[] = {
	{ .compatible = "oases" },
	{ },
};

static struct platform_driver oases_pmem_driver = {
	.probe = oases_pmem_probe,
	.remove = oases_pmem_remove,
	.driver = {
		.name = "oases",
		.of_match_table = oases_pmem_dt,
	},
};

static int __init oases_pmem_init(void)
{
	memset(&oases_pmem, 0, sizeof(oases_pmem));
	return platform_driver_register(&oases_pmem_driver);
}

static void __exit oases_pmem_exit(void)
{
	platform_driver_unregister(&oases_pmem_driver);
}


module_init(oases_pmem_init);
module_exit(oases_pmem_exit);

MODULE_AUTHOR("Baidu, Inc.");
MODULE_DESCRIPTION("OASES - Open Adaptive Security Extensions");
