/*
 * arch/arm/mach-msm/lge/lge_bootloader_log.c
 *
 * Copyright (C) 2012 LGE, Inc
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <linux/io.h>
#include <asm/setup.h>
//#include <mach/board_lge.h>
#include <linux/init.h>
#ifdef CONFIG_OF
#include <linux/of_fdt.h>
#endif

#define ATAG_LGE_BOOTLOG	0xc0e0c0e0

struct log_buffer {
	uint32_t    sig;
	uint32_t    start;
	uint32_t    size;
	uint32_t    tot_size;
	uint8_t     data[0];
};

u32 boot_logbuf_phys;
struct log_buffer *boot_logbuf_virt;

#ifdef CONFIG_OF
static int __init logbuf_parse_dt(unsigned long node, const char *uname, int depth, void *data)
{
	struct tag *tags;

	if (depth != 1 || (strcmp(uname, "chosen") != 0 && strcmp(uname, "chosen@0") != 0))
		return 0;

	tags = (struct tag *) of_get_flat_dt_prop(node, "atag,logbuf", NULL);
	if (tags) {
		boot_logbuf_phys = tags->u.revision.rev;
		printk("%s: find the bootloader log tag\n", __func__);
		printk("%s: bootloader log at 0x%x\n", __func__, boot_logbuf_phys);
	}

	return 1;
}

static int __init logbuf_of_init(void)
{
	of_scan_flat_dt(logbuf_parse_dt, NULL);
	return 0;
}
#else
static int __init parse_tag_lge_bootlog(const struct tag *tag)
{
	boot_logbuf_phys = tag->u.revision.rev;

	printk("%s: find the bootloader log tag\n", __func__);
	printk("%s: bootloader log at 0x%x\n", __func__, boot_logbuf_phys);

	return 0;
}

__tagtable(ATAG_LGE_BOOTLOG, parse_tag_lge_bootlog);
#endif

static int __init lge_bootlog_init(void)
{
	char *buffer;
	char *token;
	char *ct = "\n";

	boot_logbuf_virt = (struct log_buffer *)ioremap(boot_logbuf_phys, 256 * 1024);
	if (boot_logbuf_virt == NULL) {
		printk("%s: failed to map memory\n",__func__);
		return 0;
	}

	printk("%s: sig %x\n",__func__, boot_logbuf_virt->sig);
	printk("%s: start %d\n",__func__, boot_logbuf_virt->start);
	printk("%s: size %d\n",__func__, boot_logbuf_virt->size);
	printk("--------------------------------------------------------------\n");
	printk("below logs are got from bootloader \n");
	printk("--------------------------------------------------------------\n");
	printk("\n");
	buffer = (char *)boot_logbuf_virt->data;

	while (1) {
		token = strsep(&buffer, ct);
		if (!token) {
			printk("%s: token %p\n",__func__, token);
			break;
		}
		printk("%s\n", token);
	}
	printk("--------------------------------------------------------------\n");

	return 0;
}

static void __exit lge_bootlog_exit(void)
{
	return;
}

#ifdef CONFIG_OF
early_initcall(logbuf_of_init);
#endif
module_init(lge_bootlog_init);
module_exit(lge_bootlog_exit);

MODULE_DESCRIPTION("LGE bootloader log driver");
MODULE_AUTHOR("SungEun Kim <cleaneye.kim@lge.com>");
MODULE_LICENSE("GPL");

