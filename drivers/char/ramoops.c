/*
 * RAM Oops/Panic logger
 *
 * Copyright (C) 2010 Marco Stornelli <marco.stornelli@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/kmsg_dump.h>
#include <linux/time.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/ramoops.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>

#define RAMOOPS_KERNMSG_HDR "===="
#define MIN_MEM_SIZE 4096UL
#define RAMOOPS_DIR "ramoops"
#define RAMOOPS_NEXT "next"

static ulong record_size = MIN_MEM_SIZE;
module_param(record_size, ulong, 0400);
MODULE_PARM_DESC(record_size,
		"size of each dump done on oops/panic");

static ulong mem_address;
module_param(mem_address, ulong, 0400);
MODULE_PARM_DESC(mem_address,
		"start of reserved RAM used to store oops/panic logs");

static ulong mem_size;
module_param(mem_size, ulong, 0400);
MODULE_PARM_DESC(mem_size,
		"size of reserved RAM used to store oops/panic logs");

static int dump_oops = 1;
module_param(dump_oops, int, 0600);
MODULE_PARM_DESC(dump_oops,
		"set to 1 to dump oopses, 0 to only dump panics (default 1)");

static struct ramoops_context {
	struct kmsg_dumper dump;
	void *virt_addr;
	phys_addr_t phys_addr;
	unsigned long size;
	unsigned long record_size;
	int dump_oops;
	int current_entry;
	int count;
	int max_count;
} oops_cxt;

static struct platform_device *dummy;
static struct ramoops_platform_data *dummy_data;
static DEFINE_MUTEX(ramoops_mutex);

/* Debugfs entries for ramoops */
static struct dentry *ramoops_dir, *ramoops_next_entry;

/*
 * Entry to have access to the memory logged by ramoops. The way the data
 * is returned is as follows:
 * Data records are checked one by one for the existence of the header.
 * If a valid record is found data is returned from it, otherwise it is
 * skipped.
 * Once all the records are checked the next call after the last record
 * will return an empty buffer and the counter is reset.
 */
static ssize_t ramoops_read_next(struct file *file, char __user *buf,
					size_t count, loff_t *ppos)
{
	struct ramoops_context *cxt = &oops_cxt;

	mutex_lock(&ramoops_mutex);

	/* Do this check here so it returns an empty file once it resets */
	if (cxt->current_entry >= cxt->max_count) {
		cxt->current_entry = 0;
		count = 0;
		goto out;
	}

	/* Check to see if the current dump is valid */
	while (cxt->current_entry < cxt->max_count
		&& memcmp(cxt->virt_addr + cxt->record_size *
			  cxt->current_entry, RAMOOPS_KERNMSG_HDR,
			  strlen(RAMOOPS_KERNMSG_HDR)))
		cxt->current_entry++;

	/* In case a dump was not found return 0 */
	if (cxt->current_entry >= cxt->max_count) {
		count = 0;
		goto out;
	}

	/* Otherwise proceed as normal to return the data */
	if (*ppos + count > cxt->record_size)
		count = cxt->record_size - *ppos;
	if (*ppos > cxt->record_size) {
		count = 0;
		cxt->current_entry++;
		goto out;
	}
	if (copy_to_user(buf, cxt->virt_addr + cxt->record_size *
			 cxt->current_entry + *ppos, count)) {
		count = -EFAULT;
		goto out;
	}
	*ppos += count;

	/* completed reading this part, go to the next one */
	if (*ppos == cxt->record_size)
		cxt->current_entry++;

out:
	mutex_unlock(&ramoops_mutex);
	return count;
}

static const struct file_operations ramoops_next_fops = {
	.read = ramoops_read_next,
};

static void ramoops_do_dump(struct kmsg_dumper *dumper,
		enum kmsg_dump_reason reason, const char *s1, unsigned long l1,
		const char *s2, unsigned long l2)
{
	struct ramoops_context *cxt = container_of(dumper,
			struct ramoops_context, dump);
	unsigned long s1_start, s2_start;
	unsigned long l1_cpy, l2_cpy;
	int res, hdr_size;
	char *buf, *buf_orig;
	struct timeval timestamp;

	if (reason != KMSG_DUMP_OOPS &&
	    reason != KMSG_DUMP_PANIC &&
	    reason != KMSG_DUMP_KEXEC)
		return;

	/* Only dump oopses if dump_oops is set */
	if (reason == KMSG_DUMP_OOPS && !cxt->dump_oops)
		return;

	mutex_lock(&ramoops_mutex);
	buf = cxt->virt_addr + (cxt->count * cxt->record_size);
	buf_orig = buf;

	memset(buf, '\0', cxt->record_size);
	res = sprintf(buf, "%s", RAMOOPS_KERNMSG_HDR);
	buf += res;
	do_gettimeofday(&timestamp);
	res = sprintf(buf, "%lu.%lu\n", (long)timestamp.tv_sec, (long)timestamp.tv_usec);
	buf += res;

	hdr_size = buf - buf_orig;
	l2_cpy = min(l2, cxt->record_size - hdr_size);
	l1_cpy = min(l1, cxt->record_size - hdr_size - l2_cpy);

	s2_start = l2 - l2_cpy;
	s1_start = l1 - l1_cpy;

	memcpy(buf, s1 + s1_start, l1_cpy);
	memcpy(buf + l1_cpy, s2 + s2_start, l2_cpy);

	cxt->count = (cxt->count + 1) % cxt->max_count;
	mutex_unlock(&ramoops_mutex);
}

static int __init ramoops_probe(struct platform_device *pdev)
{
	struct ramoops_platform_data *pdata = pdev->dev.platform_data;
	struct ramoops_context *cxt = &oops_cxt;
	int err = -EINVAL;

	if (!pdata->mem_size || !pdata->record_size) {
		pr_err("The memory size and the record size must be "
			"non-zero\n");
		goto fail3;
	}

	rounddown_pow_of_two(pdata->mem_size);
	rounddown_pow_of_two(pdata->record_size);

	/* Check for the minimum memory size */
	if (pdata->mem_size < MIN_MEM_SIZE
	    && pdata->record_size < MIN_MEM_SIZE) {
		pr_err("memory size too small, minium is %lu\n", MIN_MEM_SIZE);
		goto fail3;
	}

	if (pdata->mem_size < pdata->record_size) {
		pr_err("The memory size must be larger than the "
			"records size\n");
		goto fail3;
	}

	cxt->max_count = pdata->mem_size / pdata->record_size;
	cxt->count = 0;
	cxt->size = pdata->mem_size;
	cxt->phys_addr = pdata->mem_address;
	cxt->record_size = pdata->record_size;
	cxt->current_entry = 0;
	cxt->dump_oops = pdata->dump_oops;

	if (!request_mem_region(cxt->phys_addr, cxt->size, "ramoops")) {
		pr_err("request mem region failed\n");
		err = -EINVAL;
		goto fail3;
	}

	cxt->virt_addr = ioremap(cxt->phys_addr,  cxt->size);
	if (!cxt->virt_addr) {
		pr_err("ioremap failed\n");
		goto fail2;
	}

	cxt->dump.dump = ramoops_do_dump;
	err = kmsg_dump_register(&cxt->dump);
	if (err) {
		pr_err("registering kmsg dumper failed\n");
		goto fail1;
	}

	/* Initialize debugfs entry so the memory can be easily accessed */
	ramoops_dir = debugfs_create_dir(RAMOOPS_DIR, NULL);
	if (ramoops_dir == NULL) {
		err = -ENOMEM;
		pr_err("debugfs directory entry creation failed\n");
		goto out;
	}

	/*
	 * This interface exposes the next valid entry from ramoops, starting
	 * from the start of memory area. Once there are no more entries it
	 * returns an empty buffer. Reading the entry again afterwards starts
	 * from the begining.
	 */
	ramoops_next_entry = debugfs_create_file(RAMOOPS_NEXT, 0444,
					ramoops_dir, NULL, &ramoops_next_fops);

	if (ramoops_next_entry == NULL) {
		err = -ENOMEM;
		pr_err("debugfs next entry creation failed\n");
		goto no_ramoops_next;
	}


	return 0;

fail1:
	iounmap(cxt->virt_addr);
fail2:
	release_mem_region(cxt->phys_addr, cxt->size);
fail3:
	return err;

no_ramoops_next:
	debugfs_remove(ramoops_dir);
out:
	return err;
}

static int __exit ramoops_remove(struct platform_device *pdev)
{
	struct ramoops_context *cxt = &oops_cxt;

	if (kmsg_dump_unregister(&cxt->dump) < 0)
		pr_warn("could not unregister kmsg_dumper\n");

	iounmap(cxt->virt_addr);
	release_mem_region(cxt->phys_addr, cxt->size);
	return 0;
}

static struct platform_driver ramoops_driver = {
	.remove		= __exit_p(ramoops_remove),
	.driver		= {
		.name	= "ramoops",
		.owner	= THIS_MODULE,
	},
};

static int __init ramoops_init(void)
{
	int ret;
	ret = platform_driver_probe(&ramoops_driver, ramoops_probe);
	if (ret == -ENODEV) {
		/*
		 * If we didn't find a platform device, we use module parameters
		 * building platform data on the fly.
		 */
		pr_info("platform device not found, using module parameters\n");
		dummy_data = kzalloc(sizeof(struct ramoops_platform_data),
				     GFP_KERNEL);
		if (!dummy_data)
			return -ENOMEM;
		dummy_data->mem_size = mem_size;
		dummy_data->mem_address = mem_address;
		dummy_data->record_size = record_size;
		dummy_data->dump_oops = dump_oops;
		dummy = platform_create_bundle(&ramoops_driver, ramoops_probe,
			NULL, 0, dummy_data,
			sizeof(struct ramoops_platform_data));

		if (IS_ERR(dummy))
			ret = PTR_ERR(dummy);
		else
			ret = 0;
	}

	return ret;
}

static void __exit ramoops_exit(void)
{
	platform_driver_unregister(&ramoops_driver);
	kfree(dummy_data);

	/* Clean up debugfs entries */
	debugfs_remove(ramoops_next_entry);
	debugfs_remove(ramoops_dir);
}

module_init(ramoops_init);
module_exit(ramoops_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Marco Stornelli <marco.stornelli@gmail.com>");
MODULE_DESCRIPTION("RAM Oops/Panic logger/driver");
