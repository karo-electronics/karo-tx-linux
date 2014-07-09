/*
 * Copyright 2006-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file mxc_vpu.c
 *
 * @brief VPU system initialization and file operation implementation
 *
 * @ingroup VPU
 */

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/stat.h>
#include <linux/platform_device.h>
#include <linux/kdev_t.h>
#include <linux/dma-mapping.h>
#include <linux/wait.h>
#include <linux/list.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/fsl_devices.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/sched.h>
#include <linux/vmalloc.h>
#include <linux/regulator/consumer.h>
#include <linux/page-flags.h>
#include <linux/mm_types.h>
#include <linux/types.h>
#include <linux/memblock.h>
#include <linux/memory.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/sizes.h>
#include <linux/genalloc.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/reset.h>
#include <linux/clk.h>
#include <linux/mxc_vpu.h>

/* Define one new pgprot which combined uncached and XN(never executable) */
#define pgprot_noncachedxn(prot) \
	__pgprot_modify(prot, L_PTE_MT_MASK, L_PTE_MT_UNCACHED | L_PTE_XN)

struct vpu_priv {
	struct fasync_struct *async_queue;
	struct work_struct work;
	struct workqueue_struct *workqueue;
	struct mutex lock;
	const struct mxc_vpu_soc_data *soc_data;
	int clk_enabled;
};

struct vpu_user_data {
	struct vpu_priv *vpu_data;
	int clk_enable_cnt;
};

/* To track the allocated memory buffer */
struct memalloc_record {
	struct list_head list;
	struct vpu_mem_desc mem;
};

struct iram_setting {
	u32 start;
	u32 end;
};

struct mxc_vpu_soc_data {
	unsigned vpu_pwr_mgmnt:1,
		regulator_required:1,
		quirk_subblk_en:1,
		is_mx51:1,
		is_mx53:1,
		is_mx6dl:1,
		is_mx6q:1,
		has_jpu:1;
};

static struct gen_pool *iram_pool;
static u32 iram_base;

static LIST_HEAD(mem_list);

static int vpu_major;
static struct class *vpu_class;
static struct vpu_priv *vpu_data;
static u8 open_count;
static struct clk *vpu_clk;
static struct vpu_mem_desc bitwork_mem;
static struct vpu_mem_desc pic_para_mem;
static struct vpu_mem_desc user_data_mem;
static struct vpu_mem_desc share_mem;
static struct vpu_mem_desc vshare_mem;

static void __iomem *vpu_base;
static int vpu_ipi_irq;
static u32 phy_vpu_base_addr;

static struct device *vpu_dev;

/* IRAM setting */
static struct iram_setting iram;

/* implement the blocking ioctl */
static int irq_status;
static int codec_done;
static wait_queue_head_t vpu_queue;

static int vpu_jpu_irq;

#ifdef CONFIG_PM
static unsigned int regBk[64];
static unsigned int pc_before_suspend;
#endif
static struct regulator *vpu_regulator;

#define	READ_REG(x)		readl_relaxed(vpu_base + (x))
#define	WRITE_REG(val, x)	writel_relaxed(val, vpu_base + (x))

static int vpu_clk_enable(struct vpu_priv *vpu_data)
{
	int ret = 0;

	if (vpu_data->clk_enabled++ == 0)
		ret = clk_prepare_enable(vpu_clk);

	if (WARN_ON(vpu_data->clk_enabled <= 0))
		return -EINVAL;

	return ret;
}

static int vpu_clk_disable(struct vpu_priv *vpu_data)
{
	if (WARN_ON(vpu_data->clk_enabled == 0))
		return -EINVAL;

	if (--vpu_data->clk_enabled == 0)
		clk_disable_unprepare(vpu_clk);
	return 0;
}

static inline int vpu_reset(void)
{
	return device_reset(vpu_dev);
}

static void vpu_power_up(void)
{
	int ret;

	if (IS_ERR(vpu_regulator))
		return;

	ret = regulator_enable(vpu_regulator);
	if (ret)
		dev_err(vpu_dev, "failed to power up vpu: %d\n", ret);
}

static void vpu_power_down(void)
{
	int ret;

	if (IS_ERR(vpu_regulator))
		return;

	ret = regulator_disable(vpu_regulator);
	if (ret)
		dev_err(vpu_dev, "failed to power down vpu: %d\n", ret);
}

/*!
 * Private function to alloc dma buffer
 * @return status  0 success.
 */
static int vpu_alloc_dma_buffer(struct vpu_mem_desc *mem)
{
	mem->cpu_addr = dma_alloc_coherent(vpu_dev, PAGE_ALIGN(mem->size),
					&mem->phy_addr,
					GFP_DMA | GFP_KERNEL);
	dev_dbg(vpu_dev, "[ALLOC] mem alloc cpu_addr = %p\n", mem->cpu_addr);
	if (mem->cpu_addr == NULL) {
		dev_err(vpu_dev, "Physical memory allocation error!\n");
		return -ENOMEM;
	}
	return 0;
}

/*!
 * Private function to free dma buffer
 */
static void vpu_free_dma_buffer(struct vpu_mem_desc *mem)
{
	if (mem->cpu_addr != NULL)
		dma_free_coherent(vpu_dev, PAGE_ALIGN(mem->size),
				mem->cpu_addr, mem->phy_addr);
}

/*!
 * Private function to free buffers
 * @return status  0 success.
 */
static int vpu_free_buffers(void)
{
	struct memalloc_record *rec, *n;
	struct vpu_mem_desc mem;

	list_for_each_entry_safe(rec, n, &mem_list, list) {
		mem = rec->mem;
		if (mem.cpu_addr != 0) {
			vpu_free_dma_buffer(&mem);
			dev_dbg(vpu_dev, "[FREE] freed paddr=0x%08X\n", mem.phy_addr);
			/* delete from list */
			list_del(&rec->list);
			kfree(rec);
		}
	}

	return 0;
}

static inline void vpu_worker_callback(struct work_struct *w)
{
	struct vpu_priv *dev = container_of(w, struct vpu_priv, work);

	if (dev->async_queue)
		kill_fasync(&dev->async_queue, SIGIO, POLL_IN);

	irq_status = 1;
	/*
	 * Clock is gated on when dec/enc started, gate it off when
	 * codec is done.
	 */
	if (codec_done)
		codec_done = 0;

	wake_up_interruptible(&vpu_queue);
}

/*!
 * @brief vpu interrupt handler
 */
static irqreturn_t vpu_ipi_irq_handler(int irq, void *dev_id)
{
	struct vpu_priv *dev = dev_id;
	unsigned long reg;

	reg = READ_REG(BIT_INT_REASON);
	if (reg & 0x8)
		codec_done = 1;
	WRITE_REG(0x1, BIT_INT_CLEAR);

	queue_work(dev->workqueue, &dev->work);

	return IRQ_HANDLED;
}

/*!
 * @brief vpu jpu interrupt handler
 */
static irqreturn_t vpu_jpu_irq_handler(int irq, void *dev_id)
{
	struct vpu_priv *dev = dev_id;
	unsigned long reg;

	reg = READ_REG(MJPEG_PIC_STATUS_REG);
	if (reg & 0x3)
		codec_done = 1;

	queue_work(dev->workqueue, &dev->work);

	return IRQ_HANDLED;
}

/*!
 * @brief open function for vpu file operation
 *
 * @return  0 on success or negative error code on error
 */
static int vpu_open(struct inode *inode, struct file *filp)
{
	struct vpu_user_data *user_data = devm_kzalloc(vpu_dev,
						sizeof(*user_data),
						GFP_KERNEL);
	if (user_data == NULL)
		return -ENOMEM;

	user_data->vpu_data = vpu_data;

	mutex_lock(&vpu_data->lock);

	if (open_count++ == 0) {
		pm_runtime_get_sync(vpu_dev);
		vpu_power_up();
	}

	filp->private_data = user_data;
	mutex_unlock(&vpu_data->lock);
	return 0;
}

/*!
 * @brief IO ctrl function for vpu file operation
 * @param cmd IO ctrl command
 * @return  0 on success or negative error code on error
 */
static long vpu_ioctl(struct file *filp, u_int cmd,
		     u_long arg)
{
	int ret = -EINVAL;
	struct vpu_user_data *user_data = filp->private_data;
	struct vpu_priv *vpu_data = user_data->vpu_data;

	switch (cmd) {
	case VPU_IOC_PHYMEM_ALLOC:
	{
		struct memalloc_record *rec;

		rec = kzalloc(sizeof(*rec), GFP_KERNEL);
		if (!rec)
			return -ENOMEM;

		if (copy_from_user(&rec->mem,
					(struct vpu_mem_desc *)arg,
					sizeof(struct vpu_mem_desc))) {
			kfree(rec);
			return -EFAULT;
		}

		dev_dbg(vpu_dev, "[ALLOC] mem alloc size = 0x%x\n",
			rec->mem.size);

		ret = vpu_alloc_dma_buffer(&rec->mem);
		if (ret) {
			kfree(rec);
			return ret;
		}
		if (copy_to_user((void __user *)arg, &rec->mem,
					sizeof(struct vpu_mem_desc))) {
			kfree(rec);
			return -EFAULT;
		}

		mutex_lock(&vpu_data->lock);
		list_add(&rec->list, &mem_list);
		mutex_unlock(&vpu_data->lock);

		break;
	}
	case VPU_IOC_PHYMEM_FREE:
	{
		struct memalloc_record *rec, *n;
		struct vpu_mem_desc vpu_mem;

		if (copy_from_user(&vpu_mem,
					(struct vpu_mem_desc *)arg,
					sizeof(struct vpu_mem_desc)))
			return -EFAULT;

		dev_dbg(vpu_dev, "[FREE] mem freed cpu_addr = %p\n",
			vpu_mem.cpu_addr);
		if (vpu_mem.cpu_addr != NULL)
			vpu_free_dma_buffer(&vpu_mem);

		mutex_lock(&vpu_data->lock);
		list_for_each_entry_safe(rec, n, &mem_list, list) {
			if (rec->mem.cpu_addr == vpu_mem.cpu_addr) {
				list_del(&rec->list);
				break;
			}
		}
		kfree(rec);
		mutex_unlock(&vpu_data->lock);

		break;
	}
	case VPU_IOC_WAIT4INT:
	{
		u_long timeout = arg;

		ret = wait_event_interruptible_timeout(vpu_queue,
						irq_status != 0,
						msecs_to_jiffies(timeout));
		if (ret == 0) {
			dev_warn(vpu_dev, "VPU blocking: timeout.\n");
			ret = -ETIMEDOUT;
		} else if (signal_pending(current)) {
			dev_warn(vpu_dev, "VPU interrupt received.\n");
			ret = -ERESTARTSYS;
		} else {
			irq_status = 0;
		}
		break;
	}
	case VPU_IOC_IRAM_SETTING:
		ret = copy_to_user((void __user *)arg, &iram,
				sizeof(struct iram_setting));
		if (ret)
			ret = -EFAULT;

		break;
	case VPU_IOC_CLKGATE_SETTING:
	{
		u32 clkgate_en;

		if (get_user(clkgate_en, (u32 __user *)arg))
			return -EFAULT;

		mutex_lock(&vpu_data->lock);
		if (clkgate_en) {
			ret = vpu_clk_enable(vpu_data);
			if (ret == 0)
				user_data->clk_enable_cnt++;
		} else {
			if (user_data->clk_enable_cnt == 0) {
				ret = -EINVAL;
			} else {
				if (--user_data->clk_enable_cnt == 0)
					vpu_clk_disable(vpu_data);
				ret = 0;
			}
		}
		mutex_unlock(&vpu_data->lock);
		break;
	}
	case VPU_IOC_GET_SHARE_MEM:
		mutex_lock(&vpu_data->lock);
		if (share_mem.cpu_addr == NULL) {
			if (copy_from_user(&share_mem,
						(struct vpu_mem_desc *)arg,
						sizeof(struct vpu_mem_desc))) {
				mutex_unlock(&vpu_data->lock);
				return -EFAULT;
			}
			ret = vpu_alloc_dma_buffer(&share_mem);
			if (ret) {
				mutex_unlock(&vpu_data->lock);
				return ret;
			}
		}
		if (copy_to_user((void __user *)arg,
					&share_mem,
					sizeof(struct vpu_mem_desc)))
			ret = -EFAULT;
		else
			ret = 0;
		mutex_unlock(&vpu_data->lock);
		break;
	case VPU_IOC_REQ_VSHARE_MEM:
		mutex_lock(&vpu_data->lock);
		if (vshare_mem.cpu_addr == NULL) {
			if (copy_from_user(&vshare_mem,
						(struct vpu_mem_desc *)arg,
						sizeof(struct
							vpu_mem_desc))) {
				mutex_unlock(&vpu_data->lock);
				return -EFAULT;
			}
			vshare_mem.cpu_addr = vmalloc_user(vshare_mem.size);
			if (vshare_mem.cpu_addr == NULL) {
				mutex_unlock(&vpu_data->lock);
				return -ENOMEM;
			}
		}
		if (copy_to_user((void __user *)arg, &vshare_mem,
					sizeof(struct vpu_mem_desc)))
			ret = -EFAULT;
		else
			ret = 0;
		mutex_unlock(&vpu_data->lock);
		break;
	case VPU_IOC_GET_WORK_ADDR:
		if (bitwork_mem.cpu_addr == 0) {
			if (copy_from_user(&bitwork_mem,
						(struct vpu_mem_desc *)arg,
						sizeof(struct vpu_mem_desc)))
				return -EFAULT;

			ret = vpu_alloc_dma_buffer(&bitwork_mem);
			if (ret)
				return ret;
		}
		if (copy_to_user((void __user *)arg,
					&bitwork_mem,
					sizeof(struct
						vpu_mem_desc)))
			ret = -EFAULT;
		else
			ret = 0;
		break;
	/*
	 * The following two ioctls are used when user allocates a working buffer
	 * and registers it to vpu driver.
	 */
	case VPU_IOC_QUERY_BITWORK_MEM:
		if (copy_to_user((void __user *)arg,
					&bitwork_mem,
					sizeof(struct vpu_mem_desc)))
			ret = -EFAULT;
		else
			ret = 0;
		break;
	case VPU_IOC_SET_BITWORK_MEM:
		if (copy_from_user(&bitwork_mem,
					(struct vpu_mem_desc *)arg,
					sizeof(struct vpu_mem_desc)))
			ret = -EFAULT;
		else
			ret = 0;
		break;
	case VPU_IOC_SYS_SW_RESET:
		ret = vpu_reset();
		break;
	case VPU_IOC_REG_DUMP:
	case VPU_IOC_PHYMEM_DUMP:
		ret = 0;
		break;
	case VPU_IOC_PHYMEM_CHECK:
	{
		struct vpu_mem_desc check_memory;

		ret = copy_from_user(&check_memory,
				(void __user *)arg,
				sizeof(struct vpu_mem_desc));
		if (ret != 0) {
			dev_err(vpu_dev, "copy from user failure:%d\n", ret);
			ret = -EFAULT;
			break;
		}
		check_memory.size = 1;
		if (copy_to_user((void __user *)arg, &check_memory,
					sizeof(struct vpu_mem_desc)))
			ret = -EFAULT;
		else
			ret = 0;
		break;
	}
	case VPU_IOC_LOCK_DEV:
	{
		u32 lock_en;

		if (get_user(lock_en, (u32 __user *)arg))
			return -EFAULT;

		if (lock_en)
			mutex_lock(&vpu_data->lock);
		else
			mutex_unlock(&vpu_data->lock);
		ret = 0;
		break;
	}
	default:
		dev_err(vpu_dev, "No such IOCTL, cmd is %d\n", cmd);
	}
	return ret;
}

/*!
 * @brief Release function for vpu file operation
 * @return  0 on success or negative error code on error
 */
static int vpu_release(struct inode *inode, struct file *filp)
{
	unsigned long timeout;
	struct vpu_user_data *user_data = filp->private_data;
	struct vpu_priv *vpu_data = user_data->vpu_data;

	mutex_lock(&vpu_data->lock);

	if (open_count > 0 && !--open_count) {
		/* Wait for vpu go to idle state */
		vpu_clk_enable(vpu_data);
		if (READ_REG(BIT_CUR_PC)) {

			timeout = jiffies + HZ;
			while (READ_REG(BIT_BUSY_FLAG)) {
				msleep(1);
				if (time_after(jiffies, timeout)) {
					dev_warn(vpu_dev, "VPU timeout during release\n");
					break;
				}
			}

			/* Clean up interrupt */
			cancel_work_sync(&vpu_data->work);
			flush_workqueue(vpu_data->workqueue);
			irq_status = 0;

			if (READ_REG(BIT_BUSY_FLAG)) {
				if (vpu_data->soc_data->is_mx51 ||
					vpu_data->soc_data->is_mx53) {
					dev_err(vpu_dev,
						"fatal error: can't gate/power off when VPU is busy\n");
					vpu_clk_disable(vpu_data);
					mutex_unlock(&vpu_data->lock);
					return -EBUSY;
				}
				if (vpu_data->soc_data->is_mx6dl ||
					vpu_data->soc_data->is_mx6q) {
					WRITE_REG(0x11, 0x10F0);
					timeout = jiffies + HZ;
					while (READ_REG(0x10F4) != 0x77) {
						msleep(1);
						if (time_after(jiffies, timeout))
							break;
					}

					if (READ_REG(0x10F4) != 0x77) {
						dev_err(vpu_dev,
							"fatal error: can't gate/power off when VPU is busy\n");
						WRITE_REG(0x0, 0x10F0);
						vpu_clk_disable(vpu_data);
						mutex_unlock(&vpu_data->lock);
						return -EBUSY;
					}
					vpu_reset();
				}
			}
		}

		vpu_free_buffers();

		/* Free shared memory when vpu device is idle */
		vpu_free_dma_buffer(&share_mem);
		share_mem.cpu_addr = 0;
		vfree(vshare_mem.cpu_addr);
		vshare_mem.cpu_addr = 0;

		if (user_data->clk_enable_cnt)
			vpu_clk_disable(vpu_data);

		vpu_clk_disable(vpu_data);
		vpu_power_down();
		pm_runtime_put_sync_suspend(vpu_dev);
		devm_kfree(vpu_dev, user_data);
	}
	mutex_unlock(&vpu_data->lock);

	return 0;
}

/*!
 * @brief fasync function for vpu file operation
 * @return  0 on success or negative error code on error
 */
static int vpu_fasync(int fd, struct file *filp, int mode)
{
	struct vpu_user_data *user_data = filp->private_data;
	struct vpu_priv *vpu_data = user_data->vpu_data;
	return fasync_helper(fd, filp, mode, &vpu_data->async_queue);
}

/*!
 * @brief memory map function of harware registers for vpu file operation
 * @return  0 on success or negative error code on error
 */
static int vpu_map_hwregs(struct file *fp, struct vm_area_struct *vm)
{
	unsigned long pfn;

	vm->vm_flags |= VM_IO;
	/*
	 * Since vpu registers have been mapped with ioremap() at probe
	 * which L_PTE_XN is 1, and the same physical address must be
	 * mapped multiple times with same type, so set L_PTE_XN to 1 here.
	 * Otherwise, there may be unexpected result in video codec.
	 */
	vm->vm_page_prot = pgprot_noncachedxn(vm->vm_page_prot);
	pfn = phy_vpu_base_addr >> PAGE_SHIFT;
	dev_dbg(vpu_dev, "size=0x%lx, page no.=0x%lx\n",
		 vm->vm_end - vm->vm_start, pfn);
	return remap_pfn_range(vm, vm->vm_start, pfn,
			vm->vm_end - vm->vm_start,
			vm->vm_page_prot) ? -EAGAIN : 0;
}

/*!
 * @brief memory map function of memory for vpu file operation
 * @return  0 on success or negative error code on error
 */
static int vpu_map_dma_mem(struct file *fp, struct vm_area_struct *vm)
{
	size_t request_size = vm->vm_end - vm->vm_start;

	dev_dbg(vpu_dev, "start=0x%08lx, pgoff=0x%08lx, size=%zx\n",
		vm->vm_start, vm->vm_pgoff, request_size);

	vm->vm_flags |= VM_IO;
	vm->vm_page_prot = pgprot_writecombine(vm->vm_page_prot);

	return remap_pfn_range(vm, vm->vm_start, vm->vm_pgoff,
			       request_size, vm->vm_page_prot) ? -EAGAIN : 0;
}

/* !
 * @brief memory map function of vmalloced share memory
 * @return  0 on success or negative error code on error
 */
static int vpu_map_vshare_mem(struct file *fp, struct vm_area_struct *vm)
{
	int ret;

	ret = remap_vmalloc_range(vm, (void *)(vm->vm_pgoff << PAGE_SHIFT), 0);
	vm->vm_flags |= VM_IO;
	return ret;
}
/*!
 * @brief memory map interface for vpu file operation
 * @return  0 on success or negative error code on error
 */
static int vpu_mmap(struct file *fp, struct vm_area_struct *vm)
{
	unsigned long offset;

	offset = (unsigned long)vshare_mem.cpu_addr >> PAGE_SHIFT;

	if (vm->vm_pgoff && (vm->vm_pgoff == offset))
		return vpu_map_vshare_mem(fp, vm);
	else if (vm->vm_pgoff)
		return vpu_map_dma_mem(fp, vm);
	else
		return vpu_map_hwregs(fp, vm);
}

static const struct file_operations vpu_fops = {
	.owner = THIS_MODULE,
	.open = vpu_open,
	.unlocked_ioctl = vpu_ioctl,
	.release = vpu_release,
	.fasync = vpu_fasync,
	.mmap = vpu_mmap,
};

static const struct mxc_vpu_soc_data imx6dl_vpu_data = {
	.regulator_required = 1,
	.vpu_pwr_mgmnt = 1,
	.has_jpu = 1,
};

static const struct mxc_vpu_soc_data imx6q_vpu_data = {
	.quirk_subblk_en = 1,
	.regulator_required = 1,
	.vpu_pwr_mgmnt = 1,
	.has_jpu = 1,
};

static const struct mxc_vpu_soc_data imx53_vpu_data = {
};

static const struct mxc_vpu_soc_data imx51_vpu_data = {
	.vpu_pwr_mgmnt = 1,
};

static const struct of_device_id vpu_of_match[] = {
	{ .compatible = "fsl,imx6dl-vpu", .data = &imx6dl_vpu_data, },
	{ .compatible = "fsl,imx6q-vpu", .data = &imx6q_vpu_data, },
	{ .compatible = "fsl,imx53-vpu", .data = &imx53_vpu_data, },
	{ .compatible = "fsl,imx51-vpu", .data = &imx51_vpu_data, },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, vpu_of_match);

/*!
 * This function is called by the driver framework to initialize the vpu device.
 * @param   dev The device structure for the vpu passed in by the framework.
 * @return   0 on success or negative error code on error
 */
static int vpu_dev_probe(struct platform_device *pdev)
{
	int err = 0;
	struct device *temp_class;
	struct resource *res;
	unsigned long addr = 0;
	struct device_node *np = pdev->dev.of_node;
	u32 iramsize;
	struct vpu_priv *drv_data;
	const struct of_device_id *of_id = of_match_device(vpu_of_match,
							&pdev->dev);
	const struct mxc_vpu_soc_data *soc_data = of_id->data;

	drv_data = devm_kzalloc(&pdev->dev, sizeof(*drv_data), GFP_KERNEL);
	if (drv_data == NULL)
		return -ENOMEM;

	drv_data->soc_data = soc_data;
	mutex_init(&drv_data->lock);

	init_waitqueue_head(&vpu_queue);
	drv_data->workqueue = create_workqueue("vpu_wq");
	INIT_WORK(&drv_data->work, vpu_worker_callback);

	err = of_property_read_u32(np, "iramsize", &iramsize);
	if (!err && iramsize) {
		iram_pool = of_get_named_gen_pool(np, "iram", 0);
		if (!iram_pool) {
			dev_err(&pdev->dev, "iram pool not available\n");
			return -ENOMEM;
		}

		iram_base = gen_pool_alloc(iram_pool, iramsize);
		if (!iram_base) {
			dev_err(&pdev->dev, "unable to alloc iram\n");
			return -ENOMEM;
		}

		addr = gen_pool_virt_to_phys(iram_pool, iram_base);
	}

	if (addr == 0)
		iram.start = iram.end = 0;
	else {
		iram.start = addr;
		iram.end = addr + iramsize - 1;
	}

	vpu_dev = &pdev->dev;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "vpu_regs");
	if (!res) {
		dev_err(vpu_dev, "vpu: unable to get vpu base addr\n");
		return -ENODEV;
	}
	phy_vpu_base_addr = res->start;
	vpu_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(vpu_base))
		return PTR_ERR(vpu_base);

	vpu_major = register_chrdev(vpu_major, "mxc_vpu", &vpu_fops);
	if (vpu_major < 0) {
		dev_err(vpu_dev, "vpu: unable to get a major for VPU\n");
		return vpu_major;
	}

	vpu_class = class_create(THIS_MODULE, "mxc_vpu");
	if (IS_ERR(vpu_class)) {
		err = PTR_ERR(vpu_class);
		goto err_out_chrdev;
	}

	temp_class = device_create(vpu_class, NULL, MKDEV(vpu_major, 0),
				   NULL, "mxc_vpu");
	if (IS_ERR(temp_class)) {
		err = PTR_ERR(temp_class);
		goto err_out_class;
	}

	vpu_clk = clk_get(&pdev->dev, "vpu_clk");
	if (IS_ERR(vpu_clk)) {
		err = PTR_ERR(vpu_clk);
		goto err_out_class;
	}

	vpu_ipi_irq = platform_get_irq_byname(pdev, "vpu_ipi_irq");
	if (vpu_ipi_irq < 0) {
		dev_err(vpu_dev, "vpu: unable to get vpu interrupt\n");
		err = vpu_ipi_irq;
		goto err_out_class;
	}
	err = request_irq(vpu_ipi_irq, vpu_ipi_irq_handler, 0, "VPU_CODEC_IRQ",
			  drv_data);
	if (err)
		goto err_out_class;

	vpu_regulator = devm_regulator_get(vpu_dev, "pu");
	if (IS_ERR(vpu_regulator)) {
		if (drv_data->soc_data->regulator_required) {
			dev_err(vpu_dev, "failed to get vpu power\n");
			goto err_out_class;
		} else {
			/* regulator_get will return error on MX5x,
			 * just igore it everywhere
			 */
			dev_warn(vpu_dev, "failed to get vpu power\n");
		}
	}

	platform_set_drvdata(pdev, drv_data);

	if (drv_data->soc_data->has_jpu) {
		vpu_jpu_irq = platform_get_irq_byname(pdev, "vpu_jpu_irq");
		if (vpu_jpu_irq < 0) {
			dev_err(vpu_dev, "vpu: unable to get vpu jpu interrupt\n");
			err = vpu_jpu_irq;
			goto err_out_class;
		}
		err = request_irq(vpu_jpu_irq, vpu_jpu_irq_handler, IRQF_TRIGGER_RISING,
				"VPU_JPG_IRQ", drv_data);
		if (err)
			goto err_out_class;
	}

	pm_runtime_enable(&pdev->dev);
	vpu_data = drv_data;

	dev_info(vpu_dev, "VPU initialized\n");
	return 0;

err_out_class:
	device_destroy(vpu_class, MKDEV(vpu_major, 0));
	class_destroy(vpu_class);
err_out_chrdev:
	unregister_chrdev(vpu_major, "mxc_vpu");
	return err;
}

static int vpu_dev_remove(struct platform_device *pdev)
{
	struct vpu_priv *vpu_data = platform_get_drvdata(pdev);

	pm_runtime_disable(&pdev->dev);

	free_irq(vpu_ipi_irq, &vpu_data);
#ifdef MXC_VPU_HAS_JPU
	free_irq(vpu_jpu_irq, &vpu_data);
#endif
	cancel_work_sync(&vpu_data->work);
	flush_workqueue(vpu_data->workqueue);
	destroy_workqueue(vpu_data->workqueue);

	iounmap(vpu_base);
	if (iram.start)
		gen_pool_free(iram_pool, iram_base, iram.end-iram.start+1);

	if (vpu_major > 0) {
		device_destroy(vpu_class, MKDEV(vpu_major, 0));
		class_destroy(vpu_class);
		unregister_chrdev(vpu_major, "mxc_vpu");
		vpu_major = 0;
	}

	vpu_free_dma_buffer(&bitwork_mem);
	vpu_free_dma_buffer(&pic_para_mem);
	vpu_free_dma_buffer(&user_data_mem);

	/* reset VPU state */
	vpu_power_up();
	vpu_clk_enable(vpu_data);
	vpu_reset();
	vpu_clk_disable(vpu_data);
	vpu_power_down();

	clk_put(vpu_clk);
	return 0;
}

#ifdef CONFIG_PM
static int vpu_suspend(struct device *dev)
{
	struct vpu_priv *vpu_data = dev_get_drvdata(dev);
	unsigned long timeout;

	mutex_lock(&vpu_data->lock);

	if (open_count) {
		/* Wait for vpu go to idle state, suspect vpu cannot be changed
		 * to idle state after about 1 sec
		 */
		timeout = jiffies + HZ;
		while (READ_REG(BIT_BUSY_FLAG)) {
			msleep(1);
			if (time_after(jiffies, timeout)) {
				mutex_unlock(&vpu_data->lock);
				return -EAGAIN;
			}
		}

		if (vpu_data->soc_data->is_mx53) {
			mutex_unlock(&vpu_data->lock);
			return 0;
		}

		if (bitwork_mem.cpu_addr != 0) {
			int i;

			/* Save 64 registers from BIT_CODE_BUF_ADDR */
			for (i = 0; i < 64; i++)
				regBk[i] = READ_REG(BIT_CODE_BUF_ADDR + (i * 4));
			pc_before_suspend = READ_REG(BIT_CUR_PC);
		}

		vpu_clk_disable(vpu_data);
		/* If VPU is working before suspend, disable
		 * regulator to make usecount right.
		 */
		vpu_power_down();
	}

	mutex_unlock(&vpu_data->lock);
	return 0;
}

static int vpu_resume(struct device *dev)
{
	int i;
	struct vpu_priv *vpu_data = dev_get_drvdata(dev);

	mutex_lock(&vpu_data->lock);

	if (open_count) {
		if (vpu_data->soc_data->is_mx53) {
			vpu_clk_enable(vpu_data);
			goto out;
		}

		/* If VPU is working before suspend, enable
		 * regulator to make usecount right.
		 */
		vpu_power_up();

		if (bitwork_mem.cpu_addr != NULL) {
			u32 *p = bitwork_mem.cpu_addr;
			u32 data, pc;
			u16 data_hi;
			u16 data_lo;

			vpu_clk_enable(vpu_data);

			pc = READ_REG(BIT_CUR_PC);
			if (pc) {
				dev_warn(vpu_dev, "Not power off after suspend (PC=0x%x)\n", pc);
				goto out;
			}

			/* Restore registers */
			for (i = 0; i < 64; i++)
				WRITE_REG(regBk[i], BIT_CODE_BUF_ADDR + (i * 4));

			WRITE_REG(0x0, BIT_RESET_CTRL);
			WRITE_REG(0x0, BIT_CODE_RUN);
			/* MX6 RTL has a bug not to init MBC_SET_SUBBLK_EN on reset */
			if (vpu_data->soc_data->quirk_subblk_en)
				WRITE_REG(0x0, MBC_SET_SUBBLK_EN);

			/*
			 * Re-load boot code, from the codebuffer in external RAM.
			 * Thankfully, we only need 4096 bytes, same for all platforms.
			 */
			for (i = 0; i < 2048; i += 4) {
				data = p[(i / 2) + 1];
				data_hi = (data >> 16) & 0xFFFF;
				data_lo = data & 0xFFFF;
				WRITE_REG((i << 16) | data_hi, BIT_CODE_DOWN);
				WRITE_REG(((i + 1) << 16) | data_lo,
						BIT_CODE_DOWN);

				data = p[i / 2];
				data_hi = (data >> 16) & 0xFFFF;
				data_lo = data & 0xFFFF;
				WRITE_REG(((i + 2) << 16) | data_hi,
						BIT_CODE_DOWN);
				WRITE_REG(((i + 3) << 16) | data_lo,
						BIT_CODE_DOWN);
			}

			if (pc_before_suspend) {
				WRITE_REG(0x1, BIT_BUSY_FLAG);
				WRITE_REG(0x1, BIT_CODE_RUN);
				while (READ_REG(BIT_BUSY_FLAG))
					;
			} else {
				dev_warn(vpu_dev, "PC=0 before suspend\n");
			}
		}
	}
out:
	mutex_unlock(&vpu_data->lock);
	return 0;
}

static SIMPLE_DEV_PM_OPS(vpu_pm_ops, vpu_suspend, vpu_resume);
#define VPU_PM_OPS &vpu_pm_ops
#else
#define VPU_PM_OPS NULL
#endif /* !CONFIG_PM */

/*! Driver definition
 *
 */
static struct platform_driver mxcvpu_driver = {
	.driver = {
		.name = "mxc_vpu",
		.of_match_table = vpu_of_match,
		.pm = VPU_PM_OPS,
	},
	.probe = vpu_dev_probe,
	.remove = vpu_dev_remove,
};

module_platform_driver(mxcvpu_driver);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("Linux VPU driver for Freescale i.MX/MXC");
MODULE_LICENSE("GPL");
