/**
 * GPIO memory device driver
 *
 * Creates a chardev /dev/gpiomem which will provide user access to
 * the Meson's GPIO registers when it is mmap()'d.
 * No longer need root for user GPIO access, but without relaxing permissions
 * on /dev/mem.
 *
 * Copyright (c) 2019 Wesion Co., Ltd.
 *
 * This driver is based on bcm2835-gpiomem.c:
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
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/pagemap.h>
#include <linux/io.h>

#define DEVICE_NAME "aml-gpiomem"
#define DRIVER_NAME "gpiomem-aml"
#define DEVICE_MINOR 0

#define DEVICE_NUM_MAX 2

struct aml_gpiomem_instance {
 	unsigned long gpio_regs_phys;
 	struct device *dev;
 	char dev_name[32];
 	int major;
};

static struct cdev aml_gpiomem_cdev;
static dev_t aml_gpiomem_devid;
static struct class *aml_gpiomem_class;
static struct device *aml_gpiomem_dev;
//static struct aml_gpiomem_instance *inst;
static struct aml_gpiomem_instance *gpiomem_instances[DEVICE_NUM_MAX];
static int instance_num = 0;

/****************************************************************************
*
*   GPIO mem chardev file ops
*
***************************************************************************/

static int aml_gpiomem_open(struct inode *inode, struct file *file)
{
 	int dev = iminor(inode);
 	int major = imajor(inode);
 	int ret = 0;
 	int i = 0;
 	struct aml_gpiomem_instance *inst = NULL;

 	for (i=0; i<DEVICE_NUM_MAX; i++) {
 		if (major == gpiomem_instances[i]->major) {
 			inst = gpiomem_instances[i];
 			file->private_data = gpiomem_instances[i];
 		}
 	}

 	if (!inst)
 		return -ENXIO;

 	if (dev != DEVICE_MINOR) {
 		dev_err(inst->dev, "Unknown minor device: %d", dev);
 		ret = -ENXIO;
 	}
 	return ret;
}

static int aml_gpiomem_release(struct inode *inode, struct file *file)
{
 	int dev = iminor(inode);
 	int ret = 0;
 	struct aml_gpiomem_instance *inst = file->private_data;

 	if (dev != DEVICE_MINOR) {
 		dev_err(inst->dev, "Unknown minor device %d", dev);
 		ret = -ENXIO;
 	}
 	return ret;
};

static const struct vm_operations_struct aml_gpiomem_vm_ops = {
#ifdef CONFIG_HAVE_IOREMAP_PROT
 	.access = generic_access_phys
#endif
};

static int aml_gpiomem_mmap(struct file *file, struct vm_area_struct *vma)
{
 	/* Ignore what the user says - they're getting the GPIO regs
 	   whether they like it or not! */
 	struct aml_gpiomem_instance *inst = file->private_data;
 	unsigned long gpio_page = inst->gpio_regs_phys >> PAGE_SHIFT;

 	vma->vm_page_prot = phys_mem_access_prot(file, gpio_page,
			PAGE_SIZE,
			vma->vm_page_prot);
 	vma->vm_ops = &aml_gpiomem_vm_ops;
 	if (remap_pfn_range(vma, vma->vm_start,
	 			gpio_page,
		 		PAGE_SIZE,
	 			vma->vm_page_prot)) {
 		return -EAGAIN;
 	}
 	return 0;
}

static const struct file_operations
aml_gpiomem_fops = {
 	.owner = THIS_MODULE,
 	.open = aml_gpiomem_open,
 	.release = aml_gpiomem_release,
 	.mmap = aml_gpiomem_mmap,
};

 /****************************************************************************
*
*   Probe and remove functions
*
***************************************************************************/
static int aml_gpiomem_probe(struct platform_device *pdev)
{
 	int err;
 	void *ptr_err;
 	struct device *dev = &pdev->dev;
 	struct resource *ioresource;
 	const char *str;
 	char tmp[64];
 	struct aml_gpiomem_instance *inst = NULL;

 	/* Allocate buffers and instance data */
 	inst = kzalloc(sizeof(struct aml_gpiomem_instance), GFP_KERNEL);

 	if (!inst) {
 		err = -ENOMEM;
 		goto failed_inst_alloc;
 	}

 	inst->dev = dev;

 	platform_set_drvdata(pdev, inst);

 	ioresource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
 	if (ioresource) {
 		inst->gpio_regs_phys = ioresource->start;
 	} else {
 		dev_err(inst->dev, "failed to get IO resource");
 		err = -ENOENT;
 		goto failed_get_resource;
 	}

 	err = of_property_read_string(dev->of_node, "dev_name", &str);
 	if (err) {
 		dev_err(inst->dev, "failed to get device name\n");
 		err = -ENOENT;
 		goto failed_get_device_name;
 	}
 	strncpy(inst->dev_name, str, 32);
 	inst->dev_name[31] = '\0';
 	/* Create character device entries */
 	sprintf(tmp, "aml-%s", inst->dev_name);
 	err = alloc_chrdev_region(&aml_gpiomem_devid,
			DEVICE_MINOR, 1, tmp);
 	if (err != 0) {
 		dev_err(inst->dev, "unable to allocate device number");
 		goto failed_alloc_chrdev;
 	}
 	cdev_init(&aml_gpiomem_cdev, &aml_gpiomem_fops);
 	aml_gpiomem_cdev.owner = THIS_MODULE;
 	err = cdev_add(&aml_gpiomem_cdev, aml_gpiomem_devid, 1);
 	if (err != 0) {
 		dev_err(inst->dev, "unable to register device");
 		goto failed_cdev_add;
 	}

 	/* Create sysfs entries */
 	aml_gpiomem_class = class_create(THIS_MODULE, tmp);
 	ptr_err = aml_gpiomem_class;
 	if (IS_ERR(ptr_err))
 		goto failed_class_create;

 	aml_gpiomem_dev = device_create(aml_gpiomem_class, NULL,
			aml_gpiomem_devid, NULL,
			inst->dev_name);
 	ptr_err = aml_gpiomem_dev;
 	if (IS_ERR(ptr_err))
 		goto failed_device_create;

 	inst->major = MAJOR(aml_gpiomem_devid);
 	gpiomem_instances[instance_num++] = inst;

 	dev_info(inst->dev, "Initialised: Registers at 0x%08lx\n",
	 		inst->gpio_regs_phys);

 	return 0;
failed_device_create:
 	class_destroy(aml_gpiomem_class);
failed_class_create:
 	cdev_del(&aml_gpiomem_cdev);
 	err = PTR_ERR(ptr_err);
failed_cdev_add:
 	unregister_chrdev_region(aml_gpiomem_devid, 1);
failed_alloc_chrdev:
failed_get_device_name:
failed_get_resource:
 	kfree(inst);
failed_inst_alloc:
 	dev_err(inst->dev, "could not load aml_gpiomem");
 	return err;
}

static int aml_gpiomem_remove(struct platform_device *pdev)
{
 	struct aml_gpiomem_instance *inst = platform_get_drvdata(pdev);
 	struct device *dev = inst->dev;

 	kfree(inst);
 	device_destroy(aml_gpiomem_class, aml_gpiomem_devid);
 	class_destroy(aml_gpiomem_class);
 	cdev_del(&aml_gpiomem_cdev);
 	unregister_chrdev_region(aml_gpiomem_devid, 1);

 	dev_info(dev, "GPIO mem driver removed - OK");
 	return 0;
}

/****************************************************************************
*
*   Register the driver with device tree
*
***************************************************************************/

static const struct of_device_id aml_gpiomem_of_match[] = {
 	{.compatible = "amlogic, gpiomem",},
 	{ /* sentinel */ },
};

MODULE_DEVICE_TABLE(of, aml_gpiomem_of_match);

static struct platform_driver aml_gpiomem_driver = {
 	.probe = aml_gpiomem_probe,
 	.remove = aml_gpiomem_remove,
 	.driver = {
 		.name = DRIVER_NAME,
 		.owner = THIS_MODULE,
 		.of_match_table = aml_gpiomem_of_match,
 	},
};

module_platform_driver(aml_gpiomem_driver);

MODULE_ALIAS("platform:gpiomem-aml");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("gpiomem driver for accessing GPIO from userspace");
MODULE_AUTHOR("Nick Xie <nick@khadas.com>");

