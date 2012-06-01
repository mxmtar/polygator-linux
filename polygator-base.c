/******************************************************************************/
/* polygator-base.c                                                           */
/******************************************************************************/

#include <linux/cdev.h>
// #include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/module.h>
// #include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/version.h>

#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <asm/io.h>
#if defined(CONFIG_COMPAT) && defined(HAVE_COMPAT_IOCTL) && (HAVE_COMPAT_IOCTL == 1)
#include <asm/compat.h>
#endif

#include "polygator/polygator-base.h"

MODULE_AUTHOR("Maksym Tarasevych <mxmtar@ukr.net>");
MODULE_DESCRIPTION("Polygator Linux base module");
MODULE_LICENSE("GPL");

static int polygator_major = 0;
module_param(polygator_major, int, 0);
MODULE_PARM_DESC(polygator_major, "Major number for Polygator Linux base module");

EXPORT_SYMBOL(polygator_board_register);
EXPORT_SYMBOL(polygator_board_unregister);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
	#define CLASS_DEV_CREATE(_class, _devt, _device, _name) device_create(_class, _device, _devt, NULL, "%s", _name)
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
	#define CLASS_DEV_CREATE(_class, _devt, _device, _name) device_create(_class, _device, _devt, _name)
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,15)
	#define CLASS_DEV_CREATE(_class, _devt, _device, _name) class_device_create(_class, NULL, _devt, _device, _name)
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,13)
	#define CLASS_DEV_CREATE(_class, _devt, _device, _name) class_device_create(_class, _devt, _device, _name)
#else
	#define CLASS_DEV_CREATE(_class, _devt, _device, _name) class_simple_device_add(_class, _devt, _device, _name)
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
	#define CLASS_DEV_DESTROY(_class, _devt) device_destroy(_class, _devt)
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,13)
	#define CLASS_DEV_DESTROY(_class, _devt) class_device_destroy(_class, _devt)
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,9)
	#define CLASS_DEV_DESTROY(_class, _devt) class_simple_device_remove(_devt)
#else
	#define CLASS_DEV_DESTROY(_class, _devt) class_simple_device_remove(_class, _devt)
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,13)
	static struct class *polygator_class = NULL;
#else
	static struct class_simple *polygator_class = NULL;
	#define class_create(_a, _b) class_simple_create(_a, _b)
	#define class_destroy(_a) class_simple_destroy(_a)
#endif

#define verbose(_fmt, _args...) printk(KERN_INFO "[%s] " _fmt, THIS_MODULE->name, ## _args)
#define log(_level, _fmt, _args...) printk(_level "[%s] %s:%d - %s(): " _fmt, THIS_MODULE->name, "polygator-base.c", __LINE__, __PRETTY_FUNCTION__, ## _args)
#define debug(_fmt, _args...) printk(KERN_DEBUG "[%s] %s:%d - %s(): " _fmt, THIS_MODULE->name, "polygator-base.c", __LINE__, __PRETTY_FUNCTION__, ## _args)

static struct cdev polygator_subsytem_cdev;

static struct polygator_board polygator_board_list[POLYGATOR_BOARD_MAXCOUNT];
static DEFINE_SPINLOCK(polygator_board_list_lock);

static struct file_operations polygator_subsytem_fops = {
	.owner   = THIS_MODULE,
// 	.open    = vinetic_open,
// 	.release = vinetic_release,
// 	.read    = vinetic_read,
// 	.write   = vinetic_write,
#if defined(HAVE_UNLOCKED_IOCTL)
// 	.unlocked_ioctl = vinetic_unlocked_ioctl,
#else
// 	.ioctl = vinetic_ioctl,
#endif
#if defined(CONFIG_COMPAT) && defined(HAVE_COMPAT_IOCTL) && (HAVE_COMPAT_IOCTL == 1)
// 	.compat_ioctl = vinetic_compat_ioctl,
#endif
// 	.llseek = vinetic_llseek,
};

static struct file_operations polygator_board_fops = {
	.owner   = THIS_MODULE,
// 	.open    = vinetic_open,
// 	.release = vinetic_release,
// 	.read    = vinetic_read,
// 	.write   = vinetic_write,
#if defined(HAVE_UNLOCKED_IOCTL)
// 	.unlocked_ioctl = vinetic_unlocked_ioctl,
#else
// 	.ioctl = vinetic_ioctl,
#endif
#if defined(CONFIG_COMPAT) && defined(HAVE_COMPAT_IOCTL) && (HAVE_COMPAT_IOCTL == 1)
// 	.compat_ioctl = vinetic_compat_ioctl,
#endif
// 	.llseek = vinetic_llseek,
};

struct polygator_board *polygator_board_register(struct module *owner, char *name, void * data)
{
	size_t i;
	char devname[POLYGATOR_BRDNAME_MAXLEN];
	int rc;
	int devno = 0;
	int slot_alloc = 0;
	struct polygator_board *brd = NULL;

	spin_lock(&polygator_board_list_lock);
	// check for name is not used
	for (i=0; i<POLYGATOR_BOARD_MAXCOUNT; i++)
	{
		if (!strcmp(polygator_board_list[i].name, name)) {
			spin_unlock(&polygator_board_list_lock);
			log(KERN_ERR, "\"%s\" already registered\n", name);
			goto polygator_board_register_error;
		}
	}
	// get free slot
	for (i=0; i<POLYGATOR_BOARD_MAXCOUNT; i++)
	{
		if (!polygator_board_list[i].data) {
			devno = MKDEV(polygator_major, i);
			polygator_board_list[i].devno = devno;
			snprintf(polygator_board_list[i].name, POLYGATOR_BRDNAME_MAXLEN, "%s", name);
			polygator_board_list[i].data = data;
			brd = &polygator_board_list[i];
			break;
		}
	}
	spin_unlock(&polygator_board_list_lock);
	
	if (!brd) {
		log(KERN_ERR, "\"%s\" - can't get free slot\n", name);
		goto polygator_board_register_error;
	}
	slot_alloc = 1;

	// Add char device to system
	cdev_init(&brd->cdev, &polygator_board_fops);
	brd->cdev.owner = owner;
	brd->cdev.ops = &polygator_board_fops;
	if ((rc = cdev_add(&brd->cdev, devno, 1)) < 0) {
		log(KERN_ERR, "\"%s\" - cdev_add() error=%d\n", name, rc);
		goto polygator_board_register_error;
	}
	snprintf(devname, POLYGATOR_BRDNAME_MAXLEN, "polygator!%s", name);
	CLASS_DEV_CREATE(polygator_class, devno, NULL, devname);

	verbose("\"%s\" registered\n", name);
	return brd;

polygator_board_register_error:
	if (slot_alloc) {
		spin_lock(&polygator_board_list_lock);
		for (i=0; i<POLYGATOR_BOARD_MAXCOUNT; i++)
		{
			if (!strcmp(polygator_board_list[i].name, name)) {
				polygator_board_list[i].name[0] = '\0';
				polygator_board_list[i].devno = 0;
				polygator_board_list[i].data = NULL;
				break;
			}
		}
		spin_unlock(&polygator_board_list_lock);
	}
	return NULL;
}

void polygator_board_unregister(struct polygator_board *brd)
{
	size_t i;

	CLASS_DEV_DESTROY(polygator_class, brd->devno);
	cdev_del(&brd->cdev);

	verbose("\"%s\" unregistered\n", brd->name);

	spin_lock(&polygator_board_list_lock);

	for (i=0; i<POLYGATOR_BOARD_MAXCOUNT; i++)
	{
		if (!strcmp(polygator_board_list[i].name, brd->name)) {
			polygator_board_list[i].name[0] = '\0';
			polygator_board_list[i].devno = 0;
			polygator_board_list[i].data = NULL;
			break;
		}
	}
	spin_unlock(&polygator_board_list_lock);

}

static int __init polygator_init(void)
{
	int rc;
	dev_t devno;
	int polygator_major_reg = 0;

	verbose("loading ...\n");

	// Registering polygator device class
	polygator_class = class_create(THIS_MODULE, "polygator");
	// Register char device region
	if (polygator_major) {
		devno = MKDEV(polygator_major, 0);
		rc = register_chrdev_region(devno, POLYGATOR_DEVICE_MAXCOUNT, "polygator");
	} else {
		rc = alloc_chrdev_region(&devno, 0, POLYGATOR_DEVICE_MAXCOUNT, "polygator");
		if(rc >= 0) polygator_major = MAJOR(devno);
	}
	if (rc < 0) {
		log(KERN_ERR, "register chrdev region error=%d\n", rc);
		goto polygator_init_error;
	}
	debug("polygator major=%d\n", polygator_major);
	polygator_major_reg = 1;

	// Add subsystem device
	cdev_init(&polygator_subsytem_cdev, &polygator_subsytem_fops);
	polygator_subsytem_cdev.owner = THIS_MODULE;
	polygator_subsytem_cdev.ops = &polygator_subsytem_fops;
	devno = MKDEV(polygator_major, 255);
	if ((rc = cdev_add(&polygator_subsytem_cdev, devno, 1)) < 0) {
		log(KERN_ERR, "\"subsystem\" - cdev_add() error=%d\n", rc);
		goto polygator_init_error;
	}
	CLASS_DEV_CREATE(polygator_class, devno, NULL, "polygator!subsystem");

	verbose("loaded successfull\n");
	return 0;

polygator_init_error:
	if (polygator_major_reg) unregister_chrdev_region(MKDEV(polygator_major, 0), POLYGATOR_DEVICE_MAXCOUNT);
	if (polygator_class) class_destroy(polygator_class);
	return rc;
}

static void __exit polygator_exit(void)
{
	// Destroy subsystem device
	CLASS_DEV_DESTROY(polygator_class, MKDEV(polygator_major, 255));
	// Unregister char device region
	unregister_chrdev_region(MKDEV(polygator_major, 0), POLYGATOR_DEVICE_MAXCOUNT);
	// Destroy polygator device class
	class_destroy(polygator_class);

	verbose("stopped\n");
}

module_init(polygator_init);
module_exit(polygator_exit);

/******************************************************************************/
/* end of polygator-base.c                                                    */
/******************************************************************************/
