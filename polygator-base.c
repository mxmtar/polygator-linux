/******************************************************************************/
/* polygator-base.c                                                           */
/******************************************************************************/

#include <linux/cdev.h>
// #include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/slab.h>
// #include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/version.h>
#include <linux/vmalloc.h>

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
EXPORT_SYMBOL(polygator_print_gsm_module_type);

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

struct subsystem_private_data {
	char buff[0x8000];
	size_t length;
};

static struct cdev polygator_subsytem_cdev;

static struct polygator_board *polygator_board_list[POLYGATOR_BOARD_MAXCOUNT];
static DEFINE_SPINLOCK(polygator_board_list_lock);

static int polygator_subsystem_open(struct inode *inode, struct file *filp)
{
	ssize_t res;
	size_t i;
	size_t len;

	struct subsystem_private_data *private_data;

	if (!(private_data = kmalloc(sizeof(struct subsystem_private_data), GFP_KERNEL))) {
		log(KERN_ERR, "can't get memory=%lu bytes\n", (unsigned long int)sizeof(struct subsystem_private_data));
		res = -ENOMEM;
		goto polygator_subsystem_open_error;
	}
// 	memset(private_data, 0, sizeof(struct subsystem_private_data));

	spin_lock(&polygator_board_list_lock);
	len = 0;
	for (i=0; i<POLYGATOR_BOARD_MAXCOUNT; i++)
	{
		if (polygator_board_list[i]) {
			len += sprintf(private_data->buff+len, "%s %s\r\n", polygator_board_list[i]->cdev->owner->name, polygator_board_list[i]->name);
		}
	}
	spin_unlock(&polygator_board_list_lock);

	private_data->length = len;

	filp->private_data = private_data;

	return 0;

polygator_subsystem_open_error:
	if (private_data) kfree(private_data);
	return res;
}

static int polygator_subsystem_release(struct inode *inode, struct file *filp)
{
	struct subsystem_private_data *private_data = filp->private_data;

	kfree(private_data);
	return 0;
}

static ssize_t polygator_subsystem_read(struct file *filp, char __user *buff, size_t count, loff_t *offp)
{
	size_t len;
	ssize_t res;
	struct subsystem_private_data *private_data = filp->private_data;

	res = (private_data->length > filp->f_pos)?(private_data->length - filp->f_pos):(0);

	if (res) {
		len = res;
		len = min(count, len);
		if (copy_to_user(buff, private_data->buff + filp->f_pos, len)) {
			res = -EINVAL;
			goto polygator_subsystem_read_end;
		}
		*offp = filp->f_pos + len;
	}

polygator_subsystem_read_end:
	return res;
}


#if 0
static int polygator_subsystem_generic_ioctl(struct file *filp, unsigned int cmd, unsigned long data)
{
	struct polygator_ioctl_get_board brd;
	int res = 0;
	void __user *argp = (void __user *)data;

	switch (cmd)
	{
		case POLYGATOR_GET_BOARD:
			if (copy_from_user(&brd, argp, sizeof(struct polygator_ioctl_get_board)))
				return -EINVAL;
			strcpy(brd.name, polygator_board_list[brd.no]->name);
			if (copy_to_user(argp, &brd, sizeof(struct polygator_ioctl_get_board)))
				return -EINVAL;
			break;
		default:
			res = -ENOIOCTLCMD;
			break;
	}
// polygator_subsystem_generic_ioctl_end:
	return res;
}

#if defined(HAVE_UNLOCKED_IOCTL)
static long polygator_subsystem_unlocked_ioctl(struct file *filp, unsigned int cmd, unsigned long data)
{
	return (long)polygator_subsystem_generic_ioctl(filp, cmd, data);
}
#else
static int polygator_subsystem_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long data)
{
	return (long)polygator_subsystem_generic_ioctl(filp, cmd, data);
}
#endif

#if defined(CONFIG_COMPAT) && defined(HAVE_COMPAT_IOCTL) && (HAVE_COMPAT_IOCTL == 1)
static long polygator_subsystem_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long data)
{
	return (long)polygator_subsystem_generic_ioctl(filp, cmd, data);
}
#endif
#endif

static struct file_operations polygator_subsytem_fops = {
	.owner   = THIS_MODULE,
	.open    = polygator_subsystem_open,
	.release = polygator_subsystem_release,
	.read    = polygator_subsystem_read,
// 	.write   = polygator_subsystem_write,
#if 0
#if defined(HAVE_UNLOCKED_IOCTL)
	.unlocked_ioctl = polygator_subsystem_unlocked_ioctl,
#else
	.ioctl = polygator_subsystem_ioctl,
#endif
#if defined(CONFIG_COMPAT) && defined(HAVE_COMPAT_IOCTL) && (HAVE_COMPAT_IOCTL == 1)
	.compat_ioctl = polygator_subsystem_compat_ioctl,
#endif
#endif
// 	.llseek = polygator_subsystem_llseek,
};

struct polygator_board *polygator_board_register(struct module *owner, char *name, struct cdev *cdev, struct file_operations *fops)
{
	size_t i;
	char devname[POLYGATOR_BRDNAME_MAXLEN];
	int rc;
	int devno = -1;
	struct polygator_board *brd;

	if (!(brd = kmalloc(sizeof(struct polygator_board), GFP_KERNEL))) {
		log(KERN_ERR, "\"%s\" - can't get memory for struct polygator_board\n", name);
		goto polygator_board_register_error;
	}

	spin_lock(&polygator_board_list_lock);
	// check for name is not used
	for (i=0; i<POLYGATOR_BOARD_MAXCOUNT; i++)
	{
		if ((polygator_board_list[i]) && (!strcmp(polygator_board_list[i]->name, name))) {
			spin_unlock(&polygator_board_list_lock);
			log(KERN_ERR, "\"%s\" already registered\n", name);
			goto polygator_board_register_error;
		}
	}
	// get free slot
	for (i=0; i<POLYGATOR_BOARD_MAXCOUNT; i++)
	{
		if (!polygator_board_list[i]) {
			devno = MKDEV(polygator_major, i);
			polygator_board_list[i] = brd;
			brd->devno = devno;
			snprintf(brd->name, POLYGATOR_BRDNAME_MAXLEN, "%s", name);
			break;
		}
	}
	spin_unlock(&polygator_board_list_lock);

	if (devno < 0) {
		log(KERN_ERR, "\"%s\" - can't get free slot\n", name);
		goto polygator_board_register_error;
	}

	// Add char device to system
	cdev_init(cdev, fops);
	cdev->owner = owner;
	cdev->ops = fops;
	brd->cdev = cdev;
	if ((rc = cdev_add(cdev, devno, 1)) < 0) {
		log(KERN_ERR, "\"%s\" - cdev_add() error=%d\n", name, rc);
		goto polygator_board_register_error;
	}
	snprintf(devname, POLYGATOR_BRDNAME_MAXLEN, "polygator!%s", name);
	CLASS_DEV_CREATE(polygator_class, devno, NULL, devname);

	verbose("\"%s\" registered\n", name);
	return brd;

polygator_board_register_error:
	if (devno >= 0) {
		spin_lock(&polygator_board_list_lock);
		for (i=0; i<POLYGATOR_BOARD_MAXCOUNT; i++)
		{
			if ((polygator_board_list[i]) && (!strcmp(polygator_board_list[i]->name, name))) {
				polygator_board_list[i] = NULL;
				break;
			}
		}
		spin_unlock(&polygator_board_list_lock);
	}
	if (brd) kfree(brd);
	return NULL;
}

void polygator_board_unregister(struct polygator_board *brd)
{
	size_t i;

	CLASS_DEV_DESTROY(polygator_class, brd->devno);
	cdev_del(brd->cdev);

	verbose("\"%s\" unregistered\n", brd->name);

	spin_lock(&polygator_board_list_lock);

	for (i=0; i<POLYGATOR_BOARD_MAXCOUNT; i++)
	{
		if ((polygator_board_list[i]) && (!strcmp(polygator_board_list[i]->name, brd->name))) {
			kfree(polygator_board_list[i]);
			polygator_board_list[i] = NULL;
			break;
		}
	}
	spin_unlock(&polygator_board_list_lock);

}

char *polygator_print_gsm_module_type(int type)
{
	switch (type)
	{
		case POLYGATOR_MODULE_TYPE_SIM300: return "SIM300";
		case POLYGATOR_MODULE_TYPE_SIM900: return "SIM900";
		case POLYGATOR_MODULE_TYPE_M10: return "M10";
		case POLYGATOR_MODULE_TYPE_SIM5215: return "SIM5215";
		default: return "UNKNOWN";
	}
}

static int __init polygator_init(void)
{
	size_t i;
	int rc;
	dev_t devno;
	int polygator_major_reg = 0;

	verbose("loading ...\n");

	for (i=0; i<POLYGATOR_BOARD_MAXCOUNT; i++)
		polygator_board_list[i] = NULL;

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
