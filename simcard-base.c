/******************************************************************************/
/* simcard-base.c                                                             */
/******************************************************************************/

#include <linux/kobject.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/types.h>
#include <linux/version.h>
#include <linux/vmalloc.h>

#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <asm/io.h>
#if defined(CONFIG_COMPAT) && defined(HAVE_COMPAT_IOCTL) && (HAVE_COMPAT_IOCTL == 1)
#include <asm/compat.h>
#endif

#include "polygator/simcard-base.h"
#include "polygator/simcard-def.h"

MODULE_AUTHOR("Maksym Tarasevych <mxmtar@gmail.com>");
MODULE_DESCRIPTION("Polygator Linux module for virtual SIM card support");
MODULE_LICENSE("GPL");

static int simcard_major = 0;
module_param(simcard_major, int, 0);
MODULE_PARM_DESC(simcard_major, "Major number for Polygator Linux simcard module");

EXPORT_SYMBOL(simcard_device_register);
EXPORT_SYMBOL(simcard_device_unregister);

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
	static struct class *simcard_class = NULL;
#else
	static struct class_simple *simcard_class = NULL;
	#define class_create(_a, _b) class_simple_create(_a, _b)
	#define class_destroy(_a) class_simple_destroy(_a)
#endif

#define verbose(_fmt, _args...) printk(KERN_INFO "[polygator-%s] " _fmt, THIS_MODULE->name, ## _args)
#define log(_level, _fmt, _args...) printk(_level "polygator-%s] %s:%d - %s(): " _fmt, THIS_MODULE->name, "simcard-base.c", __LINE__, __PRETTY_FUNCTION__, ## _args)
#define debug(_fmt, _args...) printk(KERN_DEBUG "[polygator-%s] %s:%d - %s(): " _fmt, THIS_MODULE->name, "simcard-base.c", __LINE__, __PRETTY_FUNCTION__, ## _args)

static struct simcard_device *simcard_device_list[SIMCARD_DEVICE_MAXCOUNT];
static DEFINE_MUTEX(simcard_device_list_lock);

static void simcard_poll_proc(unsigned long addr)
{
	int reset;
	struct simcard_device *sim = (struct simcard_device *)addr;

	spin_lock(&sim->lock);

	// reset
	reset = sim->is_reset_request(sim->data);
	if (reset != sim->reset_state) {
		// fill simcard reset container
		sim->reset.header.type = SIMCARD_CONTAINER_TYPE_RESET;
		sim->reset.header.length = sizeof(sim->reset.container.reset);
		// store data
		sim->reset.container.reset = reset;
		// set reset status bit
		sim->read_status.bits.reset = 1;
		// clear data status bit
		sim->read_status.bits.data = 0;
		// do after reset action
		if (reset) {
			sim->do_after_reset(sim->data);
		}
	}
	sim->reset_state = reset;

	// read
	if (reset && sim->is_read_ready(sim->data)) {
		// reset simcard data container
		sim->command.header.type = SIMCARD_CONTAINER_TYPE_DATA;
		sim->command.header.length = 0;
		// store data
		while ((sim->command.header.length < SIMCARD_MAX_DATA_LENGTH) && (sim->is_read_ready(sim->data))) {
			sim->command.container.data[sim->command.header.length++] = sim->read(sim->data);
		}
		// set data status bit
		sim->read_status.bits.data = 1;
	}

	if (sim->read_status.full) {
		wake_up_interruptible(&sim->read_waitq);
		wake_up_interruptible(&sim->poll_waitq);
	}

	spin_unlock(&sim->lock);

	if (sim->poll) {
		mod_timer(&sim->poll_timer, jiffies + 1);
	}
}

static int simcard_open(struct inode *inode, struct file *filp)
{
	size_t usage;
	struct simcard_device *sim;

	sim = container_of(inode->i_cdev, struct simcard_device, cdev);
	filp->private_data = sim;

	spin_lock_bh(&sim->lock);
	usage = sim->usage++;
	if (!usage) {
		sim->read_status.full = 0;
		sim->write_status.bits.speed = 1;
		sim->poll = 1;
	}
	spin_unlock_bh(&sim->lock);

	if (!usage) {
		sim->poll_timer.function = simcard_poll_proc;
		sim->poll_timer.data = (unsigned long)sim;
		sim->poll_timer.expires = jiffies + 1;
		add_timer(&sim->poll_timer);
	}
	
	return 0;
}

static int simcard_release(struct inode *inode, struct file *filp)
{
	size_t usage;
	struct simcard_device *sim = filp->private_data;

	spin_lock_bh(&sim->lock);
	usage = --sim->usage;
	if (!usage) {
		sim->poll = 0;
	}
	spin_unlock_bh(&sim->lock);

	if (!usage) {
		del_timer_sync(&sim->poll_timer);
	}

	return 0;
}

static ssize_t simcard_read(struct file *filp, char __user *buff, size_t count, loff_t *offp)
{
	ssize_t res;
	size_t length;
	struct simcard_data data;
	struct simcard_device *sim = filp->private_data;

	res = 0;
	length = 0;

	spin_lock_bh(&sim->lock);

	for (;;) {
		if (sim->read_status.full) {
			break;
		}

		if (filp->f_flags & O_NONBLOCK) {
			spin_unlock_bh(&sim->lock);
			res = -EAGAIN;
			goto simcard_read_end;
		}
		// sleeping
		spin_unlock_bh(&sim->lock);
		if ((res = wait_event_interruptible(sim->read_waitq, sim->read_status.full))) {
			goto simcard_read_end;
		}
		spin_lock_bh(&sim->lock);
	}

	// select container
	if (sim->read_status.bits.reset) {
		length = sizeof(sim->reset.header) + sim->reset.header.length;
		memcpy(&data, &sim->reset, length);
		sim->read_status.bits.reset = 0;
	} else if (sim->read_status.bits.data) {
		length = sizeof(sim->command.header) + sim->command.header.length;
		memcpy(&data, &sim->command, length);
		sim->read_status.bits.data = 0;
	} else {
		sim->read_status.full = 0;
		spin_unlock_bh(&sim->lock);
		res = -EAGAIN;
		goto simcard_read_end;
	}

	spin_unlock_bh(&sim->lock);

	length = min(length, count);
	if (copy_to_user(buff, &data, length)) {
		res = -EFAULT;
		goto simcard_read_end;
	}

	res = length;

simcard_read_end:
	return res;
}

static ssize_t simcard_write(struct file *filp, const char __user *buff, size_t count, loff_t *offp)
{
	ssize_t res;
	size_t length;
	size_t i;
	struct simcard_data data;

	struct simcard_device *sim = filp->private_data;

	res = 0;

	length = sizeof(struct simcard_data);
	length = min(length, count);
	if (copy_from_user(&data, buff, length)) {
		res = -EFAULT;
		goto simcard_write_end;
	}

	spin_lock_bh(&sim->lock);

	for (;;) {
		if (sim->write_status.full) {
			break;
		}

		if (filp->f_flags & O_NONBLOCK) {
			spin_unlock_bh(&sim->lock);
			res = -EAGAIN;
			goto simcard_write_end;
		}
		// sleeping
		spin_unlock_bh(&sim->lock);
		if ((res = wait_event_interruptible(sim->write_waitq, sim->write_status.full))) {
			goto simcard_write_end;
		}
		spin_lock_bh(&sim->lock);
	}

	switch (data.header.type) {
		case SIMCARD_CONTAINER_TYPE_DATA:
			i = 0;
			while (data.header.length) {
				if (sim->is_write_ready(sim->data)) {
					sim->write(sim->data, data.container.data[i++]);
					data.header.length--;
				}
			}
			break;
		case SIMCARD_CONTAINER_TYPE_SPEED:
			sim->set_speed(sim->data, data.container.speed);
			break;
		default:
			spin_unlock_bh(&sim->lock);
			res = -EINVAL;
			goto simcard_write_end;
	}

	spin_unlock_bh(&sim->lock);

	res = length;

simcard_write_end:
	return res;
}

static unsigned int simcard_poll(struct file *filp, struct poll_table_struct *wait_table)
{
	unsigned int res;
	struct simcard_device *sim = filp->private_data;

	res = 0;

	poll_wait(filp, &sim->poll_waitq, wait_table);

	spin_lock_bh(&sim->lock);

	if (sim->read_status.full) {
		res |= POLLIN | POLLRDNORM;
	}

	if (sim->write_status.full) {
		res |= POLLOUT | POLLWRNORM;
	}

	spin_unlock_bh(&sim->lock);

	return res;
}

static struct file_operations simcard_fops = {
	.owner		= THIS_MODULE,
	.open		= simcard_open,
	.release	= simcard_release,
	.read		= simcard_read,
	.write		= simcard_write,
	.poll		= simcard_poll,
};

struct simcard_device *simcard_device_register(struct module *owner,
							void *data,
							u_int8_t (* read)(void *data),
							void (* write)(void *data, u_int8_t value),
							int (* is_read_ready)(void *data),
							int (* is_write_ready)(void *data),
							int (* is_reset_request)(void *data),
							void (* set_speed)(void *data, int speed),
							void (* do_after_reset)(void *data))
{
	struct simcard_device *sim;
	size_t i;
	int rc;
	char devname[64];
	int devno = 0;
	int slot_alloc = 0;

	if (!(sim = kmalloc(sizeof(struct simcard_device), GFP_KERNEL))) {
		log(KERN_ERR, "can't get memory for struct simcard_device\n");
		goto simcard_device_register_error;
	}
	memset(sim, 0, sizeof(struct simcard_device));

	mutex_lock(&simcard_device_list_lock);
	// get free slot
	for (i=0; i<SIMCARD_DEVICE_MAXCOUNT; i++) {
		if (!simcard_device_list[i]) {
			sim->devno = devno = MKDEV(simcard_major, i);
			simcard_device_list[i] = sim;
			break;
		}
	}
	mutex_unlock(&simcard_device_list_lock);
	
	if (!devno) {
		log(KERN_ERR, "can't get free slot for simcard\n");
		goto simcard_device_register_error;
	}
	slot_alloc = 1;
	
	// init simcard data
	spin_lock_init(&sim->lock);
	init_waitqueue_head(&sim->poll_waitq);
	init_waitqueue_head(&sim->read_waitq);
	init_waitqueue_head(&sim->write_waitq);
	init_timer(&sim->poll_timer);
	sim->poll = 0;

	// set data
	sim->data = data;

	if (!read) {
		log(KERN_ERR, "reset callback not present\n");
		goto simcard_device_register_error;
	}
	sim->read = read;

	if (!write) {
		log(KERN_ERR, "write callback not present\n");
		goto simcard_device_register_error;
	}
	sim->write = write;

	if (!is_read_ready) {
		log(KERN_ERR, "is_read_ready callback not present\n");
		goto simcard_device_register_error;
	}
	sim->is_read_ready = is_read_ready;

	if (!is_write_ready) {
		log(KERN_ERR, "is_write_ready callback not present\n");
		goto simcard_device_register_error;
	}
	sim->is_write_ready = is_write_ready;

	if (!is_reset_request) {
		log(KERN_ERR, "is_reset_request callback not present\n");
		goto simcard_device_register_error;
	}
	sim->is_reset_request = is_reset_request;

	if (!set_speed) {
		log(KERN_ERR, "set_speed callback not present\n");
		goto simcard_device_register_error;
	}
	sim->set_speed = set_speed;

	if (!do_after_reset) {
		log(KERN_ERR, "do_after_reset callback not present\n");
		goto simcard_device_register_error;
	}
	sim->do_after_reset = do_after_reset;

	// Add char device to system
	cdev_init(&sim->cdev, &simcard_fops);
	sim->cdev.owner = owner;
	sim->cdev.ops = &simcard_fops;
	if ((rc = cdev_add(&sim->cdev, devno, 1)) < 0) {
		log(KERN_ERR, "cdev_add() error=%d\n", rc);
		goto simcard_device_register_error;
	}
	snprintf(devname, sizeof(devname), "polygator!sim%d", MINOR(sim->devno));
	if (!(sim->device = CLASS_DEV_CREATE(simcard_class, devno, NULL, devname))) {
		log(KERN_ERR, "class_dev_create() error\n");
		goto simcard_device_register_error;
	}

	return sim;

simcard_device_register_error:
	if (slot_alloc) {
		mutex_lock(&simcard_device_list_lock);
		simcard_device_list[MINOR(sim->devno)] = NULL;
		mutex_unlock(&simcard_device_list_lock);
	}
	if(sim) kfree(sim);
	return NULL;
}

void simcard_device_unregister(struct simcard_device *sim)
{
	CLASS_DEV_DESTROY(simcard_class, sim->devno);
	cdev_del(&sim->cdev);

	mutex_lock(&simcard_device_list_lock);
	simcard_device_list[MINOR(sim->devno)] = NULL;
	mutex_unlock(&simcard_device_list_lock);

	kfree(sim);
}

static int __init simcard_init(void)
{
	size_t i;
	dev_t devno;
	int simcard_major_reg = 0;
	int rc = -1;

	verbose("loading ...\n");

	// Init simcard device list
	for (i=0; i<SIMCARD_DEVICE_MAXCOUNT; i++) {
		simcard_device_list[i] = NULL;
	}

	// Registering simcard device class
	if (!(simcard_class = class_create(THIS_MODULE, "simcard"))) {
		log(KERN_ERR, "class_create() error\n");
		goto simcard_init_error;
	}
	// Register simcard char device region
	if (simcard_major) {
		devno = MKDEV(simcard_major, 0);
		rc = register_chrdev_region(devno, SIMCARD_DEVICE_MAXCOUNT, "simcard");
	} else {
		rc = alloc_chrdev_region(&devno, 0, SIMCARD_DEVICE_MAXCOUNT, "simcard");
		if(rc >= 0) simcard_major = MAJOR(devno);
	}
	if (rc < 0) {
		log(KERN_ERR, "register chrdev region error=%d\n", rc);
		goto simcard_init_error;
	}
	simcard_major_reg = 1;

	verbose("loaded successfull\n");
	return 0;

simcard_init_error:
	if (simcard_major_reg) unregister_chrdev_region(MKDEV(simcard_major, 0), SIMCARD_DEVICE_MAXCOUNT);
	if (simcard_class) class_destroy(simcard_class);
	return rc;
}

static void __exit simcard_exit(void)
{
	// Unregister simcard char device region
	unregister_chrdev_region(MKDEV(simcard_major, 0), SIMCARD_DEVICE_MAXCOUNT);
	// Destroy simcard device class
	class_destroy(simcard_class);

	verbose("stopped\n");
}

module_init(simcard_init);
module_exit(simcard_exit);

/******************************************************************************/
/* end of simcard-base.c                                                      */
/******************************************************************************/
