/******************************************************************************/
/* vinetic-base.c                                                             */
/******************************************************************************/

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
#include <linux/version.h>
#include <linux/vmalloc.h>

#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <asm/io.h>
#if defined(CONFIG_COMPAT) && defined(HAVE_COMPAT_IOCTL) && (HAVE_COMPAT_IOCTL == 1)
#include <asm/compat.h>
#endif

#include "polygator/vinetic-base.h"
#include "polygator/vinetic-def.h"
#include "polygator/vinetic-ioctl.h"

MODULE_AUTHOR("Maksym Tarasevych <mxmtar@ukr.net>");
MODULE_DESCRIPTION("Polygator Linux module for VINETIC support");
MODULE_LICENSE("GPL");

static int vinetic_major = 0;
module_param(vinetic_major, int, 0);
MODULE_PARM_DESC(vinetic_major, "Major Polygator linux module for VINETIC support");

EXPORT_SYMBOL(vinetic_device_register);
EXPORT_SYMBOL(vinetic_device_unregister);
EXPORT_SYMBOL(vinetic_rtp_channel_register);
EXPORT_SYMBOL(vinetic_rtp_channel_unregister);

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
	static struct class *vinetic_class = NULL;
#else
	static struct class_simple *vinetic_class = NULL;
	#define class_create(_a, _b) class_simple_create(_a, _b)
	#define class_destroy(_a) class_simple_destroy(_a)
#endif

#define verbose(_fmt, _args...) printk(KERN_INFO "[pg-%s] " _fmt, THIS_MODULE->name, ## _args)
#define log(_level, _fmt, _args...) printk(_level "[pg-%s] %s:%d - %s(): " _fmt, THIS_MODULE->name, "vinetic-base.c", __LINE__, __PRETTY_FUNCTION__, ## _args)
#define debug(_fmt, _args...) printk(KERN_DEBUG "[pg-%s] %s:%d - %s(): " _fmt, THIS_MODULE->name, "vinetic-base.c", __LINE__, __PRETTY_FUNCTION__, ## _args)

#define VINETIC_WAIT_COUNT 50
#define VINETIC_WAIT_TIMEOUT 2

struct vinetic_device {
	char name[VINETIC_DEVNAME_MAXLEN];
	int devno;
	u_int32_t type;
	void *data;
};

static struct vinetic_device vinetic_device_list[VINETIC_DEVICE_MAXCOUNT];
static DEFINE_SPINLOCK(vinetic_device_list_lock);

static void vinetic_poll(unsigned long addr)
{
	union vin_cmd cmd;

	u_int16_t res;

	size_t vop_pkt_len;

	size_t wait_count;

	u_int16_t *datap;

	struct vinetic_rtp_channel *rtp;
	struct vinetic *vin = (struct vinetic *)addr;

	// lock vinetic
	spin_lock(&vin->lock);

	// check data for read
	for (wait_count=0; wait_count<VINETIC_WAIT_COUNT; wait_count++)
	{
		if (!vin->is_not_ready(vin->cbdata)) break;
		udelay(VINETIC_WAIT_TIMEOUT);
	}
	if (wait_count == VINETIC_WAIT_COUNT) {
		debug("%s: timeout\n", vin->name);
		goto vinetic_poll_end;
	}
	vin->write_nwd(vin->cbdata, VIN_rOBXML);
	for (wait_count=0; wait_count<VINETIC_WAIT_COUNT; wait_count++)
	{
		if (!vin->is_not_ready(vin->cbdata)) break;
		udelay(VINETIC_WAIT_TIMEOUT);
	}
	if (wait_count == VINETIC_WAIT_COUNT) goto vinetic_poll_end;
	res = vin->read_eom(vin->cbdata);
	vin->pdata_size = res & 0xff;
	vin->cdata_size = (res >> 8) & 0x1f;

	// read voice packet from mailbox
	if (vin->pdata_size) {
		// write short commands rPOBX
		for (wait_count=0; wait_count<VINETIC_WAIT_COUNT; wait_count++)
		{
			if (!vin->is_not_ready(vin->cbdata)) break;
			udelay(VINETIC_WAIT_TIMEOUT);
		}
		if (wait_count == VINETIC_WAIT_COUNT)
			goto vinetic_poll_end;
		vin->write_nwd(vin->cbdata, VIN_rPOBX);
// 		debug("%s: 1 vin->pdata_size=%lu, vop_pkt_len=%lu\n", vin->name, (long unsigned int)vin->pdata_size, (long unsigned int)vop_pkt_len);
		while (vin->pdata_size)
		{
			// read vop/evt packet first part
			vin->pdata_size--;
			for (wait_count=0; wait_count<VINETIC_WAIT_COUNT; wait_count++)
			{
				if (!vin->is_not_ready(vin->cbdata)) break;
				udelay(VINETIC_WAIT_TIMEOUT);
			}
			if (wait_count == VINETIC_WAIT_COUNT) {
				debug("%s: timeout\n", vin->name);
				goto vinetic_poll_end;
			}
			cmd.parts.first.full = vin->read_nwd(vin->cbdata);
// 			debug("cmd.parts.first.full=%04x\n", cmd.parts.first.full);
			// read vop/evt packet second part
			vin->pdata_size--;
			for (wait_count=0; wait_count<VINETIC_WAIT_COUNT; wait_count++)
			{
				if (!vin->is_not_ready(vin->cbdata)) break;
				udelay(VINETIC_WAIT_TIMEOUT);
			}
			if (wait_count == VINETIC_WAIT_COUNT) {
				debug("%s: timeout\n", vin->name);
				goto vinetic_poll_end;
			}
			cmd.parts.second.full = vin->read_nwd(vin->cbdata);
// 			debug("cmd.parts.second.full=%04x\n", cmd.parts.second.full);
			// get packet data length
			vop_pkt_len = cmd.parts.second.vop.bits.length;
			if (vop_pkt_len > 253) {
				debug("%s: wrong voice packet length=%lu\n", vin->name, (long unsigned int)vop_pkt_len);
				goto vinetic_poll_end;
			}
			// sort packet by channel
			rtp = vin->rtp_channels[cmd.parts.first.bits.chan];
			if (rtp) {

				spin_lock(&rtp->lock);

				rtp->recv_slot[rtp->recv_slot_write].length = vop_pkt_len * 2;
				if(cmd.parts.second.vop.bits.odd) rtp->recv_slot[rtp->recv_slot_write].length--;
				datap = rtp->recv_slot[rtp->recv_slot_write].data;
// 				debug("%s: 3 vin->pdata_size=%lu, vop_pkt_len=%lu\n", vin->name, (long unsigned int)vin->pdata_size, (long unsigned int)vop_pkt_len);
				while (vin->pdata_size && vop_pkt_len)
				{
					vin->pdata_size--;
					vop_pkt_len--;
					for (wait_count=0; wait_count<VINETIC_WAIT_COUNT; wait_count++)
					{
						if (!vin->is_not_ready(vin->cbdata)) break;
						udelay(VINETIC_WAIT_TIMEOUT);
					}
					if (wait_count == VINETIC_WAIT_COUNT) {
						spin_unlock(&rtp->lock);
						debug("%s: timeout\n", vin->name);
						goto vinetic_poll_end;
					}
					if (vin->pdata_size)
						*datap++ = vin->read_nwd(vin->cbdata);
					else
						*datap++ = vin->read_eom(vin->cbdata);
// 					debug("%s: 4 vin->pdata_size=%lu, vop_pkt_len=%lu\n", vin->name, (long unsigned int)vin->pdata_size, (long unsigned int)vop_pkt_len);
				}
				// adjust packet slot index
				rtp->recv_slot_count++;
				if (rtp->recv_slot_count >= VINETIC_PACKETSLOT_MAXCOUNT) {
					rtp->recv_slot_count = VINETIC_PACKETSLOT_MAXCOUNT;
					rtp->recv_slot_read++;
					if (rtp->recv_slot_read >= VINETIC_PACKETSLOT_MAXCOUNT)
						rtp->recv_slot_read = 0;
				}
				rtp->recv_slot_write++;
				if (rtp->recv_slot_write >= VINETIC_PACKETSLOT_MAXCOUNT)
					rtp->recv_slot_write = 0;

				spin_unlock(&rtp->lock);

				// wake poll waitqueue
				wake_up_interruptible(&rtp->poll_waitq);

			} else {
				// unknown data packet - flush out mailbox
				while (vin->pdata_size && vop_pkt_len)
				{
					vin->pdata_size--;
					vop_pkt_len--;
					for (wait_count=0; wait_count<VINETIC_WAIT_COUNT; wait_count++)
					{
						if (!vin->is_not_ready(vin->cbdata)) break;
						udelay(VINETIC_WAIT_TIMEOUT);
					}
					if (wait_count == VINETIC_WAIT_COUNT) {
						debug("%s: timeout\n", vin->name);
						goto vinetic_poll_end;
					}
					if (vin->pdata_size)
						vin->read_nwd(vin->cbdata);
					else
						vin->read_eom(vin->cbdata);
				}
			}
// 			debug("%s: 2 vin->pdata_size=%lu, vop_pkt_len=%lu\n", vin->name, (long unsigned int)vin->pdata_size, (long unsigned int)vop_pkt_len);
		}
	}

	// read command data from mailbox
	
	if (vin->cdata_size) {
// 		debug("cdata_size=%lu\n", vin->cdata_size);
		// write short commands rCOBX
		for (wait_count=0; wait_count<VINETIC_WAIT_COUNT; wait_count++)
		{
			if (!vin->is_not_ready(vin->cbdata)) break;
			udelay(VINETIC_WAIT_TIMEOUT);
		}
		if (wait_count == VINETIC_WAIT_COUNT) {
			debug("%s: timeout\n", vin->name);
			goto vinetic_poll_end;
			}
		vin->write_nwd(vin->cbdata, VIN_rCOBX);
		// read rest data
		vin->read_cbox_length = vin->cdata_size;
		datap = vin->read_cbox_data;
		while (vin->cdata_size)
		{
			vin->cdata_size--;
			for (wait_count=0; wait_count<VINETIC_WAIT_COUNT; wait_count++)
			{
				if (!vin->is_not_ready(vin->cbdata)) break;
				udelay(VINETIC_WAIT_TIMEOUT);
			}
			if (wait_count == VINETIC_WAIT_COUNT) {
				debug("%s: timeout\n", vin->name);
				goto vinetic_poll_end;
			}
			if (vin->cdata_size)
				*datap++ = vin->read_nwd(vin->cbdata);
			else
				*datap++ = vin->read_eom(vin->cbdata);
		}
		wake_up_interruptible(&vin->read_cbox_waitq);
	}

	// check free mailbox space
	for (wait_count=0; wait_count<VINETIC_WAIT_COUNT; wait_count++)
	{
		if (!vin->is_not_ready(vin->cbdata)) break;
		udelay(VINETIC_WAIT_TIMEOUT);
	}
	if (wait_count == VINETIC_WAIT_COUNT) {
		debug("%s: timeout\n", vin->name);
		goto vinetic_poll_end;
	}
	vin->write_nwd(vin->cbdata, VIN_rFIBXMS);
	for (wait_count=0; wait_count<VINETIC_WAIT_COUNT; wait_count++)
	{
		if (!vin->is_not_ready(vin->cbdata)) break;
		udelay(VINETIC_WAIT_TIMEOUT);
	}
	if (wait_count == VINETIC_WAIT_COUNT) goto vinetic_poll_end;
	res = vin->read_eom(vin->cbdata);
	vin->free_pbox_space = res & 0xff;
	if (vin->free_pbox_space)
		wake_up_interruptible(&vin->free_pbox_waitq);
	vin->free_cbox_space = (res >> 8) & 0xff;
	if (vin->free_cbox_space)
		wake_up_interruptible(&vin->free_cbox_waitq);

vinetic_poll_end:
	spin_unlock(&vin->lock);
	if (vin->poll)
		mod_timer(&vin->poll_timer, jiffies + 1);
}

static int vinetic_open(struct inode *inode, struct file *filp)
{
	struct vinetic *vin;

	vin = container_of(inode->i_cdev, struct vinetic, cdev);
	filp->private_data = vin;
	return 0;
}

static int vinetic_release(struct inode *inode, struct file *filp)
{
	struct vinetic *vin;

	vin = container_of(inode->i_cdev, struct vinetic, cdev);

	spin_lock_bh(&vin->lock);
	vin->poll = 0;
	spin_unlock_bh(&vin->lock);

	del_timer_sync(&vin->poll_timer);

	return 0;
}

static ssize_t vinetic_read(struct file *filp, char __user *buff, size_t count, loff_t *offp)
{
	u_int16_t data[32];
	size_t wait_count;
	size_t cnt;
	ssize_t res;

	union vin_cmd cmd;
	union vin_cmd_short cmd_short;

	struct vinetic *vin = filp->private_data;

	cmd.full = filp->f_pos & 0xffffffff;
// 	debug("cmd.full=0x%08x\n", cmd.full);

	// check for is read command
	if (cmd.parts.first.bits.rw != VIN_READ) {
		spin_lock_bh(&vin->lock);
		*offp = 0;
		spin_unlock_bh(&vin->lock);
		log(KERN_ERR, "\"%s\": is write command=0x%08x\n", vin->name, cmd.full);
		res = -EINVAL;
		goto vinetic_read_end;
	}
	
	if (cmd.parts.first.bits.sc) {
		// short command
		cmd_short.full = cmd.parts.first.full;
		spin_lock_bh(&vin->lock);
		// write command to vinetic
		for (wait_count=0; wait_count<VINETIC_WAIT_COUNT; wait_count++)
		{
			if (!vin->is_not_ready(vin->cbdata)) break;
			udelay(VINETIC_WAIT_TIMEOUT);
		}
		if (wait_count == VINETIC_WAIT_COUNT) {
			*offp = 0;
			spin_unlock_bh(&vin->lock);
			res = -EIO;
			goto vinetic_read_end;
		}
		vin->write_nwd(vin->cbdata, cmd_short.full);
		// get actual data length to read
		switch ((cmd_short.full >> 4) & 0x1f)
		{
			case VIN_SH_CMD_CODE_rIR:
				res = 1;
				break;
			case VIN_SH_CMD_CODE_rSR:
			case VIN_SH_CMD_CODE_rI_SR:
				if (cmd_short.bits.bc) {
					res = 24;
					break;
				} else {
					switch (cmd_short.bits.chan)
					{
						case 0: case 1: case 2: case 3:
							res = 4;
							break;
						case 4: case 5: case 6: case 7:
							res = 2;
							break;
						default:
							*offp = 0;
							spin_unlock_bh(&vin->lock);
							log(KERN_ERR, "\"%s\": rSR/rI_SR wrong channel index=%u\n", vin->name, cmd_short.bits.chan);
							res = -EINVAL;
							goto vinetic_read_end;
					}
				}
				break;
			case VIN_SH_CMD_CODE_rSRS:
			case VIN_SH_CMD_CODE_rI_SRS:
				if (cmd_short.bits.bc)
					res = 16;
				else
					res = 2;
				break;
			case VIN_SH_CMD_CODE_rHWSR:
			case VIN_SH_CMD_CODE_rI_HWSR:
				res = 2;
				break;
			case VIN_SH_CMD_CODE_rBXSR:
			case VIN_SH_CMD_CODE_rI_BXSR:
				res = 2;
				break;
			case VIN_SH_CMD_CODE_rSRGPIO:
			case VIN_SH_CMD_CODE_rI_SRGPIO:
				res =1;
				break;
			case VIN_SH_CMD_CODE_rFIBXMS:
				res = 1;
				break;
			case VIN_SH_CMD_CODE_rOBXML:
				res = 1;
				break;
			case VIN_SH_CMD_CODE_rPOBX:
				*offp = 0;
				spin_unlock_bh(&vin->lock);
				log(KERN_ERR, "\"%s\": rPOBX unsupported from userspace\n", vin->name);
				res = -EINVAL;
				goto vinetic_read_end;
			case VIN_SH_CMD_CODE_rCOBX:
				*offp = 0;
				spin_unlock_bh(&vin->lock);
				log(KERN_ERR, "\"%s\": rCOBX unsupported from userspace\n", vin->name);
				res = -EINVAL;
				goto vinetic_read_end;
			default:
				*offp = 0;
				spin_unlock_bh(&vin->lock);
				log(KERN_ERR, "\"%s\": unknown short command=0x%04x\n", vin->name, cmd_short.full);
				res = -EINVAL;
				goto vinetic_read_end;
		}
		// read data from vinetic
		for (cnt=0; cnt<res; cnt++)
		{
			for (wait_count=0; wait_count<VINETIC_WAIT_COUNT; wait_count++)
			{
				if (!vin->is_not_ready(vin->cbdata)) break;
				udelay(VINETIC_WAIT_TIMEOUT);
			}
			if (wait_count == VINETIC_WAIT_COUNT) {
				*offp = 0;
				spin_unlock_bh(&vin->lock);
				res = -EIO;
				goto vinetic_read_end;
			}
			if (cnt == res-1)
				data[cnt] = vin->read_eom(vin->cbdata);
			else
				data[cnt] = vin->read_nwd(vin->cbdata);
		}
		// check data count
		if (count < res*2) {
			*offp = 0;
			spin_unlock_bh(&vin->lock);
			log(KERN_ERR, "\"%s\": data count=%lu less than actual readed=%lu\n", vin->name, (long unsigned int)count, (long unsigned int)res*2);
			res = -EINVAL;
			goto vinetic_read_end;
		}
		*offp = 0;
		spin_unlock_bh(&vin->lock);
		// copy data to user
		if (copy_to_user(buff, data, res*2)) {
			res = -EFAULT;
			goto vinetic_read_end;
		}
		res *= 2;
	} else {
		// regular command
		spin_lock_bh(&vin->lock);
		// check for free space in mailbox
		for (;;)
		{
			for (wait_count=0; wait_count<VINETIC_WAIT_COUNT; wait_count++)
			{
				if (!vin->is_not_ready(vin->cbdata)) break;
				udelay(VINETIC_WAIT_TIMEOUT);
			}
			if (wait_count == VINETIC_WAIT_COUNT) {
				*offp = 0;
				spin_unlock_bh(&vin->lock);
				res = -EIO;
				goto vinetic_read_end;
			}
			vin->write_nwd(vin->cbdata, VIN_rFIBXMS);
			for (wait_count=0; wait_count<VINETIC_WAIT_COUNT; wait_count++)
			{
				if (!vin->is_not_ready(vin->cbdata)) break;
				udelay(VINETIC_WAIT_TIMEOUT);
			}
			if (wait_count == VINETIC_WAIT_COUNT) {
				*offp = 0;
				spin_unlock_bh(&vin->lock);
				res = -EIO;
				goto vinetic_read_end;
			}
			vin->free_cbox_space = (vin->read_eom(vin->cbdata) >> 8) & 0xff;

			if (vin->free_cbox_space >= 2) break;

			if (filp->f_flags & O_NONBLOCK) {
				*offp = 0;
				spin_unlock_bh(&vin->lock);
				res = -EAGAIN;
				goto vinetic_read_end;
			}
			// sleeping
			spin_unlock_bh(&vin->lock);
			res = wait_event_interruptible_timeout(vin->free_cbox_waitq, vin->free_cbox_space >= 2, 1);
			if (res) {
				spin_lock_bh(&vin->lock);
				*offp = 0;
				spin_unlock_bh(&vin->lock);
				goto vinetic_read_end;
			}
			spin_lock_bh(&vin->lock);
		}
		// write command to vinetic
		for (wait_count=0; wait_count<VINETIC_WAIT_COUNT; wait_count++)
		{
			if (!vin->is_not_ready(vin->cbdata)) break;
			udelay(VINETIC_WAIT_TIMEOUT);
		}
		if (wait_count == VINETIC_WAIT_COUNT) {
			*offp = 0;
			spin_unlock_bh(&vin->lock);
			res = -EIO;
			goto vinetic_read_end;
		}
		vin->write_nwd(vin->cbdata, cmd.parts.first.full);
		for (wait_count=0; wait_count<VINETIC_WAIT_COUNT; wait_count++)
		{
			if (!vin->is_not_ready(vin->cbdata)) break;
			udelay(VINETIC_WAIT_TIMEOUT);
		}
		if (wait_count == VINETIC_WAIT_COUNT) {
			*offp = 0;
			spin_unlock_bh(&vin->lock);
			res = -EIO;
			goto vinetic_read_end;
		}
		vin->write_eom(vin->cbdata, cmd.parts.second.full);
		vin->read_cbox_length = 0;
		spin_unlock_bh(&vin->lock);
		res = wait_event_interruptible(vin->read_cbox_waitq, vin->read_cbox_length != 0);
		if (res) {
			spin_lock_bh(&vin->lock);
			*offp = 0;
			spin_unlock_bh(&vin->lock);
			goto vinetic_read_end;
		}
		spin_lock_bh(&vin->lock);
		res = vin->read_cbox_length * 2;
		memcpy(data, vin->read_cbox_data, vin->read_cbox_length * 2);
		*offp = 0;
		spin_unlock_bh(&vin->lock);
		// check data count
		if (count < res) {
			log(KERN_ERR, "\"%s\": data count=%lu less than actual readed=%lu\n", vin->name, (long unsigned int)count, (long unsigned int)res*2);
			res = -EINVAL;
			goto vinetic_read_end;
		}
		// copy data to user
		if (copy_to_user(buff, data, res)) {
			res = -EFAULT;
			goto vinetic_read_end;
		}
	}
vinetic_read_end:
// 	debug("res=%ld\n", (long int)res);
	wake_up_interruptible(&vin->seek_cbox_waitq);
	return res;
}

static ssize_t vinetic_write(struct file *filp, const char __user *buff, size_t count, loff_t *offp)
{
	u_int16_t data[256];
	size_t cnt, length;
	size_t wait_count;
	ssize_t res;

	union vin_cmd cmd;

	struct vinetic *vin = filp->private_data;

	cmd.full = filp->f_pos & 0xffffffff;
// 	debug("cmd.full=0x%08x\n", cmd.full);

	// check for is write command
	if (cmd.parts.first.bits.rw != VIN_WRITE) {
		spin_lock_bh(&vin->lock);
		*offp = 0;
		spin_unlock_bh(&vin->lock);
		log(KERN_ERR, "\"%s\": is read command=0x%08x\n", vin->name, cmd.full);
		res = -EINVAL;
		goto vinetic_write_end;
	}

	if (cmd.parts.first.bits.sc) {
		// short command
		memcpy(data, &cmd.parts.first.full, sizeof(u_int16_t));
		res = 0;
		length = 1;
	} else {
		// regular command
		switch (cmd.parts.first.bits.cmd)
		{
			case VIN_CMD_SOP:
				memcpy(data, &cmd.full, sizeof(u_int32_t));
				res = cmd.parts.second.sop.bits.length;
				length = cmd.parts.second.sop.bits.length + 2;
				break;
			case VIN_CMD_COP:
				memcpy(data, &cmd.full, sizeof(u_int32_t));
				res = cmd.parts.second.sop.bits.length;
				length = cmd.parts.second.cop.bits.length + 2;
				break;
			case VIN_CMD_IOP:
				memcpy(data, &cmd.full, sizeof(u_int32_t));
				res = cmd.parts.second.sop.bits.length;
				length = cmd.parts.second.iop.bits.length + 2;
				break;
			case VIN_CMD_EOP:
				memcpy(data, &cmd.full, sizeof(u_int32_t));
				res = cmd.parts.second.sop.bits.length;
				length = cmd.parts.second.eop.bits.length + 2;
				break;
			default:
				spin_lock_bh(&vin->lock);
				*offp = 0;
				spin_unlock_bh(&vin->lock);
				log(KERN_ERR, "\"%s\": unknown cmd=%d\n", vin->name, cmd.parts.first.bits.cmd);
				res = -EINVAL;
				goto vinetic_write_end;
		}
	}
	// check data count
	if (count != res*2) {
		spin_lock_bh(&vin->lock);
		*offp = 0;
		spin_unlock_bh(&vin->lock);
		log(KERN_ERR, "\"%s\": data count=%lu mismatch to %lu\n", vin->name, (long unsigned int)count, (long unsigned int)res*2);
		res = -EINVAL;
		goto vinetic_write_end;
	}

	// get user space data
	if (res > 0) {
		if (copy_from_user(&data[2], buff, res*2)) {
			spin_lock_bh(&vin->lock);
			*offp = 0;
			spin_unlock_bh(&vin->lock);
			res = -EFAULT;
			goto vinetic_write_end;
		}
	}
	// check for free space in mailbox
	spin_lock_bh(&vin->lock);
	for (;;)
	{
		for (wait_count=0; wait_count<VINETIC_WAIT_COUNT; wait_count++)
		{
			if (!vin->is_not_ready(vin->cbdata)) break;
			udelay(VINETIC_WAIT_TIMEOUT);
		}
		if (wait_count == VINETIC_WAIT_COUNT) {
			*offp = 0;
			spin_unlock_bh(&vin->lock);
			res = -EIO;
			goto vinetic_write_end;
		}
		vin->write_nwd(vin->cbdata, VIN_rFIBXMS);
		for (wait_count=0; wait_count<VINETIC_WAIT_COUNT; wait_count++)
		{
			if (!vin->is_not_ready(vin->cbdata)) break;
			udelay(VINETIC_WAIT_TIMEOUT);
		}
		if (wait_count == VINETIC_WAIT_COUNT) {
			*offp = 0;
			spin_unlock_bh(&vin->lock);
			res = -EIO;
			goto vinetic_write_end;
		}

		vin->free_cbox_space = (vin->read_eom(vin->cbdata) >> 8) & 0xff;

		if (vin->free_cbox_space >= length) break;

		if (filp->f_flags & O_NONBLOCK) {
			*offp = 0;
			spin_unlock_bh(&vin->lock);
			res = -EAGAIN;
			goto vinetic_write_end;
		}
		// sleeping
		spin_unlock_bh(&vin->lock);
		wait_event_interruptible_timeout(vin->free_cbox_waitq, vin->free_cbox_space >= length, 1);
		if (res) {
			spin_lock_bh(&vin->lock);
			*offp = 0;
			spin_unlock_bh(&vin->lock);
			goto vinetic_write_end;
		}
		spin_lock_bh(&vin->lock);
	}
	// write data to vinetic
	for (cnt=0; cnt<length; cnt++)
	{
		for (wait_count=0; wait_count<VINETIC_WAIT_COUNT; wait_count++)
		{
			if (!vin->is_not_ready(vin->cbdata)) break;
			udelay(VINETIC_WAIT_TIMEOUT);
		}
		if (wait_count == VINETIC_WAIT_COUNT) {
			*offp = 0;
			spin_unlock_bh(&vin->lock);
			res = -EIO;
			goto vinetic_write_end;
		}
		if (cnt == length-1) {
			vin->write_eom(vin->cbdata, data[cnt]);
// 			debug("%03lu: 0x%04x\n", cnt, data[cnt]);
		} else {
			vin->write_nwd(vin->cbdata, data[cnt]);
// 			debug("%03lu: 0x%04x\n", cnt, data[cnt]);
		}
	}
	*offp = 0;
	spin_unlock_bh(&vin->lock);
	res *= 2;

vinetic_write_end:
	wake_up_interruptible(&vin->seek_cbox_waitq);
	return res;
}

#if defined(HAVE_UNLOCKED_IOCTL)
static long vinetic_unlocked_ioctl(struct file *filp, unsigned int cmd, unsigned long data)
{
	u_int16_t phi;
	size_t wait_count;
	size_t cnt;
	int not_ready;
	long res = 0;
	struct vinetic *vin = filp->private_data;
	void __user *argp = (void __user *)data;

// 	debug("%s cmd=0x%08x\n", vin->name, cmd);

	switch (cmd)
	{
		case VINETIC_RESET:
			vin->reset(vin->cbdata);
			break;
		case VINETIC_DISABLE_IRQ:
			spin_lock_bh(&vin->lock);
			for (wait_count=0; wait_count<VINETIC_WAIT_COUNT; wait_count++)
			{
				if (!vin->is_not_ready(vin->cbdata)) break;
				udelay(VINETIC_WAIT_TIMEOUT);
			}
			if (wait_count == VINETIC_WAIT_COUNT) {
				spin_unlock_bh(&vin->lock);
				res = -EIO;
				break;
			}
			vin->write_nwd(vin->cbdata, 0x0801);
			for (wait_count=0; wait_count<VINETIC_WAIT_COUNT; wait_count++)
			{
				if (!vin->is_not_ready(vin->cbdata)) break;
				udelay(VINETIC_WAIT_TIMEOUT);
			}
			if (wait_count == VINETIC_WAIT_COUNT) {
				spin_unlock_bh(&vin->lock);
				res = -EIO;
				break;
			}
			vin->write_nwd(vin->cbdata, 0x4020);
			// write data to vinetic
			for (cnt=0; cnt<32; cnt++)
			{
				for (wait_count=0; wait_count<VINETIC_WAIT_COUNT; wait_count++)
				{
					if (!vin->is_not_ready(vin->cbdata)) break;
					udelay(VINETIC_WAIT_TIMEOUT);
				}
				if (wait_count == VINETIC_WAIT_COUNT) {
					spin_unlock_bh(&vin->lock);
					res = -EIO;
					goto vinetic_unlocked_ioctl_end;
				}
				if (cnt == 31)
					vin->write_eom(vin->cbdata, 0xffff);
				else
					vin->write_nwd(vin->cbdata, 0xffff);
				}
			spin_unlock_bh(&vin->lock);
			break;
		case VINETIC_GET_NOT_READY:
			not_ready = vin->is_not_ready(vin->cbdata);
			if (copy_to_user(argp, &not_ready, sizeof(int)))
				res = -EINVAL;
			break;
		case VINETIC_READ_DIA:
			phi = vin->read_dia(vin->cbdata);
			if (copy_to_user(argp, &phi, sizeof(u_int16_t)))
				res = -EINVAL;
			break;
		case VINETIC_REVISION:
			spin_lock_bh(&vin->lock);
			for (wait_count=0; wait_count<VINETIC_WAIT_COUNT; wait_count++)
			{
				if (!vin->is_not_ready(vin->cbdata)) break;
				udelay(VINETIC_WAIT_TIMEOUT);
			}
			if (wait_count == VINETIC_WAIT_COUNT) {
				spin_unlock_bh(&vin->lock);
				res = -EIO;
				break;
			}
			vin->write_nwd(vin->cbdata, 0x8801);
			for (wait_count=0; wait_count<VINETIC_WAIT_COUNT; wait_count++)
			{
				if (!vin->is_not_ready(vin->cbdata)) break;
				udelay(VINETIC_WAIT_TIMEOUT);
			}
			if (wait_count == VINETIC_WAIT_COUNT) {
				spin_unlock_bh(&vin->lock);
				res = -EIO;
				break;
			}
			vin->write_nwd(vin->cbdata, 0x8001);
			for (wait_count=0; wait_count<VINETIC_WAIT_COUNT; wait_count++)
			{
				if (!vin->is_not_ready(vin->cbdata)) break;
				udelay(VINETIC_WAIT_TIMEOUT);
			}
			if (wait_count == VINETIC_WAIT_COUNT) {
				spin_unlock_bh(&vin->lock);
				res = -EIO;
				break;
			}
			phi = vin->read_eom(vin->cbdata);
			spin_unlock_bh(&vin->lock);
			if (copy_to_user(argp, &phi, sizeof(u_int16_t)))
				res = -EINVAL;
			break;
		case VINETIC_CHECKSUM:
			spin_lock_bh(&vin->lock);
			for (wait_count=0; wait_count<VINETIC_WAIT_COUNT; wait_count++)
			{
				if (!vin->is_not_ready(vin->cbdata)) break;
				udelay(VINETIC_WAIT_TIMEOUT);
			}
			if (wait_count == VINETIC_WAIT_COUNT) {
				spin_unlock_bh(&vin->lock);
				res = -EIO;
				break;
			}
			vin->write_nwd(vin->cbdata, 0x8801);
			for (wait_count=0; wait_count<VINETIC_WAIT_COUNT; wait_count++)
			{
				if (!vin->is_not_ready(vin->cbdata)) break;
				udelay(VINETIC_WAIT_TIMEOUT);
			}
			if (wait_count == VINETIC_WAIT_COUNT) {
				spin_unlock_bh(&vin->lock);
				res = -EIO;
				break;
			}
			vin->write_nwd(vin->cbdata, 0x8901);
			for (wait_count=0; wait_count<VINETIC_WAIT_COUNT; wait_count++)
			{
				if (!vin->is_not_ready(vin->cbdata)) break;
				udelay(VINETIC_WAIT_TIMEOUT);
			}
			if (wait_count == VINETIC_WAIT_COUNT) {
				spin_unlock_bh(&vin->lock);
				res = -EIO;
				break;
			}
			phi = vin->read_eom(vin->cbdata);
			spin_unlock_bh(&vin->lock);
			if (copy_to_user(argp, &phi, sizeof(u_int16_t)))
				res = -EINVAL;
			break;
		case VINETIC_SET_POLL:
			if (copy_from_user(&vin->poll, argp, sizeof(int)))
				res = -EINVAL;
			del_timer_sync(&vin->poll_timer);
			if (vin->poll) {
				vin->poll_timer.function = vinetic_poll;
				vin->poll_timer.data = (unsigned long)vin;
				vin->poll_timer.expires = jiffies + 1;
				add_timer(&vin->poll_timer);
			}
			break;
		default:
			res = -ENOIOCTLCMD;
			break;
	}
vinetic_unlocked_ioctl_end:
	return res;
}
#else
static int vinetic_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long data)
{
	u_int16_t phi;
	size_t wait_count;
	size_t cnt;
	int not_ready;
	int res = 0;
	struct vinetic *vin = filp->private_data;
	void __user *argp = (void __user *)data;

// 	debug("%s cmd=0x%08x\n", vin->name, cmd);

	switch (cmd)
	{
		case VINETIC_RESET:
			vin->reset(vin->cbdata);
			break;
		case VINETIC_DISABLE_IRQ:
			spin_lock_bh(&vin->lock);
			for (wait_count=0; wait_count<VINETIC_WAIT_COUNT; wait_count++)
			{
				if (!vin->is_not_ready(vin->cbdata)) break;
				udelay(VINETIC_WAIT_TIMEOUT);
			}
			if (wait_count == VINETIC_WAIT_COUNT) {
				spin_unlock_bh(&vin->lock);
				res = -EIO;
				break;
			}
			vin->write_nwd(vin->cbdata, 0x0801);
			for (wait_count=0; wait_count<VINETIC_WAIT_COUNT; wait_count++)
			{
				if (!vin->is_not_ready(vin->cbdata)) break;
				udelay(VINETIC_WAIT_TIMEOUT);
			}
			if (wait_count == VINETIC_WAIT_COUNT) {
				spin_unlock_bh(&vin->lock);
				res = -EIO;
				break;
			}
			vin->write_nwd(vin->cbdata, 0x4020);
			// write data to vinetic
			for (cnt=0; cnt<32; cnt++)
			{
				for (wait_count=0; wait_count<VINETIC_WAIT_COUNT; wait_count++)
				{
					if (!vin->is_not_ready(vin->cbdata)) break;
					udelay(VINETIC_WAIT_TIMEOUT);
				}
				if (wait_count == VINETIC_WAIT_COUNT) {
					spin_unlock_bh(&vin->lock);
					res = -EIO;
					goto vinetic_ioctl_end;
				}
				if (cnt == 31)
					vin->write_eom(vin->cbdata, 0xffff);
				else
					vin->write_nwd(vin->cbdata, 0xffff);
				}
			spin_unlock_bh(&vin->lock);
			break;
		case VINETIC_GET_NOT_READY:
			not_ready = vin->is_not_ready(vin->cbdata);
			if (copy_to_user(argp, &not_ready, sizeof(int)))
				res = -EINVAL;
			break;
		case VINETIC_READ_DIA:
			phi = vin->read_dia(vin->cbdata);
			if (copy_to_user(argp, &phi, sizeof(u_int16_t)))
				res = -EINVAL;
			break;
		case VINETIC_REVISION:
			spin_lock_bh(&vin->lock);
			for (wait_count=0; wait_count<VINETIC_WAIT_COUNT; wait_count++)
			{
				if (!vin->is_not_ready(vin->cbdata)) break;
				udelay(VINETIC_WAIT_TIMEOUT);
			}
			if (wait_count == VINETIC_WAIT_COUNT) {
				spin_unlock_bh(&vin->lock);
				res = -EIO;
				break;
			}
			vin->write_nwd(vin->cbdata, 0x8801);
			for (wait_count=0; wait_count<VINETIC_WAIT_COUNT; wait_count++)
			{
				if (!vin->is_not_ready(vin->cbdata)) break;
				udelay(VINETIC_WAIT_TIMEOUT);
			}
			if (wait_count == VINETIC_WAIT_COUNT) {
				spin_unlock_bh(&vin->lock);
				res = -EIO;
				break;
			}
			vin->write_nwd(vin->cbdata, 0x8001);
			for (wait_count=0; wait_count<VINETIC_WAIT_COUNT; wait_count++)
			{
				if (!vin->is_not_ready(vin->cbdata)) break;
				udelay(VINETIC_WAIT_TIMEOUT);
			}
			if (wait_count == VINETIC_WAIT_COUNT) {
				spin_unlock_bh(&vin->lock);
				res = -EIO;
				break;
			}
			phi = vin->read_eom(vin->cbdata);
			spin_unlock_bh(&vin->lock);
			if (copy_to_user(argp, &phi, sizeof(u_int16_t)))
				res = -EINVAL;
			break;
		case VINETIC_CHECKSUM:
			spin_lock_bh(&vin->lock);
			for (wait_count=0; wait_count<VINETIC_WAIT_COUNT; wait_count++)
			{
				if (!vin->is_not_ready(vin->cbdata)) break;
				udelay(VINETIC_WAIT_TIMEOUT);
			}
			if (wait_count == VINETIC_WAIT_COUNT) {
				spin_unlock_bh(&vin->lock);
				res = -EIO;
				break;
			}
			vin->write_nwd(vin->cbdata, 0x8801);
			for (wait_count=0; wait_count<VINETIC_WAIT_COUNT; wait_count++)
			{
				if (!vin->is_not_ready(vin->cbdata)) break;
				udelay(VINETIC_WAIT_TIMEOUT);
			}
			if (wait_count == VINETIC_WAIT_COUNT) {
				spin_unlock_bh(&vin->lock);
				res = -EIO;
				break;
			}
			vin->write_nwd(vin->cbdata, 0x8901);
			for (wait_count=0; wait_count<VINETIC_WAIT_COUNT; wait_count++)
			{
				if (!vin->is_not_ready(vin->cbdata)) break;
				udelay(VINETIC_WAIT_TIMEOUT);
			}
			if (wait_count == VINETIC_WAIT_COUNT) {
				spin_unlock_bh(&vin->lock);
				res = -EIO;
				break;
			}
			phi = vin->read_eom(vin->cbdata);
			spin_unlock_bh(&vin->lock);
			if (copy_to_user(argp, &phi, sizeof(u_int16_t)))
				res = -EINVAL;
			break;
		case VINETIC_SET_POLL:
			if (copy_from_user(&vin->poll, argp, sizeof(int)))
				res = -EINVAL;
			del_timer_sync(&vin->poll_timer);
			if (vin->poll) {
				vin->poll_timer.function = vinetic_poll;
				vin->poll_timer.data = (unsigned long)vin;
				vin->poll_timer.expires = jiffies + 1;
				add_timer(&vin->poll_timer);
			}
			break;
		default:
			res = -ENOIOCTLCMD;
			break;
	}
vinetic_ioctl_end:
	return res;
}
#endif

#if defined(CONFIG_COMPAT) && defined(HAVE_COMPAT_IOCTL) && (HAVE_COMPAT_IOCTL == 1)
static long vinetic_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long data)
{
	u_int16_t phi;
	size_t wait_count;
	size_t cnt;
	int not_ready;
	long res = 0;
	struct vinetic *vin = filp->private_data;
	void __user *argp = (void __user *)data;

// 	debug("%s cmd=0x%08x\n", vin->name, cmd);

	switch (cmd)
	{
		case VINETIC_RESET:
			vin->reset(vin->cbdata);
			break;
		case VINETIC_DISABLE_IRQ:
			spin_lock_bh(&vin->lock);
			for (wait_count=0; wait_count<VINETIC_WAIT_COUNT; wait_count++)
			{
				if (!vin->is_not_ready(vin->cbdata)) break;
				udelay(VINETIC_WAIT_TIMEOUT);
			}
			if (wait_count == VINETIC_WAIT_COUNT) {
				spin_unlock_bh(&vin->lock);
				res = -EIO;
				break;
			}
			vin->write_nwd(vin->cbdata, 0x0801);
			for (wait_count=0; wait_count<VINETIC_WAIT_COUNT; wait_count++)
			{
				if (!vin->is_not_ready(vin->cbdata)) break;
				udelay(VINETIC_WAIT_TIMEOUT);
			}
			if (wait_count == VINETIC_WAIT_COUNT) {
				spin_unlock_bh(&vin->lock);
				res = -EIO;
				break;
			}
			vin->write_nwd(vin->cbdata, 0x4020);
			// write data to vinetic
			for (cnt=0; cnt<32; cnt++)
			{
				for (wait_count=0; wait_count<VINETIC_WAIT_COUNT; wait_count++)
				{
					if (!vin->is_not_ready(vin->cbdata)) break;
					udelay(VINETIC_WAIT_TIMEOUT);
				}
				if (wait_count == VINETIC_WAIT_COUNT) {
					spin_unlock_bh(&vin->lock);
					res = -EIO;
					goto vinetic_compat_ioctl_end;
				}
				if (cnt == 31)
					vin->write_eom(vin->cbdata, 0xffff);
				else
					vin->write_nwd(vin->cbdata, 0xffff);
				}
			spin_unlock_bh(&vin->lock);
			break;
		case VINETIC_GET_NOT_READY:
			not_ready = vin->is_not_ready(vin->cbdata);
			if (copy_to_user(argp, &not_ready, sizeof(int)))
				res = -EINVAL;
			break;
		case VINETIC_READ_DIA:
			phi = vin->read_dia(vin->cbdata);
			if (copy_to_user(argp, &phi, sizeof(u_int16_t)))
				res = -EINVAL;
			break;
		case VINETIC_REVISION:
			spin_lock_bh(&vin->lock);
			for (wait_count=0; wait_count<VINETIC_WAIT_COUNT; wait_count++)
			{
				if (!vin->is_not_ready(vin->cbdata)) break;
				udelay(VINETIC_WAIT_TIMEOUT);
			}
			if (wait_count == VINETIC_WAIT_COUNT) {
				spin_unlock_bh(&vin->lock);
				res = -EIO;
				break;
			}
			vin->write_nwd(vin->cbdata, 0x8801);
			for (wait_count=0; wait_count<VINETIC_WAIT_COUNT; wait_count++)
			{
				if (!vin->is_not_ready(vin->cbdata)) break;
				udelay(VINETIC_WAIT_TIMEOUT);
			}
			if (wait_count == VINETIC_WAIT_COUNT) {
				spin_unlock_bh(&vin->lock);
				res = -EIO;
				break;
			}
			vin->write_nwd(vin->cbdata, 0x8001);
			for (wait_count=0; wait_count<VINETIC_WAIT_COUNT; wait_count++)
			{
				if (!vin->is_not_ready(vin->cbdata)) break;
				udelay(VINETIC_WAIT_TIMEOUT);
			}
			if (wait_count == VINETIC_WAIT_COUNT) {
				spin_unlock_bh(&vin->lock);
				res = -EIO;
				break;
			}
			phi = vin->read_eom(vin->cbdata);
			spin_unlock_bh(&vin->lock);
			if (copy_to_user(argp, &phi, sizeof(u_int16_t)))
				res = -EINVAL;
			break;
		case VINETIC_CHECKSUM:
			spin_lock_bh(&vin->lock);
			for (wait_count=0; wait_count<VINETIC_WAIT_COUNT; wait_count++)
			{
				if (!vin->is_not_ready(vin->cbdata)) break;
				udelay(VINETIC_WAIT_TIMEOUT);
			}
			if (wait_count == VINETIC_WAIT_COUNT) {
				spin_unlock_bh(&vin->lock);
				res = -EIO;
				break;
			}
			vin->write_nwd(vin->cbdata, 0x8801);
			for (wait_count=0; wait_count<VINETIC_WAIT_COUNT; wait_count++)
			{
				if (!vin->is_not_ready(vin->cbdata)) break;
				udelay(VINETIC_WAIT_TIMEOUT);
			}
			if (wait_count == VINETIC_WAIT_COUNT) {
				spin_unlock_bh(&vin->lock);
				res = -EIO;
				break;
			}
			vin->write_nwd(vin->cbdata, 0x8901);
			for (wait_count=0; wait_count<VINETIC_WAIT_COUNT; wait_count++)
			{
				if (!vin->is_not_ready(vin->cbdata)) break;
				udelay(VINETIC_WAIT_TIMEOUT);
			}
			if (wait_count == VINETIC_WAIT_COUNT) {
				spin_unlock_bh(&vin->lock);
				res = -EIO;
				break;
			}
			phi = vin->read_eom(vin->cbdata);
			spin_unlock_bh(&vin->lock);
			if (copy_to_user(argp, &phi, sizeof(u_int16_t)))
				res = -EINVAL;
			break;
		case VINETIC_SET_POLL:
			if (copy_from_user(&vin->poll, argp, sizeof(int)))
				res = -EINVAL;
			del_timer_sync(&vin->poll_timer);
			if (vin->poll) {
				vin->poll_timer.function = vinetic_poll;
				vin->poll_timer.data = (unsigned long)vin;
				vin->poll_timer.expires = jiffies + 1;
				add_timer(&vin->poll_timer);
			}
			break;
		default:
			res = -ENOIOCTLCMD;
			break;
	}
vinetic_compat_ioctl_end:
	return res;
}
#endif

loff_t vinetic_llseek(struct file * filp, loff_t off, int whence)
{
	loff_t res;
	loff_t newpos;
	struct vinetic *vin = filp->private_data;

	switch (whence)
	{
		case 0: /* SEEK_SET */
			newpos = off;
			break;
		case 1: /* SEEK_CUR */
			newpos = off;
			break;
		case 2: /* SEEK_END */
			newpos = off;
			break;
		default: /* can't happen */
			res = -EINVAL;
			goto vinetic_llseek_end;
	}
	if (newpos < 0) return -EINVAL;

	spin_lock_bh(&vin->lock);
	for (;;)
	{
// 		debug("%ld\n", (long int)filp->f_pos);
		if (!filp->f_pos) break;
		
		if (filp->f_flags & O_NONBLOCK) {
			spin_unlock_bh(&vin->lock);
			res = -EAGAIN;
			goto vinetic_llseek_end;
		}
		// sleeping
		spin_unlock_bh(&vin->lock);
		res = wait_event_interruptible(vin->seek_cbox_waitq, filp->f_pos == 0);
		if (res) {
			goto vinetic_llseek_end;
		}
		spin_lock_bh(&vin->lock);
	}
	filp->f_pos = res = newpos;
	spin_unlock_bh(&vin->lock);

vinetic_llseek_end:
// 	debug("%ld\n", (long int)res);
	return res;
}

static struct file_operations vinetic_fops = {
	.owner   = THIS_MODULE,
	.open    = vinetic_open,
	.release = vinetic_release,
	.read    = vinetic_read,
	.write   = vinetic_write,
#if defined(HAVE_UNLOCKED_IOCTL)
	.unlocked_ioctl = vinetic_unlocked_ioctl,
#else
	.ioctl = vinetic_ioctl,
#endif
#if defined(CONFIG_COMPAT) && defined(HAVE_COMPAT_IOCTL) && (HAVE_COMPAT_IOCTL == 1)
	.compat_ioctl = vinetic_compat_ioctl,
#endif
	.llseek = vinetic_llseek,
};

static int vinetic_rtp_channel_open(struct inode *inode, struct file *filp)
{
	struct vinetic_rtp_channel *rtp;

	rtp = container_of(inode->i_cdev, struct vinetic_rtp_channel, cdev);
	filp->private_data = rtp;

	spin_lock_bh(&rtp->lock);

	// reset recv packet slot counters
	rtp->recv_slot_read = 0;
	rtp->recv_slot_write = 0;
	rtp->recv_slot_count = 0;

	spin_unlock_bh(&rtp->lock);

	return 0;
}

static int vinetic_rtp_channel_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static ssize_t vinetic_rtp_channel_read(struct file *filp, char __user *buff, size_t count, loff_t *offp)
{
	struct rtp_packet_slot recv_slot;
	struct vinetic_rtp_channel *rtp = filp->private_data;

	spin_lock_bh(&rtp->lock);
	
	for (;;)
	{
		// successfull return
		if(rtp->recv_slot_count > 0)
			break;

		spin_unlock_bh(&rtp->lock);

		if(filp->f_flags & O_NONBLOCK)
			return -EAGAIN;

		// sleeping
		udelay(125);	// :FIXME: must be added advanced sleeping scenario

		spin_lock_bh(&rtp->lock);
	}
	// copy data out of lock area
	recv_slot.length = rtp->recv_slot[rtp->recv_slot_read].length;
	memcpy(recv_slot.data, rtp->recv_slot[rtp->recv_slot_read].data, recv_slot.length);
	// adjust read position of receiving buffer
	rtp->recv_slot_read++;
	if (rtp->recv_slot_read >= VINETIC_PACKETSLOT_MAXCOUNT)
		rtp->recv_slot_read = 0;

	if (rtp->recv_slot_count)
		rtp->recv_slot_count--;

	spin_unlock_bh(&rtp->lock);

	// copy to user
	recv_slot.length = min(recv_slot.length, count);
	if(copy_to_user(buff, recv_slot.data, recv_slot.length))
		return -EFAULT;

	return recv_slot.length;
}

static ssize_t vinetic_rtp_channel_write(struct file *filp, const char __user *buff, size_t count, loff_t *offp)
{
	u_int16_t data[256];
	size_t cnt, length;
	size_t wait_count;
	ssize_t res;

	union vin_cmd *cmd;

	struct vinetic_rtp_channel *rtp = filp->private_data;
	struct vinetic *vin = rtp->vinetic;
	
	cnt = 510;
	cnt = min(count, cnt);
	res = cnt;
	
	cmd = (union vin_cmd *)data;
	cmd->parts.first.bits.rw = VIN_WRITE;
	cmd->parts.first.bits.sc = VIN_SC_NO;
	cmd->parts.first.bits.bc = VIN_BC_NO;
	cmd->parts.first.bits.cmd = VIN_CMD_VOP;
	cmd->parts.first.bits.res = 0;
	cmd->parts.first.bits.chan = rtp->index;
	cmd->parts.second.vop.bits.res1 = 0;
	cmd->parts.second.vop.bits.odd = res%2;
	cmd->parts.second.vop.bits.res0 = 0;
	cmd->parts.second.vop.bits.length = res/2 + ((res%2)?(1):(0));
	length = cmd->parts.second.vop.bits.length + 2;
	// get user space data
	if (copy_from_user(&data[2], buff, res)) {
		res = -EFAULT;
		goto vinetic_rtp_channel_write_end;
	}
	// check for free space in mailbox
	spin_lock_bh(&vin->lock);
	for (;;)
	{
		for (wait_count=0; wait_count<VINETIC_WAIT_COUNT; wait_count++)
		{
			if (!vin->is_not_ready(vin->cbdata)) break;
			udelay(VINETIC_WAIT_TIMEOUT);
		}
		if (wait_count == VINETIC_WAIT_COUNT) {
			spin_unlock_bh(&vin->lock);
			res = -EIO;
			goto vinetic_rtp_channel_write_end;
		}
		vin->write_nwd(vin->cbdata, VIN_rFIBXMS);
		for (wait_count=0; wait_count<VINETIC_WAIT_COUNT; wait_count++)
		{
			if (!vin->is_not_ready(vin->cbdata)) break;
			udelay(VINETIC_WAIT_TIMEOUT);
		}
		if (wait_count == VINETIC_WAIT_COUNT) {
			spin_unlock_bh(&vin->lock);
			res = -EIO;
			goto vinetic_rtp_channel_write_end;
		}

		vin->free_pbox_space = vin->read_eom(vin->cbdata) & 0xff;

		if (vin->free_pbox_space >= length) break;

		if (filp->f_flags & O_NONBLOCK) {
			spin_unlock_bh(&vin->lock);
			res = -EAGAIN;
			goto vinetic_rtp_channel_write_end;
		}
		// sleeping
		spin_unlock_bh(&vin->lock);
		wait_event_interruptible_timeout(vin->free_pbox_waitq, vin->free_pbox_space >= length, 1);
		if (res) {
			goto vinetic_rtp_channel_write_end;
		}
		spin_lock_bh(&vin->lock);
	}
	// write data to vinetic
	for (cnt=0; cnt<length; cnt++)
	{
		for (wait_count=0; wait_count<VINETIC_WAIT_COUNT; wait_count++)
		{
			if (!vin->is_not_ready(vin->cbdata)) break;
			udelay(VINETIC_WAIT_TIMEOUT);
		}
		if (wait_count == VINETIC_WAIT_COUNT) {
			spin_unlock_bh(&vin->lock);
			res = -EIO;
			goto vinetic_rtp_channel_write_end;
		}
		if (cnt == length-1)
			vin->write_eom(vin->cbdata, data[cnt]);
		else
			vin->write_nwd(vin->cbdata, data[cnt]);
	}
	spin_unlock_bh(&vin->lock);

vinetic_rtp_channel_write_end:
	return res;
}

static unsigned int vinetic_rtp_channel_poll(struct file *filp, struct poll_table_struct *wait_table)
{
// 	size_t wait_count;
// 	size_t fps;
	unsigned int res;

	struct vinetic_rtp_channel *rtp = filp->private_data;
// 	struct vinetic *vin = rtp->vinetic;

	res = 0;

	poll_wait(filp, &rtp->poll_waitq, wait_table);
#if 0
	spin_lock_bh(&vin->lock);
	for (wait_count=0; wait_count<VINETIC_WAIT_COUNT; wait_count++)
	{
		if (!vin->is_not_ready(vin->cbdata)) break;
			udelay(VINETIC_WAIT_TIMEOUT);
	}
	if (wait_count == VINETIC_WAIT_COUNT) {
		spin_unlock_bh(&vin->lock);
		goto vinetic_rtp_channel_poll_end;
	}
	vin->write_nwd(vin->cbdata, VIN_rFIBXMS);
	for (wait_count=0; wait_count<VINETIC_WAIT_COUNT; wait_count++)
	{
		if (!vin->is_not_ready(vin->cbdata)) break;
		udelay(VINETIC_WAIT_TIMEOUT);
	}
	if (wait_count == VINETIC_WAIT_COUNT) {
		spin_unlock_bh(&vin->lock);
		goto vinetic_rtp_channel_poll_end;
	}

	fps = vin->read_eom(vin->cbdata) & 0xff;

	if (fps >= 88)
		res |= POLLOUT | POLLWRNORM;

	spin_unlock_bh(&vin->lock); 
#endif
	spin_lock_bh(&rtp->lock);
	if (rtp->recv_slot_count > 0)
		res |= POLLIN | POLLRDNORM;
	spin_unlock_bh(&rtp->lock);

// vinetic_rtp_channel_poll_end:
	return res;
	}

static struct file_operations vinetic_rtp_channel_fops = {
	.owner   = THIS_MODULE,
	.open    = vinetic_rtp_channel_open,
	.release = vinetic_rtp_channel_release,
	.read    = vinetic_rtp_channel_read,
	.write   = vinetic_rtp_channel_write,
	.poll   = vinetic_rtp_channel_poll,
};

struct vinetic *vinetic_device_register(struct module *owner,
							char *name,
							uintptr_t cbdata,
							void (* reset)(uintptr_t cbdata),
							size_t (* is_not_ready)(uintptr_t cbdata),
							void (* write_nwd)(uintptr_t cbdata, u_int16_t value),
							void (* write_eom)(uintptr_t cbdata, u_int16_t value),
							u_int16_t (* read_nwd)(uintptr_t cbdata),
							u_int16_t (* read_eom)(uintptr_t cbdata),
							u_int16_t (* read_dia)(uintptr_t cbdata))
{
	struct vinetic *vin;
	int i;
	char devname[64];
	int devno = 0;
	int slot_alloc = 0;

	if (!(vin = kmalloc(sizeof(struct vinetic), GFP_KERNEL))) {
		log(KERN_ERR, "\"%s\" - can't get memory\n", name);
		goto vinetic_device_register_error;
	}

	spin_lock(&vinetic_device_list_lock);
	// check for name is not used
	for (i=0; i<VINETIC_DEVICE_MAXCOUNT; i++)
	{
		if (!strcmp(vinetic_device_list[i].name, name)) {
			spin_unlock(&vinetic_device_list_lock);
			log(KERN_ERR, "\"%s\" already registered\n", name);
			goto vinetic_device_register_error;
		}
	}
	// get free slot
	for (i=0; i<VINETIC_DEVICE_MAXCOUNT; i++)
	{
		if (!vinetic_device_list[i].data) {
			devno = MKDEV(vinetic_major, i);
			vinetic_device_list[i].devno = devno;
			snprintf(vinetic_device_list[i].name, VINETIC_DEVNAME_MAXLEN, "%s", name);
			vinetic_device_list[i].type = VINETIC_DEVTYPE_VINETIC;
			vinetic_device_list[i].data = vin;
			vin->name = vinetic_device_list[i].name;
			vin->devno = devno;
			break;
		}
	}
	spin_unlock(&vinetic_device_list_lock);
	
	if (!devno) {
		log(KERN_ERR, "\"%s\" - can't get free slot\n", name);
		goto vinetic_device_register_error;
	}
	slot_alloc = 1;
	
	// init vinetic private data
	for (i=0; i<8; i++) vin->rtp_channels[i] = NULL;
	spin_lock_init(&vin->lock);
	init_waitqueue_head(&vin->free_cbox_waitq);
	init_waitqueue_head(&vin->free_pbox_waitq);
	init_waitqueue_head(&vin->read_cbox_waitq);
	init_waitqueue_head(&vin->seek_cbox_waitq);
	init_timer(&vin->poll_timer);
	vin->poll = 0;

	vin->cbdata = cbdata;

	if (!reset) {
		log(KERN_ERR, "\"%s\" - reset callback not present\n", name);
		goto vinetic_device_register_error;
	}
	vin->reset = reset;

	if (!is_not_ready) {
		log(KERN_ERR, "\"%s\" - is_not_ready callback not present\n", name);
		goto vinetic_device_register_error;
	}
	vin->is_not_ready = is_not_ready;

	if (!write_nwd) {
		log(KERN_ERR, "\"%s\" - write_nwd callback not present\n", name);
		goto vinetic_device_register_error;
	}
	vin->write_nwd = write_nwd;

	if (!write_eom) {
		log(KERN_ERR, "\"%s\" - write_eom callback not present\n", name);
		goto vinetic_device_register_error;
	}
	vin->write_eom = write_eom;

	if (!read_nwd) {
		log(KERN_ERR, "\"%s\" - read_nwd callback not present\n", name);
		goto vinetic_device_register_error;
	}
	vin->read_nwd = read_nwd;

	if (!read_eom) {
		log(KERN_ERR, "\"%s\" - read_eom callback not present\n", name);
		goto vinetic_device_register_error;
	}
	vin->read_eom = read_eom;

	if (!read_dia) {
		log(KERN_ERR, "\"%s\" - read_dia callback not present\n", name);
		goto vinetic_device_register_error;
	}
	vin->read_dia = read_dia;

	// Add char device to system
	cdev_init(&vin->cdev, &vinetic_fops);
	vin->cdev.owner = owner;
	vin->cdev.ops = &vinetic_fops;
	if ((i = cdev_add(&vin->cdev, devno, 1)) < 0) {
		log(KERN_ERR, "\"%s\" - cdev_add() error=%d\n", name, i);
		goto vinetic_device_register_error;
	}
	snprintf(devname, sizeof(devname), "polygator!%s", name);
	CLASS_DEV_CREATE(vinetic_class, devno, NULL, devname);

	verbose("\"%s\" registered\n", name);
	return vin;

vinetic_device_register_error:
	if (slot_alloc) {
		spin_lock(&vinetic_device_list_lock);
		for (i=0; i<VINETIC_DEVICE_MAXCOUNT; i++)
		{
			if (!strcmp(vinetic_device_list[i].name, name)) {
				vinetic_device_list[i].name[0] = '\0';
				vinetic_device_list[i].devno = 0;
				vinetic_device_list[i].type = VINETIC_DEVTYPE_UNKNOWN;
				vinetic_device_list[i].data = NULL;
				break;
			}
		}
		spin_unlock(&vinetic_device_list_lock);
	}
	if(vin) kfree(vin);
	return NULL;
}

void vinetic_device_unregister(struct vinetic *vin)
{
	int i;

	verbose("\"%s\" unregistered\n", vin->name);

	spin_lock(&vinetic_device_list_lock);

	for (i=0; i<VINETIC_DEVICE_MAXCOUNT; i++)
	{
		if ((vinetic_device_list[i].type == VINETIC_DEVTYPE_VINETIC) &&
				(!strcmp(vinetic_device_list[i].name, vin->name))) {
			vinetic_device_list[i].name[0] = '\0';
			vinetic_device_list[i].devno = 0;
			vinetic_device_list[i].type = VINETIC_DEVTYPE_UNKNOWN;
			vinetic_device_list[i].data = NULL;
			break;
		}
	}
	spin_unlock(&vinetic_device_list_lock);

	// deleting vinetic polling timer
	del_timer_sync(&vin->poll_timer);

	CLASS_DEV_DESTROY(vinetic_class, vin->devno);
	cdev_del(&vin->cdev);
	kfree(vin);
}

struct vinetic_rtp_channel *vinetic_rtp_channel_register(struct module *owner, char *name, struct vinetic *vin, int index)
{
	struct vinetic_rtp_channel *rtp;
	int i;
	char devname[64];
	int devno = 0;
	int slot_alloc = 0;

	if (!(rtp = kmalloc(sizeof(struct vinetic_rtp_channel), GFP_KERNEL))) {
		log(KERN_ERR, "\"%s\" - can't get memory\n", name);
		goto vinetic_rtp_channel_register_error;
	}

	spin_lock(&vinetic_device_list_lock);
	// check for name is not used
	for (i=0; i<VINETIC_DEVICE_MAXCOUNT; i++)
	{
		if (!strcmp(vinetic_device_list[i].name, name)) {
			spin_unlock(&vinetic_device_list_lock);
			log(KERN_ERR, "\"%s\" already registered\n", name);
			goto vinetic_rtp_channel_register_error;
		}
	}
	// get free slot
	for (i=0; i<VINETIC_DEVICE_MAXCOUNT; i++)
	{
		if (!vinetic_device_list[i].data) {
			devno = MKDEV(vinetic_major, i);
			vinetic_device_list[i].devno = devno;
			snprintf(vinetic_device_list[i].name, VINETIC_DEVNAME_MAXLEN, "%s", name);
			vinetic_device_list[i].type = VINETIC_DEVTYPE_RTPCHAN;
			vinetic_device_list[i].data = rtp;
			rtp->name = vinetic_device_list[i].name;
			rtp->devno = devno;
			break;
		}
	}
	spin_unlock(&vinetic_device_list_lock);
	
	if (!devno) {
		log(KERN_ERR, "\"%s\" - can't get free slot\n", name);
		goto vinetic_rtp_channel_register_error;
	}
	slot_alloc = 1;

	if (!vin) {
		log(KERN_ERR, "\"%s\" - vinetic not specified\n", name);
		goto vinetic_rtp_channel_register_error;
	}

	if ((index < 0) || (index > 7)) {
		log(KERN_ERR, "\"%s\" - index %d out of range 0-7\n", name, index);
		goto vinetic_rtp_channel_register_error;
	}

	spin_lock_bh(&vin->lock);
		
	if (vin->rtp_channels[index]) {
		spin_unlock_bh(&vin->lock);
		log(KERN_ERR, "\"%s\" - vinetic rtp channel=%d already used\n", name, index);
		goto vinetic_rtp_channel_register_error;
	}

	vin->rtp_channels[index] = rtp;

	spin_unlock_bh(&vin->lock);

	// init rtp data
	spin_lock_init(&rtp->lock);

	init_waitqueue_head(&rtp->poll_waitq);
	
	rtp->index = index;
	rtp->vinetic = vin;

	// Add char device to system
	cdev_init(&rtp->cdev, &vinetic_rtp_channel_fops);
	rtp->cdev.owner = owner;
	rtp->cdev.ops = &vinetic_rtp_channel_fops;
	if ((i = cdev_add(&rtp->cdev, devno, 1)) < 0) {
		log(KERN_ERR, "\"%s\" - cdev_add() error=%d\n", name, i);
		goto vinetic_rtp_channel_register_error;
	}
	snprintf(devname, sizeof(devname), "polygator!%s", name);
	CLASS_DEV_CREATE(vinetic_class, devno, NULL, devname);

	verbose("\"%s\" registered\n", name);
	return rtp;

vinetic_rtp_channel_register_error:
	if (slot_alloc) {
		spin_lock(&vinetic_device_list_lock);
		for (i=0; i<VINETIC_DEVICE_MAXCOUNT; i++)
		{
			if (!strcmp(vinetic_device_list[i].name, name)) {
				vinetic_device_list[i].name[0] = '\0';
				vinetic_device_list[i].devno = 0;
				vinetic_device_list[i].type = VINETIC_DEVTYPE_UNKNOWN;
				vinetic_device_list[i].data = NULL;
				break;
			}
		}
		spin_unlock(&vinetic_device_list_lock);
	}
	if(rtp) kfree(rtp);
	return NULL;
}

void vinetic_rtp_channel_unregister(struct vinetic_rtp_channel *rtp)
{
	int i;
	
	if (!rtp) return;

	verbose("\"%s\" unregistered\n", rtp->name);

	spin_lock(&vinetic_device_list_lock);

	for (i=0; i<VINETIC_DEVICE_MAXCOUNT; i++)
	{
		if ((vinetic_device_list[i].type == VINETIC_DEVTYPE_RTPCHAN) &&
				(!strcmp(vinetic_device_list[i].name, rtp->name))) {
			vinetic_device_list[i].name[0] = '\0';
			vinetic_device_list[i].devno = 0;
			vinetic_device_list[i].type = VINETIC_DEVTYPE_UNKNOWN;
			vinetic_device_list[i].data = NULL;
			break;
		}
	}
	spin_unlock(&vinetic_device_list_lock);

	spin_lock_bh(&rtp->vinetic->lock);
	rtp->vinetic->rtp_channels[rtp->index] = NULL;
	spin_unlock_bh(&rtp->vinetic->lock);

	CLASS_DEV_DESTROY(vinetic_class, rtp->devno);
	cdev_del(&rtp->cdev);
	kfree(rtp);
}

static int __init vinetic_init(void)
{
	int rc;
	int i;
	dev_t devno;
	int vinetic_major_reg = 0;

	verbose("loading ...\n");

	// Init vinetic device list
	spin_lock_init(&vinetic_device_list_lock);
	for (i=0; i<VINETIC_DEVICE_MAXCOUNT; i++)
	{
		vinetic_device_list[i].name[0] = '\0';
		vinetic_device_list[i].devno = 0;
		vinetic_device_list[i].type = VINETIC_DEVTYPE_UNKNOWN;
		vinetic_device_list[i].data = NULL;
	}

	// Registering vinetic device class
	vinetic_class = class_create(THIS_MODULE, "vinetic");
	// Register char device region
	if (vinetic_major) {
		devno = MKDEV(vinetic_major, 0);
		rc = register_chrdev_region(devno, VINETIC_DEVICE_MAXCOUNT, "vinetic");
	} else {
		rc = alloc_chrdev_region(&devno, 0, VINETIC_DEVICE_MAXCOUNT, "vinetic");
		if(rc >= 0) vinetic_major = MAJOR(devno);
	}
	if (rc < 0) {
		log(KERN_ERR, "register chrdev region error=%d\n", rc);
		goto vinetic_init_error;
	}
	debug("vinetic major=%d\n", vinetic_major);
	vinetic_major_reg = 1;

	verbose("loaded successfull\n");
	return 0;

vinetic_init_error:
	if (vinetic_major_reg) unregister_chrdev_region(MKDEV(vinetic_major, 0), VINETIC_DEVICE_MAXCOUNT);
	if (vinetic_class) class_destroy(vinetic_class);
	return rc;
}

static void __exit vinetic_exit(void)
{
	// Destroy test file
	CLASS_DEV_DESTROY(vinetic_class, MKDEV(vinetic_major, 0));
	// Unregister char device region
	unregister_chrdev_region(MKDEV(vinetic_major, 0), VINETIC_DEVICE_MAXCOUNT);
	// Destroy vinetic device class
	class_destroy(vinetic_class);

	verbose("stopped\n");
}

module_init(vinetic_init);
module_exit(vinetic_exit);

/******************************************************************************/
/* end of vinetic-base.c                                                      */
/******************************************************************************/
