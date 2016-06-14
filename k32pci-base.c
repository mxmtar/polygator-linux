/******************************************************************************/
/* k32pci-base.c                                                              */
/******************************************************************************/

#include <linux/kobject.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/poll.h>
#include <linux/serial.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/types.h>
#include <linux/version.h>

#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <asm/io.h>
#if defined(CONFIG_COMPAT) && defined(HAVE_COMPAT_IOCTL) && (HAVE_COMPAT_IOCTL == 1)
#include <asm/compat.h>
#endif

#include "polygator/polygator-base.h"
#include "polygator/polygator-k32.h"

#include "polygator/vinetic-base.h"
#include "polygator/vinetic-def.h"

#include "polygator/simcard-base.h"

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,36) // 2,6,30 - orig
#define TTY_PORT
#endif

MODULE_AUTHOR("Maksym Tarasevych <mxmtar@gmail.com>");
MODULE_DESCRIPTION("Polygator Linux module for K32 PCI boards");
MODULE_LICENSE("GPL");

static int rom = 0;
module_param(rom, int, 0);
MODULE_PARM_DESC(tty_major, "Print board's ROM");

#define verbose(_fmt, _args...) printk(KERN_INFO "[polygator-%s] " _fmt, THIS_MODULE->name, ## _args)
#define log(_level, _fmt, _args...) printk(_level "[polygator-%s] %s:%d - %s(): " _fmt, THIS_MODULE->name, "k32pci-base.c", __LINE__, __PRETTY_FUNCTION__, ## _args)
#define debug(_fmt, _args...) printk(KERN_DEBUG "[polygator-%s] %s:%d - %s(): " _fmt, THIS_MODULE->name, "k32pci-base.c", __LINE__, __PRETTY_FUNCTION__, ## _args)

/*! */
#define PG_PCI_NUM_BASE			0x00
#define PG_PCI_OFFSET_RESET		0x00
#define PG_PCI_ID_BASE			0x28
#define PG_PCI_AT_BASE			0x30
#define PG_PCI_ST_BASE			0x04
#define PG_PCI_CTRL_BASE		0x04
#define PG_PCI_ROM_BASE			0x24
#define PG_PCI_VIN_ST_BASE		0x2C
#define PG_PCI_VIN_DATA_BASE	0x50
#define PG_PCI_SIM_BASE			0x60
#define PG_PCI_IMEI_BASE		0xA0
/*! */

static int k32pci_tty_at_open(struct tty_struct *tty, struct file *filp);
static void k32pci_tty_at_close(struct tty_struct *tty, struct file *filp);
static int k32pci_tty_at_write(struct tty_struct *tty, const unsigned char *buf, int count);
static int k32pci_tty_at_write_room(struct tty_struct *tty);
static int k32pci_tty_at_chars_in_buffer(struct tty_struct *tty);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20)
static void k32pci_tty_at_set_termios(struct tty_struct *tty, struct ktermios *old_termios);
#else
static void k32pci_tty_at_set_termios(struct tty_struct *tty, struct termios *old_termios);
#endif
static void k32pci_tty_at_flush_buffer(struct tty_struct *tty);
static void k32pci_tty_at_hangup(struct tty_struct *tty);

static struct tty_operations k32pci_tty_at_ops = {
	.open = k32pci_tty_at_open,
	.close = k32pci_tty_at_close,
	.write = k32pci_tty_at_write,
	.write_room = k32pci_tty_at_write_room,
	.chars_in_buffer = k32pci_tty_at_chars_in_buffer,
	.set_termios = k32pci_tty_at_set_termios,
	.flush_buffer = k32pci_tty_at_flush_buffer,
	.hangup = k32pci_tty_at_hangup,
};
#ifdef TTY_PORT
static int k32pci_tty_at_port_carrier_raised(struct tty_port *port);
static void k32pci_tty_at_port_dtr_rts(struct tty_port *port, int onoff);
static int k32pci_tty_at_port_activate(struct tty_port *tport, struct tty_struct *tty);
static void k32pci_tty_at_port_shutdown(struct tty_port *port);

static const struct tty_port_operations k32pci_tty_at_port_ops = {
	.carrier_raised = k32pci_tty_at_port_carrier_raised,
	.dtr_rts = k32pci_tty_at_port_dtr_rts,
	.activate = k32pci_tty_at_port_activate,
	.shutdown = k32pci_tty_at_port_shutdown,
};
#endif
static struct pci_device_id k32pci_board_id_table[] = {
	{ PCI_DEVICE(0xDEAD, 0xBEEF), .driver_data = 1, },
	{ 0, },
};
MODULE_DEVICE_TABLE(pci, k32pci_board_id_table);

static void k32pci_vin_reset_0(uintptr_t cbdata)
{
	outb(0x02, cbdata + PG_PCI_OFFSET_RESET);
	mdelay(10);
	outb(0x00, cbdata + PG_PCI_OFFSET_RESET);
	mdelay(10);
	outb(0x02, cbdata + PG_PCI_OFFSET_RESET);
	mdelay(10);
	outb(0x00, cbdata + PG_PCI_OFFSET_RESET);
	mdelay(2);
}

static void k32pci_vin_reset_1(uintptr_t cbdata)
{
	outb(0x04, cbdata + PG_PCI_OFFSET_RESET);
	mdelay(10);
	outb(0x00, cbdata + PG_PCI_OFFSET_RESET);
	mdelay(10);
	outb(0x04, cbdata + PG_PCI_OFFSET_RESET);
	mdelay(10);
	outb(0x00, cbdata + PG_PCI_OFFSET_RESET);
	mdelay(2);
}

static void k32pci_vin_write_nwd_0(uintptr_t cbdata, u_int16_t value)
{
	outw(value, cbdata + PG_PCI_VIN_DATA_BASE + 0 + 0);
}

static void k32pci_vin_write_nwd_1(uintptr_t cbdata, u_int16_t value)
{
	outw(value, cbdata + PG_PCI_VIN_DATA_BASE + 8 + 0);
}

static void k32pci_vin_write_eom_0(uintptr_t cbdata, u_int16_t value)
{
	outw(value, cbdata + PG_PCI_VIN_DATA_BASE + 0 + 4);
}

static void k32pci_vin_write_eom_1(uintptr_t cbdata, u_int16_t value)
{
	outw(value, cbdata + PG_PCI_VIN_DATA_BASE + 8 + 4);
}

static u_int16_t k32pci_vin_read_nwd_0(uintptr_t cbdata)
{
	u_int16_t value = inw(cbdata + PG_PCI_VIN_DATA_BASE + 0 + 0);
	return value;
}

static u_int16_t k32pci_vin_read_nwd_1(uintptr_t cbdata)
{
	u_int16_t value = inw(cbdata + PG_PCI_VIN_DATA_BASE + 8 + 0);
	return value;
}

static u_int16_t k32pci_vin_read_eom_0(uintptr_t cbdata)
{
	u_int16_t value = inw(cbdata + PG_PCI_VIN_DATA_BASE + 0 + 4);
	return value;
}

static u_int16_t k32pci_vin_read_eom_1(uintptr_t cbdata)
{
	u_int16_t value = inw(cbdata + PG_PCI_VIN_DATA_BASE + 8 + 4);
	return value;
}

static size_t k32pci_vin_is_not_ready_0(uintptr_t cbdata)
{
	size_t st = (inb(cbdata + PG_PCI_VIN_ST_BASE) >> 0) & 1;
	return st;
}

static size_t k32pci_vin_is_not_ready_1(uintptr_t cbdata)
{
	size_t st = (inb(cbdata + PG_PCI_VIN_ST_BASE) >> 1) & 1;
	return st;
}

static u_int16_t k32pci_vin_read_dia_0(uintptr_t cbdata)
{
	return 0;
}

static u_int16_t k32pci_vin_read_dia_1(uintptr_t cbdata)
{
	return 0;
}

static void k32pci_gsm_mod_set_control(uintptr_t cbdata, size_t pos, u_int8_t reg)
{
	outb(reg, cbdata + PG_PCI_CTRL_BASE + pos * 4);
}

static u_int8_t k32pci_gsm_mod_get_status(uintptr_t cbdata, size_t pos)
{
	return inb(cbdata + PG_PCI_CTRL_BASE + pos * 4);
}

static void k32pci_gsm_mod_at_write(uintptr_t cbdata, size_t pos, u_int8_t reg)
{
	outb(reg, cbdata + PG_PCI_AT_BASE + pos * 4);
}

static u_int8_t k32pci_gsm_mod_at_read(uintptr_t cbdata, size_t pos)
{
	return inb(cbdata + PG_PCI_AT_BASE + pos * 4);
}

static void k32pci_gsm_mod_sim_write(uintptr_t cbdata, size_t pos, u_int8_t reg)
{
	outb(reg, cbdata + PG_PCI_SIM_BASE + pos * 4);
}

static u_int8_t k32pci_gsm_mod_sim_read(uintptr_t cbdata, size_t pos)
{
	return inb(cbdata + PG_PCI_SIM_BASE + pos * 4);
}

static void k32pci_gsm_mod_imei_write(uintptr_t cbdata, size_t pos, u_int8_t reg)
{
	outb(reg, cbdata + PG_PCI_IMEI_BASE + pos * 4);
}

static u_int8_t k32pci_gsm_mod_imei_read(uintptr_t cbdata, size_t pos)
{
	return inb(cbdata + PG_PCI_IMEI_BASE + pos * 4);
}

static u_int8_t k32pci_sim_read(void *data)
{
	struct k32_gsm_module_data *mod = (struct k32_gsm_module_data *)data;

	return mod->sim_read(mod->cbdata, mod->pos_on_board);
}

static void k32pci_sim_write(void *data, u_int8_t value)
{
	struct k32_gsm_module_data *mod = (struct k32_gsm_module_data *)data;

	mod->sim_write(mod->cbdata, mod->pos_on_board, value);
}

static int k32pci_sim_is_read_ready(void *data)
{
	union k32_gsm_mod_status_reg status;
	struct k32_gsm_module_data *mod = (struct k32_gsm_module_data *)data;

	status.full = mod->get_status(mod->cbdata, mod->pos_on_board);

	return !status.bits.sim_rdy_rd;
}

static int k32pci_sim_is_write_ready(void *data)
{
	union k32_gsm_mod_status_reg status;
	struct k32_gsm_module_data *mod = (struct k32_gsm_module_data *)data;

	status.full = mod->get_status(mod->cbdata, mod->pos_on_board);

	return status.bits.sim_rdy_wr;
}

static int k32pci_sim_is_reset_request(void *data)
{
	union k32_gsm_mod_status_reg status;
	struct k32_gsm_module_data *mod = (struct k32_gsm_module_data *)data;

	status.full = mod->get_status(mod->cbdata, mod->pos_on_board);

	return status.bits.sim_rst_req;
}

static void k32pci_sim_set_speed(void *data, int speed)
{
	struct k32_gsm_module_data *mod = (struct k32_gsm_module_data *)data;

	switch (speed) {
		case 0x94:
		case 57600:
			mod->control.bits.sim_spd_0 = 1;
			mod->control.bits.sim_spd_1 = 0;
			break;
		case 0x95:
		case 115200:
			mod->control.bits.sim_spd_0 = 0;
			mod->control.bits.sim_spd_1 = 1;
			break;
		case 0x96:
		case 230400:
			mod->control.bits.sim_spd_0 = 1;
			mod->control.bits.sim_spd_1 = 1;
			break;
		case 0x11:
		default: // 9600 
			mod->control.bits.sim_spd_0 = 0;
			mod->control.bits.sim_spd_1 = 0;
			break;
	}

	mod->set_control(mod->cbdata, mod->pos_on_board, mod->control.full);
}

static void k32pci_sim_do_after_reset(void *data)
{
}

static void k32pci_tty_at_poll(unsigned long addr)
{
	unsigned char buff[512];
	size_t len;
	size_t xmit_write_room;
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,9,0)
	struct tty_struct *tty;
#endif
	union k32_gsm_mod_status_reg status;
	struct k32_gsm_module_data *mod = (struct k32_gsm_module_data *)addr;

	len = 0;

	// read received data
	while (len < sizeof(buff)) {
		// read status register
		status.full = mod->get_status(mod->cbdata, mod->pos_on_board);
		// select port
		if (mod->at_port_select) {
			// auxilary
			if (status.bits.imei_rdy_rd) {
				break;
			}
			// put char to receiving buffer
			buff[len++] = mod->imei_read(mod->cbdata, mod->pos_on_board);
		} else {
			// main
			if (status.bits.at_rdy_rd) {
				break;
			}
			// put char to receiving buffer
			buff[len++] = mod->at_read(mod->cbdata, mod->pos_on_board);
		}
	}

	spin_lock(&mod->at_lock);

	while (mod->at_xmit_count) {
		// read status register
		status.full = mod->get_status(mod->cbdata, mod->pos_on_board);
		// select port
		if (mod->at_port_select) {
			// auxilary
			// check for transmitter is ready
			if (status.bits.imei_rdy_wr) {
				// put char to transmitter buffer
#ifdef TTY_PORT
				mod->imei_write(mod->cbdata, mod->pos_on_board, mod->at_port.xmit_buf[mod->at_xmit_tail]);
#else
				mod->imei_write(mod->cbdata, mod->pos_on_board, mod->at_xmit_buf[mod->at_xmit_tail]);
#endif
				mod->at_xmit_tail++;
				if (mod->at_xmit_tail == SERIAL_XMIT_SIZE) {
					mod->at_xmit_tail = 0;
				}
				mod->at_xmit_count--;
			}
		} else {
			// main
			// check for transmitter is ready
			if (status.bits.at_rdy_wr) {
				// check AT transmitter work style
				if (mod->at_no_buf) {
					// put char to transmitter buffer
#ifdef TTY_PORT
					mod->at_write(mod->cbdata, mod->pos_on_board, mod->at_port.xmit_buf[mod->at_xmit_tail]);
#else
					mod->at_write(mod->cbdata, mod->pos_on_board, mod->at_xmit_buf[mod->at_xmit_tail]);
#endif
					mod->at_xmit_tail++;
					if (mod->at_xmit_tail == SERIAL_XMIT_SIZE) {
						mod->at_xmit_tail = 0;
					}
					mod->at_xmit_count--;
				} else {
					xmit_write_room = 1024;
					while ((mod->at_xmit_count) && (xmit_write_room)) {
						// put char to transmitter buffer
#ifdef TTY_PORT
						mod->at_write(mod->cbdata, mod->pos_on_board, mod->at_port.xmit_buf[mod->at_xmit_tail]);
#else
						mod->at_write(mod->cbdata, mod->pos_on_board, mod->at_xmit_buf[mod->at_xmit_tail]);
#endif
						mod->at_xmit_tail++;
						if (mod->at_xmit_tail == SERIAL_XMIT_SIZE) {
							mod->at_xmit_tail = 0;
						}
						mod->at_xmit_count--;
						xmit_write_room--;
					}
					break;
				}
			}
		}
	}

	spin_unlock(&mod->at_lock);

	if (len) {
#ifdef TTY_PORT
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,9,0)
		tty_insert_flip_string(&mod->at_port, buff, len);
		tty_flip_buffer_push(&mod->at_port);
#else
		tty = tty_port_tty_get(&mod->at_port);
		tty_insert_flip_string(tty, buff, len);
		tty_flip_buffer_push(tty);
		tty_kref_put(tty);
#endif
#else
		tty = mod->at_tty;
		tty_insert_flip_string(tty, buff, len);
		tty_flip_buffer_push(tty);
#endif
	}

	mod_timer(&mod->at_poll_timer, jiffies + 1);
}

static int k32pci_board_open(struct inode *inode, struct file *filp)
{
	ssize_t res;
	size_t i,j;
	size_t len;

	struct k32_board *board;
	struct k32_board_private_data *private_data;
	union k32_gsm_mod_status_reg status;
	struct k32_gsm_module_data *mod;

	board = container_of(inode->i_cdev, struct k32_board, cdev);

	if (!(private_data = kmalloc(sizeof(struct k32_board_private_data), GFP_KERNEL))) {
		log(KERN_ERR, "can't get memory=%lu bytes\n", (unsigned long int)sizeof(struct k32_board_private_data));
		res = -ENOMEM;
		goto k32pci_board_open_error;
	}
	private_data->board = board;

	len = 0;
	// type
	len += sprintf(private_data->buff + len, "TYPE=%u\r\n", board->type & 0x00ff);
	// position
	len += sprintf(private_data->buff + len, "POSITION=%u\r\n", board->position);
	// gsm
	for (i = 0; i < 8; i++) {
		if ((mod = board->gsm_modules[i])) {
			status.full = mod->get_status(mod->cbdata, mod->pos_on_board);
			len += sprintf(private_data->buff + len, "GSM%lu %s %s %s VIN%luALM%lu VIO=%u\r\n",
							(unsigned long int)i,
							polygator_print_gsm_module_type(mod->type),
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
							board->tty_at_channels[i]?dev_name(board->tty_at_channels[i]->device):"unknown",
							board->simcard_channels[i]?dev_name(board->simcard_channels[i]->device):"unknown",
#else
							board->tty_at_channels[i]?board->tty_at_channels[i]->device->class_id:"unknown",
							board->simcard_channels[i]?board->simcard_channels[i]->device->class_id:"unknown",
#endif
							(unsigned long int)(i/4),
							(unsigned long int)(i%4),
							status.bits.vio);
		}
	}
	// vinetic
	for (i = 0; i < 2; i++) {
		if (board->vinetics[i]) {
			len += sprintf(private_data->buff + len, "VIN%lu %s\r\n",
						   (unsigned long int)i,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
							dev_name(board->vinetics[i]->device)
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
							dev_name(board->vinetics[i]->device)
#else
							board->vinetics[i]->device->class_id
#endif
							);
			for (j = 0; j < 4; j++) {
				if (board->vinetics[i]->rtp_channels[j])
					len += sprintf(private_data->buff + len, "VIN%luRTP%lu %s\r\n",
								(unsigned long int)i,
								(unsigned long int)j,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
								dev_name(board->vinetics[i]->rtp_channels[j]->device)
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
								dev_name(board->vinetics[i]->rtp_channels[j]->device)
#else
								board->vinetics[i]->rtp_channels[j]->device->class_id
#endif
								);
			}
		}
	}

	private_data->length = len;

	filp->private_data = private_data;

	return 0;

k32pci_board_open_error:
	if (private_data) {
		kfree(private_data);
	}
	return res;
}

static int k32pci_board_release(struct inode *inode, struct file *filp)
{
	struct k32_board_private_data *private_data = filp->private_data;

	kfree(private_data);
	return 0;
}

static ssize_t k32pci_board_read(struct file *filp, char __user *buff, size_t count, loff_t *offp)
{
	size_t len;
	ssize_t res;
	struct k32_board_private_data *private_data = filp->private_data;

	res = (private_data->length > filp->f_pos)?(private_data->length - filp->f_pos):(0);

	if (res) {
		len = res;
		len = min(count, len);
		if (copy_to_user(buff, private_data->buff + filp->f_pos, len)) {
			res = -EINVAL;
			goto k32pci_board_read_end;
		}
		*offp = filp->f_pos + len;
	}

k32pci_board_read_end:
	return res;
}

static ssize_t k32pci_board_write(struct file *filp, const char __user *buff, size_t count, loff_t *offp)
{
	ssize_t res;
	char cmd[256];
	size_t len;

	u_int32_t chan;
	u_int32_t value;
	struct k32_gsm_module_data *mod;
	struct k32_board_private_data *private_data = filp->private_data;

	memset(cmd, 0, sizeof(cmd));
	len = sizeof(cmd) - 1;
	len = min(len,count);

	if (copy_from_user(cmd, buff, len)) {
		res = -EINVAL;
		goto k32pci_board_write_end;
	}

	if (sscanf(cmd, "GSM%u PWR=%u", &chan, &value) == 2) {
		if ((chan >= 0) && (chan <= 7) && (private_data->board->gsm_modules[chan])) {
			mod = private_data->board->gsm_modules[chan];
			mod->control.bits.pwr_off = !value;
			mod->set_control(mod->cbdata, mod->pos_on_board, mod->control.full);
			res = len;
		} else {
			res = - ENODEV;
		}
	} else if (sscanf(cmd, "GSM%u KEY=%u", &chan, &value) == 2) {
		if ((chan >= 0) && (chan <= 7) && (private_data->board->gsm_modules[chan])) {
			mod = private_data->board->gsm_modules[chan];
			mod->control.bits.mod_off = !value;
			mod->set_control(mod->cbdata, mod->pos_on_board, mod->control.full);
			res = len;
		} else {
			res = -ENODEV;
		}
	} else if (sscanf(cmd, "GSM%u BAUDRATE=%u", &chan, &value) == 2) {
		if ((chan >= 0) && (chan <= 7) && (private_data->board->gsm_modules[chan])) {
			mod = private_data->board->gsm_modules[chan];
			if (value == 9600) {
				mod->control.bits.com_spd = 3;
			} else {
				mod->control.bits.com_spd = 2;
			}
			mod->set_control(mod->cbdata, mod->pos_on_board, mod->control.full);
			res = len;
		} else {
			res = -ENODEV;
		}
	} else if (sscanf(cmd, "GSM%u COM=%u", &chan, &value) == 2) {
		if ((chan >= 0) && (chan <= 7) && (private_data->board->gsm_modules[chan])) {
			mod = private_data->board->gsm_modules[chan];
			mod->control.bits.rst = value;
			mod->set_control(mod->cbdata, mod->pos_on_board, mod->control.full);
			res = len;
		} else {
			res = -ENODEV;
		}
	} else if (sscanf(cmd, "GSM%u SERIAL=%u", &chan, &value) == 2) {
		if ((chan >= 0) && (chan <= 7) && (private_data->board->gsm_modules[chan])) {
			mod = private_data->board->gsm_modules[chan];
			if (mod->type == POLYGATOR_MODULE_TYPE_SIM300) {
				mod->at_port_select = value;
			}
			res = len;
		} else {
			res = -ENODEV;
		}
	} else {
		res = -ENOMSG;
	}

k32pci_board_write_end:
	return res;
}

static struct file_operations k32pci_board_fops = {
	.owner   = THIS_MODULE,
	.open    = k32pci_board_open,
	.release = k32pci_board_release,
	.read    = k32pci_board_read,
	.write   = k32pci_board_write,
};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0)
static int k32pci_board_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
#else
static int __devinit k32pci_board_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
#endif
{
	int rc;
	unsigned long addr;
	u_int32_t pow10;
	size_t i, j;
	char devname[POLYGATOR_BRDNAME_MAXLEN];
	struct k32_gsm_module_data *mod;
	struct k32_board *board = NULL;
	int get_pci_region = 0;

	rc = pci_enable_device(pdev);
	if (rc) {
		dev_err(&pdev->dev, "can't enable pci device\n");
		goto k32pci_board_probe_error;
	}
	rc = pci_request_region(pdev, 0, "k32pci");
	if (rc) {
		dev_err(&pdev->dev, "can't request I/O region\n");
		goto k32pci_board_probe_error;
	}
	get_pci_region = 1;

	// alloc memory for board data
	if (!(board = kmalloc(sizeof(struct k32_board), GFP_KERNEL))) {
		log(KERN_ERR, "can't get memory for struct k32_board\n");
		rc = -1;
		goto k32pci_board_probe_error;
	}
	memset(board, 0, sizeof(struct k32_board));

	// get starting address
	addr = pci_resource_start(pdev, 0);

	// reset board
	outb(0xff, addr + PG_PCI_OFFSET_RESET);
	mdelay(10);
	outb(0x00, addr + PG_PCI_OFFSET_RESET);

	// get board type
	board->type = 0;
	for (i = 0; i < 16; i++) {
		board->type <<= 1;
		board->type |= inb(addr + PG_PCI_ID_BASE) & 0x01;
	}
	verbose("found PCI board type=%04x\n", board->type & 0x00ff);

	if (((board->type & 0x00ff) != 0x0081) && ((board->type & 0x00ff) != 0x0082) &&
			((board->type & 0x00ff) != 0x0083) && ((board->type & 0x00ff) != 0x0084) &&
			((board->type & 0x00ff) != 0x0085)) {
		log(KERN_ERR, "PCI board type=%04x unsupported\n", board->type & 0x00ff);
		rc = -1;
		goto k32pci_board_probe_error;
	}

	// get board number
	board->position = inb(addr + PG_PCI_NUM_BASE) & 3;

	// read board rom
	memset(board->rom, 0, 256);
	board->romsize = inb(addr + PG_PCI_ROM_BASE);
	board->romsize = inb(addr + PG_PCI_ROM_BASE);
	for (i = 0; i < board->romsize; i++) {
		board->rom[i] = inb(addr + PG_PCI_ROM_BASE);
	}
	if (rom) {
		verbose("\"%.*s\"\n", (int)board->romsize, board->rom);
	}
	// get board serial number
	i = board->romsize - 1;
	pow10 = 1;
	board->sn = 0;
	while (i--) {
		if ((board->rom[i] < 0x30) || (board->rom[i] > 0x39)) {
			break;
		}
		board->sn += (board->rom[i] - 0x30) * pow10;
		pow10 *= 10;
	}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
	snprintf(devname, POLYGATOR_BRDNAME_MAXLEN, "board-k32pci-%u", board->sn);
#else
	snprintf(devname, POLYGATOR_BRDNAME_MAXLEN, "bp%u", board->sn);
#endif
	if (!(board->pg_board =  polygator_board_register(THIS_MODULE, devname, &board->cdev, &k32pci_board_fops))) {
		rc = -1;
		goto k32pci_board_probe_error;
	}

	for (j = 0; j < 2; j++) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
		snprintf(devname, VINETIC_DEVNAME_MAXLEN, "board-k32pci-%u-vin%lu", board->sn, (unsigned long int)j);
#else
		snprintf(devname, VINETIC_DEVNAME_MAXLEN, "vp%u%lu", board->sn, (unsigned long int)j);
#endif
		if (!(board->vinetics[j] = vinetic_device_register(THIS_MODULE, devname, addr,
													(j)?(k32pci_vin_reset_1):(k32pci_vin_reset_0),
													(j)?(k32pci_vin_is_not_ready_1):(k32pci_vin_is_not_ready_0),
													(j)?(k32pci_vin_write_nwd_1):(k32pci_vin_write_nwd_0),
													(j)?(k32pci_vin_write_eom_1):(k32pci_vin_write_eom_0),
													(j)?(k32pci_vin_read_nwd_1):(k32pci_vin_read_nwd_0),
													(j)?(k32pci_vin_read_eom_1):(k32pci_vin_read_eom_0),
													(j)?(k32pci_vin_read_dia_1):(k32pci_vin_read_dia_0)))) {
			rc = -1;
			goto k32pci_board_probe_error;
		}
		for (i = 0; i < 4; i++) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
			snprintf(devname, VINETIC_DEVNAME_MAXLEN, "board-k32pci-%u-vin%lu-rtp%lu", board->sn, (unsigned long int)j, (unsigned long int)i);
#else
			snprintf(devname, VINETIC_DEVNAME_MAXLEN, "rp%u%lu%lu", board->sn, (unsigned long int)j, (unsigned long int)i);
#endif
			if (!vinetic_rtp_channel_register(THIS_MODULE, devname, board->vinetics[j], i)) {
				rc = -1;
				goto k32pci_board_probe_error;
			}
		}
	}

	// set GSM module data
	for (i = 0; i < 8; i++) {
		if (!(mod = kmalloc(sizeof(struct k32_gsm_module_data), GFP_KERNEL))) {
			log(KERN_ERR, "can't get memory for struct k32_gsm_module_data\n");
			rc = -1;
			goto k32pci_board_probe_error;
		}
		memset(mod, 0, sizeof(struct k32_gsm_module_data));
		// select GSM module type
		if (((board->type & 0x00ff) == 0x0082) || ((board->type & 0x00ff) == 0x0083) ||
				((board->type & 0x00ff) == 0x0084) || ((board->type & 0x00ff) == 0x0085)) {
			if (board->rom[8] == '*') {
				if (board->rom[i] == 'M') {
					mod->type = POLYGATOR_MODULE_TYPE_M10;
				} else if (board->rom[i] == '9') {
					mod->type = POLYGATOR_MODULE_TYPE_SIM900;
				} else if (board->rom[i] == 'S'){
					mod->type = POLYGATOR_MODULE_TYPE_SIM300;
				} else if (board->rom[i] == 'G') {
					mod->type = POLYGATOR_MODULE_TYPE_SIM5215A2;
				} else if (board->rom[i] == 'g') {
					mod->type = POLYGATOR_MODULE_TYPE_SIM5215;
				} else {
					mod->type = POLYGATOR_MODULE_TYPE_UNKNOWN;
				}
			} else {
				mod->type = POLYGATOR_MODULE_TYPE_SIM300;
			}
		} else {
			mod->type = POLYGATOR_MODULE_TYPE_SIM300;
		}

		if ((board->type & 0x00ff) == 0x0085) {
			mod->at_no_buf = 0;
		} else {
			mod->at_no_buf = 1;
		}

		if (mod->type == POLYGATOR_MODULE_TYPE_SIM300) {
			mod->control.bits.mod_off = 1;		// module inactive
			mod->control.bits.sim_spd_0 = 0;
			mod->control.bits.sim_spd_1 = 0;
			mod->control.bits.rst = 0;			// M10=1 SIM300=0
			mod->control.bits.pwr_off = 1;		// power suply disabled
			mod->control.bits.sync_mode = 1;	// 0 - synchronous, 1 - asynchronous
			mod->control.bits.com_spd = 2;		// 3 - 9600, 2 - 115200
		} else if (mod->type == POLYGATOR_MODULE_TYPE_SIM900) {
			mod->control.bits.mod_off = 1;		// module inactive
			mod->control.bits.sim_spd_0 = 0;
			mod->control.bits.sim_spd_1 = 0;
			mod->control.bits.rst = 0;			// M10=1 SIM300=0
			mod->control.bits.pwr_off = 1;		// power suply disabled
			mod->control.bits.sync_mode = 1;	// 0 - synchronous, 1 - asynchronous
			mod->control.bits.com_spd = 2;		// 3 - 9600, 2 - 115200
		} else if ((mod->type == POLYGATOR_MODULE_TYPE_SIM5215) || (mod->type == POLYGATOR_MODULE_TYPE_SIM5215A2)) {
			mod->control.bits.mod_off = 1;		// module inactive
			mod->control.bits.sim_spd_0 = 0;
			mod->control.bits.sim_spd_1 = 0;
			mod->control.bits.rst = 0;			// M10=1 SIM300=0
			mod->control.bits.pwr_off = 1;		// power suply disabled
			mod->control.bits.sync_mode = 1;	// 0 - synchronous, 1 - asynchronous
			mod->control.bits.com_spd = 2;		// 3 - 9600, 2 - 115200
		} else if (mod->type == POLYGATOR_MODULE_TYPE_M10) {
			mod->control.bits.mod_off = 1;		// module inactive
			mod->control.bits.sim_spd_0 = 0;
			mod->control.bits.sim_spd_1 = 0;
			mod->control.bits.rst = 1;			// M10=1 SIM300=0
			mod->control.bits.pwr_off = 1;		// power suply disabled
			mod->control.bits.sync_mode = 1;	// 0 - synchronous, 1 - asynchronous
			mod->control.bits.com_spd = 2;		// 3 - 9600, 2 - 115200
		} else {
			kfree(mod);
			continue;
		}

		mod->pos_on_board = i;
		mod->cbdata = addr;
		mod->set_control = k32pci_gsm_mod_set_control;
		mod->get_status = k32pci_gsm_mod_get_status;
		mod->at_write = k32pci_gsm_mod_at_write;
		mod->at_read = k32pci_gsm_mod_at_read;
		mod->sim_write = k32pci_gsm_mod_sim_write;
		mod->sim_read = k32pci_gsm_mod_sim_read;
		mod->imei_write = k32pci_gsm_mod_imei_write;
		mod->imei_read = k32pci_gsm_mod_imei_read;

// 		mod->set_control(mod->cbdata, mod->pos_on_board, mod->control.full);
		init_timer(&mod->at_poll_timer);

		spin_lock_init(&mod->at_lock);
#ifdef TTY_PORT
		tty_port_init(&mod->at_port);
		mod->at_port.ops = &k32pci_tty_at_port_ops;
		mod->at_port.close_delay = 0;
		mod->at_port.closing_wait = ASYNC_CLOSING_WAIT_NONE;
#endif
		board->gsm_modules[i] = mod;
	}

	// register polygator tty at device
	for (i = 0; i < 8; i++) {
		if ((mod = board->gsm_modules[i])) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0)
			if (!(board->tty_at_channels[i] = polygator_tty_device_register(THIS_MODULE, mod, &mod->at_port, &k32pci_tty_at_ops))) {
#else
			if (!(board->tty_at_channels[i] = polygator_tty_device_register(THIS_MODULE, mod, &k32pci_tty_at_ops))) {
#endif
				log(KERN_ERR, "can't register polygator tty device\n");
				rc = -1;
				goto k32pci_board_probe_error;
			}
		}
	}

	// register polygator simcard device
	for (i = 0; i < 8; i++) {
		if (board->gsm_modules[i]) {
			if (!(board->simcard_channels[i] = simcard_device_register(THIS_MODULE,
																		board->gsm_modules[i],
																		k32pci_sim_read,
																		k32pci_sim_write,
																		k32pci_sim_is_read_ready,
																		k32pci_sim_is_write_ready,
																		k32pci_sim_is_reset_request,
																		k32pci_sim_set_speed,
																		k32pci_sim_do_after_reset))) {
				log(KERN_ERR, "can't register polygator simcard device\n");
				rc = -1;
				goto k32pci_board_probe_error;
			}
		}
	}

	pci_set_drvdata(pdev, board);
	return 0;

k32pci_board_probe_error:
	if (board) {
		for (i = 0; i < 8; i++) {
			if (board->simcard_channels[i]) {
				simcard_device_unregister(board->simcard_channels[i]);
			}
			if (board->tty_at_channels[i]) {
				polygator_tty_device_unregister(board->tty_at_channels[i]);
			}
			if (board->gsm_modules[i]) {
				del_timer_sync(&board->gsm_modules[i]->at_poll_timer);
				kfree(board->gsm_modules[i]);
			}
		}
		for (j = 0; j < 2; j++) {
			if (board->vinetics[j]) {
				for (i = 0; i < 4; i++) {
					if (board->vinetics[j]->rtp_channels[i]) {
						vinetic_rtp_channel_unregister(board->vinetics[j]->rtp_channels[i]);
					}
				}
				vinetic_device_unregister(board->vinetics[j]);
			}
		}
		if (board->pg_board) {
			polygator_board_unregister(board->pg_board);
		}
		kfree(board);
	}
	if (get_pci_region) {
		pci_release_region(pdev, 0);
	}
	return rc;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0)
static void k32pci_board_remove(struct pci_dev *pdev)
#else
static void __devexit k32pci_board_remove(struct pci_dev *pdev)
#endif
{
	size_t i, j;
	
	struct k32_board *board = pci_get_drvdata(pdev);

	for (i = 0; i < 8; i++) {
		if (board->simcard_channels[i]) {
			simcard_device_unregister(board->simcard_channels[i]);
		}
		if (board->tty_at_channels[i]) {
			polygator_tty_device_unregister(board->tty_at_channels[i]);
		}
		if (board->gsm_modules[i]) {
			del_timer_sync(&board->gsm_modules[i]->at_poll_timer);
			kfree(board->gsm_modules[i]);
		}
	}

	for (j = 0; j < 2; j++) {
		for (i = 0; i < 4; i++) {
			vinetic_rtp_channel_unregister(board->vinetics[j]->rtp_channels[i]);
		}
		vinetic_device_unregister(board->vinetics[j]);
	}

	polygator_board_unregister(board->pg_board);

	kfree(board);
	pci_release_region(pdev, 0);
}

static struct pci_driver k32pci_driver = {
	.name = "k32pci",
	.id_table = k32pci_board_id_table,
	.probe = k32pci_board_probe,
	.remove = k32pci_board_remove,
};

static int k32pci_tty_at_open(struct tty_struct *tty, struct file *filp)
{
	struct polygator_tty_device *ptd = tty->driver_data;
	struct k32_gsm_module_data *mod = (struct k32_gsm_module_data *)ptd->data;

#ifdef TTY_PORT
	return tty_port_open(&mod->at_port, tty, filp);
#else
	unsigned char *xbuf;
	
	if (!(xbuf = kmalloc(SERIAL_XMIT_SIZE, GFP_KERNEL))) {
		return -ENOMEM;
	}

	spin_lock_bh(&mod->at_lock);

	if (!mod->at_count++) {
		mod->at_xmit_buf = xbuf;
		mod->at_xmit_count = mod->at_xmit_head = mod->at_xmit_tail = 0;

		mod->at_poll_timer.function = k32pci_tty_at_poll;
		mod->at_poll_timer.data = (unsigned long)mod;
		mod->at_poll_timer.expires = jiffies + 1;
		add_timer(&mod->at_poll_timer);
	
		mod->at_tty = tty;
	} else {
		kfree(xbuf);
	}

	spin_unlock_bh(&mod->at_lock);

	return 0;
#endif
}

static void k32pci_tty_at_close(struct tty_struct *tty, struct file *filp)
{
	struct polygator_tty_device *ptd = tty->driver_data;
	struct k32_gsm_module_data *mod = (struct k32_gsm_module_data *)ptd->data;

#ifdef TTY_PORT
	tty_port_close(&mod->at_port, tty, filp);
#else
	unsigned char *xbuf = NULL;

	spin_lock_bh(&mod->at_lock);

	if (!--mod->at_count) {
		xbuf = mod->at_xmit_buf;
		mod->at_tty = NULL;
	}

	spin_unlock_bh(&mod->at_lock);
	
	if (xbuf) {
		del_timer_sync(&mod->at_poll_timer);
		kfree(mod->at_xmit_buf);
	}
#endif
	return;
}

static int k32pci_tty_at_write(struct tty_struct *tty, const unsigned char *buf, int count)
{
	int res = 0;
	size_t len;
	struct polygator_tty_device *ptd = tty->driver_data;
	struct k32_gsm_module_data *mod = (struct k32_gsm_module_data *)ptd->data;
	unsigned char *bp = (unsigned char *)buf;

	spin_lock_bh(&mod->at_lock);

	if (mod->at_xmit_count < SERIAL_XMIT_SIZE) {
		while (1) {
			if (mod->at_xmit_head == mod->at_xmit_tail) {
				if (mod->at_xmit_count) {
					len = 0;
				} else {
					len = SERIAL_XMIT_SIZE - mod->at_xmit_head;
				}
			} else if (mod->at_xmit_head > mod->at_xmit_tail) {
				len = SERIAL_XMIT_SIZE - mod->at_xmit_head;
			} else {
				len = mod->at_xmit_tail - mod->at_xmit_head;
			}
			len = min(len, (size_t)count);
			if (!len) {
				break;
			}
#ifdef TTY_PORT
			memcpy(mod->at_port.xmit_buf + mod->at_xmit_head, bp, len);
#else
			memcpy(mod->at_xmit_buf + mod->at_xmit_head, bp, len);
#endif
			mod->at_xmit_head += len;
			if (mod->at_xmit_head == SERIAL_XMIT_SIZE) {
				mod->at_xmit_head = 0;
			}
			mod->at_xmit_count += len;
			bp += len;
			count -= len;
			res += len;
		}
	}

	spin_unlock_bh(&mod->at_lock);

	return res ;
}

static int k32pci_tty_at_write_room(struct tty_struct *tty)
{
	int res;
	struct polygator_tty_device *ptd = tty->driver_data;
	struct k32_gsm_module_data *mod = (struct k32_gsm_module_data *)ptd->data;

	spin_lock_bh(&mod->at_lock);

	res = SERIAL_XMIT_SIZE - mod->at_xmit_count;

	spin_unlock_bh(&mod->at_lock);

	return res;
}

static int k32pci_tty_at_chars_in_buffer(struct tty_struct *tty)
{
	int res;
	struct polygator_tty_device *ptd = tty->driver_data;
	struct k32_gsm_module_data *mod = (struct k32_gsm_module_data *)ptd->data;

	spin_lock_bh(&mod->at_lock);

	res = mod->at_xmit_count;

	spin_unlock_bh(&mod->at_lock);

	return res;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20)
static void k32pci_tty_at_set_termios(struct tty_struct *tty, struct ktermios *old_termios)
#else
static void k32pci_tty_at_set_termios(struct tty_struct *tty, struct termios *old_termios)
#endif
{
	speed_t baud;
	struct polygator_tty_device *ptd = tty->driver_data;
	struct k32_gsm_module_data *mod = (struct k32_gsm_module_data *)ptd->data;

	baud = tty_get_baud_rate(tty);

	spin_lock_bh(&mod->at_lock);

	switch (baud) {
		case 9600:
			mod->control.bits.com_spd = 3;
			break;
		default:
			mod->control.bits.com_spd = 2;
			break;
	}

	mod->set_control(mod->cbdata, mod->pos_on_board, mod->control.full);

	spin_unlock_bh(&mod->at_lock);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
	tty_encode_baud_rate(tty, baud, baud);
#endif
}

static void k32pci_tty_at_flush_buffer(struct tty_struct *tty)
{
	struct polygator_tty_device *ptd = tty->driver_data;
	struct k32_gsm_module_data *mod = (struct k32_gsm_module_data *)ptd->data;

	spin_lock_bh(&mod->at_lock);
	mod->at_xmit_count = mod->at_xmit_head = mod->at_xmit_tail = 0;
	spin_unlock_bh(&mod->at_lock);
	tty_wakeup(tty);
}

static void k32pci_tty_at_hangup(struct tty_struct *tty)
{
#ifdef TTY_PORT
	struct polygator_tty_device *ptd = tty->driver_data;
	struct k32_gsm_module_data *mod = (struct k32_gsm_module_data *)ptd->data;
	tty_port_hangup(&mod->at_port);
#endif
}
#ifdef TTY_PORT
static int k32pci_tty_at_port_carrier_raised(struct tty_port *port)
{
	return 1;
}

static void k32pci_tty_at_port_dtr_rts(struct tty_port *port, int onoff)
{
}

static int k32pci_tty_at_port_activate(struct tty_port *port, struct tty_struct *tty)
{
	struct k32_gsm_module_data *mod = container_of(port, struct k32_gsm_module_data, at_port);

	if (tty_port_alloc_xmit_buf(port) < 0) {
		return -ENOMEM;
	}

	mod->at_xmit_count = mod->at_xmit_head = mod->at_xmit_tail = 0;

	mod->at_poll_timer.function = k32pci_tty_at_poll;
	mod->at_poll_timer.data = (unsigned long)mod;
	mod->at_poll_timer.expires = jiffies + 1;
	add_timer(&mod->at_poll_timer);

	return 0;
}

static void k32pci_tty_at_port_shutdown(struct tty_port *port)
{
	struct k32_gsm_module_data *mod = container_of(port, struct k32_gsm_module_data, at_port);

	del_timer_sync(&mod->at_poll_timer);

	tty_port_free_xmit_buf(port);
}
#endif
static int __init k32pci_init(void)
{
	int rc;

	verbose("loading ...\n");

	// Register PCI driver
	if ((rc = pci_register_driver(&k32pci_driver)) < 0) {
		log(KERN_ERR, "can't register pci driver\n");
		goto k32pci_init_error;
	}

	verbose("loaded successfull\n");
	return 0;

k32pci_init_error:
	return rc;
}

static void __exit k32pci_exit(void)
{
	// Unregister PCI driver
	pci_unregister_driver(&k32pci_driver);

	verbose("stopped\n");
}

module_init(k32pci_init);
module_exit(k32pci_exit);

/******************************************************************************/
/* end of k32pci-base.c                                                       */
/******************************************************************************/
