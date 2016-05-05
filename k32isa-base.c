/******************************************************************************/
/* k32isa-base.c                                                              */
/******************************************************************************/

#include <linux/kobject.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#include <linux/module.h>
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
MODULE_DESCRIPTION("Polygator Linux module for K32 ISA boards");
MODULE_LICENSE("GPL");

static int rom = 0;
module_param(rom, int, 0);
MODULE_PARM_DESC(tty_major, "Print board's ROM");

#define verbose(_fmt, _args...) printk(KERN_INFO "[polygator-%s] " _fmt, THIS_MODULE->name, ## _args)
#define log(_level, _fmt, _args...) printk(_level "[polygator-%s] %s:%d - %s(): " _fmt, THIS_MODULE->name, "k32isa-base.c", __LINE__, __PRETTY_FUNCTION__, ## _args)
#define debug(_fmt, _args...) printk(KERN_DEBUG "[polygator-%s] %s:%d - %s(): " _fmt, THIS_MODULE->name, "k32isa-base.c", __LINE__, __PRETTY_FUNCTION__, ## _args)

/*! */
#define PG_ISA_VS_BASE			0x220
#define PG_ISA_VS_LENGTH		0x10

#define PG_ISA_ROM_BASE			0x223
#define PG_ISA_ROM_LENGTH		0x10

#define PG_ISA_ID_BASE			0x240
#define PG_ISA_RESET_BASE		0x241
#define PG_ISA_ID_LENGTH		0x8

#define PG_ISA_AT_BASE			0x600
#define PG_ISA_AT_LENGTH		0x20

#define PG_ISA_CTRL_BASE		0x620
#define PG_ISA_CTRL_LENGTH		0x20

#define PG_ISA_SIM_BASE			0x700
#define PG_ISA_SIM_LENGTH		0x20

#define PG_ISA_IMEI_BASE		0x720
#define PG_ISA_IMEI_LENGTH		0x20

#define PG_ISA_VD_BASE			0x1000
#define PG_ISA_VD_LENGTH		0x20
/*! */

static struct k32_board *k32isa_boards[4];

static struct resource * k32isa_id_ioport_reg = NULL;
static struct resource * k32isa_at_ioport_reg = NULL;
static struct resource * k32isa_ctrl_ioport_reg = NULL;
static struct resource * k32isa_vs_ioport_reg = NULL;
static struct resource * k32isa_sim_ioport_reg = NULL;
static struct resource * k32isa_imei_ioport_reg = NULL;
static struct resource * k32isa_vd_ioport_reg[4][2];

static int k32isa_tty_at_open(struct tty_struct *tty, struct file *filp);
static void k32isa_tty_at_close(struct tty_struct *tty, struct file *filp);
static int k32isa_tty_at_write(struct tty_struct *tty, const unsigned char *buf, int count);
static int k32isa_tty_at_write_room(struct tty_struct *tty);
static int k32isa_tty_at_chars_in_buffer(struct tty_struct *tty);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20)
static void k32isa_tty_at_set_termios(struct tty_struct *tty, struct ktermios *old_termios);
#else
static void k32isa_tty_at_set_termios(struct tty_struct *tty, struct termios *old_termios);
#endif
static void k32isa_tty_at_flush_buffer(struct tty_struct *tty);
static void k32isa_tty_at_hangup(struct tty_struct *tty);

static struct tty_operations k32isa_tty_at_ops = {
	.open = k32isa_tty_at_open,
	.close = k32isa_tty_at_close,
	.write = k32isa_tty_at_write,
	.write_room = k32isa_tty_at_write_room,
	.chars_in_buffer = k32isa_tty_at_chars_in_buffer,
	.set_termios = k32isa_tty_at_set_termios,
	.flush_buffer = k32isa_tty_at_flush_buffer,
	.hangup = k32isa_tty_at_hangup,
};
#ifdef TTY_PORT
static int k32isa_tty_at_port_carrier_raised(struct tty_port *port);
static void k32isa_tty_at_port_dtr_rts(struct tty_port *port, int onoff);
static int k32isa_tty_at_port_activate(struct tty_port *tport, struct tty_struct *tty);
static void k32isa_tty_at_port_shutdown(struct tty_port *port);

static const struct tty_port_operations k32isa_tty_at_port_ops = {
	.carrier_raised = k32isa_tty_at_port_carrier_raised,
	.dtr_rts = k32isa_tty_at_port_dtr_rts,
	.activate = k32isa_tty_at_port_activate,
	.shutdown = k32isa_tty_at_port_shutdown,
};
#endif
static void k32isa_vin_reset_0(uintptr_t cbdata)
{
	outb(0x02, PG_ISA_RESET_BASE + cbdata * 2);
	mdelay(10);
	outb(0x00, PG_ISA_RESET_BASE + cbdata * 2);
	mdelay(10);
	outb(0x02, PG_ISA_RESET_BASE + cbdata * 2);
	mdelay(10);
	outb(0x00, PG_ISA_RESET_BASE + cbdata * 2);
	mdelay(2);
}

static void k32isa_vin_reset_1(uintptr_t cbdata)
{
	outb(0x04, PG_ISA_RESET_BASE + cbdata * 2);
	mdelay(10);
	outb(0x00, PG_ISA_RESET_BASE + cbdata * 2);
	mdelay(10);
	outb(0x04, PG_ISA_RESET_BASE + cbdata * 2);
	mdelay(10);
	outb(0x00, PG_ISA_RESET_BASE + cbdata * 2);
	mdelay(2);
}

static void k32isa_vin_write_nwd_0(uintptr_t cbdata, u_int16_t value)
{
	outw(value, PG_ISA_VD_BASE + cbdata * 64 + 0 + 4);
}

static void k32isa_vin_write_nwd_1(uintptr_t cbdata, u_int16_t value)
{
	outw(value, PG_ISA_VD_BASE + cbdata * 64 + 32 + 4);
}

static void k32isa_vin_write_eom_0(uintptr_t cbdata, u_int16_t value)
{
	outw(value, PG_ISA_VD_BASE + cbdata * 64 + 0 + 6);
}

static void k32isa_vin_write_eom_1(uintptr_t cbdata, u_int16_t value)
{
	outw(value, PG_ISA_VD_BASE + cbdata * 64 + 32 + 6);
}

static u_int16_t k32isa_vin_read_nwd_0(uintptr_t cbdata)
{
	u_int16_t value = inw(PG_ISA_VD_BASE + cbdata * 64 + 0 + 4);
	return value;
}

static u_int16_t k32isa_vin_read_nwd_1(uintptr_t cbdata)
{
	u_int16_t value = inw(PG_ISA_VD_BASE + cbdata * 64 + 32 + 4);
	return value;
}

static u_int16_t k32isa_vin_read_eom_0(uintptr_t cbdata)
{
	u_int16_t value = inw(PG_ISA_VD_BASE + cbdata * 64 + 0 + 6);
	return value;
}

static u_int16_t k32isa_vin_read_eom_1(uintptr_t cbdata)
{
	u_int16_t value = inw(PG_ISA_VD_BASE + cbdata * 64 + 32 + 6);
	return value;
}

static size_t k32isa_vin_is_not_ready_0(uintptr_t cbdata)
{
	size_t st = (inb(PG_ISA_VS_BASE + cbdata * 4) >> 0) & 1;
	return st;
}

static size_t k32isa_vin_is_not_ready_1(uintptr_t cbdata)
{
	size_t st = (inb(PG_ISA_VS_BASE + cbdata * 4) >> 1) & 1;
	return st;
}

static u_int16_t k32isa_vin_read_dia_0(uintptr_t cbdata)
{
	return 0;
}

static u_int16_t k32isa_vin_read_dia_1(uintptr_t cbdata)
{
	return 0;
}

static void k32isa_gsm_mod_set_control(uintptr_t cbdata, size_t pos, u_int8_t reg)
{
	outb(reg, PG_ISA_CTRL_BASE + cbdata * 8 + pos);
}

static u_int8_t k32isa_gsm_mod_get_status(uintptr_t cbdata, size_t pos)
{
	return inb(PG_ISA_CTRL_BASE + cbdata * 8 + pos);
}

static void k32isa_gsm_mod_at_write(uintptr_t cbdata, size_t pos, u_int8_t reg)
{
	outb(reg, PG_ISA_AT_BASE + cbdata * 8 + pos);
}

static u_int8_t k32isa_gsm_mod_at_read(uintptr_t cbdata, size_t pos)
{
	return inb(PG_ISA_AT_BASE + cbdata * 8 + pos);
}

static void k32isa_gsm_mod_sim_write(uintptr_t cbdata, size_t pos, u_int8_t reg)
{
	outb(reg, PG_ISA_SIM_BASE + cbdata * 8 + pos);
}

static u_int8_t k32isa_gsm_mod_sim_read(uintptr_t cbdata, size_t pos)
{
	return inb(PG_ISA_SIM_BASE + cbdata * 8 + pos);
}

static void k32isa_gsm_mod_imei_write(uintptr_t cbdata, size_t pos, u_int8_t reg)
{
	outb(reg, PG_ISA_IMEI_BASE + cbdata * 8 + pos);
}

static u_int8_t k32isa_gsm_mod_imei_read(uintptr_t cbdata, size_t pos)
{
	return inb(PG_ISA_IMEI_BASE + cbdata * 8 + pos);
}

static u_int8_t k32isa_sim_read(void *data)
{
	struct k32_gsm_module_data *mod = (struct k32_gsm_module_data *)data;

	return mod->sim_read(mod->cbdata, mod->pos_on_board);
}

static void k32isa_sim_write(void *data, u_int8_t value)
{
	struct k32_gsm_module_data *mod = (struct k32_gsm_module_data *)data;

	mod->sim_write(mod->cbdata, mod->pos_on_board, value);
}

static int k32isa_sim_is_read_ready(void *data)
{
	union k32_gsm_mod_status_reg status;
	struct k32_gsm_module_data *mod = (struct k32_gsm_module_data *)data;

	status.full = mod->get_status(mod->cbdata, mod->pos_on_board);

	return status.bits.sim_rdy_rd;
}

static int k32isa_sim_is_write_ready(void *data)
{
	union k32_gsm_mod_status_reg status;
	struct k32_gsm_module_data *mod = (struct k32_gsm_module_data *)data;

	status.full = mod->get_status(mod->cbdata, mod->pos_on_board);

	return status.bits.sim_rdy_wr;
}

static int k32isa_sim_is_reset_request(void *data)
{
	union k32_gsm_mod_status_reg status;
	struct k32_gsm_module_data *mod = (struct k32_gsm_module_data *)data;

	status.full = mod->get_status(mod->cbdata, mod->pos_on_board);

	return status.bits.sim_rst_req;
}

static void k32isa_sim_set_speed(void *data, int speed)
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

static void k32isa_tty_at_poll(unsigned long addr)
{
	char buff[512];
	size_t len;
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

static int k32isa_board_open(struct inode *inode, struct file *filp)
{
	ssize_t res;
	size_t i,j;
	size_t len;

	struct k32_board *board;
	struct k32_board_private_data *private_data;
	struct k32_gsm_module_data *mod;
	union k32_gsm_mod_status_reg status;

	board = container_of(inode->i_cdev, struct k32_board, cdev);

	if (!(private_data = kmalloc(sizeof(struct k32_board_private_data), GFP_KERNEL))) {
		log(KERN_ERR, "can't get memory=%lu bytes\n", (unsigned long int)sizeof(struct k32_board_private_data));
		res = -ENOMEM;
		goto k32isa_open_error;
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
							board->tty_at_channels[i] ? dev_name(board->tty_at_channels[i]->device) : "unknown",
							board->simcard_channels[i] ? dev_name(board->simcard_channels[i]->device) : "unknown",
#else
							board->tty_at_channels[i] ? board->tty_at_channels[i]->device->class_id : "unknown",
							board->simcard_channels[i] ? board->simcard_channels[i]->device->class_id : "unknown",
#endif
							(unsigned long int)(i / 4),
							(unsigned long int)(i % 4),
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

k32isa_open_error:
	if (private_data) {
		kfree(private_data);
	}
	return res;
}

static int k32isa_board_release(struct inode *inode, struct file *filp)
{
	struct k32_board_private_data *private_data = filp->private_data;

	kfree(private_data);
	return 0;
}

static ssize_t k32isa_board_read(struct file *filp, char __user *buff, size_t count, loff_t *offp)
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
			goto k32isa_board_read_end;
		}
		*offp = filp->f_pos + len;
	}

k32isa_board_read_end:
	return res;
}

static ssize_t k32isa_board_write(struct file *filp, const char __user *buff, size_t count, loff_t *offp)
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
		goto k32isa_board_write_end;
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

k32isa_board_write_end:
	return res;
}

static struct file_operations k32isa_board_fops = {
	.owner   = THIS_MODULE,
	.open    = k32isa_board_open,
	.release = k32isa_board_release,
	.read    = k32isa_board_read,
	.write   = k32isa_board_write,
};

static int k32isa_tty_at_open(struct tty_struct *tty, struct file *filp)
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

		mod->at_poll_timer.function = k32isa_tty_at_poll;
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

static void k32isa_tty_at_close(struct tty_struct *tty, struct file *filp)
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

static int k32isa_tty_at_write(struct tty_struct *tty, const unsigned char *buf, int count)
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

	return res;
}

static int k32isa_tty_at_write_room(struct tty_struct *tty)
{
	int res;
	struct polygator_tty_device *ptd = tty->driver_data;
	struct k32_gsm_module_data *mod = (struct k32_gsm_module_data *)ptd->data;

	spin_lock_bh(&mod->at_lock);

	res = SERIAL_XMIT_SIZE - mod->at_xmit_count;

	spin_unlock_bh(&mod->at_lock);

	return res;
}

static int k32isa_tty_at_chars_in_buffer(struct tty_struct *tty)
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
static void k32isa_tty_at_set_termios(struct tty_struct *tty, struct ktermios *old_termios)
#else
static void k32isa_tty_at_set_termios(struct tty_struct *tty, struct termios *old_termios)
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

static void k32isa_tty_at_flush_buffer(struct tty_struct *tty)
{
	struct polygator_tty_device *ptd = tty->driver_data;
	struct k32_gsm_module_data *mod = (struct k32_gsm_module_data *)ptd->data;

	spin_lock_bh(&mod->at_lock);
	mod->at_xmit_count = mod->at_xmit_head = mod->at_xmit_tail = 0;
	spin_unlock_bh(&mod->at_lock);
	tty_wakeup(tty);
}

static void k32isa_tty_at_hangup(struct tty_struct *tty)
{
#ifdef TTY_PORT
	struct polygator_tty_device *ptd = tty->driver_data;
	struct k32_gsm_module_data *mod = (struct k32_gsm_module_data *)ptd->data;

	tty_port_hangup(&mod->at_port);
#endif
}
#ifdef TTY_PORT
static int k32isa_tty_at_port_carrier_raised(struct tty_port *port)
{
	return 1;
}

static void k32isa_tty_at_port_dtr_rts(struct tty_port *port, int onoff)
{
}

static int k32isa_tty_at_port_activate(struct tty_port *port, struct tty_struct *tty)
{
	struct k32_gsm_module_data *mod = container_of(port, struct k32_gsm_module_data, at_port);

	if (tty_port_alloc_xmit_buf(port) < 0) {
		return -ENOMEM;
	}

	mod->at_xmit_count = mod->at_xmit_head = mod->at_xmit_tail = 0;

	mod->at_poll_timer.function = k32isa_tty_at_poll;
	mod->at_poll_timer.data = (unsigned long)mod;
	mod->at_poll_timer.expires = jiffies + 1;
	add_timer(&mod->at_poll_timer);

	return 0;
}

static void k32isa_tty_at_port_shutdown(struct tty_port *port)
{
	struct k32_gsm_module_data *mod = container_of(port, struct k32_gsm_module_data, at_port);

	del_timer_sync(&mod->at_poll_timer);

	tty_port_free_xmit_buf(port);
}
#endif
static int __init k32isa_init(void)
{
	int rc;
	size_t i, j, k;
	struct k32_gsm_module_data *mod;
	resource_size_t start;
	resource_size_t end;
	resource_size_t length;
	char devname[VINETIC_DEVNAME_MAXLEN];

	verbose("loading ...\n");
	
	for (k = 0; k < 4; k++) {
		k32isa_boards[k] = NULL;
		for (j = 0; j < 2; j++) {
			k32isa_vd_ioport_reg[k][j] = NULL;
		}
	}
	
	// request region for Polygator K32 ISA board ID
	start = PG_ISA_ID_BASE;
	length = PG_ISA_ID_LENGTH;
	end = start + length - 1;
	if (!(k32isa_id_ioport_reg = request_region(start, length, "polygator_k32isa"))) {
		log(KERN_ERR, "can't request i/o port region for ID %04lx-%04lx\n", (unsigned long int)start, (unsigned long int)end);
		rc = -ENOMEM;
		goto k32isa_init_error;
	}
	// request region for Polygator K32 ISA board AT
	start = PG_ISA_AT_BASE;
	length = PG_ISA_AT_LENGTH;
	end = start + length - 1;
	if (!(k32isa_at_ioport_reg = request_region(start, length, "polygator_k32isa"))) {
		log(KERN_ERR, "can't request i/o port region for AT %04lx-%04lx\n", (unsigned long int)start, (unsigned long int)end);
		rc = -ENOMEM;
		goto k32isa_init_error;
	}
	// request region for Polygator K32 ISA board CTRL
	start = PG_ISA_CTRL_BASE;
	length = PG_ISA_CTRL_LENGTH;
	end = start + length - 1;
	if (!(k32isa_ctrl_ioport_reg = request_region(start, length, "polygator_k32isa"))) {
		log(KERN_ERR, "can't request i/o port region for CTRL %04lx-%04lx\n", (unsigned long int)start, (unsigned long int)end);
		rc = -ENOMEM;
		goto k32isa_init_error;
	}
	// request region for Polygator K32 ISA board VS
	start = PG_ISA_VS_BASE;
	length = PG_ISA_VS_LENGTH;
	end = start + length - 1;
	if (!(k32isa_vs_ioport_reg = request_region(start, length, "polygator_k32isa"))) {
		log(KERN_ERR, "can't request i/o port region for VS %04lx-%04lx\n", (unsigned long int)start, (unsigned long int)end);
		rc = -ENOMEM;
		goto k32isa_init_error;
	}
	// request region for Polygator K32 ISA board SIM
	start = PG_ISA_SIM_BASE;
	length = PG_ISA_SIM_LENGTH;
	end = start + length - 1;
	if (!(k32isa_sim_ioport_reg = request_region(start, length, "polygator_k32isa"))) {
		log(KERN_ERR, "can't request i/o port region for SIM %04lx-%04lx\n", (unsigned long int)start, (unsigned long int)end);
		rc = -ENOMEM;
		goto k32isa_init_error;
	}
	// request region for Polygator K32 ISA board IMEI
	start = PG_ISA_IMEI_BASE;
	length = PG_ISA_IMEI_LENGTH;
	end = start + length - 1;
	if (!(k32isa_imei_ioport_reg = request_region(start, length, "polygator_k32isa"))) {
		log(KERN_ERR, "can't request i/o port region for IMEI %04lx-%04lx\n", (unsigned long int)start, (unsigned long int)end);
		rc = -ENOMEM;
		goto k32isa_init_error;
	}
	// request region for Polygator K32 ISA board VINETIC
	for (k = 0; k < 4; k++) {
		for (j = 0; j < 2; j++) {
			start = PG_ISA_VD_BASE + k * 64 + j * 32;
			length = 0x20;
			end = start + length - 1;
			if (!(k32isa_vd_ioport_reg[k][j] = request_region(start, length, "polygator_k32isa"))) {
				log(KERN_ERR, "can't request i/o port region for VIN %04lx-%04lx\n", (unsigned long int)start, (unsigned long int)end);
				rc = -ENOMEM;
				goto k32isa_init_error;
			}
		}
	}

	// search Polygator K32 ISA boards
	// reset all ISA board
	for (k = 0; k < 4; k++) {
		outb(0xff, PG_ISA_RESET_BASE + k * 2);
	}
	mdelay(10);
	for (k = 0; k < 4; k++) {
		outb(0x00, PG_ISA_RESET_BASE + k * 2);
	}
	// search boards
	for (k = 0; k < 4; k++) {
		// alloc memory for board data
		if (!(k32isa_boards[k] = kmalloc(sizeof(struct k32_board), GFP_KERNEL))) {
			log(KERN_ERR, "can't get memory for struct k32_board\n");
			rc = -1;
			goto k32isa_init_error;
		}
		memset(k32isa_boards[k], 0, sizeof(struct k32_board));
		// get board type
		for (i = 0; i < 16; i++) {
			k32isa_boards[k]->type <<= 1;
			k32isa_boards[k]->type |= inb(PG_ISA_ID_BASE + k*2) & 0x01;
		}
		if (k32isa_boards[k]->type == 0xffff) {
			kfree(k32isa_boards[k]);
			k32isa_boards[k] = NULL;
			continue;
		}
		if (((k32isa_boards[k]->type & 0x00ff) != 0x0006) && ((k32isa_boards[k]->type & 0x00ff) != 0x0009)) {
			log(KERN_NOTICE, "K32 ISA board type=%04x unsupported\n", k32isa_boards[k]->type & 0x00ff);
			kfree(k32isa_boards[k]);
			k32isa_boards[k] = NULL;
			continue;
		}
		verbose("found K32 ISA board type=%04x\n", k32isa_boards[k]->type & 0x00ff);
		// set board number
		k32isa_boards[k]->position = k & 3;
		// read board rom
		memset(k32isa_boards[k]->rom, 0, 256);
		k32isa_boards[k]->romsize = inb(PG_ISA_ROM_BASE + k * 4);
		k32isa_boards[k]->romsize = inb(PG_ISA_ROM_BASE + k * 4);
		for (i = 0; i < k32isa_boards[k]->romsize; i++) {
			k32isa_boards[k]->rom[i] = inb(PG_ISA_ROM_BASE + k*4);
		}
		if (rom) {
			verbose("\"%.*s\"\n", (int)k32isa_boards[k]->romsize, k32isa_boards[k]->rom);
		}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
		snprintf(devname, POLYGATOR_BRDNAME_MAXLEN, "board-k32isa-%lu", (long unsigned int)k);
#else
		snprintf(devname, POLYGATOR_BRDNAME_MAXLEN, "bi%lu", (long unsigned int)k);
#endif
		if (!(k32isa_boards[k]->pg_board =  polygator_board_register(THIS_MODULE, devname, &k32isa_boards[k]->cdev, &k32isa_board_fops))) {
			rc = -1;
			goto k32isa_init_error;
		}

		for (j = 0; j < 2; j++) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
			snprintf(devname, VINETIC_DEVNAME_MAXLEN, "board-k32isa-%lu-vin%lu", (long unsigned int)k, (unsigned long int)j);
#else
			snprintf(devname, VINETIC_DEVNAME_MAXLEN, "vi%lu%lu", (long unsigned int)k, (unsigned long int)j);
#endif
			if (!(k32isa_boards[k]->vinetics[j] = vinetic_device_register(THIS_MODULE, devname, k,
														(j)?(k32isa_vin_reset_1):(k32isa_vin_reset_0),
														(j)?(k32isa_vin_is_not_ready_1):(k32isa_vin_is_not_ready_0),
														(j)?(k32isa_vin_write_nwd_1):(k32isa_vin_write_nwd_0),
														(j)?(k32isa_vin_write_eom_1):(k32isa_vin_write_eom_0),
														(j)?(k32isa_vin_read_nwd_1):(k32isa_vin_read_nwd_0),
														(j)?(k32isa_vin_read_eom_1):(k32isa_vin_read_eom_0),
														(j)?(k32isa_vin_read_dia_1):(k32isa_vin_read_dia_0)))) {
				rc = -1;
				goto k32isa_init_error;
			}
			for (i = 0; i < 4; i++) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
				snprintf(devname, VINETIC_DEVNAME_MAXLEN, "board-k32isa-%lu-vin%lu-rtp%lu", (long unsigned int)k, (unsigned long int)j, (unsigned long int)i);
#else
				snprintf(devname, VINETIC_DEVNAME_MAXLEN, "ri%lu%lu%lu", (long unsigned int)k, (unsigned long int)j, (unsigned long int)i);
#endif
				if (!vinetic_rtp_channel_register(THIS_MODULE, devname, k32isa_boards[k]->vinetics[j], i)) {
					rc = -1;
					goto k32isa_init_error;
				}
			}
		}
		// set GSM module data
		for (i = 0; i < 8; i++) {
			if (!(mod = kmalloc(sizeof(struct k32_gsm_module_data), GFP_KERNEL))) {
				log(KERN_ERR, "can't get memory for struct k32_gsm_module_data\n");
				rc = -1;
				goto k32isa_init_error;
			}
			memset(mod, 0, sizeof(struct k32_gsm_module_data));
			// select GSM module type
			if ((k32isa_boards[k]->type & 0x00ff) == 0x0009) {
				if (k32isa_boards[k]->rom[8] == '*') {
					if (k32isa_boards[k]->rom[i] == 'M') {
						mod->type = POLYGATOR_MODULE_TYPE_M10;
					} else if (k32isa_boards[k]->rom[i] == '9') {
						mod->type = POLYGATOR_MODULE_TYPE_SIM900;
					} else if (k32isa_boards[k]->rom[i] == 'S') {
						mod->type = POLYGATOR_MODULE_TYPE_SIM300;
					} else if (k32isa_boards[k]->rom[i] == 'G') {
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
			} else if (mod->type == POLYGATOR_MODULE_TYPE_SIM5215) {
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
			mod->cbdata = k;
			mod->set_control = k32isa_gsm_mod_set_control;
			mod->get_status = k32isa_gsm_mod_get_status;
			mod->at_write = k32isa_gsm_mod_at_write;
			mod->at_read = k32isa_gsm_mod_at_read;
			mod->sim_write = k32isa_gsm_mod_sim_write;
			mod->sim_read = k32isa_gsm_mod_sim_read;
			mod->imei_write = k32isa_gsm_mod_imei_write;
			mod->imei_read = k32isa_gsm_mod_imei_read;

// 			mod->set_control(mod->cbdata, mod->pos_on_board, mod->control.full);
			init_timer(&mod->at_poll_timer);

			spin_lock_init(&mod->at_lock);
#ifdef TTY_PORT
			tty_port_init(&mod->at_port);
			mod->at_port.ops = &k32isa_tty_at_port_ops;
			mod->at_port.close_delay = 0;
			mod->at_port.closing_wait = ASYNC_CLOSING_WAIT_NONE;
#endif
			k32isa_boards[k]->gsm_modules[i] = mod;
		}

		// register polygator tty at device
		for (i = 0; i < 8; i++) {
			if ((mod = k32isa_boards[k]->gsm_modules[i])) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0)
				if (!(k32isa_boards[k]->tty_at_channels[i] = polygator_tty_device_register(THIS_MODULE, mod, &mod->at_port, &k32isa_tty_at_ops))) {
#else
				if (!(k32isa_boards[k]->tty_at_channels[i] = polygator_tty_device_register(THIS_MODULE, mod, &k32isa_tty_at_ops))) {
#endif
					log(KERN_ERR, "can't register polygator tty device\n");
					rc = -1;
					goto k32isa_init_error;
				}
			}
		}

		// register polygator simcard device
		for (i = 0; i < 8; i++) {
			if (k32isa_boards[k]->gsm_modules[i]) {
				if (!(k32isa_boards[k]->simcard_channels[i] = simcard_device_register(THIS_MODULE,
																						k32isa_boards[k]->gsm_modules[i],
																						k32isa_sim_read,
																						k32isa_sim_write,
																						k32isa_sim_is_read_ready,
																						k32isa_sim_is_write_ready,
																						k32isa_sim_is_reset_request,
																						k32isa_sim_set_speed))) {
					log(KERN_ERR, "can't register polygator simcard device\n");
					rc = -1;
					goto k32isa_init_error;
				}
			}
		}
	}

	verbose("loaded successfull\n");
	return 0;

k32isa_init_error:
	for (k = 0; k < 4; k++) {
		if (k32isa_boards[k]) {
			for (i = 0; i < 8; i++) {
				if (k32isa_boards[k]->simcard_channels[i]) {
					simcard_device_unregister(k32isa_boards[k]->simcard_channels[i]);
				}
				if (k32isa_boards[k]->tty_at_channels[i]) {
					polygator_tty_device_unregister(k32isa_boards[k]->tty_at_channels[i]);
				}
				if (k32isa_boards[k]->gsm_modules[i]) {
					del_timer_sync(&k32isa_boards[k]->gsm_modules[i]->at_poll_timer);
					kfree(k32isa_boards[k]->gsm_modules[i]);
				}
			}
			for (j = 0; j < 2; j++) {
				if (k32isa_boards[k]->vinetics[j]) {
					for (i = 0; i < 4; i++) {
						if (k32isa_boards[k]->vinetics[j]->rtp_channels[i]) {
							vinetic_rtp_channel_unregister(k32isa_boards[k]->vinetics[j]->rtp_channels[i]);
						}
					}
					vinetic_device_unregister(k32isa_boards[k]->vinetics[j]);
				}
			}
			if (k32isa_boards[k]->pg_board) {
				polygator_board_unregister(k32isa_boards[k]->pg_board);
			}
			kfree(k32isa_boards[k]);
		}
	}

	if (k32isa_id_ioport_reg) {
		release_region(PG_ISA_ID_BASE, PG_ISA_ID_LENGTH);
	}
	if (k32isa_at_ioport_reg) {
		release_region(PG_ISA_AT_BASE, PG_ISA_AT_LENGTH);
	}
	if (k32isa_ctrl_ioport_reg) {
		release_region(PG_ISA_CTRL_BASE, PG_ISA_CTRL_LENGTH);
	}
	if (k32isa_vs_ioport_reg) {
		release_region(PG_ISA_VS_BASE, PG_ISA_VS_LENGTH);
	}
	if (k32isa_sim_ioport_reg) {
		release_region(PG_ISA_SIM_BASE, PG_ISA_SIM_LENGTH);
	}
	if (k32isa_imei_ioport_reg) {
		release_region(PG_ISA_IMEI_BASE, PG_ISA_IMEI_LENGTH);
	}
	for (k = 0; k < 4; k++) {
		for (j = 0; j < 2; j++) {
			if (k32isa_vd_ioport_reg[k][j]) {
				release_region(PG_ISA_VD_BASE + k * 64 + j * 32, PG_ISA_VD_LENGTH);
			}
		}
	}

	return rc;
}

static void __exit k32isa_exit(void)
{

	size_t i, j, k;

	for (k = 0; k < 4; k++) {
		if (k32isa_boards[k]) {
			for (i = 0; i < 8; i++) {
				if (k32isa_boards[k]->simcard_channels[i]) {
					simcard_device_unregister(k32isa_boards[k]->simcard_channels[i]);
				}
				if (k32isa_boards[k]->tty_at_channels[i]) {
					polygator_tty_device_unregister(k32isa_boards[k]->tty_at_channels[i]);
				}
				if (k32isa_boards[k]->gsm_modules[i]) {
					del_timer_sync(&k32isa_boards[k]->gsm_modules[i]->at_poll_timer);
					kfree(k32isa_boards[k]->gsm_modules[i]);
				}
			}
			for (j = 0; j < 2; j++) {
				for (i = 0; i < 4; i++) {
					vinetic_rtp_channel_unregister(k32isa_boards[k]->vinetics[j]->rtp_channels[i]);
				}
				vinetic_device_unregister(k32isa_boards[k]->vinetics[j]);
			}
			polygator_board_unregister(k32isa_boards[k]->pg_board);
			kfree(k32isa_boards[k]);
		}
	}

	// release I/O port regions
	release_region(PG_ISA_ID_BASE, PG_ISA_ID_LENGTH);
	release_region(PG_ISA_AT_BASE, PG_ISA_AT_LENGTH);
	release_region(PG_ISA_CTRL_BASE, PG_ISA_CTRL_LENGTH);
	release_region(PG_ISA_VS_BASE, PG_ISA_VS_LENGTH);
	release_region(PG_ISA_SIM_BASE, PG_ISA_SIM_LENGTH);
	release_region(PG_ISA_IMEI_BASE, PG_ISA_IMEI_LENGTH);
	for (k = 0; k < 4; k++) {
		for (j = 0; j < 2; j++) {
			release_region(PG_ISA_VD_BASE + k*64 + j*32, PG_ISA_VD_LENGTH);
		}
	}

	verbose("stopped\n");
}

module_init(k32isa_init);
module_exit(k32isa_exit);

/******************************************************************************/
/* end of k32isa-base.c                                                       */
/******************************************************************************/
