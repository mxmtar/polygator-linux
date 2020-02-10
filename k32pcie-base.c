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
MODULE_DESCRIPTION("Polygator Linux module for K32 PCIE boards");
MODULE_LICENSE("GPL");

static int rom = 0;
module_param(rom, int, 0);
MODULE_PARM_DESC(rom, "Print board's ROM");

#define verbose(_fmt, _args...) printk(KERN_INFO "[polygator-%s] " _fmt, THIS_MODULE->name, ## _args)
#define log(_level, _fmt, _args...) printk(_level "[polygator-%s] %s:%d - %s(): " _fmt, THIS_MODULE->name, "k32pcie-base.c", __LINE__, __PRETTY_FUNCTION__, ## _args)
#define debug(_fmt, _args...) printk(KERN_DEBUG "[polygator-%s] %s:%d - %s(): " _fmt, THIS_MODULE->name, "k32pcie-base.c", __LINE__, __PRETTY_FUNCTION__, ## _args)

static int k32pcie_tty_at_open(struct tty_struct *tty, struct file *filp);
static void k32pcie_tty_at_close(struct tty_struct *tty, struct file *filp);
static int k32pcie_tty_at_write(struct tty_struct *tty, const unsigned char *buf, int count);
static int k32pcie_tty_at_write_room(struct tty_struct *tty);
static int k32pcie_tty_at_chars_in_buffer(struct tty_struct *tty);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20)
static void k32pcie_tty_at_set_termios(struct tty_struct *tty, struct ktermios *old_termios);
#else
static void k32pcie_tty_at_set_termios(struct tty_struct *tty, struct termios *old_termios);
#endif
static void k32pcie_tty_at_flush_buffer(struct tty_struct *tty);
static void k32pcie_tty_at_hangup(struct tty_struct *tty);

static struct tty_operations k32pcie_tty_at_ops = {
	.open = k32pcie_tty_at_open,
	.close = k32pcie_tty_at_close,
	.write = k32pcie_tty_at_write,
	.write_room = k32pcie_tty_at_write_room,
	.chars_in_buffer = k32pcie_tty_at_chars_in_buffer,
	.set_termios = k32pcie_tty_at_set_termios,
	.flush_buffer = k32pcie_tty_at_flush_buffer,
	.hangup = k32pcie_tty_at_hangup,
};
#ifdef TTY_PORT
static int k32pcie_tty_at_port_carrier_raised(struct tty_port *port);
static void k32pcie_tty_at_port_dtr_rts(struct tty_port *port, int onoff);
static int k32pcie_tty_at_port_activate(struct tty_port *tport, struct tty_struct *tty);
static void k32pcie_tty_at_port_shutdown(struct tty_port *port);

static const struct tty_port_operations k32pcie_tty_at_port_ops = {
	.carrier_raised = k32pcie_tty_at_port_carrier_raised,
	.dtr_rts = k32pcie_tty_at_port_dtr_rts,
	.activate = k32pcie_tty_at_port_activate,
	.shutdown = k32pcie_tty_at_port_shutdown,
};
#endif
static struct pci_device_id k32pcie_board_id_table[] = {
	{ PCI_DEVICE(0xDEAD, 0xBEDE), .driver_data = 1, },
	{ 0, },
};
MODULE_DEVICE_TABLE(pci, k32pcie_board_id_table);

static void k32pcie_vin_reset_0(uintptr_t cbdata)
{
	void __iomem *addr = (void __iomem *)cbdata;
	iowrite8(0, addr + 0x0000 + 0x11c00);
	mdelay(10);
	iowrite8(1, addr + 0x0000 + 0x11c00);
}

static void k32pcie_vin_reset_1(uintptr_t cbdata)
{
	void __iomem *addr = (void __iomem *)cbdata;
	iowrite8(0, addr + 0x2000 + 0x11c00);
	mdelay(10);
	iowrite8(1, addr + 0x2000 + 0x11c00);
}

static void k32pcie_vin_write_nwd_0(uintptr_t cbdata, u_int16_t value)
{
	void __iomem *addr = (void __iomem *)cbdata;
	iowrite16(value, addr + 0x11000 + 0x0000  + 0x080);
}

static void k32pcie_vin_write_nwd_1(uintptr_t cbdata, u_int16_t value)
{
	void __iomem *addr = (void __iomem *)cbdata;
	iowrite16(value, addr + 0x11000 + 0x2000 + 0x080);
}

static void k32pcie_vin_write_eom_0(uintptr_t cbdata, u_int16_t value)
{
	void __iomem *addr = (void __iomem *)cbdata;
	iowrite16(value, addr + 0x11000 + 0x0000 + 0x0c0);
}

static void k32pcie_vin_write_eom_1(uintptr_t cbdata, u_int16_t value)
{
	void __iomem *addr = (void __iomem *)cbdata;
	iowrite16(value, addr + 0x11000 + 0x2000 + 0x0c0);
}

static u_int16_t k32pcie_vin_read_nwd_0(uintptr_t cbdata)
{
	u_int16_t value;
	void __iomem *addr = (void __iomem *)cbdata;
	iowrite16((cbdata + 0x11000 + 0x0000 + 0x080) & 0xffff, addr + 0x0000 + 0x11700);
	udelay(1);
	value = ioread16(addr + 0x0000 + 0x11000);
	return value;
}

static u_int16_t k32pcie_vin_read_nwd_1(uintptr_t cbdata)
{
	u_int16_t value;
	void __iomem *addr = (void __iomem *)cbdata;
	iowrite16((cbdata + 0x11000 + 0x2000 + 0x080) & 0xffff, addr + 0x2000 + 0x11700);
	udelay(1);
	value = ioread16(addr + 0x2000 + 0x11000);
	return value;
}

static u_int16_t k32pcie_vin_read_eom_0(uintptr_t cbdata)
{
	u_int16_t value;
	void __iomem *addr = (void __iomem *)cbdata;
	iowrite16((cbdata + 0x11000 + 0x0000 + 0x0c0) & 0xffff, addr + 0x0000 + 0x11700);
	udelay(1);
	value = ioread16(addr + 0x0000 + 0x11000);
	return value;
}

static u_int16_t k32pcie_vin_read_eom_1(uintptr_t cbdata)
{
	u_int16_t value;
	void __iomem *addr = (void __iomem *)cbdata;
	iowrite16((cbdata + 0x11000 + 0x2000 + 0x0c0) & 0xffff, addr + 0x2000 + 0x11700);
	udelay(1);
	value = ioread16(addr + 0x2000 + 0x11000);
	return value;
}

static size_t k32pcie_vin_is_not_ready_0(uintptr_t cbdata)
{
	void __iomem *addr = (void __iomem *)cbdata;
	size_t st;

	st = ioread16(addr + 0x0000 + 0x11200) & 1;

	return st;
}

static size_t k32pcie_vin_is_not_ready_1(uintptr_t cbdata)
{
	void __iomem *addr = (void __iomem *)cbdata;
	size_t st;

	st = ioread16(addr + 0x2000 + 0x11200) & 1;

	return st;
}

static u_int16_t k32pcie_vin_read_dia_0(uintptr_t cbdata)
{
	u_int16_t value;
	void __iomem *addr = (void __iomem *)cbdata;
	iowrite16((cbdata + 0x11000 + 0x0000 + 0x300) & 0xffff, addr + 0x0000 + 0x11700);
	udelay(1);
	value = ioread16(addr + 0x0000 + 0x11000);
	return value;
}

static u_int16_t k32pcie_vin_read_dia_1(uintptr_t cbdata)
{
	u_int16_t value;
	void __iomem *addr = (void __iomem *)cbdata;
	iowrite16((cbdata + 0x11000 + 0x2000 + 0x300) & 0xffff, addr + 0x2000 + 0x11700);
	udelay(1);
	value = ioread16(addr + 0x2000 + 0x11000);
	return value;
}

static void k32pcie_gsm_mod_set_control(uintptr_t cbdata, size_t pos, u_int8_t reg)
{
	void __iomem *addr = (void __iomem *)cbdata;
	iowrite8(reg, addr);
}

static u_int8_t k32pcie_gsm_mod_get_status(uintptr_t cbdata, size_t pos)
{
	u_int8_t reg;
	void __iomem *addr = (void __iomem *)cbdata;
	reg = ioread8(addr);
	return reg;
}

static void k32pcie_gsm_mod_at_write(uintptr_t cbdata, size_t pos, u_int8_t reg)
{
	void __iomem *addr = (void __iomem *)cbdata;

	iowrite8(reg, addr + 0x100);
	iowrite8(0, addr + 0x3c0);
	iowrite8(1, addr + 0x3c0);
	iowrite8(0, addr + 0x3c0);
}

static u_int8_t k32pcie_gsm_mod_at_read(uintptr_t cbdata, size_t pos)
{
	u_int8_t reg;
	void __iomem *addr = (void __iomem *)cbdata;

	reg = ioread8(addr + 0x100);
	iowrite8(0, addr + 0x2c0);
	iowrite8(1, addr + 0x2c0);
	iowrite8(0, addr + 0x2c0);
	return reg;
}

static u_int16_t k32pcie_gsm_mod_at_read16(uintptr_t cbdata, size_t pos)
{
	u_int16_t reg;
	void __iomem *addr = (void __iomem *)cbdata;

	reg = ioread16(addr + 0x100);
	if ((reg & 0x0200) == 0) {
		iowrite8(0, addr + 0x2c0);
		iowrite8(1, addr + 0x2c0);
		iowrite8(0, addr + 0x2c0);
	}
	return reg;
}

static void k32pcie_gsm_mod_sim_write(uintptr_t cbdata, size_t pos, u_int8_t reg)
{
	void __iomem *addr = (void __iomem *)cbdata;

	iowrite8(reg, addr + 0x200);
	iowrite8(0, addr + 0x380);
	iowrite8(1, addr + 0x380);
	iowrite8(0, addr + 0x380);
}

static u_int8_t k32pcie_gsm_mod_sim_read(uintptr_t cbdata, size_t pos)
{
	u_int8_t reg;
	void __iomem *addr = (void __iomem *)cbdata;

	reg = ioread8(addr + 0x200);
	iowrite8(0, addr + 0x280);
	iowrite8(1, addr + 0x280);
	iowrite8(0, addr + 0x280);
	return reg;
}

static void k32pcie_gsm_mod_sim_do_after_reset(uintptr_t cbdata, size_t pos)
{
	void __iomem *addr = (void __iomem *)cbdata;

	iowrite8(0x10, addr + 0x340);
	iowrite8(0x00, addr + 0x340);
	iowrite8(0x10, addr + 0x340);
}

static void k32pcie_gsm_mod_imei_write(uintptr_t cbdata, size_t pos, u_int8_t reg)
{
// 	iowrite8(reg, cbdata + pos * 4 + );
}

static u_int8_t k32pcie_gsm_mod_imei_read(uintptr_t cbdata, size_t pos)
{
	return /*ioread8(cbdata + pos * 4 + )*/0;
}

static u_int8_t k32pcie_sim_read(void *data)
{
	struct k32_gsm_module_data *mod = (struct k32_gsm_module_data *)data;

	return mod->sim_read(mod->cbdata, mod->pos_on_board);
}

static void k32pcie_sim_write(void *data, u_int8_t value)
{
	struct k32_gsm_module_data *mod = (struct k32_gsm_module_data *)data;

	mod->sim_write(mod->cbdata, mod->pos_on_board, value);
}

static int k32pcie_sim_is_read_ready(void *data)
{
	union k32_gsm_mod_status_reg status;
	struct k32_gsm_module_data *mod = (struct k32_gsm_module_data *)data;

	status.full = mod->get_status(mod->cbdata, mod->pos_on_board);

	return status.bits_e.sim_rdy_rd;
}

static int k32pcie_sim_is_write_ready(void *data)
{
	union k32_gsm_mod_status_reg status;
	struct k32_gsm_module_data *mod = (struct k32_gsm_module_data *)data;

	status.full = mod->get_status(mod->cbdata, mod->pos_on_board);

	return status.bits_e.sim_rdy_wr;
}

static int k32pcie_sim_is_reset_request(void *data)
{
	union k32_gsm_mod_status_reg status;
	struct k32_gsm_module_data *mod = (struct k32_gsm_module_data *)data;

	status.full = mod->get_status(mod->cbdata, mod->pos_on_board);

	return status.bits_e.sim_rst_req;
}

static void k32pcie_sim_set_speed(void *data, int speed)
{
	struct k32_gsm_module_data *mod = (struct k32_gsm_module_data *)data;

	switch (speed) {
		case 0x94:
		case 57600:
			mod->control.bits_e.sim_spd = 1;
			break;
		case 0x95:
		case 115200:
			mod->control.bits_e.sim_spd = 2;
			break;
		case 0x96:
		case 230400:
			mod->control.bits_e.sim_spd = 3;
			break;
		case 0x11:
		default: // 9600
			mod->control.bits_e.sim_spd = 0;
			break;
	}

	mod->set_control(mod->cbdata, mod->pos_on_board, mod->control.full);
}

static void k32pcie_sim_do_after_reset(void *data)
{
	struct k32_gsm_module_data *mod = (struct k32_gsm_module_data *)data;

	mod->sim_do_after_reset(mod->cbdata, mod->pos_on_board);
}

static void k32pcie_tty_at_poll(struct timer_list *timer)
{
	unsigned char buff[512];
	size_t len;
	size_t xmit_write_room;
	u_int16_t rd16;
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,9,0)
	struct tty_struct *tty;
#endif
	union k32_gsm_mod_status_reg status;
	struct k32_gsm_module_data *mod = container_of(timer, struct k32_gsm_module_data, at_poll_timer);

	len = 0;

	// read received data
	while (len < sizeof(buff)) {
#if 0
		// read status register
		status.full = mod->get_status(mod->cbdata, mod->pos_on_board);
		// select port
		if (mod->at_port_select) {
			// auxilary
#if 0
			if (status.bits_e.imei_rdy_rd) {
				break;
			}
			// put char to receiving buffer
			buff[len++] = mod->imei_read(mod->cbdata, mod->pos_on_board);
#endif
		} else {
			// main
			if (status.bits_e.at_rdy_rd) {
				break;
			}
			// put char to receiving buffer
			buff[len++] = mod->at_read(mod->cbdata, mod->pos_on_board);
		}
#else
		if (mod->at_port_select) {
			// auxilary
			break;
		} else {
			// main
			rd16 = mod->at_read16(mod->cbdata, mod->pos_on_board);
			if (rd16 & 0x0200) {
				break;
			}
			// put char to receiving buffer
			buff[len++] = rd16 & 0xff;
		}
#endif
	}

	spin_lock(&mod->at_lock);

	while (mod->at_xmit_count) {
		// read status register
		status.full = mod->get_status(mod->cbdata, mod->pos_on_board);
		// select port
		if (mod->at_port_select) {
			// auxilary
#if 0
			// check for transmitter is ready
			if (status.bits_e.imei_rdy_wr) {
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
#else
			break;
#endif
		} else {
			// main
			// check for transmitter is ready
			if (status.bits_e.at_rdy_wr) {
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
					xmit_write_room = 2048;
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
// 		verbose("%lu\n", (long unsigned int)len);
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

static int k32pcie_board_open(struct inode *inode, struct file *filp)
{
	ssize_t res;
	size_t i, j;
	size_t len;

	struct k32_board *board;
	struct k32_board_private_data *private_data;
	union k32_gsm_mod_status_reg status;
	struct k32_gsm_module_data *mod;

	board = container_of(inode->i_cdev, struct k32_board, cdev);

	if (!(private_data = kmalloc(sizeof(struct k32_board_private_data), GFP_KERNEL))) {
		log(KERN_ERR, "can't get memory=%lu bytes\n", (unsigned long int)sizeof(struct k32_board_private_data));
		res = -ENOMEM;
		goto k32pcie_board_open_error;
	}
	private_data->board = board;

	len = 0;
	// type
	len += sprintf(private_data->buff + len, "TYPE=%u\r\n", board->type & 0x00ff);
	// position
	len += sprintf(private_data->buff + len, "POSITION=%u\r\n", board->position);
	// gsm
	for (i = 0; i < 8; ++i) {
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
							status.bits_e.vio);
		}
	}
	// vinetic
	for (i = 0; i < 2; ++i) {
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
			for (j = 0; j < 4; ++j) {
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

k32pcie_board_open_error:
	if (private_data) {
		kfree(private_data);
	}
	return res;
}

static int k32pcie_board_release(struct inode *inode, struct file *filp)
{
	struct k32_board_private_data *private_data = filp->private_data;

	kfree(private_data);
	return 0;
}

static ssize_t k32pcie_board_read(struct file *filp, char __user *buff, size_t count, loff_t *offp)
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
			goto k32pcie_board_read_end;
		}
		*offp = filp->f_pos + len;
	}

k32pcie_board_read_end:
	return res;
}

static ssize_t k32pcie_board_write(struct file *filp, const char __user *buff, size_t count, loff_t *offp)
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
		goto k32pcie_board_write_end;
	}

	if (sscanf(cmd, "GSM%u PWR=%u", &chan, &value) == 2) {
		if ((chan >= 0) && (chan <= 7) && (private_data->board->gsm_modules[chan])) {
			mod = private_data->board->gsm_modules[chan];
			mod->control.bits_e.pwr = !value;
			mod->set_control(mod->cbdata, mod->pos_on_board, mod->control.full);
			res = len;
		} else {
			res = - ENODEV;
		}
	} else if (sscanf(cmd, "GSM%u KEY=%u", &chan, &value) == 2) {
		if ((chan >= 0) && (chan <= 7) && (private_data->board->gsm_modules[chan])) {
			mod = private_data->board->gsm_modules[chan];
			mod->control.bits_e.key = !value;
			mod->set_control(mod->cbdata, mod->pos_on_board, mod->control.full);
			res = len;
		} else {
			res = -ENODEV;
		}
	} else if (sscanf(cmd, "GSM%u BAUDRATE=%u", &chan, &value) == 2) {
		if ((chan >= 0) && (chan <= 7) && (private_data->board->gsm_modules[chan])) {
			mod = private_data->board->gsm_modules[chan];
			switch (value) {
				case 9600:
					mod->control.bits_e.com_spd = 0;
					break;
				default:
				case 115200:
					mod->control.bits_e.com_spd = 3;
					break;
			}
			mod->set_control(mod->cbdata, mod->pos_on_board, mod->control.full);
			res = len;
		} else {
			res = -ENODEV;
		}
	} else if (sscanf(cmd, "GSM%u COM=%u", &chan, &value) == 2) {
		if ((chan >= 0) && (chan <= 7) && (private_data->board->gsm_modules[chan])) {
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
	} else if (sscanf(cmd, "GSM%u TESTPOINT", &chan) == 1) {
		if ((chan >= 0) && (chan <= 7)) {
			if (chan < 4) {
				iowrite8(chan % 4, private_data->board->iomem_base + (private_data->board->position * 0x4000) + 0x0000 + 0x11400);
			} else {
				iowrite8(chan % 4, private_data->board->iomem_base + (private_data->board->position * 0x4000) + 0x2000 + 0x11400);
			}
			res = len;
		} else {
			res = -ENODEV;
		}
	} else if (sscanf(cmd, "SIMBANK MODE=%u", &value) == 1) {
		iowrite8(value, private_data->board->iomem_base + (private_data->board->position * 0x4000) + 0x0000 + 0x11900);
		iowrite8(value, private_data->board->iomem_base + (private_data->board->position * 0x4000) + 0x2000 + 0x11900);
		res = len;
	} else {
		res = -ENOMSG;
	}

k32pcie_board_write_end:
	return res;
}

static struct file_operations k32pcie_board_fops = {
	.owner   = THIS_MODULE,
	.open    = k32pcie_board_open,
	.release = k32pcie_board_release,
	.read    = k32pcie_board_read,
	.write   = k32pcie_board_write,
};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0)
static int k32pcie_board_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
#else
static int __devinit k32pcie_board_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
#endif
{
	int rc = -1;
	size_t i, j;
	char *cp, *dp;
	char devname[POLYGATOR_BRDNAME_MAXLEN];
	struct k32_gsm_module_data *mod;
	struct k32_board *board = NULL;

	// alloc memory for board data
	if (!(board = kmalloc(sizeof(struct k32_board), GFP_KERNEL))) {
		log(KERN_ERR, "can't get memory for struct k32_board\n");
		goto k32pcie_board_probe_error;
	}
	memset(board, 0, sizeof(struct k32_board));

	rc = pci_enable_device(pdev);
	if (rc) {
		dev_err(&pdev->dev, "can't enable pci device\n");
		goto k32pcie_board_probe_error;
	}

	rc = pci_request_region(pdev, 0, "k32pcie");
	if (rc) {
		dev_err(&pdev->dev, "can't request I/O region\n");
		goto k32pcie_board_probe_error;
	}
	board->iomem_req = 1;
	if (!(board->iomem_base = pci_iomap(pdev, 0, pci_resource_end(pdev, 0) - pci_resource_start(pdev, 0) + 1))) {
		dev_err(&pdev->dev, "can't request i/o memory region\n");
		rc = -ENOMEM;
		goto k32pcie_board_probe_error;
	}
	for (i = 0; i < 4; ++i) {
		board->type = ioread16(board->iomem_base + (i * 0x4000) + 0x0000 + 0x11e00);
		if (board->type != 0xffff) {
			board->position = i;
			break;
		}
	}
	verbose("found PCI-E board type=%04x\n", board->type & 0x00ff);
	for (i = 1; i < 256; ++i) {
		iowrite8(i, board->iomem_base + (board->position * 0x4000) + 0x0000 + 0x11d00);
		board->rom[i - 1] = ioread8(board->iomem_base + (board->position * 0x4000) + 0x0000 + 0x11d00);
	}
 	if (rom) {
		verbose("\"%.*s\"\n", (int)255, board->rom);
 	}

	if (((board->type & 0x00ff) != 0x0002) && ((board->type & 0x00ff) != 0x0004) &&
			((board->type & 0x00ff) != 0x0005) && ((board->type & 0x00ff) != 0x0006) &&
			((board->type & 0x00ff) != 0x0007)) {
		log(KERN_ERR, "PCI-E board type=%04x unsupported\n", board->type & 0x00ff);
		rc = -1;
		goto k32pcie_board_probe_error;
	}
	// reset all internal counters
	iowrite8(0, board->iomem_base + (board->position * 0x4000) + 0x0000 + 0x11f00);
	iowrite8(0, board->iomem_base + (board->position * 0x4000) + 0x2000 + 0x11f00);
	mdelay(10);
	iowrite8(1, board->iomem_base + (board->position * 0x4000) + 0x0000 + 0x11f00);
	iowrite8(1, board->iomem_base + (board->position * 0x4000) + 0x2000 + 0x11f00);
	// local oscilator
	iowrite8(0, board->iomem_base + (board->position * 0x4000) + 0x0000 + 0x11a00);
	iowrite8(0, board->iomem_base + (board->position * 0x4000) + 0x2000 + 0x11a00);
	// bank
	iowrite8(1, board->iomem_base + (board->position * 0x4000) + 0x0000 + 0x11900);
	iowrite8(1, board->iomem_base + (board->position * 0x4000) + 0x2000 + 0x11900);

	// get board serial number
	if ((cp = strstr(board->rom, "sn"))) {
		while ((*cp) && (*cp != 0x20)) {
			cp++;
		}
		cp++;
		dp = board->serial_number;
		while ((*cp) && (*cp != 0x20)) {
			*dp++ = *cp++;
		}
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
	snprintf(devname, POLYGATOR_BRDNAME_MAXLEN, "board-k32pcie-%s-%u", board->serial_number, board->position);
#else
	snprintf(devname, POLYGATOR_BRDNAME_MAXLEN, "be%s%u", board->serial_number, board->position);
#endif
	if (!(board->pg_board = polygator_board_register(THIS_MODULE, devname, &board->cdev, &k32pcie_board_fops))) {
		rc = -1;
		goto k32pcie_board_probe_error;
	}

	for (j = 0; j < 2; ++j) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
		snprintf(devname, VINETIC_DEVNAME_MAXLEN, "board-k32pcie-%s-%u-vin%lu", board->serial_number, board->position, (unsigned long int)j);
#else
		snprintf(devname, VINETIC_DEVNAME_MAXLEN, "ve%s%u%lu", board->serial_number, board->position, (unsigned long int)j);
#endif
		if (!(board->vinetics[j] = vinetic_device_register(THIS_MODULE, devname, ((uintptr_t)board->iomem_base) + (board->position * 0x4000),
													(j)?(k32pcie_vin_reset_1):(k32pcie_vin_reset_0),
													(j)?(k32pcie_vin_is_not_ready_1):(k32pcie_vin_is_not_ready_0),
													(j)?(k32pcie_vin_write_nwd_1):(k32pcie_vin_write_nwd_0),
													(j)?(k32pcie_vin_write_eom_1):(k32pcie_vin_write_eom_0),
													(j)?(k32pcie_vin_read_nwd_1):(k32pcie_vin_read_nwd_0),
													(j)?(k32pcie_vin_read_eom_1):(k32pcie_vin_read_eom_0),
													(j)?(k32pcie_vin_read_dia_1):(k32pcie_vin_read_dia_0)))) {
			rc = -1;
			goto k32pcie_board_probe_error;
		}
		for (i = 0; i < 4; ++i) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
			snprintf(devname, VINETIC_DEVNAME_MAXLEN, "board-k32pcie-%s-%u-vin%lu-rtp%lu", board->serial_number, board->position, (unsigned long int)j, (unsigned long int)i);
#else
			snprintf(devname, VINETIC_DEVNAME_MAXLEN, "re%s%u%lu%lu", board->serial_number, board->position, (unsigned long int)j, (unsigned long int)i);
#endif
			if (!vinetic_rtp_channel_register(THIS_MODULE, devname, board->vinetics[j], i)) {
				rc = -1;
				goto k32pcie_board_probe_error;
			}
		}
	}

	// set GSM module data
	for (i = 0; i < 8; ++i) {
		if (!(mod = kmalloc(sizeof(struct k32_gsm_module_data), GFP_KERNEL))) {
			log(KERN_ERR, "can't get memory for struct k32_gsm_module_data\n");
			rc = -1;
			goto k32pcie_board_probe_error;
		}
		memset(mod, 0, sizeof(struct k32_gsm_module_data));
		// select GSM module type
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
		if (mod->type == POLYGATOR_MODULE_TYPE_M10) {
			mod->control.bits_e.pwr = 1;		// power suply disabled
			mod->control.bits_e.key = 1;		// module key inactive
			mod->control.bits_e.gap = 3;
			mod->control.bits_e.sim_spd = 0;	// sim data rate default 9600
			mod->control.bits_e.com_spd = 3;	// at baud rate default 115200
		} else if (mod->type == POLYGATOR_MODULE_TYPE_SIM900) {
			mod->control.bits_e.pwr = 1;		// power suply disabled
			mod->control.bits_e.key = 1;		// module key inactive
			mod->control.bits_e.sim_spd = 0;	// sim data rate default 9600
			mod->control.bits_e.com_spd = 3;	// at baud rate default 115200
		} else if (mod->type == POLYGATOR_MODULE_TYPE_SIM300) {
			mod->control.bits_e.pwr = 1;		// power suply disabled
			mod->control.bits_e.key = 1;		// module key inactive
			mod->control.bits_e.sim_spd = 0;	// sim data rate default 9600
			mod->control.bits_e.com_spd = 3;	// at baud rate default 115200
		} else if ((mod->type == POLYGATOR_MODULE_TYPE_SIM5215) || (mod->type == POLYGATOR_MODULE_TYPE_SIM5215A2)) {
			mod->control.bits_e.pwr = 1;		// power suply disabled
			mod->control.bits_e.key = 1;		// module key inactive
			mod->control.bits_e.sim_spd = 0;	// sim data rate default 9600
			mod->control.bits_e.com_spd = 3;	// at baud rate default 115200
		} else {
			kfree(mod);
			continue;
		}

		mod->pos_on_board = i;
		mod->cbdata = ((uintptr_t)board->iomem_base) + 0x10000 + board->position * 0x4000 + (i / 4) * 0x2000 + (i % 4) * 0x400;
		mod->set_control = k32pcie_gsm_mod_set_control;
		mod->get_status = k32pcie_gsm_mod_get_status;
		mod->at_write = k32pcie_gsm_mod_at_write;
		mod->at_read = k32pcie_gsm_mod_at_read;
		mod->at_read16 = k32pcie_gsm_mod_at_read16;
		mod->sim_write = k32pcie_gsm_mod_sim_write;
		mod->sim_read = k32pcie_gsm_mod_sim_read;
		mod->sim_do_after_reset = k32pcie_gsm_mod_sim_do_after_reset;
		mod->imei_write = k32pcie_gsm_mod_imei_write;
		mod->imei_read = k32pcie_gsm_mod_imei_read;

        //mod->set_control(mod->cbdata, mod->pos_on_board, mod->control.full);
        timer_setup(&mod->at_poll_timer, k32pcie_tty_at_poll, 0);

		spin_lock_init(&mod->at_lock);
#ifdef TTY_PORT
		tty_port_init(&mod->at_port);
		mod->at_port.ops = &k32pcie_tty_at_port_ops;
		mod->at_port.close_delay = 0;
		mod->at_port.closing_wait = ASYNC_CLOSING_WAIT_NONE;
#endif
		board->gsm_modules[i] = mod;
	}

	// register polygator tty at device
	for (i = 0; i < 8; ++i) {
		if ((mod = board->gsm_modules[i])) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0)
			if (!(board->tty_at_channels[i] = polygator_tty_device_register(THIS_MODULE, mod, &mod->at_port, &k32pcie_tty_at_ops))) {
#else
			if (!(board->tty_at_channels[i] = polygator_tty_device_register(THIS_MODULE, mod, &k32pcie_tty_at_ops))) {
#endif
				log(KERN_ERR, "can't register polygator tty device\n");
				rc = -1;
				goto k32pcie_board_probe_error;
			}
		}
	}

	// register polygator simcard device
	for (i = 0; i < 8; ++i) {
		if (board->gsm_modules[i]) {
			if (!(board->simcard_channels[i] = simcard_device_register(THIS_MODULE,
																		board->gsm_modules[i],
																		k32pcie_sim_read,
																		k32pcie_sim_write,
																		k32pcie_sim_is_read_ready,
																		k32pcie_sim_is_write_ready,
																		k32pcie_sim_is_reset_request,
																		k32pcie_sim_set_speed,
																		k32pcie_sim_do_after_reset))) {
				log(KERN_ERR, "can't register polygator simcard device\n");
				rc = -1;
				goto k32pcie_board_probe_error;
			}
		}
	}

	pci_set_drvdata(pdev, board);

	return 0;

k32pcie_board_probe_error:

	if (board) {
		for (i = 0; i < 8; ++i) {
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
		for (j = 0; j < 2; ++j) {
			if (board->vinetics[j]) {
				for (i = 0; i < 4; ++i) {
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
		if (board->iomem_req) {
			pci_release_region(pdev, 0);
		}
		if (board->iomem_base) {
			pci_iounmap(pdev, board->iomem_base);
		}
		kfree(board);
	}
	return rc;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0)
static void k32pcie_board_remove(struct pci_dev *pdev)
#else
static void __devexit k32pcie_board_remove(struct pci_dev *pdev)
#endif
{
	size_t i, j;

	struct k32_board *board = pci_get_drvdata(pdev);

	for (i = 0; i < 8; ++i) {
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

	for (j = 0; j < 2; ++j) {
		if (board->vinetics[j]) {
			for (i = 0; i < 4; ++i) {
				if (board->vinetics[j]->rtp_channels[i]) {
					vinetic_rtp_channel_unregister(board->vinetics[j]->rtp_channels[i]);
				}
			}
			vinetic_device_unregister(board->vinetics[j]);
		}
	}

	polygator_board_unregister(board->pg_board);

	if (board->iomem_req) {
		pci_release_region(pdev, 0);
	}
	if (board->iomem_base) {
		pci_iounmap(pdev, board->iomem_base);
	}

	kfree(board);
}

static struct pci_driver k32pcie_driver = {
	.name = "k32pcie",
	.id_table = k32pcie_board_id_table,
	.probe = k32pcie_board_probe,
	.remove = k32pcie_board_remove,
};

static int k32pcie_tty_at_open(struct tty_struct *tty, struct file *filp)
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

        mod_timer(&mod->at_poll_timer, jiffies + 1);

        mod->at_tty = tty;
    } else {
        kfree(xbuf);
    }

    spin_unlock_bh(&mod->at_lock);

    return 0;
#endif
}

static void k32pcie_tty_at_close(struct tty_struct *tty, struct file *filp)
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

static int k32pcie_tty_at_write(struct tty_struct *tty, const unsigned char *buf, int count)
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

static int k32pcie_tty_at_write_room(struct tty_struct *tty)
{
	int res;
	struct polygator_tty_device *ptd = tty->driver_data;
	struct k32_gsm_module_data *mod = (struct k32_gsm_module_data *)ptd->data;

	spin_lock_bh(&mod->at_lock);

	res = SERIAL_XMIT_SIZE - mod->at_xmit_count;

	spin_unlock_bh(&mod->at_lock);

	return res;
}

static int k32pcie_tty_at_chars_in_buffer(struct tty_struct *tty)
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
static void k32pcie_tty_at_set_termios(struct tty_struct *tty, struct ktermios *old_termios)
#else
static void k32pcie_tty_at_set_termios(struct tty_struct *tty, struct termios *old_termios)
#endif
{
	speed_t baud;
	struct polygator_tty_device *ptd = tty->driver_data;
	struct k32_gsm_module_data *mod = (struct k32_gsm_module_data *)ptd->data;

	baud = tty_get_baud_rate(tty);

	spin_lock_bh(&mod->at_lock);

	switch (baud) {
		case 9600:
			mod->control.bits_e.com_spd = 0;
			break;
		default:
		case 115200:
			mod->control.bits_e.com_spd = 3;
			break;
	}

	mod->set_control(mod->cbdata, mod->pos_on_board, mod->control.full);

	spin_unlock_bh(&mod->at_lock);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
	tty_encode_baud_rate(tty, baud, baud);
#endif
}

static void k32pcie_tty_at_flush_buffer(struct tty_struct *tty)
{
	struct polygator_tty_device *ptd = tty->driver_data;
	struct k32_gsm_module_data *mod = (struct k32_gsm_module_data *)ptd->data;

	spin_lock_bh(&mod->at_lock);
	mod->at_xmit_count = mod->at_xmit_head = mod->at_xmit_tail = 0;
	spin_unlock_bh(&mod->at_lock);
	tty_wakeup(tty);
}

static void k32pcie_tty_at_hangup(struct tty_struct *tty)
{
#ifdef TTY_PORT
	struct polygator_tty_device *ptd = tty->driver_data;
	struct k32_gsm_module_data *mod = (struct k32_gsm_module_data *)ptd->data;
	tty_port_hangup(&mod->at_port);
#endif
}
#ifdef TTY_PORT
static int k32pcie_tty_at_port_carrier_raised(struct tty_port *port)
{
	return 1;
}

static void k32pcie_tty_at_port_dtr_rts(struct tty_port *port, int onoff)
{
}

static int k32pcie_tty_at_port_activate(struct tty_port *port, struct tty_struct *tty)
{
    struct k32_gsm_module_data *mod = container_of(port, struct k32_gsm_module_data, at_port);

    if (tty_port_alloc_xmit_buf(port) < 0) {
        return -ENOMEM;
    }

    mod->at_xmit_count = mod->at_xmit_head = mod->at_xmit_tail = 0;

    mod_timer(&mod->at_poll_timer, jiffies + 1);

    return 0;
}

static void k32pcie_tty_at_port_shutdown(struct tty_port *port)
{
	struct k32_gsm_module_data *mod = container_of(port, struct k32_gsm_module_data, at_port);

	del_timer_sync(&mod->at_poll_timer);

	tty_port_free_xmit_buf(port);
}
#endif
static int __init k32pcie_init(void)
{
	int rc;

	verbose("loading ...\n");

	// Register PCI driver
	if ((rc = pci_register_driver(&k32pcie_driver)) < 0) {
		log(KERN_ERR, "can't register pci driver\n");
		goto k32pcie_init_error;
	}

	verbose("loaded successfull\n");
	return 0;

k32pcie_init_error:
	return rc;
}

static void __exit k32pcie_exit(void)
{
	// Unregister PCI driver
	pci_unregister_driver(&k32pcie_driver);

	verbose("stopped\n");
}

module_init(k32pcie_init);
module_exit(k32pcie_exit);
