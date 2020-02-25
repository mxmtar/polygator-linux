/******************************************************************************/
/* g20-base.c                                                                 */
/******************************************************************************/

#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/serial.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/version.h>

#include "../arch/arm/include/asm/io.h"
#include "../arch/arm/mach-at91/include/mach/hardware.h"
#include "../arch/arm/mach-at91/include/mach/io.h"
#include "../arch/arm/mach-at91/include/mach/at91_pio.h"
#include "../arch/arm/mach-at91/include/mach/at91sam9260_matrix.h"

#include "polygator/polygator-base.h"

#include "polygator/vinetic-base.h"
#include "polygator/vinetic-def.h"

#include "polygator/simcard-base.h"

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,36) // 2,6,30 - orig
#define TTY_PORT
#endif

MODULE_AUTHOR("Maksym Tarasevych <mxmtar@gmail.com>");
MODULE_DESCRIPTION("Polygator Linux module for G20 device");
MODULE_LICENSE("GPL");

#define verbose(_fmt, _args...) printk(KERN_INFO "[polygator-%s] " _fmt, THIS_MODULE->name, ## _args)
#define log(_level, _fmt, _args...) printk(_level "[polygator-%s] %s:%d - %s(): " _fmt, THIS_MODULE->name, "g20-base.c", __LINE__, __PRETTY_FUNCTION__, ## _args)
#define debug(_fmt, _args...) printk(KERN_DEBUG "[polygator-%s] %s:%d - %s(): " _fmt, THIS_MODULE->name, "g20-base.c", __LINE__, __PRETTY_FUNCTION__, ## _args)

/*! */
#define G20_CS_VINETIC			0x1100
#define G20_CS_STATUS_VINETIC	0x1120
#define G20_PRESENCE_TEST1		0x1180
#define G20_MODE_AUTONOM		0x1190
#define G20_SIP_ONLY			0x11A0
#define G20_PRESENCE_TEST0		0x11B0
#define G20_RESET_VINETIC		0x11C0
#define G20_CS_ROM				0x11D0
#define G20_CS_PRESENCE			0x11E0
#define G20_RESET_BOARD			0x11F0
#define MB_MODE_AUTONOM			0x2300	// 0 - bank, 1 -standalone
#define MB_CS_ROM_KROSS			0x4000
#define MB_RESET_ROM			0x4800
/*! */

union g20_gsm_mod_status_reg {
	struct {
		uint8_t status:1;
		uint8_t at_rd_empty:1;
		uint8_t at_wr_empty:1;
		uint8_t sim_rd_empty:1;
		uint8_t sim_wr_empty:1;
		uint8_t sim_rst_req:1;
		uint8_t imei_rd_empty:1;
		uint8_t imei_wr_empty:1;
	} __attribute__((packed)) bits;
	uint8_t full;
} __attribute__((packed));

union g20_gsm_mod_control_reg {
	struct {
		uint8_t vbat:1; // 1 - disable, 0 - enable
		uint8_t pkey:1;
		uint8_t gap:2;
		uint8_t cn_speed_a:1;
		uint8_t cn_speed_b:1;
		uint8_t at_baudrate:2;
	} __attribute__((packed)) bits;
	uint8_t full;
} __attribute__((packed));

struct g20_gsm_module_data {

	int type;
	size_t pos_on_board;

	union g20_gsm_mod_control_reg control;

	uintptr_t cbdata;

	void (* set_control)(uintptr_t cbdata, size_t pos, uint8_t reg);
	uint8_t (* get_status)(uintptr_t cbdata, size_t pos);
	void (* at_write)(uintptr_t cbdata, size_t pos, uint8_t reg);
	uint8_t (* at_read)(uintptr_t cbdata, size_t pos);
	void (* sim_write)(uintptr_t cbdata, size_t pos, uint8_t reg);
	uint8_t (* sim_read)(uintptr_t cbdata, size_t pos);
	void (* imei_write)(uintptr_t cbdata, size_t pos, uint8_t reg);
	uint8_t (* imei_read)(uintptr_t cbdata, size_t pos);

	// at section
	int at_port_select;
	spinlock_t at_lock;
#ifdef TTY_PORT
	struct tty_port at_port;
#else
	size_t at_count;
	struct tty_struct *at_tty;
	unsigned char *at_xmit_buf;
#endif
	size_t at_xmit_count;
	size_t at_xmit_head;
	size_t at_xmit_tail;
	struct timer_list at_poll_timer;
};

struct g20_board {

	size_t index;

	struct polygator_board *pg_board;
	struct cdev cdev;

	uint8_t rom[256];
	size_t romsize;
	uint32_t sn;
	uint16_t type;

	struct vinetic *vinetic;

	struct g20_gsm_module_data *gsm_modules[4];

	struct polygator_tty_device *tty_at_channels[4];

	struct simcard_device *simcard_channels[4];
};

struct g20_board_private_data {
	struct g20_board *board;
	char buff[0x0C00];
	size_t length;
};

static int g20_tty_at_open(struct tty_struct *tty, struct file *filp);
static void g20_tty_at_close(struct tty_struct *tty, struct file *filp);
static int g20_tty_at_write(struct tty_struct *tty, const unsigned char *buf, int count);
static int g20_tty_at_write_room(struct tty_struct *tty);
static int g20_tty_at_chars_in_buffer(struct tty_struct *tty);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20)
static void g20_tty_at_set_termios(struct tty_struct *tty, struct ktermios *old_termios);
#else
static void g20_tty_at_set_termios(struct tty_struct *tty, struct termios *old_termios);
#endif
static void g20_tty_at_flush_buffer(struct tty_struct *tty);
static void g20_tty_at_hangup(struct tty_struct *tty);

static struct tty_operations g20_tty_at_ops = {
	.open = g20_tty_at_open,
	.close = g20_tty_at_close,
	.write = g20_tty_at_write,
	.write_room = g20_tty_at_write_room,
	.chars_in_buffer = g20_tty_at_chars_in_buffer,
	.set_termios = g20_tty_at_set_termios,
	.flush_buffer = g20_tty_at_flush_buffer,
	.hangup = g20_tty_at_hangup,
};

static int g20_tty_at_port_carrier_raised(struct tty_port *port);
static void g20_tty_at_port_dtr_rts(struct tty_port *port, int onoff);
static int g20_tty_at_port_activate(struct tty_port *tport, struct tty_struct *tty);
static void g20_tty_at_port_shutdown(struct tty_port *port);

static const struct tty_port_operations g20_tty_at_port_ops = {
	.carrier_raised = g20_tty_at_port_carrier_raised,
	.dtr_rts = g20_tty_at_port_dtr_rts,
	.activate = g20_tty_at_port_activate,
	.shutdown = g20_tty_at_port_shutdown,
};

static char mainboard_rom[256];
static struct g20_board *g20_boards[4];

static struct resource * g20_cs3_iomem_reg = NULL;
static struct resource * g20_cs4_iomem_reg = NULL;

static void __iomem * g20_cs3_base_ptr = NULL;
static void __iomem * g20_cs4_base_ptr = NULL;

static void g20_vinetic_reset(uintptr_t cbdata)
{
	uintptr_t addr;

	addr = (uintptr_t)g20_cs3_base_ptr;
	addr += cbdata + G20_RESET_VINETIC;
	iowrite8(0, addr);
	mdelay(10);
	iowrite8(1,  addr);
	mdelay(2);
// 	log(KERN_INFO, "%08lx\n", addr);
}
static void g20_vinetic_write_nwd(uintptr_t cbdata, uint16_t value)
{
	uintptr_t addr;

	addr = (uintptr_t)g20_cs4_base_ptr;
	addr += cbdata + G20_CS_VINETIC + 0x04;
	iowrite16(value, addr);
// 	log(KERN_INFO, "%08lx: %04x\n", addr, value);
}
static void g20_vinetic_write_eom(uintptr_t cbdata, uint16_t value)
{
	uintptr_t addr;

	addr = (uintptr_t)g20_cs4_base_ptr;
	addr += cbdata + G20_CS_VINETIC + 0x06;
	iowrite16(value, addr);
// 	log(KERN_INFO, "%08lx: %04x\n", addr, value);
}
static uint16_t g20_vinetic_read_nwd(uintptr_t cbdata)
{
	uint16_t value;
	uintptr_t addr;

	addr = (uintptr_t)g20_cs4_base_ptr;
	addr += cbdata + G20_CS_VINETIC + 0x04;
	value = ioread16(addr);
// 	log(KERN_INFO, "%08lx: %04x\n", addr, value);
	return value;
}
static uint16_t g20_vinetic_read_eom(uintptr_t cbdata)
{
	uint16_t value;
	uintptr_t addr;

	addr = (uintptr_t)g20_cs4_base_ptr;
	addr += cbdata + G20_CS_VINETIC + 0x06;
	value = ioread16(addr);
// 	log(KERN_INFO, "%08lx: %04x\n", addr, value);
	return value;
}
#if 0
static size_t g20_vinetic_is_not_ready(uintptr_t cbdata)
{
	size_t status;
	uintptr_t addr;

	addr = (uintptr_t)g20_cs3_base_ptr;
	addr += cbdata + G20_CS_STATUS_VINETIC;
	status = ioread8(addr);
// 	log(KERN_INFO, "%08lx: %04x\n", addr, (unsigned int)status);
	status &= 1;
// 	log(KERN_INFO, "%08lx: %lu\n", addr, (long unsigned int)status);
	return status;
}
#else
static size_t g20_vinetic_is_not_ready(uintptr_t cbdata)
{
	uintptr_t addr;
	union vin_reg_ir reg_ir;

	addr = (uintptr_t)g20_cs4_base_ptr;
	addr += cbdata + G20_CS_VINETIC + 0x18;
	reg_ir.full = ioread16(addr);
	return reg_ir.bits.rdyq;
}
#endif
static uint16_t g20_vinetic_read_dia(uintptr_t cbdata)
{
	uint16_t value;
	uintptr_t addr;

	addr = (uintptr_t)g20_cs4_base_ptr;
	addr += cbdata + G20_CS_VINETIC + 0x18;
	value = ioread16(addr);
	return value;
}

static void g20_gsm_mod_set_control(uintptr_t cbdata, size_t pos, uint8_t reg)
{
	uintptr_t addr = cbdata;

	iowrite8(reg, addr);
}

static uint8_t g20_gsm_mod_get_status(uintptr_t cbdata, size_t pos)
{
	uintptr_t addr = cbdata;

	return ioread8(addr);
}

static void g20_gsm_mod_at_write(uintptr_t cbdata, size_t pos, uint8_t reg)
{
	uintptr_t addr = cbdata;

	addr += 0x10;

	iowrite8(reg, addr);
}

static uint8_t g20_gsm_mod_at_read(uintptr_t cbdata, size_t pos)
{
	uintptr_t addr = cbdata;

	addr += 0x10;

	return ioread8(addr);
}

static void g20_gsm_mod_sim_write(uintptr_t cbdata, size_t pos, uint8_t reg)
{
	uintptr_t addr = cbdata;

	addr += 0x20;

	iowrite8(reg, addr);
}

static uint8_t g20_gsm_mod_sim_read(uintptr_t cbdata, size_t pos)
{
	uintptr_t addr = cbdata;

	addr += 0x20;

	return ioread8(addr);
}

static void g20_gsm_mod_imei_write(uintptr_t cbdata, size_t pos, uint8_t reg)
{
	uintptr_t addr = cbdata;

	addr += 0x30;

	iowrite8(reg, addr);
}

static uint8_t g20_gsm_mod_imei_read(uintptr_t cbdata, size_t pos)
{
	uintptr_t addr = cbdata;

	addr += 0x30;

	return ioread8(addr);
}

static uint8_t g20_sim_read(void *data)
{
	struct g20_gsm_module_data *mod = (struct g20_gsm_module_data *)data;

	return mod->sim_read(mod->cbdata, mod->pos_on_board);
}

static void g20_sim_write(void *data, uint8_t value)
{
	struct g20_gsm_module_data *mod = (struct g20_gsm_module_data *)data;

	mod->sim_write(mod->cbdata, mod->pos_on_board, value);
}

static int g20_sim_is_read_ready(void *data)
{
	union g20_gsm_mod_status_reg status;
	struct g20_gsm_module_data *mod = (struct g20_gsm_module_data *)data;

	status.full = mod->get_status(mod->cbdata, mod->pos_on_board);

	return status.bits.sim_rd_empty;
}

static int g20_sim_is_write_ready(void *data)
{
	union g20_gsm_mod_status_reg status;
	struct g20_gsm_module_data *mod = (struct g20_gsm_module_data *)data;

	status.full = mod->get_status(mod->cbdata, mod->pos_on_board);

	return status.bits.sim_wr_empty;
}

static int g20_sim_is_reset_request(void *data)
{
	union g20_gsm_mod_status_reg status;
	struct g20_gsm_module_data *mod = (struct g20_gsm_module_data *)data;

	status.full = mod->get_status(mod->cbdata, mod->pos_on_board);

	return status.bits.sim_rst_req;
}

static void g20_sim_set_speed(void *data, int speed)
{
	struct g20_gsm_module_data *mod = (struct g20_gsm_module_data *)data;

	switch (speed)
	{
		case 57600:
			mod->control.bits.cn_speed_a = 1;
			mod->control.bits.cn_speed_b = 0;
			break;
		case 115200:
			mod->control.bits.cn_speed_a = 0;
			mod->control.bits.cn_speed_b = 1;
			break;
		default: // 9600 
			mod->control.bits.cn_speed_a = 0;
			mod->control.bits.cn_speed_b = 0;
			break;
	}

	mod->set_control(mod->cbdata, mod->pos_on_board, mod->control.full);
}

static void g20_tty_at_poll(unsigned long addr)
{
	char buff[512];
	size_t len;
	struct tty_struct *tty;
	union g20_gsm_mod_status_reg status;
	struct g20_gsm_module_data *mod = (struct g20_gsm_module_data *)addr;

	len = 0;
#if 0
	// read received data
	while (len < sizeof(buff))
	{
		// read status register
		at->status.full = at->mod_status(at->cbdata, at->pos_on_board);
		if (at->status.bits.at_rd_empty)
			break;
		// put char to receiving buffer
		buff[len++] = at->mod_at_read(at->cbdata, at->pos_on_board);
	}
#else
	// read received data
	while (len < sizeof(buff))
	{
		// read status register
		status.full = mod->get_status(mod->cbdata, mod->pos_on_board);
		// select port
		if (mod->at_port_select) {
			// auxilary
			if (status.bits.imei_rd_empty)
				break;
			// put char to receiving buffer
			buff[len++] = mod->imei_read(mod->cbdata, mod->pos_on_board);
		} else {
			// main
			if (status.bits.at_rd_empty)
				break;
			// put char to receiving buffer
			buff[len++] = mod->at_read(mod->cbdata, mod->pos_on_board);
		}
	}
#endif

	spin_lock(&mod->at_lock);

	 while (mod->at_xmit_count)
	 {
		// read status register
		status.full = mod->get_status(mod->cbdata, mod->pos_on_board);
		// select port
		if (mod->at_port_select) {
			// auxilary
			// check for transmitter is ready
			if (status.bits.imei_wr_empty) {
				// put char to transmitter buffer
#ifdef TTY_PORT
				mod->imei_write(mod->cbdata, mod->pos_on_board, mod->at_port.xmit_buf[mod->at_xmit_tail]);
#else
				mod->imei_write(mod->cbdata, mod->pos_on_board, mod->at_xmit_buf[mod->at_xmit_tail]);
#endif
				mod->at_xmit_tail++;
				if (mod->at_xmit_tail == SERIAL_XMIT_SIZE)
					mod->at_xmit_tail = 0;
				mod->at_xmit_count--;
			}
		} else {
			// main
			// check for transmitter is ready
			if (status.bits.at_wr_empty) {
				// put char to transmitter buffer
#ifdef TTY_PORT
				mod->at_write(mod->cbdata, mod->pos_on_board, mod->at_port.xmit_buf[mod->at_xmit_tail]);
#else
				mod->at_write(mod->cbdata, mod->pos_on_board, mod->at_xmit_buf[mod->at_xmit_tail]);
#endif
				mod->at_xmit_tail++;
				if (mod->at_xmit_tail == SERIAL_XMIT_SIZE)
					mod->at_xmit_tail = 0;
				mod->at_xmit_count--;
			}
		}
	}

	spin_unlock(&mod->at_lock);

	if (len) {
#ifdef TTY_PORT
		tty = tty_port_tty_get(&mod->at_port);
		tty_insert_flip_string(tty, buff, len);
		tty_flip_buffer_push(tty);
		tty_kref_put(tty);
#else
		tty = mod->at_tty;
		tty_insert_flip_string(tty, buff, len);
		tty_flip_buffer_push(tty);
#endif
	}

	mod_timer(&mod->at_poll_timer, jiffies + 1);
}

static int g20_board_open(struct inode *inode, struct file *filp)
{
	ssize_t res;
	size_t i, j;
	size_t len;

	struct g20_board *brd;
	struct g20_board_private_data *private_data;
	union g20_gsm_mod_status_reg status;
	struct g20_gsm_module_data *mod;

	brd = container_of(inode->i_cdev, struct g20_board, cdev);

	if (!(private_data = kmalloc(sizeof(struct g20_board_private_data), GFP_KERNEL))) {
		log(KERN_ERR, "can't get memory=%lu bytes\n", (unsigned long int)sizeof(struct g20_board_private_data));
		res = -ENOMEM;
		goto g20_open_error;
	}
	private_data->board = brd;

	len = 0;
	for (i=0; i<4; i++)
	{
		if (brd->tty_at_channels[i]) {
			mod = brd->gsm_modules[i];
			status.full = mod->get_status(mod->cbdata, mod->pos_on_board);
			len += sprintf(private_data->buff+len, "GSM%lu %s %s %s VIN%luALM%lu VIO=%u\r\n",
							(unsigned long int)i,
							polygator_print_gsm_module_type(mod->type),
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
							brd->tty_at_channels[i]?dev_name(brd->tty_at_channels[i]->device):"unknown",
							brd->simcard_channels[i]?dev_name(brd->simcard_channels[i]->device):"unknown",
#else
							brd->tty_at_channels[i]?brd->tty_at_channels[i]->device->class_id:"unknown",
							brd->simcard_channels[i]?brd->simcard_channels[i]->device->class_id:"unknown",
#endif
							(unsigned long int)(i/4),
							(unsigned long int)(i%4),
							status.bits.status);
		}
	}
	if (brd->vinetic) {
		len += sprintf(private_data->buff+len, "VIN0 %s\r\n",
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
						dev_name(brd->vinetic->device)
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
						dev_name(brd->vinetic->device)
#else
						brd->vinetic->device->class_id
#endif
						);
		for (j=0; j<4; j++)
		{
			if (brd->vinetic->rtp_channels[j])
				len += sprintf(private_data->buff+len, "VIN0RTP%lu %s\r\n",
								(unsigned long int)j,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
								dev_name(brd->vinetic->rtp_channels[j]->device)
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
								dev_name(brd->vinetic->rtp_channels[j]->device)
#else
								brd->vinetic->rtp_channels[j]->device->class_id
#endif
							);
		}
	}

	private_data->length = len;

	filp->private_data = private_data;

	return 0;

g20_open_error:
	if (private_data) kfree(private_data);
	return res;
}

static int g20_board_release(struct inode *inode, struct file *filp)
{
	struct g20_board_private_data *private_data = filp->private_data;

	kfree(private_data);
	return 0;
}

static ssize_t g20_board_read(struct file *filp, char __user *buff, size_t count, loff_t *offp)
{
	size_t len;
	ssize_t res;
	struct g20_board_private_data *private_data = filp->private_data;

	res = (private_data->length > filp->f_pos)?(private_data->length - filp->f_pos):(0);

	if (res) {
		len = res;
		len = min(count, len);
		if (copy_to_user(buff, private_data->buff + filp->f_pos, len)) {
			res = -EINVAL;
			goto g20_board_read_end;
		}
		*offp = filp->f_pos + len;
	}

g20_board_read_end:
	return res;
}

static ssize_t g20_board_write(struct file *filp, const char __user *buff, size_t count, loff_t *offp)
{
	ssize_t res;
	char cmd[256];
	size_t len;

	uint32_t at_chan;
	uint32_t pwr_state;
	uint32_t key_state;
	uint32_t baudrate;
	uint32_t serial;
	struct g20_gsm_module_data *mod;
	struct g20_board_private_data *private_data = filp->private_data;

	memset(cmd, 0, sizeof(cmd));
	len = sizeof(cmd) - 1;
	len = min(len, count);

	if (copy_from_user(cmd, buff, len)) {
		res = -EINVAL;
		goto g20_board_write_end;
	}

	if (sscanf(cmd, "GSM%u PWR=%u", &at_chan, &pwr_state) == 2) {
		if ((at_chan >= 0) && (at_chan <= 3) && (private_data->board->gsm_modules[at_chan])) {
			mod = private_data->board->gsm_modules[at_chan];
			mod->control.bits.vbat = !pwr_state;
			mod->set_control(mod->cbdata, mod->pos_on_board, mod->control.full);
			res = len;
		} else
			res= -ENODEV;
	} else if (sscanf(cmd, "GSM%u KEY=%u", &at_chan, &key_state) == 2) {
		if ((at_chan >= 0) && (at_chan <= 3) && (private_data->board->gsm_modules[at_chan])) {
			mod = private_data->board->gsm_modules[at_chan];
			mod->control.bits.pkey = !key_state;
			mod->set_control(mod->cbdata, mod->pos_on_board, mod->control.full);
			res = len;
		} else
			res= -ENODEV;
	} else if (sscanf(cmd, "GSM%u BAUDRATE=%u", &at_chan, &baudrate) == 2) {
		if ((at_chan >= 0) && (at_chan <= 3) && (private_data->board->gsm_modules[at_chan])) {
			mod = private_data->board->gsm_modules[at_chan];
			if (baudrate == 9600)
				mod->control.bits.at_baudrate = 0;
			else
				mod->control.bits.at_baudrate = 2;
			mod->set_control(mod->cbdata, mod->pos_on_board, mod->control.full);
			res = len;
		} else
			res= -ENODEV;
	} else if (sscanf(cmd, "GSM%u SERIAL=%u", &at_chan, &serial) == 2) {
		if ((at_chan >= 0) && (at_chan <= 7) && (private_data->board->gsm_modules[at_chan])) {
			mod = private_data->board->gsm_modules[at_chan];
			if (mod->type == POLYGATOR_MODULE_TYPE_SIM300)
				mod->at_port_select = serial;
			res = len;
		} else
			res = -ENODEV;
	} else
		res = -ENOMSG;

g20_board_write_end:
	return res;
}

static struct file_operations g20_board_fops = {
	.owner   = THIS_MODULE,
	.open    = g20_board_open,
	.release = g20_board_release,
	.read    = g20_board_read,
	.write   = g20_board_write,
};

static int g20_tty_at_open(struct tty_struct *tty, struct file *filp)
{
	struct polygator_tty_device *ptd = tty->driver_data;
	struct g20_gsm_module_data *mod = (struct g20_gsm_module_data *)ptd->data;

#ifdef TTY_PORT
	return tty_port_open(&mod->at_port, tty, filp);
#else
	unsigned char *xbuf;
	
	if (!(xbuf = kmalloc(SERIAL_XMIT_SIZE, GFP_KERNEL)))
		return -ENOMEM;

	spin_lock_bh(&mod->at_lock);

	if (!mod->at_count++) {
		mod->at_xmit_buf = xbuf;
		mod->at_xmit_count = mod->at_xmit_head = mod->at_xmit_tail = 0;

		mod->at_poll_timer.function = k32pci_tty_at_poll;
		mod->at_poll_timer.data = (unsigned long)mod;
		mod->at_poll_timer.expires = jiffies + 1;
		add_timer(&mod->at_poll_timer);
	
		mod->at_tty = tty;
	} else
		kfree(xbuf);

	spin_unlock_bh(&mod->at_lock);

	return 0;
#endif
}

static void g20_tty_at_close(struct tty_struct *tty, struct file *filp)
{
	struct polygator_tty_device *ptd = tty->driver_data;
	struct g20_gsm_module_data *mod = (struct g20_gsm_module_data *)ptd->data;

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

static int g20_tty_at_write(struct tty_struct *tty, const unsigned char *buf, int count)
{
	int res = 0;
	size_t len;
	struct polygator_tty_device *ptd = tty->driver_data;
	struct g20_gsm_module_data *mod = (struct g20_gsm_module_data *)ptd->data;

	spin_lock_bh(&mod->at_lock);

	if (mod->at_xmit_count < SERIAL_XMIT_SIZE) {
		while (1)
		{
			if (mod->at_xmit_head == mod->at_xmit_tail) {
				if (mod->at_xmit_count)
					len = 0;
				else
					len = SERIAL_XMIT_SIZE - mod->at_xmit_head;
			} else if (mod->at_xmit_head > mod->at_xmit_tail)
				len = SERIAL_XMIT_SIZE - mod->at_xmit_head;
			else
				len = mod->at_xmit_tail - mod->at_xmit_head;

			len = min(len, (size_t)count);
			if (!len)
				break;
#ifdef TTY_PORT
			memcpy(mod->at_port.xmit_buf + mod->at_xmit_head, buf, len);
#else
			memcpy(mod->at_xmit_buf + mod->at_xmit_head, buf, len);
#endif
			mod->at_xmit_head += len;
			if (mod->at_xmit_head == SERIAL_XMIT_SIZE)
				mod->at_xmit_head = 0;
			mod->at_xmit_count += len;
			buf += len;
			count -= len;
			res += len;
		}
	}

	spin_unlock_bh(&mod->at_lock);

	return res ;
}

static int g20_tty_at_write_room(struct tty_struct *tty)
{
	int res;
	struct polygator_tty_device *ptd = tty->driver_data;
	struct g20_gsm_module_data *mod = (struct g20_gsm_module_data *)ptd->data;

	spin_lock_bh(&mod->at_lock);

	res = SERIAL_XMIT_SIZE - mod->at_xmit_count;

	spin_unlock_bh(&mod->at_lock);

	return res;
}

static int g20_tty_at_chars_in_buffer(struct tty_struct *tty)
{
	int res;
	struct polygator_tty_device *ptd = tty->driver_data;
	struct g20_gsm_module_data *mod = (struct g20_gsm_module_data *)ptd->data;

	spin_lock_bh(&mod->at_lock);

	res = mod->at_xmit_count;

	spin_unlock_bh(&mod->at_lock);

	return res;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20)
static void g20_tty_at_set_termios(struct tty_struct *tty, struct ktermios *old_termios)
#else
static void g20_tty_at_set_termios(struct tty_struct *tty, struct termios *old_termios)
#endif
{
	speed_t baud;
	struct polygator_tty_device *ptd = tty->driver_data;
	struct g20_gsm_module_data *mod = (struct g20_gsm_module_data *)ptd->data;

	baud = tty_get_baud_rate(tty);

	spin_lock_bh(&mod->at_lock);

	switch (baud)
	{
		case 9600:
			mod->control.bits.at_baudrate = 0;
			break;
		default:
			mod->control.bits.at_baudrate = 2;
			break;
	}
	
	mod->set_control(mod->cbdata, mod->pos_on_board, mod->control.full);

	spin_unlock_bh(&mod->at_lock);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
	tty_encode_baud_rate(tty, baud, baud);
#endif
}

static void g20_tty_at_flush_buffer(struct tty_struct *tty)
{
	struct polygator_tty_device *ptd = tty->driver_data;
	struct g20_gsm_module_data *mod = (struct g20_gsm_module_data *)ptd->data;

	spin_lock_bh(&mod->at_lock);
	mod->at_xmit_count = mod->at_xmit_head = mod->at_xmit_tail = 0;
	spin_unlock_bh(&mod->at_lock);
	tty_wakeup(tty);
}

static void g20_tty_at_hangup(struct tty_struct *tty)
{
#ifdef TTY_PORT
	struct polygator_tty_device *ptd = tty->driver_data;
	struct g20_gsm_module_data *mod = (struct g20_gsm_module_data *)ptd->data;
	tty_port_hangup(&mod->at_port);
#endif
}
#ifdef TTY_PORT
static int g20_tty_at_port_carrier_raised(struct tty_port *port)
{
	return 1;
}

static void g20_tty_at_port_dtr_rts(struct tty_port *port, int onoff)
{
}

static int g20_tty_at_port_activate(struct tty_port *port, struct tty_struct *tty)
{
	struct g20_gsm_module_data *mod = container_of(port, struct g20_gsm_module_data, at_port);

	if (tty_port_alloc_xmit_buf(port) < 0)
		return -ENOMEM;
	mod->at_xmit_count = mod->at_xmit_head = mod->at_xmit_tail = 0;

	mod->at_poll_timer.function = g20_tty_at_poll;
	mod->at_poll_timer.data = (unsigned long)mod;
	mod->at_poll_timer.expires = jiffies + 1;
	add_timer(&mod->at_poll_timer);

	return 0;
}

static void g20_tty_at_port_shutdown(struct tty_port *port)
{
	struct g20_gsm_module_data *mod = container_of(port, struct g20_gsm_module_data, at_port);

	del_timer_sync(&mod->at_poll_timer);

	tty_port_free_xmit_buf(port);
}
#endif
static int __init g20_init(void)
{
	size_t i, k;
	u32 data;
	u8 modtype;
	char devname[VINETIC_DEVNAME_MAXLEN];
	struct g20_gsm_module_data *mod;
	int rc = 0;

	verbose("loading ...\n");

	for (k=0; k<4; k++)
		g20_boards[k] = NULL;

	// Assign CS3, CS4 to SMC
	data = at91_sys_read(AT91_MATRIX_EBICSA);
	data &= ~(AT91_MATRIX_CS3A | AT91_MATRIX_CS4A);
	data |= (AT91_MATRIX_CS3A_SMC | AT91_MATRIX_CS4A_SMC);
	at91_sys_write(AT91_MATRIX_EBICSA, data);

	// Configure PIOC for using CS3, CS4
	at91_sys_write(AT91_PIOC + PIO_PDR, (1 << 14)|(1 << 8)); /* Disable Register */
	data = at91_sys_read(AT91_PIOC + PIO_PSR); /* Status Register */
	at91_sys_write(AT91_PIOC + PIO_ASR, (1 << 14)|(1 << 8)); /* Peripheral A Select Register */
	data = at91_sys_read(AT91_PIOC + PIO_ABSR); /* AB Status Register */

	// Configure SMC CS3 timings
	at91_sys_write(AT91_SMC + 0x30 + 0x0, 0x01020102); // old at91_sys_write(AT91_SMC + 0x30 + 0x0, 0x03030303);
	at91_sys_write(AT91_SMC + 0x30 + 0x4, 0x0f0d0f0d); // old at91_sys_write(AT91_SMC + 0x30 + 0x4, 0x100d100d);
	at91_sys_write(AT91_SMC + 0x30 + 0x8, 0x00150015); // old at91_sys_write(AT91_SMC + 0x30 + 0x8, 0x00000000);
	at91_sys_write(AT91_SMC + 0x30 + 0xc, 0x10001103);
	// Configure SMC CS4 timings
	at91_sys_write(AT91_SMC + 0x40 + 0x0, 0x01020102);
	at91_sys_write(AT91_SMC + 0x40 + 0x4, 0x0f0d0f0d);
	at91_sys_write(AT91_SMC + 0x40 + 0x8, 0x00150015);
	at91_sys_write(AT91_SMC + 0x40 + 0xc, 0x10111003);

	// Request and remap i/o memory region for cs3
	if (check_mem_region(AT91_CHIPSELECT_3, 0x10000)) {
		log(KERN_ERR, "i/o memory region for cs3 already used\n");
		rc = -ENOMEM;
		goto g20_init_error;
	}
	if (!(g20_cs3_iomem_reg = request_mem_region(AT91_CHIPSELECT_3, 0x10000, "polygator_g20"))) {
		log(KERN_ERR, "can't request i/o memory region for cs3\n");
		rc = -ENOMEM;
		goto g20_init_error;
	}
	if (!(g20_cs3_base_ptr = ioremap_nocache(AT91_CHIPSELECT_3, 0x10000))) {
		log(KERN_ERR, "can't remap i/o memory for cs3\n");
		rc = -ENOMEM;
		goto g20_init_error;
	}

	// Request and remap i/o memory region for cs4
	if (check_mem_region(AT91_CHIPSELECT_4, 0x10000)) {
		log(KERN_ERR, "i/o memory region for cs4 already used\n");
		rc = -ENOMEM;
		goto g20_init_error;
	}
	if (!(g20_cs4_iomem_reg = request_mem_region(AT91_CHIPSELECT_4, 0x10000, "polygator_g20"))) {
		log(KERN_ERR, "can't request i/o memory region for cs4\n");
		rc = -ENOMEM;
		goto g20_init_error;
	}
	if (!(g20_cs4_base_ptr = ioremap_nocache(AT91_CHIPSELECT_4, 0x10000))) {
		log(KERN_ERR, "can't remap i/o memory for cs4\n");
		rc = -ENOMEM;
		goto g20_init_error;
	}

	// Read ROM from mainboard
	iowrite8(0, g20_cs3_base_ptr + MB_RESET_ROM);
	mdelay(1);
	iowrite8(1, g20_cs3_base_ptr + MB_RESET_ROM);
	mdelay(1);
	iowrite8(0, g20_cs3_base_ptr + MB_RESET_ROM);
	for (i=0; i<sizeof(mainboard_rom); i++)
		mainboard_rom[i] = ioread8(g20_cs3_base_ptr + MB_CS_ROM_KROSS);
	verbose("mainboard: \"%s\"\n", &mainboard_rom[3]);
	// set MainBoard SIM standalone mode
	iowrite8(1, g20_cs3_base_ptr + MB_MODE_AUTONOM);
	// Search for g20 boards
	for (k=0; k<4; k++)
	{
		// Reset G20 board
		iowrite8(0, g20_cs3_base_ptr + G20_RESET_BOARD + (0x0200 * k));
		mdelay(1);
		iowrite8(1, g20_cs3_base_ptr + G20_RESET_BOARD + (0x0200 * k));
		// Test G8 board present
		iowrite8(0x55, g20_cs3_base_ptr + G20_PRESENCE_TEST0 + (0x0200 * k));
		iowrite8(0xAA, g20_cs3_base_ptr + G20_PRESENCE_TEST1 + (0x0200 * k));
		if ((ioread8(g20_cs3_base_ptr + G20_PRESENCE_TEST0 + (0x0200 * k)) == 0x55) &&
			(ioread8(g20_cs3_base_ptr + G20_PRESENCE_TEST1 + (0x0200 * k)) == 0xAA)) {
			// board present
			// alloc memory for board data
			if (!(g20_boards[k] = kmalloc(sizeof(struct g20_board), GFP_KERNEL))) {
				log(KERN_ERR, "can't get memory for struct g20_board\n");
				rc = -1;
				goto g20_init_error;
			}
			memset(g20_boards[k], 0, sizeof(struct g20_board));
			g20_boards[k]->index = k;
			snprintf(devname, POLYGATOR_BRDNAME_MAXLEN, "board-g20-%lu", (unsigned long int)k);
			if (!(g20_boards[k]->pg_board =  polygator_board_register(THIS_MODULE, devname, &g20_boards[k]->cdev, &g20_board_fops))) {
				rc = -1;
				goto g20_init_error;
			}
			for (i=0; i<sizeof(g20_boards[k]->rom); i++)
				g20_boards[k]->rom[i] = ioread8(g20_cs3_base_ptr + G20_CS_ROM + (0x0200 * k));
			verbose("found board G4: \"%s\"\n", &g20_boards[k]->rom[3]);
			// set autonom
			iowrite8(1, g20_cs3_base_ptr + G20_MODE_AUTONOM + (0x0200 * k));
			// set sip only
			iowrite8(0, g20_cs3_base_ptr + G20_SIP_ONLY + (0x0200 * k));
			// Register vinetic
			snprintf(devname, VINETIC_DEVNAME_MAXLEN, "board-g20-%lu-vin0", (unsigned long int)k);
			if (!(g20_boards[k]->vinetic = vinetic_device_register(THIS_MODULE, devname, (0x0200 * k),
																g20_vinetic_reset,
																g20_vinetic_is_not_ready,
																g20_vinetic_write_nwd,
																g20_vinetic_write_eom,
																g20_vinetic_read_nwd,
																g20_vinetic_read_eom,
																g20_vinetic_read_dia))) {
				rc = -1;
				goto g20_init_error;
			}
			for (i=0; i<4; i++)
			{
				snprintf(devname, VINETIC_DEVNAME_MAXLEN, "board-g20-%lu-vin0-rtp%lu", (unsigned long int)k, (unsigned long int)i);
				if (!(vinetic_rtp_channel_register(THIS_MODULE, devname, g20_boards[k]->vinetic, i))) {
					rc = -1;
					goto g20_init_error;
				}
			}
			// get board GSM module type
			modtype = ioread8(g20_cs3_base_ptr + G20_CS_PRESENCE + (0x0200 * k));
			// set AT command channels
			for (i=0; i<4; i++)
			{
				if (!(mod= kmalloc(sizeof(struct g20_gsm_module_data), GFP_KERNEL))) {
					log(KERN_ERR, "can't get memory for struct g20_gsm_module_data\n");
					rc = -1;
					goto g20_init_error;
				}
				memset(mod, 0, sizeof(struct g20_gsm_module_data));
				// select GSM module type
				switch (modtype)
				{
					case 4:
					case 5:
						mod->type = POLYGATOR_MODULE_TYPE_M10;
						break;
					case 6:
						mod->type = POLYGATOR_MODULE_TYPE_SIM5215;
						break;
					case 7:
						mod->type = POLYGATOR_MODULE_TYPE_SIM900;
						break;
					default:
						mod->type = POLYGATOR_MODULE_TYPE_UNKNOWN;
						verbose("unsupported GSM module type=%u\n", modtype);
						break;
				}

				if (mod->type == POLYGATOR_MODULE_TYPE_UNKNOWN) {
					kfree(mod);
					continue;
				}

				mod->control.bits.vbat = 1;
				mod->control.bits.pkey = 1;
				mod->control.bits.cn_speed_a = 0;
				mod->control.bits.cn_speed_b = 0;
				mod->control.bits.at_baudrate = 2;

				mod->pos_on_board = i;
				mod->cbdata = (((uintptr_t)g20_cs3_base_ptr) + 0x1000 + (0x0200 * k) + (0x40 * (i%4)));
				mod->set_control = g20_gsm_mod_set_control;
				mod->get_status = g20_gsm_mod_get_status;
				mod->at_write = g20_gsm_mod_at_write;
				mod->at_read = g20_gsm_mod_at_read;
				mod->sim_write = g20_gsm_mod_sim_write;
				mod->sim_read = g20_gsm_mod_sim_read;
				mod->imei_write = g20_gsm_mod_imei_write;
				mod->imei_read = g20_gsm_mod_imei_read;

				mod->set_control(mod->cbdata, mod->pos_on_board, mod->control.full);
				init_timer(&mod->at_poll_timer);

				spin_lock_init(&mod->at_lock);
#ifdef TTY_PORT
				tty_port_init(&mod->at_port);
				mod->at_port.ops = &g20_tty_at_port_ops;
				mod->at_port.close_delay = 0;
				mod->at_port.closing_wait = ASYNC_CLOSING_WAIT_NONE;
#endif
				g20_boards[k]->gsm_modules[i] = mod;
			}

			// register polygator tty at device
			for (i=0; i<4; i++)
			{
				if (g20_boards[k]->gsm_modules[i]) {
					if (!(g20_boards[k]->tty_at_channels[i] = polygator_tty_device_register(THIS_MODULE, g20_boards[k]->gsm_modules[i], &g20_tty_at_ops))) {
						log(KERN_ERR, "can't register polygator tty device\n");
						rc = -1;
						goto g20_init_error;
					}
				}
			}

			// register polygator simcard device
			for (i=0; i<4; i++)
			{
				if (g20_boards[k]->gsm_modules[i]) {
					if (!(g20_boards[k]->simcard_channels[i] = simcard_device_register(THIS_MODULE,
																							g20_boards[k]->gsm_modules[i],
																							g20_sim_read,
																							g20_sim_write,
																							g20_sim_is_read_ready,
																							g20_sim_is_write_ready,
																							g20_sim_is_reset_request,
																							g20_sim_set_speed))) {
						log(KERN_ERR, "can't register polygator simcard device\n");
						rc = -1;
						goto g20_init_error;
					}
				}
			}
		}
	}

	verbose("loaded successfull\n");
	return rc;

g20_init_error:
	for (k=0; k<4; k++)
	{
		if (g20_boards[k]) {
			// channels
			for (i=0; i<4; i++)
			{
				if (g20_boards[k]->simcard_channels[i]) simcard_device_unregister(g20_boards[k]->simcard_channels[i]);
				if (g20_boards[k]->tty_at_channels[i]) polygator_tty_device_unregister(g20_boards[k]->tty_at_channels[i]);
				if (g20_boards[k]->gsm_modules[i]) {
					del_timer_sync(&g20_boards[k]->gsm_modules[i]->at_poll_timer);
					kfree(g20_boards[k]->gsm_modules[i]);
				}
			}
			// vinetic
			for (i=0; i<4; i++)
				vinetic_rtp_channel_unregister(g20_boards[k]->vinetic->rtp_channels[i]);
			vinetic_device_unregister(g20_boards[k]->vinetic);
			// board
			if (g20_boards[k]->pg_board) polygator_board_unregister(g20_boards[k]->pg_board);
			kfree(g20_boards[k]);
		}
	}

	if (g20_cs3_iomem_reg) release_mem_region(AT91_CHIPSELECT_3, 0x10000);
	if (g20_cs3_base_ptr) iounmap(g20_cs3_base_ptr);
	if (g20_cs4_iomem_reg) release_mem_region(AT91_CHIPSELECT_4, 0x10000);
	if (g20_cs4_base_ptr) iounmap(g20_cs4_base_ptr);
	return rc;
}

static void __exit g20_exit(void)
{
	size_t i, k;

	for (k=0; k<4; k++)
	{
		if (g20_boards[k]) {
			// channels
			for (i=0; i<4; i++)
			{
				if (g20_boards[k]->simcard_channels[i]) simcard_device_unregister(g20_boards[k]->simcard_channels[i]);
				if (g20_boards[k]->tty_at_channels[i]) polygator_tty_device_unregister(g20_boards[k]->tty_at_channels[i]);
				if (g20_boards[k]->gsm_modules[i]) {
					del_timer_sync(&g20_boards[k]->gsm_modules[i]->at_poll_timer);
					kfree(g20_boards[k]->gsm_modules[i]);
				}
			}
			// vinetic
			for (i=0; i<4; i++)
				vinetic_rtp_channel_unregister(g20_boards[k]->vinetic->rtp_channels[i]);
			vinetic_device_unregister(g20_boards[k]->vinetic);
			// board
			polygator_board_unregister(g20_boards[k]->pg_board);
			kfree(g20_boards[k]);
		}
	}

	iounmap(g20_cs3_base_ptr);
	release_mem_region(AT91_CHIPSELECT_3, 0x10000);
	iounmap(g20_cs4_base_ptr);
	release_mem_region(AT91_CHIPSELECT_4, 0x10000);

	verbose("stopped\n");
}

module_init(g20_init);
module_exit(g20_exit);

/******************************************************************************/
/* end of g20-base.c                                                          */
/******************************************************************************/
