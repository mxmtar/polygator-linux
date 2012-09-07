/******************************************************************************/
/* g8-base.c                                                                  */
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

MODULE_AUTHOR("Maksym Tarasevych <mxmtar@ukr.net>");
MODULE_DESCRIPTION("Polygator Linux module for G8 device");
MODULE_LICENSE("GPL");

static int tty_at_major = 0;
module_param(tty_at_major, int, 0);
MODULE_PARM_DESC(tty_at_major, "Major number for AT-command channel of Polygator G8 device");

#define verbose(_fmt, _args...) printk(KERN_INFO "[polygator-%s] " _fmt, THIS_MODULE->name, ## _args)
#define log(_level, _fmt, _args...) printk(_level "[polygator-%s] %s:%d - %s(): " _fmt, THIS_MODULE->name, "g8-base.c", __LINE__, __PRETTY_FUNCTION__, ## _args)
#define debug(_fmt, _args...) printk(KERN_DEBUG "[polygator-%s] %s:%d - %s(): " _fmt, THIS_MODULE->name, "g8-base.c", __LINE__, __PRETTY_FUNCTION__, ## _args)

/*! */
#define G8_CS_VINETIC			0x1100
#define G8_CS_STATUS_VINETIC	0x1120
#define G8_PRESENCE_TEST1		0x1180
#define G8_MODE_AUTONOM			0x1190
#define G8_SIP_ONLY				0x11A0
#define G8_PRESENCE_TEST0		0x11B0
#define G8_RESET_VINETIC		0x11C0
#define G8_CS_ROM				0x11D0
#define G8_CS_PRESENCE			0x11E0
#define G8_RESET_BOARD			0x11F0
#define MB_CS_ROM_KROSS			0x4000
#define MB_RESET_ROM			0x4800
/*! */

#define G8_TTY_AT_DEVICE_MAXCOUNT 8

union g8_at_ch_status_reg {
	struct {
		u_int8_t status:1;
		u_int8_t at_rd_empty:1;
		u_int8_t at_wr_empty:1;
		u_int8_t sim_rd_empty:1;
		u_int8_t sim_wr_empty:1;
		u_int8_t gap:3;
	} __attribute__((packed)) bits;
	u_int8_t full;
} __attribute__((packed));

union g8_at_ch_control_reg {
	struct {
		u_int8_t vbat:1; // 1 - disable, 0 - enable
		u_int8_t pkey:1;
		u_int8_t gap:2;
		u_int8_t cn_speed_a:1;
		u_int8_t cn_speed_b:1;
		u_int8_t at_baudrate:2;
	} __attribute__((packed)) bits;
	u_int8_t full;
} __attribute__((packed));

struct g8_tty_at_channel {

	int gsm_mod_type;
	size_t pos_on_board;

	union g8_at_ch_status_reg status;
	union g8_at_ch_control_reg control;

	struct timer_list poll_timer;

	struct tty_port port;

	spinlock_t lock;

	size_t xmit_count;
	size_t xmit_head;
	size_t xmit_tail;

	uintptr_t cbdata;

	void (* mod_control)(uintptr_t cbdata, size_t pos, u_int8_t reg);
	u_int8_t (* mod_status)(uintptr_t cbdata, size_t pos);
	void (* mod_at_write)(uintptr_t cbdata, size_t pos, u_int8_t reg);
	u_int8_t (* mod_at_read)(uintptr_t cbdata, size_t pos);

	int tty_at_minor;
	struct device *device;
};

struct g8_board {

	struct polygator_board *pg_board;
	struct cdev cdev;

	char rom[256];

	struct vinetic *vinetic[2];

	struct g8_tty_at_channel *tty_at_channels[8];
};

struct g8_board_private_data {
	struct g8_board *board;
	char buff[0x0C00];
	size_t length;
};

static struct tty_driver *g8_tty_at_driver = NULL;

static int g8_tty_at_open(struct tty_struct *tty, struct file *filp);
static void g8_tty_at_close(struct tty_struct *tty, struct file *filp);
static int g8_tty_at_write(struct tty_struct *tty, const unsigned char *buf, int count);
static int g8_tty_at_write_room(struct tty_struct *tty);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20)
static void g8_tty_at_set_termios(struct tty_struct *tty, struct ktermios *old_termios);
#else
static void g8_tty_at_set_termios(struct tty_struct *tty, struct termios *old_termios);
#endif
static void g8_tty_at_flush_buffer(struct tty_struct *tty);
static void g8_tty_at_hangup(struct tty_struct *tty);

static struct tty_operations g8_tty_at_ops = {
	.open = g8_tty_at_open,
	.close = g8_tty_at_close,
	.write = g8_tty_at_write,
	.write_room = g8_tty_at_write_room,
// 	.chars_in_buffer = g8_tty_at_chars_in_buffer,
	.set_termios = g8_tty_at_set_termios,
	.flush_buffer = g8_tty_at_flush_buffer,
	.hangup = g8_tty_at_hangup,
};

static int g8_tty_at_port_carrier_raised(struct tty_port *port);
static void g8_tty_at_port_dtr_rts(struct tty_port *port, int onoff);
static int g8_tty_at_port_activate(struct tty_port *tport, struct tty_struct *tty);
static void g8_tty_at_port_shutdown(struct tty_port *port);

static const struct tty_port_operations g8_tty_at_port_ops = {
	.carrier_raised =g8_tty_at_port_carrier_raised,
	.dtr_rts = g8_tty_at_port_dtr_rts,
	.activate = g8_tty_at_port_activate,
	.shutdown = g8_tty_at_port_shutdown,
};

static char mainboard_rom[256];
static struct g8_board *g8_board = NULL;

static struct resource * g8_cs3_iomem_reg = NULL;
static struct resource * g8_cs4_iomem_reg = NULL;

static void __iomem * g8_cs3_base_ptr = NULL;
static void __iomem * g8_cs4_base_ptr = NULL;

static void g8_vinetic_reset(uintptr_t cbdata)
{
	uintptr_t addr;

	addr = (uintptr_t)g8_cs3_base_ptr;
	addr += cbdata + G8_RESET_VINETIC;
	iowrite8(0, addr);
	mdelay(10);
	iowrite8(1,  addr);
	mdelay(2);
// 	log(KERN_INFO, "%08lx\n", addr);
}
static void g8_vinetic_write_nwd(uintptr_t cbdata, u_int16_t value)
{
	uintptr_t addr;

	addr = (uintptr_t)g8_cs4_base_ptr;
	addr += cbdata + G8_CS_VINETIC + 0x04;
	iowrite16(value, addr);
// 	log(KERN_INFO, "%08lx: %04x\n", addr, value);
}
static void g8_vinetic_write_eom(uintptr_t cbdata, u_int16_t value)
{
	uintptr_t addr;

	addr = (uintptr_t)g8_cs4_base_ptr;
	addr += cbdata + G8_CS_VINETIC + 0x06;
	iowrite16(value, addr);
// 	log(KERN_INFO, "%08lx: %04x\n", addr, value);
}
static u_int16_t g8_vinetic_read_nwd(uintptr_t cbdata)
{
	u_int16_t value;
	uintptr_t addr;

	addr = (uintptr_t)g8_cs4_base_ptr;
	addr += cbdata + G8_CS_VINETIC + 0x04;
	value = ioread16(addr);
// 	log(KERN_INFO, "%08lx: %04x\n", addr, value);
	return value;
}
static u_int16_t g8_vinetic_read_eom(uintptr_t cbdata)
{
	u_int16_t value;
	uintptr_t addr;

	addr = (uintptr_t)g8_cs4_base_ptr;
	addr += cbdata + G8_CS_VINETIC + 0x06;
	value = ioread16(addr);
// 	log(KERN_INFO, "%08lx: %04x\n", addr, value);
	return value;
}
static size_t g8_vinetic_is_not_ready(uintptr_t cbdata)
{
#if 0
	size_t status;
	uintptr_t addr;
	addr = (uintptr_t)g8_cs3_base_ptr;
	addr += cbdata + G8_CS_STATUS_VINETIC;
	status = ioread8(addr);
// 	log(KERN_INFO, "%08lx: %04x\n", addr, (unsigned int)status);
	status &= 1;
// 	log(KERN_INFO, "%08lx: %lu\n", addr, (long unsigned int)status);
	return status;
#else
	union vin_reg_ir reg_ir;
	uintptr_t addr;
	addr = (uintptr_t)g8_cs4_base_ptr;
	addr += cbdata + G8_CS_VINETIC + 0x18;
	reg_ir.full = ioread16(addr);
// 	log(KERN_INFO, "%08lx: %04x\n", addr, reg_ir.full);
	return reg_ir.bits.rdyq;
#endif
}
static u_int16_t g8_vinetic_read_dia(uintptr_t cbdata)
{
	return 0;
}

static void g8_mod_control(uintptr_t cbdata, size_t pos, u_int8_t reg)
{
	uintptr_t addr = cbdata;

	iowrite8(reg, addr);
}

static u_int8_t g8_mod_status(uintptr_t cbdata, size_t pos)
{
	uintptr_t addr = cbdata;

	return ioread8(addr);
}

static void g8_mod_at_write(uintptr_t cbdata, size_t pos, u_int8_t reg)
{
	uintptr_t addr = cbdata;

	addr += 0x10;

	iowrite8(reg, addr);
}

static u_int8_t g8_mod_at_read(uintptr_t cbdata, size_t pos)
{
	uintptr_t addr = cbdata;

	addr += 0x10;

	return ioread8(addr);
}

static void g8_tty_at_poll(unsigned long addr)
{
	char buff[512];
	size_t len;
	struct tty_struct *tty;
	struct g8_tty_at_channel *ch = (struct g8_tty_at_channel *)addr;

	len = 0;

	// read received data
	while (len < sizeof(buff))
	{
		// read status register
		ch->status.full = ch->mod_status(ch->cbdata, ch->pos_on_board);
		if (ch->status.bits.at_rd_empty)
			break;
		// put char to receiving buffer
		buff[len++] = ch->mod_at_read(ch->cbdata, ch->pos_on_board);
	}

	spin_lock(&ch->lock);
	if (ch->xmit_count) {
		// read status register
		ch->status.full = ch->mod_status(ch->cbdata, ch->pos_on_board);
		// check for transmitter is ready
		if (ch->status.bits.at_wr_empty) {
// 			verbose("test=%lu head=%lu tail=%lu %c\n", (unsigned long int)ch->xmit_count, (unsigned long int)ch->xmit_head, (unsigned long int)ch->xmit_tail, *(ch->port.xmit_buf + ch->xmit_tail));
			ch->mod_at_write(ch->cbdata, ch->pos_on_board, *(ch->port.xmit_buf + ch->xmit_tail));
			ch->xmit_tail++;
			if (ch->xmit_tail == SERIAL_XMIT_SIZE)
				ch->xmit_tail = 0;
			ch->xmit_count--;
		}
	}
	spin_unlock(&ch->lock);

	if (len) {
		tty = tty_port_tty_get(&ch->port);
		tty_insert_flip_string(tty, buff, len);
		tty_flip_buffer_push(tty);
		tty_kref_put(tty);
	}

	mod_timer(&ch->poll_timer, jiffies + 1);
}

static int g8_board_open(struct inode *inode, struct file *filp)
{
	ssize_t res;
	size_t i, j;
	size_t len;

	struct g8_board *brd;
	struct g8_board_private_data *private_data;

	brd = container_of(inode->i_cdev, struct g8_board, cdev);

	if (!(private_data = kmalloc(sizeof(struct g8_board_private_data), GFP_KERNEL))) {
		log(KERN_ERR, "can't get memory=%lu bytes\n", (unsigned long int)sizeof(struct g8_board_private_data));
		res = -ENOMEM;
		goto g8_open_error;
	}
	private_data->board = brd;

	len = 0;
	for (i=0; i<8; i++)
	{
		if (brd->tty_at_channels[i]) {
			brd->tty_at_channels[i]->status.full = brd->tty_at_channels[i]->mod_status(brd->tty_at_channels[i]->cbdata, brd->tty_at_channels[i]->pos_on_board);
			len += sprintf(private_data->buff+len, "GSM%lu g8AT%d %s VIN%luALM%lu VIO=%u\r\n",
															(unsigned long int)i, brd->tty_at_channels[i]->tty_at_minor, polygator_print_gsm_module_type(brd->tty_at_channels[i]->gsm_mod_type),
															(unsigned long int)(i/4), (unsigned long int)(i%4),
															brd->tty_at_channels[i]->status.bits.status);
		}
	}
	for (j=0; j<2; j++)
	{
		if (brd->vinetic[j]) {
			len += sprintf(private_data->buff+len, "VIN%lu board-g8-vin%lu\r\n", (unsigned long int)j, (unsigned long int)j);
			for (i=0; i<4; i++)
			{
				if (brd->vinetic[j]->rtp_channels[i])
					len += sprintf(private_data->buff+len, "VIN%luRTP%lu board-g8-vin%lu-rtp%lu\r\n", (unsigned long int)j, (unsigned long int)i, (unsigned long int)j, (unsigned long int)i);
			}
		}
	}

	private_data->length = len;

	filp->private_data = private_data;

	return 0;

g8_open_error:
	if (private_data) kfree(private_data);
	return res;
}

static int g8_board_release(struct inode *inode, struct file *filp)
{
	struct g8_board_private_data *private_data = filp->private_data;

	kfree(private_data);
	return 0;
}

static ssize_t g8_board_read(struct file *filp, char __user *buff, size_t count, loff_t *offp)
{
	size_t len;
	ssize_t res;
	struct g8_board_private_data *private_data = filp->private_data;

	res = (private_data->length > filp->f_pos)?(private_data->length - filp->f_pos):(0);

	if (res) {
		len = res;
		len = min(count, len);
		if (copy_to_user(buff, private_data->buff + filp->f_pos, len)) {
			res = -EINVAL;
			goto g8_board_read_end;
		}
		*offp = filp->f_pos + len;
	}

g8_board_read_end:
	return res;
}

static ssize_t g8_board_write(struct file *filp, const char __user *buff, size_t count, loff_t *offp)
{
	ssize_t res;
	char cmd[256];
	size_t len;

	u_int32_t at_chan;
	u_int32_t pwr_state;
	u_int32_t key_state;
	u_int32_t baudrate;
	struct g8_board_private_data *private_data = filp->private_data;

	memset(cmd, 0, sizeof(cmd));
	len = sizeof(cmd) - 1;
	len = min(len,count);

	if (copy_from_user(cmd, buff, len)) {
		res = -EINVAL;
		goto g8_board_write_end;
	}

	if (sscanf(cmd, "GSM%u PWR=%u", &at_chan, &pwr_state) == 2) {
		if ((at_chan >= 0) && (at_chan <= 7) && (private_data->board->tty_at_channels[at_chan])) {
			private_data->board->tty_at_channels[at_chan]->control.bits.vbat = !pwr_state;
			private_data->board->tty_at_channels[at_chan]->mod_control(private_data->board->tty_at_channels[at_chan]->cbdata, at_chan, private_data->board->tty_at_channels[at_chan]->control.full);
			res = len;
		} else
			res= -ENODEV;
	} else if (sscanf(cmd, "GSM%u KEY=%u", &at_chan, &key_state) == 2) {
		if ((at_chan >= 0) && (at_chan <= 7) && (private_data->board->tty_at_channels[at_chan])) {
			private_data->board->tty_at_channels[at_chan]->control.bits.pkey = !key_state;
			private_data->board->tty_at_channels[at_chan]->mod_control(private_data->board->tty_at_channels[at_chan]->cbdata, at_chan, private_data->board->tty_at_channels[at_chan]->control.full);
			res = len;
		} else
			res= -ENODEV;
	} else if (sscanf(cmd, "GSM%u BAUDRATE=%u", &at_chan, &baudrate) == 2) {
		if ((at_chan >= 0) && (at_chan <= 7) && (private_data->board->tty_at_channels[at_chan])) {
			if (baudrate == 9600)
				private_data->board->tty_at_channels[at_chan]->control.bits.at_baudrate = 0;
			else
				private_data->board->tty_at_channels[at_chan]->control.bits.at_baudrate = 2;
			private_data->board->tty_at_channels[at_chan]->mod_control(private_data->board->tty_at_channels[at_chan]->cbdata, at_chan, private_data->board->tty_at_channels[at_chan]->control.full);
			res = len;
		} else
			res= -ENODEV;
	} else
		res = -ENOMSG;

g8_board_write_end:
	return res;
}

static struct file_operations g8_board_fops = {
	.owner   = THIS_MODULE,
	.open    = g8_board_open,
	.release = g8_board_release,
	.read    = g8_board_read,
	.write   = g8_board_write,
};

static int g8_tty_at_open(struct tty_struct *tty, struct file *filp)
{
	struct g8_tty_at_channel *ch;

	if ((tty->index < 0) && (tty->index > 7)) {
		return -ENODEV;
	}
	ch = g8_board->tty_at_channels[tty->index];
	tty->driver_data = ch;

	return tty_port_open(&ch->port, tty, filp);
}

static void g8_tty_at_close(struct tty_struct *tty, struct file *filp)
{
	struct g8_tty_at_channel *ch = tty->driver_data;

	tty_port_close(&ch->port, tty, filp);
	return;
}

static int g8_tty_at_write(struct tty_struct *tty, const unsigned char *buf, int count)
{
	int res = 0;
	size_t len;
	struct g8_tty_at_channel *ch = tty->driver_data;

	spin_lock_bh(&ch->lock);

	if (ch->xmit_count < SERIAL_XMIT_SIZE) {
		while (1)
		{
			if (ch->xmit_head == ch->xmit_tail) {
				if (ch->xmit_count)
					len = 0;
				else
					len = SERIAL_XMIT_SIZE - ch->xmit_head;
			} else if (ch->xmit_head > ch->xmit_tail)
				len = SERIAL_XMIT_SIZE - ch->xmit_head;
			else
				len = ch->xmit_tail - ch->xmit_head;

			len = min(len, (size_t)count);
			if (!len)
				break;

			memcpy(ch->port.xmit_buf + ch->xmit_head, buf, len);
			ch->xmit_head += len;
			if (ch->xmit_head == SERIAL_XMIT_SIZE)
				ch->xmit_head = 0;
			ch->xmit_count += len;
			buf += len;
			count -= len;
			res += len;
		}
	}

	spin_unlock_bh(&ch->lock);

	return res;
}

static int g8_tty_at_write_room(struct tty_struct *tty)
{
	int res;
	struct g8_tty_at_channel *ch = tty->driver_data;

	spin_lock_bh(&ch->lock);

	res = SERIAL_XMIT_SIZE - ch->xmit_count;

	spin_unlock_bh(&ch->lock);
	
	return res;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20)
static void g8_tty_at_set_termios(struct tty_struct *tty, struct ktermios *old_termios)
#else
static void g8_tty_at_set_termios(struct tty_struct *tty, struct termios *old_termios)
#endif
{
	speed_t baud;
	struct g8_tty_at_channel *ch = tty->driver_data;

	baud = tty_get_baud_rate(tty);

	switch (baud)
	{
		case 9600:
			ch->control.bits.at_baudrate = 0;
			break;
		default:
			ch->control.bits.at_baudrate = 2;
			break;
	}
	
	ch->mod_control(ch->cbdata, ch->pos_on_board, ch->control.full);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
	tty_encode_baud_rate(tty, baud, baud);
#endif
}

static void g8_tty_at_flush_buffer(struct tty_struct *tty)
{
	struct g8_tty_at_channel *ch = tty->driver_data;

	spin_lock_bh(&ch->lock);
	ch->xmit_count = ch->xmit_head = ch->xmit_tail = 0;
	spin_unlock_bh(&ch->lock);
	tty_wakeup(tty);
}

static void g8_tty_at_hangup(struct tty_struct *tty)
{
	struct g8_tty_at_channel *ch = tty->driver_data;
	tty_port_hangup(&ch->port);
}

static int g8_tty_at_port_carrier_raised(struct tty_port *port)
{
	return 1;
}

static void g8_tty_at_port_dtr_rts(struct tty_port *port, int onoff)
{
}

static int g8_tty_at_port_activate(struct tty_port *port, struct tty_struct *tty)
{
	struct g8_tty_at_channel *ch = container_of(port, struct g8_tty_at_channel, port);

	if (tty_port_alloc_xmit_buf(port) < 0)
		return -ENOMEM;
	ch->xmit_count = ch->xmit_head = ch->xmit_tail = 0;

	ch->poll_timer.function = g8_tty_at_poll;
	ch->poll_timer.data = (unsigned long)ch;
	ch->poll_timer.expires = jiffies + 1;
	add_timer(&ch->poll_timer);

	return 0;
}

static void g8_tty_at_port_shutdown(struct tty_port *port)
{
	struct g8_tty_at_channel *ch = container_of(port, struct g8_tty_at_channel, port);

	del_timer_sync(&ch->poll_timer);

	tty_port_free_xmit_buf(port);
}

static int __init g8_init(void)
{
	size_t i,j;
	u32 data;
	u8 modtype;
	char devname[VINETIC_DEVNAME_MAXLEN];
	int rc = 0;

	verbose("loading ...\n");

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
		goto g8_init_error;
	}
	if (!(g8_cs3_iomem_reg = request_mem_region(AT91_CHIPSELECT_3, 0x10000, "polygator_g8"))) {
		log(KERN_ERR, "can't request i/o memory region for cs3\n");
		rc = -ENOMEM;
		goto g8_init_error;
	}
	if (!(g8_cs3_base_ptr = ioremap_nocache(AT91_CHIPSELECT_3, 0x10000))) {
		log(KERN_ERR, "can't remap i/o memory for cs3\n");
		rc = -ENOMEM;
		goto g8_init_error;
	}

	// Request and remap i/o memory region for cs4
	if (check_mem_region(AT91_CHIPSELECT_4, 0x10000)) {
		log(KERN_ERR, "i/o memory region for cs4 already used\n");
		rc = -ENOMEM;
		goto g8_init_error;
	}
	if (!(g8_cs4_iomem_reg = request_mem_region(AT91_CHIPSELECT_4, 0x10000, "polygator_g8"))) {
		log(KERN_ERR, "can't request i/o memory region for cs4\n");
		rc = -ENOMEM;
		goto g8_init_error;
	}
	if (!(g8_cs4_base_ptr = ioremap_nocache(AT91_CHIPSELECT_4, 0x10000))) {
		log(KERN_ERR, "can't remap i/o memory for cs4\n");
		rc = -ENOMEM;
		goto g8_init_error;
	}

	// registering tty device for AT-command channel
	g8_tty_at_driver = alloc_tty_driver(G8_TTY_AT_DEVICE_MAXCOUNT);
	if (!g8_tty_at_driver) {
		log(KERN_ERR, "can't allocated memory for tty driver\n");
		return -ENOMEM;
	}

	g8_tty_at_driver->owner = THIS_MODULE;
	g8_tty_at_driver->driver_name = "g8_tty_at";
	g8_tty_at_driver->name = "polygator/g8AT";
	g8_tty_at_driver->major = tty_at_major;
	g8_tty_at_driver->minor_start = 0;
	g8_tty_at_driver->type = TTY_DRIVER_TYPE_SERIAL;
	g8_tty_at_driver->subtype = SERIAL_TYPE_NORMAL;
	g8_tty_at_driver->init_termios = tty_std_termios;
	g8_tty_at_driver->init_termios.c_iflag &= ~ICRNL;
	g8_tty_at_driver->init_termios.c_cflag = B9600 | CS8 | HUPCL | CLOCAL | CREAD;
	g8_tty_at_driver->init_termios.c_lflag &= ~ECHO;
	g8_tty_at_driver->init_termios.c_ispeed = 9600;
	g8_tty_at_driver->init_termios.c_ospeed = 9600;
	g8_tty_at_driver->flags = TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;
	tty_set_operations(g8_tty_at_driver, &g8_tty_at_ops);

	if ((rc = tty_register_driver(g8_tty_at_driver))) {
		log(KERN_ERR, "can't register g8_tty_at driver: rc=%d\n", rc);
		// release allocated tty driver environment
		put_tty_driver(g8_tty_at_driver);
		g8_tty_at_driver = NULL;
		goto g8_init_error;
	}
	debug("tty_at_major=%d\n", g8_tty_at_driver->major);

	// Read ROM from mainboard
	iowrite8(0, g8_cs3_base_ptr + MB_RESET_ROM);
	mdelay(1);
	iowrite8(1, g8_cs3_base_ptr + MB_RESET_ROM);
	mdelay(1);
	iowrite8(0, g8_cs3_base_ptr + MB_RESET_ROM);
	for (i=0; i<sizeof(mainboard_rom); i++)
		mainboard_rom[i] = ioread8(g8_cs3_base_ptr + MB_CS_ROM_KROSS);
	verbose("mainboard: %s\n", &mainboard_rom[3]);
	// Reset G8 board both logical parts
	iowrite8(0, g8_cs3_base_ptr + G8_RESET_BOARD);
	iowrite8(0, g8_cs3_base_ptr + G8_RESET_BOARD + 0x0200);
	mdelay(1);
	iowrite8(1, g8_cs3_base_ptr + G8_RESET_BOARD);
	iowrite8(1, g8_cs3_base_ptr + G8_RESET_BOARD + 0x0200);
	// Test G8 board present
	iowrite8(0x55, g8_cs3_base_ptr + G8_PRESENCE_TEST0);
	iowrite8(0xAA, g8_cs3_base_ptr + G8_PRESENCE_TEST1);
	iowrite8(0x5A, g8_cs3_base_ptr + G8_PRESENCE_TEST0 + 0x0200);
	iowrite8(0xA5, g8_cs3_base_ptr + G8_PRESENCE_TEST1 + 0x0200);
	if ((ioread8(g8_cs3_base_ptr + G8_PRESENCE_TEST0) == 0x5A) && (ioread8(g8_cs3_base_ptr + G8_PRESENCE_TEST1) == 0xA5) &&
		(ioread8(g8_cs3_base_ptr + G8_PRESENCE_TEST0 + 0x0200) == 0x5A) && (ioread8(g8_cs3_base_ptr + G8_PRESENCE_TEST1 + 0x0200) == 0xA5)) {
		// board present
		// alloc memory for board data
		if (!(g8_board = kmalloc(sizeof(struct g8_board), GFP_KERNEL))) {
			log(KERN_ERR, "can't get memory for struct g8_board\n");
			rc = -1;
			goto g8_init_error;
		}
		memset(g8_board, 0, sizeof(struct g8_board));

		snprintf(devname, POLYGATOR_BRDNAME_MAXLEN, "board-g8");
		if (!(g8_board->pg_board =  polygator_board_register(THIS_MODULE, devname, &g8_board->cdev, &g8_board_fops))) {
			rc = -1;
			goto g8_init_error;
		}
		for (i=0; i<sizeof(g8_board->rom); i++)
			g8_board->rom[i] = ioread8(g8_cs3_base_ptr + G8_CS_ROM);
		verbose("found board G8: %s\n", &g8_board->rom[3]);
		// set autonom
		iowrite8(1, g8_cs3_base_ptr + G8_MODE_AUTONOM);
		iowrite8(1, g8_cs3_base_ptr + G8_MODE_AUTONOM + 0x0200);
		// set sip only
		iowrite8(0, g8_cs3_base_ptr + G8_SIP_ONLY);
		iowrite8(0, g8_cs3_base_ptr + G8_SIP_ONLY + 0x0200);
		// Register vinetic
		for (j=0; j<2; j++)
		{
			snprintf(devname, VINETIC_DEVNAME_MAXLEN, "board-g8-vin%lu", (unsigned long int)j);
			if (!(g8_board->vinetic[j] = vinetic_device_register(THIS_MODULE, devname, (0x0200 * j),
															g8_vinetic_reset,
															g8_vinetic_is_not_ready,
															g8_vinetic_write_nwd,
															g8_vinetic_write_eom,
															g8_vinetic_read_nwd,
															g8_vinetic_read_eom,
															g8_vinetic_read_dia))) {
				rc = -1;
				goto g8_init_error;
			}
			for (i=0; i<4; i++)
			{
				snprintf(devname, VINETIC_DEVNAME_MAXLEN, "board-g8-vin%lu-rtp%lu", (unsigned long int)j, (unsigned long int)i);
				if (!(vinetic_rtp_channel_register(THIS_MODULE, devname, g8_board->vinetic[j], i))) {
					rc = -1;
					goto g8_init_error;
				}
			}
		}
		// get board GSM module type
		modtype = ioread8(g8_cs3_base_ptr + G8_CS_PRESENCE);
		// set AT command channels
		for (i=0; i<8; i++)
		{
			if (!(g8_board->tty_at_channels[i] = kmalloc(sizeof(struct g8_tty_at_channel), GFP_KERNEL))) {
				log(KERN_ERR, "can't get memory for struct g8_tty_at_channel\n");
				rc = -1;
				goto g8_init_error;
			}
			memset(g8_board->tty_at_channels[i], 0, sizeof(struct g8_tty_at_channel));
			if (modtype == 4)
				g8_board->tty_at_channels[i]->gsm_mod_type = POLYGATOR_MODULE_TYPE_M10;
			if (modtype == 6)
				g8_board->tty_at_channels[i]->gsm_mod_type = POLYGATOR_MODULE_TYPE_SIM5215;
			g8_board->tty_at_channels[i]->control.bits.vbat = 1;
			g8_board->tty_at_channels[i]->control.bits.pkey = 1;
			g8_board->tty_at_channels[i]->control.bits.cn_speed_a = 0;
			g8_board->tty_at_channels[i]->control.bits.cn_speed_b = 0;
			g8_board->tty_at_channels[i]->control.bits.at_baudrate = 2;

			spin_lock_init(&g8_board->tty_at_channels[i]->lock);
			tty_port_init(&g8_board->tty_at_channels[i]->port);
			g8_board->tty_at_channels[i]->port.ops = &g8_tty_at_port_ops;
			g8_board->tty_at_channels[i]->port.close_delay = 0;
			g8_board->tty_at_channels[i]->port.closing_wait = ASYNC_CLOSING_WAIT_NONE;

			g8_board->tty_at_channels[i]->pos_on_board = i;
			g8_board->tty_at_channels[i]->tty_at_minor = i;
			g8_board->tty_at_channels[i]->cbdata = (((uintptr_t)g8_cs3_base_ptr) + 0x1000 + (0x0200 * (i/4)) + (0x40 * (i%4)));
			g8_board->tty_at_channels[i]->mod_control = g8_mod_control;
			g8_board->tty_at_channels[i]->mod_status = g8_mod_status;
			g8_board->tty_at_channels[i]->mod_at_write = g8_mod_at_write;
			g8_board->tty_at_channels[i]->mod_at_read = g8_mod_at_read;

			// register device on sysfs
			g8_board->tty_at_channels[i]->device = tty_register_device(g8_tty_at_driver, i, NULL);
			if (IS_ERR(g8_board->tty_at_channels[i]->device)) {
				log(KERN_ERR, "can't register tty device\n");
				rc = -1;
				goto g8_init_error;
			}

			g8_board->tty_at_channels[i]->mod_control(g8_board->tty_at_channels[i]->cbdata, g8_board->tty_at_channels[i]->pos_on_board, g8_board->tty_at_channels[i]->control.full);

			init_timer(&g8_board->tty_at_channels[i]->poll_timer);
		}
	}

	verbose("loaded successfull\n");
	return rc;

g8_init_error:
	if (g8_board) {
		for (i=0; i<8; i++)
		{
			if (g8_board->tty_at_channels[i]) {
				del_timer_sync(&g8_board->tty_at_channels[i]->poll_timer);
				if (g8_board->tty_at_channels[i]->device)
					tty_unregister_device(g8_tty_at_driver, g8_board->tty_at_channels[i]->tty_at_minor);
				kfree(g8_board->tty_at_channels[i]);
			}
		}
		for (j=0; j<2; j++)
		{
			if (g8_board->vinetic[j]) {
				for (i=0; i<4; i++)
				{
					if (g8_board->vinetic[j]->rtp_channels[i])
						vinetic_rtp_channel_unregister(g8_board->vinetic[j]->rtp_channels[i]);
				}
				vinetic_device_unregister(g8_board->vinetic[j]);
			}
		}
		if (g8_board->pg_board) polygator_board_unregister(g8_board->pg_board);
		kfree(g8_board);
	}
	if (g8_tty_at_driver) {
		tty_unregister_driver(g8_tty_at_driver);
		put_tty_driver(g8_tty_at_driver);
	}
	if (g8_cs3_iomem_reg) release_mem_region(AT91_CHIPSELECT_3, 0x10000);
	if (g8_cs3_base_ptr) iounmap(g8_cs3_base_ptr);
	if (g8_cs4_iomem_reg) release_mem_region(AT91_CHIPSELECT_4, 0x10000);
	if (g8_cs4_base_ptr) iounmap(g8_cs4_base_ptr);
	return rc;
}

static void __exit g8_exit(void)
{
	size_t i, j;

	if (g8_board) {
		for (i=0; i<8; i++)
		{
			if (g8_board->tty_at_channels[i]) {
				del_timer_sync(&g8_board->tty_at_channels[i]->poll_timer);
				if (g8_board->tty_at_channels[i]->device)
					tty_unregister_device(g8_tty_at_driver, g8_board->tty_at_channels[i]->tty_at_minor);
				kfree(g8_board->tty_at_channels[i]);
			}
		}
		for (j=0; j<2; j++)
		{
			if (g8_board->vinetic[j]) {
				for (i=0; i<4; i++)
				{
					if (g8_board->vinetic[j]->rtp_channels[i])
						vinetic_rtp_channel_unregister(g8_board->vinetic[j]->rtp_channels[i]);
				}
				vinetic_device_unregister(g8_board->vinetic[j]);
			}
		}
		if (g8_board->pg_board) polygator_board_unregister(g8_board->pg_board);
		kfree(g8_board);
	}

	tty_unregister_driver(g8_tty_at_driver);
	put_tty_driver(g8_tty_at_driver);

	iounmap(g8_cs3_base_ptr);
	release_mem_region(AT91_CHIPSELECT_3, 0x10000);
	iounmap(g8_cs4_base_ptr);
	release_mem_region(AT91_CHIPSELECT_4, 0x10000);

	verbose("stopped\n");
}

module_init(g8_init);
module_exit(g8_exit);

/******************************************************************************/
/* end of g8-base.c                                                           */
/******************************************************************************/