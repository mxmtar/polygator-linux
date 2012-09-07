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
#define MB_CS_ROM_KROSS			0x4000
#define MB_RESET_ROM			0x4800
/*! */

union g20_at_ch_status_reg {
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

union g20_at_ch_control_reg {
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

struct g20_tty_at_data {

	int gsm_mod_type;
	size_t pos_on_board;

	uintptr_t cbdata;

	union g20_at_ch_status_reg status;
	union g20_at_ch_control_reg control;

	struct timer_list poll_timer;

	spinlock_t lock;

	size_t xmit_count;
	size_t xmit_head;
	size_t xmit_tail;

	struct tty_port port;

	void (* mod_control)(uintptr_t cbdata, size_t pos, u_int8_t reg);
	u_int8_t (* mod_status)(uintptr_t cbdata, size_t pos);
	void (* mod_at_write)(uintptr_t cbdata, size_t pos, u_int8_t reg);
	u_int8_t (* mod_at_read)(uintptr_t cbdata, size_t pos);
};

struct g20_board {

	size_t index;

	struct polygator_board *pg_board;
	struct cdev cdev;

	char rom[256];

	struct vinetic *vinetic;

	struct polygator_tty_device *tty_at_channels[4];
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
// 	.chars_in_buffer = g20_tty_at_chars_in_buffer,
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
static void g20_vinetic_write_nwd(uintptr_t cbdata, u_int16_t value)
{
	uintptr_t addr;

	addr = (uintptr_t)g20_cs4_base_ptr;
	addr += cbdata + G20_CS_VINETIC + 0x04;
	iowrite16(value, addr);
// 	log(KERN_INFO, "%08lx: %04x\n", addr, value);
}
static void g20_vinetic_write_eom(uintptr_t cbdata, u_int16_t value)
{
	uintptr_t addr;

	addr = (uintptr_t)g20_cs4_base_ptr;
	addr += cbdata + G20_CS_VINETIC + 0x06;
	iowrite16(value, addr);
// 	log(KERN_INFO, "%08lx: %04x\n", addr, value);
}
static u_int16_t g20_vinetic_read_nwd(uintptr_t cbdata)
{
	u_int16_t value;
	uintptr_t addr;

	addr = (uintptr_t)g20_cs4_base_ptr;
	addr += cbdata + G20_CS_VINETIC + 0x04;
	value = ioread16(addr);
// 	log(KERN_INFO, "%08lx: %04x\n", addr, value);
	return value;
}
static u_int16_t g20_vinetic_read_eom(uintptr_t cbdata)
{
	u_int16_t value;
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
static u_int16_t g20_vinetic_read_dia(uintptr_t cbdata)
{
	u_int16_t value;
	uintptr_t addr;

	addr = (uintptr_t)g20_cs4_base_ptr;
	addr += cbdata + G20_CS_VINETIC + 0x18;
	value = ioread16(addr);
	return value;
}

static void g20_mod_control(uintptr_t cbdata, size_t pos, u_int8_t reg)
{
	uintptr_t addr = cbdata;

	iowrite8(reg, addr);
}

static u_int8_t g20_mod_status(uintptr_t cbdata, size_t pos)
{
	uintptr_t addr = cbdata;

	return ioread8(addr);
}

static void g20_mod_at_write(uintptr_t cbdata, size_t pos, u_int8_t reg)
{
	uintptr_t addr = cbdata;

	addr += 0x10;

	iowrite8(reg, addr);
}

static u_int8_t g20_mod_at_read(uintptr_t cbdata, size_t pos)
{
	uintptr_t addr = cbdata;

	addr += 0x10;

	return ioread8(addr);
}

static void g20_tty_at_poll(unsigned long addr)
{
	char buff[512];
	size_t len;
	struct tty_struct *tty;
	struct g20_tty_at_data *at = (struct g20_tty_at_data *)addr;

	len = 0;

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

	spin_lock(&at->lock);
	while (at->xmit_count)
	{
		// read status register
		at->status.full = at->mod_status(at->cbdata, at->pos_on_board);
		// check for transmitter is ready
		if (!at->status.bits.at_wr_empty)
			break;
		// put char to transmitter
// 		verbose("test=%lu head=%lu tail=%lu %c\n", (unsigned long int)at->xmit_count, (unsigned long int)at->xmit_head, (unsigned long int)at->xmit_tail, *(at->port.xmit_buf + at->xmit_tail));
// 		at->mod_at_write(at->cbdata, at->pos_on_board, *(at->port.xmit_buf + at->xmit_tail));
		at->mod_at_write(at->cbdata, at->pos_on_board, at->port.xmit_buf[at->xmit_tail]);
		at->xmit_tail++;
		if (at->xmit_tail == SERIAL_XMIT_SIZE)
			at->xmit_tail = 0;
		at->xmit_count--;
	}
	spin_unlock(&at->lock);

	if (len) {
		tty = tty_port_tty_get(&at->port);
		tty_insert_flip_string(tty, buff, len);
		tty_flip_buffer_push(tty);
		tty_kref_put(tty);
	}

	mod_timer(&at->poll_timer, jiffies + 1);
}

static int g20_board_open(struct inode *inode, struct file *filp)
{
	ssize_t res;
	size_t i;
	size_t len;

	struct g20_board *brd;
	struct g20_board_private_data *private_data;
	struct g20_tty_at_data *tty_at_data;

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
			tty_at_data = (struct g20_tty_at_data *)brd->tty_at_channels[i]->data;
			tty_at_data->status.full = tty_at_data->mod_status(tty_at_data->cbdata, tty_at_data->pos_on_board);
			len += sprintf(private_data->buff+len, "GSM%lu tty%d %s VIN0ALM%lu VIO=%u\r\n",
															(unsigned long int)i, brd->tty_at_channels[i]->tty_minor, polygator_print_gsm_module_type(tty_at_data->gsm_mod_type),
															(unsigned long int)i, tty_at_data->status.bits.status);
		}
	}
	if (brd->vinetic) {
		len += sprintf(private_data->buff+len, "VIN0 board-g20-%lu-vin0\r\n", (unsigned long int)brd->index);
		for (i=0; i<4; i++)
		{
			if (brd->vinetic->rtp_channels[i])
				len += sprintf(private_data->buff+len, "VIN0RTP%lu board-g20-%lu-vin0-rtp%lu\r\n", (unsigned long int)i, (unsigned long int)brd->index, (unsigned long int)i);
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

	u_int32_t at_chan;
	u_int32_t pwr_state;
	u_int32_t key_state;
	u_int32_t baudrate;
	struct g20_tty_at_data *tty_at_data;
	struct g20_board_private_data *private_data = filp->private_data;

	memset(cmd, 0, sizeof(cmd));
	len = sizeof(cmd) - 1;
	len = min(len,count);

	if (copy_from_user(cmd, buff, len)) {
		res = -EINVAL;
		goto g20_board_write_end;
	}

	if (sscanf(cmd, "GSM%u PWR=%u", &at_chan, &pwr_state) == 2) {
		if ((at_chan >= 0) && (at_chan <= 3) && (private_data->board->tty_at_channels[at_chan])) {
			tty_at_data = (struct g20_tty_at_data *)private_data->board->tty_at_channels[at_chan]->data;
			tty_at_data->control.bits.vbat = !pwr_state;
			tty_at_data->mod_control(tty_at_data->cbdata, at_chan, tty_at_data->control.full);
			res = len;
		} else
			res= -ENODEV;
	} else if (sscanf(cmd, "GSM%u KEY=%u", &at_chan, &key_state) == 2) {
		if ((at_chan >= 0) && (at_chan <= 3) && (private_data->board->tty_at_channels[at_chan])) {
			tty_at_data = (struct g20_tty_at_data *)private_data->board->tty_at_channels[at_chan]->data;
			tty_at_data->control.bits.pkey = !key_state;
			tty_at_data->mod_control(tty_at_data->cbdata, at_chan, tty_at_data->control.full);
			res = len;
		} else
			res= -ENODEV;
	} else if (sscanf(cmd, "GSM%u BAUDRATE=%u", &at_chan, &baudrate) == 2) {
		if ((at_chan >= 0) && (at_chan <= 3) && (private_data->board->tty_at_channels[at_chan])) {
			tty_at_data = (struct g20_tty_at_data *)private_data->board->tty_at_channels[at_chan]->data;
			if (baudrate == 9600)
				tty_at_data->control.bits.at_baudrate = 0;
			else
				tty_at_data->control.bits.at_baudrate = 2;
			tty_at_data->mod_control(tty_at_data->cbdata, at_chan, tty_at_data->control.full);
			res = len;
		} else
			res= -ENODEV;
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
	struct g20_tty_at_data *at = (struct g20_tty_at_data *)ptd->data;

	return tty_port_open(&at->port, tty, filp);
}

static void g20_tty_at_close(struct tty_struct *tty, struct file *filp)
{
	struct polygator_tty_device *ptd = tty->driver_data;
	struct g20_tty_at_data *at = (struct g20_tty_at_data *)ptd->data;

	tty_port_close(&at->port, tty, filp);
	return;
}

static int g20_tty_at_write(struct tty_struct *tty, const unsigned char *buf, int count)
{
	int res = 0;
	size_t len;
	struct polygator_tty_device *ptd = tty->driver_data;
	struct g20_tty_at_data *at = (struct g20_tty_at_data *)ptd->data;

	spin_lock_bh(&at->lock);

	if (at->xmit_count < SERIAL_XMIT_SIZE) {
		while (1)
		{
			if (at->xmit_head == at->xmit_tail) {
				if (at->xmit_count)
					len = 0;
				else
					len = SERIAL_XMIT_SIZE - at->xmit_head;
			} else if (at->xmit_head > at->xmit_tail)
				len = SERIAL_XMIT_SIZE - at->xmit_head;
			else
				len = at->xmit_tail - at->xmit_head;

			len = min(len, (size_t)count);
			if (!len)
				break;

			memcpy(at->port.xmit_buf + at->xmit_head, buf, len);
			at->xmit_head += len;
			if (at->xmit_head == SERIAL_XMIT_SIZE)
				at->xmit_head = 0;
			at->xmit_count += len;
			buf += len;
			count -= len;
			res += len;
		}
	}

	spin_unlock_bh(&at->lock);
	
	return res;
}

static int g20_tty_at_write_room(struct tty_struct *tty)
{
	int res;
	struct polygator_tty_device *ptd = tty->driver_data;
	struct g20_tty_at_data *at = (struct g20_tty_at_data *)ptd->data;

	spin_lock_bh(&at->lock);

	res = SERIAL_XMIT_SIZE - at->xmit_count;

	spin_unlock_bh(&at->lock);
	
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
	struct g20_tty_at_data *at = (struct g20_tty_at_data *)ptd->data;

	baud = tty_get_baud_rate(tty);

	switch (baud)
	{
		case 9600:
			at->control.bits.at_baudrate = 0;
			break;
		default:
			at->control.bits.at_baudrate = 2;
			break;
	}
	
	at->mod_control(at->cbdata, at->pos_on_board, at->control.full);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
	tty_encode_baud_rate(tty, baud, baud);
#endif
}

static void g20_tty_at_flush_buffer(struct tty_struct *tty)
{
	struct polygator_tty_device *ptd = tty->driver_data;
	struct g20_tty_at_data *at = (struct g20_tty_at_data *)ptd->data;

	spin_lock_bh(&at->lock);
	at->xmit_count = at->xmit_head = at->xmit_tail = 0;
	spin_unlock_bh(&at->lock);
	tty_wakeup(tty);
}

static void g20_tty_at_hangup(struct tty_struct *tty)
{
	struct polygator_tty_device *ptd = tty->driver_data;
	struct g20_tty_at_data *at = (struct g20_tty_at_data *)ptd->data;
	
	tty_port_hangup(&at->port);
}

static int g20_tty_at_port_carrier_raised(struct tty_port *port)
{
	return 1;
}

static void g20_tty_at_port_dtr_rts(struct tty_port *port, int onoff)
{
}

static int g20_tty_at_port_activate(struct tty_port *port, struct tty_struct *tty)
{
	struct g20_tty_at_data *at = container_of(port, struct g20_tty_at_data, port);

	if (tty_port_alloc_xmit_buf(port) < 0)
		return -ENOMEM;
	at->xmit_count = at->xmit_head = at->xmit_tail = 0;

	at->poll_timer.function = g20_tty_at_poll;
	at->poll_timer.data = (unsigned long)at;
	at->poll_timer.expires = jiffies + 1;
	add_timer(&at->poll_timer);

	return 0;
}

static void g20_tty_at_port_shutdown(struct tty_port *port)
{
	struct g20_tty_at_data *at = container_of(port, struct g20_tty_at_data, port);

	del_timer_sync(&at->poll_timer);

	tty_port_free_xmit_buf(port);
}

static int __init g20_init(void)
{
	size_t i, k;
	u32 data;
	u8 modtype;
	char devname[VINETIC_DEVNAME_MAXLEN];
	struct g20_tty_at_data *tty_at_data;
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
				if (!(tty_at_data = kmalloc(sizeof(struct g20_tty_at_data), GFP_KERNEL))) {
					log(KERN_ERR, "can't get memory for struct g20_tty_at_data\n");
					rc = -1;
					goto g20_init_error;
				}
				memset(tty_at_data, 0, sizeof(struct g20_tty_at_data));
				// select GSM module type
				switch (modtype)
				{
					case 4:
					case 5:
						tty_at_data->gsm_mod_type = POLYGATOR_MODULE_TYPE_M10;
						break;
					case 6:
						tty_at_data->gsm_mod_type = POLYGATOR_MODULE_TYPE_SIM5215;
						break;
					case 7:
						tty_at_data->gsm_mod_type = POLYGATOR_MODULE_TYPE_SIM900;
						break;
					default:
						verbose("unsupported GSM module type=%u\n", modtype);
						break;
				}
				spin_lock_init(&tty_at_data->lock);
				tty_at_data->control.bits.vbat = 1;
				tty_at_data->control.bits.pkey = 1;
				tty_at_data->control.bits.cn_speed_a = 0;
				tty_at_data->control.bits.cn_speed_b = 0;
				tty_at_data->control.bits.at_baudrate = 2;

				tty_at_data->pos_on_board = i;
				tty_at_data->cbdata = (((uintptr_t)g20_cs3_base_ptr) + 0x1000 + (0x0200 * k) + (0x40 * (i%4)));
				tty_at_data->mod_control = g20_mod_control;
				tty_at_data->mod_status = g20_mod_status;
				tty_at_data->mod_at_write = g20_mod_at_write;
				tty_at_data->mod_at_read = g20_mod_at_read;

				tty_port_init(&tty_at_data->port);
				tty_at_data->port.ops = &g20_tty_at_port_ops;
				tty_at_data->port.close_delay = 0;
				tty_at_data->port.closing_wait = ASYNC_CLOSING_WAIT_NONE;

				// register polygator tty at device
				if (!(g20_boards[k]->tty_at_channels[i] = polygator_tty_device_register(&g20_tty_at_ops))) {
					log(KERN_ERR, "can't register polygator tty device\n");
					kfree(tty_at_data);
					rc = -1;
					goto g20_init_error;
				}
				g20_boards[k]->tty_at_channels[i]->data = tty_at_data;

				tty_at_data->mod_control(tty_at_data->cbdata, tty_at_data->pos_on_board, tty_at_data->control.full);
				init_timer(&tty_at_data->poll_timer);
			}
		}
	}

	verbose("loaded successfull\n");
	return rc;

g20_init_error:
	for (k=0; k<4; k++)
	{
		if (g20_boards[k]) {
			for (i=0; i<4; i++)
			{
				if (g20_boards[k]->tty_at_channels[i]) {
					tty_at_data = (struct g20_tty_at_data *)g20_boards[k]->tty_at_channels[i]->data;
					if (tty_at_data) {
						del_timer_sync(&tty_at_data->poll_timer);
						kfree(tty_at_data);
					}
					polygator_tty_device_unregister(g20_boards[k]->tty_at_channels[i]);
				}
			}
			if (g20_boards[k]->vinetic) {
				for (i=0; i<4; i++)
				{
					if (g20_boards[k]->vinetic->rtp_channels[i])
						vinetic_rtp_channel_unregister(g20_boards[k]->vinetic->rtp_channels[i]);
				}
				vinetic_device_unregister(g20_boards[k]->vinetic);
			}
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
	struct g20_tty_at_data *tty_at_data;

	for (k=0; k<4; k++)
	{
		if (g20_boards[k]) {
			for (i=0; i<4; i++)
			{
				tty_at_data = (struct g20_tty_at_data *)g20_boards[k]->tty_at_channels[i]->data;
				del_timer_sync(&tty_at_data->poll_timer);
				kfree(tty_at_data);
				polygator_tty_device_unregister(g20_boards[k]->tty_at_channels[i]);
			}
			for (i=0; i<4; i++)
				vinetic_rtp_channel_unregister(g20_boards[k]->vinetic->rtp_channels[i]);
			vinetic_device_unregister(g20_boards[k]->vinetic);
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