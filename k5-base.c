/******************************************************************************/
/* k5-base.c                                                                  */
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
MODULE_DESCRIPTION("Polygator Linux module for K5 device");
MODULE_LICENSE("GPL");

static int tty_at_major = 0;
module_param(tty_at_major, int, 0);
MODULE_PARM_DESC(tty_at_major, "Major number for AT-command channel of Polygator K5 device");

#define verbose(_fmt, _args...) printk(KERN_INFO "[polygator-%s] " _fmt, THIS_MODULE->name, ## _args)
#define log(_level, _fmt, _args...) printk(_level "[polygator-%s] %s:%d - %s(): " _fmt, THIS_MODULE->name, "k5-base.c", __LINE__, __PRETTY_FUNCTION__, ## _args)
#define debug(_fmt, _args...) printk(KERN_DEBUG "[polygator-%s] %s:%d - %s(): " _fmt, THIS_MODULE->name, "k5-base.c", __LINE__, __PRETTY_FUNCTION__, ## _args)

/*! */
#define K5_CS_STATUS_1	0x10C0
#define K5_CS_AT_COM_1	0x10D0
#define K5_CS_SWITCH	0x1170
#define K5_MODE_AUTONOM	0x1190
#define K5_RESET_BOARD	0x11F0
#define K5_CS_STATUS_2	0x1200
#define K5_CS_AT_COM_2	0x1210
/*! */

#define K5_TTY_AT_DEVICE_MAXCOUNT 2

union k5_at_ch_status_reg {
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

union k5_at_ch_control_reg {
	struct {
		u_int8_t vbat:1; // 0 - disable, 1 - enable
		u_int8_t pkey:1;
		u_int8_t gap:2;
		u_int8_t cn_speed_a:1;
		u_int8_t cn_speed_b:1;
		u_int8_t at_baudrate:2;
	} __attribute__((packed)) bits;
	u_int8_t full;
} __attribute__((packed));

struct k5_tty_at_channel {

	int gsm_mod_type;
	size_t pos_on_board;

	union k5_at_ch_status_reg status;
	union k5_at_ch_control_reg control;

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

struct k5_board {

	struct polygator_board *pg_board;
	struct cdev cdev;

	struct vinetic *vinetic;

	struct k5_tty_at_channel *tty_at_channels[2];
};

struct k5_board_private_data {
	struct k5_board *board;
	char buff[0x0C00];
	size_t length;
};

static struct tty_driver *k5_tty_at_driver = NULL;

static int k5_tty_at_open(struct tty_struct *tty, struct file *filp);
static void k5_tty_at_close(struct tty_struct *tty, struct file *filp);
static int k5_tty_at_write(struct tty_struct *tty, const unsigned char *buf, int count);
static int k5_tty_at_write_room(struct tty_struct *tty);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20)
static void k5_tty_at_set_termios(struct tty_struct *tty, struct ktermios *old_termios);
#else
static void k5_tty_at_set_termios(struct tty_struct *tty, struct termios *old_termios);
#endif
static void k5_tty_at_flush_buffer(struct tty_struct *tty);
static void k5_tty_at_hangup(struct tty_struct *tty);

static struct tty_operations k5_tty_at_ops = {
	.open = k5_tty_at_open,
	.close = k5_tty_at_close,
	.write = k5_tty_at_write,
	.write_room = k5_tty_at_write_room,
// 	.chars_in_buffer = k5_tty_at_chars_in_buffer,
	.set_termios = k5_tty_at_set_termios,
	.flush_buffer = k5_tty_at_flush_buffer,
	.hangup = k5_tty_at_hangup,
};

static int k5_tty_at_port_carrier_raised(struct tty_port *port);
static void k5_tty_at_port_dtr_rts(struct tty_port *port, int onoff);
static int k5_tty_at_port_activate(struct tty_port *tport, struct tty_struct *tty);
static void k5_tty_at_port_shutdown(struct tty_port *port);

static const struct tty_port_operations k5_tty_at_port_ops = {
	.carrier_raised =k5_tty_at_port_carrier_raised,
	.dtr_rts = k5_tty_at_port_dtr_rts,
	.activate = k5_tty_at_port_activate,
	.shutdown = k5_tty_at_port_shutdown,
};

static struct k5_board *k5_board = NULL;

static struct resource * k5_cs3_iomem_reg = NULL;
static struct resource * k5_cs4_iomem_reg = NULL;

static void __iomem * k5_cs3_base_ptr = NULL;
static void __iomem * k5_cs4_base_ptr = NULL;

static void k5_vinetic_reset(uintptr_t cbdata)
{
	iowrite16(0, cbdata + 0x20);
	mdelay(10);
	iowrite16(1, cbdata + 0x20);
	mdelay(2);
}
static void k5_vinetic_write_nwd(uintptr_t cbdata, u_int16_t value)
{
	iowrite16(value, cbdata + 0x04);
// 	log(KERN_INFO, "%08lx: %04x\n", cbdata + 0x04, value);
}
static void k5_vinetic_write_eom(uintptr_t cbdata, u_int16_t value)
{
	iowrite16(value, cbdata + 0x06);
// 	log(KERN_INFO, "%08lx: %04x\n", cbdata + 0x06, value);
}
static u_int16_t k5_vinetic_read_nwd(uintptr_t cbdata)
{
	u_int16_t value = ioread16(cbdata + 0x04);
// 	log(KERN_INFO, "%08lx: %04x\n", cbdata + 0x04, value);
	return value;
}
static u_int16_t k5_vinetic_read_eom(uintptr_t cbdata)
{
	u_int16_t value = ioread16(cbdata + 0x06);
// 	log(KERN_INFO, "%08lx: %04x\n", cbdata + 0x06, value);
	return value;
}
static size_t k5_vinetic_is_not_ready(uintptr_t cbdata)
{
	union vin_reg_ir reg_ir;
	reg_ir.full = ioread16(cbdata + 0x18);
// 	log(KERN_INFO, "%08lx: %04x\n", cbdata + 0x18, reg_ir.full);
	return reg_ir.bits.rdyq;
}
static u_int16_t k5_vinetic_read_dia(uintptr_t cbdata)
{
	u_int16_t value = ioread16(cbdata + 0x18);
// 	log(KERN_INFO, "%08lx: %04x\n", cbdata + 0x18, value);
	return value;
}

static void k5_mod_control(uintptr_t cbdata, size_t pos, u_int8_t reg)
{
	uintptr_t addr = cbdata;
	if (pos)
		addr += K5_CS_STATUS_2;
	else
		addr += K5_CS_STATUS_1;
	iowrite8(reg, addr);
}

static u_int8_t k5_mod_status(uintptr_t cbdata, size_t pos)
{
	uintptr_t addr = cbdata;
	if (pos)
		addr += K5_CS_STATUS_2;
	else
		addr += K5_CS_STATUS_1;
	return ioread8(addr);
}

static void k5_mod_at_write(uintptr_t cbdata, size_t pos, u_int8_t reg)
{
	uintptr_t addr = cbdata;
	if (pos)
		addr += K5_CS_AT_COM_2;
	else
		addr += K5_CS_AT_COM_1;
	iowrite8(reg, addr);
}

static u_int8_t k5_mod_at_read(uintptr_t cbdata, size_t pos)
{
	uintptr_t addr = cbdata;
	if (pos)
		addr += K5_CS_AT_COM_2;
	else
		addr += K5_CS_AT_COM_1;
	return ioread8(addr);
}

static void k5_tty_at_poll(unsigned long addr)
{
	char buff[512];
	size_t len;
	struct tty_struct *tty;
	struct k5_tty_at_channel *ch = (struct k5_tty_at_channel *)addr;

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

static int k5_board_open(struct inode *inode, struct file *filp)
{
	ssize_t res;
	size_t i;
	size_t len;

	struct k5_board *brd;
	struct k5_board_private_data *private_data;

	brd = container_of(inode->i_cdev, struct k5_board, cdev);

	if (!(private_data = kmalloc(sizeof(struct k5_board_private_data), GFP_KERNEL))) {
		log(KERN_ERR, "can't get memory=%lu bytes\n", (unsigned long int)sizeof(struct k5_board_private_data));
		res = -ENOMEM;
		goto k5_open_error;
	}
	private_data->board = brd;

	len = 0;
	for (i=0; i<2; i++)
	{
		if (brd->tty_at_channels[i]) {
			brd->tty_at_channels[i]->status.full = brd->tty_at_channels[i]->mod_status(brd->tty_at_channels[i]->cbdata, brd->tty_at_channels[i]->pos_on_board);
			if (i)
				len += sprintf(private_data->buff+len, "GSM%lu k5AT%d %s VIN0PCM0 VIO=%u\r\n", (unsigned long int)i, brd->tty_at_channels[i]->tty_at_minor, polygator_print_gsm_module_type(brd->tty_at_channels[i]->gsm_mod_type), brd->tty_at_channels[i]->status.bits.status);
			else
				len += sprintf(private_data->buff+len, "GSM%lu k5AT%d %s VIN0ALM3 VIO=%u\r\n", (unsigned long int)i, brd->tty_at_channels[i]->tty_at_minor, polygator_print_gsm_module_type(brd->tty_at_channels[i]->gsm_mod_type), brd->tty_at_channels[i]->status.bits.status);
		}
	}
	if (brd->vinetic) {
		len += sprintf(private_data->buff+len, "VIN0 board-k5-vin0\r\n");
		for (i=0; i<4; i++)
		{
			if (brd->vinetic->rtp_channels[i])
				len += sprintf(private_data->buff+len, "VIN0RTP%lu board-k5-vin0-rtp%lu\r\n", (unsigned long int)i, (unsigned long int)i);
		}
	}

	private_data->length = len;

	filp->private_data = private_data;

	return 0;

k5_open_error:
	if (private_data) kfree(private_data);
	return res;
}

static int k5_board_release(struct inode *inode, struct file *filp)
{
	struct k5_board_private_data *private_data = filp->private_data;

	kfree(private_data);
	return 0;
}

static ssize_t k5_board_read(struct file *filp, char __user *buff, size_t count, loff_t *offp)
{
	size_t len;
	ssize_t res;
	struct k5_board_private_data *private_data = filp->private_data;

	res = (private_data->length > filp->f_pos)?(private_data->length - filp->f_pos):(0);

	if (res) {
		len = res;
		len = min(count, len);
		if (copy_to_user(buff, private_data->buff + filp->f_pos, len)) {
			res = -EINVAL;
			goto k5_board_read_end;
		}
		*offp = filp->f_pos + len;
	}

k5_board_read_end:
	return res;
}

static ssize_t k5_board_write(struct file *filp, const char __user *buff, size_t count, loff_t *offp)
{
	ssize_t res;
	char cmd[256];
	size_t len;

	u_int32_t at_chan;
	u_int32_t pwr_state;
	u_int32_t key_state;
	u_int32_t baudrate;
	struct k5_board_private_data *private_data = filp->private_data;

	memset(cmd, 0, sizeof(cmd));
	len = sizeof(cmd) - 1;
	len = min(len,count);

	if (copy_from_user(cmd, buff, len)) {
		res = -EINVAL;
		goto k5_board_write_end;
	}

	if (sscanf(cmd, "GSM%u PWR=%u", &at_chan, &pwr_state) == 2) {
		if (private_data->board->tty_at_channels[0]) {
			private_data->board->tty_at_channels[0]->control.bits.vbat = pwr_state;
			private_data->board->tty_at_channels[0]->mod_control(private_data->board->tty_at_channels[0]->cbdata, 0, private_data->board->tty_at_channels[0]->control.full);
			res = len;
		} else
			res= -ENODEV;
	} else if (sscanf(cmd, "GSM%u KEY=%u", &at_chan, &key_state) == 2) {
		if ((at_chan >= 0) && (at_chan <= 1) && (private_data->board->tty_at_channels[at_chan])) {
			private_data->board->tty_at_channels[at_chan]->control.bits.pkey = !key_state;
			private_data->board->tty_at_channels[at_chan]->mod_control(private_data->board->tty_at_channels[at_chan]->cbdata, at_chan, private_data->board->tty_at_channels[at_chan]->control.full);
			res = len;
		} else
			res= -ENODEV;
	} else if (sscanf(cmd, "GSM%u BAUDRATE=%u", &at_chan, &baudrate) == 2) {
		if ((at_chan >= 0) && (at_chan <= 1) && (private_data->board->tty_at_channels[at_chan])) {
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

k5_board_write_end:
	return res;
}

static struct file_operations k5_board_fops = {
	.owner   = THIS_MODULE,
	.open    = k5_board_open,
	.release = k5_board_release,
	.read    = k5_board_read,
	.write   = k5_board_write,
};

static int k5_tty_at_open(struct tty_struct *tty, struct file *filp)
{
	struct k5_tty_at_channel *ch;

	if ((tty->index != 0) && (tty->index != 1)) {
		return -ENODEV;
	}
	ch = k5_board->tty_at_channels[tty->index];
	tty->driver_data = ch;

	return tty_port_open(&ch->port, tty, filp);
}

static void k5_tty_at_close(struct tty_struct *tty, struct file *filp)
{
	struct k5_tty_at_channel *ch = tty->driver_data;

	tty_port_close(&ch->port, tty, filp);
	return;
}

static int k5_tty_at_write(struct tty_struct *tty, const unsigned char *buf, int count)
{
	int res = 0;
	size_t len;
	struct k5_tty_at_channel *ch = tty->driver_data;

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

static int k5_tty_at_write_room(struct tty_struct *tty)
{
	int res;
	struct k5_tty_at_channel *ch = tty->driver_data;

	spin_lock_bh(&ch->lock);

	res = SERIAL_XMIT_SIZE - ch->xmit_count;

	spin_unlock_bh(&ch->lock);
	
	return res;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20)
static void k5_tty_at_set_termios(struct tty_struct *tty, struct ktermios *old_termios)
#else
static void k5_tty_at_set_termios(struct tty_struct *tty, struct termios *old_termios)
#endif
{
	speed_t baud;
	struct k5_tty_at_channel *ch = tty->driver_data;

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

static void k5_tty_at_flush_buffer(struct tty_struct *tty)
{
	struct k5_tty_at_channel *ch = tty->driver_data;

	spin_lock_bh(&ch->lock);
	ch->xmit_count = ch->xmit_head = ch->xmit_tail = 0;
	spin_unlock_bh(&ch->lock);
	tty_wakeup(tty);
}

static void k5_tty_at_hangup(struct tty_struct *tty)
{
	struct k5_tty_at_channel *ch = tty->driver_data;
	tty_port_hangup(&ch->port);
}

static int k5_tty_at_port_carrier_raised(struct tty_port *port)
{
	return 1;
}

static void k5_tty_at_port_dtr_rts(struct tty_port *port, int onoff)
{
}

static int k5_tty_at_port_activate(struct tty_port *port, struct tty_struct *tty)
{
	struct k5_tty_at_channel *ch = container_of(port, struct k5_tty_at_channel, port);

	if (tty_port_alloc_xmit_buf(port) < 0)
		return -ENOMEM;
	ch->xmit_count = ch->xmit_head = ch->xmit_tail = 0;

	ch->poll_timer.function = k5_tty_at_poll;
	ch->poll_timer.data = (unsigned long)ch;
	ch->poll_timer.expires = jiffies + 1;
	add_timer(&ch->poll_timer);

	return 0;
}

static void k5_tty_at_port_shutdown(struct tty_port *port)
{
	struct k5_tty_at_channel *ch = container_of(port, struct k5_tty_at_channel, port);

	del_timer_sync(&ch->poll_timer);

	tty_port_free_xmit_buf(port);
}

static int __init k5_init(void)
{
	size_t i;
	u32 data;
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
		goto k5_init_error;
	}
	if (!(k5_cs3_iomem_reg = request_mem_region(AT91_CHIPSELECT_3, 0x10000, "polygator_k5"))) {
		log(KERN_ERR, "can't request i/o memory region for cs3\n");
		rc = -ENOMEM;
		goto k5_init_error;
	}
	if (!(k5_cs3_base_ptr = ioremap_nocache(AT91_CHIPSELECT_3, 0x10000))) {
		log(KERN_ERR, "can't remap i/o memory for cs3\n");
		rc = -ENOMEM;
		goto k5_init_error;
	}

	// Request and remap i/o memory region for cs4
	if (check_mem_region(AT91_CHIPSELECT_4, 0x10000)) {
		log(KERN_ERR, "i/o memory region for cs4 already used\n");
		rc = -ENOMEM;
		goto k5_init_error;
	}
	if (!(k5_cs4_iomem_reg = request_mem_region(AT91_CHIPSELECT_4, 0x10000, "polygator_k5"))) {
		log(KERN_ERR, "can't request i/o memory region for cs4\n");
		rc = -ENOMEM;
		goto k5_init_error;
	}
	if (!(k5_cs4_base_ptr = ioremap_nocache(AT91_CHIPSELECT_4, 0x10000))) {
		log(KERN_ERR, "can't remap i/o memory for cs4\n");
		rc = -ENOMEM;
		goto k5_init_error;
	}

	// registering tty device for AT-command channel
	k5_tty_at_driver = alloc_tty_driver(K5_TTY_AT_DEVICE_MAXCOUNT);
	if (!k5_tty_at_driver) {
		log(KERN_ERR, "can't allocated memory for tty driver\n");
		return -ENOMEM;
	}

	k5_tty_at_driver->owner = THIS_MODULE;
	k5_tty_at_driver->driver_name = "k5_tty_at";
	k5_tty_at_driver->name = "polygator/k5AT";
	k5_tty_at_driver->major = tty_at_major;
	k5_tty_at_driver->minor_start = 0;
	k5_tty_at_driver->type = TTY_DRIVER_TYPE_SERIAL;
	k5_tty_at_driver->subtype = SERIAL_TYPE_NORMAL;
	k5_tty_at_driver->init_termios = tty_std_termios;
	k5_tty_at_driver->init_termios.c_iflag &= ~ICRNL;
	k5_tty_at_driver->init_termios.c_cflag = B9600 | CS8 | HUPCL | CLOCAL | CREAD;
	k5_tty_at_driver->init_termios.c_lflag &= ~ECHO;
	k5_tty_at_driver->init_termios.c_ispeed = 9600;
	k5_tty_at_driver->init_termios.c_ospeed = 9600;
	k5_tty_at_driver->flags = TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;
	tty_set_operations(k5_tty_at_driver, &k5_tty_at_ops);

	if ((rc = tty_register_driver(k5_tty_at_driver))) {
		log(KERN_ERR, "can't register k5_tty_at driver: rc=%d\n", rc);
		// release allocated tty driver environment
		put_tty_driver(k5_tty_at_driver);
		k5_tty_at_driver = NULL;
		goto k5_init_error;
	}
	debug("tty_at_major=%d\n", k5_tty_at_driver->major);

	// Reset K5 board
	iowrite8(0, k5_cs3_base_ptr + K5_RESET_BOARD);
	mdelay(10);
	iowrite8(1, k5_cs3_base_ptr + K5_RESET_BOARD);
	// Set AUTONOM SIM CARD
	iowrite8(1, k5_cs3_base_ptr + K5_MODE_AUTONOM);
	// Set audio path to channel 1 M10
	iowrite8(0, k5_cs3_base_ptr + K5_CS_SWITCH);

	// alloc memory for board data
	if (!(k5_board = kmalloc(sizeof(struct k5_board), GFP_KERNEL))) {
		log(KERN_ERR, "can't get memory for struct K5_board\n");
		rc = -1;
		goto k5_init_error;
	}
	memset(k5_board, 0, sizeof(struct k5_board));

	snprintf(devname, POLYGATOR_BRDNAME_MAXLEN, "board-k5");
	if (!(k5_board->pg_board =  polygator_board_register(THIS_MODULE, devname, &k5_board->cdev, &k5_board_fops))) {
		rc = -1;
		goto k5_init_error;
	}
	// Register vinetic
	if (!(k5_board->vinetic = vinetic_device_register(THIS_MODULE, "board-k5-vin0", (uintptr_t)k5_cs4_base_ptr,
													k5_vinetic_reset,
													k5_vinetic_is_not_ready,
													k5_vinetic_write_nwd,
													k5_vinetic_write_eom,
													k5_vinetic_read_nwd,
													k5_vinetic_read_eom,
													k5_vinetic_read_dia))) {
		rc = -1;
		goto k5_init_error;
	}
	for (i=0; i<4; i++)
	{
		snprintf(devname, VINETIC_DEVNAME_MAXLEN, "board-k5-vin0-rtp%lu", (unsigned long int)i);
		if (!(vinetic_rtp_channel_register(THIS_MODULE, devname, k5_board->vinetic, i))) {
			rc = -1;
			goto k5_init_error;
		}
	}

	// set AT command channels
	for (i=0; i<2; i++)
	{
		if (!(k5_board->tty_at_channels[i] = kmalloc(sizeof(struct k5_tty_at_channel), GFP_KERNEL))) {
			log(KERN_ERR, "can't get memory for struct k5_tty_at_channel\n");
			rc = -1;
			goto k5_init_error;
		}
		memset(k5_board->tty_at_channels[i], 0, sizeof(struct k5_tty_at_channel));

		k5_board->tty_at_channels[i]->gsm_mod_type = (i)?(POLYGATOR_MODULE_TYPE_SIM5215):(POLYGATOR_MODULE_TYPE_M10);
		k5_board->tty_at_channels[i]->control.bits.vbat = 0;
		k5_board->tty_at_channels[i]->control.bits.pkey = 1;
		k5_board->tty_at_channels[i]->control.bits.cn_speed_a = 0;
		k5_board->tty_at_channels[i]->control.bits.cn_speed_b = 0;
		k5_board->tty_at_channels[i]->control.bits.at_baudrate = 2;

		spin_lock_init(&k5_board->tty_at_channels[i]->lock);
		tty_port_init(&k5_board->tty_at_channels[i]->port);
		k5_board->tty_at_channels[i]->port.ops = &k5_tty_at_port_ops;
		k5_board->tty_at_channels[i]->port.close_delay = 0;
		k5_board->tty_at_channels[i]->port.closing_wait = ASYNC_CLOSING_WAIT_NONE;

		k5_board->tty_at_channels[i]->pos_on_board = i;
		k5_board->tty_at_channels[i]->tty_at_minor = i;
		k5_board->tty_at_channels[i]->cbdata = (uintptr_t)k5_cs3_base_ptr;
		k5_board->tty_at_channels[i]->mod_control = k5_mod_control;
		k5_board->tty_at_channels[i]->mod_status = k5_mod_status;
		k5_board->tty_at_channels[i]->mod_at_write = k5_mod_at_write;
		k5_board->tty_at_channels[i]->mod_at_read = k5_mod_at_read;

		// register device on sysfs
		k5_board->tty_at_channels[i]->device = tty_register_device(k5_tty_at_driver, i, NULL);
		if (IS_ERR(k5_board->tty_at_channels[i]->device)) {
			log(KERN_ERR, "can't register tty device\n");
			rc = -1;
			goto k5_init_error;
		}

		k5_board->tty_at_channels[i]->mod_control(k5_board->tty_at_channels[i]->cbdata, k5_board->tty_at_channels[i]->pos_on_board, k5_board->tty_at_channels[i]->control.full);

		init_timer(&k5_board->tty_at_channels[i]->poll_timer);
	}

	verbose("loaded successfull\n");
	return rc;

k5_init_error:
	if (k5_board) {
		for (i=0; i<2; i++)
		{
			if (k5_board->tty_at_channels[i]) {
				del_timer_sync(&k5_board->tty_at_channels[i]->poll_timer);
				if (k5_board->tty_at_channels[i]->device)
					tty_unregister_device(k5_tty_at_driver, k5_board->tty_at_channels[i]->tty_at_minor);
				kfree(k5_board->tty_at_channels[i]);
			}
		}
		if (k5_board->vinetic) {
			for (i=0; i<4; i++)
			{
				if (k5_board->vinetic->rtp_channels[i])
					vinetic_rtp_channel_unregister(k5_board->vinetic->rtp_channels[i]);
			}
			vinetic_device_unregister(k5_board->vinetic);
		}
		if (k5_board->pg_board) polygator_board_unregister(k5_board->pg_board);
		kfree(k5_board);
	}
	if (k5_tty_at_driver) {
		tty_unregister_driver(k5_tty_at_driver);
		put_tty_driver(k5_tty_at_driver);
	}
	if (k5_cs3_iomem_reg) release_mem_region(AT91_CHIPSELECT_3, 0x10000);
	if (k5_cs3_base_ptr) iounmap(k5_cs3_base_ptr);
	if (k5_cs4_iomem_reg) release_mem_region(AT91_CHIPSELECT_4, 0x10000);
	if (k5_cs4_base_ptr) iounmap(k5_cs4_base_ptr);
	return rc;
}

static void __exit k5_exit(void)
{
	size_t i;

	for (i=0; i<2; i++)
	{
		del_timer_sync(&k5_board->tty_at_channels[i]->poll_timer);
		tty_unregister_device(k5_tty_at_driver, k5_board->tty_at_channels[i]->tty_at_minor);
		kfree(k5_board->tty_at_channels[i]);
	}
	for (i=0; i<4; i++)
		vinetic_rtp_channel_unregister(k5_board->vinetic->rtp_channels[i]);
	vinetic_device_unregister(k5_board->vinetic);
	polygator_board_unregister(k5_board->pg_board);
	kfree(k5_board);

	tty_unregister_driver(k5_tty_at_driver);
	put_tty_driver(k5_tty_at_driver);

	release_mem_region(AT91_CHIPSELECT_3, 0x10000);
	iounmap(k5_cs3_base_ptr);
	release_mem_region(AT91_CHIPSELECT_4, 0x10000);
	iounmap(k5_cs4_base_ptr);

	verbose("stopped\n");
}

module_init(k5_init);
module_exit(k5_exit);

/******************************************************************************/
/* end of k5-base.c                                                           */
/******************************************************************************/
