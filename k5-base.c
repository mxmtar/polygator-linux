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
// #include "../arch/arm/mach-at91/include/mach/io.h"
#include "../arch/arm/mach-at91/include/mach/at91_pio.h"
#include "../arch/arm/mach-at91/include/mach/at91sam9260_matrix.h"

#include "polygator/polygator-base.h"

#include "polygator/vinetic-base.h"
#include "polygator/vinetic-def.h"

#include "polygator/simcard-base.h"

#ifndef __ASSEMBLY__
static inline unsigned int at91_sys_read(unsigned int reg_offset)
{
	void __iomem *addr = (void __iomem *)AT91_VA_BASE_SYS;

	return __raw_readl(addr + reg_offset);
}
static inline void at91_sys_write(unsigned int reg_offset, unsigned long value)
{
	void __iomem *addr = (void __iomem *)AT91_VA_BASE_SYS;

	__raw_writel(value, addr + reg_offset);
}
#endif
#ifndef AT91_PIOC
#define AT91_PIOC	(AT91SAM9260_BASE_PIOC - AT91_BASE_SYS)
#endif
#ifndef AT91_SMC0
#undef AT91_SMC
#define AT91_SMC	(AT91SAM9260_BASE_SMC - AT91_BASE_SYS)
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,36) // 2,6,30 - orig
#define TTY_PORT
#endif

MODULE_AUTHOR("Maksym Tarasevych <mxmtar@gmail.com>");
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
#define K5_CS_SIM_COM_1	0x10E0
#define K5_CS_FXO		0x1130
#define K5_CS_SWITCH	0x1170
#define K5_MODE_AUTONOM	0x1190
#define K5_RESET_BOARD	0x11F0
#define K5_CS_STATUS_2	0x1200
#define K5_CS_AT_COM_2	0x1210
#define K5_CS_SIM_COM_2	0x1220
/*! */

#define K5_TTY_AT_DEVICE_MAXCOUNT 2

union k5_at_ch_status_reg {
	struct {
		uint8_t status:1;
		uint8_t at_rd_empty:1;
		uint8_t at_wr_empty:1;
		uint8_t sim_rd_empty:1;
		uint8_t sim_wr_empty:1;
		uint8_t sim_rst_req:1;
		uint8_t gap:2;
	} __attribute__((packed)) bits;
	uint8_t full;
} __attribute__((packed));

union k5_at_ch_control_reg {
	struct {
		uint8_t vbat:1; // 0 - disable, 1 - enable
		uint8_t pkey:1;
		uint8_t gap:2;
		uint8_t cn_speed_a:1;
		uint8_t cn_speed_b:1;
		uint8_t at_baudrate:2;
	} __attribute__((packed)) bits;
	uint8_t full;
} __attribute__((packed));

struct k5_gsm_module_data {

	int type;
	size_t pos_on_board;

	union k5_at_ch_control_reg control;

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
	int at_no_buf;
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

struct k5_board {

	struct polygator_board *pg_board;
	struct cdev cdev;

	uint8_t rom[256];
	size_t romsize;
	uint32_t sn;
	uint16_t type;

	struct vinetic *vinetic;

	struct k5_gsm_module_data *gsm_modules[2];

	struct polygator_tty_device *tty_at_channels[8];

	struct simcard_device *simcard_channels[8];
};

struct k5_board_private_data {
	struct k5_board *board;
	char buff[0x0C00];
	size_t length;
};

static int k5_tty_at_open(struct tty_struct *tty, struct file *filp);
static void k5_tty_at_close(struct tty_struct *tty, struct file *filp);
static int k5_tty_at_write(struct tty_struct *tty, const unsigned char *buf, int count);
static int k5_tty_at_write_room(struct tty_struct *tty);
static int k5_tty_at_chars_in_buffer(struct tty_struct *tty);
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
	.chars_in_buffer = k5_tty_at_chars_in_buffer,
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
	void __iomem *addr;

	addr = (void __iomem *)k5_cs3_base_ptr;
	addr += cbdata + 0x20;
	iowrite8(0, addr);
	mdelay(10);
	iowrite8(1, addr);
	mdelay(2);
}
static void k5_vinetic_write_nwd(uintptr_t cbdata, uint16_t value)
{
	void __iomem *addr;

	addr = (void __iomem *)k5_cs4_base_ptr;
	addr += cbdata + 0x04;
	iowrite16(value, addr);
// 	log(KERN_INFO, "%08lx: %04x\n", cbdata + 0x04, value);
}
static void k5_vinetic_write_eom(uintptr_t cbdata, uint16_t value)
{
	void __iomem *addr;

	addr = (void __iomem *)k5_cs4_base_ptr;
	addr += cbdata + 0x06;
	iowrite16(value, addr);
// 	log(KERN_INFO, "%08lx: %04x\n", cbdata + 0x06, value);
}
static uint16_t k5_vinetic_read_nwd(uintptr_t cbdata)
{
	uint16_t value;
	void __iomem *addr;

	addr = (void __iomem *)k5_cs4_base_ptr;
	addr += cbdata + 0x04;
	value = ioread16(addr);
// 	log(KERN_INFO, "%08lx: %04x\n", cbdata + 0x04, value);
	return value;
}
static uint16_t k5_vinetic_read_eom(uintptr_t cbdata)
{
	uint16_t value;
	void __iomem *addr;

	addr = (void __iomem *)k5_cs4_base_ptr;
	addr += cbdata + 0x06;
	value = ioread16(addr);
// 	log(KERN_INFO, "%08lx: %04x\n", cbdata + 0x06, value);
	return value;
}
static size_t k5_vinetic_is_not_ready(uintptr_t cbdata)
{
	union vin_reg_ir reg_ir;
	void __iomem *addr;

	addr = (void __iomem *)k5_cs4_base_ptr;
	addr += cbdata + 0x18;
	reg_ir.full = ioread16(addr);
// 	log(KERN_INFO, "%08lx: %04x\n", cbdata + 0x18, reg_ir.full);
	return reg_ir.bits.rdyq;
}
static uint16_t k5_vinetic_read_dia(uintptr_t cbdata)
{
	uint16_t value;
	void __iomem *addr;

	addr = (void __iomem *)k5_cs4_base_ptr;
	addr += cbdata + 0x18;
	value = ioread16(cbdata + 0x18);
// 	log(KERN_INFO, "%08lx: %04x\n", cbdata + 0x18, value);
	return value;
}

static void k5_gsm_mod_set_control(uintptr_t cbdata, size_t pos, uint8_t reg)
{
	void __iomem *addr;
	
	addr = (void __iomem *)cbdata;
	if (pos) {
		addr += K5_CS_STATUS_2;
	} else {
		addr += K5_CS_STATUS_1;
	}
	iowrite8(reg, addr);
}

static uint8_t k5_gsm_mod_get_status(uintptr_t cbdata, size_t pos)
{
	void __iomem *addr;
	
	addr = (void __iomem *)cbdata;
	if (pos) {
		addr += K5_CS_STATUS_2;
	} else {
		addr += K5_CS_STATUS_1;
	}
	return ioread8(addr);
}

static void k5_gsm_mod_at_write(uintptr_t cbdata, size_t pos, uint8_t reg)
{
	void __iomem *addr;
	
	addr = (void __iomem *)cbdata;
	if (pos) {
		addr += K5_CS_AT_COM_2;
	} else {
		addr += K5_CS_AT_COM_1;
	}
	iowrite8(reg, addr);
}

static uint8_t k5_gsm_mod_at_read(uintptr_t cbdata, size_t pos)
{
	void __iomem *addr;

	addr = (void __iomem *)cbdata;
	if (pos) {
		addr += K5_CS_AT_COM_2;
	} else {
		addr += K5_CS_AT_COM_1;
	}
	return ioread8(addr);
}

static void k5_gsm_mod_sim_write(uintptr_t cbdata, size_t pos, uint8_t reg)
{
	void __iomem *addr;

	addr = (void __iomem *)cbdata;
	if (pos) {
		addr += K5_CS_SIM_COM_2;
	} else {
		addr += K5_CS_SIM_COM_1;
	}
	iowrite8(reg, addr);
}

static uint8_t k5_gsm_mod_sim_read(uintptr_t cbdata, size_t pos)
{
	void __iomem *addr;

	addr = (void __iomem *)cbdata;
	if (pos) {
		addr += K5_CS_SIM_COM_2;
	} else {
		addr += K5_CS_SIM_COM_1;
	}
	return ioread8(addr);
}

static uint8_t k5_sim_read(void *data)
{
	struct k5_gsm_module_data *mod = (struct k5_gsm_module_data *)data;

	return mod->sim_read(mod->cbdata, mod->pos_on_board);
}

static void k5_sim_write(void *data, uint8_t value)
{
	struct k5_gsm_module_data *mod = (struct k5_gsm_module_data *)data;

	mod->sim_write(mod->cbdata, mod->pos_on_board, value);
}

static int k5_sim_is_read_ready(void *data)
{
	union k5_at_ch_status_reg status;
	struct k5_gsm_module_data *mod = (struct k5_gsm_module_data *)data;

	status.full = mod->get_status(mod->cbdata, mod->pos_on_board);

	return status.bits.sim_rd_empty;
}

static int k5_sim_is_write_ready(void *data)
{
	union k5_at_ch_status_reg status;
	struct k5_gsm_module_data *mod = (struct k5_gsm_module_data *)data;

	status.full = mod->get_status(mod->cbdata, mod->pos_on_board);

	return status.bits.sim_wr_empty;
}

static int k5_sim_is_reset_request(void *data)
{
	union k5_at_ch_status_reg status;
	struct k5_gsm_module_data *mod = (struct k5_gsm_module_data *)data;

	status.full = mod->get_status(mod->cbdata, mod->pos_on_board);

	return status.bits.sim_rst_req;
}

static void k5_sim_set_speed(void *data, int speed)
{
	struct k5_gsm_module_data *mod = (struct k5_gsm_module_data *)data;

	switch (speed) {
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

static void k5_tty_at_poll(unsigned long addr)
{
	char buff[512];
	size_t len;
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,9,0)
	struct tty_struct *tty;
#endif
	union k5_at_ch_status_reg status;
	struct k5_gsm_module_data *mod = (struct k5_gsm_module_data *)addr;

	len = 0;

	// read received data
	while (len < sizeof(buff)) {
		// read status register
		status.full = mod->get_status(mod->cbdata, mod->pos_on_board);
		if (status.bits.at_rd_empty) {
			break;
		}
		// put char to receiving buffer
		buff[len++] = mod->at_read(mod->cbdata, mod->pos_on_board);
	}

	spin_lock(&mod->at_lock);
	if (mod->at_xmit_count) {
		// read status register
		status.full = mod->get_status(mod->cbdata, mod->pos_on_board);
		// check for transmitter is ready
		if (status.bits.at_wr_empty) {
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

static int k5_board_open(struct inode *inode, struct file *filp)
{
	ssize_t res;
	size_t i, j;
	size_t len;

	struct k5_board *brd;
	struct k5_board_private_data *private_data;
	struct k5_gsm_module_data *mod;
	union k5_at_ch_status_reg status;

	brd = container_of(inode->i_cdev, struct k5_board, cdev);

	if (!(private_data = kmalloc(sizeof(struct k5_board_private_data), GFP_KERNEL))) {
		log(KERN_ERR, "can't get memory=%lu bytes\n", (unsigned long int)sizeof(struct k5_board_private_data));
		res = -ENOMEM;
		goto k5_open_error;
	}
	private_data->board = brd;

	len = 0;

	// gsm
	for (i = 0; i < 2; i++) {
		if ((mod = brd->gsm_modules[i])) {
			status.full = mod->get_status(mod->cbdata, mod->pos_on_board);
			len += sprintf(private_data->buff+len, "GSM%lu %s %s %s VIN0%s VIO=%u\r\n",
							(unsigned long int)i, // #
							polygator_print_gsm_module_type(mod->type), // signaling
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
							brd->tty_at_channels[i]?dev_name(brd->tty_at_channels[i]->device):"unknown", // signaling
							brd->simcard_channels[i]?dev_name(brd->simcard_channels[i]->device):"unknown", // sim
#else
							brd->tty_at_channels[i]?brd->tty_at_channels[i]->device->class_id:"unknown", // signaling
							brd->simcard_channels[i]?brd->simcard_channels[i]->device->class_id:"unknown", // sim
#endif
							i?"PCM0":"ALM3", // voice
							status.bits.status); // status
		}
	}
	// fxs
	for (i = 0; i < 2; i++) {
		len += sprintf(private_data->buff+len, "FXS%lu VIN0%s\r\n",
						(unsigned long int)i, // #
						i?"ALM2":"ALM0"); // voice
	}
	// fxo
	for (i = 0; i < 1; i++) {
		len += sprintf(private_data->buff+len, "FXO%lu VIN0%s RING=%u\r\n",
						(unsigned long int)i, // #
						"ALM1", // voice
						ioread8(k5_cs3_base_ptr + K5_CS_FXO) & 1); // ring
	}
	// vinetic
	if (brd->vinetic) {
		len += sprintf(private_data->buff+len, "VIN%lu %s\r\n",
							0L,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
							dev_name(brd->vinetic->device)
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
							dev_name(brd->vinetic->device)
#else
							brd->vinetic->device->class_id
#endif
							);
		for (j = 0; j < 4; j++) {
			if (brd->vinetic->rtp_channels[j]) {
				len += sprintf(private_data->buff+len, "VIN%luRTP%lu %s\r\n",
								0L,
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

	uint32_t ch;
	uint32_t value;
	struct k5_gsm_module_data *mod;
	struct k5_board_private_data *private_data = filp->private_data;

	memset(cmd, 0, sizeof(cmd));
	len = sizeof(cmd) - 1;
	len = min(len,count);

	if (copy_from_user(cmd, buff, len)) {
		res = -EINVAL;
		goto k5_board_write_end;
	}

	if (sscanf(cmd, "GSM%u PWR=%u", &ch, &value) == 2) {
		if ((ch >= 0) && (ch <= 2) && ((mod = private_data->board->gsm_modules[ch]))) {
			mod->control.bits.vbat = value;
			mod->set_control(mod->cbdata, mod->pos_on_board, mod->control.full);
			res = len;
		} else {
			res= -ENODEV;
		}
	} else if (sscanf(cmd, "GSM%u KEY=%u", &ch, &value) == 2) {
		if ((ch >= 0) && (ch <= 2) && ((mod = private_data->board->gsm_modules[ch]))) {
			mod->control.bits.pkey = !value;
			mod->set_control(mod->cbdata, mod->pos_on_board, mod->control.full);
			res = len;
		} else {
			res= -ENODEV;
		}
	} else if (sscanf(cmd, "GSM%u BAUDRATE=%u", &ch, &value) == 2) {
		if ((ch >= 0) && (ch <= 2) && ((mod = private_data->board->gsm_modules[ch]))) {
			if (value == 9600) {
				mod->control.bits.at_baudrate = 0;
			} else {
				mod->control.bits.at_baudrate = 2;
			}
			mod->set_control(mod->cbdata, mod->pos_on_board, mod->control.full);
			res = len;
		} else {
			res= -ENODEV;
		}
	} else if (sscanf(cmd, "FXO0 HOOK=%u", &value) == 1) {
		iowrite8(value & 1, k5_cs3_base_ptr + K5_CS_FXO);
		res = len;
	} else {
		res = -ENOMSG;
	}

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
	struct polygator_tty_device *ptd = tty->driver_data;
	struct k5_gsm_module_data *mod = (struct k5_gsm_module_data *)ptd->data;

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
		mod->at_poll_timer.function = k5_tty_at_poll;
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

static void k5_tty_at_close(struct tty_struct *tty, struct file *filp)
{
	struct polygator_tty_device *ptd = tty->driver_data;
	struct k5_gsm_module_data *mod = (struct k5_gsm_module_data *)ptd->data;

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

static int k5_tty_at_write(struct tty_struct *tty, const unsigned char *buf, int count)
{
	int res = 0;
	size_t len;
	struct polygator_tty_device *ptd = tty->driver_data;
	struct k5_gsm_module_data *mod = (struct k5_gsm_module_data *)ptd->data;
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

static int k5_tty_at_write_room(struct tty_struct *tty)
{
	int res;
	struct polygator_tty_device *ptd = tty->driver_data;
	struct k5_gsm_module_data *mod = (struct k5_gsm_module_data *)ptd->data;

	spin_lock_bh(&mod->at_lock);

	res = SERIAL_XMIT_SIZE - mod->at_xmit_count;

	spin_unlock_bh(&mod->at_lock);

	return res;
}

static int k5_tty_at_chars_in_buffer(struct tty_struct *tty)
{
	int res;
	struct polygator_tty_device *ptd = tty->driver_data;
	struct k5_gsm_module_data *mod = (struct k5_gsm_module_data *)ptd->data;

	spin_lock_bh(&mod->at_lock);

	res = mod->at_xmit_count;

	spin_unlock_bh(&mod->at_lock);

	return res;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20)
static void k5_tty_at_set_termios(struct tty_struct *tty, struct ktermios *old_termios)
#else
static void k5_tty_at_set_termios(struct tty_struct *tty, struct termios *old_termios)
#endif
{
	speed_t baud;
	struct polygator_tty_device *ptd = tty->driver_data;
	struct k5_gsm_module_data *mod = (struct k5_gsm_module_data *)ptd->data;

	baud = tty_get_baud_rate(tty);

	spin_lock_bh(&mod->at_lock);

	switch (baud) {
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

static void k5_tty_at_flush_buffer(struct tty_struct *tty)
{
	struct polygator_tty_device *ptd = tty->driver_data;
	struct k5_gsm_module_data *mod = (struct k5_gsm_module_data *)ptd->data;

	spin_lock_bh(&mod->at_lock);
	mod->at_xmit_count = mod->at_xmit_head = mod->at_xmit_tail = 0;
	spin_unlock_bh(&mod->at_lock);
	tty_wakeup(tty);
}

static void k5_tty_at_hangup(struct tty_struct *tty)
{
#ifdef TTY_PORT
	struct polygator_tty_device *ptd = tty->driver_data;
	struct k5_gsm_module_data *mod = (struct k5_gsm_module_data *)ptd->data;

	tty_port_hangup(&mod->at_port);
#endif
}
#ifdef TTY_PORT
static int k5_tty_at_port_carrier_raised(struct tty_port *port)
{
	return 1;
}

static void k5_tty_at_port_dtr_rts(struct tty_port *port, int onoff)
{
}

static int k5_tty_at_port_activate(struct tty_port *port, struct tty_struct *tty)
{
	struct k5_gsm_module_data *mod = container_of(port, struct k5_gsm_module_data, at_port);

	if (tty_port_alloc_xmit_buf(port) < 0) {
		return -ENOMEM;
	}
	mod->at_xmit_count = mod->at_xmit_head = mod->at_xmit_tail = 0;

	mod->at_poll_timer.function = k5_tty_at_poll;
	mod->at_poll_timer.data = (unsigned long)mod;
	mod->at_poll_timer.expires = jiffies + 1;
	add_timer(&mod->at_poll_timer);

	return 0;
}

static void k5_tty_at_port_shutdown(struct tty_port *port)
{
	struct k5_gsm_module_data *mod = container_of(port, struct k5_gsm_module_data, at_port);

	del_timer_sync(&mod->at_poll_timer);

	tty_port_free_xmit_buf(port);
}
#endif
static int __init k5_init(void)
{
	size_t i;
	struct k5_gsm_module_data *mod;
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
	if (!(k5_board->vinetic = vinetic_device_register(THIS_MODULE, "board-k5-vin0", 0,
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
	for (i = 0; i < 4; i++) {
		snprintf(devname, VINETIC_DEVNAME_MAXLEN, "board-k5-vin0-rtp%lu", (unsigned long int)i);
		if (!(vinetic_rtp_channel_register(THIS_MODULE, devname, k5_board->vinetic, i))) {
			rc = -1;
			goto k5_init_error;
		}
	}

	// set AT command channels
	for (i = 0; i < 2; i++) {
		if (!(mod = kmalloc(sizeof(struct k5_gsm_module_data), GFP_KERNEL))) {
			log(KERN_ERR, "can't get memory for struct k5_gsm_module_data\n");
			rc = -1;
			goto k5_init_error;
		}
		memset(mod, 0, sizeof(struct k5_gsm_module_data));

		mod->type = (i)?(POLYGATOR_MODULE_TYPE_SIM5215):(POLYGATOR_MODULE_TYPE_M10);

		mod->control.bits.vbat = 0;
		mod->control.bits.pkey = 1;
		mod->control.bits.cn_speed_a = 0;
		mod->control.bits.cn_speed_b = 0;
		mod->control.bits.at_baudrate = 2;

		mod->pos_on_board = i;
		mod->cbdata = (uintptr_t)k5_cs3_base_ptr;
		mod->set_control	= k5_gsm_mod_set_control;
		mod->get_status		= k5_gsm_mod_get_status;
		mod->at_write		= k5_gsm_mod_at_write;
		mod->at_read		= k5_gsm_mod_at_read;
		mod->sim_write		= k5_gsm_mod_sim_write;
		mod->sim_read		= k5_gsm_mod_sim_read;
#if 0
		mod->imei_write		= k5_gsm_mod_imei_write;
		mod->imei_read		= k5_gsm_mod_imei_read;
#endif
		mod->set_control(mod->cbdata, mod->pos_on_board, mod->control.full);
		init_timer(&mod->at_poll_timer);

		spin_lock_init(&mod->at_lock);
#ifdef TTY_PORT
		tty_port_init(&mod->at_port);
		mod->at_port.ops = &k5_tty_at_port_ops;
		mod->at_port.close_delay = 0;
		mod->at_port.closing_wait = ASYNC_CLOSING_WAIT_NONE;
#endif
		k5_board->gsm_modules[i] = mod;
	}

	// register polygator tty at device
	for (i = 0; i < 2; i++) {
		if ((mod = k5_board->gsm_modules[i])) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0)
			if (!(k5_board->tty_at_channels[i] = polygator_tty_device_register(THIS_MODULE, mod, &mod->at_port, &k5_tty_at_ops))) {
#else
			if (!(k5_board->tty_at_channels[i] = polygator_tty_device_register(THIS_MODULE, mod, &k5_tty_at_ops))) {
#endif
				log(KERN_ERR, "can't register polygator tty device\n");
				rc = -1;
				goto k5_init_error;
			}
		}
	}

	// register polygator simcard device
	for (i = 0; i < 2; i++) {
		if (k5_board->gsm_modules[i]) {
			if (!(k5_board->simcard_channels[i] = simcard_device_register(THIS_MODULE,
					k5_board->gsm_modules[i],
					k5_sim_read,
					k5_sim_write,
					k5_sim_is_read_ready,
					k5_sim_is_write_ready,
					k5_sim_is_reset_request,
					k5_sim_set_speed))) {
				log(KERN_ERR, "can't register polygator simcard device\n");
				rc = -1;
				goto k5_init_error;
			}
		}
	}

	verbose("loaded successfull\n");
	return rc;

k5_init_error:
	if (k5_board) {
		for (i = 0; i < 2; i++) {
			if (k5_board->simcard_channels[i]) {
				simcard_device_unregister(k5_board->simcard_channels[i]);
			}
			if (k5_board->tty_at_channels[i]) {
				polygator_tty_device_unregister(k5_board->tty_at_channels[i]);
			}
			if (k5_board->gsm_modules[i]) {
				del_timer_sync(&k5_board->gsm_modules[i]->at_poll_timer);
				kfree(k5_board->gsm_modules[i]);
			}
		}
		if (k5_board->vinetic) {
			for (i = 0; i < 4; i++) {
				if (k5_board->vinetic->rtp_channels[i])
					vinetic_rtp_channel_unregister(k5_board->vinetic->rtp_channels[i]);
			}
			vinetic_device_unregister(k5_board->vinetic);
		}
		if (k5_board->pg_board) {
			polygator_board_unregister(k5_board->pg_board);
		}
		kfree(k5_board);
	}

	if (k5_cs3_iomem_reg) {
		release_mem_region(AT91_CHIPSELECT_3, 0x10000);
	}
	if (k5_cs3_base_ptr) {
		iounmap(k5_cs3_base_ptr);
	}
	if (k5_cs4_iomem_reg) {
		release_mem_region(AT91_CHIPSELECT_4, 0x10000);
	}
	if (k5_cs4_base_ptr) {
		iounmap(k5_cs4_base_ptr);
	}
	return rc;
}

static void __exit k5_exit(void)
{
	size_t i;

	for (i = 0; i < 2; i++) {
		if (k5_board->simcard_channels[i]) {
			simcard_device_unregister(k5_board->simcard_channels[i]);
		}
		if (k5_board->tty_at_channels[i]) {
			polygator_tty_device_unregister(k5_board->tty_at_channels[i]);
		}
		if (k5_board->gsm_modules[i]) {
			del_timer_sync(&k5_board->gsm_modules[i]->at_poll_timer);
			kfree(k5_board->gsm_modules[i]);
		}
	}
	for (i = 0; i < 4; i++) {
		if (k5_board->vinetic->rtp_channels[i]) {
			vinetic_rtp_channel_unregister(k5_board->vinetic->rtp_channels[i]);
		}
	}
	vinetic_device_unregister(k5_board->vinetic);
	polygator_board_unregister(k5_board->pg_board);
	kfree(k5_board);

	release_mem_region(AT91_CHIPSELECT_3, 0x10000);
	iounmap(k5_cs3_base_ptr);
	release_mem_region(AT91_CHIPSELECT_4, 0x10000);
	iounmap(k5_cs4_base_ptr);

	verbose("stopped\n");
}

module_init(k5_init);
module_exit(k5_exit);
