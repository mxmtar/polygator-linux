/******************************************************************************/
/* gsm8ch-base.c                                                              */
/******************************************************************************/

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

#include "polygator/vinetic-base.h"
#include "polygator/vinetic-def.h"

MODULE_AUTHOR("Maksym Tarasevych <mxmtar@ukr.net>");
MODULE_DESCRIPTION("Polygator Linux module for gsm8ch boards");
MODULE_LICENSE("GPL");

static int tty_at_major = 0;
module_param(tty_at_major, int, 0);
MODULE_PARM_DESC(tty_at_major, "Major number for AT-command channel of Polygator gsm8ch boards");

#define verbose(_fmt, _args...) printk(KERN_INFO "[polygator-%s] " _fmt, THIS_MODULE->name, ## _args)
#define log(_level, _fmt, _args...) printk(_level "[polygator-%s] %s:%d - %s(): " _fmt, THIS_MODULE->name, "gsm8ch-base.c", __LINE__, __PRETTY_FUNCTION__, ## _args)
#define debug(_fmt, _args...) printk(KERN_DEBUG "[polygator-%s] %s:%d - %s(): " _fmt, THIS_MODULE->name, "gsm8ch-base.c", __LINE__, __PRETTY_FUNCTION__, ## _args)

/*! */
#define PG_PCI_NUM_BASE			0x00
#define PG_PCI_OFFSET_RESET		0x00
#define PG_PCI_ID_BASE			0x28
#define PG_PCI_COM_BASE			0x30
#define PG_PCI_ST_BASE			0x04
#define PG_PCI_CTL_BASE			0x04
#define PG_PCI_ROM_BASE			0x24 // 0x36
#define PG_PCI_VIN_ST_BASE		0x2C
#define PG_PCI_VIN_DATA_BASE	0x50
#define PG_PCI_IMEI_BASE		0xA0
/*! */

#define GSM8CH_TTY_AT_DEVICE_MAXCOUNT 256

union gsm8ch_at_ch_status_reg {
	struct {
		u_int8_t com_rdy_rd:1;
		u_int8_t com_rdy_wr:1;
		u_int8_t vio:1;
		u_int8_t gap0:2;
		u_int8_t gsm_reset_req:1;
		u_int8_t imei_rdy_rd:1;
		u_int8_t imei_rdy_wr:1;
	} __attribute__((packed)) bits;
	u_int8_t full;
} __attribute__((packed));

union gsm8ch_at_ch_control_reg {
	struct {
		u_int8_t mod_off:1;
		u_int8_t mode:1;	//	GR = 0, PiML = 1
		u_int8_t rst:1;
		u_int8_t gap0:1;
		u_int8_t pwr_off:1;
		u_int8_t sync_mode:1;
		u_int8_t gap1:2;
	} __attribute__((packed)) bits;
	u_int8_t full;
} __attribute__((packed));

struct gsm8ch_tty_at_channel {

	int gsm_mod_type;
	size_t pos_on_board;

	union gsm8ch_at_ch_status_reg status;
	union gsm8ch_at_ch_control_reg control;

	struct timer_list poll_timer;

	struct tty_port port;

	spinlock_t lock;

	uintptr_t cbdata;

	void (* mod_control)(uintptr_t cbdata, size_t pos, u_int8_t reg);
	u_int8_t (* mod_status)(uintptr_t cbdata, size_t pos);
	void (* mod_at_write)(uintptr_t cbdata, size_t pos, u_int8_t reg);
	u_int8_t (* mod_at_read)(uintptr_t cbdata, size_t pos);

	int tty_at_minor;
	struct device *device;
};

struct gsm8ch_board {

	struct polygator_board *pg_board;
	struct cdev cdev;

	u_int8_t rom[256];
	size_t romsize;
	u_int32_t sn;
	u_int16_t type;

	struct vinetic *vinetics[2];

	struct gsm8ch_tty_at_channel *tty_at_channels[8];
};

struct gsm8ch_board_private_data {
	struct gsm8ch_board *board;
	char buff[0x0C00];
	size_t length;
};

struct gsm8ch_tty_at_channel *gsm8ch_tty_at_channel_list[GSM8CH_TTY_AT_DEVICE_MAXCOUNT];
static DEFINE_MUTEX(gsm8ch_tty_at_channel_list_lock);

static struct tty_driver *gsm8ch_tty_at_driver = NULL;

static int gsm8ch_tty_at_open(struct tty_struct *tty, struct file *filp);
static void gsm8ch_tty_at_close(struct tty_struct *tty, struct file *filp);
static int gsm8ch_tty_at_write(struct tty_struct *tty, const unsigned char *buf, int count);
static int gsm8ch_tty_at_write_room(struct tty_struct *tty);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20)
static void gsm8ch_tty_at_set_termios(struct tty_struct *tty, struct ktermios *old_termios);
#else
static void gsm8ch_tty_at_set_termios(struct tty_struct *tty, struct termios *old_termios);
#endif
static void gsm8ch_tty_at_hangup(struct tty_struct *tty);

static struct tty_operations gsm8ch_tty_at_ops = {
	.open = gsm8ch_tty_at_open,
	.close = gsm8ch_tty_at_close,
	.write = gsm8ch_tty_at_write,
	.write_room = gsm8ch_tty_at_write_room,
// 	.chars_in_buffer = gsm8ch_tty_at_chars_in_buffer,
	.set_termios = gsm8ch_tty_at_set_termios,
// 	.flush_buffer = gsm8ch_tty_at_flush_buffer,
	.hangup = gsm8ch_tty_at_hangup,
};

static int gsm8ch_tty_at_port_carrier_raised(struct tty_port *port);
static void gsm8ch_tty_at_port_dtr_rts(struct tty_port *port, int onoff);
static int gsm8ch_tty_at_port_activate(struct tty_port *tport, struct tty_struct *tty);
static void gsm8ch_tty_at_port_shutdown(struct tty_port *port);

static const struct tty_port_operations gsm8ch_tty_at_port_ops = {
	.carrier_raised =gsm8ch_tty_at_port_carrier_raised,
	.dtr_rts = gsm8ch_tty_at_port_dtr_rts,
	.activate = gsm8ch_tty_at_port_activate,
	.shutdown = gsm8ch_tty_at_port_shutdown,
};

static struct pci_device_id gsm8ch_pci_board_id_table[] = {
	{ PCI_DEVICE(0xDEAD, 0xBEEF), .driver_data = 1, },
	{ 0, },
};
MODULE_DEVICE_TABLE(pci, gsm8ch_pci_board_id_table);

static void gsm8ch_pci_reset_0(uintptr_t cbdata)
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

static void gsm8ch_pci_reset_1(uintptr_t cbdata)
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

static void gsm8ch_pci_write_nwd_0(uintptr_t cbdata, u_int16_t value)
{
	outw(value, cbdata + PG_PCI_VIN_DATA_BASE + 0 + 0);
// 	debug("%04x\n", value);
}

static void gsm8ch_pci_write_nwd_1(uintptr_t cbdata, u_int16_t value)
{
	outw(value, cbdata + PG_PCI_VIN_DATA_BASE + 8 + 0);
// 	debug("%04x\n", value);
}

static void gsm8ch_pci_write_eom_0(uintptr_t cbdata, u_int16_t value)
{
	outw(value, cbdata + PG_PCI_VIN_DATA_BASE + 0 + 4);
// 	debug("%04x\n", value);
}

static void gsm8ch_pci_write_eom_1(uintptr_t cbdata, u_int16_t value)
{
	outw(value, cbdata + PG_PCI_VIN_DATA_BASE + 8 + 4);
// 	debug("%04x\n", value);brd->tty_at_channels[i]
}

static u_int16_t gsm8ch_pci_read_nwd_0(uintptr_t cbdata)
{
	u_int16_t value = inw(cbdata + PG_PCI_VIN_DATA_BASE + 0 + 0);
// 	debug("%04x\n", value);
	return value;
}

static u_int16_t gsm8ch_pci_read_nwd_1(uintptr_t cbdata)
{
	u_int16_t value = inw(cbdata + PG_PCI_VIN_DATA_BASE + 8 + 0);
// 	debug("%04x\n", value);
	return value;
}

static u_int16_t gsm8ch_pci_read_eom_0(uintptr_t cbdata)
{
	u_int16_t value = inw(cbdata + PG_PCI_VIN_DATA_BASE + 0 + 4);
// 	debug("%04x\n", value);
	return value;
}

static u_int16_t gsm8ch_pci_read_eom_1(uintptr_t cbdata)
{
	u_int16_t value = inw(cbdata + PG_PCI_VIN_DATA_BASE + 8 + 4);
// 	debug("%04x\n", value);
	return value;
}

static size_t gsm8ch_pci_is_not_ready_0(uintptr_t cbdata)
{
	size_t st = (inb(cbdata + PG_PCI_VIN_ST_BASE) >> 0) & 1;
	return st;
// 	union vin_reg_ir reg_ir;
// 	reg_ir.full = inw(cbdata + PG_PCI_VIN_DATA_BASE + 0 + 8);
// 	debug("%04x\n", reg_ir.full);
// 	return reg_ir.bits.rdyq;
}

static size_t gsm8ch_pci_is_not_ready_1(uintptr_t cbdata)
{
	size_t st = (inb(cbdata + PG_PCI_VIN_ST_BASE) >> 1) & 1;
	return st;
// 	union vin_reg_ir reg_ir;
// 	reg_ir.full = inw(cbdata + PG_PCI_VIN_DATA_BASE + 0 + 8);
// 	debug("%04x\n", reg_ir.full);private_data->board = brd;
// 	return reg_ir.bits.rdyq;
}

static u_int16_t gsm8ch_pci_read_dia_0(uintptr_t cbdata)
{
	//return inw(cbdata + PG_PCI_VIN_DATA_BASE + 0 + 8);
	return 0;
}

static u_int16_t gsm8ch_pci_read_dia_1(uintptr_t cbdata)
{
// 	return inw(cbdata + PG_PCI_VIN_DATA_BASE + 0 + 8);
	return 0;
}

static void gsm8ch_pci_mod_control(uintptr_t cbdata, size_t pos, u_int8_t reg)
{
	outb(reg, cbdata + PG_PCI_CTL_BASE + pos*4);
}

static u_int8_t gsm8ch_pci_mod_status(uintptr_t cbdata, size_t pos)
{
	return inb(cbdata + PG_PCI_CTL_BASE + pos*4);
}

static void gsm8ch_pci_mod_at_write(uintptr_t cbdata, size_t pos, u_int8_t reg)
{
	outb(reg, cbdata + PG_PCI_COM_BASE + pos*4);
}

static u_int8_t gsm8ch_pci_mod_at_read(uintptr_t cbdata, size_t pos)
{
	return inb(cbdata + PG_PCI_COM_BASE + pos*4);
}

static void gsm8ch_tty_at_poll(unsigned long addr)
{
	char buff[256];
	size_t len;
	struct tty_struct *tty;
	struct gsm8ch_tty_at_channel *ch = (struct gsm8ch_tty_at_channel *)addr;

	len = 0;

	spin_lock(&ch->lock);

	// read status register
	ch->status.full = ch->mod_status(ch->cbdata, ch->pos_on_board);
	// check for ready receiving data
	while((!ch->status.bits.com_rdy_rd) && (len < sizeof(buff)))
	{
		// put char to receiving buffer
		buff[len++] = ch->mod_at_read(ch->cbdata, ch->pos_on_board);
		// read status register
		ch->status.full = ch->mod_status(ch->cbdata, ch->pos_on_board);
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

static int gsm8ch_board_open(struct inode *inode, struct file *filp)
{
	ssize_t res;
	size_t i,j;
	size_t len;

	struct gsm8ch_board *brd;
	struct gsm8ch_board_private_data *private_data;

	brd = container_of(inode->i_cdev, struct gsm8ch_board, cdev);

	if (!(private_data = kmalloc(sizeof(struct gsm8ch_board_private_data), GFP_KERNEL))) {
		log(KERN_ERR, "can't get memory=%lu bytes\n", (unsigned long int)sizeof(struct gsm8ch_board_private_data));
		res = -ENOMEM;
		goto gsm8ch_open_error;
	}
	private_data->board = brd;

	len = 0;
	for (i=0; i<8; i++)
	{
		if (brd->tty_at_channels[i]) {
			brd->tty_at_channels[i]->status.full = brd->tty_at_channels[i]->mod_status(brd->tty_at_channels[i]->cbdata, brd->tty_at_channels[i]->pos_on_board);
			len += sprintf(private_data->buff+len, "GSM%lu gsm8chAT%d %s VIO=%u\r\n", (unsigned long int)i, brd->tty_at_channels[i]->tty_at_minor, polygator_print_gsm_module_type(brd->tty_at_channels[i]->gsm_mod_type), brd->tty_at_channels[i]->status.bits.vio);
		}
	}
	for (i=0; i<2; i++)
	{
		if (brd->vinetics[i]) {
			len += sprintf(private_data->buff+len, "VIN%lu board-gsm8ch-pci-%u-vin%lu\r\n", (unsigned long int)i, brd->sn, (unsigned long int)i);
			for (j=0; j<4; j++)
			{
				if (brd->vinetics[i]->rtp_channels[j])
					len += sprintf(private_data->buff+len, "VIN%luRTP%lu board-gsm8ch-pci-%u-vin%lu-rtp%lu\r\n", (unsigned long int)i, (unsigned long int)j, brd->sn, (unsigned long int)i, (unsigned long int)j);
			}
		}
	}

	private_data->length = len;

	filp->private_data = private_data;

	return 0;

gsm8ch_open_error:
	if (private_data) kfree(private_data);
	return res;
}

static int gsm8ch_board_release(struct inode *inode, struct file *filp)
{
	struct gsm8ch_board_private_data *private_data = filp->private_data;

	kfree(private_data);
	return 0;
}

static ssize_t gsm8ch_board_read(struct file *filp, char __user *buff, size_t count, loff_t *offp)
{
	size_t len;
	ssize_t res;
	struct gsm8ch_board_private_data *private_data = filp->private_data;

	res = (private_data->length > filp->f_pos)?(private_data->length - filp->f_pos):(0);

	if (res) {
		len = res;
		len = min(count, len);
		if (copy_to_user(buff, private_data->buff + filp->f_pos, len)) {
			res = -EINVAL;
			goto gsm8ch_board_read_end;
		}
		*offp = filp->f_pos + len;
	}

gsm8ch_board_read_end:
	return res;
}

static ssize_t gsm8ch_board_write(struct file *filp, const char __user *buff, size_t count, loff_t *offp)
{
	ssize_t res;
	char cmd[256];
	size_t len;

	u_int32_t at_chan;
	u_int32_t pwr_state;
	u_int32_t key_state;
	u_int32_t baudrate;
	struct gsm8ch_board_private_data *private_data = filp->private_data;

	memset(cmd, 0, sizeof(cmd));
	len = sizeof(cmd) - 1;
	len = min(len,count);

	if (copy_from_user(cmd, buff, len)) {
		res = -EINVAL;
		goto gsm8ch_board_write_end;
	}

	if (sscanf(cmd, "GSM%u PWR=%u", &at_chan, &pwr_state) == 2) {
		if ((at_chan >= 0) && (at_chan <= 7) && (private_data->board->tty_at_channels[at_chan])) {
			private_data->board->tty_at_channels[at_chan]->control.bits.pwr_off = !pwr_state;
			private_data->board->tty_at_channels[at_chan]->mod_control(private_data->board->tty_at_channels[at_chan]->cbdata, at_chan, private_data->board->tty_at_channels[at_chan]->control.full);
			res = len;
		} else
			res = - ENODEV;
	} else if (sscanf(cmd, "GSM%u KEY=%u", &at_chan, &key_state) == 2) {
		if ((at_chan >= 0) && (at_chan <= 7) && (private_data->board->tty_at_channels[at_chan])) {
			private_data->board->tty_at_channels[at_chan]->control.bits.mod_off = !key_state;
			private_data->board->tty_at_channels[at_chan]->mod_control(private_data->board->tty_at_channels[at_chan]->cbdata, at_chan, private_data->board->tty_at_channels[at_chan]->control.full);
			res = len;
		} else
			res = -ENODEV;
	} else if (sscanf(cmd, "GSM%u BAUDRATE=%u", &at_chan, &baudrate) == 2) {
		if ((at_chan >= 0) && (at_chan <= 7) && (private_data->board->tty_at_channels[at_chan])) {
			if ((baudrate == 9600) || (private_data->board->tty_at_channels[at_chan]->gsm_mod_type == POLYGATOR_MODULE_TYPE_SIM300))
				private_data->board->tty_at_channels[at_chan]->control.bits.gap1 = 3;
			else
				private_data->board->tty_at_channels[at_chan]->control.bits.gap1 = 2;
			private_data->board->tty_at_channels[at_chan]->mod_control(private_data->board->tty_at_channels[at_chan]->cbdata, at_chan, private_data->board->tty_at_channels[at_chan]->control.full);
			res = len;
		} else
			res = -ENODEV;
	} else
		res = -ENOMSG;

gsm8ch_board_write_end:
	return res;
}

static struct file_operations gsm8ch_board_fops = {
	.owner   = THIS_MODULE,
	.open    = gsm8ch_board_open,
	.release = gsm8ch_board_release,
	.read    = gsm8ch_board_read,
	.write   = gsm8ch_board_write,
};

static int __devinit gsm8ch_pci_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	int rc;
	unsigned long addr;
	int gsm_mod_type;
	u_int32_t pow10;
	size_t i,j;
	char devname[POLYGATOR_BRDNAME_MAXLEN];
	struct gsm8ch_board *brd = NULL;
	int get_pci_region = 0;

	rc = pci_enable_device(pdev);
	if (rc) {
		dev_err(&pdev->dev, "can't enable pci device\n");
		goto gsm8ch_pci_probe_error;
	}
	rc = pci_request_region(pdev, 0, "gsm8ch");
	if (rc) {
		dev_err(&pdev->dev, "can't request I/O region\n");
		goto gsm8ch_pci_probe_error;
	}
	get_pci_region = 1;

	// alloc memory for board data
	if (!(brd = kmalloc(sizeof(struct gsm8ch_board), GFP_KERNEL))) {
		log(KERN_ERR, "can't get memory for struct gsm8ch_board\n");
		rc = -1;
		goto gsm8ch_pci_probe_error;
	}
	memset(brd, 0, sizeof(struct gsm8ch_board));

	// get starting address
	addr = pci_resource_start(pdev, 0);

	// reset board
	outb(0xff, addr + PG_PCI_OFFSET_RESET);
	mdelay(10);
	outb(0x00, addr + PG_PCI_OFFSET_RESET);

	// get board type
	brd->type = 0;
	for (i=0; i<16; i++)
	{
		brd->type <<= 1;
		brd->type |= inb(addr + PG_PCI_ID_BASE) & 0x01;
	}
	verbose("found PCI board type=%04x\n", brd->type & 0x00ff);

	if (((brd->type & 0x00ff) != 0x0081) && ((brd->type & 0x00ff) != 0x0082) && ((brd->type & 0x00ff) != 0x0083)) {
		log(KERN_ERR, "PCI board type=%04x unsupported\n", brd->type & 0x00ff);
		rc = -1;
		goto gsm8ch_pci_probe_error;
	}

	// read board rom
	memset(brd->rom, 0, 256);
	brd->romsize = inb(addr + PG_PCI_ROM_BASE);
	brd->romsize = inb(addr + PG_PCI_ROM_BASE);
	for (i=0; i<brd->romsize; i++) brd->rom[i] = inb(addr + PG_PCI_ROM_BASE);
	verbose("\"%.*s\"\n", (int)brd->romsize, brd->rom);

	// get board serial number
	i = brd->romsize - 1;
	pow10 = 1;
	brd->sn = 0;
	while (i--)
	{
		if ((brd->rom[i] < 0x30) || (brd->rom[i] > 0x39))
			break;
		brd->sn += (brd->rom[i] - 0x30) * pow10;
		pow10 *= 10;
	}
	snprintf(devname, POLYGATOR_BRDNAME_MAXLEN, "board-gsm8ch-pci-%u", brd->sn);
	if (!(brd->pg_board =  polygator_board_register(THIS_MODULE, devname, &brd->cdev, &gsm8ch_board_fops))) {
		rc = -1;
		goto gsm8ch_pci_probe_error;
	}

	for (j=0; j<2; j++)
	{
		snprintf(devname, POLYGATOR_BRDNAME_MAXLEN, "board-gsm8ch-pci-%u-vin%lu", brd->sn, (unsigned long int)j);
		if (!(brd->vinetics[j] = vinetic_device_register(THIS_MODULE, devname, addr,
													(j)?(gsm8ch_pci_reset_1):(gsm8ch_pci_reset_0),
													(j)?(gsm8ch_pci_is_not_ready_1):(gsm8ch_pci_is_not_ready_0),
													(j)?(gsm8ch_pci_write_nwd_1):(gsm8ch_pci_write_nwd_0),
													(j)?(gsm8ch_pci_write_eom_1):(gsm8ch_pci_write_eom_0),
													(j)?(gsm8ch_pci_read_nwd_1):(gsm8ch_pci_read_nwd_0),
													(j)?(gsm8ch_pci_read_eom_1):(gsm8ch_pci_read_eom_0),
													(j)?(gsm8ch_pci_read_dia_1):(gsm8ch_pci_read_dia_0)))) {
			rc = -1;
			goto gsm8ch_pci_probe_error;
		}
		for (i=0; i<4; i++)
		{
			snprintf(devname, POLYGATOR_BRDNAME_MAXLEN, "board-gsm8ch-pci-%u-vin%lu-rtp%lu", brd->sn, (unsigned long int)j, (unsigned long int)i);
			if (!vinetic_rtp_channel_register(THIS_MODULE, devname, brd->vinetics[j], i)) {
				rc = -1;
				goto gsm8ch_pci_probe_error;
			}
		}
	}

	// set AT command channels
	for (i=0; i<8; i++)
	{
		if (((brd->type & 0x00ff) == 0x0082) || ((brd->type & 0x00ff) == 0x0083)) {
			if (brd->rom[8] == '*') {
				if (brd->rom[i] == 'M')
					gsm_mod_type = POLYGATOR_MODULE_TYPE_M10;
				else if (brd->rom[i] == '9')
					gsm_mod_type = POLYGATOR_MODULE_TYPE_SIM900;
				else if (brd->rom[i] == 'S')
					gsm_mod_type = POLYGATOR_MODULE_TYPE_SIM300;
				else
					gsm_mod_type = POLYGATOR_MODULE_TYPE_UNKNOWN;
			} else
				gsm_mod_type = POLYGATOR_MODULE_TYPE_SIM300;
		} else
			gsm_mod_type = POLYGATOR_MODULE_TYPE_SIM300;

		if (gsm_mod_type != POLYGATOR_MODULE_TYPE_UNKNOWN) {
			//
			if (!(brd->tty_at_channels[i] = kmalloc(sizeof(struct gsm8ch_tty_at_channel), GFP_KERNEL))) {
				log(KERN_ERR, "can't get memory for struct gsm8ch_tty_at_channel\n");
				rc = -1;
				goto gsm8ch_pci_probe_error;
			}
			memset(brd->tty_at_channels[i], 0, sizeof(struct gsm8ch_tty_at_channel));
			spin_lock_init(&brd->tty_at_channels[i]->lock);
			tty_port_init(&brd->tty_at_channels[i]->port);
			brd->tty_at_channels[i]->port.ops = &gsm8ch_tty_at_port_ops;
			brd->tty_at_channels[i]->port.close_delay = 0;
			brd->tty_at_channels[i]->port.closing_wait = ASYNC_CLOSING_WAIT_NONE;
			//
			brd->tty_at_channels[i]->pos_on_board = i;
			brd->tty_at_channels[i]->tty_at_minor = -1;
			brd->tty_at_channels[i]->cbdata = addr;
			brd->tty_at_channels[i]->mod_control = gsm8ch_pci_mod_control;
			brd->tty_at_channels[i]->mod_status = gsm8ch_pci_mod_status;
			brd->tty_at_channels[i]->mod_at_write = gsm8ch_pci_mod_at_write;
			brd->tty_at_channels[i]->mod_at_read = gsm8ch_pci_mod_at_read;
			// get free slot from channel list
			mutex_lock(&gsm8ch_tty_at_channel_list_lock);
			for (j=0; j<GSM8CH_TTY_AT_DEVICE_MAXCOUNT; j++)
			{
				if (!gsm8ch_tty_at_channel_list[j]) {
					gsm8ch_tty_at_channel_list[j] = brd->tty_at_channels[i];
					brd->tty_at_channels[i]->tty_at_minor = j;
					break;
				}
			}
			mutex_unlock(&gsm8ch_tty_at_channel_list_lock);

			if (brd->tty_at_channels[i]->tty_at_minor < 0) {
				log(KERN_ERR, "can't get free slot in gsm8ch_tty_at_channel_list\n");
				rc = -1;
				goto gsm8ch_pci_probe_error;
			}
			//
			if (gsm_mod_type == POLYGATOR_MODULE_TYPE_SIM300) {
				brd->tty_at_channels[i]->gsm_mod_type = POLYGATOR_MODULE_TYPE_SIM300;
				brd->tty_at_channels[i]->control.bits.mod_off = 1;		// module inactive
				brd->tty_at_channels[i]->control.bits.mode = 0;			// GR
				brd->tty_at_channels[i]->control.bits.rst = 0;			// M10=1 SIM300=0
				brd->tty_at_channels[i]->control.bits.gap0 = 0;			// don't care
				brd->tty_at_channels[i]->control.bits.pwr_off = 1;		// power suply disabled
				brd->tty_at_channels[i]->control.bits.sync_mode = 1;	// 0 - synchronous, 1 - asynchronous
				brd->tty_at_channels[i]->control.bits.gap1 = 3;			// 3 - 9600, 2 - 115200
			} else if (gsm_mod_type == POLYGATOR_MODULE_TYPE_SIM900) {
				brd->tty_at_channels[i]->gsm_mod_type = POLYGATOR_MODULE_TYPE_SIM900;
				brd->tty_at_channels[i]->control.bits.mod_off = 1;		// module inactive
				brd->tty_at_channels[i]->control.bits.mode = 0;			// GR
				brd->tty_at_channels[i]->control.bits.rst = 0;			// M10=1 SIM300=0
				brd->tty_at_channels[i]->control.bits.gap0 = 0;			// don't care
				brd->tty_at_channels[i]->control.bits.pwr_off = 1;		// power suply disabled
				brd->tty_at_channels[i]->control.bits.sync_mode = 1;	// 0 - synchronous, 1 - asynchronous
				brd->tty_at_channels[i]->control.bits.gap1 = 2;			// 3 - 9600, 2 - 115200
			} else if (gsm_mod_type == POLYGATOR_MODULE_TYPE_M10) {
				brd->tty_at_channels[i]->gsm_mod_type = POLYGATOR_MODULE_TYPE_M10;
				brd->tty_at_channels[i]->control.bits.mod_off = 1;		// module inactive
				brd->tty_at_channels[i]->control.bits.mode = 0;			// GR
				brd->tty_at_channels[i]->control.bits.rst = 1;			// M10=1 SIM300=0
				brd->tty_at_channels[i]->control.bits.gap0 = 0;		// don't care
				brd->tty_at_channels[i]->control.bits.pwr_off = 1;		// power suply disabled
				brd->tty_at_channels[i]->control.bits.sync_mode = 1;	// 0 - synchronous, 1 - asynchronous
				brd->tty_at_channels[i]->control.bits.gap1 = 2;			// 3 - 9600, 2 - 115200
			}
			// register device on sysfs
			brd->tty_at_channels[i]->device = tty_register_device(gsm8ch_tty_at_driver, j, NULL);
			if (IS_ERR(brd->tty_at_channels[i]->device)) {
				log(KERN_ERR, "can't register tty device\n");
				rc = -1;
				goto gsm8ch_pci_probe_error;
			}

			brd->tty_at_channels[i]->mod_control(brd->tty_at_channels[i]->cbdata, brd->tty_at_channels[i]->pos_on_board, brd->tty_at_channels[i]->control.full);
			
			init_timer(&brd->tty_at_channels[i]->poll_timer);
		}
	}

	pci_set_drvdata(pdev, brd);
	return 0;

gsm8ch_pci_probe_error:
	if (brd) {
		for (i=0; i<8; i++)
		{
			if (brd->tty_at_channels[i]) {
				del_timer_sync(&brd->tty_at_channels[i]->poll_timer);
				if (brd->tty_at_channels[i]->device)
					tty_unregister_device(gsm8ch_tty_at_driver, brd->tty_at_channels[i]->tty_at_minor);
				mutex_lock(&gsm8ch_tty_at_channel_list_lock);
				gsm8ch_tty_at_channel_list[brd->tty_at_channels[i]->tty_at_minor] = NULL;
				mutex_unlock(&gsm8ch_tty_at_channel_list_lock);
				kfree(brd->tty_at_channels[i]);
			}
		}
		for (j=0; j<2; j++)
		{
			if (brd->vinetics[j]) {
				for (i=0; i<4; i++)
				{
					if (brd->vinetics[j]->rtp_channels[i])
						vinetic_rtp_channel_unregister(brd->vinetics[j]->rtp_channels[i]);
				}
				vinetic_device_unregister(brd->vinetics[j]);
			}
		}
		if (brd->pg_board) polygator_board_unregister(brd->pg_board);

		kfree(brd);
	}
	if (get_pci_region) pci_release_region(pdev, 0);
	return rc;
}

static void __devexit gsm8ch_pci_remove(struct pci_dev *pdev)
{
	size_t i,j;
	struct gsm8ch_board *brd = pci_get_drvdata(pdev);

	for (i=0; i<8; i++)
	{
		if (brd->tty_at_channels[i]) {
			del_timer_sync(&brd->tty_at_channels[i]->poll_timer);
			tty_unregister_device(gsm8ch_tty_at_driver, brd->tty_at_channels[i]->tty_at_minor);
			mutex_lock(&gsm8ch_tty_at_channel_list_lock);
			gsm8ch_tty_at_channel_list[brd->tty_at_channels[i]->tty_at_minor] = NULL;
			mutex_unlock(&gsm8ch_tty_at_channel_list_lock);
			kfree(brd->tty_at_channels[i]);
		}
	}

	for (j=0; j<2; j++)
	{
		for (i=0; i<4; i++)
		{
			vinetic_rtp_channel_unregister(brd->vinetics[j]->rtp_channels[i]);
		}
		vinetic_device_unregister(brd->vinetics[j]);
	}

	polygator_board_unregister(brd->pg_board);

	kfree(brd);
	pci_release_region(pdev, 0);
}

static struct pci_driver gsm8ch_pci_driver = {
	.name = "gsm8ch",
	.id_table = gsm8ch_pci_board_id_table,
	.probe = gsm8ch_pci_probe,
	.remove = gsm8ch_pci_remove,
};

static int gsm8ch_tty_at_open(struct tty_struct *tty, struct file *filp)
{

	size_t i;
	struct gsm8ch_tty_at_channel *ch = NULL;

	if (mutex_lock_interruptible(&gsm8ch_tty_at_channel_list_lock))
		return -ERESTARTSYS;

	for (i=0; i<GSM8CH_TTY_AT_DEVICE_MAXCOUNT; i++)
	{
		if ((gsm8ch_tty_at_channel_list[i]) && (gsm8ch_tty_at_channel_list[i]->tty_at_minor == tty->index)) {
			ch = gsm8ch_tty_at_channel_list[i];
			break;
		}
	}

	if (!ch) {
		mutex_unlock(&gsm8ch_tty_at_channel_list_lock);
		return -ENODEV;
	}

	tty->driver_data = ch;

	mutex_unlock(&gsm8ch_tty_at_channel_list_lock);

	return tty_port_open(&ch->port, tty, filp);
}

static void gsm8ch_tty_at_close(struct tty_struct *tty, struct file *filp)
{
	struct gsm8ch_tty_at_channel *ch = tty->driver_data;

	tty_port_close(&ch->port, tty, filp);
	return;
}

static int gsm8ch_tty_at_write(struct tty_struct *tty, const unsigned char *buf, int count)
{
	int res;
	struct gsm8ch_tty_at_channel *ch = tty->driver_data;

	spin_lock_bh(&ch->lock);

	// read status register
	ch->status.full = ch->mod_status(ch->cbdata, ch->pos_on_board);

	if (ch->status.bits.com_rdy_wr) {
		ch->mod_at_write(ch->cbdata, ch->pos_on_board, *buf);
		res = 1;
	} else
		res = 0;

	spin_unlock_bh(&ch->lock);

	return res ;
}

static int gsm8ch_tty_at_write_room(struct tty_struct *tty)
{
	int res;
	struct gsm8ch_tty_at_channel *ch = tty->driver_data;

	spin_lock_bh(&ch->lock);

	// read status register
	ch->status.full = ch->mod_status(ch->cbdata, ch->pos_on_board);

	if (ch->status.bits.com_rdy_wr)
		res = 1;
	else
		res = 0;

	spin_unlock_bh(&ch->lock);

	return res;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20)
static void gsm8ch_tty_at_set_termios(struct tty_struct *tty, struct ktermios *old_termios)
#else
static void gsm8ch_tty_at_set_termios(struct tty_struct *tty, struct termios *old_termios)
#endif
{
	speed_t baud;
	struct gsm8ch_tty_at_channel *ch = tty->driver_data;

	baud = tty_get_baud_rate(tty);

	spin_lock_bh(&ch->lock);

	switch (baud)
	{
		case 9600:
			ch->control.bits.gap1 = 3;
			break;
		default:
			ch->control.bits.gap1 = 2;
			break;
	}

	ch->mod_control(ch->cbdata, ch->pos_on_board, ch->control.full);

	spin_unlock_bh(&ch->lock);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
	tty_encode_baud_rate(tty, baud, baud);
#endif
}

static void gsm8ch_tty_at_hangup(struct tty_struct *tty)
{
	struct gsm8ch_tty_at_channel *ch = tty->driver_data;
	tty_port_hangup(&ch->port);
}

static int gsm8ch_tty_at_port_carrier_raised(struct tty_port *port)
{
	return 1;
}

static void gsm8ch_tty_at_port_dtr_rts(struct tty_port *port, int onoff)
{
}

static int gsm8ch_tty_at_port_activate(struct tty_port *port, struct tty_struct *tty)
{
	struct gsm8ch_tty_at_channel *ch = container_of(port, struct gsm8ch_tty_at_channel, port);

	ch->poll_timer.function = gsm8ch_tty_at_poll;
	ch->poll_timer.data = (unsigned long)ch;
	ch->poll_timer.expires = jiffies + 1;
	add_timer(&ch->poll_timer);

	return 0;
}

static void gsm8ch_tty_at_port_shutdown(struct tty_port *port)
{
	struct gsm8ch_tty_at_channel *ch = container_of(port, struct gsm8ch_tty_at_channel, port);

	del_timer_sync(&ch->poll_timer);
}

static int __init gsm8ch_init(void)
{
	int rc;
	size_t i;

	verbose("loading ...\n");

	for (i=0; i<GSM8CH_TTY_AT_DEVICE_MAXCOUNT; i++)
		gsm8ch_tty_at_channel_list[i] = NULL;

	// registering tty device for AT-command channel
	gsm8ch_tty_at_driver = alloc_tty_driver(GSM8CH_TTY_AT_DEVICE_MAXCOUNT);
	if (!gsm8ch_tty_at_driver) {
		log(KERN_ERR, "can't allocated memory for tty driver\n");
		return -ENOMEM;
	}

	gsm8ch_tty_at_driver->owner = THIS_MODULE;
	gsm8ch_tty_at_driver->driver_name = "gsm8ch_tty_at";
	gsm8ch_tty_at_driver->name = "polygator/gsm8chAT";
	gsm8ch_tty_at_driver->major = tty_at_major;
	gsm8ch_tty_at_driver->minor_start = 0;
	gsm8ch_tty_at_driver->type = TTY_DRIVER_TYPE_SERIAL;
	gsm8ch_tty_at_driver->subtype = SERIAL_TYPE_NORMAL;
	gsm8ch_tty_at_driver->init_termios = tty_std_termios;
	gsm8ch_tty_at_driver->init_termios.c_cflag = B9600 | CS8 | HUPCL | CLOCAL | CREAD;
	gsm8ch_tty_at_driver->init_termios.c_ispeed = 9600;
	gsm8ch_tty_at_driver->init_termios.c_ospeed = 9600;
	gsm8ch_tty_at_driver->flags = TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;
	tty_set_operations(gsm8ch_tty_at_driver, &gsm8ch_tty_at_ops);

	if ((rc = tty_register_driver(gsm8ch_tty_at_driver))) {
		log(KERN_ERR, "can't register gsm8ch_tty_at driver: rc=%d\n", rc);
		// release allocated tty driver environment
		put_tty_driver(gsm8ch_tty_at_driver);
		gsm8ch_tty_at_driver = NULL;
		goto gsm8ch_init_error;
	}
	debug("tty_at_major=%d\n", gsm8ch_tty_at_driver->major);

	// Register PCI driver
	if ((rc = pci_register_driver(&gsm8ch_pci_driver)) < 0) {
		log(KERN_ERR, "can't register pci driver\n");
		goto gsm8ch_init_error;
	}

	verbose("loaded successfull\n");
	return 0;

gsm8ch_init_error:
	if (gsm8ch_tty_at_driver) {
		tty_unregister_driver(gsm8ch_tty_at_driver);
		put_tty_driver(gsm8ch_tty_at_driver);
	}
	return rc;
}

static void __exit gsm8ch_exit(void)
{
	// Unregister PCI driver
	pci_unregister_driver(&gsm8ch_pci_driver);

	// unregistering tty device for AT-command channel
	tty_unregister_driver(gsm8ch_tty_at_driver);
	put_tty_driver(gsm8ch_tty_at_driver);

	verbose("stopped\n");
}

module_init(gsm8ch_init);
module_exit(gsm8ch_exit);

/******************************************************************************/
/* end of gsm8ch-base.c                                                       */
/******************************************************************************/
