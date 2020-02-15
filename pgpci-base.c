#include <linux/kobject.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
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
#include "polygator/polygator-types.h"

#include "polygator/vinetic-base.h"
#include "polygator/vinetic-def.h"

#include "polygator/simcard-base.h"

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,36) // 2,6,30 - orig
#define TTY_PORT
#endif

MODULE_AUTHOR("Maksym Tarasevych <mxmtar@gmail.com>");
MODULE_DESCRIPTION("Linux module for Polygator Gateway PCI boards");
MODULE_LICENSE("GPL");

#define verbose(_fmt, _args...) printk(KERN_INFO "[polygator-%s] " _fmt, THIS_MODULE->name, ## _args)
#define log(_level, _fmt, _args...) printk(_level "[polygator-%s] %s:%d - %s(): " _fmt, THIS_MODULE->name, "pgpci-base.c", __LINE__, __PRETTY_FUNCTION__, ## _args)
#define debug(_fmt, _args...) printk(KERN_DEBUG "[polygator-%s] %s:%d - %s(): " _fmt, THIS_MODULE->name, "pgpci-base.c", __LINE__, __PRETTY_FUNCTION__, ## _args)

#define PGPCI_UART_CLOCK 36864000 * 4

union board_control_reg {
    struct {
        uint32_t reset:1;
        uint32_t reserved:31;
    } __attribute__((packed)) bits;
    uint32_t full;
} __attribute__((packed));

union radio_module_control_reg {
	struct {
		uint32_t power_supply:1;
		uint32_t power_key:1;
		uint32_t reset:1;
		uint32_t reserved:29;
	} __attribute__((packed)) bits;
	uint32_t full;
} __attribute__((packed));

union radio_module_uart_control_reg {
	struct {
		uint32_t reset:1;
		uint32_t csize:2;
		uint32_t cstopb:1;
		uint32_t parenb:1;
		uint32_t parodd:1;
		uint32_t cread:1;
		uint32_t reserved:24;
		uint32_t loopback:1;
	} __attribute__((packed)) bits;
	uint32_t full;
} __attribute__((packed));

union radio_module_smart_card_control_reg {
    struct {
        uint32_t reset:1;
        uint32_t enable:1;
        uint32_t inverse:1;
        uint32_t etu:11;
        uint32_t egt:8;
        uint32_t reserved:10;
    } __attribute__((packed)) bits;
    uint32_t full;
} __attribute__((packed));

union radio_module_status_reg {
	struct {
		uint32_t status:1;
		uint32_t reserved:31;
	} __attribute__((packed)) bits;
	uint32_t full;
} __attribute__((packed));

union radio_module_uart_tx_status_reg {
    struct {
        uint32_t wp:11;
        uint32_t rp:11;
        uint32_t fl:1;
        uint32_t reserved:9;
    } __attribute__((packed)) bits;
    uint32_t full;
} __attribute__((packed));

union radio_module_uart_rx_status_reg {
    struct {
        uint32_t wp:11;
        uint32_t rp:11;
        uint32_t fl:1;
        uint32_t reserved:8;
        uint32_t valid:1;
    } __attribute__((packed)) bits;
    uint32_t full;
} __attribute__((packed));

union radio_module_smart_card_status_reg {
    struct {
        uint32_t reset:1;
        uint32_t reserved:31;
    } __attribute__((packed)) bits;
    uint32_t full;
} __attribute__((packed));

union radio_module_smart_card_tx_status_reg {
    struct {
        uint32_t wp:9;
        uint32_t rp:9;
        uint32_t fl:1;
        uint32_t reserved:13;
    } __attribute__((packed)) bits;
    uint32_t full;
} __attribute__((packed));

union radio_module_smart_card_rx_status_reg {
    struct {
        uint32_t wp:8;
        uint32_t rp:8;
        uint32_t fl:1;
        uint32_t reserved:15;
    } __attribute__((packed)) bits;
    uint32_t full;
} __attribute__((packed));

struct pgpci_board;

struct radio_module_data {

	uint32_t type;

	struct pgpci_board *board;
	size_t position;

    spinlock_t lock;

    int power_on_id;
    union radio_module_control_reg control_data;
    union radio_module_uart_control_reg uart_control_data;
    uint32_t uart_btu_data;
    union radio_module_smart_card_control_reg smart_card_control_data;
    union radio_module_uart_tx_status_reg uart_tx_status;

	// uart section
	int at_port_select;
	spinlock_t at_lock;
	int at_no_buf;
#ifdef TTY_PORT
	struct tty_port at_port;
#else
	size_t at_count;
	struct tty_struct *at_tty;
#endif
	struct timer_list uart_poll_timer;
	u_int8_t uart_rx_buf[2048];
};

struct pgpci_board {

	struct polygator_board *pg_board;
	struct cdev cdev;

	int iomem_req;
	void __iomem *iomem_base;
	u8 irq_line;
	u8 irq_pin;

	uint32_t xw_version;
	uint32_t serial_number;
	uint32_t position;

	union board_control_reg control_data;

	struct vinetic *vinetics[2];

	struct radio_module_data *gsm_modules[8];

	struct polygator_tty_device *tty_at_channels[8];

	struct simcard_device *simcard_channels[8];

	union radio_module_status_reg radio_module_status[8];
};

struct pgpci_board_private_data {
	struct pgpci_board *board;
	char buff[10000];
	size_t length;
};

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
static struct pci_device_id pgpci_board_id_table[] = {
	{ PCI_DEVICE(0xdead, 0xbede), .driver_data = 1, },
	{ 0, },
};
MODULE_DEVICE_TABLE(pci, pgpci_board_id_table);

static void k32pci_vin_reset_0(uintptr_t cbdata)
{
	void __iomem *addr = (void __iomem *)cbdata;
	iowrite8(0, addr + 0x0000 + 0x11c00);
	mdelay(10);
	iowrite8(1, addr + 0x0000 + 0x11c00);
}

static void k32pci_vin_reset_1(uintptr_t cbdata)
{
	void __iomem *addr = (void __iomem *)cbdata;
	iowrite8(0, addr + 0x2000 + 0x11c00);
	mdelay(10);
	iowrite8(1, addr + 0x2000 + 0x11c00);
}

static void k32pci_vin_write_nwd_0(uintptr_t cbdata, u_int16_t value)
{
	void __iomem *addr = (void __iomem *)cbdata;
	iowrite16(value, addr + 0x11000 + 0x0000  + 0x080);
}

static void k32pci_vin_write_nwd_1(uintptr_t cbdata, u_int16_t value)
{
	void __iomem *addr = (void __iomem *)cbdata;
	iowrite16(value, addr + 0x11000 + 0x2000 + 0x080);
}

static void k32pci_vin_write_eom_0(uintptr_t cbdata, u_int16_t value)
{
	void __iomem *addr = (void __iomem *)cbdata;
	iowrite16(value, addr + 0x11000 + 0x0000 + 0x0c0);
}

static void k32pci_vin_write_eom_1(uintptr_t cbdata, u_int16_t value)
{
	void __iomem *addr = (void __iomem *)cbdata;
	iowrite16(value, addr + 0x11000 + 0x2000 + 0x0c0);
}

static u_int16_t k32pci_vin_read_nwd_0(uintptr_t cbdata)
{
	u_int16_t value;
	void __iomem *addr = (void __iomem *)cbdata;
	iowrite16((cbdata + 0x11000 + 0x0000 + 0x080) & 0xffff, addr + 0x0000 + 0x11700);
	udelay(1);
	value = ioread16(addr + 0x0000 + 0x11000);
	return value;
}

static u_int16_t k32pci_vin_read_nwd_1(uintptr_t cbdata)
{
	u_int16_t value;
	void __iomem *addr = (void __iomem *)cbdata;
	iowrite16((cbdata + 0x11000 + 0x2000 + 0x080) & 0xffff, addr + 0x2000 + 0x11700);
	udelay(1);
	value = ioread16(addr + 0x2000 + 0x11000);
	return value;
}

static u_int16_t k32pci_vin_read_eom_0(uintptr_t cbdata)
{
	u_int16_t value;
	void __iomem *addr = (void __iomem *)cbdata;
	iowrite16((cbdata + 0x11000 + 0x0000 + 0x0c0) & 0xffff, addr + 0x0000 + 0x11700);
	udelay(1);
	value = ioread16(addr + 0x0000 + 0x11000);
	return value;
}

static u_int16_t k32pci_vin_read_eom_1(uintptr_t cbdata)
{
	u_int16_t value;
	void __iomem *addr = (void __iomem *)cbdata;
	iowrite16((cbdata + 0x11000 + 0x2000 + 0x0c0) & 0xffff, addr + 0x2000 + 0x11700);
	udelay(1);
	value = ioread16(addr + 0x2000 + 0x11000);
	return value;
}

static size_t k32pci_vin_is_not_ready_0(uintptr_t cbdata)
{
	void __iomem *addr = (void __iomem *)cbdata;
	size_t st;

	st = ioread16(addr + 0x0000 + 0x11200) & 1;

	return st;
}

static size_t k32pci_vin_is_not_ready_1(uintptr_t cbdata)
{
	void __iomem *addr = (void __iomem *)cbdata;
	size_t st;

	st = ioread16(addr + 0x2000 + 0x11200) & 1;

	return st;
}

static u_int16_t k32pci_vin_read_dia_0(uintptr_t cbdata)
{
	u_int16_t value;
	void __iomem *addr = (void __iomem *)cbdata;
	iowrite16((cbdata + 0x11000 + 0x0000 + 0x300) & 0xffff, addr + 0x0000 + 0x11700);
	udelay(1);
	value = ioread16(addr + 0x0000 + 0x11000);
	return value;
}

static u_int16_t k32pci_vin_read_dia_1(uintptr_t cbdata)
{
	u_int16_t value;
	void __iomem *addr = (void __iomem *)cbdata;
	iowrite16((cbdata + 0x11000 + 0x2000 + 0x300) & 0xffff, addr + 0x2000 + 0x11700);
	udelay(1);
	value = ioread16(addr + 0x2000 + 0x11000);
	return value;
}


static int pgpci_sim_is_reset_requested(void *cbdata)
{
    union radio_module_smart_card_status_reg status;
    struct radio_module_data *mod = (struct radio_module_data *)cbdata;
    struct pgpci_board *board = mod->board;

    status.full = ioread32(board->iomem_base + 0x002a0 + ((mod->position & 7) << 2));

    return status.bits.reset;
}

static size_t pgpci_sim_get_write_room(void *cbdata)
{
    union radio_module_smart_card_tx_status_reg smart_card_tx_status;
    struct radio_module_data *mod = (struct radio_module_data *)cbdata;
    struct pgpci_board *board = mod->board;
    size_t res = 0;

    smart_card_tx_status.full = ioread32(board->iomem_base + 0x002c0 + ((mod->position & 7) << 2));

    if (smart_card_tx_status.bits.fl) {
        res = 0;
    } else {
        if (smart_card_tx_status.bits.rp > smart_card_tx_status.bits.wp) {
            res = smart_card_tx_status.bits.rp - smart_card_tx_status.bits.wp;
        } else if (smart_card_tx_status.bits.wp > smart_card_tx_status.bits.rp) {
            res = 512 + smart_card_tx_status.bits.rp - smart_card_tx_status.bits.wp;
        } else {
            res = 512;
        }
    }

    return res;
}

static size_t pgpci_sim_write(void *cbdata, uint8_t *data, size_t length)
{
    union radio_module_smart_card_tx_status_reg smart_card_tx_status;
    struct radio_module_data *mod = (struct radio_module_data *)cbdata;
    struct pgpci_board *board = mod->board;
    size_t i, chunk;

    smart_card_tx_status.full = ioread32(board->iomem_base + 0x002c0 + ((mod->position & 7) << 2));

    if (smart_card_tx_status.bits.fl) {
        chunk = 0;
    } else {
        if (smart_card_tx_status.bits.rp > smart_card_tx_status.bits.wp) {
            chunk = smart_card_tx_status.bits.rp - smart_card_tx_status.bits.wp;
        } else if (smart_card_tx_status.bits.wp > smart_card_tx_status.bits.rp) {
            chunk = 512 + smart_card_tx_status.bits.rp - smart_card_tx_status.bits.wp;
        } else {
            chunk = 512;
        }
    }
    length = min(length, chunk);

    i = 0;
    while (i < length) {
        if (smart_card_tx_status.bits.rp > smart_card_tx_status.bits.wp) {
            chunk = smart_card_tx_status.bits.rp - smart_card_tx_status.bits.wp;
        } else {
            chunk = 512 - smart_card_tx_status.bits.wp;
        }
        chunk = min(chunk, length - i);
        memcpy_toio(board->iomem_base + 0x88000 + ((mod->position & 7) << 16) + smart_card_tx_status.bits.wp, data + i, chunk);
        smart_card_tx_status.bits.wp += chunk;
        i += chunk;
    }

    return length;
}

static size_t pgpci_sim_read(void *cbdata, uint8_t *data, size_t length)
{
    union radio_module_smart_card_rx_status_reg smart_card_rx_status;
    struct radio_module_data *mod = (struct radio_module_data *)cbdata;
    struct pgpci_board *board = mod->board;
    size_t res, i, chunk;

    smart_card_rx_status.full = ioread32(board->iomem_base + 0x002e0 + ((mod->position & 7) << 2));

    if (smart_card_rx_status.bits.fl) {
        res = 256;
    } else if (smart_card_rx_status.bits.wp > smart_card_rx_status.bits.rp) {
        res = smart_card_rx_status.bits.wp - smart_card_rx_status.bits.rp;
    } else if (smart_card_rx_status.bits.wp < smart_card_rx_status.bits.rp) {
        res = 256 + smart_card_rx_status.bits.wp - smart_card_rx_status.bits.rp;
    } else  {
        res = 0;
    }

    res = min(res, length);

    // read received data
    if (res) {
        i = 0;
        while (i < res) {
            if ((smart_card_rx_status.bits.fl) || (smart_card_rx_status.bits.wp < smart_card_rx_status.bits.rp)) {
                chunk = 256 - smart_card_rx_status.bits.rp;
            } else {
                chunk = smart_card_rx_status.bits.wp - smart_card_rx_status.bits.rp;
            }
            chunk = min(chunk, res - i);
            memcpy_fromio(data + i, board->iomem_base + 0x8c000 + ((mod->position & 7) << 16) + smart_card_rx_status.bits.rp, chunk);
            smart_card_rx_status.bits.rp += chunk;
            i += chunk;
        }
        iowrite32(smart_card_rx_status.full, board->iomem_base + 0x002e0 + ((mod->position & 7) << 2));
    }

    return res;
}

static void pgpci_sim_set_etu_count(void *cbdata, uint32_t etu)
{
    struct radio_module_data *mod = (struct radio_module_data *)cbdata;
    struct pgpci_board *board = mod->board;

    switch (etu) {
        case 0x94:
            mod->smart_card_control_data.bits.etu = 63;
            break;
        case 0x95:
            mod->smart_card_control_data.bits.etu = 31;
            break;
        case 0x96:
            mod->smart_card_control_data.bits.etu = 15;
            break;
        default:
            mod->smart_card_control_data.bits.etu = 371;
            break;
    }

    iowrite32(mod->smart_card_control_data.full, board->iomem_base + 0x00160 + ((mod->position & 7) << 2));
}

static void pgpci_power_on(void *cbdata)
{
	struct radio_module_data *mod = (struct radio_module_data *)cbdata;
	struct pgpci_board *board = mod->board;

	spin_lock(&mod->lock);
	mod->power_on_id = -1;
	mod->control_data.bits.power_supply = 1;
	iowrite32(mod->control_data.full, board->iomem_base + 0x00100 + ((mod->position & 7) << 2));
	spin_unlock(&mod->lock);
}

static void pgpci_uart_poll(struct timer_list *timer)
{
	size_t i, len, chunk;
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,9,0)
	struct tty_struct *tty;
#endif
	union radio_module_uart_rx_status_reg rx_status;

    struct radio_module_data *mod = container_of(timer, struct radio_module_data, uart_poll_timer);
	struct pgpci_board *board = mod->board;

	spin_lock(&mod->at_lock);

	// get uart tx buffer status
	mod->uart_tx_status.full = ioread32(board->iomem_base + 0x00240 + ((mod->position & 7) << 2));

	// check for data in uart rx buffer
	rx_status.full = ioread32(board->iomem_base + 0x00260 + ((mod->position & 7) << 2));

	if (rx_status.bits.valid) {
		if (rx_status.bits.fl) {
			len = 2048;
		} else if (rx_status.bits.wp > rx_status.bits.rp) {
			len = rx_status.bits.wp - rx_status.bits.rp;
		} else if (rx_status.bits.wp < rx_status.bits.rp) {
			len = 2048 + rx_status.bits.wp - rx_status.bits.rp;
		} else  {
			len = 0;
		}
	} else {
		len = 0;
	}

	spin_unlock(&mod->at_lock);

	// read received data
	if (len) {
		i = 0;
		while (i < len) {
			if ((rx_status.bits.fl) || (rx_status.bits.wp < rx_status.bits.rp)) {
				chunk = 2048 - rx_status.bits.rp;
			} else {
				chunk = rx_status.bits.wp - rx_status.bits.rp;
			}
			chunk = min(chunk, len - i);
			memcpy_fromio(mod->uart_rx_buf + i, board->iomem_base + 0x84000 + ((mod->position & 7) << 16) + rx_status.bits.rp, chunk);
			rx_status.bits.rp += chunk;
			i += chunk;
		}
		iowrite32(rx_status.full, board->iomem_base + 0x00260 + ((mod->position & 7) << 2));
	}

	if (len) {
#ifdef TTY_PORT
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,9,0)
		tty_insert_flip_string(&mod->at_port, mod->uart_rx_buf, len);
		tty_flip_buffer_push(&mod->at_port);
#else
		tty = tty_port_tty_get(&mod->at_port);
		tty_insert_flip_string(tty, mod->uart_rx_buf, len);
		tty_flip_buffer_push(tty);
		tty_kref_put(tty);
#endif
#else
		tty = mod->at_tty;
		tty_insert_flip_string(tty, mod->uart_rx_buf, len);
		tty_flip_buffer_push(tty);
#endif
	}

	mod_timer(&mod->uart_poll_timer, jiffies + 1);
}


static irqreturn_t pgpci_board_interrupt(int irq, void *data)
{
	irqreturn_t res = IRQ_NONE;

	return res;
}

static inline uint32_t pgpci_baudrate_to_btu(speed_t baudrate)
{
	uint32_t btu, rem;

	btu = PGPCI_UART_CLOCK / baudrate;
	rem = PGPCI_UART_CLOCK % baudrate;
	if (rem < (baudrate >> 1)) {
		btu -= 1;
	}

	return btu;
}

static int pgpci_board_open(struct inode *inode, struct file *filp)
{
	ssize_t res;
	size_t i, j;
	size_t len;

	struct pgpci_board *board;
	struct pgpci_board_private_data *private_data;
	struct radio_module_data *mod;

	board = container_of(inode->i_cdev, struct pgpci_board, cdev);

	if (!(private_data = kmalloc(sizeof(struct pgpci_board_private_data), GFP_KERNEL))) {
		log(KERN_ERR, "can't get memory=%lu bytes\n", (unsigned long int)sizeof(struct pgpci_board_private_data));
		res = -ENOMEM;
		goto pgpci_board_open_error;
	}
	private_data->board = board;

	// get radio_module_status
	memcpy_fromio(board->radio_module_status, board->iomem_base + 0x00220, sizeof(union radio_module_status_reg) * 8);

	len = 0;
	// type
	len += sprintf(private_data->buff + len, "{\r\n\t\"hardware\": %u,", (board->xw_version >> 16) & 0xffff);
	// firmware
	len += sprintf(private_data->buff + len, "\r\n\t\"firmware\": \"%u.%u.%u\",", (board->xw_version >> 8) & 0xff, (board->xw_version >> 4) & 0xf, board->xw_version & 0xf);
	if (board->serial_number != 0xffffffff) {
		len += sprintf(private_data->buff + len, "\r\n\t\"msn\": %u,", board->serial_number);
	}
	// position
	len += sprintf(private_data->buff + len, "\r\n\t\"position\": %u,", board->position);
	// radio channels
	len += sprintf(private_data->buff + len, "\r\n\t\"channels\": [");
	for (i = 0; i < 8; ++i) {
		if ((mod = board->gsm_modules[i])) {
			len += sprintf(private_data->buff + len, "%s\r\n\t\t{\
														\r\n\t\t\t\"tty\": \"%s\",\
														\r\n\t\t\t\"sim\": \"%s\",\
														\r\n\t\t\t\"module\": \"%s\",\
														\r\n\t\t\t\"power\": \"%s\",\
														\r\n\t\t\t\"key\": \"%s\",\
														\r\n\t\t\t\"status\": \"%s\",\
														\r\n\t\t\t\"audio\": {\
														\r\n\t\t\t\t\"vinetic\": %lu,\
														\r\n\t\t\t\t\"rtp\": %lu\
														\r\n\t\t\t}\
														\r\n\t\t}",
							i ? "," : "",
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
							board->tty_at_channels[i]?dev_name(board->tty_at_channels[i]->device):"unknown",
							board->simcard_channels[i]?dev_name(board->simcard_channels[i]->device):"unknown",
#else
							board->tty_at_channels[i]?board->tty_at_channels[i]->device->class_id:"unknown",
							board->simcard_channels[i]?board->simcard_channels[i]->device->class_id:"unknown",
#endif
							polygator_print_gsm_module_type(mod->type),
							mod->control_data.bits.power_supply ? "on" : "off",
							mod->control_data.bits.power_key ? "on" : "off",
							board->radio_module_status[i].bits.status ? "on" : "off",
							(unsigned long int)(i / 4),
							(unsigned long int)(i % 4));
		}
	}
	len += sprintf(private_data->buff + len, "\r\n\t],");
	// vinetic
	len += sprintf(private_data->buff + len, "\r\n\t\"vinetic\": [");
	for (i = 0; i < 2; ++i) {
		if (board->vinetics[i]) {
			len += sprintf(private_data->buff + len, "%s\r\n\t\t{\r\n\t\t\t\"path\": \"%s\",",
							i ? "," : "",
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
							dev_name(board->vinetics[i]->device)
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
							dev_name(board->vinetics[i]->device)
#else
							board->vinetics[i]->device->class_id
#endif
							);
			len += sprintf(private_data->buff + len, "\r\n\t\t\t\"rtp\": [");
			for (j = 0; j < 4; ++j) {
				if (board->vinetics[i]->rtp_channels[j])
					len += sprintf(private_data->buff + len, "%s\r\n\t\t\t\t{\r\n\t\t\t\t\t\"path\": \"%s\"\r\n\t\t\t\t}",
								j ? "," : "",
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
								dev_name(board->vinetics[i]->rtp_channels[j]->device)
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
								dev_name(board->vinetics[i]->rtp_channels[j]->device)
#else
								board->vinetics[i]->rtp_channels[j]->device->class_id
#endif
								);
			}
			len += sprintf(private_data->buff + len, "\r\n\t\t\t]");
		}
		len += sprintf(private_data->buff + len, "\r\n\t\t}");
	}
	len += sprintf(private_data->buff + len, "\r\n\t]");
	len += sprintf(private_data->buff + len, "\r\n}\r\n");

	private_data->length = len;

	filp->private_data = private_data;

	return 0;

pgpci_board_open_error:
	if (private_data) {
		kfree(private_data);
	}
	return res;
}

static int pgpci_board_release(struct inode *inode, struct file *filp)
{
	struct pgpci_board_private_data *private_data = filp->private_data;

	kfree(private_data);
	return 0;
}

static ssize_t pgpci_board_read(struct file *filp, char __user *buff, size_t count, loff_t *offp)
{
	size_t len;
	ssize_t res;
	struct pgpci_board_private_data *private_data = filp->private_data;

	res = (private_data->length > filp->f_pos)?(private_data->length - filp->f_pos):(0);

	if (res) {
		len = res;
		len = min(count, len);
		if (copy_to_user(buff, private_data->buff + filp->f_pos, len)) {
			res = -EINVAL;
			goto pgpci_board_read_end;
		}
		*offp = filp->f_pos + len;
	}

pgpci_board_read_end:
	return res;
}

static ssize_t pgpci_board_write(struct file *filp, const char __user *buff, size_t count, loff_t *offp)
{
	ssize_t res;
	char cmd[256];
	size_t len;
	char buf[256];

	uint32_t chan;
	uint32_t value;
	struct radio_module_data *mod;
	struct pgpci_board_private_data *private_data = filp->private_data;
	struct pgpci_board *board = private_data->board;

	memset(cmd, 0, sizeof(cmd));
	len = sizeof(cmd) - 1;
	len = min(len,count);

	if (copy_from_user(cmd, buff, len)) {
		res = -EINVAL;
		goto pgpci_board_write_end;
	}

	if (sscanf(cmd, "channel[%u].power_supply(%u)", &chan, &value) == 2) {
		if ((chan >= 0) && (chan <= 7) && (board->gsm_modules[chan])) {
			mod = board->gsm_modules[chan];
			spin_lock_bh(&mod->lock);
			if (value) {
				if (mod->control_data.bits.power_supply == 0) {
					if (mod->power_on_id == -1) {
						res = polygator_power_on_schedule(pgpci_power_on, mod);
						if (res >= 0) {
							mod->power_on_id = res;
							res = len;
						}
					} else {
						res = -EAGAIN;
					}
				} else {
					res = len;
				}
			} else {
				if (mod->power_on_id != -1) {
					polygator_power_on_cancel(mod->power_on_id);
					mod->power_on_id = -1;
				}
				mod->control_data.bits.power_supply = 0;
				iowrite32(mod->control_data.full, board->iomem_base + 0x00100 + ((mod->position & 7) << 2));
				res = len;
			}
			spin_unlock_bh(&mod->lock);
		} else {
			res = -ENODEV;
		}
	} else if (sscanf(cmd, "channel[%u].power_key(%u)", &chan, &value) == 2) {
		if ((chan >= 0) && (chan <= 7) && (board->gsm_modules[chan])) {
			mod = board->gsm_modules[chan];
			spin_lock_bh(&mod->lock);
			mod->control_data.bits.power_key = value;
			iowrite32(mod->control_data.full, board->iomem_base + 0x00100 + ((mod->position & 7) << 2));
			spin_unlock_bh(&mod->lock);
			res = len;
		} else {
			res = -ENODEV;
		}
	} else if (sscanf(cmd, "channel[%u].uart.baudrate(%u)", &chan, &value) == 2) {
		if ((chan >= 0) && (chan <= 7) && (board->gsm_modules[chan])) {
			mod = board->gsm_modules[chan];
			mod->uart_btu_data = pgpci_baudrate_to_btu(value);
			iowrite32(mod->uart_btu_data, board->iomem_base + 0x00140 + ((mod->position & 7) << 2));
			mod->uart_control_data.bits.reset = 1;
			iowrite32(mod->uart_control_data.full, board->iomem_base + 0x00120 + ((mod->position & 7) << 2));
			udelay(1);
			mod->uart_control_data.bits.reset = 0;
			iowrite32(mod->uart_control_data.full, board->iomem_base + 0x00120 + ((mod->position & 7) << 2));
			res = len;
		} else {
			res = -ENODEV;
		}
	} else if (sscanf(cmd, "channel[%u].uart.loopback(%u)", &chan, &value) == 2) {
		if ((chan >= 0) && (chan <= 7) && (board->gsm_modules[chan])) {
			mod = board->gsm_modules[chan];
			mod->uart_control_data.bits.loopback = value;
			iowrite32(mod->uart_control_data.full, board->iomem_base + 0x00120 + ((mod->position & 7) << 2));
			res = len;
		} else {
			res = -ENODEV;
		}
    } else if (sscanf(cmd, "channel[%u].smart_card.enable(%u)", &chan, &value) == 2) {
        if ((chan >= 0) && (chan <= 7) && (board->gsm_modules[chan])) {
            mod = board->gsm_modules[chan];
            mod->smart_card_control_data.bits.enable = value;
            iowrite32(mod->smart_card_control_data.full, board->iomem_base + 0x00160 + ((mod->position & 7) << 2));
            res = len;
        } else {
            res = -ENODEV;
        }
	} else if (sscanf(cmd, "channel[%u].testpoint(%u)", &chan, &value) == 2) {
		if ((chan >= 0) && (chan <= 7)) {
			iowrite32((((chan & 0xff) << 8) + (value & 0xff)), board->iomem_base + 0x000e0);
			res = len;
		} else {
			res = -ENODEV;
		}
	} else if (sscanf(cmd, "board.led[%u].mode(%u)", &chan, &value) == 2) {
		iowrite32(value, board->iomem_base + 0x000a0 + ((chan & 3) << 2));
		res = len;
	} else if (sscanf(cmd, "board.write(0x%x,0x%x)", &chan, &value) == 2) {
		iowrite32(value, board->iomem_base + chan);
		res = len;
	} else if (sscanf(cmd, "board.read(0x%x)", &chan) == 1) {
		verbose("read[0x%08x]=0x%08x\n", chan, ioread32(board->iomem_base + chan));
		res = len;
	} else if (sscanf(cmd, "board.fromio(0x%x,%u)", &chan, &value) == 2) {
		memcpy_fromio(buf, board->iomem_base + chan, min(sizeof(buf), (size_t)value));
		verbose("read[0x%08x]=0x%02x\n", chan, buf[0]);
		res = len;
	} else {
		res = -ENOMSG;
	}

pgpci_board_write_end:
	return res;
}

static struct file_operations pgpci_board_fops = {
	.owner   = THIS_MODULE,
	.open    = pgpci_board_open,
	.release = pgpci_board_release,
	.read    = pgpci_board_read,
	.write   = pgpci_board_write,
};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0)
static int pgpci_board_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
#else
static int __devinit pgpci_board_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
#endif
{
	int rc = -1;
	size_t i, j;
	int pci_irq_requested = 0;
	char devname[POLYGATOR_BRDNAME_MAXLEN];
	struct radio_module_data *mod;
	struct pgpci_board *board = NULL;

	// alloc memory for board data
	if (!(board = kmalloc(sizeof(struct pgpci_board), GFP_KERNEL))) {
		log(KERN_ERR, "can't get memory for struct pgpci_board\n");
		goto pgpci_board_probe_error;
	}
	memset(board, 0, sizeof(struct pgpci_board));

	rc = pci_enable_device(pdev);
	if (rc) {
		dev_err(&pdev->dev, "can't enable pci device\n");
		goto pgpci_board_probe_error;
	}

	rc = pci_request_region(pdev, 0, "pgpci");
	if (rc) {
		dev_err(&pdev->dev, "can't request I/O region\n");
		goto pgpci_board_probe_error;
	}
	board->iomem_req = 1;
	if (!(board->iomem_base = pci_iomap(pdev, 0, pci_resource_end(pdev, 0) - pci_resource_start(pdev, 0) + 1))) {
		dev_err(&pdev->dev, "can't request i/o memory region\n");
		rc = -ENOMEM;
		goto pgpci_board_probe_error;
	}
	board->xw_version = ioread32(board->iomem_base + 0x00000);
	board->serial_number = ioread32(board->iomem_base + 0x00020);
	board->position = ioread32(board->iomem_base + 0x00040);
	if (board->serial_number != 0xffffffff) {
		verbose("found board hw %u fw %u.%u.%u sn %u\n", (board->xw_version >> 16) & 0xffff, (board->xw_version >> 8) & 0xff, (board->xw_version >> 4) & 0xf, board->xw_version & 0xf, board->serial_number);
	} else {
		verbose("found board hw %u fw %u.%u.%u\n", (board->xw_version >> 16) & 0xffff, (board->xw_version >> 8) & 0xff, (board->xw_version >> 4) & 0xf, board->xw_version & 0xf);
	}
	

	// reset board
	board->control_data.bits.reset = 1;
	iowrite32(board->control_data.full, board->iomem_base + 0x00080);
	udelay(1);
	board->control_data.bits.reset = 0;
	iowrite32(board->control_data.full, board->iomem_base + 0x00080);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
	snprintf(devname, POLYGATOR_BRDNAME_MAXLEN, "board-pgpci-%02x%02x%x", pdev->bus->number, PCI_SLOT(pdev->devfn), PCI_FUNC(pdev->devfn));
#else
	snprintf(devname, POLYGATOR_BRDNAME_MAXLEN, "bp%02x%02x%x", pdev->bus->number, PCI_SLOT(pdev->devfn), PCI_FUNC(pdev->devfn));
#endif

	if (!(board->pg_board = polygator_board_register(&pdev->dev, THIS_MODULE, devname, &board->cdev, &pgpci_board_fops))) {
		rc = -1;
		goto pgpci_board_probe_error;
	}

	for (j = 0; j < 2; ++j) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
		snprintf(devname, VINETIC_DEVNAME_MAXLEN, "board-pgpci-%02x%02x%x-vin%lu", pdev->bus->number, PCI_SLOT(pdev->devfn), PCI_FUNC(pdev->devfn), (unsigned long int)j);
#else
		snprintf(devname, VINETIC_DEVNAME_MAXLEN, "v%02x%02x%x%lu", board->serial_number, board->position, (unsigned long int)j);
#endif
		if (!(board->vinetics[j] = vinetic_device_register(THIS_MODULE, devname, ((uintptr_t)board->iomem_base) + (board->position * 0x4000),
													(j)?(k32pci_vin_reset_1):(k32pci_vin_reset_0),
													(j)?(k32pci_vin_is_not_ready_1):(k32pci_vin_is_not_ready_0),
													(j)?(k32pci_vin_write_nwd_1):(k32pci_vin_write_nwd_0),
													(j)?(k32pci_vin_write_eom_1):(k32pci_vin_write_eom_0),
													(j)?(k32pci_vin_read_nwd_1):(k32pci_vin_read_nwd_0),
													(j)?(k32pci_vin_read_eom_1):(k32pci_vin_read_eom_0),
													(j)?(k32pci_vin_read_dia_1):(k32pci_vin_read_dia_0)))) {
			rc = -1;
			goto pgpci_board_probe_error;
		}
		for (i = 0; i < 4; ++i) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
			snprintf(devname, VINETIC_DEVNAME_MAXLEN, "board-pgpci-%02x%02x%x-vin%lu-rtp%lu", pdev->bus->number, PCI_SLOT(pdev->devfn), PCI_FUNC(pdev->devfn), (unsigned long int)j, (unsigned long int)i);
#else
			snprintf(devname, VINETIC_DEVNAME_MAXLEN, "r%02x%02x%x%lu%lu", pdev->bus->number, PCI_SLOT(pdev->devfn), PCI_FUNC(pdev->devfn), (unsigned long int)j, (unsigned long int)i);
#endif
			if (!vinetic_rtp_channel_register(THIS_MODULE, devname, board->vinetics[j], i)) {
				rc = -1;
				goto pgpci_board_probe_error;
			}
		}
	}

	// set radio module data
	for (i = 0; i < 8; ++i) {
		if (!(mod = kmalloc(sizeof(struct radio_module_data), GFP_KERNEL))) {
			log(KERN_ERR, "can't get memory for struct radio_module_data\n");
			rc = -1;
			goto pgpci_board_probe_error;
		}
		memset(mod, 0, sizeof(struct radio_module_data));

		// check for supported radio module type
		mod->type = ioread32(board->iomem_base  + 0x00060 + ((i & 7) << 2));
		mod->type &= 0xffff;
		if ((mod->type != POLYGATOR_MODULE_TYPE_SIM300) && (mod->type != POLYGATOR_MODULE_TYPE_SIM900) &&
			(mod->type != POLYGATOR_MODULE_TYPE_SIM5215) && (mod->type == POLYGATOR_MODULE_TYPE_SIM5215A2) &&
			(mod->type != POLYGATOR_MODULE_TYPE_M10)) {
			kfree(mod);
			continue;
		}

        mod->board = board;
        mod->position = i;

        spin_lock_init(&mod->lock);

        // init radio module control register
        mod->power_on_id = -1;
        mod->control_data.full = ioread32(board->iomem_base + 0x00100 + ((i & 7) << 2));

        // init radio module uart control register
        mod->uart_btu_data = pgpci_baudrate_to_btu(115200);
        iowrite32(mod->uart_btu_data, board->iomem_base + 0x00140 + ((i & 7) << 2));
        mod->uart_control_data.bits.reset = 1;
        mod->uart_control_data.bits.csize = 3;
        mod->uart_control_data.bits.cstopb = 0;
        mod->uart_control_data.bits.cread = 1;
        mod->uart_control_data.bits.parenb = 0;
        mod->uart_control_data.bits.parodd = 0;
        mod->uart_control_data.bits.loopback = 0;
        iowrite32(mod->uart_control_data.full, board->iomem_base + 0x00120 + ((i & 7) << 2));
        mod->uart_control_data.bits.reset = 0;
        iowrite32(mod->uart_control_data.full, board->iomem_base + 0x00120 + ((i & 7) << 2));

        timer_setup(&mod->uart_poll_timer, pgpci_uart_poll, 0);

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
    for (i = 0; i < 8; ++i) {
        if ((mod = board->gsm_modules[i])) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0)
            if (!(board->tty_at_channels[i] = polygator_tty_device_register(&pdev->dev, mod, &mod->at_port, &k32pci_tty_at_ops))) {
#else
            if (!(board->tty_at_channels[i] = polygator_tty_device_register(&pdev->dev, mod, &k32pci_tty_at_ops))) {
#endif
                log(KERN_ERR, "can't register polygator tty device\n");
                rc = -1;
                goto pgpci_board_probe_error;
            }
        }
    }

    // register polygator simcard device
    for (i = 0; i < 8; ++i) {
        if ((mod = board->gsm_modules[i])) {
            if (!(board->simcard_channels[i] = simcard_device_register2(THIS_MODULE, mod))) {
                log(KERN_ERR, "can't register polygator simcard device\n");
                rc = -1;
                goto pgpci_board_probe_error;
            } else {
                simcard_device_set_is_reset_requested(board->simcard_channels[i], pgpci_sim_is_reset_requested);
                simcard_device_set_get_write_room(board->simcard_channels[i], pgpci_sim_get_write_room);
                simcard_device_set_write2(board->simcard_channels[i], pgpci_sim_write);
                simcard_device_set_read2(board->simcard_channels[i], pgpci_sim_read);
                simcard_device_set_etu_count(board->simcard_channels[i], pgpci_sim_set_etu_count);
                // init smart card control register
                mod->smart_card_control_data.bits.reset = 0;
                mod->smart_card_control_data.bits.enable = 0;
                mod->smart_card_control_data.bits.inverse = 0;
                mod->smart_card_control_data.bits.etu = 371;
                mod->smart_card_control_data.bits.egt = 0;
                iowrite32(mod->smart_card_control_data.full, board->iomem_base + 0x00160 + ((i & 7) << 2));
            }
        }
    }

    // reset board
    board->control_data.bits.reset = 1;
    iowrite32(board->control_data.full, board->iomem_base + 0x00080);
    udelay(1);
    board->control_data.bits.reset = 0;
    iowrite32(board->control_data.full, board->iomem_base + 0x00080);

	// set interrupt handler
	if (!(rc = pci_read_config_byte(pdev, PCI_INTERRUPT_PIN, &board->irq_pin))) {
		if (board->irq_pin) {
			if (!(rc = pci_read_config_byte(pdev, PCI_INTERRUPT_LINE, &board->irq_line))) {
				if ((rc = request_irq(pdev->irq, pgpci_board_interrupt, IRQF_SHARED, "pgpci", board))) {
					log(KERN_ERR, "%s: Unable to request IRQ %d (error %d)\n", "pgpci", pdev->irq, rc);
					goto pgpci_board_probe_error;
				} else {
					pci_irq_requested = 1;
				}
			} else {
				dev_err(&pdev->dev, "pci_read_config_byte(pdev, PCI_INTERRUPT_LINE, &board->irq_pin) error=%d\n", rc);
				goto pgpci_board_probe_error;
			}
		} else {
			log(KERN_ERR, "PG PCI board must drive at least one interrupt pin\n");
			rc = -1;
			goto pgpci_board_probe_error;
		}
	} else {
		dev_err(&pdev->dev, "pci_read_config_byte(pdev, PCI_INTERRUPT_PIN, &board->irq_pin) error=%d\n", rc);
		goto pgpci_board_probe_error;
	}

	pci_set_drvdata(pdev, board);

	return 0;

pgpci_board_probe_error:

	if (board) {
		if (pci_irq_requested) {
			free_irq(pdev->irq, board);
		}
		for (i = 0; i < 8; ++i) {
			if (board->simcard_channels[i]) {
				simcard_device_unregister(board->simcard_channels[i]);
			}
			if (board->tty_at_channels[i]) {
				polygator_tty_device_unregister(board->tty_at_channels[i]);
			}
			if (board->gsm_modules[i]) {
				del_timer_sync(&board->gsm_modules[i]->uart_poll_timer);
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
static void pgpci_board_remove(struct pci_dev *pdev)
#else
static void __devexit pgpci_board_remove(struct pci_dev *pdev)
#endif
{
	size_t i, j;

	struct pgpci_board *board = pci_get_drvdata(pdev);

	free_irq(pdev->irq, board);

	for (i = 0; i < 8; ++i) {
		if (board->simcard_channels[i]) {
			simcard_device_unregister(board->simcard_channels[i]);
		}
		if (board->tty_at_channels[i]) {
			polygator_tty_device_unregister(board->tty_at_channels[i]);
		}
		if (board->gsm_modules[i]) {
			del_timer_sync(&board->gsm_modules[i]->uart_poll_timer);
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

static struct pci_driver pgpci_driver = {
	.name = "pgpci",
	.id_table = pgpci_board_id_table,
	.probe = pgpci_board_probe,
	.remove = pgpci_board_remove,
};

static int k32pci_tty_at_open(struct tty_struct *tty, struct file *filp)
{
	struct polygator_tty_device *ptd = tty->driver_data;
	struct radio_module_data *mod = (struct radio_module_data *)ptd->data;

#ifdef TTY_PORT
	return tty_port_open(&mod->at_port, tty, filp);
#else
	spin_lock_bh(&mod->at_lock);
	if (!mod->at_count++) {
		mod_timer(&mod->uart_poll_timer, jiffies + 1);
		mod->at_tty = tty;
	}
	spin_unlock_bh(&mod->at_lock);
	return 0;
#endif
}

static void k32pci_tty_at_close(struct tty_struct *tty, struct file *filp)
{
	struct polygator_tty_device *ptd = tty->driver_data;
	struct radio_module_data *mod = (struct radio_module_data *)ptd->data;

#ifdef TTY_PORT
	tty_port_close(&mod->at_port, tty, filp);
#else
	spin_lock_bh(&mod->at_lock);
	if (!--mod->at_count) {
		del_timer_sync(&mod->uart_poll_timer);
		mod->at_tty = NULL;
	}
	spin_unlock_bh(&mod->at_lock);
#endif
	return;
}

static int k32pci_tty_at_write(struct tty_struct *tty, const unsigned char *buf, int count)
{
	int res = 0;
	size_t i, len, chunk;
	struct polygator_tty_device *ptd = tty->driver_data;
	struct radio_module_data *mod = (struct radio_module_data *)ptd->data;
	struct pgpci_board *board = mod->board;

	spin_lock_bh(&mod->at_lock);

	if (mod->uart_tx_status.bits.fl) {
		res = 0;
	} else {
		if (mod->uart_tx_status.bits.rp > mod->uart_tx_status.bits.wp) {
			res = mod->uart_tx_status.bits.rp - mod->uart_tx_status.bits.wp;
		} else if (mod->uart_tx_status.bits.wp > mod->uart_tx_status.bits.rp) {
			res = 2048 + mod->uart_tx_status.bits.rp - mod->uart_tx_status.bits.wp;
		} else {
			res = 2048;
		}
	}
	len = res = min(res, count);

	i = 0;
	while (i < len) {
		if (mod->uart_tx_status.bits.rp > mod->uart_tx_status.bits.wp) {
			chunk = mod->uart_tx_status.bits.rp - mod->uart_tx_status.bits.wp;
		} else {
			chunk = 2048 - mod->uart_tx_status.bits.wp;
		}
		chunk = min(chunk, len - i);
		memcpy_toio(board->iomem_base + 0x80000 + ((mod->position & 7) << 16) + mod->uart_tx_status.bits.wp, buf + i, chunk);
		mod->uart_tx_status.bits.wp += chunk;
		i += chunk;
	}

	if (mod->uart_tx_status.bits.wp == mod->uart_tx_status.bits.rp) {
		mod->uart_tx_status.bits.fl = 1;
	}

	spin_unlock_bh(&mod->at_lock);

	return res ;
}

static int k32pci_tty_at_write_room(struct tty_struct *tty)
{
	int res;
	struct polygator_tty_device *ptd = tty->driver_data;
	struct radio_module_data *mod = (struct radio_module_data *)ptd->data;

	spin_lock_bh(&mod->at_lock);

	if (mod->uart_tx_status.bits.fl) {
		res = 0;
	} else {
		if (mod->uart_tx_status.bits.rp > mod->uart_tx_status.bits.wp) {
			res = mod->uart_tx_status.bits.rp - mod->uart_tx_status.bits.wp;
		} else if (mod->uart_tx_status.bits.wp > mod->uart_tx_status.bits.rp) {
			res = 2048 + mod->uart_tx_status.bits.rp - mod->uart_tx_status.bits.wp;
		} else {
			res = 2048;
		}
	}

	spin_unlock_bh(&mod->at_lock);

	return res;
}

static int k32pci_tty_at_chars_in_buffer(struct tty_struct *tty)
{
	int res;
	struct polygator_tty_device *ptd = tty->driver_data;
	struct radio_module_data *mod = (struct radio_module_data *)ptd->data;

	spin_lock_bh(&mod->at_lock);

	if (mod->uart_tx_status.bits.fl) {
		res = 2048;
	} else {
		if (mod->uart_tx_status.bits.wp > mod->uart_tx_status.bits.rp) {
			res = mod->uart_tx_status.bits.wp - mod->uart_tx_status.bits.rp;
		} else if (mod->uart_tx_status.bits.rp > mod->uart_tx_status.bits.wp) {
			res = 2048 + mod->uart_tx_status.bits.wp - mod->uart_tx_status.bits.rp;
		} else {
			res = 0;
		}
	}

	spin_unlock_bh(&mod->at_lock);

	return res;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20)
static void k32pci_tty_at_set_termios(struct tty_struct *tty, struct ktermios *old_termios)
#else
static void k32pci_tty_at_set_termios(struct tty_struct *tty, struct termios *old_termios)
#endif
{
	speed_t baudrate;
	struct polygator_tty_device *ptd = tty->driver_data;
	struct radio_module_data *mod = (struct radio_module_data *)ptd->data;
	struct pgpci_board *board = mod->board;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20)
	struct ktermios *termios = &tty->termios;
#else
	struct termios *termios = &tty->termios;
#endif

	baudrate = tty_get_baud_rate(tty);

	spin_lock_bh(&mod->at_lock);

	mod->uart_btu_data = pgpci_baudrate_to_btu(baudrate);
	if ((termios->c_cflag & CSIZE) == CS8) {
		mod->uart_control_data.bits.csize = 3;
	} else if ((termios->c_cflag & CSIZE) == CS7) {
		mod->uart_control_data.bits.csize = 2;
	} else if ((termios->c_cflag & CSIZE) == CS6) {
		mod->uart_control_data.bits.csize = 1;
	} else {
		mod->uart_control_data.bits.csize = 0;
	}
	mod->uart_control_data.bits.cstopb = (termios->c_cflag & CSTOPB) ? 1 : 0;
	mod->uart_control_data.bits.cread = (termios->c_cflag & CREAD) ? 1 : 0;
	mod->uart_control_data.bits.parenb = (termios->c_cflag & PARENB) ? 1 : 0;
	mod->uart_control_data.bits.parodd = (termios->c_cflag & PARODD) ? 1 : 0;

	iowrite32(mod->uart_btu_data, board->iomem_base + 0x00140 + ((mod->position & 7) << 2));
	mod->uart_control_data.bits.reset = 1;
	iowrite32(mod->uart_control_data.full, board->iomem_base + 0x00120 + ((mod->position & 7) << 2));
	udelay(1);
	mod->uart_control_data.bits.reset = 0;
	iowrite32(mod->uart_control_data.full, board->iomem_base + 0x00120 + ((mod->position & 7) << 2));

	spin_unlock_bh(&mod->at_lock);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
	tty_encode_baud_rate(tty, baudrate, baudrate);
#endif
}

static void k32pci_tty_at_flush_buffer(struct tty_struct *tty)
{
	struct polygator_tty_device *ptd = tty->driver_data;
	struct radio_module_data *mod = (struct radio_module_data *)ptd->data;
	struct pgpci_board *board = mod->board;

	spin_lock_bh(&mod->at_lock);
	mod->uart_tx_status.full = 0;
	mod->uart_control_data.bits.reset = 1;
	iowrite32(mod->uart_control_data.full, board->iomem_base + 0x00120 + ((mod->position & 7) << 2));
	udelay(1);
	mod->uart_control_data.bits.reset = 0;
	iowrite32(mod->uart_control_data.full, board->iomem_base + 0x00120 + ((mod->position & 7) << 2));
	spin_unlock_bh(&mod->at_lock);
	tty_wakeup(tty);
}

static void k32pci_tty_at_hangup(struct tty_struct *tty)
{
#ifdef TTY_PORT
	struct polygator_tty_device *ptd = tty->driver_data;
	struct radio_module_data *mod = (struct radio_module_data *)ptd->data;
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
	struct radio_module_data *mod = container_of(port, struct radio_module_data, at_port);

    mod_timer(&mod->uart_poll_timer, jiffies + 1);

	return 0;
}

static void k32pci_tty_at_port_shutdown(struct tty_port *port)
{
	struct radio_module_data *mod = container_of(port, struct radio_module_data, at_port);

	del_timer_sync(&mod->uart_poll_timer);
}
#endif
static int __init pgpci_init(void)
{
	int rc;

	verbose("loading ...\n");

	// Register PCI driver
	if ((rc = pci_register_driver(&pgpci_driver)) < 0) {
		log(KERN_ERR, "can't register pci driver\n");
		goto pgpci_init_error;
	}

	verbose("loaded successfull\n");
	return 0;

pgpci_init_error:
	return rc;
}

static void __exit pgpci_exit(void)
{
	// Unregister PCI driver
	pci_unregister_driver(&pgpci_driver);

	verbose("stopped\n");
}

module_init(pgpci_init);
module_exit(pgpci_exit);
