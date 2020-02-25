/******************************************************************************/
/* fxo4-base.c                                                                */
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

#include <asm/io.h>
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

MODULE_AUTHOR("Maksym Tarasevych <mxmtar@gmail.com>");
MODULE_DESCRIPTION("Polygator Linux module for FXO4 device");
MODULE_LICENSE("GPL");

#define verbose(_fmt, _args...) printk(KERN_INFO "[polygator-%s] " _fmt, THIS_MODULE->name, ## _args)
#define log(_level, _fmt, _args...) printk(_level "[polygator-%s] %s:%d - %s(): " _fmt, THIS_MODULE->name, "fxo4-base.c", __LINE__, __PRETTY_FUNCTION__, ## _args)
#define debug(_fmt, _args...) printk(KERN_DEBUG "[polygator-%s] %s:%d - %s(): " _fmt, THIS_MODULE->name, "fxo4-base.c", __LINE__, __PRETTY_FUNCTION__, ## _args)

/*! */
#define FXO4_HOOK_0			0x1000
#define FXO4_HOOK_1			0x1020
#define FXO4_HOOK_2			0x1030
#define FXO4_HOOK_3			0x1040
#define FXO4_RING_0			0x1060
#define FXO4_RING_1			0x1070
#define FXO4_RING_2			0x1080
#define FXO4_RING_3			0x1090

#define FXO4_VINETIC_BASE	0x1100
#define FXO4_VINETIC_RESET	0x11C0

#define FXO4_BOARD_TEST_0	0x1180
#define FXO4_BOARD_TEST_1	0x11B0
#define FXO4_BOARD_TYPE		0x11E0	// 0xe907
#define FXO4_BOARD_RESET	0x11F0
/*! */

struct fxo4_board {

	struct polygator_board *pg_board;
	struct cdev cdev;

	uint8_t rom[256];
	size_t romsize;
	uint32_t sn;
	uint16_t type;

	struct vinetic *vinetic;
};

struct fxo4_board_private_data {
	struct fxo4_board *board;
	char buff[0x0C00];
	size_t length;
};

static struct fxo4_board *fxo4_board = NULL;

static struct resource *fxo4_cs3_iomem_reg = NULL;
static void __iomem *fxo4_cs3_base_ptr = NULL;

static void fxo4_vinetic_reset(uintptr_t cbdata)
{
	iowrite16(0, cbdata + 0xC0);
	mdelay(10);
	iowrite16(1, cbdata + 0xC0);
	mdelay(2);
}
static void fxo4_vinetic_write_nwd(uintptr_t cbdata, uint16_t value)
{
	iowrite16(value, cbdata + 0x04);
// 	log(KERN_INFO, "%08lx: %04x\n", cbdata + 0x04, value);
}
static void fxo4_vinetic_write_eom(uintptr_t cbdata, uint16_t value)
{
	iowrite16(value, cbdata + 0x06);
// 	log(KERN_INFO, "%08lx: %04x\n", cbdata + 0x06, value);
}
static uint16_t fxo4_vinetic_read_nwd(uintptr_t cbdata)
{
	uint16_t value = ioread16(cbdata + 0x04);
// 	log(KERN_INFO, "%08lx: %04x\n", cbdata + 0x04, value);
	return value;
}
static uint16_t fxo4_vinetic_read_eom(uintptr_t cbdata)
{
	uint16_t value = ioread16(cbdata + 0x06);
// 	log(KERN_INFO, "%08lx: %04x\n", cbdata + 0x06, value);
	return value;
}
static size_t fxo4_vinetic_is_not_ready(uintptr_t cbdata)
{
	union vin_reg_ir reg_ir;
	reg_ir.full = ioread16(cbdata + 0x18);
// 	log(KERN_INFO, "%08lx: %04x\n", cbdata + 0x18, reg_ir.full);
	return reg_ir.bits.rdyq;
}
static uint16_t fxo4_vinetic_read_dia(uintptr_t cbdata)
{
	uint16_t value = ioread16(cbdata + 0x18);
// 	log(KERN_INFO, "%08lx: %04x\n", cbdata + 0x18, value);
	return value;
}

static int fxo4_board_open(struct inode *inode, struct file *filp)
{
	ssize_t res;
	size_t i, j;
	size_t len;

	struct fxo4_board *brd;
	struct fxo4_board_private_data *private_data;

	brd = container_of(inode->i_cdev, struct fxo4_board, cdev);

	if (!(private_data = kmalloc(sizeof(struct fxo4_board_private_data), GFP_KERNEL))) {
		log(KERN_ERR, "can't get memory=%lu bytes\n", (unsigned long int)sizeof(struct fxo4_board_private_data));
		res = -ENOMEM;
		goto fxo4_open_error;
	}
	private_data->board = brd;

	len = 0;

	// fxo
	for (i = 0; i < 4; i++) {
		if (i == 0) {
			len += sprintf(private_data->buff+len, "FXO0 VIN0ALM0 RING=%u\r\n", ioread8(fxo4_cs3_base_ptr + FXO4_RING_0) & 1);
		} else if (i == 1) {
			len += sprintf(private_data->buff+len, "FXO1 VIN0ALM1 RING=%u\r\n", ioread8(fxo4_cs3_base_ptr + FXO4_RING_1) & 1);
		} else if (i == 2) {
			len += sprintf(private_data->buff+len, "FXO2 VIN0ALM2 RING=%u\r\n", ioread8(fxo4_cs3_base_ptr + FXO4_RING_2) & 1);
		} else if (i == 3) {
			len += sprintf(private_data->buff+len, "FXO3 VIN0ALM3 RING=%u\r\n", ioread8(fxo4_cs3_base_ptr + FXO4_RING_3) & 1);
		}
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

fxo4_open_error:
	if (private_data) {
		kfree(private_data);
	}
	return res;
}

static int fxo4_board_release(struct inode *inode, struct file *filp)
{
	struct fxo4_board_private_data *private_data = filp->private_data;

	kfree(private_data);
	return 0;
}

static ssize_t fxo4_board_read(struct file *filp, char __user *buff, size_t count, loff_t *offp)
{
	size_t len;
	ssize_t res;
	struct fxo4_board_private_data *private_data = filp->private_data;

	res = (private_data->length > filp->f_pos)?(private_data->length - filp->f_pos):(0);

	if (res) {
		len = res;
		len = min(count, len);
		if (copy_to_user(buff, private_data->buff + filp->f_pos, len)) {
			res = -EINVAL;
			goto fxo4_board_read_end;
		}
		*offp = filp->f_pos + len;
	}

fxo4_board_read_end:
	return res;
}

static ssize_t fxo4_board_write(struct file *filp, const char __user *buff, size_t count, loff_t *offp)
{
	ssize_t res;
	char cmd[256];
	size_t len;

	uint32_t ch;
	uint32_t value;
// 	struct fxo4_board_private_data *private_data = filp->private_data;

	memset(cmd, 0, sizeof(cmd));
	len = sizeof(cmd) - 1;
	len = min(len,count);

	if (copy_from_user(cmd, buff, len)) {
		res = -EINVAL;
		goto fxo4_board_write_end;
	}

	if (sscanf(cmd, "FXO%u HOOK=%u", &ch, &value) == 1) {
		if (ch == 0) {
			iowrite8(value & 1, fxo4_cs3_base_ptr + FXO4_HOOK_0);
			res = len;
		} else if (ch == 1) {
			iowrite8(value & 1, fxo4_cs3_base_ptr + FXO4_HOOK_1);
			res = len;
		} else if (ch == 2) {
			iowrite8(value & 1, fxo4_cs3_base_ptr + FXO4_HOOK_2);
			res = len;
		} else if (ch == 3) {
			iowrite8(value & 1, fxo4_cs3_base_ptr + FXO4_HOOK_3);
			res = len;
		} else {
			res= -ENODEV;
		}
	} else {
		res = -ENOMSG;
	}

fxo4_board_write_end:
	return res;
}

static struct file_operations fxo4_board_fops = {
	.owner   = THIS_MODULE,
	.open    = fxo4_board_open,
	.release = fxo4_board_release,
	.read    = fxo4_board_read,
	.write   = fxo4_board_write,
};

static int __init fxo4_init(void)
{
	size_t i;
	u32 tmpu32;
	char devname[VINETIC_DEVNAME_MAXLEN];
	int rc = 0;

	verbose("loading ...\n");

	// Assign CS3, CS4 to SMC
	tmpu32 = at91_sys_read(AT91_MATRIX_EBICSA);
	tmpu32 &= ~(AT91_MATRIX_CS3A | AT91_MATRIX_CS4A);
	tmpu32 |= (AT91_MATRIX_CS3A_SMC | AT91_MATRIX_CS4A_SMC);
	at91_sys_write(AT91_MATRIX_EBICSA, tmpu32);

	// Configure PIOC for using CS3, CS4
	at91_sys_write(AT91_PIOC + PIO_PDR, (1 << 14)|(1 << 8)); /* Disable Register */
	tmpu32 = at91_sys_read(AT91_PIOC + PIO_PSR); /* Status Register */
	at91_sys_write(AT91_PIOC + PIO_ASR, (1 << 14)|(1 << 8)); /* Peripheral A Select Register */
	tmpu32 = at91_sys_read(AT91_PIOC + PIO_ABSR); /* AB Status Register */

	// Configure SMC CS3 timings
	at91_sys_write(AT91_SMC + 0x30 + 0x0, 0x01030103);
	at91_sys_write(AT91_SMC + 0x30 + 0x4, 0x0f0c0f0c);
	at91_sys_write(AT91_SMC + 0x30 + 0x8, 0x00140014);
	at91_sys_write(AT91_SMC + 0x30 + 0xc, 0x10001003);

	// Request and remap i/o memory region for CS3
	if (check_mem_region(AT91_CHIPSELECT_3, 0x10000)) {
		log(KERN_ERR, "i/o memory region for CS3 already used\n");
		rc = -ENOMEM;
		goto fxo4_init_error;
	}
	if (!(fxo4_cs3_iomem_reg = request_mem_region(AT91_CHIPSELECT_3, 0x10000, "polygator_fxo4"))) {
		log(KERN_ERR, "can't request i/o memory region for CS3\n");
		rc = -ENOMEM;
		goto fxo4_init_error;
	}
	if (!(fxo4_cs3_base_ptr = ioremap_nocache(AT91_CHIPSELECT_3, 0x10000))) {
		log(KERN_ERR, "can't remap i/o memory for CS3\n");
		rc = -ENOMEM;
		goto fxo4_init_error;
	}

	// Reset FXO4 board
	iowrite8(0, fxo4_cs3_base_ptr + FXO4_BOARD_RESET);
	mdelay(10);
	iowrite8(1, fxo4_cs3_base_ptr + FXO4_BOARD_RESET);

	// alloc memory for board data
	if (!(fxo4_board = kmalloc(sizeof(struct fxo4_board), GFP_KERNEL))) {
		log(KERN_ERR, "can't get memory for struct fxo4_board\n");
		rc = -1;
		goto fxo4_init_error;
	}
	memset(fxo4_board, 0, sizeof(struct fxo4_board));

	snprintf(devname, POLYGATOR_BRDNAME_MAXLEN, "board-fxo4");
	if (!(fxo4_board->pg_board =  polygator_board_register(THIS_MODULE, devname, &fxo4_board->cdev, &fxo4_board_fops))) {
		rc = -1;
		goto fxo4_init_error;
	}
	// Register vinetic
	if (!(fxo4_board->vinetic = vinetic_device_register(THIS_MODULE, "board-fxo4-vin0", (uintptr_t)(fxo4_cs3_base_ptr + FXO4_VINETIC_BASE),
													fxo4_vinetic_reset,
													fxo4_vinetic_is_not_ready,
													fxo4_vinetic_write_nwd,
													fxo4_vinetic_write_eom,
													fxo4_vinetic_read_nwd,
													fxo4_vinetic_read_eom,
													fxo4_vinetic_read_dia))) {
		rc = -1;
		goto fxo4_init_error;
	}
	for (i = 0; i < 4; i++) {
		snprintf(devname, VINETIC_DEVNAME_MAXLEN, "board-fxo4-vin0-rtp%lu", (unsigned long int)i);
		if (!(vinetic_rtp_channel_register(THIS_MODULE, devname, fxo4_board->vinetic, i))) {
			rc = -1;
			goto fxo4_init_error;
		}
	}

	verbose("loaded successfull\n");
	return rc;

fxo4_init_error:
	if (fxo4_board) {
		if (fxo4_board->vinetic) {
			for (i = 0; i < 4; i++) {
				if (fxo4_board->vinetic->rtp_channels[i]) {
					vinetic_rtp_channel_unregister(fxo4_board->vinetic->rtp_channels[i]);
				}
			}
			vinetic_device_unregister(fxo4_board->vinetic);
		}
		if (fxo4_board->pg_board) {
			polygator_board_unregister(fxo4_board->pg_board);
		}
		kfree(fxo4_board);
	}

	if (fxo4_cs3_iomem_reg) {
		release_mem_region(AT91_CHIPSELECT_3, 0x10000);
	}
	if (fxo4_cs3_base_ptr) {
		iounmap(fxo4_cs3_base_ptr);
	}
	return rc;
}

static void __exit fxo4_exit(void)
{
	size_t i;

	for (i = 0; i < 4; i++) {
		if (fxo4_board->vinetic->rtp_channels[i]) {
			vinetic_rtp_channel_unregister(fxo4_board->vinetic->rtp_channels[i]);
		}
	}
	vinetic_device_unregister(fxo4_board->vinetic);
	polygator_board_unregister(fxo4_board->pg_board);
	kfree(fxo4_board);

	release_mem_region(AT91_CHIPSELECT_3, 0x10000);
	iounmap(fxo4_cs3_base_ptr);

	verbose("stopped\n");
}

module_init(fxo4_init);
module_exit(fxo4_exit);

/******************************************************************************/
/* end of fxo4-base.c                                                         */
/******************************************************************************/
