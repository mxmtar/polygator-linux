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
MODULE_DESCRIPTION("Polygator Linux module for FXS device");
MODULE_LICENSE("GPL");

#define verbose(_fmt, _args...) printk(KERN_INFO "[polygator-%s] " _fmt, THIS_MODULE->name, ## _args)
#define log(_level, _fmt, _args...) printk(_level "[polygator-%s] %s:%d - %s(): " _fmt, THIS_MODULE->name, "fxs-base.c", __LINE__, __PRETTY_FUNCTION__, ## _args)
#define debug(_fmt, _args...) printk(KERN_DEBUG "[polygator-%s] %s:%d - %s(): " _fmt, THIS_MODULE->name, "fxs-base.c", __LINE__, __PRETTY_FUNCTION__, ## _args)

#define MB_CS_ROM_KROSS			0x4000
#define MB_RESET_ROM			0x4800

#define FXS_BOARD_MAX_COUNT		5
#define FXS_BOARD_TYPE_FXS8		0x5301

#define FXS_BOARD_MEM_BASE		0x1100
#define FXS_BOARD_MEM_LENGTH	0x0200

#define FXS_BOARD_VIN_MEM_BASE		0x0000
#define FXS_BOARD_VIN_MEM_LENGTH	0x0020

#define FXS_BOARD_REG_RESET		0x00F0
#define FXS_BOARD_REG_TYPE		0x00E0
#define FXS_BOARD_REG_VERSION	0x00D0
#define FXS_BOARD_REG_TEST0		0x0080
#define FXS_BOARD_REG_TEST1		0x00B0

struct fxs_board {

	struct polygator_board *pg_board;
	struct cdev cdev;

	char name[POLYGATOR_BRDNAME_MAXLEN];

	u_int16_t type;
	u_int16_t version;

	size_t vinetics_count;
	struct vinetic *vinetics[2];
};

struct fxs_board_private_data {
	struct fxs_board *board;
	char buff[0x0C00];
	size_t length;
};

static char mainboard_rom[256];
static struct fxs_board *fxs_boards[FXS_BOARD_MAX_COUNT];

static struct resource *fxs_cs3_iomem_reg = NULL;
static void __iomem *fxs_cs3_base_ptr = NULL;

static void fxs_vinetic_reset(uintptr_t cbdata)
{
	iowrite16(0, cbdata + 0x1e);
	mdelay(10);
	iowrite16(1, cbdata + 0x1e);
	mdelay(2);
}
static void fxs_vinetic_write_nwd(uintptr_t cbdata, u_int16_t value)
{
	iowrite16(value, cbdata + 0x04);
}
static void fxs_vinetic_write_eom(uintptr_t cbdata, u_int16_t value)
{
	iowrite16(value, cbdata + 0x06);
}
static u_int16_t fxs_vinetic_read_nwd(uintptr_t cbdata)
{
	u_int16_t value = ioread16(cbdata + 0x04);
	return value;
}
static u_int16_t fxs_vinetic_read_eom(uintptr_t cbdata)
{
	u_int16_t value = ioread16(cbdata + 0x06);
	return value;
}
static size_t fxs_vinetic_is_not_ready(uintptr_t cbdata)
{
	union vin_reg_ir reg_ir;
	reg_ir.full = ioread16(cbdata + 0x18);
	return reg_ir.bits.rdyq;
}
static u_int16_t fxs_vinetic_read_dia(uintptr_t cbdata)
{
	u_int16_t value = ioread16(cbdata + 0x18);
	return value;
}

static int fxs_board_open(struct inode *inode, struct file *filp)
{
	ssize_t res;
	size_t i, j;
	size_t len;

	struct fxs_board *brd;
	struct vinetic *vin;
	struct fxs_board_private_data *private_data;

	brd = container_of(inode->i_cdev, struct fxs_board, cdev);

	if (!(private_data = kmalloc(sizeof(struct fxs_board_private_data), GFP_KERNEL))) {
		log(KERN_ERR, "can't get memory=%lu bytes\n", (unsigned long int)sizeof(struct fxs_board_private_data));
		res = -ENOMEM;
		goto fxs_open_error;
	}
	private_data->board = brd;

	len = 0;

	// fxs
	for (j = 0; j < 2; j++) {
		for (i = 0; i < 4; i++) {
			len += sprintf(private_data->buff+len, "FXS%lu VIN%luALM%lu\r\n", (unsigned long int)(j * 4 + i), (unsigned long int)j, (unsigned long int)i);
		}
	}
	// vinetic
	for (j = 0; j < brd->vinetics_count; j++) {
		if ((vin = brd->vinetics[j])) {
			len += sprintf(private_data->buff + len, "VIN%lu %s\r\n",
								(unsigned long int)j,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
								dev_name(vin->device)
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
								dev_name(vin->device)
#else
								vin->device->class_id
#endif
								);
			for (i = 0; i < 4; i++) {
				if (vin->rtp_channels[i]) {
					len += sprintf(private_data->buff + len, "VIN%luRTP%lu %s\r\n",
									(unsigned long int)j,
									(unsigned long int)i,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
									dev_name(vin->rtp_channels[i]->device)
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
									dev_name(vin->rtp_channels[i]->device)
#else
									vin->rtp_channels[i]->device->class_id
#endif
									);
				}
			}
		}
	}

	private_data->length = len;

	filp->private_data = private_data;

	return 0;

fxs_open_error:
	if (private_data) {
		kfree(private_data);
	}
	return res;
}

static int fxs_board_release(struct inode *inode, struct file *filp)
{
	struct fxs_board_private_data *private_data = filp->private_data;

	kfree(private_data);
	return 0;
}

static ssize_t fxs_board_read(struct file *filp, char __user *buff, size_t count, loff_t *offp)
{
	size_t len;
	ssize_t res;
	struct fxs_board_private_data *private_data = filp->private_data;

	res = (private_data->length > filp->f_pos)?(private_data->length - filp->f_pos):(0);

	if (res) {
		len = res;
		len = min(count, len);
		if (copy_to_user(buff, private_data->buff + filp->f_pos, len)) {
			res = -EINVAL;
			goto fxs_board_read_end;
		}
		*offp = filp->f_pos + len;
	}

fxs_board_read_end:
	return res;
}

static ssize_t fxs_board_write(struct file *filp, const char __user *buff, size_t count, loff_t *offp)
{
	ssize_t res;
	char cmd[256];
	size_t len;

	memset(cmd, 0, sizeof(cmd));
	len = sizeof(cmd) - 1;
	len = min(len, count);

	if (copy_from_user(cmd, buff, len)) {
		res = -EINVAL;
		goto fxs_board_write_end;
	}

	res = -ENOMSG;

fxs_board_write_end:
	return res;
}

static struct file_operations fxs_board_fops = {
	.owner   = THIS_MODULE,
	.open    = fxs_board_open,
	.release = fxs_board_release,
	.read    = fxs_board_read,
	.write   = fxs_board_write,
};

static int __init fxs_init(void)
{
	size_t i, j, k;
	struct fxs_board *brd;
	struct vinetic *vin;
	char devname[VINETIC_DEVNAME_MAXLEN];
	u32 tmpu32;
	int rc = 0;

	verbose("loading ...\n");

	for (k = 0; k < FXS_BOARD_MAX_COUNT; k++) {
		fxs_boards[k] = NULL;
	}
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
		goto fxs_init_error;
	}
	if (!(fxs_cs3_iomem_reg = request_mem_region(AT91_CHIPSELECT_3, 0x10000, "polygator"))) {
		log(KERN_ERR, "can't request i/o memory region for CS3\n");
		rc = -ENOMEM;
		goto fxs_init_error;
	}
	if (!(fxs_cs3_base_ptr = ioremap_nocache(AT91_CHIPSELECT_3, 0x10000))) {
		log(KERN_ERR, "can't remap i/o memory for CS3\n");
		rc = -ENOMEM;
		goto fxs_init_error;
	}

	// Read ROM from mainboard
	iowrite8(0, fxs_cs3_base_ptr + MB_RESET_ROM);
	mdelay(1);
	iowrite8(1, fxs_cs3_base_ptr + MB_RESET_ROM);
	mdelay(1);
	iowrite8(0, fxs_cs3_base_ptr + MB_RESET_ROM);
	for (i = 0; i < sizeof(mainboard_rom); i++) {
		mainboard_rom[i] = ioread8(fxs_cs3_base_ptr + MB_CS_ROM_KROSS);
	}
	verbose("mainboard: \"%s\"\n", &mainboard_rom[3]);

	// test board slots
	for (k = 0; k < FXS_BOARD_MAX_COUNT; k++) {
		// alloc memory for board data
		if (!(brd = kmalloc(sizeof(struct fxs_board), GFP_KERNEL))) {
			log(KERN_ERR, "can't get memory for struct fxs_board\n");
			rc = -1;
			goto fxs_init_error;
		}
		memset(brd, 0, sizeof(struct fxs_board));
		// Reset FXS board
		iowrite16(0, fxs_cs3_base_ptr + FXS_BOARD_MEM_BASE + k * FXS_BOARD_MEM_LENGTH + FXS_BOARD_REG_RESET);
		mdelay(10);
		iowrite16(1, fxs_cs3_base_ptr + FXS_BOARD_MEM_BASE + k * FXS_BOARD_MEM_LENGTH + FXS_BOARD_REG_RESET);
		// check for FXS board present
		iowrite16(0x5555, fxs_cs3_base_ptr + FXS_BOARD_MEM_BASE + k * FXS_BOARD_MEM_LENGTH + FXS_BOARD_REG_TEST0);
		iowrite16(0xaaaa, fxs_cs3_base_ptr + FXS_BOARD_MEM_BASE + k * FXS_BOARD_MEM_LENGTH + FXS_BOARD_REG_TEST1);
		if ((ioread16(fxs_cs3_base_ptr + FXS_BOARD_MEM_BASE + k * FXS_BOARD_MEM_LENGTH + FXS_BOARD_REG_TEST0) == 0x5555) &&
			(ioread16(fxs_cs3_base_ptr + FXS_BOARD_MEM_BASE + k * FXS_BOARD_MEM_LENGTH + FXS_BOARD_REG_TEST1) == 0xaaaa) &&
			(ioread16(fxs_cs3_base_ptr + FXS_BOARD_MEM_BASE + k * FXS_BOARD_MEM_LENGTH + FXS_BOARD_REG_TYPE) == FXS_BOARD_TYPE_FXS8)) {
			if (k == 0) {
				snprintf(brd->name, POLYGATOR_BRDNAME_MAXLEN, "board-fxs8-ll");
			} else if (k == 1) {
				snprintf(brd->name, POLYGATOR_BRDNAME_MAXLEN, "board-fxs8-lr");
			} else if (k == 2) {
				snprintf(brd->name, POLYGATOR_BRDNAME_MAXLEN, "board-fxs8-rl");
			} else if (k == 3) {
				snprintf(brd->name, POLYGATOR_BRDNAME_MAXLEN, "board-fxs8-rr");
			} else {
				snprintf(brd->name, POLYGATOR_BRDNAME_MAXLEN, "board-fxs8-cx");
			}
			brd->type = FXS_BOARD_TYPE_FXS8;
			brd->vinetics_count = 2;
		} else {
			// board not present
			kfree(brd);
			continue;
		}
		verbose("found %s (ver.%u)", brd->name, ioread16(fxs_cs3_base_ptr + FXS_BOARD_MEM_BASE + k * FXS_BOARD_MEM_LENGTH + FXS_BOARD_REG_VERSION));
		fxs_boards[k] = brd;
		// register vinetics
		for (j = 0; j < brd->vinetics_count; j++) {
			snprintf(devname, VINETIC_DEVNAME_MAXLEN, "%s-vin%lu", brd->name, (unsigned long int)j);
			if (!(brd->vinetics[j] = vinetic_device_register(THIS_MODULE, devname, (uintptr_t)(fxs_cs3_base_ptr + FXS_BOARD_MEM_BASE + k * FXS_BOARD_MEM_LENGTH + FXS_BOARD_VIN_MEM_BASE + j * FXS_BOARD_VIN_MEM_LENGTH),
																fxs_vinetic_reset,
																fxs_vinetic_is_not_ready,
																fxs_vinetic_write_nwd,
																fxs_vinetic_write_eom,
																fxs_vinetic_read_nwd,
																fxs_vinetic_read_eom,
																fxs_vinetic_read_dia))) {
				rc = -1;
				goto fxs_init_error;
			}
			for (i = 0; i < 4; i++) {
				snprintf(devname, VINETIC_DEVNAME_MAXLEN, "%s-vin%lu-rtp%lu", brd->name, (unsigned long int)j, (unsigned long int)i);
				if (!(vinetic_rtp_channel_register(THIS_MODULE, devname, brd->vinetics[j], i))) {
					rc = -1;
					goto fxs_init_error;
				}
			}
		}
	}
	// change board cyclic position 4 -> 2, 2 -> 3, 3 -> 4
	brd = fxs_boards[4];
	fxs_boards[4] = fxs_boards[3];
	fxs_boards[3] = fxs_boards[2];
	fxs_boards[2] = brd;
	// register board
	for (k = 0; k < FXS_BOARD_MAX_COUNT; k++) {
		if ((brd = fxs_boards[k])) {
			if (!(brd->pg_board =  polygator_board_register(THIS_MODULE, brd->name, &brd->cdev, &fxs_board_fops))) {
				rc = -1;
				goto fxs_init_error;
			}
		}
	}
	verbose("loaded successfull\n");
	return rc;

fxs_init_error:
	for (k = 0; k < FXS_BOARD_MAX_COUNT; k++) {
		if ((brd = fxs_boards[k])) {
			for (j = 0; j < brd->vinetics_count; j++) {
				if ((vin = brd->vinetics[j])) {
					for (i = 0; i < 4; i++) {
						if (vin->rtp_channels[i]) {
							vinetic_rtp_channel_unregister(vin->rtp_channels[i]);
						}
					}
					vinetic_device_unregister(vin);
				}
			}
			if (brd->pg_board) {
				polygator_board_unregister(brd->pg_board);
			}
			kfree(brd);
		}
	}

	if (fxs_cs3_iomem_reg) {
		release_mem_region(AT91_CHIPSELECT_3, 0x10000);
	}
	if (fxs_cs3_base_ptr) {
		iounmap(fxs_cs3_base_ptr);
	}
	return rc;
}

static void __exit fxs_exit(void)
{
	size_t i, j, k;
	struct fxs_board *brd;
	struct vinetic *vin;

	for (k = 0; k < FXS_BOARD_MAX_COUNT; k++) {
		if ((brd = fxs_boards[k])) {
			for (j = 0; j < brd->vinetics_count; j++) {
				if ((vin = brd->vinetics[j])) {
					for (i = 0; i < 4; i++) {
						if (vin->rtp_channels[i]) {
							vinetic_rtp_channel_unregister(vin->rtp_channels[i]);
						}
					}
					vinetic_device_unregister(vin);
				}
			}
			if (brd->pg_board) {
				polygator_board_unregister(brd->pg_board);
			}
			kfree(brd);
		}
	}

	release_mem_region(AT91_CHIPSELECT_3, 0x10000);
	iounmap(fxs_cs3_base_ptr);

	verbose("stopped\n");
}

module_init(fxs_init);
module_exit(fxs_exit);
