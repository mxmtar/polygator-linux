/******************************************************************************/
/* gsm8ch-base.c                                                              */
/******************************************************************************/

#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/poll.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/version.h>
#include <linux/vmalloc.h>

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

#define verbose(_fmt, _args...) printk(KERN_INFO "[polygator-%s] " _fmt, THIS_MODULE->name, ## _args)
#define log(_level, _fmt, _args...) printk(_level "[polygator-%s] %s:%d - %s(): " _fmt, THIS_MODULE->name, "gsm8ch-base.c", __LINE__, __PRETTY_FUNCTION__, ## _args)
#define debug(_fmt, _args...) printk(KERN_DEBUG "[polygator-%s] %s:%d - %s(): " _fmt, THIS_MODULE->name, "gsm8ch-base.c", __LINE__, __PRETTY_FUNCTION__, ## _args)

struct gsm8ch_board {

	struct polygator_board *pg_board;
	struct cdev cdev;

	u_int8_t rom[256];
	size_t romsize;
	u_int32_t sn;

	struct vinetic *vinetics[2];
};

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
}

static void gsm8ch_pci_reset_1(uintptr_t cbdata)
{
	outb(0x04, cbdata + PG_PCI_OFFSET_RESET);
	mdelay(10);
	outb(0x00, cbdata + PG_PCI_OFFSET_RESET);
}

static void gsm8ch_pci_write_nwd_0(uintptr_t cbdata, u_int16_t value)
{
	outw(value, cbdata + PG_PCI_VIN_DATA_BASE + 0 + 0);
// 	debug("%04x\n", value);
}

static void gsm8ch_pci_write_nwd_1(uintptr_t cbdata, u_int16_t value)
{
	outw(value, cbdata + PG_PCI_VIN_DATA_BASE + 0 + 0);
// 	debug("%04x\n", value);
}

static void gsm8ch_pci_write_eom_0(uintptr_t cbdata, u_int16_t value)
{
	outw(value, cbdata + PG_PCI_VIN_DATA_BASE + 0 + 4);
// 	debug("%04x\n", value);
}

static void gsm8ch_pci_write_eom_1(uintptr_t cbdata, u_int16_t value)
{
	outw(value, cbdata + PG_PCI_VIN_DATA_BASE + 0 + 4);
// 	debug("%04x\n", value);
}

static u_int16_t gsm8ch_pci_read_nwd_0(uintptr_t cbdata)
{
	u_int16_t value = inw(cbdata + PG_PCI_VIN_DATA_BASE + 0 + 0);
// 	debug("%04x\n", value);
	return value;
}

static u_int16_t gsm8ch_pci_read_nwd_1(uintptr_t cbdata)
{
	u_int16_t value = inw(cbdata + PG_PCI_VIN_DATA_BASE + 0 + 0);
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
	u_int16_t value = inw(cbdata + PG_PCI_VIN_DATA_BASE + 0 + 4);
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
// 	debug("%04x\n", reg_ir.full);
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

static int gsm8ch_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static int gsm8ch_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static struct file_operations gsm8ch_fops = {
	.owner   = THIS_MODULE,
	.open    = gsm8ch_open,
	.release = gsm8ch_release,
// 	.read    = gsm8ch_read,
// 	.write   = gsm8ch_write,
#if defined(HAVE_UNLOCKED_IOCTL)
// 	.unlocked_ioctl = gsm8ch_unlocked_ioctl,
#else
// 	.ioctl = gsm8ch_ioctl,
#endif
#if defined(CONFIG_COMPAT) && defined(HAVE_COMPAT_IOCTL) && (HAVE_COMPAT_IOCTL == 1)
// 	.compat_ioctl = gsm8ch_compat_ioctl,
#endif
// 	.llseek = gsm8ch_llseek,
};

static int __devinit gsm8ch_pci_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	int rc;
	unsigned long addr;
// 	u_int32_t no;
	u_int16_t type;
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
	type = 0;
	for (i=0; i<16; i++)
	{
		type <<= 1;
		type |= inb(addr + PG_PCI_ID_BASE) & 0x01;
	}
	verbose("found PCI board type=%04x\n", type & 0x00ff);

	if (((type & 0x00ff) != 0x0081) && ((type & 0x00ff) != 0x0082) && ((type & 0x00ff) != 0x0083)) {
		log(KERN_ERR, "PCI board type=%04x unsupported\n", type & 0x00ff);
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
	snprintf(devname, POLYGATOR_BRDNAME_MAXLEN, "brd-gsm8ch-pci-%u", brd->sn);
	if (!(brd->pg_board =  polygator_board_register(THIS_MODULE, devname, &brd->cdev, &gsm8ch_fops))) {
		rc = -1;
		goto gsm8ch_pci_probe_error;
	}

	for (j=0; j<2; j++)
	{
		snprintf(devname, POLYGATOR_BRDNAME_MAXLEN, "brd-gsm8ch-pci-%u-vin%lu", brd->sn, (unsigned long int)j);
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
			snprintf(devname, POLYGATOR_BRDNAME_MAXLEN, "brd-gsm8ch-pci-%u-vin%lu-rtp%lu", brd->sn, (unsigned long int)j, (unsigned long int)i);
			if (!vinetic_rtp_channel_register(THIS_MODULE, devname, brd->vinetics[j], i)) {
				rc = -1;
				goto gsm8ch_pci_probe_error;
			}
		}
	}

	pci_set_drvdata(pdev, brd);

	return 0;

gsm8ch_pci_probe_error:
	if (brd) {
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

static int __init gsm8ch_init(void)
{
	int rc;

	verbose("loading ...\n");

	// Register PCI driver
	if ((rc = pci_register_driver(&gsm8ch_pci_driver)) < 0) {
		log(KERN_ERR, "can't register pci driver\n");
		goto gsm8ch_init_error;
	}

	verbose("loaded successfull\n");
	return 0;

gsm8ch_init_error:
	return rc;
}

static void __exit gsm8ch_exit(void)
{
	// Unregister PCI driver
	pci_unregister_driver(&gsm8ch_pci_driver);

	verbose("stopped\n");
}

module_init(gsm8ch_init);
module_exit(gsm8ch_exit);

/******************************************************************************/
/* end of gsm8ch-base.c                                                       */
/******************************************************************************/
