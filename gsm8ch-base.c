/******************************************************************************/
/* vinetic-base.c                                                             */
/******************************************************************************/

#include <linux/delay.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/poll.h>
#include <linux/spinlock.h>
#include <linux/version.h>
#include <linux/vmalloc.h>

#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <asm/io.h>
#if defined(CONFIG_COMPAT) && defined(HAVE_COMPAT_IOCTL) && (HAVE_COMPAT_IOCTL == 1)
#include <asm/compat.h>
#endif

#include "polygator/vinetic-base.h"
#include "polygator/vinetic-def.h"

MODULE_AUTHOR("Maksym Tarasevych <mxmtar@ukr.net>");
MODULE_DESCRIPTION("Polygator Linux module for gsm8ch boards");
MODULE_LICENSE("GPL");

#define verbose(_fmt, _args...) printk(KERN_INFO "[pg-%s] " _fmt, THIS_MODULE->name, ## _args)
#define log(_level, _fmt, _args...) printk(_level "[pg-%s] %s:%d - %s(): " _fmt, THIS_MODULE->name, "gsm8ch-base.c", __LINE__, __PRETTY_FUNCTION__, ## _args)
#define debug(_fmt, _args...) printk(KERN_DEBUG "[pg-%s] %s:%d - %s(): " _fmt, THIS_MODULE->name, "gsm8ch-base.c", __LINE__, __PRETTY_FUNCTION__, ## _args)


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

static struct vinetic * pci_board_vinetics[2];

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

static int __devinit gsm8ch_pci_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	int rc;
	unsigned long addr;
// 	u_int32_t no;
// 	u_int16_t id;

	rc = pci_enable_device(pdev);
	if (rc) {
		dev_err(&pdev->dev, "can't enable pci device\n");
		return rc;
	}
	rc = pci_request_region(pdev, 0, "vinetic");
	if (rc) {
		dev_err(&pdev->dev, "can't request I/O region\n");
		return rc;
	}

	// get starting address
	addr = pci_resource_start(pdev, 0);

	// reset board
	outb(0xff, addr + PG_PCI_OFFSET_RESET);
	mdelay(10);
	outb(0x00, addr + PG_PCI_OFFSET_RESET);

	pci_board_vinetics[0] = vinetic_device_register(THIS_MODULE, "vin0", addr,
													gsm8ch_pci_reset_0,
													gsm8ch_pci_is_not_ready_0,
													gsm8ch_pci_write_nwd_0,
													gsm8ch_pci_write_eom_0,
													gsm8ch_pci_read_nwd_0,
													gsm8ch_pci_read_eom_0,
													gsm8ch_pci_read_dia_0);
	vinetic_rtp_channel_register(THIS_MODULE, "vin0rtp0", pci_board_vinetics[0], 0);
	vinetic_rtp_channel_register(THIS_MODULE, "vin0rtp1", pci_board_vinetics[0], 1);
	vinetic_rtp_channel_register(THIS_MODULE, "vin0rtp2", pci_board_vinetics[0], 2);
	vinetic_rtp_channel_register(THIS_MODULE, "vin0rtp3", pci_board_vinetics[0], 3);

	pci_board_vinetics[1] = vinetic_device_register(THIS_MODULE, "vin1", addr,
													gsm8ch_pci_reset_1,
													gsm8ch_pci_is_not_ready_1,
												 	gsm8ch_pci_write_nwd_1,
												 	gsm8ch_pci_write_eom_1,
												 	gsm8ch_pci_read_nwd_1,
											 		gsm8ch_pci_read_eom_1,
													gsm8ch_pci_read_dia_1);
	vinetic_rtp_channel_register(THIS_MODULE, "vin1rtp0", pci_board_vinetics[1], 0);
	vinetic_rtp_channel_register(THIS_MODULE, "vin1rtp1", pci_board_vinetics[1], 1);
	vinetic_rtp_channel_register(THIS_MODULE, "vin1rtp2", pci_board_vinetics[1], 2);
	vinetic_rtp_channel_register(THIS_MODULE, "vin1rtp3", pci_board_vinetics[1], 3);

/*
	// get board number
	no = inb(addr + EGGSM_PCI_NUM_BASE) & 3;
	// get board id
	id = 0;
	for (i=0; i<EGGSM_BOARD_ID_LENGTH; i++)
	{
		id <<= 1;
		id |= inb(addr + EGGSM_PCI_ID_BASE) & 0x01;
	}

	if (((id & 0x00ff) == 0x0081) || ((id & 0x00ff) == 0x0082) || ((id & 0x00ff) == 0x0083)) {
		// init board
		if (eggsm_board_init(no, EGGSM_BOARD_TYPE_PCI, id, addr) < 0) {
			log(KERN_ERR, "can't init board PCI #%d: ID=%04x\n", no, id);
			pci_release_region(pdev, 0);
			return -1;
		}
		pci_set_drvdata(pdev, &eggsm_boards[no + EGGSM_BOARDS_PCI_START]);
	} else {
		log(KERN_INFO, "#%d: PCI board type ID=%04x unsupported\n", no, id);
		pci_release_region(pdev, 0);
		return -1;
	}
*/

	return 0;
}

static void __devexit gsm8ch_pci_remove(struct pci_dev *pdev)
{
	pci_get_drvdata(pdev);

	vinetic_rtp_channel_unregister(pci_board_vinetics[0]->rtp_channels[0]);
	vinetic_rtp_channel_unregister(pci_board_vinetics[0]->rtp_channels[1]);
	vinetic_rtp_channel_unregister(pci_board_vinetics[0]->rtp_channels[2]);
	vinetic_rtp_channel_unregister(pci_board_vinetics[0]->rtp_channels[3]);
	vinetic_device_unregister(pci_board_vinetics[0]);

	vinetic_rtp_channel_unregister(pci_board_vinetics[1]->rtp_channels[0]);
	vinetic_rtp_channel_unregister(pci_board_vinetics[1]->rtp_channels[1]);
	vinetic_rtp_channel_unregister(pci_board_vinetics[1]->rtp_channels[2]);
	vinetic_rtp_channel_unregister(pci_board_vinetics[1]->rtp_channels[3]);
	vinetic_device_unregister(pci_board_vinetics[1]);

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
