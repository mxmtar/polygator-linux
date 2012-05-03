/******************************************************************************/
/* k5-base.c                                                                  */
/******************************************************************************/

#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/version.h>

#include "../arch/arm/include/asm/io.h"
#include "../arch/arm/mach-at91/include/mach/hardware.h"
#include "../arch/arm/mach-at91/include/mach/io.h"
#include "../arch/arm/mach-at91/include/mach/at91_pio.h"
#include "../arch/arm/mach-at91/include/mach/at91sam9260_matrix.h"

#include "polygator/vinetic-base.h"
#include "polygator/vinetic-def.h"

MODULE_AUTHOR("Maksym Tarasevych <mxmtar@ukr.net>");
MODULE_DESCRIPTION("Polygator Linux module for K5 device");
MODULE_LICENSE("GPL");

#define verbose(_fmt, _args...) printk(KERN_INFO "[pg-%s] " _fmt, THIS_MODULE->name, ## _args)
#define log(_level, _fmt, _args...) printk(_level "[pg-%s] %s:%d - %s(): " _fmt, THIS_MODULE->name, "k5-base.c", __LINE__, __PRETTY_FUNCTION__, ## _args)
#define debug(_fmt, _args...) printk(KERN_DEBUG "[pg-%s] %s:%d - %s(): " _fmt, THIS_MODULE->name, "k5-base.c", __LINE__, __PRETTY_FUNCTION__, ## _args)

static struct resource * k5_cs3_iomem_reg = NULL;
static struct resource * k5_cs4_iomem_reg = NULL;

static void __iomem * k5_cs3_base_ptr = NULL;
static void __iomem * k5_cs4_base_ptr = NULL;

static struct vinetic * k5_vinetic;
static struct vinetic_rtp_channel * k5_vinetic_rtp_channels[4];

static void k5_vinetic_reset(uintptr_t cbdata)
{
	iowrite16(0, cbdata + 0x20);
	mdelay(1);
	iowrite16(1, cbdata + 0x20);
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

static int __init k5_init(void)
{
	size_t i;
	u32 data;
	char name[VINETIC_DEVNAME_MAXLEN];
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
	at91_sys_write(AT91_SMC + 0x30 + 0x0, 0x03030303);
	at91_sys_write(AT91_SMC + 0x30 + 0x4, 0x100d100d);
	at91_sys_write(AT91_SMC + 0x30 + 0x8, 0x00000000);
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

// 	// Reset K5 board
// 	iowrite16(0, k5_cs4_base_ptr + 32);
// 	mdelay(10);
// 	iowrite16(1, k5_cs4_base_ptr + 32);

	// Register vinetic
	k5_vinetic = NULL;
	if (!(k5_vinetic = vinetic_device_register(THIS_MODULE, "vin0", (uintptr_t)k5_cs4_base_ptr,
													k5_vinetic_reset,
													k5_vinetic_is_not_ready,
													k5_vinetic_write_nwd,
													k5_vinetic_write_eom,
													k5_vinetic_read_nwd,
													k5_vinetic_read_eom,
													k5_vinetic_read_dia))) {
		log(KERN_ERR, "can't register vinetic device \"vin0\"\n");
		goto k5_init_error;
	}
	for (i=0; i<4; i++)
	{
		k5_vinetic_rtp_channels[i] = NULL;
		snprintf(name, VINETIC_DEVNAME_MAXLEN, "vin0rtp%lu", (unsigned long int)i);
		if (!(k5_vinetic_rtp_channels[i] = vinetic_rtp_channel_register(THIS_MODULE, name, k5_vinetic, i))) {
			log(KERN_ERR, "can't register vinetic rtp channel \"%s\"\n", name);
			goto k5_init_error;
		}
	}

	verbose("loaded successfull\n");
	return rc;

k5_init_error:
	for (i=0; i<4; i++)
	{
		if (k5_vinetic_rtp_channels[i]) vinetic_rtp_channel_unregister(k5_vinetic_rtp_channels[i]);
	}
	if (k5_vinetic) vinetic_device_unregister(k5_vinetic);
	if (k5_cs3_iomem_reg) release_mem_region(AT91_CHIPSELECT_3, 0x10000);
	if (k5_cs3_base_ptr) iounmap(k5_cs3_base_ptr);
	if (k5_cs4_iomem_reg) release_mem_region(AT91_CHIPSELECT_4, 0x10000);
	if (k5_cs4_base_ptr) iounmap(k5_cs4_base_ptr);
	return rc;
}

static void __exit k5_exit(void)
{
	size_t i;

	for (i=0; i<4; i++)
		vinetic_rtp_channel_unregister(k5_vinetic_rtp_channels[i]);
	vinetic_device_unregister(k5_vinetic);

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
