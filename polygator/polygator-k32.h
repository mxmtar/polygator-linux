#ifndef POLYGATOR_K32_H
#define POLYGATOR_K32_H

#include <linux/types.h>

#include "polygator-types.h"

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,36) // 2,6,30 - orig
#define TTY_PORT
#endif

union k32_gsm_mod_status_reg {
	struct {
		uint8_t at_rdy_rd:1;
		uint8_t at_rdy_wr:1;
		uint8_t vio:1;
		uint8_t sim_rdy_rd:1;
		uint8_t sim_rdy_wr:1;
		uint8_t sim_rst_req:1;
		uint8_t imei_rdy_rd:1;
		uint8_t imei_rdy_wr:1;
	} __attribute__((packed)) bits;
	struct {
		uint8_t vio:1;
		uint8_t at_rdy_rd:1;
		uint8_t at_rdy_wr:1;
		uint8_t sim_rdy_rd:1;
		uint8_t sim_rdy_wr:1;
		uint8_t sim_rst_req:1;
		uint8_t imei_rdy_rd:1;
		uint8_t imei_rdy_wr:1;
	} __attribute__((packed)) bits_e;
	uint8_t full;
} __attribute__((packed));

union k32_gsm_mod_control_reg {
	struct {
		uint8_t mod_off:1;
		uint8_t sim_spd_1:1;
		uint8_t rst:1;
		uint8_t sim_spd_0:1;
		uint8_t pwr_off:1;
		uint8_t sync_mode:1;
		uint8_t com_spd:2;
	} __attribute__((packed)) bits;
	struct {
		uint8_t pwr:1;
		uint8_t key:1;
		uint8_t gap:2;
		uint8_t sim_spd:2;
		uint8_t com_spd:2;
	} __attribute__((packed)) bits_e;
	uint8_t full;
} __attribute__((packed));

struct k32_gsm_module_data {

	int type;
	size_t pos_on_board;

	union k32_gsm_mod_control_reg control;

	uintptr_t cbdata;

	void (* set_control)(uintptr_t cbdata, size_t pos, uint8_t reg);
	uint8_t (* get_status)(uintptr_t cbdata, size_t pos);
	void (* at_write)(uintptr_t cbdata, size_t pos, uint8_t reg);
	uint8_t (* at_read)(uintptr_t cbdata, size_t pos);
	uint16_t (* at_read16)(uintptr_t cbdata, size_t pos);
	void (* sim_write)(uintptr_t cbdata, size_t pos, uint8_t reg);
	uint8_t (* sim_read)(uintptr_t cbdata, size_t pos);
	void (* sim_do_after_reset)(uintptr_t cbdata, size_t pos);
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

struct k32_board {

	struct polygator_board *pg_board;
	struct cdev cdev;

	uint8_t rom[256];
	size_t romsize;
	uint32_t sn;
	uint8_t serial_number[256];
	uint16_t type;
	uint16_t position;

	int iomem_req;
	void __iomem *iomem_base;

	struct vinetic *vinetics[2];

	struct k32_gsm_module_data *gsm_modules[8];

	struct polygator_tty_device *tty_at_channels[8];

	struct simcard_device *simcard_channels[8];
};

struct k32_board_private_data {
	struct k32_board *board;
	char buff[0x0C00];
	size_t length;
};

#endif /* POLYGATOR_K32_H */
