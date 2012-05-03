/******************************************************************************/
/* vinetic-base.c                                                             */
/******************************************************************************/

#ifndef __VINETIC_BASE_H__
#define __VINETIC_BASE_H__

#include <linux/cdev.h>
#include <linux/poll.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/types.h>
#include <linux/wait.h>

#define VINETIC_DEVICE_MAXCOUNT 256
#define VINETIC_DEVNAME_MAXLEN 16

#define VINETIC_PACKETSLOT_MAXCOUNT 32

enum {
	VINETIC_DEVTYPE_UNKNOWN = 0,
	VINETIC_DEVTYPE_VINETIC = 1,
	VINETIC_DEVTYPE_RTPCHAN = 2,
};

struct vinetic_rtp_channel;
struct vinetic {
	char *name;
	int devno;
	spinlock_t lock;
	struct timer_list poll_timer;
	int poll;

	// FIBXMS
	size_t free_cbox_space;
	size_t free_pbox_space;
	// OBXML
	size_t cdata_size;
	size_t pdata_size;

	wait_queue_head_t free_cbox_waitq;
	wait_queue_head_t free_pbox_waitq;

	wait_queue_head_t seek_cbox_waitq;
	wait_queue_head_t read_cbox_waitq;
	u_int16_t read_cbox_data[32];
	size_t read_cbox_length;

	struct vinetic_rtp_channel *rtp_channels[8];

	uintptr_t cbdata;
	void (* reset)(uintptr_t cbdata);
	size_t (* is_not_ready)(uintptr_t cbdata);
	void (* write_nwd)(uintptr_t cbdata, u_int16_t value);
	void (* write_eom)(uintptr_t cbdata, u_int16_t value);
	u_int16_t (* read_nwd)(uintptr_t cbdata);
	u_int16_t (* read_eom)(uintptr_t cbdata);
	u_int16_t (* read_dia)(uintptr_t cbdata);

	struct cdev cdev;
};

struct vinetic_rtp_channel {
	char *name;
	int devno;
	spinlock_t lock;

	wait_queue_head_t poll_waitq;

	struct vinetic *vinetic;
	int index;
	
	struct rtp_packet_slot {
		u_int16_t data[256];
		size_t length;
		} recv_slot[VINETIC_PACKETSLOT_MAXCOUNT];
	size_t recv_slot_read;
	size_t recv_slot_write;
	size_t recv_slot_count;

	struct cdev cdev;
};

struct vinetic *vinetic_device_register(struct module *owner,
							char *name,
							uintptr_t cbdata,
							void (* reset)(uintptr_t cbdata),
							size_t (* is_not_ready)(uintptr_t cbdata),
							void (* write_nwd)(uintptr_t cbdata, u_int16_t value),
							void (* write_eom)(uintptr_t cbdata, u_int16_t value),
							u_int16_t (* read_nwd)(uintptr_t cbdata),
							u_int16_t (* read_eom)(uintptr_t cbdata),
							u_int16_t (* read_dia)(uintptr_t cbdata));
void vinetic_device_unregister(struct vinetic *vin);

struct vinetic_rtp_channel *vinetic_rtp_channel_register(struct module *owner, char *name, struct vinetic *vin, int index);
void vinetic_rtp_channel_unregister(struct vinetic_rtp_channel *rtp);

#endif //__VINETIC_BASE_H__

/******************************************************************************/
/* end of vinetic-base.c                                                      */
/******************************************************************************/
