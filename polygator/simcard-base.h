/******************************************************************************/
/* simcard-base.h                                                             */
/******************************************************************************/

#ifndef __SIMCARD_BASE_H__
#define __SIMCARD_BASE_H__

#include <linux/cdev.h>
#include <linux/poll.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/types.h>
#include <linux/wait.h>

#include "polygator-types.h"

#include "simcard-def.h"

#define SIMCARD_DEVICE_MAXCOUNT 256
#define SIMCARD_DEVNAME_MAXLEN 256

union simcard_data_status {
	struct {
		u_int32_t data:1;
		u_int32_t reset:1;
		u_int32_t speed:1;
		u_int32_t reserved:29;
	}  __attribute__((packed)) bits;
	u_int32_t full;
} __attribute__((packed));

struct simcard_device {
	int devno;
	spinlock_t lock;

	size_t usage;

	struct timer_list poll_timer;
	size_t poll;

	wait_queue_head_t poll_waitq;
	wait_queue_head_t read_waitq;
	wait_queue_head_t write_waitq;

	void *data;

	u_int8_t (* read)(void *data);
	void (* write)(void *data, u_int8_t value);
	int (* is_read_ready)(void *data);
	int (* is_write_ready)(void *data);
	int (* is_reset_request)(void *data);
	void (* set_speed)(void *data, int speed);
	void (* do_after_reset)(void *data);

	union simcard_data_status read_status;
	union simcard_data_status write_status;

	int reset_state;

	struct simcard_data command;
	struct simcard_data reset;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
	struct device *device;
#else
	struct class_device *device;
#endif
	struct cdev cdev;
};

struct simcard_device *simcard_device_register(struct module *owner,
												void *data,
												u_int8_t (* read)(void *data),
												void (* write)(void *data, u_int8_t value),
												int (* is_read_ready)(void *data),
												int (* is_write_ready)(void *data),
												int (* is_reset_request)(void *data),
												void (* set_speed)(void *data, int speed),
												void (* do_after_reset)(void *data));
void simcard_device_unregister(struct simcard_device *sim);

#endif //__SIMCARD_BASE_H__

/******************************************************************************/
/* end of simcard-base.h                                                      */
/******************************************************************************/
