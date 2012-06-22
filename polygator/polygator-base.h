/******************************************************************************/
/* polygator-base.c                                                           */
/******************************************************************************/

#ifndef __POLYGATOR_BASE_H__
#define __POLYGATOR_BASE_H__

#define POLYGATOR_DEVICE_MAXCOUNT 256

#define POLYGATOR_BOARD_MAXCOUNT 32

#define POLYGATOR_BRDNAME_MAXLEN 256

enum {
	POLYGATOR_MODULE_TYPE_UNKNOWN = 0,
	POLYGATOR_MODULE_TYPE_SIM300 = 1,
	POLYGATOR_MODULE_TYPE_SIM900 = 2,
	POLYGATOR_MODULE_TYPE_M10 = 3,
	POLYGATOR_MODULE_TYPE_SIM5215 = 4,
};

#ifdef __KERNEL__

#include <linux/cdev.h>
#include <linux/fs.h>
// #include <linux/poll.h>
// #include <linux/spinlock.h>
// #include <linux/timer.h>
#include <linux/types.h>
// #include <linux/types.h>
// #include <linux/wait.h>

struct polygator_board {
	int devno;
	char name[POLYGATOR_BRDNAME_MAXLEN];
	struct cdev *cdev;
};

struct polygator_board *polygator_board_register(struct module *owner, char *name, struct cdev *cdef, struct file_operations *fops);
void polygator_board_unregister(struct polygator_board  *brd);
char *polygator_print_gsm_module_type(int type);

#endif //__KERNEL__

#endif //__POLYGATOR_BASE_H__

/******************************************************************************/
/* end of polygator-base.c                                                    */
/******************************************************************************/
