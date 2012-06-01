/******************************************************************************/
/* polygator-base.c                                                           */
/******************************************************************************/

#ifndef __POLYGATOR_BASE_H__
#define __POLYGATOR_BASE_H__

#include <linux/cdev.h>
#include <linux/fs.h>
// #include <linux/poll.h>
// #include <linux/spinlock.h>
// #include <linux/timer.h>
#include <linux/types.h>
// #include <linux/types.h>
// #include <linux/wait.h>

#define POLYGATOR_DEVICE_MAXCOUNT 256

#define POLYGATOR_BOARD_MAXCOUNT 32

#define POLYGATOR_BRDNAME_MAXLEN 256

struct polygator_board {
	char name[POLYGATOR_BRDNAME_MAXLEN];
	int devno;
	struct cdev cdev;
	void *data;
};

struct polygator_board *polygator_board_register(struct module *owner, char *name, void *data);
void polygator_board_unregister(struct polygator_board  *brd);

#endif //__POLYGATOR_BASE_H__

/******************************************************************************/
/* end of polygator-base.c                                                    */
/******************************************************************************/
