/******************************************************************************/
/* polygator-ioctl.c                                                          */
/******************************************************************************/

#ifndef __POLYGATOR_IOCTL_H__
#define __POLYGATOR_IOCTL_H__

#include <linux/types.h>
#include <linux/ioctl.h>

struct polygator_ioctl_get_board {
	u_int32_t no;
	char name[POLYGATOR_BRDNAME_MAXLEN];
};

#define POLYGATOR_MAGIC			'p'
#define POLYGATOR_GET_BOARD		_IOWR(POLYGATOR_MAGIC, 0, struct polygator_ioctl_get_board)

#endif //__POLYGATOR_IOCTL_H__

/******************************************************************************/
/* end of polygator-ioctl.c                                                   */
/******************************************************************************/
