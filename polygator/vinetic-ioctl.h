/******************************************************************************/
/* vinetic-ioctl.c                                                            */
/******************************************************************************/

#ifndef __VINETIC_IOCTL_H__
#define __VINETIC_IOCTL_H__

#include <linux/types.h>
#include <linux/ioctl.h>

#define VINETIC_MAGIC			'v'
#define VINETIC_RESET			_IO(VINETIC_MAGIC, 0)
#define VINETIC_DISABLE_IRQ		_IO(VINETIC_MAGIC, 1)
#define VINETIC_GET_NOT_READY	_IOR(VINETIC_MAGIC, 2, int)
#define VINETIC_READ_DIA		_IOR(VINETIC_MAGIC, 3, u_int16_t)
#define VINETIC_REVISION		_IOR(VINETIC_MAGIC, 4, u_int16_t)
#define VINETIC_CHECKSUM		_IOR(VINETIC_MAGIC, 5, u_int16_t)
#define VINETIC_SET_POLL		_IOW(VINETIC_MAGIC, 6, int)

#endif //__VINETIC_IOCTL_H__

/******************************************************************************/
/* end of vinetic-ioctl.c                                                     */
/******************************************************************************/
