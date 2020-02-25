#ifndef POLYGATOR_TYPES_H
#define POLYGATOR_TYPES_H

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <sys/types.h>
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,26)
typedef unsigned long int uintptr_t;
#endif

#endif /* POLYGATOR_TYPES_H */
