#ifndef SIMCARD_DEF_H
#define SIMCARD_DEF_H

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <sys/types.h>
#endif

#define SIMCARD_MAX_DATA_LENGTH 512

enum {
    SIMCARD_CONTAINER_TYPE_UNKNOWN  = 0,
    SIMCARD_CONTAINER_TYPE_DATA     = 1,
    SIMCARD_CONTAINER_TYPE_RESET    = 2,
    SIMCARD_CONTAINER_TYPE_SPEED    = 3,
};

struct simcard_data {
	struct simcard_data_header {
		uint32_t type;
		uint32_t length;
	} __attribute__((packed)) header;
	union {
		uint8_t data[SIMCARD_MAX_DATA_LENGTH];
		uint32_t reset;
		uint32_t speed;
	} __attribute__((packed)) body;
} __attribute__((packed));

#endif /* SIMCARD_DEF_H */
