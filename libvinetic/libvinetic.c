/******************************************************************************/
/* libvinetic.c                                                               */
/******************************************************************************/

#define _LARGEFILE64_SOURCE

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>

#include <errno.h>
#include <fcntl.h>
#include <libgen.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <endian.h>

#include "polygator/libvinetic.h"

void vin_init(struct vinetic_context *ctx, char *path)
{
	strncpy(ctx->dev_path, path, sizeof(ctx->dev_path));
	ctx->dev_fd = -1;
}

void vin_set_pram(struct vinetic_context *ctx, char *path)
{
	strncpy(ctx->pram_path, path, sizeof(ctx->pram_path));
}

void vin_set_dram(struct vinetic_context *ctx, char *path)
{
	strncpy(ctx->dram_path, path, sizeof(ctx->dram_path));
}

void vin_set_alm_dsp_ab(struct vinetic_context *ctx, char *path)
{
	strncpy(ctx->alm_dsp_ab_path, path, sizeof(ctx->alm_dsp_ab_path));
}

void vin_set_alm_dsp_cd(struct vinetic_context *ctx, char *path)
{
	strncpy(ctx->alm_dsp_cd_path, path, sizeof(ctx->alm_dsp_cd_path));
}

int vin_open(struct vinetic_context *ctx)
{
	ctx->dev_fd = open(ctx->dev_path, O_RDWR);
	ctx->error = errno;
	return ctx->dev_fd;
}

void vin_close(struct vinetic_context *ctx)
{
	close(ctx->dev_fd);
	ctx->dev_fd = -1;
}

int vin_reset(struct vinetic_context *ctx)
{
	int rc = ioctl(ctx->dev_fd, VINETIC_RESET, NULL);
	ctx->error = errno;
	return rc;
}

int vin_is_not_ready(struct vinetic_context *ctx)
{
	int not_ready;
	if (ioctl(ctx->dev_fd, VINETIC_GET_NOT_READY, &not_ready) < 0) {
		ctx->error = errno;
		not_ready = -1;
	}
	return not_ready;
}

u_int16_t vin_read_dia(struct vinetic_context *ctx)
{
	u_int16_t dia;
	if (ioctl(ctx->dev_fd, VINETIC_READ_DIA, &dia) < 0) {
		ctx->error = errno;
		dia = 0xffff;
	}
	return dia;
}

int vin_resync(struct vinetic_context *ctx)
{
	int rc;
	union vin_cmd_short cmd_short;

	// Re-SYNChronize PCM clock
	cmd_short.full = VIN_wRESYNC;
	rc = vin_write(ctx, &cmd_short.full, sizeof(union vin_cmd_short));
	ctx->errorline = __LINE__ - 1;
	ctx->error = errno;
	return rc;
}


int vin_poll_set(struct vinetic_context *ctx, int poll)
{
	int rc = ioctl(ctx->dev_fd, VINETIC_SET_POLL, &poll);
	ctx->error = errno;
	return rc;
}

char *vin_dev_name(struct vinetic_context *ctx)
{
	return basename(ctx->dev_path);
}

char *vin_error_str(struct vinetic_context *ctx)
{
	return strerror(ctx->error);
}

char *vin_revision_str(struct vinetic_context *ctx)
{
	switch (ctx->revision)
	{
		case VIN_REV_13: return "1.3";
		case VIN_REV_14: return "1.4";
		default: return "unknown";
	}
}

ssize_t vin_write(struct vinetic_context *ctx, const void *buf, size_t count)
{
	ssize_t rc;
	off64_t lsrc;
	union vin_cmd cmd;
	u_int8_t *data = (u_int8_t *)buf;
	size_t length;

	cmd.full = 0;
	if (count < 2) {
		rc = -EINVAL;
		goto vin_write_end;
	} else if (count == 2) {
		memcpy(&cmd.parts.first.full, data, 2);
		length = 0;
	} else if(count == 4) {
		memcpy(&cmd.full, data, 4);
		length = 0;
	} else {
		memcpy(&cmd.full, data, 4);
		length = count - 4;
	}

	if ((lsrc = lseek64(ctx->dev_fd, cmd.full, SEEK_SET)) < 0) {
		ctx->error = errno;
		printf("lseek()=%ld\n", (long int)lsrc);
		goto vin_write_end;
	}

	if ((rc = write(ctx->dev_fd, data+4, length)) < 0) {
		ctx->error = errno;
		printf("write()=%ld\n", (long int)rc);
		goto vin_write_end;
	}

vin_write_end:
	return rc;
}

ssize_t vin_read(struct vinetic_context *ctx, union vin_cmd cmd, void *buf, size_t count)
{
	ssize_t rc;
	off64_t lsrc;

	if ((lsrc = lseek64(ctx->dev_fd, cmd.full, SEEK_SET)) < 0) {
		ctx->error = errno;
		goto vin_read_end;
	}

	if ((rc = read(ctx->dev_fd, buf, count)) < 0) {
		ctx->error = errno;
		goto vin_read_end;
	}

vin_read_end:
	return rc;
}

u_int16_t vin_phi_revision(struct vinetic_context *ctx)
{
	u_int16_t rev;

	if (ioctl(ctx->dev_fd, VINETIC_REVISION, &rev) < 0) {
		ctx->error = errno;
		rev = 0;
	}
	ctx->revision = rev;
	return rev;
}

u_int16_t vin_phi_checksum(struct vinetic_context *ctx)
{
	u_int16_t csum;

	if (ioctl(ctx->dev_fd, VINETIC_CHECKSUM, &csum) < 0) {
		ctx->error = errno;
		csum = 0;
	}

	return csum;
}

int vin_phi_disable_interrupt(struct vinetic_context *ctx)
{
	int rc = ioctl(ctx->dev_fd, VINETIC_DISABLE_IRQ, NULL);
	ctx->error = errno;
	return rc;
}

int vin_check_mbx_empty(struct vinetic_context *ctx)
{
	ssize_t res;
	size_t cnt;
	union vin_cmd cmd;
	struct vin_read_bxsr bxsr;

	res = 0;
	cnt = 255;
	cmd.full = 0;
	cmd.parts.first.full = VIN_rBXSR;

	for (;;)
	{
		if ((res = vin_read(ctx, cmd, &bxsr, sizeof(struct vin_read_bxsr))) < 0) {
			ctx->error = errno;
			goto vin_check_mbx_empty_end;
		}
		if (bxsr.bxsr2.bits.mbx_empty) break;
		if (!cnt--) {
			res = -EIO;
			ctx->error = -EIO;
			goto vin_check_mbx_empty_end;
		}
		usleep(125);
	}
vin_check_mbx_empty_end:
	return res;
}

int vin_wait_dl_rdy(struct vinetic_context *ctx)
{
	ssize_t res;
	size_t cnt;
	union vin_cmd cmd;
	struct vin_read_hwsr hwsr;

	res = 0;
	cnt = 8000;
	cmd.full = 0;
	cmd.parts.first.full = VIN_rHWSR;

	for (;;)
	{
		if ((res = vin_read(ctx, cmd, &hwsr, sizeof(struct vin_read_hwsr))) < 0) {
			ctx->error = errno;
			goto vin_wait_dl_ready_end;
		}
		if (hwsr.hwsr2.bits.dl_rdy) break;
		if (!cnt--) {
			res = -EIO;
			ctx->error = -EIO;
			goto vin_wait_dl_ready_end;
		}
		usleep(125);
	}
vin_wait_dl_ready_end:
	return res;
}

int vin_download_edsp_firmvare(struct vinetic_context *ctx)
{
	union vin_cmd cmd;
	union vin_cmd_short cmd_short;

	struct vin_cmd_eop_set_pram_address cmd_eop_set_pram_address;
	struct vin_cmd_eop_access_pram cmd_eop_access_pram;

	struct vin_cmd_eop_set_dram_address cmd_eop_set_dram_address;
	struct vin_cmd_eop_access_dram cmd_eop_access_dram;
#if 0
	struct vin_cmd_eop_crc_pram cmd_eop_crc_pram;
	struct vin_cmd_eop_crc_dram cmd_eop_crc_dram;
	u_int16_t seg_addr_start_high;
	u_int16_t seg_addr_start_low;
	u_int32_t seg_addr_start;
	u_int16_t seg_addr_end_high;
	u_int16_t seg_addr_end_low;
	u_int32_t seg_addr_end;
#endif
	ssize_t res;
	size_t i;
	size_t seg_size;
	size_t seg_chunk_size;

	int fd;
	off_t fd_offset;
	struct stat fd_stat;

	struct vin_xram_segment_header {
		u_int32_t size;
		u_int16_t high;
		u_int16_t low;
	} __attribute__((packed)) xram_segment_header;

	// Check for MBX-EMPTY
	if (vin_check_mbx_empty(ctx) < 0) {
		ctx->errorline = __LINE__ - 1;
		ctx->error = errno;
		goto vin_download_edsp_firmware_error;

	}
	// Maximazing Command In-Box
	cmd_short.full = VIN_wMAXCBX;
	if ((res = vin_write(ctx, &cmd_short.full, sizeof(union vin_cmd_short))) < 0) {
		ctx->errorline = __LINE__ - 1;
		ctx->error = errno;
		goto vin_download_edsp_firmware_error;
	}
	// Load EDSP Micro Program
	cmd_short.full = VIN_wLEMP;
	if ((res = vin_write(ctx, &cmd_short.full, sizeof(union vin_cmd_short))) < 0) {
		ctx->errorline = __LINE__ - 1;
		ctx->error = errno;
		goto vin_download_edsp_firmware_error;
	}
	// open PRAM file
	if ((fd = open(ctx->pram_path, O_RDONLY)) < 0) {
		ctx->errorline = __LINE__ - 1;
		ctx->error = errno;
		goto vin_download_edsp_firmware_error;
	}
	if (fstat(fd, &fd_stat) < 0) {
		ctx->errorline = __LINE__ - 1;
		ctx->error = errno;
		goto vin_download_edsp_firmware_error;
	}
	fd_offset = 0;
	while (fd_offset < fd_stat.st_size)
	{
		// Read PRAM segment header
		if (lseek(fd, fd_offset, SEEK_SET) < 0) {
			ctx->errorline = __LINE__ - 1;
			ctx->error = errno;
			goto vin_download_edsp_firmware_error;
		}
		if (read(fd, &xram_segment_header, sizeof(struct vin_xram_segment_header)) < 0) {
			ctx->errorline = __LINE__ - 1;
			ctx->error = errno;
			goto vin_download_edsp_firmware_error;
		}
		fd_offset += sizeof(struct vin_xram_segment_header);
		seg_size = be32toh(xram_segment_header.size);
		// Set PRAM Address
		cmd_eop_set_pram_address.header.parts.first.bits.rw = VIN_WRITE;
		cmd_eop_set_pram_address.header.parts.first.bits.sc = VIN_SC_NO;
		cmd_eop_set_pram_address.header.parts.first.bits.bc = VIN_BC_NO;
		cmd_eop_set_pram_address.header.parts.first.bits.cmd = VIN_CMD_EOP;
		cmd_eop_set_pram_address.header.parts.first.bits.res = 0;
		cmd_eop_set_pram_address.header.parts.first.bits.chan = 0;
		cmd_eop_set_pram_address.header.parts.second.eop.bits.mod = VIN_MOD_TEST;
		cmd_eop_set_pram_address.header.parts.second.eop.bits.ecmd  = VIN_EOP_SETPRAM;
		cmd_eop_set_pram_address.header.parts.second.eop.bits.length = 2;
		cmd_eop_set_pram_address.high_addres1 = be16toh(xram_segment_header.high);
		cmd_eop_set_pram_address.low_addres1 = be16toh(xram_segment_header.low);
		if ((res = vin_write(ctx, &cmd_eop_set_pram_address, sizeof(union vin_cmd) + sizeof(u_int16_t)*2)) < 0) {
			ctx->errorline = __LINE__ - 1;
			ctx->error = errno;
			goto vin_download_edsp_firmware_error;
		}
		while (seg_size)
		{
			// Check for MBX-EMPTY
			if (vin_check_mbx_empty(ctx) < 0) {
				ctx->errorline = __LINE__ - 1;
				ctx->error = errno;
				goto vin_download_edsp_firmware_error;
			}
			// Read PRAM segment data
			if (lseek(fd, fd_offset, SEEK_SET) < 0) {
				ctx->errorline = __LINE__ - 1;
				ctx->error = errno;
				goto vin_download_edsp_firmware_error;
			}
			seg_chunk_size = (seg_size < 252) ? (seg_size) : (252);
// 			printf("pram: %lu %lu\n", (unsigned long int)seg_chunk_size, (unsigned long int)seg_chunk_size);
			if (read(fd, cmd_eop_access_pram.data, seg_chunk_size*2) < 0) {
				ctx->errorline = __LINE__ - 1;
				ctx->error = errno;
				goto vin_download_edsp_firmware_error;
			}
			fd_offset += seg_chunk_size*2;
			for (i=0; i<seg_chunk_size; i++)
				cmd_eop_access_pram.data[i] = be16toh(cmd_eop_access_pram.data[i]);
			// Access PRAM
			cmd_eop_access_pram.header.parts.first.bits.rw = VIN_WRITE;
			cmd_eop_access_pram.header.parts.first.bits.sc = VIN_SC_NO;
			cmd_eop_access_pram.header.parts.first.bits.bc = VIN_BC_NO;
			cmd_eop_access_pram.header.parts.first.bits.cmd = VIN_CMD_EOP;
			cmd_eop_access_pram.header.parts.first.bits.res = 0;
			cmd_eop_access_pram.header.parts.first.bits.chan = 0;
			cmd_eop_access_pram.header.parts.second.eop.bits.mod = VIN_MOD_TEST;
			cmd_eop_access_pram.header.parts.second.eop.bits.ecmd  = VIN_EOP_ACCESSPRAM;
			cmd_eop_access_pram.header.parts.second.eop.bits.length = seg_chunk_size;
			if ((res = vin_write(ctx, &cmd_eop_access_pram, sizeof(union vin_cmd) + seg_chunk_size*2)) < 0) {
				ctx->errorline = __LINE__ - 1;
				ctx->error = errno;
				goto vin_download_edsp_firmware_error;
			}
			seg_size -= seg_chunk_size;
		}
	}
#if 0
	// Reset PRAM CRC
	cmd_eop_crc_pram.header.parts.first.bits.rw = VIN_WRITE;
	cmd_eop_crc_pram.header.parts.first.bits.sc = VIN_SC_NO;
	cmd_eop_crc_pram.header.parts.first.bits.bc = VIN_BC_NO;
	cmd_eop_crc_pram.header.parts.first.bits.cmd = VIN_CMD_EOP;
	cmd_eop_crc_pram.header.parts.first.bits.res = 0;
	cmd_eop_crc_pram.header.parts.first.bits.chan = 0;
	cmd_eop_crc_pram.header.parts.second.eop.bits.mod = VIN_MOD_TEST;
	cmd_eop_crc_pram.header.parts.second.eop.bits.ecmd  = VIN_EOP_CRC_PRAM;
	cmd_eop_crc_pram.header.parts.second.eop.bits.length = 0;
	if ((res = vin_write(ctx, &cmd_eop_crc_pram, sizeof(union vin_cmd)) < 0)) {
		ctx->errorline = __LINE__ - 1;
		ctx->error = errno;
		goto vin_download_edsp_firmware_error;
	}
	fd_offset = 0;
	while (fd_offset < fd_stat.st_size)
	{
		// Read PRAM segment header
		if (lseek(fd, fd_offset, SEEK_SET) < 0) {
			ctx->errorline = __LINE__ - 1;
			ctx->error = errno;
			goto vin_download_edsp_firmware_error;
		}
		if (read(fd, &xram_segment_header, sizeof(struct vin_xram_segment_header)) < 0) {
			ctx->errorline = __LINE__ - 1;
			ctx->error = errno;
			goto vin_download_edsp_firmware_error;
		}
		seg_addr_start_high = be16toh(xram_segment_header.high);
		seg_addr_start_low = be16toh(xram_segment_header.low);
		seg_addr_start = (seg_addr_start_high << 16) + (seg_addr_start_low << 0);
		seg_size = be32toh(xram_segment_header.size);
		seg_addr_end = seg_addr_start + (seg_size/3)*2 - 2;
		seg_addr_end_high = (seg_addr_end >> 16) & 0xffff;
		seg_addr_end_low = (seg_addr_end >> 0) & 0xffff;
// 		printf("pram seg %08x:%08x - %lu\n", seg_addr_start, seg_addr_end, (unsigned long int)seg_size);
		fd_offset += sizeof(struct vin_xram_segment_header) + seg_size*2;
		// Set PRAM Address
		cmd_eop_set_pram_address.header.parts.first.bits.rw = VIN_WRITE;
		cmd_eop_set_pram_address.header.parts.first.bits.sc = VIN_SC_NO;
		cmd_eop_set_pram_address.header.parts.first.bits.bc = VIN_BC_NO;
		cmd_eop_set_pram_address.header.parts.first.bits.cmd = VIN_CMD_EOP;
		cmd_eop_set_pram_address.header.parts.first.bits.res = 0;
		cmd_eop_set_pram_address.header.parts.first.bits.chan = 0;
		cmd_eop_set_pram_address.header.parts.second.eop.bits.mod = VIN_MOD_TEST;
		cmd_eop_set_pram_address.header.parts.second.eop.bits.ecmd  = VIN_EOP_SETPRAM;
		cmd_eop_set_pram_address.header.parts.second.eop.bits.length = 4;
		cmd_eop_set_pram_address.high_addres1 = seg_addr_start_high;
		cmd_eop_set_pram_address.low_addres1 = seg_addr_start_low;
		cmd_eop_set_pram_address.high_addres2 = seg_addr_end_high;
		cmd_eop_set_pram_address.low_addres2 = seg_addr_end_low;
		if ((res = vin_write(ctx, &cmd_eop_set_pram_address, sizeof(union vin_cmd) + sizeof(u_int16_t)*4)) < 0) {
			ctx->errorline = __LINE__ - 1;
			ctx->error = errno;
			goto vin_download_edsp_firmware_error;
		}
		// Read PRAM CRC
		cmd_eop_crc_pram.header.parts.first.bits.rw = VIN_READ;
		cmd_eop_crc_pram.header.parts.first.bits.sc = VIN_SC_NO;
		cmd_eop_crc_pram.header.parts.first.bits.bc = VIN_BC_NO;
		cmd_eop_crc_pram.header.parts.first.bits.cmd = VIN_CMD_EOP;
		cmd_eop_crc_pram.header.parts.first.bits.res = 0;
		cmd_eop_crc_pram.header.parts.first.bits.chan = 0;
		cmd_eop_crc_pram.header.parts.second.eop.bits.mod = VIN_MOD_TEST;
		cmd_eop_crc_pram.header.parts.second.eop.bits.ecmd  = VIN_EOP_CRC_PRAM;
		cmd_eop_crc_pram.header.parts.second.eop.bits.length = 1;
		if ((res = vin_read(ctx, cmd_eop_crc_pram.header, &cmd_eop_crc_pram, sizeof(struct vin_cmd_eop_crc_pram)) < 0)) {
			ctx->errorline = __LINE__ - 1;
			ctx->error = errno;
			goto vin_download_edsp_firmware_error;
		}
	}
	printf("\npram: checksum=0x%04x\n", cmd_eop_crc_pram.crc);
#endif
	close(fd);

	// Check for MBX-EMPTY
	if (vin_check_mbx_empty(ctx) < 0) {
		ctx->errorline = __LINE__ - 1;
		ctx->error = errno;
		goto vin_download_edsp_firmware_error;
	}

	// open DRAM file
	if ((fd = open(ctx->dram_path, O_RDONLY)) < 0) {
		ctx->errorline = __LINE__ - 1;
		ctx->error = errno;
		goto vin_download_edsp_firmware_error;
	}
	if (fstat(fd, &fd_stat) < 0) {
		ctx->errorline = __LINE__ - 1;
		ctx->error = errno;
		goto vin_download_edsp_firmware_error;
	}
	fd_offset = 0;
	while (fd_offset < fd_stat.st_size)
	{
		// Read DRAM segment header
		if (lseek(fd, fd_offset, SEEK_SET) < 0) {
			ctx->errorline = __LINE__ - 1;
			ctx->error = errno;
			goto vin_download_edsp_firmware_error;
		}
		if (read(fd, &xram_segment_header, sizeof(struct vin_xram_segment_header)) < 0) {
			ctx->errorline = __LINE__ - 1;
			ctx->error = errno;
			goto vin_download_edsp_firmware_error;
		}
		fd_offset += sizeof(struct vin_xram_segment_header);
		// Set DRAM Address
		cmd_eop_set_dram_address.header.parts.first.bits.rw = VIN_WRITE;
		cmd_eop_set_dram_address.header.parts.first.bits.sc = VIN_SC_NO;
		cmd_eop_set_dram_address.header.parts.first.bits.bc = VIN_BC_NO;
		cmd_eop_set_dram_address.header.parts.first.bits.cmd = VIN_CMD_EOP;
		cmd_eop_set_dram_address.header.parts.first.bits.res = 0;
		cmd_eop_set_dram_address.header.parts.first.bits.chan = 0;
		cmd_eop_set_dram_address.header.parts.second.eop.bits.mod = VIN_MOD_TEST;
		cmd_eop_set_dram_address.header.parts.second.eop.bits.ecmd  = VIN_EOP_SET_DRAM;
		cmd_eop_set_dram_address.header.parts.second.eop.bits.length = 1;
		cmd_eop_set_dram_address.addres1 = be16toh(xram_segment_header.low);
		if ((res = vin_write(ctx, &cmd_eop_set_dram_address, sizeof(union vin_cmd) + sizeof(u_int16_t))) < 0) {
			ctx->errorline = __LINE__ - 1;
			ctx->error = errno;
			goto vin_download_edsp_firmware_error;
		}
		seg_size = be32toh(xram_segment_header.size);
		while (seg_size)
		{
			// Check for MBX-EMPTY
			if (vin_check_mbx_empty(ctx) < 0) {
				ctx->errorline = __LINE__ - 1;
				ctx->error = errno;
				goto vin_download_edsp_firmware_error;
			}
			// Read DRAM segment data
			if (lseek(fd, fd_offset, SEEK_SET) < 0) {
				ctx->errorline = __LINE__ - 1;
				ctx->error = errno;
				goto vin_download_edsp_firmware_error;
			}
			seg_chunk_size = (seg_size < 252) ? (seg_size) : (252);
// 			printf("dram: %lu %lu\n", (unsigned long int)seg_chunk_size, (unsigned long int)seg_chunk_size);
			if (read(fd, cmd_eop_access_dram.data, seg_chunk_size*2) < 0) {
				ctx->errorline = __LINE__ - 1;
				ctx->error = errno;
				goto vin_download_edsp_firmware_error;
			}
			fd_offset += seg_chunk_size*2;
			for (i=0; i<seg_chunk_size; i++)
				cmd_eop_access_dram.data[i] = be16toh(cmd_eop_access_dram.data[i]);
			// Access DRAM
			cmd_eop_access_dram.header.parts.first.bits.rw = VIN_WRITE;
			cmd_eop_access_dram.header.parts.first.bits.sc = VIN_SC_NO;
			cmd_eop_access_dram.header.parts.first.bits.bc = VIN_BC_NO;
			cmd_eop_access_dram.header.parts.first.bits.cmd = VIN_CMD_EOP;
			cmd_eop_access_dram.header.parts.first.bits.res = 0;
			cmd_eop_access_dram.header.parts.first.bits.chan = 0;
			cmd_eop_access_dram.header.parts.second.eop.bits.mod = VIN_MOD_TEST;
			cmd_eop_access_dram.header.parts.second.eop.bits.ecmd  = VIN_EOP_ACCESS_DRAM;
			cmd_eop_access_dram.header.parts.second.eop.bits.length = seg_chunk_size;
			if ((res = vin_write(ctx, &cmd_eop_access_dram, sizeof(union vin_cmd) + seg_chunk_size*2)) < 0) {
				ctx->errorline = __LINE__ - 1;
				ctx->error = errno;
				goto vin_download_edsp_firmware_error;
			}
			seg_size -= seg_chunk_size;
		}
	}
#if 0
	// Reset DRAM CRC
	cmd_eop_crc_dram.header.parts.first.bits.rw = VIN_WRITE;
	cmd_eop_crc_dram.header.parts.first.bits.sc = VIN_SC_NO;
	cmd_eop_crc_dram.header.parts.first.bits.bc = VIN_BC_NO;
	cmd_eop_crc_dram.header.parts.first.bits.cmd = VIN_CMD_EOP;
	cmd_eop_crc_dram.header.parts.first.bits.res = 0;
	cmd_eop_crc_dram.header.parts.first.bits.chan = 0;
	cmd_eop_crc_dram.header.parts.second.eop.bits.mod = VIN_MOD_TEST;
	cmd_eop_crc_dram.header.parts.second.eop.bits.ecmd  = VIN_EOP_CRC_DRAM;
	cmd_eop_crc_dram.header.parts.second.eop.bits.length = 0;
	if ((res = vin_write(ctx, &cmd_eop_crc_dram, sizeof(union vin_cmd)) < 0)) {
		ctx->errorline = __LINE__ - 1;
		ctx->error = errno;
		goto vin_download_edsp_firmware_error;
	}
	fd_offset = 0;
	while (fd_offset < fd_stat.st_size)
	{
		// Read DRAM segment header
		if (lseek(fd, fd_offset, SEEK_SET) < 0) {
			ctx->errorline = __LINE__ - 1;
			ctx->error = errno;
			goto vin_download_edsp_firmware_error;
		}
		if (read(fd, &xram_segment_header, sizeof(struct vin_xram_segment_header)) < 0) {
			ctx->errorline = __LINE__ - 1;
			ctx->error = errno;
			goto vin_download_edsp_firmware_error;
		}
		seg_addr_start_high = be16toh(xram_segment_header.high);
		seg_addr_start_low = be16toh(xram_segment_header.low);
		seg_addr_start = (seg_addr_start_high << 16) + (seg_addr_start_low << 0);
		seg_size = be32toh(xram_segment_header.size);
		seg_addr_end = seg_addr_start + seg_size - 1;
		seg_addr_end_high = (seg_addr_end >> 16) & 0xffff;
		seg_addr_end_low = (seg_addr_end >> 0) & 0xffff;
// 		printf("dram seg %08x:%08x - %lu\n", seg_addr_start, seg_addr_end, (unsigned long int)seg_size);
		fd_offset += sizeof(struct vin_xram_segment_header) + seg_size*2;
		// Set DRAM Address
		cmd_eop_set_dram_address.header.parts.first.bits.rw = VIN_WRITE;
		cmd_eop_set_dram_address.header.parts.first.bits.sc = VIN_SC_NO;
		cmd_eop_set_dram_address.header.parts.first.bits.bc = VIN_BC_NO;
		cmd_eop_set_dram_address.header.parts.first.bits.cmd = VIN_CMD_EOP;
		cmd_eop_set_dram_address.header.parts.first.bits.res = 0;
		cmd_eop_set_dram_address.header.parts.first.bits.chan = 0;
		cmd_eop_set_dram_address.header.parts.second.eop.bits.mod = VIN_MOD_TEST;
		cmd_eop_set_dram_address.header.parts.second.eop.bits.ecmd  = VIN_EOP_SET_DRAM;
		cmd_eop_set_dram_address.header.parts.second.eop.bits.length = 2;
		cmd_eop_set_dram_address.addres1 = seg_addr_start_low;
		cmd_eop_set_dram_address.addres1 = seg_addr_end_low;
		if ((res = vin_write(ctx, &cmd_eop_set_dram_address, sizeof(union vin_cmd) + sizeof(u_int16_t)*2)) < 0) {
			ctx->errorline = __LINE__ - 1;
			ctx->error = errno;
			goto vin_download_edsp_firmware_error;
		}
		// Read DRAM CRC
		cmd_eop_crc_dram.header.parts.first.bits.rw = VIN_READ;
		cmd_eop_crc_dram.header.parts.first.bits.sc = VIN_SC_NO;
		cmd_eop_crc_dram.header.parts.first.bits.bc = VIN_BC_NO;
		cmd_eop_crc_dram.header.parts.first.bits.cmd = VIN_CMD_EOP;
		cmd_eop_crc_dram.header.parts.first.bits.res = 0;
		cmd_eop_crc_dram.header.parts.first.bits.chan = 0;
		cmd_eop_crc_dram.header.parts.second.eop.bits.mod = VIN_MOD_TEST;
		cmd_eop_crc_dram.header.parts.second.eop.bits.ecmd  = VIN_EOP_CRC_DRAM;
		cmd_eop_crc_dram.header.parts.second.eop.bits.length = 1;
		if ((res = vin_read(ctx, cmd_eop_crc_dram.header, &cmd_eop_crc_dram, sizeof(struct vin_cmd_eop_crc_dram)) < 0)) {
			ctx->errorline = __LINE__ - 1;
			ctx->error = errno;
			goto vin_download_edsp_firmware_error;
		}
	}
	printf("\ndram: checksum=0x%04x\n", cmd_eop_crc_dram.crc);
#endif
	close(fd);

	// Check for MBX-EMPTY
	if (vin_check_mbx_empty(ctx) < 0) {
		ctx->errorline = __LINE__ - 1;
		ctx->error = errno;
		goto vin_download_edsp_firmware_error;
	}
	// Download End
	cmd.parts.first.bits.rw = VIN_WRITE;
	cmd.parts.first.bits.sc = VIN_SC_NO;
	cmd.parts.first.bits.bc = VIN_BC_NO;
	cmd.parts.first.bits.cmd = VIN_CMD_EOP;
	cmd.parts.first.bits.res = 0;
	cmd.parts.first.bits.chan = 0;
	cmd.parts.second.eop.bits.mod = VIN_MOD_TEST;
	cmd.parts.second.eop.bits.ecmd  = VIN_EOP_DOWNLOAD_END;
	cmd.parts.second.eop.bits.length = 0;
	if ((res = vin_write(ctx, &cmd, sizeof(union vin_cmd))) < 0) {
		ctx->errorline = __LINE__ - 1;
		ctx->error = errno;
		goto vin_download_edsp_firmware_error;
	}
	// Wait for DL-RDY
	if (vin_wait_dl_rdy(ctx) < 0) {
		ctx->errorline = __LINE__ - 1;
		ctx->error = errno;
		goto vin_download_edsp_firmware_error;
	}
	// Check for MBX-EMPTY
	if (vin_check_mbx_empty(ctx) < 0) {
		ctx->errorline = __LINE__ - 1;
		ctx->error = errno;
		goto vin_download_edsp_firmware_error;
	}
	// Minimazing Command In-Box
	cmd_short.full = VIN_wMINCBX;
	if ((res = vin_write(ctx, &cmd_short.full, sizeof(union vin_cmd_short))) < 0) {
		ctx->errorline = __LINE__ - 1;
		ctx->error = errno;
		goto vin_download_edsp_firmware_error;
	}

	// Start EDSP 
	cmd_short.full = VIN_wSTEDSP;
	if ((res = vin_write(ctx, &cmd_short.full, sizeof(union vin_cmd_short))) < 0) {
		ctx->errorline = __LINE__ - 1;
		ctx->error = errno;
		goto vin_download_edsp_firmware_error;
	}

	return 0;

vin_download_edsp_firmware_error:
	close(fd);
	return -1;
}

int vin_download_alm_dsp(struct vinetic_context *ctx, char *path)
{
	struct vin_cmd_eop_set_fpi_address cmd_eop_set_fpi_address;
	struct vin_cmd_eop_access_fpi_memory cmd_eop_access_fpi_memory;
// 	struct vin_cmd_eop_crc_fpi cmd_eop_crc_fpi;

	u_int32_t addr_start, addr_end;
	size_t data_size, data_chunk_size;
	u_int16_t *data = NULL;
	u_int16_t *datap;
// 	u_int16_t checksum;
// 	u_int16_t dschkr;

	FILE * fp = NULL;
	char fpbuf[32];
	int flag_address = 0;
	int flag_data = 0;
	int flag_checksum = 0;
	int flag_dschkr = 0;

	ssize_t res;
	size_t i;
	u_int32_t tmp_u32;

	// open ALM DSP patch file
	if ((fp = fopen(path, "r")) < 0) {
		ctx->errorline = __LINE__ - 1;
		ctx->error = errno;
		goto vin_download_alm_dsp_error;
	}

	// get address
	while (fgets(fpbuf, sizeof(fpbuf), fp))
	{
		if (!strncasecmp(fpbuf, "[ADDRESS]", strlen("[ADDRESS]"))) {
			flag_address = 1;
			break;
		}
	}
	if (flag_address) {
		// get start address
		if (fgets(fpbuf, sizeof(fpbuf), fp)) {
			if (sscanf(fpbuf, "0x%08X", &tmp_u32) != 1) {
				ctx->errorline = __LINE__ - 1;
				ctx->error = errno;
				goto vin_download_alm_dsp_error;
			}
		} else {
			ctx->errorline = __LINE__ - 1;
			ctx->error = errno;
			goto vin_download_alm_dsp_error;
		}
		addr_start = tmp_u32;
		// get end address
		if (fgets(fpbuf, sizeof(fpbuf), fp)) {
			if (sscanf(fpbuf, "0x%08X", &tmp_u32) != 1) {
				ctx->errorline = __LINE__ - 1;
				ctx->error = errno;
				goto vin_download_alm_dsp_error;
			}
		} else {
			ctx->errorline = __LINE__ - 1;
			ctx->error = errno;
			goto vin_download_alm_dsp_error;
		}
		addr_end = tmp_u32;
	} else {
		ctx->errorline = __LINE__ - 1;
		ctx->error = errno;
		goto vin_download_alm_dsp_error;
	}
// 	printf("\naddr start=0x%08x\n", addr_start);
// 	printf("addr end=0x%08x\n", addr_end);

	// get data
	data_size = (addr_end - addr_start + 1);
// 	printf("data size=%lu\n", data_size);
	while (fgets(fpbuf, sizeof(fpbuf), fp))
	{
		if (!strncasecmp(fpbuf, "[DATA]", strlen("[DATA]"))) {
			flag_data = 1;
			break;
		}
	}
	if (flag_data) {
		// alloc data buffer
		if ((data = malloc(data_size * 2))) {
			datap = data;
			//
			for (i=0; i<data_size; i++)
			{
				if (fgets(fpbuf, sizeof(fpbuf), fp)) {
					if (sscanf(fpbuf, "0x%04X", &tmp_u32) != 1) {
						ctx->errorline = __LINE__ - 1;
						ctx->error = errno;
						goto vin_download_alm_dsp_error;
					}
				} else {
					ctx->errorline = __LINE__ - 1;
					ctx->error = errno;
					goto vin_download_alm_dsp_error;
				}
			*datap++ = tmp_u32 & 0xffff;
			}
		} else {
			ctx->errorline = __LINE__ - 1;
			ctx->error = errno;
			goto vin_download_alm_dsp_error;
		}
	}

	// get checksum
	while (fgets(fpbuf, sizeof(fpbuf), fp))
	{
		if (!strncasecmp(fpbuf, "[CHECKSUM]", strlen("[CHECKSUM]"))) {
			flag_checksum = 1;
			break;
		}
	}
	if (flag_checksum) {
		if (fgets(fpbuf, sizeof(fpbuf), fp)) {
			if (sscanf(fpbuf, "0x%04X", &tmp_u32) != 1) {
				ctx->errorline = __LINE__ - 1;
				ctx->error = errno;
				goto vin_download_alm_dsp_error;
			}
		} else {
			ctx->errorline = __LINE__ - 1;
			ctx->error = errno;
			goto vin_download_alm_dsp_error;
		}
// 		checksum = tmp_u32 & 0xffff;
	}
// 	printf("checksum=0x%04x\n", checksum);

	// get dschkr
	while (fgets(fpbuf, sizeof(fpbuf), fp))
	{
		if (!strncasecmp(fpbuf, "[DSCHKR]", strlen("[DSCHKR]"))) {
			flag_dschkr = 1;
			break;
		}
	}
	if (flag_dschkr) {
		if (fgets(fpbuf, sizeof(fpbuf), fp)) {
			if (sscanf(fpbuf, "0x%04X", &tmp_u32) != 1) {
				ctx->errorline = __LINE__ - 1;
				ctx->error = errno;
				goto vin_download_alm_dsp_error;
			}
		} else {
			ctx->errorline = __LINE__ - 1;
			ctx->error = errno;
			goto vin_download_alm_dsp_error;
		}
// 		dschkr = tmp_u32 & 0xffff;
	}
// 	printf("dschkr=0x%04x\n", dschkr);

	// Check for MBX-EMPTY
	if (vin_check_mbx_empty(ctx) < 0) {
		ctx->errorline = __LINE__ - 1;
		ctx->error = errno;
		goto vin_download_alm_dsp_error;
	}
	// Set FPI Address
	cmd_eop_set_fpi_address.header.parts.first.bits.rw = VIN_WRITE;
	cmd_eop_set_fpi_address.header.parts.first.bits.sc = VIN_SC_NO;
	cmd_eop_set_fpi_address.header.parts.first.bits.bc = VIN_BC_NO;
	cmd_eop_set_fpi_address.header.parts.first.bits.cmd = VIN_CMD_EOP;
	cmd_eop_set_fpi_address.header.parts.first.bits.res = 0;
	cmd_eop_set_fpi_address.header.parts.first.bits.chan = 0;
	cmd_eop_set_fpi_address.header.parts.second.eop.bits.mod = VIN_MOD_TEST;
	cmd_eop_set_fpi_address.header.parts.second.eop.bits.ecmd  = VIN_EOP_SET_FPI;
	cmd_eop_set_fpi_address.header.parts.second.eop.bits.length = 4;
	cmd_eop_set_fpi_address.high_addres1 = (addr_start >> 16) & 0xffff;
	cmd_eop_set_fpi_address.low_addres1 = addr_start & 0xffff;
	cmd_eop_set_fpi_address.high_addres2 = (addr_end >> 16) & 0xffff;
	cmd_eop_set_fpi_address.low_addres2 = addr_end & 0xffff;
	if ((res = vin_write(ctx, &cmd_eop_set_fpi_address, sizeof(struct vin_cmd_eop_set_fpi_address))) < 0) {
		ctx->errorline = __LINE__ - 1;
		ctx->error = errno;
		goto vin_download_alm_dsp_error;
	}
#if 0
	// CRC FPI - reset
	cmd_eop_crc_fpi.header.parts.first.bits.rw = VIN_WRITE;
	cmd_eop_crc_fpi.header.parts.first.bits.sc = VIN_SC_NO;
	cmd_eop_crc_fpi.header.parts.first.bits.bc = VIN_BC_NO;
	cmd_eop_crc_fpi.header.parts.first.bits.cmd = VIN_CMD_EOP;
	cmd_eop_crc_fpi.header.parts.first.bits.res = 0;
	cmd_eop_crc_fpi.header.parts.first.bits.chan = 0;
	cmd_eop_crc_fpi.header.parts.second.eop.bits.mod = VIN_MOD_TEST;
	cmd_eop_crc_fpi.header.parts.second.eop.bits.ecmd  = VIN_EOP_CRC_FPI;
	cmd_eop_crc_fpi.header.parts.second.eop.bits.length = 0;
	if ((res = vin_write(ctx, &cmd_eop_crc_fpi, sizeof(union vin_cmd))) < 0) {
		ctx->errorline = __LINE__ - 1;
		ctx->error = errno;
		goto vin_download_alm_dsp_error;
	}
#endif
	// Write FPI data
	datap = data;
	while (data_size)
	{
		// Check for MBX-EMPTY
		if (vin_check_mbx_empty(ctx) < 0) {
			ctx->errorline = __LINE__ - 1;
			ctx->error = errno;
			goto vin_download_alm_dsp_error;
		}
		data_chunk_size = (data_size < 29) ? (data_size) : (29);
		for (i=0; i<data_chunk_size; i++)
			memcpy(&cmd_eop_access_fpi_memory.data[i], datap++, sizeof(u_int16_t));
		// Access FPI Memory
		cmd_eop_access_fpi_memory.header.parts.first.bits.rw = VIN_WRITE;
		cmd_eop_access_fpi_memory.header.parts.first.bits.sc = VIN_SC_NO;
		cmd_eop_access_fpi_memory.header.parts.first.bits.bc = VIN_BC_NO;
		cmd_eop_access_fpi_memory.header.parts.first.bits.cmd = VIN_CMD_EOP;
		cmd_eop_access_fpi_memory.header.parts.first.bits.res = 0;
		cmd_eop_access_fpi_memory.header.parts.first.bits.chan = 0;
		cmd_eop_access_fpi_memory.header.parts.second.eop.bits.mod = VIN_MOD_TEST;
		cmd_eop_access_fpi_memory.header.parts.second.eop.bits.ecmd  = VIN_EOP_ACCESS_DRAM;
		cmd_eop_access_fpi_memory.header.parts.second.eop.bits.length = data_chunk_size;
		if ((res = vin_write(ctx, &cmd_eop_access_fpi_memory, sizeof(union vin_cmd) + data_chunk_size*2)) < 0) {
			ctx->errorline = __LINE__ - 1;
			ctx->error = errno;
			goto vin_download_alm_dsp_error;
		}
		data_size -= data_chunk_size;
	}

#if 0
	// CRC FPI - reset
	cmd_eop_crc_fpi.header.parts.first.bits.rw = VIN_WRITE;
	cmd_eop_crc_fpi.header.parts.first.bits.sc = VIN_SC_NO;
	cmd_eop_crc_fpi.header.parts.first.bits.bc = VIN_BC_NO;
	cmd_eop_crc_fpi.header.parts.first.bits.cmd = VIN_CMD_EOP;
	cmd_eop_crc_fpi.header.parts.first.bits.res = 0;
	cmd_eop_crc_fpi.header.parts.first.bits.chan = 0;
	cmd_eop_crc_fpi.header.parts.second.eop.bits.mod = VIN_MOD_TEST;
	cmd_eop_crc_fpi.header.parts.second.eop.bits.ecmd  = VIN_EOP_CRC_FPI;
	cmd_eop_crc_fpi.header.parts.second.eop.bits.length = 0;
	if ((res = vin_write(ctx, &cmd_eop_crc_fpi, sizeof(union vin_cmd))) < 0) {
		ctx->errorline = __LINE__ - 1;
		ctx->error = errno;
		goto vin_download_alm_dsp_error;
	}
	// CRC FPI - read
	cmd_eop_crc_fpi.header.parts.first.bits.rw = VIN_READ;
	cmd_eop_crc_fpi.header.parts.first.bits.sc = VIN_SC_NO;
	cmd_eop_crc_fpi.header.parts.first.bits.bc = VIN_BC_NO;
	cmd_eop_crc_fpi.header.parts.first.bits.cmd = VIN_CMD_EOP;
	cmd_eop_crc_fpi.header.parts.first.bits.res = 0;
	cmd_eop_crc_fpi.header.parts.first.bits.chan = 0;
	cmd_eop_crc_fpi.header.parts.second.eop.bits.mod = VIN_MOD_TEST;
	cmd_eop_crc_fpi.header.parts.second.eop.bits.ecmd  = VIN_EOP_CRC_FPI;
	cmd_eop_crc_fpi.header.parts.second.eop.bits.length = 1;
	if (vin_read(ctx, cmd_eop_crc_fpi.header, &cmd_eop_crc_fpi, sizeof(struct vin_cmd_eop_crc_fpi)) < 0) {
		ctx->errorline = __LINE__ - 1;
		ctx->error = errno;
		goto vin_download_alm_dsp_error;
	}
	printf("crc=0x%04x\n", cmd_eop_crc_fpi.crc);
#endif
#if 0
	// DSCHKR
	struct vin_cmd_sop_dschkr cmd_sop_dschkr;
	// read
	cmd_sop_dschkr.header.parts.first.bits.rw = VIN_READ;
	cmd_sop_dschkr.header.parts.first.bits.sc = VIN_SC_NO;
	cmd_sop_dschkr.header.parts.first.bits.bc = VIN_BC_NO;
	cmd_sop_dschkr.header.parts.first.bits.cmd = VIN_CMD_SOP;
	cmd_sop_dschkr.header.parts.second.sop.bits.offset = VIN_SOP_DSCHKR;
	cmd_sop_dschkr.header.parts.second.sop.bits.length  = 1;
	if (vin_read(ctx, cmd_sop_dschkr.header, &cmd_sop_dschkr, sizeof(struct vin_cmd_sop_dschkr)) < 0) {
		ctx->errorline = __LINE__ - 1;
		ctx->error = errno;
		goto vin_download_alm_dsp_error;
	}
	printf("crc=0x%04x\n", cmd_sop_dschkr.ds_check);
#endif

	free(data);
	fclose(fp);
	return 0;

vin_download_alm_dsp_error:
	if (data) free(data);
	if (fp) fclose(fp);
	return -1;
}

int vin_jump_alm_dsp(struct vinetic_context *ctx, unsigned int chan)
{
	struct vin_cmd_sop_ccr cmd_sop_ccr;

	ssize_t res;

	// Common Configuration Register
	cmd_sop_ccr.header.parts.first.bits.rw = VIN_WRITE;
	cmd_sop_ccr.header.parts.first.bits.sc = VIN_SC_NO;
	cmd_sop_ccr.header.parts.first.bits.bc = VIN_BC_NO;
	cmd_sop_ccr.header.parts.first.bits.cmd = VIN_CMD_SOP;
	cmd_sop_ccr.header.parts.first.bits.res = 0;
	cmd_sop_ccr.header.parts.first.bits.chan = chan;
	cmd_sop_ccr.header.parts.second.sop.bits.offset = VIN_SOP_CCR;
	cmd_sop_ccr.header.parts.second.sop.bits.length = 1;
	cmd_sop_ccr.ccr.pd_cbias = 0;
	cmd_sop_ccr.ccr.pd_cvcm = 0;
	cmd_sop_ccr.ccr.jump_ac1 = 0;
	cmd_sop_ccr.ccr.jump_ac2 = 0;
	cmd_sop_ccr.ccr.jump_ac3 = 1;
	cmd_sop_ccr.ccr.jump_dc = 0;
	cmd_sop_ccr.ccr.jump_res0 = 0;
	if ((res = vin_write(ctx, &cmd_sop_ccr, sizeof(struct vin_cmd_sop_ccr))) < 0) {
		ctx->errorline = __LINE__ - 1;
		ctx->error = errno;
		goto vin_jump_alm_dsp_error;
	}
	return 0;

vin_jump_alm_dsp_error:
	return -1;
}

int vin_coder_channel_jb_statistic_reset(struct vinetic_context *ctx, unsigned int chan)
{
	union vin_cmd cmd;

	cmd.parts.first.bits.rw = VIN_WRITE;
	cmd.parts.first.bits.sc = VIN_SC_NO;
	cmd.parts.first.bits.bc = VIN_BC_NO;
	cmd.parts.first.bits.cmd = VIN_CMD_EOP;
	cmd.parts.first.bits.res = 0;
	cmd.parts.first.bits.chan = chan & 0x7;
	cmd.parts.second.eop.bits.mod = VIN_MOD_CODER;
	cmd.parts.second.eop.bits.ecmd  = VIN_EOP_CODER_JBSTAT;
	cmd.parts.second.eop.bits.length = 0;
	if (vin_write(ctx, &cmd, sizeof(union vin_cmd)) < 0) {
		ctx->errorline = __LINE__ - 1;
		ctx->error = errno;
		goto vin_coder_channel_jb_statistic_reset_error;
	}

	return 0;

vin_coder_channel_jb_statistic_reset_error:
	return -1;

}

int vin_read_fw_version(struct vinetic_context *ctx)
{
	struct vin_cmd_eop_edsp_sw_version_register cmd_eop_edsp_sw_version_register;

	// Read EDSP SW Version Register
	cmd_eop_edsp_sw_version_register.header.parts.first.bits.rw = VIN_READ;
	cmd_eop_edsp_sw_version_register.header.parts.first.bits.sc = VIN_SC_NO;
	cmd_eop_edsp_sw_version_register.header.parts.first.bits.bc = VIN_BC_NO;
	cmd_eop_edsp_sw_version_register.header.parts.first.bits.cmd = VIN_CMD_EOP;
	cmd_eop_edsp_sw_version_register.header.parts.first.bits.res = 0;
	cmd_eop_edsp_sw_version_register.header.parts.first.bits.chan = 0;
	cmd_eop_edsp_sw_version_register.header.parts.second.eop.bits.mod = VIN_MOD_TEST;
	cmd_eop_edsp_sw_version_register.header.parts.second.eop.bits.ecmd  = VIN_EOP_EDSPSWVERSREG;
	cmd_eop_edsp_sw_version_register.header.parts.second.eop.bits.length = 2;

	if (vin_read(ctx, cmd_eop_edsp_sw_version_register.header, &cmd_eop_edsp_sw_version_register,
			sizeof(struct vin_cmd_eop_edsp_sw_version_register)) < 0) {
		ctx->error = errno;
		goto vin_read_fw_version_error;
	}

	memcpy(&ctx->edsp_sw_version_register, &cmd_eop_edsp_sw_version_register.edsp_sw_version_register, sizeof(struct vin_edsp_sw_version_register));

	return 0;

vin_read_fw_version_error:
	return -1;
}

double vin_gainem_to_gaindb(u_int8_t em)
{
	double g;
	u_int8_t e = (em >> 5) & 0x7;
	u_int8_t m = em & 0x1f;
	g = pow(2, (9 - e)) * (32 + m);
	return 24.08 + 20 * log10(g / 32768);
}

u_int8_t vin_gaindb_to_gainem(double g)
{
	if(g > VIN_GAINDB_MAX) g = VIN_GAINDB_MAX;
	if(g < VIN_GAINDB_MIN) g = VIN_GAINDB_MIN;
	//
	double fe = ceil(3 - (g / 6.02));
	double fm = trunc(powf(10, (g / 20)) * pow(2, (2 + fe)) - 32);
	u_int8_t e = ((u_int8_t)fe) & 0x7;
	u_int8_t m = ((u_int8_t)fm) & 0x1f;
	return (e << 5) | m;
}

/******************************************************************************/
/* end of libvinetic.c                                                        */
/******************************************************************************/
