/******************************************************************************/
/* vinetic-def.h                                                              */
/******************************************************************************/

#ifndef __VINETIC_DEF_H__
#define __VINETIC_DEF_H__

#include <linux/types.h>

/*!
 * \brief VINETIC data packet type
 */
enum {
	VIN_CMD_SOP = 0x01,			/*! Status Operation Command Packet */
	VIN_CMD_COP = 0x02,			/*! Coefficient Operation Command Packet */
	VIN_CMD_IOP = 0x03,			/*! Interface Operation Command Packet */
	VIN_CMD_VOP = 0x04,			/*! Voice or SID Packets */
	VIN_CMD_EVT = 0x05,			/*! Event Packet */
	VIN_CMD_EOP = 0x06,			/*! EDSP Operation Command Packet */
	VIN_CMD_FRD = 0x10,			/*! FAX Relay Data Packet  */
	VIN_CMD_FRS = 0x11,			/*! FAX Relay Status Packet */
	VIN_CMD_CIDR = 0x14,		/*! CID Receiver Packet */
};
/*!
 * \brief VINETIC read/write flag
 */
enum {
	VIN_WRITE = 0,
	VIN_READ = 1,
};
/*!
 * \brief VINETIC short command flag
 */
enum {
	VIN_SC_NO = 0,
	VIN_SC = 1,
};
/*!
 * \brief VINETIC broadcast command flag
 */
enum {
	VIN_BC_NO = 0,
	VIN_BC = 1,
};
/*!
 * \brief VINETIC enable/disable flag
 */
enum {
	VIN_DIS = 0,
	VIN_EN = 1,
};
/*!
 * \brief VINETIC on/off flag
 */
enum {
	VIN_OFF = 0,
	VIN_ON = 1,
};
/*!
 * \brief VINETIC active/inactive flag
 */
enum {
	VIN_INACTIVE = 0,
	VIN_ACTIVE = 1,
};
/*!
 * \brief VINETIC SIGNAL ARRAY
 */
enum {
	VIN_SIG_NULL = 0x00,
	/*! PCM */
	VIN_SIG_PCM_OUT00 = 0x01,
	VIN_SIG_PCM_OUT01 = 0x02,
	VIN_SIG_PCM_OUT02 = 0x03,
	VIN_SIG_PCM_OUT03 = 0x04,
	VIN_SIG_PCM_OUT04 = 0x05,
	VIN_SIG_PCM_OUT05 = 0x06,
	VIN_SIG_PCM_OUT06 = 0x07,
	VIN_SIG_PCM_OUT07 = 0x08,
	VIN_SIG_PCM_OUT08 = 0x09,
	VIN_SIG_PCM_OUT09 = 0x0A,
	VIN_SIG_PCM_OUT10 = 0x0B,
	VIN_SIG_PCM_OUT11 = 0x0C,
	VIN_SIG_PCM_OUT12 = 0x0D,
	VIN_SIG_PCM_OUT13 = 0x0E,
	VIN_SIG_PCM_OUT14 = 0x0F,
	VIN_SIG_PCM_OUT15 = 0x10,
	/*! ALM */
	VIN_SIG_ALM_OUT00 = 0x11,
	VIN_SIG_ALM_OUT01 = 0x12,
	VIN_SIG_ALM_OUT02 = 0x13,
	VIN_SIG_ALM_OUT03 = 0x14,
	/*! CODER */
	VIN_SIG_COD_OUT00 = 0x18,
	VIN_SIG_COD_OUT01 = 0x19,
	VIN_SIG_COD_OUT02 = 0x1A,
	VIN_SIG_COD_OUT03 = 0x1B,
	VIN_SIG_COD_OUT04 = 0x1C,
	VIN_SIG_COD_OUT05 = 0x1D,
	VIN_SIG_COD_OUT06 = 0x1E,
	VIN_SIG_COD_OUT07 = 0x1F,
	/*! SIGNALING */
	VIN_SIG_SIG_OUTA0 = 0x28,
	VIN_SIG_SIG_OUTB0 = 0x29,
	VIN_SIG_SIG_OUTA1 = 0x2A,
	VIN_SIG_SIG_OUTB1 = 0x2B,
	VIN_SIG_SIG_OUTA2 = 0x2C,
	VIN_SIG_SIG_OUTB2 = 0x2D,
	VIN_SIG_SIG_OUTA3 = 0x2E,
	VIN_SIG_SIG_OUTB3 = 0x2F,
};

union vin_reg_ir {
	struct vin_reg_ir_bits {
		u_int16_t hw_stat:1;
		u_int16_t mbx_evt:1;
		u_int16_t gpio:1;
		u_int16_t res0:1;
		u_int16_t reset:1;
		u_int16_t res1:2;
		u_int16_t rdyq:1;
		u_int16_t isr0:1;
		u_int16_t isr1:1;
		u_int16_t isr2:1;
		u_int16_t isr3:1;
		u_int16_t isre4:1;
		u_int16_t isre5:1;
		u_int16_t isre6:1;
		u_int16_t isre7:1;
	} __attribute__((packed)) bits;
	u_int16_t full;
} __attribute__((packed));

union vin_reg_sre1 {
	struct vin_reg_sre1_bits {
		u_int16_t cis_act:1;
		u_int16_t cis_req:1;
		u_int16_t cis_buf:1;
		u_int16_t dtmfg_act:1;
		u_int16_t dtmfg_req:1;
		u_int16_t dtmfg_buf:1;
		u_int16_t res:2;
		u_int16_t utd2_ok:1;
		u_int16_t utd1_ok:1;
		u_int16_t dtmfr_dtc:4;
		u_int16_t dtmfr_pdt:1;
		u_int16_t dtmfr_dt:1;
	} __attribute__((packed)) bits;
	u_int16_t full;
} __attribute__((packed));

union vin_reg_sre2 {
	struct vin_reg_sre2_bits {
		u_int16_t dec_chg:1;
		u_int16_t dec_err:1;
		u_int16_t vpou_jbh:1;
		u_int16_t vpou_jbl:1;
		u_int16_t res0:1;
		u_int16_t pvpu_of:1;
		u_int16_t etu_of:1;
		u_int16_t res1:1;
		u_int16_t atd2_am:1;
		u_int16_t atd2_npr:2;
		u_int16_t atd2_dt:1;
		u_int16_t atd1_am:1;
		u_int16_t atd1_npr:2;
		u_int16_t atd1_dt:1;
	} __attribute__((packed)) bits;
	u_int16_t full;
} __attribute__((packed));

union vin_reg_srs1 {
	struct vin_reg_srs1_bits {
		u_int16_t du_io:5;
		u_int16_t lm_ok:1;
		u_int16_t lm_tres:1;
		u_int16_t res0:1;
		u_int16_t vtrlim:1;
		u_int16_t icon:1;
		u_int16_t gndkh:1;
		u_int16_t gnkp:1;
		u_int16_t gndk:1;
		u_int16_t hook:1;
		u_int16_t ramp_ready:1;
		u_int16_t res1:1;
	} __attribute__((packed)) bits;
	u_int16_t full;
} __attribute__((packed));

union vin_reg_srs2 {
	struct vin_reg_srs2_bits {
		u_int16_t cs_fail_dcctl:1;
		u_int16_t cs_fail_dsp:1;
		u_int16_t cs_fail_cram:1;
		u_int16_t otemp:1;
		u_int16_t res:12;
	} __attribute__((packed)) bits;
	u_int16_t full;
} __attribute__((packed));

union vin_reg_bxsr1 {
	struct vin_reg_bxsr1_bits {
		u_int16_t res0:8;
		u_int16_t cerr:1;
		u_int16_t res1:7;
	} __attribute__((packed)) bits;
	u_int16_t full;
} __attribute__((packed));

union vin_reg_bxsr2 {
	struct vin_reg_bxsr2_bits {
		u_int16_t mbx_empty:1;
		u_int16_t cibx_of:1;
		u_int16_t pibx_of:1;
		u_int16_t cobx_data:1;
		u_int16_t pobx_data:1;
		u_int16_t host_err:1;
		u_int16_t res:10;
	} __attribute__((packed)) bits;
	u_int16_t full;
} __attribute__((packed));

union vin_reg_hwsr1 {
	struct vin_reg_hwsr1_bits {
		u_int16_t hw_err:1;
		u_int16_t res0:1;
		u_int16_t woke_up:1;
		u_int16_t mclk_fail:1;
		u_int16_t res1:3;
		u_int16_t edsp_wd_fail:1;
		u_int16_t txa_crash:1;
		u_int16_t txb_crash:1;
		u_int16_t res2:1;
		u_int16_t sync_fail:1;
		u_int16_t res3:4;
	} __attribute__((packed)) bits;
	u_int16_t full;
} __attribute__((packed));

union vin_reg_hwsr2 {
	struct vin_reg_hwsr2_bits {
		u_int16_t edsp_mips_ol:1;
		u_int16_t crc_err:1;
		u_int16_t crc_rdy:1;
		u_int16_t res0:5;
		u_int16_t dl_rdy:1;
		u_int16_t res2:7;
	} __attribute__((packed)) bits;
	u_int16_t full;
} __attribute__((packed));

union vin_reg_srgpio {
	struct vin_reg_srgpio_bits {
		u_int16_t gpio0:1;
		u_int16_t gpio1:1;
		u_int16_t gpio2:1;
		u_int16_t gpio3:1;
		u_int16_t gpio4:1;
		u_int16_t gpio5:1;
		u_int16_t gpio6:1;
		u_int16_t gpio7:1;
		u_int16_t res:8;
	} __attribute__((packed)) bits;
	u_int16_t full;
} __attribute__((packed));

/*!
 * \brief VINETIC Short Command
 */
union vin_cmd_short {
	struct vin_cmd_short_bits {
		u_int16_t chan:4;
		u_int16_t subcmd:4;
		u_int16_t cmd:4;
		u_int16_t om:1;
		u_int16_t bc:1;
		u_int16_t sc:1;	// 1 - short command
		u_int16_t rw:1;	// 1 - read, 0 - write
	} __attribute__((packed)) bits;
	u_int16_t full;
} __attribute__((packed));

typedef union vin_reg_ir vin_read_ir_t;

struct vin_read_sr_bc {
	union vin_reg_sre1 sre1_0;
	union vin_reg_sre2 sre2_0;
	union vin_reg_srs1 srs1_0;
	union vin_reg_srs2 srs2_0;
	union vin_reg_sre1 sre1_1;
	union vin_reg_sre2 sre2_1;
	union vin_reg_srs1 srs1_1;
	union vin_reg_srs2 srs2_1;
	union vin_reg_sre1 sre1_2;
	union vin_reg_sre2 sre2_2;
	union vin_reg_srs1 srs1_2;
	union vin_reg_srs2 srs2_2;
	union vin_reg_sre1 sre1_3;
	union vin_reg_sre2 sre2_3;
	union vin_reg_srs1 srs1_3;
	union vin_reg_srs2 srs2_3;
	union vin_reg_sre1 sre1_4;
	union vin_reg_sre2 sre2_4;
	union vin_reg_sre1 sre1_5;
	union vin_reg_sre2 sre2_5;
	union vin_reg_sre1 sre1_6;
	union vin_reg_sre2 sre2_6;
	union vin_reg_sre1 sre1_7;
	union vin_reg_sre2 sre2_7;
} __attribute__((packed));

typedef struct vin_read_sr_bc vin_read_sr_bc_t;

struct vin_read_bxsr {
	union vin_reg_bxsr1 bxsr1;
	union vin_reg_bxsr2 bxsr2;
} __attribute__((packed));

typedef struct vin_read_bxsr vin_read_bxsr_t;

struct vin_read_hwsr {
	union vin_reg_hwsr1 hwsr1;
	union vin_reg_hwsr2 hwsr2;
} __attribute__((packed));

typedef struct vin_read_hwsr vin_read_hwsr_t;

struct vin_status_registers {
	vin_read_sr_bc_t sr;
	vin_read_hwsr_t hwsr;
	vin_read_bxsr_t bxsr;
};


enum {
	VIN_SH_CMD_CODE_rIR			= 0x000, // 0 0000 0000
	VIN_SH_CMD_CODE_rSR			= 0x001, // 0 0000 0001
	VIN_SH_CMD_CODE_rHWSR		= 0x002, // 0 0000 0010
	VIN_SH_CMD_CODE_rSRGPIO		= 0x003, // 0 0000 0011
	VIN_SH_CMD_CODE_rBXSR		= 0x004, // 0 0000 0100
	VIN_SH_CMD_CODE_rSRS		= 0x006, // 0 0000 0110
	VIN_SH_CMD_CODE_rI_SR		= 0x009, // 0 0000 1001
	VIN_SH_CMD_CODE_rI_HWSR		= 0x00A, // 0 0000 1010
	VIN_SH_CMD_CODE_rI_SRGPIO	= 0x00B, // 0 0000 1011
	VIN_SH_CMD_CODE_rI_BXSR		= 0x00C, // 0 0000 1100
	VIN_SH_CMD_CODE_wCHECKSUM	= 0x00D, // 0 0000 1101
	VIN_SH_CMD_CODE_rI_SRS		= 0x00E, // 0 0000 1110

	VIN_SH_CMD_CODE_rFIBXMS		= 0x010, // 0 0001 0000
	VIN_SH_CMD_CODE_rOBXML		= 0x011, // 0 0001 0001
	VIN_SH_CMD_CODE_rPOBX		= 0x012, // 0 0001 0010
	VIN_SH_CMD_CODE_rCOBX		= 0x013, // 0 0001 0011
	VIN_SH_CMD_CODE_wMAXCBX		= 0x014, // 0 0001 0100
	VIN_SH_CMD_CODE_wMINCBX		= 0x015, // 0 0001 0101

	VIN_SH_CMD_CODE_wLEMP		= 0x023, // 0 0010 0011
	VIN_SH_CMD_CODE_wSTEDSP		= 0x024, // 0 0010 0100

	VIN_SH_CMD_CODE_wLPMP		= 0x030, // 0 0011 0000
	VIN_SH_CMD_CODE_wSWRST		= 0x031, // 0 0011 0001
	VIN_SH_CMD_CODE_wRESYNC		= 0x032, // 0 0011 0010
	VIN_SH_CMD_CODE_wPHIERR		= 0x033, // 0 0011 0011

	VIN_OP_MODE_POWER_DOWN_HIGH_IMPEDANCE	= 0x100, // 1 0000 0000
	VIN_OP_MODE_ACTIVE_HIGH_VBATH			= 0x120, // 1 0010 0000
};

#define VIN_CMD_SHORT_BUILD(_rw, _bc, _om, _cmd, _subcmd, _chan) \
	((_rw & 1) << 15) \
	| (1 << 14) \
	| ((_bc & 1) << 13) \
	| ((_om & 1) << 12) \
	| ((_cmd & 0xf) << 8) \
	| ((_subcmd & 0xf) << 4) \
	| ((_chan & 0xf) << 0)

#define VIN_rIR \
	VIN_CMD_SHORT_BUILD(VIN_READ, 0, (VIN_SH_CMD_CODE_rIR >> 8) & 0x1, (VIN_SH_CMD_CODE_rIR >> 4) & 0xf, VIN_SH_CMD_CODE_rIR & 0xf, 0)

#define VIN_rSR(_bc, _chan) \
	VIN_CMD_SHORT_BUILD(VIN_READ, _bc, (VIN_SH_CMD_CODE_rSR >> 8) & 0x1, (VIN_SH_CMD_CODE_rSR >> 4) & 0xf, VIN_SH_CMD_CODE_rSR & 0xf, _chan)

#define VIN_rHWSR \
	VIN_CMD_SHORT_BUILD(VIN_READ, 0, (VIN_SH_CMD_CODE_rHWSR >> 8) & 0x1, (VIN_SH_CMD_CODE_rHWSR >> 4) & 0xf, VIN_SH_CMD_CODE_rHWSR & 0xf, 0)

#define VIN_rSRGPIO \
	VIN_CMD_SHORT_BUILD(VIN_READ, 0, (VIN_SH_CMD_CODE_rSRGPIO >> 8) & 0x1, (VIN_SH_CMD_CODE_rSRGPIO >> 4) & 0xf, VIN_SH_CMD_CODE_rSRGPIO & 0xf, 0)

#define VIN_rBXSR \
	VIN_CMD_SHORT_BUILD(VIN_READ, 0, (VIN_SH_CMD_CODE_rBXSR >> 8) & 0x1, (VIN_SH_CMD_CODE_rBXSR >> 4) & 0xf, VIN_SH_CMD_CODE_rBXSR & 0xf, 0)

#define VIN_rSRS(_chan) \
	VIN_CMD_SHORT_BUILD(VIN_READ, 0, (VIN_SH_CMD_CODE_rSRS >> 8) & 0x1, (VIN_SH_CMD_CODE_rSRS >> 4) & 0xf, VIN_SH_CMD_CODE_rSRS & 0xf, _chan)

#define VIN_rI_SR(_bc, _chan) \
	VIN_CMD_SHORT_BUILD(VIN_READ, _bc, (VIN_SH_CMD_CODE_rI_SR >> 8) & 0x1, (VIN_SH_CMD_CODE_rI_SR >> 4) & 0xf, VIN_SH_CMD_CODE_rI_SR & 0xf, _chan)

#define VIN_rI_HWSR \
	VIN_CMD_SHORT_BUILD(VIN_READ, 0, (VIN_SH_CMD_CODE_rI_HWSR >> 8) & 0x1, (VIN_SH_CMD_CODE_rI_HWSR >> 4) & 0xf, VIN_SH_CMD_CODE_rI_HWSR & 0xf, 0)

#define VIN_rI_SRGPIO \
	VIN_CMD_SHORT_BUILD(VIN_READ, 0, (VIN_SH_CMD_CODE_rI_SRGPIO >> 8) & 0x1, (VIN_SH_CMD_CODE_rI_SRGPIO >> 4) & 0xf, VIN_SH_CMD_CODE_rI_SRGPIO & 0xf, 0)

#define VIN_rI_BXSR \
	VIN_CMD_SHORT_BUILD(VIN_READ, 0, (VIN_SH_CMD_CODE_rI_BXSR >> 8) & 0x1, (VIN_SH_CMD_CODE_rI_BXSR >> 4) & 0xf, VIN_SH_CMD_CODE_rI_BXSR & 0xf, 0)

#define VIN_wCHECKSUM \
	VIN_CMD_SHORT_BUILD(VIN_WRITE, 0, (VIN_SH_CMD_CODE_wCHECKSUM >> 8) & 0x1, (VIN_SH_CMD_CODE_wCHECKSUM >> 4) & 0xf, VIN_SH_CMD_CODE_wCHECKSUM & 0xf, 0)

#define VIN_rI_SRS(_chan) \
	VIN_CMD_SHORT_BUILD(VIN_READ, 0, (VIN_SH_CMD_CODE_rI_SRS >> 8) & 0x1, (VIN_SH_CMD_CODE_rI_SRS >> 4) & 0xf, VIN_SH_CMD_CODE_rI_SRS & 0xf, _chan)

#define VIN_rFIBXMS \
	VIN_CMD_SHORT_BUILD(VIN_READ, 0, (VIN_SH_CMD_CODE_rFIBXMS >> 8) & 0x1, (VIN_SH_CMD_CODE_rFIBXMS >> 4) & 0xf, VIN_SH_CMD_CODE_rFIBXMS & 0xf, 0)

#define VIN_rOBXML \
	VIN_CMD_SHORT_BUILD(VIN_READ, 0, (VIN_SH_CMD_CODE_rOBXML >> 8) & 0x1, (VIN_SH_CMD_CODE_rOBXML >> 4) & 0xf, VIN_SH_CMD_CODE_rOBXML & 0xf, 0)

#define VIN_rPOBX \
	VIN_CMD_SHORT_BUILD(VIN_READ, 0, (VIN_SH_CMD_CODE_rPOBX >> 8) & 0x1, (VIN_SH_CMD_CODE_rPOBX >> 4) & 0xf, VIN_SH_CMD_CODE_rPOBX & 0xf, 0)

#define VIN_rCOBX \
	VIN_CMD_SHORT_BUILD(VIN_READ, 0, (VIN_SH_CMD_CODE_rCOBX >> 8) & 0x1, (VIN_SH_CMD_CODE_rCOBX >> 4) & 0xf, VIN_SH_CMD_CODE_rCOBX & 0xf, 0)

#define VIN_wMAXCBX \
	VIN_CMD_SHORT_BUILD(VIN_WRITE, 0, (VIN_SH_CMD_CODE_wMAXCBX >> 8) & 0x1, (VIN_SH_CMD_CODE_wMAXCBX >> 4) & 0xf, VIN_SH_CMD_CODE_wMAXCBX & 0xf, 0)

#define VIN_wMINCBX \
	VIN_CMD_SHORT_BUILD(VIN_WRITE, 0, (VIN_SH_CMD_CODE_wMINCBX >> 8) & 0x1, (VIN_SH_CMD_CODE_wMINCBX >> 4) & 0xf, VIN_SH_CMD_CODE_wMINCBX & 0xf, 0)

#define VIN_wLEMP \
	VIN_CMD_SHORT_BUILD(VIN_WRITE, 0, (VIN_SH_CMD_CODE_wLEMP >> 8) & 0x1, (VIN_SH_CMD_CODE_wLEMP >> 4) & 0xf, VIN_SH_CMD_CODE_wLEMP & 0xf, 0)

#define VIN_wSTEDSP \
	VIN_CMD_SHORT_BUILD(VIN_WRITE, 0, (VIN_SH_CMD_CODE_wSTEDSP >> 8) & 0x1, (VIN_SH_CMD_CODE_wSTEDSP >> 4) & 0xf, VIN_SH_CMD_CODE_wSTEDSP & 0xf, 0)

#define VIN_wLPMP \
	VIN_CMD_SHORT_BUILD(VIN_WRITE, 0, (VIN_SH_CMD_CODE_wLPMP >> 8) & 0x1, (VIN_SH_CMD_CODE_wLPMP >> 4) & 0xf, VIN_SH_CMD_CODE_wLPMP & 0xf, 0)

#define VIN_wSWRST(_all) \
	VIN_CMD_SHORT_BUILD(VIN_WRITE, _all, (VIN_SH_CMD_CODE_wSWRST >> 8) & 0x1, (VIN_SH_CMD_CODE_wSWRST >> 4) & 0xf, VIN_SH_CMD_CODE_wSWRST & 0xf, 0)

#define VIN_wRESYNC \
	VIN_CMD_SHORT_BUILD(VIN_WRITE, 0, (VIN_SH_CMD_CODE_wRESYNC >> 8) & 0x1, (VIN_SH_CMD_CODE_wRESYNC >> 4) & 0xf, VIN_SH_CMD_CODE_wRESYNC & 0xf, 0)

#define VIN_wPHIERR \
	VIN_CMD_SHORT_BUILD(VIN_WRITE, 0, (VIN_SH_CMD_CODE_wPHIERR >> 8) & 0x1, (VIN_SH_CMD_CODE_wPHIERR >> 4) & 0xf, VIN_SH_CMD_CODE_wPHIERR & 0xf, 0)


#define VIN_POWER_DOWN_HIGH_IMPEDANCE(_chan) \
	VIN_CMD_SHORT_BUILD(VIN_WRITE, 0, (VIN_SH_CMD_CODE_POWER_DOWN_HIGH_IMPEDANCE >> 8) & 0x1, (VIN_SH_CMD_CODE_POWER_DOWN_HIGH_IMPEDANCE >> 4) & 0xf, VIN_SH_CMD_CODE_POWER_DOWN_HIGH_IMPEDANCE & 0xf, _chan)

#define VIN_ACTIVE_HIGH_VBATH(_chan) \
	VIN_CMD_SHORT_BUILD(VIN_WRITE, 0, (VIN_SH_CMD_CODE_ACTIVE_HIGH_VBATH >> 8) & 0x1, (VIN_SH_CMD_CODE_ACTIVE_HIGH_VBATH >> 4) & 0xf, VIN_SH_CMD_CODE_ACTIVE_HIGH_VBATH & 0xf, _chan)

/*!
 * \brief VINETIC Command
 */
union vin_cmd_first {
	struct vin_cmd_first_bits {
		u_int16_t chan:4;	// don't care
		u_int16_t res:4;	// 0
		u_int16_t cmd:5;
		u_int16_t bc:1;	
		u_int16_t sc:1;	
		u_int16_t rw:1;		// rd = 1, wr = 0
	} __attribute__((packed)) bits;
	u_int16_t full;
} __attribute__((packed));

union vin_cmd_sop {
	struct vin_cmd_sop_bits {
		u_int16_t length:8;
		u_int16_t offset:8;
	} __attribute__((packed)) bits;
	u_int16_t full;
} __attribute__((packed));

union vin_cmd_cop {
	struct vin_cmd_cop_bits {
		u_int16_t length:8;
		u_int16_t offset:8;
	} __attribute__((packed)) bits;
	u_int16_t full;
} __attribute__((packed));

union vin_cmd_iop {
	struct vin_cmd_iop_bits {
		u_int16_t length:8;
		u_int16_t offset:8;
	} __attribute__((packed)) bits;
	u_int16_t full;
} __attribute__((packed));

union vin_cmd_vop {
	struct vin_cmd_vop_bits {
		u_int16_t length:8;
		u_int16_t res0:5;
		u_int16_t odd:1;
		u_int16_t res1:2;
	} __attribute__((packed)) bits;
	u_int16_t full;
} __attribute__((packed));

union vin_cmd_eop {
	struct vin_cmd_eop_bits {
		u_int16_t length:8;
		u_int16_t ecmd:5;
		u_int16_t mod:3;
	} __attribute__((packed)) bits;
	u_int16_t full;
} __attribute__((packed));

union vin_cmd {
	struct vin_cmd_parts {
		union vin_cmd_first first;
		union {
			union vin_cmd_sop sop;
			union vin_cmd_cop cop;
			union vin_cmd_iop iop;
			union vin_cmd_vop vop;
			union vin_cmd_eop eop;
			u_int16_t full;
		} __attribute__((packed)) second;
	} __attribute__((packed)) parts;
	u_int32_t full;
} __attribute__((packed));

enum {
	VIN_SOP_DCCHKR = 0x00,
	VIN_SOP_DSCHKR = 0x01,
	VIN_SOP_CCR = 0x02,
	VIN_SOP_LMRES = 0x03,
	VIN_SOP_CCHKR = 0x04,
	VIN_SOP_IOCTL1 = 0x05,
	VIN_SOP_IOCTL2 = 0x06,
	VIN_SOP_BCR1 = 0x07,
	VIN_SOP_BCR2 = 0x08,
	VIN_SOP_DSCR = 0x09,
	VIN_SOP_LMCR = 0x0A,
	VIN_SOP_RTR = 0x0B,
	VIN_SOP_OFR = 0x0C,
	VIN_SOP_AUTOMOD = 0x0D,
	VIN_SOP_TSTR1 = 0x0E,
	VIN_SOP_TSTR2 = 0x0F,
	VIN_SOP_TSTR3 = 0x10,
};

struct vin_cmd_sop_generic {
	union vin_cmd header;
	u_int16_t word;
} __attribute__((packed));

struct vin_cmd_sop_dcchkr {
	union vin_cmd header;
	u_int16_t dc_check;
} __attribute__((packed));

struct vin_cmd_sop_dschkr {
	union vin_cmd header;
	u_int16_t ds_check;
} __attribute__((packed));

struct vin_cmd_sop_ccr {
	union vin_cmd header;
	struct vin_sop_ccr {
		u_int16_t pd_cbias:1;
		u_int16_t pd_cvcm:1;
		u_int16_t jump_ac1:1;
		u_int16_t jump_ac2:1;
		u_int16_t jump_ac3:1;
		u_int16_t jump_dc:1;
		u_int16_t res0:10;
	} __attribute__((packed)) sop_ccr;
} __attribute__((packed));

struct vin_cmd_sop_lmres {
	union vin_cmd header;
	u_int16_t lm_res;
} __attribute__((packed));

struct vin_cmd_sop_cchkr {
	union vin_cmd header;
	u_int16_t c_check;
} __attribute__((packed));

struct vin_cmd_sop_ioctl1 {
	union vin_cmd header;
	struct vin_sop_ioctl1 {
		u_int16_t oen_io0:1;
		u_int16_t oen_io1:1;
		u_int16_t oen_io2:1;
		u_int16_t oen_io3:1;
		u_int16_t oen_io4:1;
		u_int16_t res0:3;
		u_int16_t inen_io0:1;
		u_int16_t inen_io1:1;
		u_int16_t inen_io2:1;
		u_int16_t inen_io3:1;
		u_int16_t inen_io4:1;
		u_int16_t res1:3;
	} __attribute__((packed)) sop_ioctl1;
} __attribute__((packed));

struct vin_cmd_sop_ioctl2 {
	union vin_cmd header;
	struct vin_sop_ioctl2 {
		u_int16_t dup_io:4;
		u_int16_t dup:4;
		u_int16_t dd_io0:1;
		u_int16_t dd_io1:1;
		u_int16_t dd_io2:1;
		u_int16_t dd_io3:1;
		u_int16_t dd_io4:1;
		u_int16_t dup_ovt:3;
	} __attribute__((packed)) sop_ioctl2;
} __attribute__((packed));

struct vin_cmd_sop_bcr1 {
	union vin_cmd header;
	struct vin_sop_bcr1 {
		u_int16_t sel_slic:4;
		u_int16_t test_en:1;
		u_int16_t en_ik:1;
		u_int16_t dc_hold:1;
		u_int16_t lmabs_en:1;
		u_int16_t him_an:1;
		u_int16_t him_res:1;
		u_int16_t pram_dcc:1;
		u_int16_t lmac64:1;
		u_int16_t dup_gndk:4;
	} __attribute__((packed)) sop_bcr1;
} __attribute__((packed));

struct vin_cmd_sop_bcr2 {
	union vin_cmd header;
	struct vin_sop_bcr2 {
		u_int16_t hpr_dis:1;
		u_int16_t hpx_dis:1;
		u_int16_t frr_dis:1;
		u_int16_t frx_dis:1;
		u_int16_t ar_dis:1;
		u_int16_t ax_dis:1;
		u_int16_t im_dis:1;
		u_int16_t th_dis:1;
		u_int16_t res0:1;
		u_int16_t ac_short_en:1;
		u_int16_t rev_pol:1;
		u_int16_t ttx_12k:1;
		u_int16_t ttx_en:1;
		u_int16_t soft_dis:1;
		u_int16_t cram_en:1;
		u_int16_t lprx_cr:1;
	} __attribute__((packed)) sop_bcr2;
} __attribute__((packed));

struct vin_cmd_sop_dscr {
	union vin_cmd header;
	struct vin_sop_dscr {
		u_int16_t tg1_en:1;
		u_int16_t tg2_en:1;
		u_int16_t ptg:1;
		u_int16_t cor8:1;
		u_int16_t dg_key:4;
		u_int16_t res0:8;
	} __attribute__((packed)) sop_dscr;
} __attribute__((packed));

struct vin_cmd_sop_lmcr {
	union vin_cmd header;
	struct vin_sop_lmcr {
		u_int16_t lm_sel:4;
		u_int16_t ramp_en:1;
		u_int16_t lm_rect:1;
		u_int16_t lm_filt:1;
		u_int16_t lm_filt_sel:1;
		u_int16_t dc_ad16:1;
		u_int16_t lm_once:1;
		u_int16_t lm2pcm:1;
		u_int16_t lm_en:1;
		u_int16_t lm_itime:4;
	} __attribute__((packed)) sop_lmcr;
} __attribute__((packed));

struct vin_cmd_sop_rtr {
	union vin_cmd header;
	struct vin_sop_rtr {
		u_int16_t rext_en:1;
		u_int16_t asynch_r:1;
		u_int16_t ring_offset:2;
		u_int16_t pcm2dc:1;
		u_int16_t rtr_fast:1;
		u_int16_t rtr_sel:1;
		u_int16_t rtr_exts:1;
		u_int16_t lcas_en:1;
		u_int16_t res0:7;
	} __attribute__((packed)) sop_rtr;
} __attribute__((packed));

struct vin_cmd_sop_ofr {
	union vin_cmd header;
	u_int16_t dc_offset;
} __attribute__((packed));

struct vin_cmd_sop_automod {
	union vin_cmd header;
	struct vin_sop_automod {
		u_int16_t pdot_dis:1;
		u_int16_t auto_bat:1;
		u_int16_t auto_offh:1;
		u_int16_t res0:13;
	} __attribute__((packed)) sop_automod;
} __attribute__((packed));

struct vin_cmd_sop_tstr1 {
	union vin_cmd header;
	struct vin_sop_tstr1 {
		u_int16_t pd_hvi:1;
		u_int16_t pd_iref:1;
		u_int16_t pd_acref:1;
		u_int16_t pd_ovtc:1;
		u_int16_t pd_ofhc:1;
		u_int16_t pd_gnkc:1;
		u_int16_t pd_dcref:1;
		u_int16_t pd_akdith:1;
		u_int16_t pd_dcbuf:1;
		u_int16_t pd_dc_da:1;
		u_int16_t pd_dc_ad:1;
		u_int16_t pd_dc_pr:1;
		u_int16_t pd_ac_da:1;
		u_int16_t pd_ac_ad:1;
		u_int16_t pd_ac_po:1;
		u_int16_t pd_ac_pr:1;
	} __attribute__((packed)) sop_tstr1;
} __attribute__((packed));

struct vin_cmd_sop_tstr2 {
	union vin_cmd header;
	struct vin_sop_tstr2 {
		u_int16_t cox_16:1;
		u_int16_t cor_64:1;
		u_int16_t opim_hw:1;
		u_int16_t opim_an:1;
		u_int16_t low_perf:2;
		u_int16_t ac_xgain:1;
		u_int16_t ac_rgain:1;
		u_int16_t ac_short:1;
		u_int16_t ac_dlb_8k:1;
		u_int16_t ac_dlb_32k:1;
		u_int16_t ac_dlb_128k:1;
		u_int16_t ac_dlb_16m:1;
		u_int16_t ac_alb_16m:1;
		u_int16_t ac_alb_pp:1;
		u_int16_t res0:1;
	} __attribute__((packed)) sop_tstr2;
} __attribute__((packed));

struct vin_cmd_sop_tstr3 {
	union vin_cmd header;
	struct vin_sop_tstr3 {
		u_int16_t no_auto_dith_ac:1;
		u_int16_t no_dith_ac:1;
		u_int16_t atst_res0:1;
		u_int16_t atst_res1:1;
		u_int16_t atst_res2:1;
		u_int16_t atst_res3:1;
		u_int16_t atst_res4:1;
		u_int16_t atst_res5:1;
		u_int16_t dc_lp1_dis:1;
		u_int16_t dc_lp2_dis:1;
		u_int16_t dcc_dis:1;
		u_int16_t res0:1;
		u_int16_t dc_pofi_hi:1;
		u_int16_t dc_dlb_1m:1;
		u_int16_t dc_alb_1m:1;
		u_int16_t dc_alb_pp:1;
	} __attribute__((packed)) sop_tstr3;
} __attribute__((packed));

struct vin_cmd_cop_generic {
	union vin_cmd header;
	u_int16_t word[4];
} __attribute__((packed));

enum {
	VIN_MOD_PCM = 0x0,
	VIN_MOD_ALI = 0x1,
	VIN_MOD_CODER = 0x3,
	VIN_MOD_CONT = 0x5,
	VIN_MOD_TEST = 0x7,
};

enum {
	VIN_EOP_PCM_CONT = 0x00,
	VIN_EOP_PCM_CHAN = 0x01,
	VIN_EOP_NEAR_END_LEC = 0x02,
};
/*!
 * \brief Command_PCM_Interface_Control
	Description: This command activates the PCM-Interface-Module.
	This command is necessary before a PCM channel can be configured.
 */
struct vin_cmd_eop_pcm_interface_control {
	union vin_cmd header;
	struct vin_eop_pcm_interface_control {
		u_int16_t pcmro:3;
		u_int16_t shift:1;
		u_int16_t drive_0:1;
		u_int16_t r_slope:1;
		u_int16_t x_slope:1;
		u_int16_t db_clk:1;
		u_int16_t pcmxo:3;
		u_int16_t res:3;
		u_int16_t ds:1;
		u_int16_t en:1;
	} __attribute__((packed)) eop_pcm_interface_control;
} __attribute__((packed));
/*!
 * \brief PCM Receive Offset
	Receive bit-offset
 */
enum {
	VIN_PCMRO_0 = 0, /*! No offset */
	VIN_PCMRO_1 = 1, /*! One data period is added */
	VIN_PCMRO_2 = 2, /*! Two data periods are added */
	VIN_PCMRO_3 = 3, /*! Three data periods are added */
	VIN_PCMRO_4 = 4, /*! Four data periods are added */
	VIN_PCMRO_5 = 5, /*! Five data periods are added */
	VIN_PCMRO_6 = 6, /*! Six data periods are added */
	VIN_PCMRO_7 = 7, /*! Seven data periods are added */
};
/*!
 * \brief Driving Mode for Bit 0
	only available in single-clocking mode
 */
enum {
	VIN_DRIVE_0_ENTIRE = 0, /*! Bit 0 is driven the entire clock period */
	VIN_DRIVE_0_HALF = 1, /*! Bit 0 is driven during the first half of the clock period */
};
/*!
 * \brief Receive slope
 */
enum {
	VIN_R_SLOPE_FALL = 0, /*! Receive slope is falling edge */
	VIN_R_SLOPE_RISE = 1, /*! Receive slope is rising edge */
};
/*!
 * \brief Transmit slope
 */
enum {
	VIN_X_SLOPE_RISE = 0, /*! Transmit slope is falling edge */
	VIN_X_SLOPE_FALL = 1, /*! Transmit slope is rising edge */
};
/*!
 * \brief PCM Transmit Offset
	Transmit bit-offset
 */
enum {
	VIN_PCMXO_0 = 0, /*! No offset */
	VIN_PCMXO_1 = 1, /*! One data period is added */
	VIN_PCMXO_2 = 2, /*! Two data periods are added */
	VIN_PCMXO_3 = 3, /*! Three data periods are added */
	VIN_PCMXO_4 = 4, /*! Four data periods are added */
	VIN_PCMXO_5 = 5, /*! Five data periods are added */
	VIN_PCMXO_6 = 6, /*! Six data periods are added */
	VIN_PCMXO_7 = 7, /*! Seven data periods are added */
};
/*!
 * \brief Command_PCM_Interface_Channel
	Description: This command configures the PCM channels.
 */
struct vin_cmd_eop_pcm_interface_channel {
	union vin_cmd header;
	struct vin_eop_pcm_interface_channel {
		u_int16_t i1:6;
		u_int16_t bp:1;
		u_int16_t hp:1;
		u_int16_t codnr:4;
		u_int16_t cod:3;
		u_int16_t en:1;
		u_int16_t rts:7;
		u_int16_t r_hw:1;
		u_int16_t xts:7;
		u_int16_t x_hw:1;
		u_int16_t gain2:8;
		u_int16_t gain1:8;
		u_int16_t i3:6;
		u_int16_t res0:2;
		u_int16_t i2:6;
		u_int16_t res1:2;
		u_int16_t i5:6;
		u_int16_t res2:2;
		u_int16_t i4:6;
		u_int16_t res3:2;
	} __attribute__((packed)) eop_pcm_interface_channel;
} __attribute__((packed));
/*!
 * \brief Bit Packing
	Defines the location of the data bits in case of ADPCM coding.
 */
enum {
	VIN_BP_LSB = 0, /*! ADPCM bits alignment to LSB's */
	VIN_BP_MSB = 1, /*! ADPCM bits alignment to MSB's */
};
/*!
 * \brief Coder
	Selection of the coder algorithm:
	Note: for linear mode CHAN+1 must be inactive and can not be used
 */
enum {
	VIN_COD_LIN = 0, /*! Linear mode  */
	VIN_COD_G711_ALAW = 2, /*! G.711, A-Law */
	VIN_COD_G711_MLAW = 3, /*! G.711, μ-Law */
	VIN_COD_G726_16 = 4, /*! G.726, 16 kbit/s */
	VIN_COD_G726_24 = 5, /*! G.726, 24 kbit/s */
	VIN_COD_G726_32 = 6, /*! G.726, 32 kbit/s */
	VIN_COD_G726_40 = 7, /*! G.726, 40 kbit/s */
};
/*!
 * \brief Receive Highway
	Selection of the PCM highway for receiving PCM data
 */
enum {
	VIN_R_HW_A = 0, /*! PCM highway A is selected */
	VIN_R_HW_B = 1, /*! PCM highway B is selected */
};

/*!
 * \brief Transmit Highway
	Selection of the PCM highway for transmitting PCM data
 */
enum {
	VIN_X_HW_A = 0, /*! PCM highway A is selected */
	VIN_X_HW_B = 1, /*! PCM highway B is selected */
};
/*!
 * \brief Command_PCM_Near_End_LEC
	Description: This command activates one of the near end LEC's in the addressed PCM channel.
 */
struct vin_cmd_eop_pcm_near_end_lec {
	union vin_cmd header;
	struct vin_eop_pcm_near_end_lec {
		u_int16_t lecnr:4;
		u_int16_t nlpm:2;
		u_int16_t nlp:1;
		u_int16_t as:1;
		u_int16_t oldc:1;
		u_int16_t dtm:1;
		u_int16_t res:5;
		u_int16_t en:1;
	} __attribute__((packed)) eop_pcm_near_end_lec;
} __attribute__((packed));
/*!
 * \brief NLP Mode
 */
enum {
	VIN_NLPM_LIMIT = 0, /*! NLP limits the signal when it is active */
	VIN_NLPM_RES = 1, /*! Reserved */
	VIN_NLPM_SIGN_NOISE = 2, /*! NLP sends sign noise when it is active */
	VIN_NLPM_WHITE_NOISE = 3, /*! NLP sends white noise when it is active */
};
/*!
 * \brief Adaptation Stop Bit
*/
enum {
	VIN_AS_RUN = 0, /*! Adaptation running */
	VIN_AS_STOP = 1, /*! Adaptation stop */
};
/*!
 * \brief Old Coefficients
*/
enum {
	VIN_OLDC_ZERO = 0, /*! All LEC filter coefficients are initialized with zero */
	VIN_OLDC_NO = 1, /*! Do not initialize the filter coefficient of the LEC */
};
/*!
 * \brief Turbo Mode
*/
enum {
	VIN_DTM_ON = 0, /*! Turbo mode is on */
	VIN_DTM_OFF = 1, /*! Turbo mode is off */
};
/*!
 * \brief Analog Line Interface Module (ALI)
 */
enum {
	VIN_EOP_ALI_CONT = 0x00,
	VIN_EOP_ALI_CHAN = 0x01,
	VIN_EOP_ALI_NEAR_END_LEC = 0x02,
};
/*!
 * \brief Command_Analog_Line_Interface_Control
	Description: This command activates or deactivates the SW Analog line interface handling.
 */
struct vin_cmd_eop_ali_control {
	union vin_cmd header;
	struct vin_eop_ali_control {
		u_int16_t res:15;
		u_int16_t en:1;
	} __attribute__((packed)) eop_ali_control;
} __attribute__((packed));
/*!
 * \brief Command_Analog_Line_Interface_Channel
	Description: This command configures the Analog Line Interface channels.
 */
struct vin_cmd_eop_ali_channel {
	union vin_cmd header;
	struct vin_eop_ali_channel {
		u_int16_t i1:6;
		u_int16_t res0:9;
		u_int16_t en:1;
		u_int16_t gainr:8;
		u_int16_t gainx:8;
		u_int16_t i3:6;
		u_int16_t res1:2;
		u_int16_t i2:6;
		u_int16_t res2:2;
		u_int16_t i5:6;
		u_int16_t res3:2;
		u_int16_t i4:6;
		u_int16_t res4:2;
	} __attribute__((packed)) eop_ali_channel;
} __attribute__((packed));
/*!
 * \brief Command_ALI_Near_End_LEC
	Description: This command activates one of the near end LEC's in the addressed channel.
 */
struct vin_cmd_eop_ali_near_end_lec {
	union vin_cmd header;
	struct vin_eop_ali_near_end_lec {
		u_int16_t lecnr:4;
		u_int16_t nlpm:2;
		u_int16_t nlp:1;
		u_int16_t as:1;
		u_int16_t oldc:1;
		u_int16_t dtm:1;
		u_int16_t res:5;
		u_int16_t en:1;
	} __attribute__((packed)) eop_ali_near_end_lec;
} __attribute__((packed));
/*!
 * \brief Coder-Module
 */
enum {
	VIN_EOP_CODER_CONT = 0x00,
	VIN_EOP_CODER_CHAN_SC = 0x01,
	VIN_EOP_CODER_CONF = 0x10,
	VIN_EOP_CODER_CONF_RTP = 0x11,
	VIN_EOP_CODER_JBSTAT = 0x14,
};
/*!
 * \brief Command_Coder_Control
	Description: This command activates or deactivates the Coder-Module.
 */
struct vin_cmd_eop_coder_control {
	union vin_cmd header;
	struct vin_eop_coder_control {
		u_int16_t res:15;
		u_int16_t en:1;
	} __attribute__((packed)) eop_coder_control;
} __attribute__((packed));
/*!
 * \brief Command_Coder_Channel_Speech_Compression
	Description: This command activates one coder channel.
 */
struct vin_cmd_eop_coder_channel_speech_compression {
	union vin_cmd header;
	struct vin_eop_coder_channel_speech_compression {
		u_int16_t i1:6;
		u_int16_t res0:1;
		u_int16_t ns:1;
		u_int16_t codnr:4;
		u_int16_t res1:3;
		u_int16_t en:1;
		u_int16_t enc:5;
		u_int16_t pte:3;
		u_int16_t sic:1;
		u_int16_t pst:1;
		u_int16_t im:1;
		u_int16_t dec:1;
		u_int16_t bfi:1;
		u_int16_t cng:1;
		u_int16_t pf:1;
		u_int16_t hp:1;
		u_int16_t gain2:8;
		u_int16_t gain1:8;
		u_int16_t i3:6;
		u_int16_t res2:2;
		u_int16_t i2:6;
		u_int16_t res3:2;
		u_int16_t i5:6;
		u_int16_t res4:2;
		u_int16_t i4:6;
		u_int16_t res5:2;
	} __attribute__((packed)) eop_coder_channel_speech_compression;
} __attribute__((packed));
enum {
	VIN_NS_INACTIVE = 0,
	VIN_NS_ACTIVE = 1,
};
/*!
 * \brief VINETIC Encoder Packet Time
 */
enum {
	VIN_PTE_5 = 0, /*! 5 ms, not possible for G.729, G.723.1 and iLBC */
	VIN_PTE_10 = 1, /*! 10 ms, not possible for G.723.1 and iLBC */
	VIN_PTE_20 = 2, /*! 20 ms, not possible for G.723.1 and iBLC 13.3 kB/s */
	VIN_PTE_30 = 3, /*! 30 ms, not possible for iLBC 15.2 kB/s */
	VIN_PTE_5_5 = 4, /*! 5.5 ms, only allowed for G.711 G.726 32 kbit/s and G.726 16 kbit/s */
	VIN_PTE_11 = 5, /*! 11 ms, only allowed for G.711 G.726 32 kbit/s and G.726 16 kbit/s */
};
/*!
 * \brief VINETIC Encoder Algorithm
 */
enum {
	VIN_ENC_NO = 0x00, /*! No encoder data are sent to the host,
							no encoder is active and the host does not get an
							interrupt from the encoder side. This configuration is
							useful for example when the host will use the decoder
							path only. */
	VIN_ENC_G711_ALAW = 0x02, /*! G.711, 64 kbit/s, A-Law */
	VIN_ENC_G711_MLAW = 0x03, /*! G.711, 64 kbit/s, μ-Law */
	VIN_ENC_G726_16 = 0x04, /*! G.726, 16 kbit/s */
	VIN_ENC_G726_24 = 0x05, /*! G.726, 24 kbit/s */
	VIN_ENC_G726_32 = 0x06, /*! G.726, 32 kbit/s */
	VIN_ENC_G726_40 = 0x07, /*! G.726, 40 kbit/s */
	VIN_ENC_G728_16 = 0x10, /*! G.728, 16 kbit/s */
	VIN_ENC_G729AB_8 = 0x12, /*! G.729A,B, 8 kbit/s */
	VIN_ENC_G729E_11_8 = 0x13, /*! G.729E, 11.8 kbit/s */
	VIN_ENC_ILBC_15_2 = 0x1a, /*! iLBC, 15.2 kB/s */
	VIN_ENC_ILBS_13_3 = 0x01b, /*! iLBC, 13.3 kB/s */
	VIN_ENC_G7231_5_3 = 0x1c, /*! G.723.1, 5.3 kbit/s */
	VIN_ENC_G7231_6_3 = 0x1d, /*! G.723.1, 6.3 kbit/s */
};
/*!
 * \brief Command_Coder_Configuration_RTP_Support
	Description: This command determines the start value for the timestamp.
	It can also be used to overwrite or to read the actual
	timestamp. The timestamp is used for all active channels.
 */
struct vin_cmd_eop_coder_configuration_rtp_support {
	union vin_cmd header;
	struct vin_eop_coder_configuration_rtp_support {
		u_int16_t time_stamp_hw;
		u_int16_t time_stamp_lw;
	} __attribute__((packed)) eop_coder_configuration_rtp_support;
} __attribute__((packed));
/*!
 * \brief Command_Coder_Channel_Configuration_RTP_Support
	Description: This command configures one coder channel.
 */
struct vin_cmd_eop_coder_channel_configuration_rtp_support {
	union vin_cmd header;
	struct vin_eop_coder_channel_configuration_rtp_support {
		u_int16_t ssrc_hw; // 1
		u_int16_t ssrc_lw; // 2
		u_int16_t seq_nr; // 3
		u_int16_t pt_00011:7; // 4
		u_int16_t sid_00011:1;
		u_int16_t pt_00010:7;
		u_int16_t sid_00010:1;
		u_int16_t pt_00101:7; // 5
		u_int16_t sid_00101:1;
		u_int16_t pt_00100:7;
		u_int16_t sid_00100:1;
		u_int16_t pt_00111:7; // 6
		u_int16_t sid_00111:1;
		u_int16_t pt_00110:7;
		u_int16_t sid_00110:1;
		u_int16_t pt_01001:7; // 7
		u_int16_t sid_01001:1;
		u_int16_t pt_01000:7;
		u_int16_t sid_01000:1;
		u_int16_t pt_01011:7; // 8
		u_int16_t sid_01011:1;
		u_int16_t pt_01010:7;
		u_int16_t sid_01010:1;
		u_int16_t pt_01101:7; // 9
		u_int16_t sid_01101:1;
		u_int16_t pt_01100:7;
		u_int16_t sid_01100:1;
		u_int16_t pt_01111:7; // 10
		u_int16_t sid_01111:1;
		u_int16_t pt_01110:7;
		u_int16_t sid_01110:1;
		u_int16_t pt_10001:7; // 11
		u_int16_t sid_10001:1;
		u_int16_t pt_10000:7;
		u_int16_t sid_10000:1;
		u_int16_t pt_10011:7; // 12
		u_int16_t sid_10011:1;
		u_int16_t pt_10010:7;
		u_int16_t sid_10010:1;
		u_int16_t pt_10101:7; // 13
		u_int16_t sid_10101:1;
		u_int16_t pt_10100:7;
		u_int16_t sid_10100:1;
		u_int16_t pt_10111:7; // 14
		u_int16_t sid_10111:1;
		u_int16_t pt_10110:7;
		u_int16_t sid_10110:1;
		u_int16_t pt_11001:7; // 15
		u_int16_t sid_11001:1;
		u_int16_t pt_11000:7;
		u_int16_t sid_11000:1;
		u_int16_t pt_11011:7; // 16
		u_int16_t sid_11011:1;
		u_int16_t pt_11010:7;
		u_int16_t sid_11010:1;
		u_int16_t pt_11101:7; // 17
		u_int16_t sid_11101:1;
		u_int16_t pt_11100:7;
		u_int16_t sid_11100:1;
		u_int16_t pt_11111:7; // 18
		u_int16_t sid_11111:1;
		u_int16_t pt_11110:7;
		u_int16_t sid_11110:1;
	} __attribute__((packed)) eop_coder_channel_configuration_rtp_support;
} __attribute__((packed));
/*!
 * \brief Command_Coder_Channel_JB_Statistics
	Description: This command delivers statistic information for the Voice Play Out Unit.
 */
struct vin_cmd_eop_coder_channel_jb_statistics {
	union vin_cmd header;
	struct vin_eop_coder_channel_jb_statistics {
		u_int16_t packet_pod; /*! Packet play out delay for the last received packet in timestamp units (125 μs). */
		u_int16_t max_packet_pod; /*! Maximum packet play out delay in timestamp units (125 μs)
									since the beginning of the connection or since the last statistic reset.  */
		u_int16_t min_packet_pod; /*! Minimum packet play out delay in timestamp units since the
									beginning of the connection or since the last statistic reset. */
		u_int16_t jitter; /*! Actual estimated jitter buffer size in timestamp units (125 μs)
							which is necessary to compensate the network jitter. */
		u_int16_t max_jitter;  /*! Maximum estimated jitter buffer size since the beginning of
								the connection or since the last statistic reset. */
		u_int16_t min_jitter; /*! Minimum estimated jitter buffer size since the beginning of
								the connection or since the last statistic reset. */
		u_int16_t packets_hw; /*! Total number(hw) of received packets since the beginning of the
								connection or since the last statistic reset.
								Note: All received packets are counted (valid, invalid, early, late, duplicate packets). */
		u_int16_t packets_lw; /*! Total number(lw) of received packets since the beginning of the
								connection or since the last statistic reset.
								Note: All received packets are counted (valid, invalid, early, late, duplicate packets). */
		u_int16_t discarded_packets; /*! Total number of discarded packets (invalid, early, late, duplicate packets)
										since the beginning of the connection or since the last statistic reset. */
		u_int16_t late_packets; /*! Total number of late packets since the beginning of the connection or since
									the last statistic reset. Late packets are packets which have been received after
									the estimated play out time for that packet was over. Late packets will be discarded. */
		u_int16_t early_packets; /*! Total number of early packets since the beginning of the connection or since
									the last statistic reset. An early packet is a packet which can not be stored
									in the jitter buffer because the packet requires a too high play out delay. */
		u_int16_t resnc; /*! Number of resynchronizations since the beginning of the connection or since the last statistic reset. */
		u_int16_t is_underflow_hw; /*! Total number of injected samples (each samples represents 125 μs) since the beginning
									of the connection or since the last statistic reset due to jitter buffer underflows.
									Jitter buffer underflow means that the jitter buffer is empty and therefore the missing packets
									are replaced by error concealment or packet repetition. */
		u_int16_t is_underflow_lw;
		u_int16_t is_no_underflow_hw; /*! Total number of injected samples since the beginning of the connection or since the last
									statistic reset in case of normal jitter buffer operation, which means when there is no jitter
									buffer underflow . This counter is increased in case of lost packets, discarded late packets
									or discarded packets due to jitter buffer overflows if the missed packet is replaced by error
									concealment or packet repetition. */
		u_int16_t is_no_underflow_lw;
		u_int16_t is_increment_hw; /*! Total number of injected samples since the beginning of the connection or since the last
									statistic reset in case of jitter buffer increments. This counter counts all additional samples
									which are injected to increase the jitter buffer size, which means to increase the packet play
									out delay. Thus the total number of injected samples during the connection is the sum of
									IS_UNDERFLOW, IS_NO_UNDERFLOW and IS_INCREMENT. */
		u_int16_t is_increment_lw;
		u_int16_t sk_decrement_hw; /*! Total number of skipped lost samples since the beginning of the connection or since the last
									statistic reset in case of jitter buffer decrements. Lost packets, discarded late packets or discarded
									packets due to jitter buffer overflows are normally replaced by artificial signals (error concealment,
									packet repetition). In case of jitter buffer adjustments it could be that the missed voice is skipped
									by the jitter buffer to decrease the jitter buffer size, which means to decrease the packet play out delay. */
		u_int16_t sk_decrement_lw;
		u_int16_t ds_decrement_hw; /*! Total number of dropped samples since the beginning of the connection or since the last statistic
									reset in case of jitter buffer decrements. This counter is increased when the jitter buffer has discarded
									a voice frame to reduce the jitter buffer size, which means to reduce the packet play out delay. */
		u_int16_t ds_decrement_lw;
		u_int16_t ds_overflow_hw; /*! Total number of dropped samples since the beginning of the connection or since the last statistic
									reset in case of jitter buffer overflows. Jitter buffer overflow means that the jitter buffer is full
									and therefore the received packet can not be stored within the jitter buffer.
									Notes 1. Discarded SID packets are not counted because SID packets have no defined frame size.
											Discarded SID packets will be counted by one of the other statistic counters, IS_NO_UNDERFLOW,
											SK_DECREMENT, IS_UNDERFLOW or SID (if within a SID sequence one SID packet has been dropped). */
		u_int16_t ds_overflow_lw;
		u_int16_t sid_hw; /*! Total number of comfort noise samples since the beginning of the connection or since the last statistic reset.
							This counter contains the total sum off all silence periods which are generated by the decoder. */
		u_int16_t sid_lw;
	} __attribute__((packed)) eop_coder_channel_jb_statistics;
} __attribute__((packed));
/*!
 * \brief Control Commands
 */
enum {
	VIN_EOP_CERR_ACK = 0x00,
	VIN_EOP_ENDIAN_CONT = 0x04,
};
/*!
 * \brief Command_CERR_Acknowledge
	This command clears the CERR bit in the BXSR1 register.
 */
struct vin_cmd_eop_cerr_acknowledge {
	union vin_cmd header;
} __attribute__((packed));

/*!
 * \brief Command_Endian_Control
	Description: This command controls the endian interpretation of the data words for the EOP/EVT/VOP commands.
 */
struct vin_cmd_eop_endian_control {
	union vin_cmd header;
	struct vin_eop_endian_control {
		u_int16_t le:1;
		u_int16_t res:15;
	} __attribute__((packed)) eop_endian_control;
} __attribute__((packed));
enum {
	VIN_BIG_ENDIAN = 0,
	VIN_LITTLE_ENDIAN = 1,
};
/*!
 * \brief Test and Download
 */
enum {
	VIN_EOP_EDSPSWVERSREG = 0x06,
	VIN_EOP_SET_FPI = 0x14,
	VIN_EOP_ACCESS_FPI = 0x15,
	VIN_EOP_CRC_FPI = 0x16,
	VIN_EOP_SETPRAM = 0x18,
	VIN_EOP_ACCESSPRAM = 0x19,
	VIN_EOP_CRC_PRAM = 0x1a,
	VIN_EOP_SET_DRAM = 0x1b,
	VIN_EOP_ACCESS_DRAM = 0x1c,
	VIN_EOP_CRC_DRAM = 0x1d,
	VIN_EOP_DOWNLOAD_END = 0x1f,
};

struct vin_cmd_eop_edsp_sw_version_register {
	union vin_cmd header;
	struct vin_eop_edsp_sw_version_register {
		u_int16_t main_version:6;
		u_int16_t features:6;
		u_int16_t prt:1;
		u_int16_t mv:2;
		u_int16_t cv:1;
		u_int16_t release;
	} __attribute__((packed)) eop_edsp_sw_version_register;
} __attribute__((packed));

/*!
 * \brief VINETIC Supported Features
 */
/*! List bellow is valid for VINETIC®-4VIP, V1.4, RTP Versions (PRT=0) */
enum {
	VIN_4VIP1_RTP4 = 0x00, /*! firmware (type 0) 4VIP1-RTP4, G.729A/B/E, G.723.1, G.711, G.726 included. */
	VIN_4VIP2_RTP4 = 0x04, /*! firmware (type 4) 4VIP2-RTP4, G.729A/B, G.728, G.723.1, G.711, G.726 included. */
	VIN_4VIP4_RTP4 = 0x0c, /*! firmware (type 12) 4VIP4-RTP4, G.728, G.729A/B/E, G.711, G.726 included. */
	VIN_4VIP6_RTP4 = 0x10, /*! firmware (type 16) 4VIP6-RTP4, Fax data pump (T.38) + G.729A/B/E, G.711, G.726 included. */
	VIN_4VIP7_RTP4 = 0x14, /*! firmware (type 20) 4VIP7-RTP4, Fax data pump, G.728, G.729A/B included. */
	VIN_4VIP8_RTP4 = 0x18, /*! firmware (type 24) 4VIP8-RTP4, Fax data pump, G.728, G.723.1 included. */
};
/*! List bellow is valid for VINETIC®-4VIP, V1.4, AAL Versions (PRT=1) */
enum {
	VIN_4VIP1_AAL4 = 0x00, /*! firmware (type 64) 4VIP1-AAL4, G.729A/B/E, G.723.1, G.711, G.726 included. */
	VIN_4VIP2_AAL4 = 0x04, /*! firmware (type 68) 4VIP2-AAL4, G.729A/B, G.728, G.723.1, G.711, G.726 included. */
	VIN_4VIP4_AAL4 = 0x0c, /*! firmware (type 76) 4VIP4-AAL4, G.728, G.729A/B/E, G.711, G.726 included. */
};
/*! List bellow is valid for VINETIC®-4C, V2.2, (PRT=0) */
enum {
	VIN_4C_TDM = 0x20, /*! firmware (type 32) 4C-TDM, without coder module. */
};
/*! List bellow is valid for VINETIC®-4M (MV=10B), V2.2, RTP Version (PRT=0) */
enum {
	VIN_4M5_RTP4 = 0x00, /*! firmware (type 256) 4M-RTP4, G.711, G.726 included. */
};
/*! List bellow is valid for VINETIC®-4M (MV=10B), V2.2, AAL Version (PRT=1) */
enum {
	VIN_4M5_AAL4 = 0x00, /*! firmware (type 320) 4M-AAL4, G.711, G.726 included. */
};

enum {
	VIN_PRT_RTP = 0,
	VIN_PRT_AAL = 1,
};

enum {
	VIN_MV_VIP = 0,
	VIN_MV_4M = 2,
};

struct vin_cmd_eop_set_fpi_address {
	union vin_cmd header;
	u_int16_t high_addres1;
	u_int16_t low_addres1;
	u_int16_t high_addres2;
	u_int16_t low_addres2;
} __attribute__((packed));

struct vin_cmd_eop_access_fpi_memory {
	union vin_cmd header;
	u_int16_t data[32];
} __attribute__((packed));

struct vin_cmd_eop_crc_fpi {
	union vin_cmd header;
	u_int16_t crc;
} __attribute__((packed));

struct vin_cmd_eop_set_pram_address {
	union vin_cmd header;
	u_int16_t high_addres1;
	u_int16_t low_addres1;
	u_int16_t high_addres2;
	u_int16_t low_addres2;
} __attribute__((packed));

struct vin_cmd_eop_access_pram {
	union vin_cmd header;
	u_int16_t data[252];
} __attribute__((packed));

struct vin_cmd_eop_crc_pram {
	union vin_cmd header;
	u_int16_t crc;
} __attribute__((packed));

struct vin_cmd_eop_set_dram_address {
	union vin_cmd header;
	u_int16_t addres1;
	u_int16_t addres2;
} __attribute__((packed));

struct vin_cmd_eop_access_dram {
	union vin_cmd header;
	u_int16_t data[252];
} __attribute__((packed));

struct vin_cmd_eop_crc_dram {
	union vin_cmd header;
	u_int16_t crc;
} __attribute__((packed));

#endif //__VINETIC_DEF_H__

/******************************************************************************/
/* end of vinetic-def.h                                                       */
/******************************************************************************/
