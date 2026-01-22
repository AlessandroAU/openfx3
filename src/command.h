#ifndef COMMAND_H
#define COMMAND_H

#include <stdint.h>

#define CMD_GET_FW_VERSION		0xb0
#define CMD_START			0xb1
#define CMD_GET_FW_MODE			0xb2
#define CMD_STOP			0xb3
#define CMD_GET_DROP_STATS		0xb4
#define CMD_RESET_DROP_STATS		0xb5
#define CMD_GET_ACQ_STATUS		0xb6
#define CMD_REBOOT_BOOTLOADER		0xb7
#define CMD_START_BENCHMARK		0xb8
#define CMD_I2C_WRITE			0xb9
#define CMD_I2C_READ			0xba
#define CMD_I2C_WRITE_READ		0xbb

struct version_info {
	uint8_t major;
	uint8_t minor;
};

/* Acquisition configuration flags (directly maps to GPIF_CONFIG register bits) */
struct acq_config {
	uint8_t bus_width;     /* Bus width: 8, 16, 24, or 32 bits */
	uint8_t internal_clk:1;/* Use internal clock (1) or external clock (0) */
	uint8_t clk_invert:1;  /* Sample on falling edge instead of rising */
	uint8_t clk_out:1;     /* Output clock on CLK pin (internal clock only) */
	uint8_t ddr:1;         /* DDR mode: sample on both clock edges */
	uint8_t endian:1;      /* Swap byte order (big-endian) */
	uint8_t reserved:3;    /* Reserved for future use */
};

struct cmd_start_acquisition {
	uint8_t bus_mhz;          /* GPIF bus clock in MHz (0=external clock) */
	struct acq_config config; /* Acquisition configuration */
};

struct cmd_drop_stats {
	uint64_t total_buffers;  /* Buffers successfully committed */
	uint64_t total_bytes;    /* Bytes successfully committed */
	uint32_t stalls;         /* Backpressure events (DMA STALL, not data loss) */
	uint32_t gpif_errors;    /* GPIF/PIB errors (from PIB ISR) */
};

struct cmd_acq_status {
	uint8_t active;        /* 1 if acquisition is running, 0 otherwise */
	uint8_t bus_width;     /* Configured bus width in bits */
	uint8_t pll_fbdiv;     /* PLL feedback divider used (20-27) */
	uint8_t dma_buffer_count; /* Number of DMA buffers in pool */
	uint32_t bus_freq_hz;  /* Actual bus clock frequency in Hz */
	uint32_t sys_clk_hz;   /* System clock frequency in Hz */
	uint16_t gpif_div;     /* GPIF divider register value */
	uint16_t dma_buffer_size; /* Size of each DMA buffer in bytes (max 64KB) */
};

#endif /* COMMAND_H */
