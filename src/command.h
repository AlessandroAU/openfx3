#define CMD_GET_FW_VERSION		0xb0
#define CMD_START			0xb1
#define CMD_GET_FW_MODE			0xb2
#define CMD_STOP			0xb3
#define CMD_GET_DROP_STATS		0xb4
#define CMD_RESET_DROP_STATS		0xb5
#define CMD_GET_ACQ_STATUS		0xb6
#define CMD_REBOOT_BOOTLOADER		0xb7
#define CMD_START_BENCHMARK		0xb8

struct version_info {
	uint8_t major;
	uint8_t minor;
};

struct cmd_start_acquisition {
	uint8_t bus_mhz;    /* GPIF bus clock in MHz (e.g., 20, 40, 65, 80, 100) */
	uint8_t bus_width;  /* Bus width in bits (8, 16, 24, or 32) */
	uint8_t reserved;   /* Reserved for future use, set to 0 */
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
	uint8_t reserved;      /* Reserved for future use */
	uint32_t bus_freq_hz;  /* Actual bus clock frequency in Hz */
	uint32_t sys_clk_hz;   /* System clock frequency in Hz */
	uint16_t gpif_div;     /* GPIF divider register value */
	uint16_t reserved2;    /* Padding for alignment */
};
