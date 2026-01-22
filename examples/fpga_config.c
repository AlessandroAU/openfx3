/*
 * FPGA Configuration Example - Configure FX3 Validation FPGA via I2C
 *
 * Communicates with the FPGA I2C slave to configure:
 *   - Clock mode: slave (external clock) or master (FPGA generates clock)
 *   - Counter width: 8, 16, 24, or 32 bits
 *   - Clock divider for master mode (1MHz - 100MHz)
 *
 * FPGA I2C Address: 0x40
 * Register Map:
 *   0x00: Control Register
 *         Bit 0: Clock Mode (0=slave/external, 1=master/generate)
 *         Bit 2:1: Counter Width (00=8bit, 01=16bit, 10=24bit, 11=32bit)
 *   0x01: Frequency Low Byte (master mode) - target frequency in MHz
 *   0x02: Frequency High Byte (master mode)
 *         DDS-based clock generation, supports 1-100 MHz in 1 MHz steps
 *   0x03: Status Register (read-only)
 *         Bit 0: Current clock mode
 *         Bit 1: PLL locked
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <openfx3/device.h>
#include <openfx3/platform.h>

/* FPGA I2C address (7-bit) - FPGA uses 0x40 in 8-bit format = 0x20 in 7-bit */
#define FPGA_I2C_ADDR       0x40

/* FPGA register addresses */
#define REG_CONTROL         0x00
#define REG_DIV_LO          0x01
#define REG_DIV_HI          0x02
#define REG_STATUS          0x03

/* Control register bits */
#define CTRL_CLOCK_MODE     (1 << 0)    /* 0=slave, 1=master */
#define CTRL_WIDTH_MASK     (3 << 1)    /* Bits 2:1 */
#define CTRL_WIDTH_8BIT     (0 << 1)
#define CTRL_WIDTH_16BIT    (1 << 1)
#define CTRL_WIDTH_24BIT    (2 << 1)
#define CTRL_WIDTH_32BIT    (3 << 1)

/* Status register bits */
#define STATUS_CLOCK_MODE   (1 << 0)
#define STATUS_PLL_LOCKED   (1 << 1)

static void print_usage(const char *prog) {
    printf("Usage: %s [options]\n", prog);
    printf("\nConfigures the FX3 Validation FPGA via I2C.\n");
    printf("\nOptions:\n");
    printf("  -h, --help          Show this help\n");
    printf("  -w, --width <N>     Set counter width (8, 16, 24, or 32 bits)\n");
    printf("  -f, --freq <MHz>    Set clock frequency:\n");
    printf("                        0 = slave mode (external clock input)\n");
    printf("                        1-100 = master mode at specified MHz\n");
    printf("  --firmware <path>   Firmware file for FX3 USB (default: embedded)\n");
    printf("\nExamples:\n");
    printf("  %s                       Read current configuration\n", prog);
    printf("  %s -f 10 -w 32           Master mode, 10MHz, 32-bit counter\n", prog);
    printf("  %s -f 0 -w 16            Slave mode (external clock), 16-bit counter\n", prog);
}

/* Write a single register */
static int fpga_write_reg(libusb_device_handle *handle, uint8_t reg, uint8_t value) {
    uint8_t buf[2] = { reg, value };
    return fx3_i2c_write(handle, FPGA_I2C_ADDR, buf, 2);
}

/* Read a single register using separate write + read transactions */
static int fpga_read_reg(libusb_device_handle *handle, uint8_t reg, uint8_t *value) {
    /* First, write the register address */
    int ret = fx3_i2c_write(handle, FPGA_I2C_ADDR, &reg, 1);
    if (ret != 0) {
        return ret;
    }
    /* Then read the value */
    return fx3_i2c_read(handle, FPGA_I2C_ADDR, value, 1);
}

/* Convert frequency to register value
 * FPGA uses DDS with 200MHz base clock - register value is direct MHz (1-100)
 */
static uint16_t freq_to_reg(float freq_mhz) {
    if (freq_mhz < 1) freq_mhz = 1;
    if (freq_mhz > 100) freq_mhz = 100;
    return (uint16_t)(freq_mhz + 0.5f);  /* Round to nearest integer */
}

/* Read and print current FPGA status (utility for debugging) */
static int __attribute__((unused)) print_status(libusb_device_handle *handle) {
    uint8_t ctrl, freq_lo, freq_hi, status;

    if (fpga_read_reg(handle, REG_CONTROL, &ctrl) != 0) {
        fprintf(stderr, "Failed to read control register\n");
        return -1;
    }
    if (fpga_read_reg(handle, REG_DIV_LO, &freq_lo) != 0) {
        fprintf(stderr, "Failed to read frequency low register\n");
        return -1;
    }
    if (fpga_read_reg(handle, REG_DIV_HI, &freq_hi) != 0) {
        fprintf(stderr, "Failed to read frequency high register\n");
        return -1;
    }
    if (fpga_read_reg(handle, REG_STATUS, &status) != 0) {
        fprintf(stderr, "Failed to read status register\n");
        return -1;
    }

    uint16_t freq_mhz = (freq_hi << 8) | freq_lo;

    const char *width_str;
    switch ((ctrl & CTRL_WIDTH_MASK) >> 1) {
        case 0: width_str = "8-bit"; break;
        case 1: width_str = "16-bit"; break;
        case 2: width_str = "24-bit"; break;
        case 3: width_str = "32-bit"; break;
        default: width_str = "unknown"; break;
    }

    printf("\nFPGA Configuration:\n");
    printf("  Control Register:   0x%02x\n", ctrl);
    printf("  Clock Mode:         %s\n", (ctrl & CTRL_CLOCK_MODE) ? "Master (FPGA generates)" : "Slave (external)");
    printf("  Counter Width:      %s\n", width_str);
    printf("  Master Clock Freq:  %u MHz\n", freq_mhz);
    printf("\nStatus Register:      0x%02x\n", status);
    printf("  Active Clock Mode:  %s\n", (status & STATUS_CLOCK_MODE) ? "Master" : "Slave");
    printf("  PLL Locked:         %s\n", (status & STATUS_PLL_LOCKED) ? "Yes" : "No");

    return 0;
}

int main(int argc, char *argv[]) {
    const char *usb_fw_path = NULL;
    int set_width = -1;       /* -1=no change, 8/16/24/32 */
    float set_freq = -1;      /* -1=no change, 0=slave, >0=master */

    /* Parse arguments */
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            return 0;
        } else if ((strcmp(argv[i], "-w") == 0 || strcmp(argv[i], "--width") == 0) && i + 1 < argc) {
            set_width = atoi(argv[++i]);
            if (set_width != 8 && set_width != 16 && set_width != 24 && set_width != 32) {
                fprintf(stderr, "Invalid width: %d (must be 8, 16, 24, or 32)\n", set_width);
                return 1;
            }
        } else if ((strcmp(argv[i], "-f") == 0 || strcmp(argv[i], "--freq") == 0) && i + 1 < argc) {
            set_freq = atof(argv[++i]);
            if (set_freq < 0 || set_freq > 100) {
                fprintf(stderr, "Invalid frequency: %.2f MHz (must be 0-100)\n", set_freq);
                return 1;
            }
        } else if (strcmp(argv[i], "--firmware") == 0 && i + 1 < argc) {
            usb_fw_path = argv[++i];
        } else {
            fprintf(stderr, "Unknown option: %s\n", argv[i]);
            print_usage(argv[0]);
            return 1;
        }
    }

    printf("FPGA Configuration Utility\n");
    printf("==========================\n\n");

    /* Initialize libusb */
    libusb_context *ctx = NULL;
    int ret = libusb_init(&ctx);
    if (ret < 0) {
        fprintf(stderr, "Failed to initialize libusb: %s\n", libusb_error_name(ret));
        return 1;
    }

    /* Initialize FX3 device */
    libusb_device_handle *handle = NULL;
    ret = fx3_init(ctx, &handle, usb_fw_path, 0);
    if (ret < 0) {
        fprintf(stderr, "Failed to initialize FX3 device (error %d)\n", ret);
        libusb_exit(ctx);
        return 1;
    }

    /* Get and display firmware version */
    uint8_t major, minor;
    if (fx3_get_firmware_version(handle, &major, &minor) == 0) {
        printf("FX3 firmware version: %d.%d\n", major, minor);
    }

    /* Check if FPGA is responding */
    uint8_t dummy;
    if (fx3_i2c_read(handle, FPGA_I2C_ADDR, &dummy, 1) != 0) {
        fprintf(stderr, "FPGA not responding at I2C address 0x%02x\n", FPGA_I2C_ADDR);
        fprintf(stderr, "Make sure the FPGA is programmed with the validation bitstream.\n");
        fx3_close(handle);
        libusb_exit(ctx);
        return 1;
    }
    printf("FPGA detected at I2C address 0x%02x\n", FPGA_I2C_ADDR);

    /* Apply configuration changes */
    if (set_freq >= 0 || set_width >= 0) {
        uint8_t ctrl;
        if (fpga_read_reg(handle, REG_CONTROL, &ctrl) != 0) {
            fprintf(stderr, "Failed to read control register\n");
            fx3_close(handle);
            libusb_exit(ctx);
            return 1;
        }

        if (set_freq >= 0) {
            if (set_freq == 0) {
                /* Slave mode (external clock) */
                ctrl &= ~CTRL_CLOCK_MODE;
                printf("Setting mode: Slave (external clock)\n");
            } else {
                /* Master mode with specified frequency */
                ctrl |= CTRL_CLOCK_MODE;
                uint16_t freq_reg = freq_to_reg(set_freq);
                printf("Setting mode: Master at %u MHz\n", freq_reg);

                if (fpga_write_reg(handle, REG_DIV_LO, freq_reg & 0xFF) != 0) {
                    fprintf(stderr, "Failed to write frequency low register\n");
                    fx3_close(handle);
                    libusb_exit(ctx);
                    return 1;
                }
                if (fpga_write_reg(handle, REG_DIV_HI, (freq_reg >> 8) & 0xFF) != 0) {
                    fprintf(stderr, "Failed to write frequency high register\n");
                    fx3_close(handle);
                    libusb_exit(ctx);
                    return 1;
                }
            }
        }

        if (set_width >= 0) {
            ctrl &= ~CTRL_WIDTH_MASK;
            switch (set_width) {
                case 8:  ctrl |= CTRL_WIDTH_8BIT; break;
                case 16: ctrl |= CTRL_WIDTH_16BIT; break;
                case 24: ctrl |= CTRL_WIDTH_24BIT; break;
                case 32: ctrl |= CTRL_WIDTH_32BIT; break;
            }
            printf("Setting counter width: %d-bit\n", set_width);
        }

        if (fpga_write_reg(handle, REG_CONTROL, ctrl) != 0) {
            fprintf(stderr, "Failed to write control register\n");
            fx3_close(handle);
            libusb_exit(ctx);
            return 1;
        }
    }

    /* Always read back and display current configuration */
    printf("\nCurrent configuration:\n");
    uint8_t ctrl_readback;
    if (fpga_read_reg(handle, REG_CONTROL, &ctrl_readback) == 0) {
        const char *width_str;
        switch ((ctrl_readback & CTRL_WIDTH_MASK) >> 1) {
            case 0: width_str = "8-bit"; break;
            case 1: width_str = "16-bit"; break;
            case 2: width_str = "24-bit"; break;
            case 3: width_str = "32-bit"; break;
            default: width_str = "unknown"; break;
        }
        printf("  Control: 0x%02x (mode=%s, width=%s)\n",
               ctrl_readback,
               (ctrl_readback & CTRL_CLOCK_MODE) ? "master" : "slave",
               width_str);
    } else {
        fprintf(stderr, "  Failed to read back control register\n");
    }

    /* Cleanup */
    fx3_close(handle);
    libusb_exit(ctx);

    return 0;
}
