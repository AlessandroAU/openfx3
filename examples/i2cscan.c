/*
 * I2C Bus Scanner - similar to i2cdetect
 *
 * Scans the I2C bus for devices and displays results in a grid format.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <openfx3/device.h>
#include <openfx3/platform.h>

static void print_usage(const char *prog) {
    printf("Usage: %s [options] [start_addr [end_addr]]\n", prog);
    printf("\nOptions:\n");
    printf("  -h, --help          Show this help\n");
    printf("  -v, --verbose       Verbose output\n");
    printf("  -r, --reset         Reset device and reload firmware\n");
    printf("  --firmware <path>   Firmware file (default: embedded)\n");
    printf("\nAddress range:\n");
    printf("  start_addr     Starting address in hex (default: 0x08)\n");
    printf("  end_addr       Ending address in hex (default: 0x77)\n");
    printf("\nExamples:\n");
    printf("  %s              Scan full range 0x08-0x77\n", prog);
    printf("  %s 0x50         Scan single address 0x50\n", prog);
    printf("  %s 0x50 0x57    Scan range 0x50-0x57 (EEPROM addresses)\n", prog);
    printf("  %s 50 57        Hex prefix is optional\n", prog);
}

static int parse_addr(const char *str) {
    char *endptr;
    long val = strtol(str, &endptr, 16);
    if (*endptr != '\0' || val < 0 || val > 0x7F) {
        return -1;
    }
    return (int)val;
}

int main(int argc, char *argv[]) {
    int verbose = 0;
    int start_addr = 0x08;
    int end_addr = 0x77;
    int force_reload = 0;
    const char *fw_path = NULL;

    /* Parse arguments */
    int positional = 0;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            return 0;
        } else if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--verbose") == 0) {
            verbose = 1;
        } else if (strcmp(argv[i], "-r") == 0 || strcmp(argv[i], "--reset") == 0) {
            force_reload = 1;
        } else if (strcmp(argv[i], "--firmware") == 0 && i + 1 < argc) {
            fw_path = argv[++i];
        } else if (argv[i][0] != '-') {
            int addr = parse_addr(argv[i]);
            if (addr < 0) {
                fprintf(stderr, "Invalid address: %s\n", argv[i]);
                return 1;
            }
            if (positional == 0) {
                start_addr = addr;
                end_addr = addr;  /* Default to single address if only one given */
            } else if (positional == 1) {
                end_addr = addr;
            } else {
                fprintf(stderr, "Too many arguments\n");
                print_usage(argv[0]);
                return 1;
            }
            positional++;
        } else {
            fprintf(stderr, "Unknown option: %s\n", argv[i]);
            print_usage(argv[0]);
            return 1;
        }
    }

    if (start_addr > end_addr) {
        fprintf(stderr, "Start address (0x%02x) must be <= end address (0x%02x)\n",
                start_addr, end_addr);
        return 1;
    }

    printf("FX3 I2C Bus Scanner\n");
    printf("===================\n\n");

    /* Initialize libusb */
    libusb_context *ctx = NULL;
    int ret = libusb_init(&ctx);
    if (ret < 0) {
        fprintf(stderr, "Failed to initialize libusb: %s\n", libusb_error_name(ret));
        return 1;
    }

    /* Reset and reload firmware if requested */
    libusb_device_handle *handle = NULL;
    if (force_reload) {
        /* Try to open device and send reboot command */
        ret = fx3_open_device(ctx, &handle, 0);
        if (ret >= 0 && handle) {
            printf("Resetting device to bootloader...\n");
            fx3_reboot_bootloader(handle);
            fx3_close(handle);
            handle = NULL;
            /* Wait for device to reset and re-enumerate */
            sleep_ms(500);
        }
    }

    /* Initialize FX3 device */
    ret = fx3_init(ctx, &handle, fw_path, force_reload);
    if (ret < 0) {
        fprintf(stderr, "Failed to initialize FX3 device (error %d)\n", ret);
        libusb_exit(ctx);
        return 1;
    }

    /* Get and display firmware version */
    uint8_t major, minor;
    if (fx3_get_firmware_version(handle, &major, &minor) == 0) {
        printf("Firmware version: %d.%d\n", major, minor);
    }

    printf("\nScanning I2C bus (0x%02x-0x%02x)...\n\n", start_addr, end_addr);

    /* Print header */
    printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");

    /* Track found devices */
    uint8_t found[112];
    int found_count = 0;

    /* Determine row range based on address range */
    int start_row = start_addr >> 4;
    int end_row = end_addr >> 4;

    /* Scan addresses and build result grid */
    for (int row = start_row; row <= end_row; row++) {
        printf("%02x:", row << 4);

        for (int col = 0; col < 16; col++) {
            uint8_t addr = (row << 4) | col;

            /* Skip addresses outside the requested range */
            if (addr < start_addr || addr > end_addr) {
                printf("   ");
                continue;
            }

            /* Try to read 1 byte from this address */
            uint8_t dummy;
            ret = fx3_i2c_read(handle, addr, &dummy, 1);

            if (ret == 0) {
                /* Device found */
                printf(" %02x", addr);
                if (found_count < 112) {
                    found[found_count++] = addr;
                }
            } else {
                /* No device or error */
                printf(" --");
            }
        }
        printf("\n");
    }

    /* Summary */
    printf("\n");
    if (found_count == 0) {
        printf("No I2C devices found.\n");
    } else {
        printf("Found %d device(s) at address(es):", found_count);
        for (int i = 0; i < found_count; i++) {
            printf(" 0x%02x", found[i]);
        }
        printf("\n");

        /* If verbose, try to identify common devices */
        if (verbose) {
            printf("\nDevice identification (common addresses):\n");
            for (int i = 0; i < found_count; i++) {
                uint8_t addr = found[i];
                const char *desc = NULL;

                /* Common I2C device addresses */
                switch (addr) {
                    case 0x20: case 0x21: case 0x22: case 0x23:
                    case 0x24: case 0x25: case 0x26: case 0x27:
                        desc = "PCF8574/PCA9555 I/O Expander or similar";
                        break;
                    case 0x38: case 0x39: case 0x3a: case 0x3b:
                        desc = "PCF8574A I/O Expander or AHT10/AHT20 sensor";
                        break;
                    case 0x3c: case 0x3d:
                        desc = "SSD1306 OLED display";
                        break;
                    case 0x3e: case 0x3f:
                        desc = "PCF8574A I/O Expander";
                        break;
                    case 0x48: case 0x49: case 0x4a: case 0x4b:
                        desc = "ADS1115/TMP102/LM75 or similar";
                        break;
                    case 0x50: case 0x51: case 0x52: case 0x53:
                    case 0x54: case 0x55: case 0x56: case 0x57:
                        desc = "24Cxx EEPROM";
                        break;
                    case 0x68:
                        desc = "DS1307/DS3231 RTC or MPU6050 IMU";
                        break;
                    case 0x69:
                        desc = "MPU6050 IMU (AD0 high)";
                        break;
                    case 0x76: case 0x77:
                        desc = "BME280/BMP280/MS5611 pressure sensor";
                        break;
                    default:
                        desc = "Unknown device";
                        break;
                }
                printf("  0x%02x: %s\n", addr, desc);
            }
        }
    }

    /* Cleanup */
    fx3_close(handle);
    libusb_exit(ctx);

    return 0;
}
