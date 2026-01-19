/*
 * I2C EEPROM Programmer - Programs firmware file to EEPROM
 *
 * Writes a .fw firmware image to I2C EEPROM for FX3 boot.
 * The M24M02 is a 2Mbit (256KB) EEPROM using 4 I2C addresses (0x50-0x53).
 * Page size is 256 bytes; writes must not cross page boundaries.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <openfx3/device.h>
#include <openfx3/platform.h>

/* M24M02 EEPROM parameters */
#define EEPROM_BASE_ADDR    0x50
#define EEPROM_PAGE_SIZE    256
#define EEPROM_WRITE_TIME   5    /* 5ms typical write cycle time */
#define EEPROM_BANK_SIZE    0x10000  /* 64KB per bank */

/* Default firmware path */
#define DEFAULT_EEPROM_FW_PATH "build/openfx3.fw"

static void print_usage(const char *prog) {
    printf("Usage: %s [firmware.fw] [options]\n", prog);
    printf("\nPrograms a firmware image to I2C EEPROM for FX3 boot.\n");
    printf("\nArguments:\n");
    printf("  firmware.fw         Firmware file to program (default: %s)\n", DEFAULT_EEPROM_FW_PATH);
    printf("\nOptions:\n");
    printf("  -h, --help          Show this help\n");
    printf("  -r, --reset         Reset device and reload firmware first\n");
    printf("  --firmware <path>   Firmware file for FX3 USB (default: embedded)\n");
    printf("  -n, --dry-run       Don't actually program, just show what would be done\n");
    printf("  --offset <hex>      EEPROM offset to program at (default: 0x0)\n");
    printf("  -e, --erase         Erase EEPROM (fill with 0xFF) instead of programming\n");
}

static int find_eeprom(libusb_device_handle *handle, uint8_t *base_addr, int *num_banks) {
    printf("Scanning for M24M02 EEPROM...\n");

    /* Retry scan a few times - I2C may need warmup on fresh connection */
    for (int attempt = 0; attempt < 3; attempt++) {
        *num_banks = 0;
        *base_addr = 0;

        for (uint8_t addr = 0x50; addr <= 0x57; addr++) {
            uint8_t dummy;
            int ret = fx3_i2c_read(handle, addr, &dummy, 1);
            if (ret == 0) {
                if (*num_banks == 0) {
                    *base_addr = addr;
                }
                (*num_banks)++;
                printf("  Found device at 0x%02x\n", addr);
            }
        }

        if (*num_banks > 0)
            break;

        /* Wait and retry */
        if (attempt < 2) {
            printf("  Retrying scan...\n");
            sleep_ms(100);
        }
    }

    if (*num_banks == 0) {
        printf("  No EEPROM found!\n");
        return -1;
    }

    if (*num_banks == 4 && *base_addr == 0x50) {
        printf("  Detected M24M02 (256KB EEPROM)\n");
    } else if (*num_banks == 2) {
        printf("  Detected 128KB EEPROM\n");
    } else if (*num_banks == 1) {
        printf("  Detected 64KB EEPROM\n");
    }

    return 0;
}

/* Max data per write: full EEPROM page (256 bytes) */
#define EEPROM_MAX_WRITE    256

static int eeprom_write_page(libusb_device_handle *handle, uint8_t base_addr,
                              uint32_t offset, const uint8_t *data, size_t len) {
    if (len == 0 || len > EEPROM_MAX_WRITE)
        return -1;

    /* Calculate bank and internal offset */
    uint8_t bank = (offset >> 16) & 0x03;
    uint16_t internal_offset = offset & 0xFFFF;
    uint8_t i2c_addr = base_addr + bank;

    /* Build write buffer: 2-byte address + data */
    uint8_t write_buf[258];
    write_buf[0] = (internal_offset >> 8) & 0xFF;
    write_buf[1] = internal_offset & 0xFF;
    memcpy(&write_buf[2], data, len);

    int ret = fx3_i2c_write(handle, i2c_addr, write_buf, len + 2);
    if (ret != 0) {
        return ret;
    }

    /* Poll for write completion - EEPROM NACKs until write cycle completes
     * M24M02 typical write time is 5ms max, but often faster.
     * Poll aggressively first, then back off. */
    for (int i = 0; i < 50; i++) {
        uint8_t dummy;
        ret = fx3_i2c_read(handle, i2c_addr, &dummy, 1);
        if (ret == 0) {
            return 0;  /* Write complete */
        }
        /* Short delay between polls - USB round trip is ~1ms anyway */
        //sleep_ms(1);
    }

    return -1;  /* Timeout */
}

static int eeprom_read(libusb_device_handle *handle, uint8_t base_addr,
                       uint32_t offset, uint8_t *data, size_t len) {
    uint8_t bank = (offset >> 16) & 0x03;
    uint16_t internal_offset = offset & 0xFFFF;
    uint8_t i2c_addr = base_addr + bank;

    /* Set address pointer */
    uint8_t addr_buf[2];
    addr_buf[0] = (internal_offset >> 8) & 0xFF;
    addr_buf[1] = internal_offset & 0xFF;

    int ret = fx3_i2c_write(handle, i2c_addr, addr_buf, 2);
    if (ret != 0)
        return ret;

    ret = fx3_i2c_read(handle, i2c_addr, data, len);
    return ret;
}

static int program_firmware(libusb_device_handle *handle, uint8_t base_addr,
                            int num_banks, uint32_t start_offset,
                            const uint8_t *fw_data, size_t fw_size,
                            int verify, int dry_run) {
    uint32_t eeprom_size = num_banks * EEPROM_BANK_SIZE;

    if (start_offset + fw_size > eeprom_size) {
        fprintf(stderr, "Firmware (%zu bytes) won't fit at offset 0x%x (EEPROM size: %u bytes)\n",
                fw_size, start_offset, eeprom_size);
        return -1;
    }

    printf("\nProgramming %zu bytes at offset 0x%06x...\n", fw_size, start_offset);

    if (dry_run) {
        printf("(Dry run - no actual programming)\n");
        return 0;
    }

    /* Program in chunks, respecting page boundaries */
    size_t bytes_written = 0;
    uint32_t current_offset = start_offset;
    int errors = 0;

    while (bytes_written < fw_size) {
        /* Calculate chunk size - don't cross page boundary */
        uint32_t page_offset = current_offset % EEPROM_PAGE_SIZE;
        size_t chunk = EEPROM_PAGE_SIZE - page_offset;
        if (chunk > EEPROM_MAX_WRITE) chunk = EEPROM_MAX_WRITE;
        if (bytes_written + chunk > fw_size) {
            chunk = fw_size - bytes_written;
        }

        /* Don't cross bank boundary */
        uint32_t bank_offset = current_offset & 0xFFFF;
        if (bank_offset + chunk > EEPROM_BANK_SIZE) {
            chunk = EEPROM_BANK_SIZE - bank_offset;
        }

        int ret = eeprom_write_page(handle, base_addr, current_offset,
                                    &fw_data[bytes_written], chunk);
        if (ret != 0) {
            fprintf(stderr, "\nWrite failed at offset 0x%06x\n", current_offset);
            errors++;
            if (errors > 10) {
                fprintf(stderr, "Too many errors, aborting\n");
                return -1;
            }
        }

        bytes_written += chunk;
        current_offset += chunk;

        /* Progress */
        int pct = (bytes_written * 100) / fw_size;
        printf("\rProgramming: %zu / %zu bytes (%d%%)...", bytes_written, fw_size, pct);
        fflush(stdout);
    }

    printf("\rProgramming: %zu / %zu bytes (100%%) - Done!     \n", bytes_written, fw_size);

    /* Verify if requested */
    if (verify) {
        printf("Verifying...\n");

        uint8_t *verify_buf = malloc(fw_size);
        if (!verify_buf) {
            fprintf(stderr, "Failed to allocate verify buffer\n");
            return -1;
        }

        size_t bytes_read = 0;
        current_offset = start_offset;

        while (bytes_read < fw_size) {
            size_t chunk = fw_size - bytes_read;
            if (chunk > 60) chunk = 60;

            /* Don't cross bank boundary */
            uint32_t bank_offset = current_offset & 0xFFFF;
            if (bank_offset + chunk > EEPROM_BANK_SIZE) {
                chunk = EEPROM_BANK_SIZE - bank_offset;
            }

            int ret = eeprom_read(handle, base_addr, current_offset,
                                  &verify_buf[bytes_read], chunk);
            if (ret != 0) {
                fprintf(stderr, "\nVerify read failed at offset 0x%06x\n", current_offset);
                free(verify_buf);
                return -1;
            }

            bytes_read += chunk;
            current_offset += chunk;

            int pct = (bytes_read * 100) / fw_size;
            printf("\rVerifying: %zu / %zu bytes (%d%%)...", bytes_read, fw_size, pct);
            fflush(stdout);
        }

        printf("\rVerifying: %zu / %zu bytes (100%%)              \n", bytes_read, fw_size);

        /* Compare */
        int verify_errors = 0;
        for (size_t i = 0; i < fw_size; i++) {
            if (verify_buf[i] != fw_data[i]) {
                if (verify_errors < 10) {
                    fprintf(stderr, "Mismatch at 0x%06zx: wrote 0x%02x, read 0x%02x\n",
                            start_offset + i, fw_data[i], verify_buf[i]);
                }
                verify_errors++;
            }
        }

        free(verify_buf);

        if (verify_errors > 0) {
            fprintf(stderr, "Verification FAILED: %d errors\n", verify_errors);
            return -1;
        }

        printf("Verification PASSED!\n");
    }

    return errors > 0 ? -1 : 0;
}

int main(int argc, char *argv[]) {
    int force_reload = 0;
    const char *usb_fw_path = NULL;
    const char *eeprom_fw_path = NULL;
    uint32_t offset = 0x0;
    int dry_run = 0;
    int erase_mode = 0;

    /* Parse arguments */
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            return 0;
        } else if (strcmp(argv[i], "-r") == 0 || strcmp(argv[i], "--reset") == 0) {
            force_reload = 1;
        } else if (strcmp(argv[i], "--firmware") == 0 && i + 1 < argc) {
            usb_fw_path = argv[++i];
        } else if (strcmp(argv[i], "-n") == 0 || strcmp(argv[i], "--dry-run") == 0) {
            dry_run = 1;
        } else if (strcmp(argv[i], "-e") == 0 || strcmp(argv[i], "--erase") == 0) {
            erase_mode = 1;
        } else if (strcmp(argv[i], "--offset") == 0 && i + 1 < argc) {
            offset = strtoul(argv[++i], NULL, 16);
        } else if (argv[i][0] != '-' && eeprom_fw_path == NULL) {
            eeprom_fw_path = argv[i];
        } else {
            fprintf(stderr, "Unknown option: %s\n", argv[i]);
            print_usage(argv[0]);
            return 1;
        }
    }

    if (eeprom_fw_path == NULL && !erase_mode) {
        eeprom_fw_path = DEFAULT_EEPROM_FW_PATH;
    }

    printf("FX3 I2C EEPROM Programmer\n");
    printf("=========================\n\n");

    uint8_t *fw_data = NULL;
    long fw_size = 0;

    if (erase_mode) {
        printf("Mode: ERASE (fill with 0xFF)\n");
    } else {
        /* Read firmware file */
        printf("Reading firmware file: %s\n", eeprom_fw_path);
        FILE *f = fopen(eeprom_fw_path, "rb");
        if (!f) {
            fprintf(stderr, "Failed to open firmware file: %s\n", eeprom_fw_path);
            return 1;
        }

        fseek(f, 0, SEEK_END);
        fw_size = ftell(f);
        fseek(f, 0, SEEK_SET);

        if (fw_size <= 0) {
            fprintf(stderr, "Invalid firmware file (empty or unreadable)\n");
            fclose(f);
            return 1;
        }

        if (fw_size > 256 * 1024) {
            fprintf(stderr, "Firmware too large: %ld bytes (max 256KB for M24M02 EEPROM)\n", fw_size);
            fprintf(stderr, "Consider reducing firmware size or using SPI flash boot instead.\n");
            fclose(f);
            return 1;
        }

        fw_data = malloc(fw_size);
        if (!fw_data) {
            fprintf(stderr, "Failed to allocate memory for firmware\n");
            fclose(f);
            return 1;
        }

        if (fread(fw_data, 1, fw_size, f) != (size_t)fw_size) {
            fprintf(stderr, "Failed to read firmware file\n");
            free(fw_data);
            fclose(f);
            return 1;
        }
        fclose(f);

        /* Validate firmware header */
        if (fw_size < 4 || fw_data[0] != 'C' || fw_data[1] != 'Y') {
            fprintf(stderr, "Invalid firmware file (missing CY header)\n");
            free(fw_data);
            return 1;
        }

        printf("Firmware size: %ld bytes\n", fw_size);
        printf("I2C config: 0x%02x, Image type: 0x%02x\n", fw_data[2], fw_data[3]);
    }

    /* Initialize libusb */
    libusb_context *ctx = NULL;
    int ret = libusb_init(&ctx);
    if (ret < 0) {
        fprintf(stderr, "Failed to initialize libusb: %s\n", libusb_error_name(ret));
        free(fw_data);
        return 1;
    }

    /* Reset and reload firmware if requested */
    libusb_device_handle *handle = NULL;
    if (force_reload) {
        ret = fx3_open_device(ctx, &handle, 0);
        if (ret >= 0 && handle) {
            printf("Resetting device to bootloader...\n");
            fx3_reboot_bootloader(handle);
            fx3_close(handle);
            handle = NULL;
            sleep_ms(500);
        }
    }

    /* Initialize FX3 device */
    ret = fx3_init(ctx, &handle, usb_fw_path, force_reload);
    if (ret < 0) {
        fprintf(stderr, "Failed to initialize FX3 device (error %d)\n", ret);
        free(fw_data);
        libusb_exit(ctx);
        return 1;
    }

    /* Get and display firmware version */
    uint8_t major, minor;
    if (fx3_get_firmware_version(handle, &major, &minor) == 0) {
        printf("Running firmware version: %d.%d\n\n", major, minor);
    }

    /* Find EEPROM */
    uint8_t base_addr;
    int num_banks;
    if (find_eeprom(handle, &base_addr, &num_banks) < 0) {
        free(fw_data);
        fx3_close(handle);
        libusb_exit(ctx);
        return 1;
    }

    printf("\n");

    if (erase_mode) {
        /* Erase mode: fill entire EEPROM with 0xFF */
        uint32_t eeprom_size = num_banks * EEPROM_BANK_SIZE;
        fw_size = eeprom_size;
        fw_data = malloc(fw_size);
        if (!fw_data) {
            fprintf(stderr, "Failed to allocate memory for erase buffer\n");
            fx3_close(handle);
            libusb_exit(ctx);
            return 1;
        }
        memset(fw_data, 0xFF, fw_size);
        printf("Erasing %ld bytes (%d KB)...\n", fw_size, (int)(fw_size / 1024));
    }

    /* Program the firmware (or erase pattern), always verify */
    ret = program_firmware(handle, base_addr, num_banks, offset,
                           fw_data, fw_size, 1, dry_run);

    if (ret == 0) {
        if (erase_mode) {
            printf("\n*** Erase complete! ***\n");
        } else {
            printf("\n*** Programming complete! ***\n");
            printf("\nTo boot from EEPROM:\n");
            printf("  1. Set PMODE[2:0] pins to select I2C boot\n");
            printf("  2. Power cycle the device\n");
        }
    } else {
        printf("\n*** %s FAILED ***\n", erase_mode ? "Erase" : "Programming");
    }

    /* Cleanup */
    free(fw_data);
    fx3_close(handle);
    libusb_exit(ctx);

    return (ret == 0) ? 0 : 1;
}
