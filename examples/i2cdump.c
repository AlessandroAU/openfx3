/*
 * I2C EEPROM Dump - Reads and displays EEPROM contents
 *
 * Uses the fx3_i2c_read_reg function which does a write-then-read
 * in a single I2C transaction (repeated start).
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <openfx3/device.h>
#include <openfx3/platform.h>

/* M24M02 EEPROM parameters */
#define EEPROM_BASE_ADDR    0x50

static void print_usage(const char *prog) {
    printf("Usage: %s [options]\n", prog);
    printf("\nOptions:\n");
    printf("  -h, --help          Show this help\n");
    printf("  -r, --reset         Reset device and reload firmware\n");
    printf("  --firmware <path>   Firmware file (default: embedded)\n");
    printf("  --offset <hex>      Start offset (default: 0x0)\n");
    printf("  --length <n>        Number of bytes to read (default: 256)\n");
}

static void hexdump(uint32_t offset, const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; i += 16) {
        printf("%06x: ", (unsigned)(offset + i));

        /* Hex bytes */
        for (size_t j = 0; j < 16; j++) {
            if (i + j < len) {
                printf("%02x ", data[i + j]);
            } else {
                printf("   ");
            }
            if (j == 7) printf(" ");
        }

        printf(" |");

        /* ASCII */
        for (size_t j = 0; j < 16 && i + j < len; j++) {
            uint8_t c = data[i + j];
            printf("%c", (c >= 32 && c < 127) ? c : '.');
        }

        printf("|\n");
    }
}

int main(int argc, char *argv[]) {
    int force_reload = 0;
    const char *fw_path = NULL;
    uint32_t offset = 0x0;
    size_t length = 256 * 4;

    /* Parse arguments */
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            return 0;
        } else if (strcmp(argv[i], "-r") == 0 || strcmp(argv[i], "--reset") == 0) {
            force_reload = 1;
        } else if (strcmp(argv[i], "--firmware") == 0 && i + 1 < argc) {
            fw_path = argv[++i];
        } else if (strcmp(argv[i], "--offset") == 0 && i + 1 < argc) {
            offset = strtoul(argv[++i], NULL, 16);
        } else if (strcmp(argv[i], "--length") == 0 && i + 1 < argc) {
            length = strtoul(argv[++i], NULL, 10);
            if (length > 4096) length = 4096;  /* Reasonable limit */
        } else {
            fprintf(stderr, "Unknown option: %s\n", argv[i]);
            print_usage(argv[0]);
            return 1;
        }
    }

    printf("FX3 I2C EEPROM Dump\n");
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
    ret = fx3_init(ctx, &handle, fw_path, force_reload);
    if (ret < 0) {
        fprintf(stderr, "Failed to initialize FX3 device (error %d)\n", ret);
        libusb_exit(ctx);
        return 1;
    }

    /* Get and display firmware version */
    uint8_t major, minor;
    if (fx3_get_firmware_version(handle, &major, &minor) == 0) {
        printf("Firmware version: %d.%d\n\n", major, minor);
    }

    /* Find EEPROM */
    printf("Scanning for EEPROM...\n");
    uint8_t base_addr = 0;
    int num_banks = 0;

    for (uint8_t addr = 0x50; addr <= 0x57; addr++) {
        uint8_t dummy;
        ret = fx3_i2c_read(handle, addr, &dummy, 1);
        if (ret == 0) {
            if (num_banks == 0) {
                base_addr = addr;
            }
            num_banks++;
            printf("  Found device at 0x%02x\n", addr);
        }
    }

    if (num_banks == 0) {
        printf("  No EEPROM found!\n");
        fx3_close(handle);
        libusb_exit(ctx);
        return 1;
    }

    printf("\n");

    /* Calculate total EEPROM size */
    uint32_t eeprom_size = num_banks * 0x10000;  /* 64KB per bank */
    printf("EEPROM size: %u KB (%d banks)\n", eeprom_size / 1024, num_banks);
    printf("Reading from offset 0x%06x, length %zu bytes\n\n", offset, length);

    if (offset >= eeprom_size) {
        fprintf(stderr, "Offset 0x%x is beyond EEPROM size 0x%x\n", offset, eeprom_size);
        fx3_close(handle);
        libusb_exit(ctx);
        return 1;
    }

    /* Allocate buffer */
    uint8_t *buffer = malloc(length);
    if (!buffer) {
        fprintf(stderr, "Failed to allocate buffer\n");
        fx3_close(handle);
        libusb_exit(ctx);
        return 1;
    }

    /* Read EEPROM in chunks using fx3_i2c_read_reg (write-read with repeated start) */
    size_t bytes_read = 0;
    uint32_t current_offset = offset;

    while (bytes_read < length) {
        /* Calculate which bank and internal offset */
        uint8_t bank = (current_offset >> 16) & 0x03;
        uint16_t internal_offset = current_offset & 0xFFFF;
        uint8_t i2c_addr = base_addr + bank;

        /* Read up to 60 bytes at a time (limit of fx3_i2c_read_reg) */
        size_t chunk = length - bytes_read;
        if (chunk > 60) chunk = 60;

        /* Don't cross bank boundary */
        if (internal_offset + chunk > 0x10000) {
            chunk = 0x10000 - internal_offset;
        }

        /* fx3_i2c_read_reg does: write 1 byte (reg), then read N bytes
         * For EEPROM we need 2-byte address, so this won't work directly.
         * Let's just do separate write + read for now */
        uint8_t addr_buf[2];
        addr_buf[0] = (internal_offset >> 8) & 0xFF;
        addr_buf[1] = internal_offset & 0xFF;

        ret = fx3_i2c_write(handle, i2c_addr, addr_buf, 2);
        if (ret != 0) {
            fprintf(stderr, "Failed to set EEPROM address at 0x%06x (error %d)\n",
                    current_offset, ret);
            /* Try to continue with next chunk */
            memset(&buffer[bytes_read], 0xFF, chunk);
            bytes_read += chunk;
            current_offset += chunk;
            continue;
        }

        ret = fx3_i2c_read(handle, i2c_addr, &buffer[bytes_read], chunk);
        if (ret != 0) {
            fprintf(stderr, "Failed to read at 0x%06x (error %d)\n", current_offset, ret);
            memset(&buffer[bytes_read], 0xFF, chunk);
        }

        bytes_read += chunk;
        current_offset += chunk;

        /* Progress indicator for large reads */
        if (length > 256 && bytes_read % 256 == 0) {
            printf("\rRead %zu / %zu bytes...", bytes_read, length);
            fflush(stdout);
        }
    }

    if (length > 256) {
        printf("\r                              \r");
    }

    /* Display hex dump */
    hexdump(offset, buffer, length);

    /* Cleanup */
    free(buffer);
    fx3_close(handle);
    libusb_exit(ctx);

    return 0;
}
