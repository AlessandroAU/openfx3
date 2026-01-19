/*
 * I2C EEPROM Test - Generic read/write test for I2C EEPROMs
 *
 * Supports various EEPROM types with auto-detection:
 *   - Small EEPROMs (24C01-24C16): 1-byte address, single I2C address
 *   - Medium EEPROMs (24C32-24C512): 2-byte address, single I2C address
 *   - Large EEPROMs (M24M01, M24M02): 2-byte address, multiple I2C addresses
 *
 * The test writes a pattern, reads it back, verifies, and restores original data.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <openfx3/device.h>
#include <openfx3/platform.h>

/* EEPROM detection result */
typedef struct {
    uint8_t base_addr;      /* First I2C address */
    int num_addrs;          /* Number of consecutive addresses responding */
    int addr_bytes;         /* 1 or 2 byte addressing */
    uint32_t total_size;    /* Estimated total size in bytes */
    const char *type;       /* Detected type string */
} eeprom_info_t;

/* Test parameters */
#define DEFAULT_TEST_OFFSET  0x100  /* Avoid offset 0 which may have boot data */
#define DEFAULT_TEST_SIZE    16
#define EEPROM_WRITE_TIME_MS 5      /* Typical write cycle time */

static void print_usage(const char *prog) {
    printf("Usage: %s [options]\n", prog);
    printf("\nTests I2C EEPROM read/write functionality.\n");
    printf("\nOptions:\n");
    printf("  -h, --help          Show this help\n");
    printf("  -r, --reset         Reset device and reload firmware\n");
    printf("  --firmware <path>   Firmware file (default: embedded)\n");
    printf("  --offset <hex>      EEPROM offset to test (default: 0x%x)\n", DEFAULT_TEST_OFFSET);
    printf("  --size <n>          Number of bytes to test (default: %d, max 64)\n", DEFAULT_TEST_SIZE);
    printf("  --addr <hex>        I2C address to use (default: auto-detect at 0x50-0x57)\n");
    printf("  --addr-bytes <1|2>  Address byte count (default: auto-detect)\n");
}

static int detect_eeprom(libusb_device_handle *handle, eeprom_info_t *info) {
    printf("Scanning for I2C EEPROM...\n");

    memset(info, 0, sizeof(*info));

    /* Scan EEPROM address range 0x50-0x57 */
    uint8_t first_addr = 0;
    int count = 0;

    for (uint8_t addr = 0x50; addr <= 0x57; addr++) {
        uint8_t dummy;
        int ret = fx3_i2c_read(handle, addr, &dummy, 1);
        if (ret == 0) {
            if (count == 0) {
                first_addr = addr;
            }
            count++;
            printf("  Found device at 0x%02x\n", addr);
        }
    }

    if (count == 0) {
        printf("  No EEPROM found!\n");
        return -1;
    }

    info->base_addr = first_addr;
    info->num_addrs = count;

    /* Determine EEPROM type based on address pattern */
    if (count == 4 && first_addr == 0x50) {
        /* M24M02: 256KB using 4 addresses */
        info->addr_bytes = 2;
        info->total_size = 256 * 1024;
        info->type = "M24M02 (256KB)";
    } else if (count == 2 && first_addr == 0x50) {
        /* M24M01: 128KB using 2 addresses */
        info->addr_bytes = 2;
        info->total_size = 128 * 1024;
        info->type = "M24M01 (128KB)";
    } else if (count == 1) {
        /* Single address - could be various sizes
         * Try to detect by probing high addresses
         * Default to 2-byte addressing (24C32 and larger) */
        info->addr_bytes = 2;
        info->total_size = 4 * 1024;  /* Conservative default: 4KB */
        info->type = "Generic EEPROM (2-byte addr)";
    } else {
        /* Unknown configuration */
        info->addr_bytes = 2;
        info->total_size = count * 64 * 1024;
        info->type = "Unknown multi-address EEPROM";
    }

    printf("  Detected: %s\n", info->type);
    printf("  Base address: 0x%02x, %d address(es), %d-byte addressing\n",
           info->base_addr, info->num_addrs, info->addr_bytes);

    return 0;
}

static int eeprom_write(libusb_device_handle *handle, const eeprom_info_t *info,
                        uint32_t offset, const uint8_t *data, size_t len) {
    if (len == 0 || len > 64) {
        fprintf(stderr, "Write size must be 1-64 bytes\n");
        return -1;
    }

    /* Calculate I2C address for multi-address EEPROMs */
    uint8_t i2c_addr = info->base_addr;
    uint16_t internal_offset = offset;

    if (info->num_addrs > 1) {
        /* Large EEPROM: upper bits select address */
        uint8_t bank = (offset >> 16) & 0x03;
        i2c_addr = info->base_addr + bank;
        internal_offset = offset & 0xFFFF;
    }

    /* Build write buffer */
    uint8_t write_buf[68];  /* Max: 2 addr + 64 data */
    size_t buf_len = 0;

    if (info->addr_bytes == 2) {
        write_buf[0] = (internal_offset >> 8) & 0xFF;
        write_buf[1] = internal_offset & 0xFF;
        buf_len = 2;
    } else {
        write_buf[0] = internal_offset & 0xFF;
        buf_len = 1;
    }

    memcpy(&write_buf[buf_len], data, len);
    buf_len += len;

    /* Perform write */
    int ret = fx3_i2c_write(handle, i2c_addr, write_buf, buf_len);
    if (ret != 0) {
        fprintf(stderr, "I2C write failed\n");
        return ret;
    }

    /* Poll for write completion (EEPROM NACKs during write cycle) */
    for (int i = 0; i < 20; i++) {
        sleep_ms(EEPROM_WRITE_TIME_MS);
        uint8_t dummy;
        ret = fx3_i2c_read(handle, i2c_addr, &dummy, 1);
        if (ret == 0) {
            return 0;  /* Write complete */
        }
    }

    fprintf(stderr, "EEPROM write timeout\n");
    return -1;
}

static int eeprom_read(libusb_device_handle *handle, const eeprom_info_t *info,
                       uint32_t offset, uint8_t *data, size_t len) {
    /* Calculate I2C address for multi-address EEPROMs */
    uint8_t i2c_addr = info->base_addr;
    uint16_t internal_offset = offset;

    if (info->num_addrs > 1) {
        uint8_t bank = (offset >> 16) & 0x03;
        i2c_addr = info->base_addr + bank;
        internal_offset = offset & 0xFFFF;
    }

    /* Set address pointer */
    uint8_t addr_buf[2];
    size_t addr_len;

    if (info->addr_bytes == 2) {
        addr_buf[0] = (internal_offset >> 8) & 0xFF;
        addr_buf[1] = internal_offset & 0xFF;
        addr_len = 2;
    } else {
        addr_buf[0] = internal_offset & 0xFF;
        addr_len = 1;
    }

    int ret = fx3_i2c_write(handle, i2c_addr, addr_buf, addr_len);
    if (ret != 0) {
        fprintf(stderr, "I2C address write failed\n");
        return ret;
    }

    /* Read data */
    ret = fx3_i2c_read(handle, i2c_addr, data, len);
    if (ret != 0) {
        fprintf(stderr, "I2C data read failed\n");
        return ret;
    }

    return 0;
}

static void print_hex(const char *label, const uint8_t *data, size_t len) {
    printf("%s: ", label);
    for (size_t i = 0; i < len; i++) {
        printf("%02x ", data[i]);
    }
    printf("\n");
}

int main(int argc, char *argv[]) {
    int force_reload = 0;
    const char *fw_path = NULL;
    uint32_t test_offset = DEFAULT_TEST_OFFSET;
    size_t test_size = DEFAULT_TEST_SIZE;
    int manual_addr = -1;
    int manual_addr_bytes = 0;

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
            test_offset = strtoul(argv[++i], NULL, 16);
        } else if (strcmp(argv[i], "--size") == 0 && i + 1 < argc) {
            test_size = strtoul(argv[++i], NULL, 10);
            if (test_size > 64) test_size = 64;
            if (test_size == 0) test_size = 1;
        } else if (strcmp(argv[i], "--addr") == 0 && i + 1 < argc) {
            manual_addr = strtoul(argv[++i], NULL, 16);
        } else if (strcmp(argv[i], "--addr-bytes") == 0 && i + 1 < argc) {
            manual_addr_bytes = atoi(argv[++i]);
        } else {
            fprintf(stderr, "Unknown option: %s\n", argv[i]);
            print_usage(argv[0]);
            return 1;
        }
    }

    printf("I2C EEPROM Test\n");
    printf("===============\n\n");

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

    /* Detect or configure EEPROM */
    eeprom_info_t eeprom;

    if (manual_addr >= 0) {
        /* Manual configuration */
        eeprom.base_addr = manual_addr;
        eeprom.num_addrs = 1;
        eeprom.addr_bytes = (manual_addr_bytes > 0) ? manual_addr_bytes : 2;
        eeprom.total_size = 0;
        eeprom.type = "Manual";
        printf("Using manual config: addr=0x%02x, %d-byte addressing\n\n",
               eeprom.base_addr, eeprom.addr_bytes);
    } else {
        if (detect_eeprom(handle, &eeprom) < 0) {
            fx3_close(handle);
            libusb_exit(ctx);
            return 1;
        }
    }

    printf("\n=== Read/Write Test ===\n");
    printf("Offset: 0x%06x\n", test_offset);
    printf("Size:   %zu bytes\n\n", test_size);

    /* Generate test pattern */
    uint8_t write_data[64];
    uint8_t read_data[64];
    uint8_t original_data[64];

    for (size_t i = 0; i < test_size; i++) {
        write_data[i] = (uint8_t)(0xA5 ^ i ^ (test_offset & 0xFF));
    }

    /* Step 1: Read original data */
    printf("Step 1: Reading original data...\n");
    ret = eeprom_read(handle, &eeprom, test_offset, original_data, test_size);
    if (ret < 0) {
        fprintf(stderr, "Failed to read original data\n");
        fx3_close(handle);
        libusb_exit(ctx);
        return 1;
    }
    print_hex("Original", original_data, test_size);

    /* Step 2: Write test pattern */
    printf("\nStep 2: Writing test pattern...\n");
    print_hex("Writing ", write_data, test_size);
    ret = eeprom_write(handle, &eeprom, test_offset, write_data, test_size);
    if (ret < 0) {
        fprintf(stderr, "Failed to write test pattern\n");
        fx3_close(handle);
        libusb_exit(ctx);
        return 1;
    }
    printf("Write complete.\n");

    /* Step 3: Read back and verify */
    printf("\nStep 3: Reading back and verifying...\n");
    memset(read_data, 0, test_size);
    ret = eeprom_read(handle, &eeprom, test_offset, read_data, test_size);
    if (ret < 0) {
        fprintf(stderr, "Failed to read back data\n");
        fx3_close(handle);
        libusb_exit(ctx);
        return 1;
    }
    print_hex("Read    ", read_data, test_size);

    /* Compare */
    int errors = 0;
    for (size_t i = 0; i < test_size; i++) {
        if (read_data[i] != write_data[i]) {
            fprintf(stderr, "Mismatch at byte %zu: wrote 0x%02x, read 0x%02x\n",
                    i, write_data[i], read_data[i]);
            errors++;
        }
    }

    if (errors == 0) {
        printf("\n*** VERIFICATION PASSED ***\n");
    } else {
        printf("\n*** VERIFICATION FAILED (%d errors) ***\n", errors);
    }

    /* Step 4: Restore original data */
    printf("\nStep 4: Restoring original data...\n");
    ret = eeprom_write(handle, &eeprom, test_offset, original_data, test_size);
    if (ret < 0) {
        fprintf(stderr, "Warning: Failed to restore original data\n");
    } else {
        printf("Original data restored.\n");
    }

    /* Cleanup */
    fx3_close(handle);
    libusb_exit(ctx);

    return (errors == 0) ? 0 : 1;
}
