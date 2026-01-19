/*
 * FX3 Device Module
 *
 * Reusable module for managing Cypress FX3 devices: discovery, firmware
 * upload, configuration, and acquisition control.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include <openfx3/device.h>
#include <openfx3/platform.h>
#include "firmware.h"  /* Embedded RLE-compressed firmware */

//#define FX3_DEVICE_DEBUG

#define FX3_FW_CHUNK_SIZE 4096

#define GET_LSW(v)  ((uint16_t)((v) & 0xFFFF))
#define GET_MSW(v)  ((uint16_t)((v) >> 16))

/*
 * Decompress embedded firmware.
 *
 * Format (optimized for firmware with large zero regions):
 *   0x00, 0x00, LO, HI    -> Zero run of (LO | HI<<8) bytes
 *   0x00, 0x01, byte      -> Literal byte (escape)
 *   0x00, 0x02, count, byte -> Run of 'count' identical bytes
 *   Other                 -> Literal byte
 */
static int fx3_decompress(const uint8_t *src, size_t src_len,
                          uint8_t *dst, size_t dst_len) {
    size_t si = 0, di = 0;

    while (si < src_len && di < dst_len) {
        if (src[si] == 0x00) {
            if (si + 1 >= src_len) return -1;
            uint8_t cmd = src[si + 1];

            if (cmd == 0x00) {
                /* Zero run: 0x00, 0x00, LO, HI */
                if (si + 3 >= src_len) return -1;
                size_t count = src[si + 2] | ((size_t)src[si + 3] << 8);
                si += 4;
                while (count-- > 0 && di < dst_len) {
                    dst[di++] = 0x00;
                }
            } else if (cmd == 0x01) {
                /* Escaped literal: 0x00, 0x01, byte */
                if (si + 2 >= src_len) return -1;
                dst[di++] = src[si + 2];
                si += 3;
            } else if (cmd == 0x02) {
                /* Non-zero run: 0x00, 0x02, count, byte */
                if (si + 3 >= src_len) return -1;
                uint8_t count = src[si + 2];
                uint8_t byte = src[si + 3];
                si += 4;
                while (count-- > 0 && di < dst_len) {
                    dst[di++] = byte;
                }
            } else {
                return -1;  /* Unknown escape */
            }
        } else {
            /* Literal non-zero byte */
            dst[di++] = src[si++];
        }
    }

    return (int)di;
}

static int fx3_ram_write(libusb_device_handle *handle, uint8_t *buf,
                         uint32_t ram_addr, int len) {
    int index = 0;
    while (len > 0) {
        int size = (len < FX3_FW_CHUNK_SIZE) ? len : FX3_FW_CHUNK_SIZE;

        int ret = libusb_control_transfer(handle,
            LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT,
            FX3_CMD_FW_UPLOAD,
            GET_LSW(ram_addr), GET_MSW(ram_addr),
            &buf[index], size,
            1000);

        if (ret != size) {
            fprintf(stderr, "Firmware write failed at 0x%08X: %s\n",
                    ram_addr, libusb_error_name(ret));
            return -1;
        }

        ram_addr += size;
        index += size;
        len -= size;
    }
    return 0;
}

/*
 * Upload firmware from a memory buffer.
 * The buffer must contain valid FX3 firmware format.
 */
static int fx3_upload_firmware_buffer(libusb_device_handle *handle,
                                      uint8_t *firmware, long filesize,
                                      const char *source_name) {
    if (filesize < 8 || filesize > 512 * 1024) {
        fprintf(stderr, "Invalid firmware size: %ld\n", filesize);
        return -1;
    }

    if (firmware[0] != 'C' || firmware[1] != 'Y') {
        fprintf(stderr, "Invalid firmware magic (expected 'CY', got '%c%c')\n",
                firmware[0], firmware[1]);
        return -1;
    }

    printf("Uploading firmware '%s' (%ld bytes)...\n", source_name, filesize);

    uint32_t checksum = 0;
    int index = 4;

    while (index < filesize) {
        uint32_t *data_p = (uint32_t *)(firmware + index);
        uint32_t length = data_p[0];
        uint32_t address = data_p[1];

        if (length != 0) {
            for (uint32_t i = 0; i < length; i++) {
                checksum += data_p[2 + i];
            }

            int ret = fx3_ram_write(handle, firmware + index + 8, address, length * 4);
            if (ret != 0) {
                fprintf(stderr, "Failed to write firmware section at 0x%08X\n", address);
                return -1;
            }

            printf("  Section: addr=0x%08X, len=%u bytes\n", address, length * 4);
        } else {
            if (checksum != data_p[2]) {
                fprintf(stderr, "Firmware checksum mismatch (expected 0x%08X, got 0x%08X)\n",
                        data_p[2], checksum);
                return -1;
            }

            printf("Checksum OK, jumping to entry point 0x%08X\n", address);

            libusb_control_transfer(handle,
                LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT,
                FX3_CMD_FW_UPLOAD,
                GET_LSW(address), GET_MSW(address),
                NULL, 0,
                1000);
            break;
        }

        index += (8 + length * 4);
    }

    printf("Firmware upload complete\n");
    return 0;
}

/*
 * Upload embedded firmware (RLE compressed).
 */
static int fx3_upload_embedded_firmware(libusb_device_handle *handle) {
    /* Decompress firmware */
    uint8_t *firmware = (uint8_t *)malloc(FX3_FIRMWARE_ORIGINAL_SIZE);
    if (!firmware) {
        fprintf(stderr, "Failed to allocate memory for firmware decompression\n");
        return -1;
    }

    int decompressed_size = fx3_decompress(
        fx3_firmware_compressed, FX3_FIRMWARE_COMPRESSED_SIZE,
        firmware, FX3_FIRMWARE_ORIGINAL_SIZE);

    if (decompressed_size != FX3_FIRMWARE_ORIGINAL_SIZE) {
        fprintf(stderr, "Firmware decompression failed (got %d, expected %d)\n",
                decompressed_size, FX3_FIRMWARE_ORIGINAL_SIZE);
        free(firmware);
        return -1;
    }

    int ret = fx3_upload_firmware_buffer(handle, firmware, decompressed_size, "[embedded]");
    free(firmware);
    return ret;
}

int fx3_upload_firmware(libusb_device_handle *handle, const char *firmware_path) {
    /* Use embedded firmware if no path specified */
    if (!firmware_path) {
        return fx3_upload_embedded_firmware(handle);
    }

    FILE *fp = fopen(firmware_path, "rb");
    if (!fp) {
        fprintf(stderr, "Failed to open firmware file: %s\n", firmware_path);
        return -1;
    }

    fseek(fp, 0, SEEK_END);
    long filesize = ftell(fp);
    fseek(fp, 0, SEEK_SET);

    uint8_t *firmware = (uint8_t *)malloc(filesize);
    if (!firmware) {
        fprintf(stderr, "Failed to allocate memory for firmware\n");
        fclose(fp);
        return -1;
    }

    if (fread(firmware, 1, filesize, fp) != (size_t)filesize) {
        fprintf(stderr, "Failed to read firmware file\n");
        free(firmware);
        fclose(fp);
        return -1;
    }
    fclose(fp);

    int ret = fx3_upload_firmware_buffer(handle, firmware, filesize, firmware_path);
    free(firmware);
    return ret;
}

int fx3_open_device(libusb_context *ctx, libusb_device_handle **handle_out,
                    int force_firmware) {
    libusb_device **devlist;
    ssize_t count = libusb_get_device_list(ctx, &devlist);

    for (ssize_t i = 0; i < count; i++) {
        struct libusb_device_descriptor desc;
        if (libusb_get_device_descriptor(devlist[i], &desc) != 0)
            continue;

        if (desc.idVendor != FX3_VID)
            continue;

        if (!FX3_IS_BOOTLOADER_PID(desc.idProduct) && desc.idProduct != FX3_PID_FIRMWARE)
            continue;

        int ret = libusb_open(devlist[i], handle_out);
        if (ret != 0) {
            fprintf(stderr, "Failed to open device: %s\n", libusb_error_name(ret));
            continue;
        }

        char manufacturer[64] = {0};
        char product[64] = {0};
        if (desc.iManufacturer) {
            libusb_get_string_descriptor_ascii(*handle_out, desc.iManufacturer,
                (unsigned char *)manufacturer, sizeof(manufacturer));
        }
        if (desc.iProduct) {
            libusb_get_string_descriptor_ascii(*handle_out, desc.iProduct,
                (unsigned char *)product, sizeof(product));
        }

        printf("Opened FX3 device (VID=%04X PID=%04X)\n", desc.idVendor, desc.idProduct);
        printf("  Manufacturer: '%s'\n", manufacturer);
        printf("  Product: '%s'\n", product);

        int needs_firmware = 0;
        if (FX3_IS_BOOTLOADER_PID(desc.idProduct)) {
            printf("Device is in bootloader mode\n");
            needs_firmware = 1;
        } else if (force_firmware) {
            printf("Force firmware reload requested\n");
            needs_firmware = 1;
        } else if (strcmp(manufacturer, "OpenFX3") != 0 || strcmp(product, "OpenFX3") != 0) {
            printf("Device has unknown/stale firmware\n");
            needs_firmware = 1;
        }

        if (needs_firmware) {
            libusb_free_device_list(devlist, 1);
            return 1;
        }

        libusb_free_device_list(devlist, 1);
        return 0;
    }

    libusb_free_device_list(devlist, 1);
    fprintf(stderr, "No FX3 device found\n");
    return -1;
}

void fx3_close(libusb_device_handle *handle) {
    if (handle) {
        libusb_release_interface(handle, 0);
        libusb_close(handle);
    }
}

int fx3_init(libusb_context *ctx, libusb_device_handle **handle_out,
             const char *firmware_path, int force_reload) {
    /* firmware_path: NULL = use embedded firmware, non-NULL = load from file */
    int ret;

    *handle_out = NULL;

    ret = fx3_open_device(ctx, handle_out, force_reload);

    if (ret < 0) {
        return -1;
    }

    if (ret == 1) {
        printf("\n--- Uploading firmware ---\n");
        ret = fx3_upload_firmware(*handle_out, firmware_path);
        fx3_close(*handle_out);
        *handle_out = NULL;

        if (ret != 0) {
            fprintf(stderr, "Firmware upload failed\n");
            return -2;
        }

        printf("Waiting for device to re-enumerate...\n");
        for (int wait = 0; wait < 30; wait++) {
            sleep_ms(100);

            ret = fx3_open_device(ctx, handle_out, 0);
            if (ret == 0) {
                printf("Device ready after %d ms\n", (wait + 1) * 100);
                break;
            }
        }

        if (ret != 0) {
            fprintf(stderr, "Device did not re-enumerate\n");
            return -3;
        }
    }

    ret = libusb_set_configuration(*handle_out, 1);
    if (ret != 0 && ret != LIBUSB_ERROR_BUSY) {
        fprintf(stderr, "Warning: Failed to set configuration: %s\n", libusb_error_name(ret));
    }

    ret = libusb_claim_interface(*handle_out, 0);
    if (ret != 0) {
        fprintf(stderr, "Failed to claim interface: %s\n", libusb_error_name(ret));
        fx3_close(*handle_out);
        *handle_out = NULL;
        return -4;
    }

    printf("Interface claimed\n");

    /* Print endpoint configuration */
    fx3_print_endpoints(*handle_out);

    return 0;
}

int fx3_start_acquisition(libusb_device_handle *handle, int bus_mhz,
                          int bus_width, int internal_clock) {
    /* Command data: bus_mhz, bus_width, reserved */
    uint8_t cmd_data[3] = {
        (uint8_t)bus_mhz,
        (uint8_t)bus_width,
        0  /* reserved */
    };
    /* wValue bit 0: clock source (0=external, 1=internal) */
    uint16_t wValue = internal_clock ? 0x0001 : 0x0000;

    int ret = libusb_control_transfer(handle,
        LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT,
        FX3_CMD_START,
        wValue, 0x0000,
        cmd_data, sizeof(cmd_data),
        1000);

    return (ret < 0) ? -1 : 0;
}

int fx3_start_benchmark(libusb_device_handle *handle) {
    int ret = libusb_control_transfer(handle,
        LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT,
        FX3_CMD_START_BENCHMARK,
        0x0000, 0x0000,
        NULL, 0,
        1000);

    return (ret < 0) ? -1 : 0;
}

int fx3_stop_acquisition(libusb_device_handle *handle) {
    int ret = libusb_control_transfer(handle,
        LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT,
        FX3_CMD_STOP,
        0x0000, 0x0000,
        NULL, 0,
        1000);
    return (ret < 0) ? -1 : 0;
}

int fx3_get_firmware_version(libusb_device_handle *handle,
                             uint8_t *major_out, uint8_t *minor_out) {
    uint8_t version[2] = {0, 0};
    int ret = libusb_control_transfer(handle,
        LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN,
        FX3_CMD_GET_FW_VERSION,
        0x0000, 0x0000,
        version, sizeof(version),
        1000);

    if (ret < 0) {
        fprintf(stderr, "Failed to get firmware version: %s\n", libusb_error_name(ret));
        return -1;
    }

    if (major_out) *major_out = version[0];
    if (minor_out) *minor_out = version[1];

    return 0;
}

int fx3_get_drop_stats(libusb_device_handle *handle,
                       struct fx3_drop_stats *stats_out) {
    uint8_t buf[24] = {0};  /* 2 x uint64_t + 2 x uint32_t */
    int ret = libusb_control_transfer(handle,
        LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN,
        FX3_CMD_GET_DROP_STATS,
        0x0000, 0x0000,
        buf, sizeof(buf),
        1000);

    if (ret < 0) {
#ifdef FX3_DEVICE_DEBUG
        fprintf(stderr, "[DEBUG] fx3_get_drop_stats failed: %s (ret=%d)\n",
                libusb_error_name(ret), ret);
#endif
        return -1;
    }

    if (stats_out) {
        /* Little-endian decode */
        stats_out->total_buffers = (uint64_t)buf[0] | ((uint64_t)buf[1] << 8) |
                                   ((uint64_t)buf[2] << 16) | ((uint64_t)buf[3] << 24) |
                                   ((uint64_t)buf[4] << 32) | ((uint64_t)buf[5] << 40) |
                                   ((uint64_t)buf[6] << 48) | ((uint64_t)buf[7] << 56);
        stats_out->total_bytes = (uint64_t)buf[8] | ((uint64_t)buf[9] << 8) |
                                 ((uint64_t)buf[10] << 16) | ((uint64_t)buf[11] << 24) |
                                 ((uint64_t)buf[12] << 32) | ((uint64_t)buf[13] << 40) |
                                 ((uint64_t)buf[14] << 48) | ((uint64_t)buf[15] << 56);
        stats_out->stalls = buf[16] | (buf[17] << 8) | (buf[18] << 16) | (buf[19] << 24);
        stats_out->gpif_errors = buf[20] | (buf[21] << 8) | (buf[22] << 16) | (buf[23] << 24);
    }

    return 0;
}

int fx3_reset_drop_stats(libusb_device_handle *handle) {
    int ret = libusb_control_transfer(handle,
        LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT,
        FX3_CMD_RESET_DROP_STATS,
        0x0000, 0x0000,
        NULL, 0,
        1000);

    return (ret < 0) ? -1 : 0;
}

int fx3_get_acq_status(libusb_device_handle *handle, struct fx3_acq_status *status) {
    /* Wire format matches firmware struct cmd_acq_status (16 bytes) */
    uint8_t buf[16];
    int ret = libusb_control_transfer(handle,
        LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN,
        FX3_CMD_GET_ACQ_STATUS,
        0x0000, 0x0000,
        buf, sizeof(buf),
        1000);

    if (ret < 0) {
        return -1;
    }

    status->active = buf[0];
    status->bus_width = buf[1];
    status->pll_fbdiv = buf[2];
    status->reserved = buf[3];
    status->bus_freq_hz = buf[4] | (buf[5] << 8) | (buf[6] << 16) | (buf[7] << 24);
    status->sys_clk_hz = buf[8] | (buf[9] << 8) | (buf[10] << 16) | (buf[11] << 24);
    status->gpif_div = buf[12] | (buf[13] << 8);
    status->reserved2 = buf[14] | (buf[15] << 8);

    return 0;
}

int fx3_wait_acquisition_ready(libusb_device_handle *handle, int timeout_ms) {
    const int poll_interval_ms = 1;  /* Poll every 1ms */
    platform_timer_t timer;
    struct fx3_acq_status status;

    platform_timer_start(&timer);

    /* Check immediately first, then loop with sleep until timeout */
    do {
        if (fx3_get_acq_status(handle, &status) < 0) {
            return -1;  /* Communication error */
        }
        if (status.active) {
#ifdef FX3_DEVICE_DEBUG
            printf("\n[DEBUG] Acquisition ready after %.2f ms\n", platform_timer_elapsed_ms(&timer));
#endif
            return 0;   /* Acquisition is active */
        }
        /* Sleep and retry */
        sleep_ms(poll_interval_ms);
    } while (platform_timer_elapsed_ms(&timer) < timeout_ms);

#ifdef FX3_DEVICE_DEBUG
    printf("\n[DEBUG] Acquisition timeout after %.2f ms\n", platform_timer_elapsed_ms(&timer));
#endif
    return -1;  /* Timeout */
}

int fx3_reboot_bootloader(libusb_device_handle *handle) {
    int ret = libusb_control_transfer(handle,
        LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT,
        FX3_CMD_REBOOT_BOOTLOADER,
        0x0000, 0x0000,
        NULL, 0,
        1000);

    /* Note: Device will reset immediately, so we may get a timeout or
     * pipe error - that's expected. Return success if command was sent. */
    return (ret < 0 && ret != LIBUSB_ERROR_TIMEOUT && ret != LIBUSB_ERROR_PIPE &&
            ret != LIBUSB_ERROR_NO_DEVICE && ret != LIBUSB_ERROR_IO) ? -1 : 0;
}

static const char *endpoint_type_str(uint8_t type) {
    switch (type & 0x03) {
        case LIBUSB_ENDPOINT_TRANSFER_TYPE_CONTROL:     return "Control";
        case LIBUSB_ENDPOINT_TRANSFER_TYPE_ISOCHRONOUS: return "Isochronous";
        case LIBUSB_ENDPOINT_TRANSFER_TYPE_BULK:        return "Bulk";
        case LIBUSB_ENDPOINT_TRANSFER_TYPE_INTERRUPT:   return "Interrupt";
        default: return "Unknown";
    }
}

static const char *endpoint_dir_str(uint8_t addr) {
    return (addr & LIBUSB_ENDPOINT_IN) ? "IN" : "OUT";
}

int fx3_print_endpoints(libusb_device_handle *handle) {
    libusb_device *dev = libusb_get_device(handle);
    if (!dev) {
        fprintf(stderr, "Failed to get device from handle\n");
        return -1;
    }

    struct libusb_config_descriptor *config;
    int ret = libusb_get_active_config_descriptor(dev, &config);
    if (ret != 0) {
        fprintf(stderr, "Failed to get config descriptor: %s\n", libusb_error_name(ret));
        return -1;
    }

    /* Get device speed to determine MaxPower units */
    int speed = libusb_get_device_speed(dev);
    const char *speed_str;
    int power_multiplier;
    switch (speed) {
        case LIBUSB_SPEED_SUPER_PLUS:
            speed_str = "SuperSpeed Plus (USB 3.1)";
            power_multiplier = 8;
            break;
        case LIBUSB_SPEED_SUPER:
            speed_str = "SuperSpeed (USB 3.0)";
            power_multiplier = 8;
            break;
        case LIBUSB_SPEED_HIGH:
            speed_str = "High Speed (USB 2.0)";
            power_multiplier = 2;
            break;
        case LIBUSB_SPEED_FULL:
            speed_str = "Full Speed (USB 1.1)";
            power_multiplier = 2;
            break;
        case LIBUSB_SPEED_LOW:
            speed_str = "Low Speed (USB 1.0)";
            power_multiplier = 2;
            break;
        default:
            speed_str = "Unknown";
            power_multiplier = 2;
            break;
    }

    printf("\n=== USB Configuration ===\n");
    printf("Device speed: %s\n", speed_str);
    printf("Configuration value: %d\n", config->bConfigurationValue);
    printf("Number of interfaces: %d\n", config->bNumInterfaces);
    printf("Max power: %d mA\n", config->MaxPower * power_multiplier);

    for (int i = 0; i < config->bNumInterfaces; i++) {
        const struct libusb_interface *iface = &config->interface[i];

        for (int j = 0; j < iface->num_altsetting; j++) {
            const struct libusb_interface_descriptor *altsetting = &iface->altsetting[j];

            printf("\n--- Interface %d (alt %d) ---\n",
                   altsetting->bInterfaceNumber, altsetting->bAlternateSetting);
            printf("  Class: 0x%02X, SubClass: 0x%02X, Protocol: 0x%02X\n",
                   altsetting->bInterfaceClass,
                   altsetting->bInterfaceSubClass,
                   altsetting->bInterfaceProtocol);
            printf("  Number of endpoints: %d\n", altsetting->bNumEndpoints);

            for (int k = 0; k < altsetting->bNumEndpoints; k++) {
                const struct libusb_endpoint_descriptor *ep = &altsetting->endpoint[k];

                printf("\n  Endpoint 0x%02X:\n", ep->bEndpointAddress);
                printf("    Address: %d %s\n",
                       ep->bEndpointAddress & 0x0F,
                       endpoint_dir_str(ep->bEndpointAddress));
                printf("    Type: %s\n", endpoint_type_str(ep->bmAttributes));
                printf("    Max packet size: %d bytes\n", ep->wMaxPacketSize);
                printf("    Interval: %d\n", ep->bInterval);

                /* For isochronous endpoints, decode additional info */
                if ((ep->bmAttributes & 0x03) == LIBUSB_ENDPOINT_TRANSFER_TYPE_ISOCHRONOUS) {
                    int sync_type = (ep->bmAttributes >> 2) & 0x03;
                    int usage_type = (ep->bmAttributes >> 4) & 0x03;
                    const char *sync_str[] = {"No Sync", "Async", "Adaptive", "Sync"};
                    const char *usage_str[] = {"Data", "Feedback", "Implicit FB Data", "Reserved"};
                    printf("    Sync type: %s\n", sync_str[sync_type]);
                    printf("    Usage type: %s\n", usage_str[usage_type]);
                }
            }
        }
    }

    libusb_free_config_descriptor(config);
    printf("\n");
    return 0;
}

