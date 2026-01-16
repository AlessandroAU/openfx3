/*
 * FX3 Device Module
 *
 * Reusable module for managing Cypress FX3 devices: discovery, firmware
 * upload, configuration, and acquisition control.
 */

#ifndef FX3_DEVICE_H
#define FX3_DEVICE_H

#include <stdint.h>
#include <libusb-1.0/libusb.h>

/* Device identifiers */
#define FX3_VID              0x04B4
#define FX3_PID_BOOTLOADER   0x00F3
#define FX3_PID_BOOTLOADER2  0x1234 // devices flashed with that special sigrok firmware that changes the bootloader PID
#define FX3_PID_FIRMWARE     0x1337

/* Helper macro to check if PID is a bootloader */
#define FX3_IS_BOOTLOADER_PID(pid) ((pid) == FX3_PID_BOOTLOADER || (pid) == FX3_PID_BOOTLOADER2)

/* Endpoints */
#define FX3_EP_BULK_IN       0x82

/* Vendor commands */
#define FX3_CMD_FW_UPLOAD    0xA0
#define FX3_CMD_GET_FW_VERSION 0xb0
#define FX3_CMD_START        0xb1
#define FX3_CMD_GET_FW_MODE  0xb2
#define FX3_CMD_STOP         0xb3
#define FX3_CMD_GET_DROP_STATS 0xb4
#define FX3_CMD_RESET_DROP_STATS 0xb5
#define FX3_CMD_GET_ACQ_STATUS 0xb6
#define FX3_CMD_REBOOT_BOOTLOADER 0xb7
#define FX3_CMD_START_BENCHMARK 0xb8

/*
 * Initialize an FX3 device, uploading firmware if necessary.
 *
 * This function:
 *   1. Opens the first FX3 device found (bootloader or firmware PID)
 *   2. If the device needs firmware (bootloader mode, force_reload, or wrong firmware):
 *      - Uploads the specified firmware (or embedded firmware if NULL)
 *      - Waits for re-enumeration
 *      - Re-opens the device
 *   3. Sets configuration and claims interface 0
 *
 * Parameters:
 *   ctx           - libusb context (must be initialized)
 *   handle_out    - Pointer to receive the opened device handle
 *   firmware_path - Path to firmware file, or NULL to use embedded firmware
 *   force_reload  - If non-zero, forces firmware upload even if already loaded
 *
 * Returns:
 *   0 on success
 *   -1 if no device found
 *   -2 if firmware upload failed
 *   -3 if device did not re-enumerate after firmware upload
 *   -4 if failed to claim interface
 */
int fx3_init(libusb_context *ctx, libusb_device_handle **handle_out,
             const char *firmware_path, int force_reload);

/*
 * Close an FX3 device and release resources.
 *
 * Parameters:
 *   handle - Device handle to close (can be NULL)
 */
void fx3_close(libusb_device_handle *handle);

/*
 * Upload firmware to an FX3 device in bootloader mode.
 *
 * This is a lower-level function - prefer fx3_init() for typical use.
 *
 * Parameters:
 *   handle        - Device handle (must be open)
 *   firmware_path - Path to the .fw firmware file
 *
 * Returns:
 *   0 on success
 *   -1 on failure (file not found, invalid format, upload error)
 */
int fx3_upload_firmware(libusb_device_handle *handle, const char *firmware_path);

/*
 * Open an FX3 device without uploading firmware.
 *
 * This is a lower-level function - prefer fx3_init() for typical use.
 *
 * Parameters:
 *   ctx           - libusb context
 *   handle_out    - Pointer to receive the opened device handle
 *   force_firmware - If non-zero, treats loaded firmware as needing reload
 *
 * Returns:
 *   0 if device is ready to use (has correct firmware)
 *   1 if device needs firmware upload
 *   -1 if no device found
 */
int fx3_open_device(libusb_context *ctx, libusb_device_handle **handle_out,
                    int force_firmware);

/*
 * Start acquisition with specified parameters.
 *
 * Parameters:
 *   handle        - Device handle
 *   bus_mhz       - GPIF bus clock rate in MHz (e.g., 20, 40, 65, 80, 100)
 *   bus_width     - Bus width in bits (8, 16, 24, or 32)
 *   internal_clock - If non-zero, use internal clock; otherwise external
 *
 * Returns:
 *   0 on success, -1 on failure
 */
int fx3_start_acquisition(libusb_device_handle *handle, int bus_mhz,
                          int bus_width, int internal_clock);

/*
 * Start USB benchmark mode.
 *
 * This starts the firmware benchmark mode which continuously sends
 * pre-filled DMA buffers to USB without GPIF involvement. Used for
 * measuring maximum USB throughput.
 *
 * Parameters:
 *   handle - Device handle
 *
 * Returns:
 *   0 on success, -1 on failure
 */
int fx3_start_benchmark(libusb_device_handle *handle);

/*
 * Stop acquisition (or benchmark).
 *
 * Parameters:
 *   handle - Device handle
 *
 * Returns:
 *   0 on success, -1 on failure
 */
int fx3_stop_acquisition(libusb_device_handle *handle);

/*
 * Get firmware version.
 *
 * Parameters:
 *   handle    - Device handle
 *   major_out - Pointer to receive major version (can be NULL)
 *   minor_out - Pointer to receive minor version (can be NULL)
 *
 * Returns:
 *   0 on success, -1 on failure
 */
int fx3_get_firmware_version(libusb_device_handle *handle,
                             uint8_t *major_out, uint8_t *minor_out);

/*
 * Drop statistics from firmware
 * Must match struct cmd_drop_stats in firmware command.h
 */
struct fx3_drop_stats {
    uint64_t total_buffers;      /* Buffers successfully committed */
    uint64_t total_bytes;        /* Bytes successfully committed */
    uint32_t stalls;             /* Backpressure events (DMA STALL, not data loss) */
    uint32_t gpif_errors;        /* GPIF/PIB errors (from PIB ISR) */
};

/*
 * Get drop statistics via vendor command (EP0).
 *
 * Parameters:
 *   handle    - Device handle
 *   stats_out - Pointer to receive drop statistics
 *
 * Returns:
 *   0 on success, -1 on failure
 */
int fx3_get_drop_stats(libusb_device_handle *handle,
                       struct fx3_drop_stats *stats_out);

/*
 * Reset drop statistics counters on the device.
 *
 * Resets all stats counters (total_buffers, overruns, stalls) to zero.
 * Useful for clearing startup transients before measuring.
 *
 * Parameters:
 *   handle - Device handle
 *
 * Returns:
 *   0 on success, -1 on failure
 */
int fx3_reset_drop_stats(libusb_device_handle *handle);

/*
 * Acquisition status information
 * Must match struct cmd_acq_status in firmware command.h
 */
struct fx3_acq_status {
    uint8_t active;        /* 1 if acquisition is running, 0 otherwise */
    uint8_t bus_width;     /* Configured bus width in bits */
    uint8_t pll_fbdiv;     /* PLL feedback divider used (20-27) */
    uint8_t reserved;      /* Reserved for future use */
    uint32_t bus_freq_hz;  /* Actual bus clock frequency in Hz (0 = external) */
    uint32_t sys_clk_hz;   /* System clock frequency in Hz */
    uint16_t gpif_div;     /* GPIF divider register value */
    uint16_t reserved2;    /* Padding for alignment */
};

/*
 * Get acquisition status including bus configuration.
 *
 * Parameters:
 *   handle - Device handle
 *   status - Output structure for status information
 *
 * Returns:
 *   0 on success, -1 on failure
 */
int fx3_get_acq_status(libusb_device_handle *handle, struct fx3_acq_status *status);

/*
 * Wait for acquisition to become active.
 *
 * Polls the device until acquisition is running or timeout expires.
 *
 * Parameters:
 *   handle     - Device handle
 *   timeout_ms - Maximum time to wait in milliseconds
 *
 * Returns:
 *   0 if acquisition is active, -1 on timeout or error
 */
int fx3_wait_acquisition_ready(libusb_device_handle *handle, int timeout_ms);

/*
 * Print USB endpoint configuration.
 *
 * Enumerates and prints all USB endpoints for the device, including:
 *   - Configuration details (value, number of interfaces, max power)
 *   - Interface details (class, subclass, protocol)
 *   - Endpoint details (address, direction, type, max packet size, interval)
 *   - Isochronous-specific info (sync type, usage type) if applicable
 *
 * Parameters:
 *   handle - Device handle
 *
 * Returns:
 *   0 on success, -1 on failure
 */
int fx3_print_endpoints(libusb_device_handle *handle);

/*
 * Reboot device into bootloader mode.
 *
 * Sends a vendor command that causes the FX3 to perform a hard reset,
 * returning to USB bootloader mode for firmware re-upload.
 *
 * After calling this, the device will disconnect and re-enumerate with
 * the bootloader PID (0x00F3). Wait ~100ms then call fx3_init() again
 * to upload new firmware.
 *
 * Parameters:
 *   handle - Device handle
 *
 * Returns:
 *   0 on success (command sent), -1 on failure
 *   Note: Success means the command was sent; the device will disconnect
 *   immediately so subsequent operations on this handle will fail.
 */
int fx3_reboot_bootloader(libusb_device_handle *handle);

#endif /* FX3_DEVICE_H */
