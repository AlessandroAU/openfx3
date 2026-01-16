/*
 * FX3 Acquisition Session API
 *
 * High-level API for managing USB bulk/isochronous data acquisition with
 * automatic transfer management, internal event thread, and callback-based
 * data delivery.
 */

#ifndef FX3_ACQUISITION_H
#define FX3_ACQUISITION_H

#include <stdint.h>
#include <libusb-1.0/libusb.h>

/* Opaque session handle */
typedef struct fx3_acquisition_session fx3_acquisition_session_t;

/*
 * Data acquisition callback - invoked with each chunk of data.
 * Called from libusb event context - keep processing minimal.
 *
 * Parameters:
 *   data      - Pointer to received data
 *   length    - Number of bytes in this chunk
 *   user_data - User-provided context pointer
 *
 * Returns:
 *   0 to continue acquisition
 *   Non-zero to stop acquisition
 */
typedef int (*fx3_acquisition_callback_t)(
    const uint8_t *data,
    int length,
    void *user_data
);

/*
 * Acquisition session statistics
 */
struct fx3_acquisition_stats {
    unsigned long long total_bytes;   /* Total bytes received */
    unsigned total_transfers;         /* Completed USB transfers */
    unsigned usb_errors;              /* USB transfer errors */
};

/*
 * Create and start an acquisition session.
 *
 * Allocates USB transfers and buffers, submits them for async I/O,
 * and starts an internal thread to process USB events.
 * The device should already have acquisition started via fx3_start_acquisition().
 *
 * Parameters:
 *   handle              - Device handle (acquisition should already be started)
 *   ctx                 - libusb context
 *   transfer_size       - Bytes per USB transfer (e.g., 1048576 for 1MB)
 *   num_transfers       - Number of transfers in flight (e.g., 8)
 *   timeout_ms          - USB transfer timeout in milliseconds
 *   chunk_size          - Bytes per callback invocation (0 = whole transfer)
 *   skip_initial_bytes  - Skip this many bytes before invoking callback (for DMA pool)
 *   callback            - User callback function
 *   user_data           - Passed to callback
 *   session_out         - Receives session handle
 *
 * Returns:
 *   0 on success, -1 on failure
 */
int fx3_acquisition_create(
    libusb_device_handle *handle,
    libusb_context *ctx,
    int transfer_size,
    int num_transfers,
    int timeout_ms,
    int chunk_size,
    unsigned long long skip_initial_bytes,
    fx3_acquisition_callback_t callback,
    void *user_data,
    fx3_acquisition_session_t **session_out
);

/*
 * Process USB events and invoke callbacks.
 *
 * Note: With the internal event thread, this function is typically not
 * needed. It's provided for compatibility and manual event handling if
 * the internal thread is disabled.
 *
 * Call this in a loop until it returns non-zero.
 *
 * Parameters:
 *   session    - Session handle
 *   timeout_ms - libusb event handling timeout
 *
 * Returns:
 *   0  - Still running, call again
 *   1  - Stopped (callback returned non-zero or no active transfers)
 *   -1 - libusb error
 */
int fx3_acquisition_process(fx3_acquisition_session_t *session, int timeout_ms);

/*
 * Request acquisition stop.
 *
 * Signals the session to stop accepting new data. Active transfers will
 * complete or be cancelled. The internal event thread will drain remaining
 * events before exiting.
 *
 * Parameters:
 *   session - Session handle
 */
void fx3_acquisition_stop(fx3_acquisition_session_t *session);

/*
 * Stop acquisition and free all resources.
 *
 * Cancels any pending transfers, waits for the internal event thread to
 * finish draining, and frees all memory.
 * Safe to call with NULL.
 *
 * Parameters:
 *   session - Session handle (can be NULL)
 */
void fx3_acquisition_destroy(fx3_acquisition_session_t *session);

/*
 * Get session statistics.
 *
 * Thread-safe - can be called while acquisition is running.
 *
 * Parameters:
 *   session   - Session handle
 *   stats_out - Pointer to receive statistics
 *
 * Returns:
 *   0 on success, -1 on failure
 */
int fx3_acquisition_get_stats(fx3_acquisition_session_t *session,
                              struct fx3_acquisition_stats *stats_out);

/*
 * Check if acquisition is still running.
 *
 * Thread-safe. Returns true if the internal event thread is active
 * and transfers are in flight.
 *
 * Parameters:
 *   session - Session handle
 *
 * Returns:
 *   Non-zero if running, 0 if stopped
 */
int fx3_acquisition_is_running(fx3_acquisition_session_t *session);

/*
 * Wait for acquisition to complete.
 *
 * Blocks until the acquisition session stops (callback returns non-zero,
 * all transfers complete, or error occurs).
 *
 * Parameters:
 *   session - Session handle
 */
void fx3_acquisition_wait(fx3_acquisition_session_t *session);

#endif /* FX3_ACQUISITION_H */
