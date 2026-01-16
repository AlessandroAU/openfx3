/*
 * FX3 Acquisition Session Implementation
 *
 * Manages USB async transfers with automatic event handling via internal thread.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdatomic.h>

#include <openfx3/acquisition.h>
#include <openfx3/device.h>
#include <openfx3/platform.h>

/*
 * Internal session structure
 */
struct fx3_acquisition_session {
    libusb_device_handle *handle;
    libusb_context *ctx;
    struct libusb_transfer **transfers;
    uint8_t **buffers;
    int num_transfers;
    int transfer_size;
    int timeout_ms;
    int chunk_size;
    unsigned long long skip_initial_bytes;

    fx3_acquisition_callback_t callback;
    void *user_data;

    atomic_int active_transfers;
    atomic_bool running;
    atomic_bool stop_requested;
    atomic_ullong total_bytes;
    atomic_ullong bytes_skipped;
    atomic_ullong bytes_seen;
    atomic_uint transfer_count;
    atomic_uint usb_errors;

    /* Internal event thread */
    platform_thread_t event_thread;
    atomic_bool thread_running;
};

/*
 * Internal event thread function.
 * Continuously processes libusb events until stopped.
 */
static void *fx3_event_thread_func(void *arg) {
    fx3_acquisition_session_t *session = (fx3_acquisition_session_t *)arg;
    struct timeval tv;

    /* Keep running while:
     * 1. thread_running is true (normal operation), OR
     * 2. there are still active transfers (draining cancelled transfers)
     */
    while (atomic_load(&session->thread_running) ||
           atomic_load(&session->active_transfers) > 0) {
        tv.tv_sec = 0;
        tv.tv_usec = 50000;  /* 50ms timeout for responsive shutdown */

        int completed = 0;
        int ret = libusb_handle_events_timeout_completed(session->ctx, &tv, &completed);

        if (ret < 0 && ret != LIBUSB_ERROR_TIMEOUT && ret != LIBUSB_ERROR_INTERRUPTED) {
            /* Serious error - stop the thread */
            atomic_store(&session->running, false);
            break;
        }
    }

    return NULL;
}

static void LIBUSB_CALL fx3_internal_transfer_callback(struct libusb_transfer *transfer) {
    fx3_acquisition_session_t *session = (fx3_acquisition_session_t *)transfer->user_data;

    switch (transfer->status) {
        case LIBUSB_TRANSFER_COMPLETED: {
            int actual_length = transfer->actual_length;

            if (actual_length > 0 && atomic_load(&session->running)) {
                uint8_t *data = transfer->buffer;
                int data_offset = 0;
                int data_length = actual_length;
                unsigned long long skipped_this = 0;

                /* Track total bytes seen (for skip window alignment) */
                unsigned long long prev_seen = atomic_fetch_add(&session->bytes_seen, (unsigned long long)actual_length);

                /* Skip initial bytes window (e.g., DMA pool) */
                if (session->skip_initial_bytes > 0 && prev_seen < session->skip_initial_bytes) {
                    unsigned long long skip_remaining = session->skip_initial_bytes - prev_seen;
                    if (skip_remaining >= (unsigned long long)data_length) {
                        skipped_this += (unsigned long long)data_length;
                        atomic_fetch_add(&session->bytes_skipped, skipped_this);
                        atomic_fetch_add(&session->transfer_count, 1);
                        goto resubmit_or_exit;
                    } else {
                        data_offset += (int)skip_remaining;
                        data_length -= (int)skip_remaining;
                        skipped_this += skip_remaining;
                    }
                }

                /* Invoke user callback with non-skipped data */
                if (data_length > 0) {
                    data = transfer->buffer + data_offset;
                    int remaining = data_length;
                    int chunk = session->chunk_size > 0 ? session->chunk_size : remaining;

                    while (remaining > 0 && atomic_load(&session->running)) {
                        int len = (remaining < chunk) ? remaining : chunk;
                        if (session->callback(data, len, session->user_data) != 0) {
                            atomic_store(&session->stop_requested, true);
                            atomic_store(&session->running, false);
                            break;
                        }
                        data += len;
                        remaining -= len;
                    }
                }

                /* Account for skipped and delivered bytes */
                atomic_fetch_add(&session->bytes_skipped, skipped_this);
                atomic_fetch_add(&session->total_bytes, (unsigned long long)data_length);
                atomic_fetch_add(&session->transfer_count, 1);
            }

resubmit_or_exit:
            /* Resubmit if still running */
            if (atomic_load(&session->running)) {
                int r = libusb_submit_transfer(transfer);
                if (r < 0) {
                    atomic_fetch_add(&session->usb_errors, 1);
                    atomic_fetch_sub(&session->active_transfers, 1);
                }
            } else {
                atomic_fetch_sub(&session->active_transfers, 1);
            }
            break;
        }

        case LIBUSB_TRANSFER_CANCELLED:
            atomic_fetch_sub(&session->active_transfers, 1);
            break;

        case LIBUSB_TRANSFER_TIMED_OUT:
            /* Retry on timeout */
            if (atomic_load(&session->running)) {
                int r = libusb_submit_transfer(transfer);
                if (r < 0) {
                    atomic_fetch_add(&session->usb_errors, 1);
                    atomic_fetch_sub(&session->active_transfers, 1);
                }
            } else {
                atomic_fetch_sub(&session->active_transfers, 1);
            }
            break;

        default:
            atomic_fetch_add(&session->usb_errors, 1);
            atomic_fetch_sub(&session->active_transfers, 1);
            break;
    }
}

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
    fx3_acquisition_session_t **session_out)
{
    if (!handle || !ctx || !callback || !session_out || num_transfers <= 0 || transfer_size <= 0) {
        return -1;
    }

    fx3_acquisition_session_t *session = (fx3_acquisition_session_t *)calloc(1, sizeof(*session));
    if (!session) {
        return -1;
    }

    session->handle = handle;
    session->ctx = ctx;
    session->transfer_size = transfer_size;
    session->num_transfers = num_transfers;
    session->timeout_ms = timeout_ms;
    session->chunk_size = chunk_size;
    session->skip_initial_bytes = skip_initial_bytes;
    session->callback = callback;
    session->user_data = user_data;

    atomic_store(&session->active_transfers, 0);
    atomic_store(&session->running, false);
    atomic_store(&session->stop_requested, false);
    atomic_store(&session->total_bytes, 0);
    atomic_store(&session->bytes_skipped, 0);
    atomic_store(&session->bytes_seen, 0);
    atomic_store(&session->transfer_count, 0);
    atomic_store(&session->usb_errors, 0);
    atomic_store(&session->thread_running, false);

    /* Allocate transfer and buffer arrays */
    session->transfers = (struct libusb_transfer **)calloc(num_transfers, sizeof(struct libusb_transfer *));
    session->buffers = (uint8_t **)calloc(num_transfers, sizeof(uint8_t *));
    if (!session->transfers || !session->buffers) {
        free(session->transfers);
        free(session->buffers);
        free(session);
        return -1;
    }

    /* Allocate individual transfers and buffers */
    for (int i = 0; i < num_transfers; i++) {
        session->buffers[i] = (uint8_t *)platform_aligned_alloc(transfer_size, 4096);
        if (!session->buffers[i]) {
            goto cleanup_error;
        }

        session->transfers[i] = libusb_alloc_transfer(0);
        if (!session->transfers[i]) {
            goto cleanup_error;
        }

        libusb_fill_bulk_transfer(session->transfers[i],
                                  handle,
                                  FX3_EP_BULK_IN,
                                  session->buffers[i],
                                  transfer_size,
                                  fx3_internal_transfer_callback,
                                  session,
                                  timeout_ms);
    }

    /* Submit all transfers */
    atomic_store(&session->running, true);
    for (int i = 0; i < num_transfers; i++) {
        int ret = libusb_submit_transfer(session->transfers[i]);
        if (ret < 0) {
            fprintf(stderr, "Failed to submit transfer %d: %s\n", i, libusb_error_name(ret));
            atomic_store(&session->running, false);
            goto cleanup_error;
        }
        atomic_fetch_add(&session->active_transfers, 1);
    }

    /* Start internal event thread */
    atomic_store(&session->thread_running, true);
    if (platform_thread_create(&session->event_thread, fx3_event_thread_func, session) != 0) {
        fprintf(stderr, "Failed to create event thread\n");
        atomic_store(&session->running, false);
        atomic_store(&session->thread_running, false);
        goto cleanup_error;
    }

    fx3_reset_drop_stats(handle);

    *session_out = session;
    return 0;

cleanup_error:
    /* Cancel any submitted transfers */
    if (atomic_load(&session->active_transfers) > 0) {
        for (int i = 0; i < num_transfers; i++) {
            if (session->transfers[i]) {
                libusb_cancel_transfer(session->transfers[i]);
            }
        }
        /* Drain events to complete cancellations */
        struct timeval tv = {0, 50000};
        int completed = 0;
        while (atomic_load(&session->active_transfers) > 0) {
            libusb_handle_events_timeout_completed(ctx, &tv, &completed);
        }
    }

    /* Free resources */
    for (int i = 0; i < num_transfers; i++) {
        if (session->transfers[i]) {
            libusb_free_transfer(session->transfers[i]);
        }
        if (session->buffers[i]) {
            platform_aligned_free(session->buffers[i]);
        }
    }
    free(session->transfers);
    free(session->buffers);
    free(session);
    return -1;
}

int fx3_acquisition_process(fx3_acquisition_session_t *session, int timeout_ms) {
    if (!session) {
        return -1;
    }

    /* Check if we should stop */
    if (!atomic_load(&session->running) || atomic_load(&session->active_transfers) <= 0) {
        return 1;  /* Stopped */
    }

    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    int completed = 0;
    int ret = libusb_handle_events_timeout_completed(session->ctx, &tv, &completed);

    if (ret < 0 && ret != LIBUSB_ERROR_TIMEOUT && ret != LIBUSB_ERROR_INTERRUPTED) {
        return -1;  /* libusb error */
    }

    /* Check again after processing events */
    if (!atomic_load(&session->running) || atomic_load(&session->active_transfers) <= 0) {
        return 1;  /* Stopped */
    }

    return 0;  /* Still running */
}

void fx3_acquisition_stop(fx3_acquisition_session_t *session) {
    if (!session) {
        return;
    }
    atomic_store(&session->running, false);
    atomic_store(&session->thread_running, false);
}

void fx3_acquisition_destroy(fx3_acquisition_session_t *session) {
    if (!session) {
        return;
    }

    /* Stop accepting new data and signal thread to exit */
    atomic_store(&session->running, false);
    atomic_store(&session->thread_running, false);

    /* Cancel all pending transfers */
    for (int i = 0; i < session->num_transfers; i++) {
        if (session->transfers[i]) {
            libusb_cancel_transfer(session->transfers[i]);
        }
    }

    /* Wait for event thread to finish (it will drain remaining events) */
    platform_thread_join(session->event_thread);

    /* Free transfers and buffers */
    for (int i = 0; i < session->num_transfers; i++) {
        if (session->transfers[i]) {
            libusb_free_transfer(session->transfers[i]);
        }
        if (session->buffers[i]) {
            platform_aligned_free(session->buffers[i]);
        }
    }

    free(session->transfers);
    free(session->buffers);
    free(session);
}

int fx3_acquisition_get_stats(fx3_acquisition_session_t *session,
                              struct fx3_acquisition_stats *stats_out) {
    if (!session || !stats_out) {
        return -1;
    }

    stats_out->total_bytes = atomic_load(&session->total_bytes);
    stats_out->total_transfers = atomic_load(&session->transfer_count);
    stats_out->usb_errors = atomic_load(&session->usb_errors);

    return 0;
}

int fx3_acquisition_is_running(fx3_acquisition_session_t *session) {
    if (!session) {
        return 0;
    }
    return atomic_load(&session->running) && atomic_load(&session->active_transfers) > 0;
}

void fx3_acquisition_wait(fx3_acquisition_session_t *session) {
    if (!session) {
        return;
    }
    /* Poll until acquisition stops - thread will be joined in destroy() */
    while (fx3_acquisition_is_running(session)) {
        sleep_ms(10);
    }
}
