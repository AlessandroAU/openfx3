/*
 * Counter Validation
 *
 * Shared counter validation logic for streaming test applications.
 */

#include "counter_validation.h"

void cv_init(cv_state_t *state, int bus_width, int dma_buf_size) {
    /* Set counter width and mask based on bus width */
    switch (bus_width) {
        case 8:  state->counter_mask = 0xFF;       state->counter_bytes = 1; break;
        case 16: state->counter_mask = 0xFFFF;     state->counter_bytes = 2; break;
        case 24: state->counter_mask = 0xFFFFFF;   state->counter_bytes = 3; break;
        default: state->counter_mask = 0xFFFFFFFF; state->counter_bytes = 4; break;
    }

    state->dma_buffer_size = dma_buf_size;

    /* Initialize state */
    cv_reset(state);
}

void cv_reset(cv_state_t *state) {
    /* Reset hot-path state (non-atomic) */
    state->expected_counter = 0;
    state->counter_initialized = false;

    /* Reset atomic statistics */
    atomic_store(&state->counter_errors, 0);
    atomic_store(&state->total_counters_checked, 0);
    atomic_store(&state->errors_logged, 0);
}

void cv_validate_full(cv_state_t *state, const uint8_t *data, int length, unsigned transfer_num) {
    /* Load hot-path state into locals for fast access */
    uint32_t expected = state->expected_counter;
    bool initialized = state->counter_initialized;
    const uint32_t mask = state->counter_mask;
    const int counter_bytes = state->counter_bytes;

    /* Local counters - updated atomically once at end */
    unsigned local_errors = 0;

    /* Fast path for 32-bit counters (most common case) */
    if (counter_bytes == 4) {
        const uint32_t *p = (const uint32_t *)data;
        const uint32_t *end = (const uint32_t *)(data + (length & ~3));

        /* Sync to first counter value without counting as error */
        if (!initialized && p < end) {
            expected = *p++ + 1;
            initialized = true;
        }

        while (p < end) {
            uint32_t counter = *p;
            if (counter != expected) {
                local_errors++;
                if (atomic_load(&state->errors_logged) < CV_ERROR_LOG_MAX) {
                    unsigned logged = atomic_fetch_add(&state->errors_logged, 1);
                    if (logged < CV_ERROR_LOG_MAX) {
                        state->error_log[logged].transfer_num = transfer_num;
                        state->error_log[logged].offset = (int)((const uint8_t *)p - data);
                        state->error_log[logged].expected = expected;
                        state->error_log[logged].actual = counter;
                    }
                }
                expected = counter + 1;
            } else {
                expected++;
            }
            p++;
        }

        state->expected_counter = expected;
        state->counter_initialized = initialized;
        atomic_fetch_add(&state->total_counters_checked, (unsigned long long)(end - (const uint32_t *)data));
        if (local_errors > 0) {
            atomic_fetch_add(&state->counter_errors, local_errors);
        }
        return;
    }

    /* Generic path for other widths (8, 16, 24-bit) */
    int offset = 0;
    unsigned long long local_checked = 0;

    while (offset + counter_bytes <= length) {
        /* Read counter value based on width (little-endian) */
        uint32_t counter = 0;
        for (int b = 0; b < counter_bytes; b++) {
            counter |= (uint32_t)data[offset + b] << (b * 8);
        }

        local_checked++;

        /* Sync to first counter value without counting as error */
        if (!initialized) {
            expected = (counter + 1) & mask;
            initialized = true;
            offset += counter_bytes;
            continue;
        }

        uint32_t expected_masked = expected & mask;
        if (counter != expected_masked) {
            local_errors++;
            unsigned logged = atomic_load(&state->errors_logged);
            if (logged < CV_ERROR_LOG_MAX) {
                state->error_log[logged].transfer_num = transfer_num;
                state->error_log[logged].offset = offset;
                state->error_log[logged].expected = expected_masked;
                state->error_log[logged].actual = counter;
                atomic_fetch_add(&state->errors_logged, 1);
            }
            expected = (counter + 1) & mask;
        } else {
            expected = (expected + 1) & mask;
        }

        offset += counter_bytes;
    }

    state->expected_counter = expected;
    state->counter_initialized = initialized;
    atomic_fetch_add(&state->total_counters_checked, local_checked);
    if (local_errors > 0) {
        atomic_fetch_add(&state->counter_errors, local_errors);
    }
}

void cv_validate_headers(cv_state_t *state, const uint8_t *data, int length, unsigned transfer_num) {
    /* Load hot-path state into locals for fast access */
    uint32_t expected = state->expected_counter;
    bool initialized = state->counter_initialized;
    const uint32_t mask = state->counter_mask;
    const int counter_bytes = state->counter_bytes;
    const int dma_buffer_size = state->dma_buffer_size;

    /* Calculate counters per DMA buffer based on width */
    int counters_per_buffer = dma_buffer_size / counter_bytes;

    /* Local counters - updated atomically once at end */
    unsigned local_errors = 0;
    unsigned long long local_checked = 0;

    /* Iterate through each DMA buffer in the transfer */
    for (int offset = 0; offset + counter_bytes <= length; offset += dma_buffer_size) {
        /* Read counter value based on width (little-endian) */
        uint32_t counter = 0;
        for (int b = 0; b < counter_bytes; b++) {
            counter |= (uint32_t)data[offset + b] << (b * 8);
        }

        local_checked++;

        /* Sync to first counter value without counting as error */
        if (!initialized) {
            expected = (counter + counters_per_buffer) & mask;
            initialized = true;
            continue;
        }

        uint32_t expected_masked = expected & mask;
        if (counter != expected_masked) {
            local_errors++;
            unsigned logged = atomic_load(&state->errors_logged);
            if (logged < CV_ERROR_LOG_MAX) {
                state->error_log[logged].transfer_num = transfer_num;
                state->error_log[logged].offset = offset;
                state->error_log[logged].expected = expected_masked;
                state->error_log[logged].actual = counter;
                atomic_fetch_add(&state->errors_logged, 1);
            }
            /* Resync and skip to next buffer's expected value */
            expected = (counter + counters_per_buffer) & mask;
        } else {
            expected = (expected + counters_per_buffer) & mask;
        }
    }

    /* Write back hot-path state */
    state->expected_counter = expected;
    state->counter_initialized = initialized;

    /* Update atomic statistics once per transfer */
    if (local_errors > 0) {
        atomic_fetch_add(&state->counter_errors, local_errors);
    }
    atomic_fetch_add(&state->total_counters_checked, local_checked);
}
