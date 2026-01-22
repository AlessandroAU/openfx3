/*
 * Counter Validation
 *
 * Shared counter validation logic for streaming test applications.
 * Validates sequential counters in data streams with configurable width.
 */

#ifndef COUNTER_VALIDATION_H
#define COUNTER_VALIDATION_H

#include <stdint.h>
#include <stdbool.h>
#include <stdatomic.h>

//-----------------------------------------------------------------------------
// Configuration
//-----------------------------------------------------------------------------

#define CV_ERROR_LOG_MAX 16  /* Max errors to log details for */

//-----------------------------------------------------------------------------
// Types
//-----------------------------------------------------------------------------

typedef struct {
    unsigned transfer_num;
    int offset;
    uint32_t expected;
    uint32_t actual;
} cv_error_log_entry_t;

typedef struct {
    /* Configuration (set before use) */
    int counter_bytes;          /* Counter width in bytes (1, 2, 3, or 4) */
    uint32_t counter_mask;      /* Mask based on counter width */
    int dma_buffer_size;        /* DMA buffer size for header-only validation */

    /* Hot-path state (non-atomic - only accessed from callback thread) */
    uint32_t expected_counter;      /* Next expected counter value */
    bool counter_initialized;       /* True after first counter seen */

    /* Statistics (atomic for cross-thread reads from main thread) */
    atomic_uint counter_errors;
    atomic_ullong total_counters_checked;

    /* Error log */
    cv_error_log_entry_t error_log[CV_ERROR_LOG_MAX];
    atomic_uint errors_logged;
} cv_state_t;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

/**
 * Initialize counter validation state.
 *
 * @param state         Pointer to state structure
 * @param bus_width     Bus width in bits (8, 16, 24, or 32)
 * @param dma_buf_size  DMA buffer size for header-only validation
 */
void cv_init(cv_state_t *state, int bus_width, int dma_buf_size);

/**
 * Reset counter validation state for a new test run.
 * Clears counters and statistics but preserves configuration.
 *
 * @param state  Pointer to state structure
 */
void cv_reset(cv_state_t *state);

/**
 * Validate every counter in a buffer (full stream validation).
 *
 * @param state         Pointer to state structure
 * @param data          Buffer to validate
 * @param length        Buffer length in bytes
 * @param transfer_num  Transfer number for error logging
 */
void cv_validate_full(cv_state_t *state, const uint8_t *data, int length, unsigned transfer_num);

/**
 * Validate only the first counter of each DMA buffer (header-only validation).
 *
 * @param state         Pointer to state structure
 * @param data          Buffer to validate
 * @param length        Buffer length in bytes
 * @param transfer_num  Transfer number for error logging
 */
void cv_validate_headers(cv_state_t *state, const uint8_t *data, int length, unsigned transfer_num);

/**
 * Get current error count.
 */
static inline unsigned cv_get_error_count(cv_state_t *state) {
    return atomic_load(&state->counter_errors);
}

/**
 * Get total counters checked.
 */
static inline unsigned long long cv_get_counters_checked(cv_state_t *state) {
    return atomic_load(&state->total_counters_checked);
}

/**
 * Get number of logged errors.
 */
static inline unsigned cv_get_errors_logged(cv_state_t *state) {
    return atomic_load(&state->errors_logged);
}

/**
 * Get the current expected counter value (for summary output).
 * Note: Not thread-safe, call only when validation is stopped.
 */
static inline uint32_t cv_get_expected_counter(cv_state_t *state) {
    return state->expected_counter;
}

#endif /* COUNTER_VALIDATION_H */
