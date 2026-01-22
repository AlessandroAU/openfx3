/*
 * USB Benchmark Test
 *
 * Measures maximum USB throughput by streaming pre-filled buffers from
 * the FX3 device without GPIF involvement. This tests pure USB/DMA
 * performance.
 *
 * Each buffer contains an incrementing 32-bit counter in the first 4 bytes
 * which is validated to detect any dropped or reordered buffers.
 *
 * Press Ctrl+C to exit.
 *
 * Build:
 *   gcc -O2 -o benchmark benchmark.c -Ilib/include -Lbuild -lopenfx3 -lusb-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdatomic.h>
#include <signal.h>

#include <libusb-1.0/libusb.h>

#include <openfx3/openfx3.h>
#include <openfx3/platform.h>

//-----------------------------------------------------------------------------
// Configuration
//-----------------------------------------------------------------------------

#define STATS_UPDATE_INTERVAL_MS 100
#define ERROR_LOG_MAX            16

/* Throughput moving-window (seconds) */
#define THROUGHPUT_WINDOW_SEC     1
#define THROUGHPUT_SAMPLES ((THROUGHPUT_WINDOW_SEC * 1000) / STATS_UPDATE_INTERVAL_MS + 4)

//-----------------------------------------------------------------------------
// Globals
//-----------------------------------------------------------------------------

static libusb_context *g_usb_ctx = NULL;
static libusb_device_handle *g_handle = NULL;
static volatile sig_atomic_t g_quit = 0;
static fx3_acquisition_session_t *g_session = NULL;

/* Moving-window throughput samples */
typedef struct {
    double t;
    unsigned long long bytes;
} throughput_sample_t;

static throughput_sample_t g_throughput_samples[THROUGHPUT_SAMPLES];
static int g_throughput_next = 0;
static int g_throughput_count = 0;

static atomic_uint g_transfer_count;

/* DMA buffer size (queried from firmware) */
static int g_dma_buffer_size = 0;

/* Counter validation state */
static uint32_t g_expected_counter = 0;
static bool g_counter_initialized = false;
static atomic_uint g_counter_errors;
static atomic_ullong g_buffers_checked;

/* Error log */
typedef struct {
    unsigned transfer_num;
    int buffer_index;
    uint32_t expected;
    uint32_t actual;
} error_log_entry_t;

static error_log_entry_t g_error_log[ERROR_LOG_MAX];
static atomic_uint g_errors_logged;

//-----------------------------------------------------------------------------
// Timer
//-----------------------------------------------------------------------------

static platform_interval_timer_t g_timer;

//-----------------------------------------------------------------------------
// Signal handler
//-----------------------------------------------------------------------------

static void signal_handler(int sig) {
    (void)sig;
    g_quit = 1;
    if (g_session) {
        fx3_acquisition_stop(g_session);
    }
}

//-----------------------------------------------------------------------------
// Counter validation
//-----------------------------------------------------------------------------

static void validate_buffer(const uint8_t *data, int length, unsigned transfer_num) {
    /* Each g_dma_buffer_size chunk has a counter in the first 4 bytes */
    int num_buffers = length / g_dma_buffer_size;

    for (int i = 0; i < num_buffers; i++) {
        const uint32_t *buf32 = (const uint32_t *)(data + i * g_dma_buffer_size);
        uint32_t counter = buf32[0];

        atomic_fetch_add(&g_buffers_checked, 1);

        if (!g_counter_initialized) {
            g_expected_counter = counter;
            g_counter_initialized = true;
        }

        if (counter != g_expected_counter) {
            atomic_fetch_add(&g_counter_errors, 1);

            /* Log error details */
            unsigned logged = atomic_load(&g_errors_logged);
            if (logged < ERROR_LOG_MAX) {
                g_error_log[logged].transfer_num = transfer_num;
                g_error_log[logged].buffer_index = i;
                g_error_log[logged].expected = g_expected_counter;
                g_error_log[logged].actual = counter;
                atomic_fetch_add(&g_errors_logged, 1);
            }

            /* Resync to actual counter */
            g_expected_counter = counter + 1;
        } else {
            g_expected_counter++;
        }
    }
}

//-----------------------------------------------------------------------------
// Acquisition Callback
//-----------------------------------------------------------------------------

static int acquisition_callback(const uint8_t *data, int length, void *user_data) {
    (void)user_data;

    unsigned transfer_num = atomic_fetch_add(&g_transfer_count, 1) + 1;
    validate_buffer(data, length, transfer_num);

    return g_quit ? 1 : 0;
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(int argc, char **argv) {
    int ret;
    const char *fw_path = NULL;
    int force_reload = 1;
    int duration_sec = 0;  /* 0 = run until Ctrl+C */

    /* Parse arguments */
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-r") == 0 || strcmp(argv[i], "--reset") == 0) {
            force_reload = 1;
        } else if (strcmp(argv[i], "-d") == 0 && i + 1 < argc) {
            duration_sec = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--firmware") == 0 && i + 1 < argc) {
            fw_path = argv[++i];
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            printf("Usage: %s [options]\n", argv[0]);
            printf("\nOptions:\n");
            printf("  -d <seconds>        Run for specified duration (default: until Ctrl+C)\n");
            printf("  -r, --reset         Reset device and reload firmware\n");
            printf("  --firmware <path>   Firmware file (default: embedded)\n");
            printf("  -h, --help          Show this help\n");
            printf("\nMeasures maximum USB throughput from FX3 device.\n");
            printf("Benchmark mode sends pre-filled buffers without GPIF.\n");
            printf("Validates incrementing counter in first 4 bytes of each DMA buffer.\n");
            printf("Press Ctrl+C to exit.\n");
            return 0;
        }
    }

    /* Setup signal handler */
    signal(SIGINT, signal_handler);
#ifndef _WIN32
    signal(SIGTERM, signal_handler);
#endif

    printf("FX3 USB Benchmark\n");
    printf("=================\n\n");

    /* Initialize libusb */
    ret = libusb_init(&g_usb_ctx);
    if (ret != 0) {
        fprintf(stderr, "Failed to initialize libusb: %s\n", libusb_error_name(ret));
        return 1;
    }

    /* Reset and reload firmware if requested */
    if (force_reload) {
        ret = fx3_open_device(g_usb_ctx, &g_handle, 0);
        if (ret >= 0 && g_handle) {
            printf("Resetting device to bootloader...\n");
            fx3_reboot_bootloader(g_handle);
            fx3_close(g_handle);
            g_handle = NULL;
            sleep_ms(500);
        }
    }

    /* Initialize device */
    ret = fx3_init(g_usb_ctx, &g_handle, fw_path, force_reload);
    if (ret != 0) {
        libusb_exit(g_usb_ctx);
        return 1;
    }

    /* Get firmware version */
    uint8_t major, minor;
    if (fx3_get_firmware_version(g_handle, &major, &minor) == 0) {
        printf("Firmware version: %d.%d\n", major, minor);
    }

    /* Reset counters */
    atomic_store(&g_transfer_count, 0);
    atomic_store(&g_counter_errors, 0);
    atomic_store(&g_buffers_checked, 0);
    atomic_store(&g_errors_logged, 0);
    g_expected_counter = 0;
    g_counter_initialized = false;

    /* Start benchmark mode */
    printf("Starting benchmark mode...\n");
    ret = fx3_start_benchmark(g_handle);
    if (ret != 0) {
        fprintf(stderr, "Failed to start benchmark\n");
        fx3_close(g_handle);
        libusb_exit(g_usb_ctx);
        return 1;
    }

    /* Query acquisition parameters from firmware (DMA buffer config) */
    struct fx3_acquisition_params acq_params;
    if (fx3_acquisition_get_default_params(g_handle, &acq_params) != 0) {
        fprintf(stderr, "Failed to get acquisition parameters from firmware\n");
        fx3_stop_acquisition(g_handle);
        fx3_close(g_handle);
        libusb_exit(g_usb_ctx);
        return 1;
    }

    /* Store DMA buffer size for counter validation */
    g_dma_buffer_size = acq_params.dma_buffer_size;

    /* Wait for benchmark to become active (poll instead of fixed sleep) */
    if (fx3_wait_acquisition_ready(g_handle, 1000) != 0) {
        fprintf(stderr, "Timeout waiting for benchmark to start\n");
        fx3_close(g_handle);
        libusb_exit(g_usb_ctx);
        return 1;
    }

    printf("DMA config: %d buffers x %d bytes, transfer size: %d bytes\n",
           acq_params.dma_buffer_count, acq_params.dma_buffer_size, acq_params.transfer_size);
    if (duration_sec > 0) {
        printf("Duration: %d seconds\n", duration_sec);
    }
    printf("\nPress Ctrl+C to stop.\n\n");

    /* Create host-side acquisition session using firmware-derived parameters */
    ret = fx3_acquisition_create(g_handle, g_usb_ctx,
                                 acq_params.transfer_size,
                                 acq_params.num_transfers,
                                 acq_params.timeout_ms,
                                 0,
                                 acq_params.skip_bytes,
                                 acquisition_callback,
                                 NULL,
                                 &g_session);
    if (ret != 0) {
        fprintf(stderr, "Failed to create acquisition session\n");
        fx3_stop_acquisition(g_handle);
        fx3_close(g_handle);
        libusb_exit(g_usb_ctx);
        return 1;
    }

    /* Warmup period */
    printf("Warming up (100ms)...");
    fflush(stdout);

    platform_interval_timer_t warmup_timer;
    platform_interval_timer_start(&warmup_timer);

    printf("\r                      \r");
    fflush(stdout);

    /* Reset stats after warmup */
    atomic_store(&g_transfer_count, 0);
    atomic_store(&g_counter_errors, 0);
    atomic_store(&g_buffers_checked, 0);
    atomic_store(&g_errors_logged, 0);
    g_expected_counter = 0;
    g_counter_initialized = false;
    g_throughput_next = 0;
    g_throughput_count = 0;

    /* Print table header */
    printf("   Time   |   MB/s   |  Total MB  | Transfers | CntErr\n");
    printf("----------------------------------------------------------\n");
    fflush(stdout);

    /* Start timing */
    platform_interval_timer_start(&g_timer);
    unsigned long long last_bytes = 0;

    /* Seed initial throughput sample */
    {
        struct fx3_acquisition_stats stats;
        fx3_acquisition_get_stats(g_session, &stats);
        double seed_t = platform_interval_timer_elapsed_sec(&g_timer);
        g_throughput_samples[0].t = seed_t;
        g_throughput_samples[0].bytes = stats.total_bytes;
        g_throughput_next = 1 % THROUGHPUT_SAMPLES;
        g_throughput_count = 1;
    }

    /* Main loop */
    while (!g_quit && fx3_acquisition_is_running(g_session)) {
        double elapsed = platform_interval_timer_elapsed_sec(&g_timer);

        /* Check duration limit */
        if (duration_sec > 0 && elapsed >= duration_sec) {
            break;
        }

        /* Periodic stats update */
        if (platform_interval_timer_interval_ms(&g_timer) >= STATS_UPDATE_INTERVAL_MS) {
            struct fx3_acquisition_stats stats;
            fx3_acquisition_get_stats(g_session, &stats);
            unsigned long long total_bytes = stats.total_bytes;
            unsigned transfers = atomic_load(&g_transfer_count);
            unsigned errors = atomic_load(&g_counter_errors);

            double interval_sec = platform_interval_timer_interval_ms(&g_timer) / 1000.0;
            unsigned long long interval_bytes = total_bytes - last_bytes;

            /* Append sample */
            int cap = THROUGHPUT_SAMPLES;
            g_throughput_samples[g_throughput_next].t = elapsed;
            g_throughput_samples[g_throughput_next].bytes = total_bytes;
            g_throughput_next = (g_throughput_next + 1) % cap;
            if (g_throughput_count < cap) g_throughput_count++;

            /* Compute moving-window average */
            double window_start = elapsed - (double)THROUGHPUT_WINDOW_SEC;
            int oldest = (g_throughput_next - g_throughput_count + cap) % cap;
            int chosen = oldest;
            for (int i = 0; i < g_throughput_count; i++) {
                int idx = (oldest + i) % cap;
                if (g_throughput_samples[idx].t >= window_start) {
                    chosen = idx;
                    break;
                }
            }

            double time_delta = elapsed - g_throughput_samples[chosen].t;
            unsigned long long bytes_delta = total_bytes - g_throughput_samples[chosen].bytes;
            double moving_mbps;
            if (time_delta > 1e-6) {
                moving_mbps = (bytes_delta / 1e6) / time_delta;
            } else {
                moving_mbps = (interval_bytes / 1e6) / (interval_sec > 0.0 ? interval_sec : 1e-6);
            }

            printf("\r %6.1fs  | %7.2f  | %9.1f  | %9u | %6u ",
                   elapsed, moving_mbps, total_bytes / 1e6, transfers, errors);
            fflush(stdout);

            last_bytes = total_bytes;
            platform_interval_timer_reset_interval(&g_timer);
        }

        sleep_ms(10);
    }

    /* Capture final stats */
    double elapsed = platform_interval_timer_elapsed_sec(&g_timer);
    unsigned transfer_count = atomic_load(&g_transfer_count);
    unsigned counter_errors = atomic_load(&g_counter_errors);
    unsigned long long buffers_checked = atomic_load(&g_buffers_checked);
    unsigned errors_logged = atomic_load(&g_errors_logged);

    struct fx3_acquisition_stats final_stats;
    fx3_acquisition_get_stats(g_session, &final_stats);
    unsigned long long total_bytes = final_stats.total_bytes;

    printf("\n\nShutting down...\n");

    /* Cleanup */
    fx3_acquisition_destroy(g_session);
    g_session = NULL;
    fx3_stop_acquisition(g_handle);

    /* Print logged errors */
    if (errors_logged > 0) {
        printf("\nFirst %u counter errors:\n", errors_logged);
        for (unsigned i = 0; i < errors_logged && i < ERROR_LOG_MAX; i++) {
            printf("  ERROR: transfer %u, buffer %d: expected %u, got %u (gap: %d)\n",
                   g_error_log[i].transfer_num,
                   g_error_log[i].buffer_index,
                   g_error_log[i].expected,
                   g_error_log[i].actual,
                   (int)(g_error_log[i].actual - g_error_log[i].expected));
        }
    }

    /* Print final stats */
    printf("\n");
    printf("==================== FINAL STATS ====================\n");
    printf("  Duration:          %.2f seconds\n", elapsed);
    printf("  Transfers:         %u\n", transfer_count);
    printf("  Total bytes:       %.2f MB\n", total_bytes / 1e6);
    printf("  Average:           %.1f MB/s\n", elapsed > 0 ? (total_bytes / 1e6) / elapsed : 0);
    printf("  Buffers checked:   %llu\n", buffers_checked);
    printf("  Counter errors:    %u\n", counter_errors);
    printf("  Last counter:      %u\n", g_expected_counter > 0 ? g_expected_counter - 1 : 0);
    printf("=====================================================\n");

    if (counter_errors == 0 && transfer_count > 0) {
        printf("\nSUCCESS: All %llu buffer counters validated correctly!\n", buffers_checked);
    } else if (counter_errors > 0) {
        printf("\nFAILURE: %u counter discontinuities detected!\n", counter_errors);
    }

    fx3_close(g_handle);
    libusb_exit(g_usb_ctx);

    return (counter_errors > 0) ? 1 : 0;
}
