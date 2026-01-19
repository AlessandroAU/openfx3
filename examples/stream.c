/*
 * Stream Test
 *
 * USB streaming test tool for FX3 devices. Measures throughput and optionally
 * validates 32-bit incrementing counters in the data stream.
 *
 * Press Ctrl+C to exit.
 *
 * Build:
 *   gcc -O2 -o stream stream.c -Ilib/include -Lbuild -lopenfx3 -lusb-1.0
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

#define COUNTER_SIZE             4            /* 4-byte counter */
#define DMA_BUFFER_SIZE          32768           /* FX3 DMA buffer size (must match firmware) */
/* Per-transfer buffer size */
#define SKIP_INITIAL_BYTES       (DMA_BUFFER_SIZE * 12)  /* Skip first 384KB (DMA pool size) */
#define TRANSFER_SIZE            (32768 * 16)  /* 512 KiB per transfer */
#define TRANSFER_TIMEOUT_MS      1000            /* timeout for streaming */
#define ASYNC_TRANSFERS          16           /* Number of transfers in flight */

#define ERROR_LOG_MAX            16           /* Max errors to log details for */
#define STATS_UPDATE_INTERVAL_MS 100           /* Update display every 500ms */

/* Throughput moving-window (seconds) */
#define THROUGHPUT_WINDOW_SEC     1           /* Window in seconds for moving average */
/* Number of samples kept for the window (a few extra slots for safety) */
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
    double t; /* seconds since timer start */
    unsigned long long bytes;
} throughput_sample_t;

static throughput_sample_t g_throughput_samples[THROUGHPUT_SAMPLES];
static int g_throughput_next = 0; /* next insert index */
static int g_throughput_count = 0; /* number of valid samples stored */

/* Sequence validation state - only accessed from callback (serialized by libusb) */
static uint32_t g_expected_counter = 0;
static bool g_counter_initialized = false;
static atomic_uint g_counter_errors;
static atomic_ullong g_total_counters_checked;
static atomic_uint g_transfer_count;

/* Error log - written from callback, read after test */
typedef struct {
    unsigned transfer_num;
    int offset;
    uint32_t expected;
    uint32_t actual;
} error_log_entry_t;

static error_log_entry_t g_error_log[ERROR_LOG_MAX];
static atomic_uint g_errors_logged;

/* Validation mode: 0=none, 1=all counters, 2=headers only */
static int g_validate_mode = 0;

//-----------------------------------------------------------------------------
// Timer (uses platform.h)
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
// Counter validation (called from callback - keep minimal)
//-----------------------------------------------------------------------------

/* Validate every 32-bit value as a sequential counter (full stream validation) */
static void validate_buffer_full(uint32_t *counters, int num_counters, unsigned transfer_num) {
    for (int i = 0; i < num_counters; i++) {
        uint32_t counter = counters[i];
        atomic_fetch_add(&g_total_counters_checked, 1);

        if (!g_counter_initialized) {
            g_expected_counter = counter;
            g_counter_initialized = true;
        }

        if (counter != g_expected_counter) {
            atomic_fetch_add(&g_counter_errors, 1);
            /* Log error details (no printf in callback) */
            unsigned logged = atomic_load(&g_errors_logged);
            if (logged < ERROR_LOG_MAX) {
                g_error_log[logged].transfer_num = transfer_num;
                g_error_log[logged].offset = i * COUNTER_SIZE;
                g_error_log[logged].expected = g_expected_counter;
                g_error_log[logged].actual = counter;
                atomic_fetch_add(&g_errors_logged, 1);
            }
            g_expected_counter = counter + 1;
        } else {
            g_expected_counter++;
        }
    }
}

/* Validate only the first 4-byte header of each DMA buffer */
static void validate_buffer_headers(const uint8_t *data, int length, unsigned transfer_num) {
    /* Iterate through each DMA buffer in the transfer */
    for (int offset = 0; offset + COUNTER_SIZE <= length; offset += DMA_BUFFER_SIZE) {
        uint32_t counter = *(const uint32_t *)(data + offset);
        atomic_fetch_add(&g_total_counters_checked, 1);

        if (!g_counter_initialized) {
            g_expected_counter = counter;
            g_counter_initialized = true;
        }

        if (counter != g_expected_counter) {
            atomic_fetch_add(&g_counter_errors, 1);
            /* Log error details (no printf in callback) */
            unsigned logged = atomic_load(&g_errors_logged);
            if (logged < ERROR_LOG_MAX) {
                g_error_log[logged].transfer_num = transfer_num;
                g_error_log[logged].offset = offset;
                g_error_log[logged].expected = g_expected_counter;
                g_error_log[logged].actual = counter;
                atomic_fetch_add(&g_errors_logged, 1);
            }
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

    /* Validate counters if enabled */
    if (g_validate_mode == 1) {
        int num_counters = length / COUNTER_SIZE;
        validate_buffer_full((uint32_t *)data, num_counters, transfer_num);
    } else if (g_validate_mode == 2) {
        validate_buffer_headers(data, length, transfer_num);
    }

    return g_quit ? 1 : 0;  /* Return non-zero to stop */
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(int argc, char **argv) {
    int ret;
    const char *fw_path = NULL;
    int bus_mhz = 100;
    int bus_width = 32;
    int force_reload = 1;

    /* Parse arguments */
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-f") == 0 && i + 1 < argc) {
            bus_mhz = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-w") == 0 && i + 1 < argc) {
            bus_width = atoi(argv[++i]);
            if (bus_width != 8 && bus_width != 16 && bus_width != 24 && bus_width != 32) {
                fprintf(stderr, "Error: bus width must be 8, 16, 24, or 32\n");
                return 1;
            }
        } else if (strcmp(argv[i], "-r") == 0 || strcmp(argv[i], "--reset") == 0) {
            force_reload = 1;
        } else if (strcmp(argv[i], "--validate-all") == 0) {
            g_validate_mode = 1;
        } else if (strcmp(argv[i], "--validate-headers") == 0) {
            g_validate_mode = 2;
        } else if (strcmp(argv[i], "--firmware") == 0 && i + 1 < argc) {
            fw_path = argv[++i];
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            printf("Usage: %s [options]\n", argv[0]);
            printf("\nOptions:\n");
            printf("  -f <MHz>            Bus clock in MHz (default: 100)\n");
            printf("  -w <bits>           Bus width: 8, 16, 24, 32 (default: 32)\n");
            printf("  --validate-all      Validate every 32-bit counter in stream\n");
            printf("  --validate-headers  Validate only first counter per DMA buffer\n");
            printf("  -r, --reset         Reset device and reload firmware\n");
            printf("  --firmware <path>   Firmware file (default: embedded)\n");
            printf("  -h, --help          Show this help\n");
            printf("\nStreams data from FX3 device and measures throughput.\n");
            printf("Press Ctrl+C to exit.\n");
            return 0;
        } else {
            fprintf(stderr, "Error: unknown option '%s'\n", argv[i]);
            return 1;
        }
    }

    /* Setup signal handler */
    signal(SIGINT, signal_handler);
#ifndef _WIN32
    signal(SIGTERM, signal_handler);
#endif

    printf("FX3 Stream Test%s\n", g_validate_mode ? " (counter validation)" : "");
    printf("===============%s\n\n", g_validate_mode ? "=====================" : "");

    /* Initialize libusb */
    ret = libusb_init(&g_usb_ctx);
    if (ret != 0) {
        fprintf(stderr, "Failed to initialize libusb: %s\n", libusb_error_name(ret));
        return 1;
    }

    /* Reset and reload firmware if requested */
    if (force_reload) {
        /* Try to open device and send reboot command */
        ret = fx3_open_device(g_usb_ctx, &g_handle, 0);
        if (ret >= 0 && g_handle) {
            printf("Resetting device to bootloader...\n");
            fx3_reboot_bootloader(g_handle);
            fx3_close(g_handle);
            g_handle = NULL;
            /* Wait for device to reset and re-enumerate */
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

    /* Reset counters before printing header */
    atomic_store(&g_transfer_count, 0);
    atomic_store(&g_counter_errors, 0);
    atomic_store(&g_total_counters_checked, 0);
    atomic_store(&g_errors_logged, 0);

    g_expected_counter = 0;
    g_counter_initialized = false;

    /* Start firmware-side acquisition */
    ret = fx3_start_acquisition(g_handle, bus_mhz, bus_width, 1 /* internal clock */);
    if (ret != 0) {
        fprintf(stderr, "Failed to start acquisition\n");
        fx3_close(g_handle);
        libusb_exit(g_usb_ctx);
        return 1;
    }

    /* Wait for acquisition to become active (poll instead of fixed sleep) */
    if (fx3_wait_acquisition_ready(g_handle, 1000) != 0) {
        fprintf(stderr, "Timeout waiting for acquisition to start\n");
        fx3_close(g_handle);
        libusb_exit(g_usb_ctx);
        return 1;
    }

    /* Query and print actual bus configuration, then print table header */
    struct fx3_acq_status acq_status;
    if (fx3_get_acq_status(g_handle, &acq_status) == 0 && acq_status.bus_freq_hz > 0) {
        printf("Starting acquisition: %u-bit @ %lu.%02lu MHz (requested %d MHz)\n",
               acq_status.bus_width,
               (unsigned long)(acq_status.bus_freq_hz / 1000000),
               (unsigned long)((acq_status.bus_freq_hz % 1000000) / 10000),
               bus_mhz);
        printf("Clock config: SysClk=%lu.%01lu MHz, FBDIV=%u, DIV=%u\n",
               (unsigned long)(acq_status.sys_clk_hz / 1000000),
               (unsigned long)((acq_status.sys_clk_hz % 1000000) / 100000),
               acq_status.pll_fbdiv,
               acq_status.gpif_div);
    } else {
        printf("Starting acquisition: %d MHz, %d-bit bus\n", bus_mhz, bus_width);
    }
    printf("Transfer size: %d bytes\n", TRANSFER_SIZE);
    printf("\nPress Ctrl+C to stop.\n\n");

    /* Create host-side acquisition session (chunk_size=0 means whole transfer at once) */
    ret = fx3_acquisition_create(g_handle, g_usb_ctx,
                                 TRANSFER_SIZE,
                                 ASYNC_TRANSFERS,
                                 TRANSFER_TIMEOUT_MS,
                                 0,  /* chunk_size: 0 = whole transfer */
                                 SKIP_INITIAL_BYTES,  /* skip_initial_bytes */
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

    /* Print table header */
    if (g_validate_mode) {
        printf("   Time   |   MB/s   |  Host MB  |   FW MB   |  FW Buf  | CntErr | Stalls | GpifErr\n");
        printf("--------------------------------------------------------------------------------------------\n");
    } else {
        printf("   Time   |   MB/s   |  Host MB  |   FW MB   |  FW Buf  | Stalls | GpifErr\n");
        printf("----------------------------------------------------------------------------------\n");
    }
    fflush(stdout);

    /* Start timing */
    platform_interval_timer_start(&g_timer);
    unsigned long long last_bytes = 0;

    /* Seed initial throughput sample (time ~= 0, bytes == 0) */
    {
        struct fx3_acquisition_stats stats;
        fx3_acquisition_get_stats(g_session, &stats);
        double seed_t = platform_interval_timer_elapsed_sec(&g_timer);
        g_throughput_samples[0].t = seed_t;
        g_throughput_samples[0].bytes = stats.total_bytes;
        g_throughput_next = 1 % THROUGHPUT_SAMPLES;
        g_throughput_count = 1;
    }

    /* Main acquisition loop - internal thread handles USB events */
    while (!g_quit && fx3_acquisition_is_running(g_session)) {
        /* Periodic stats update */
        if (platform_interval_timer_interval_ms(&g_timer) >= STATS_UPDATE_INTERVAL_MS) {
            double elapsed = platform_interval_timer_elapsed_sec(&g_timer);

            struct fx3_acquisition_stats stats;
            fx3_acquisition_get_stats(g_session, &stats);
            unsigned long long total_bytes = stats.total_bytes;
            unsigned errors = atomic_load(&g_counter_errors);

            /* Record sample for moving-window throughput */
            double interval_sec = platform_interval_timer_interval_ms(&g_timer) / 1000.0;
            unsigned long long interval_bytes = total_bytes - last_bytes;

            /* Append sample (elapsed seconds, cumulative bytes) */
            int cap = THROUGHPUT_SAMPLES;
            g_throughput_samples[g_throughput_next].t = elapsed;
            g_throughput_samples[g_throughput_next].bytes = total_bytes;
            g_throughput_next = (g_throughput_next + 1) % cap;
            if (g_throughput_count < cap) g_throughput_count++;

            /* Compute moving-window average over last THROUGHPUT_WINDOW_SEC seconds */
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
                /* Fallback to instant if window too small */
                moving_mbps = (interval_bytes / 1e6) / (interval_sec > 0.0 ? interval_sec : 1e-6);
            }

            /* Query firmware stats */
            struct fx3_drop_stats drops = {0};
            fx3_get_drop_stats(g_handle, &drops);

            if (g_validate_mode) {
                printf("\r %6.1fs  | %7.2f  | %8.1f  | %8.1f  | %8llu | %6u | %6u | %7u ",
                       elapsed, moving_mbps, total_bytes / 1e6, drops.total_bytes / 1e6,
                       (unsigned long long)drops.total_buffers,
                       errors, drops.stalls, drops.gpif_errors);
            } else {
                printf("\r %6.1fs  | %7.2f  | %8.1f  | %8.1f  | %8llu | %6u | %7u ",
                       elapsed, moving_mbps, total_bytes / 1e6, drops.total_bytes / 1e6,
                       (unsigned long long)drops.total_buffers,
                       drops.stalls, drops.gpif_errors);
            }
            fflush(stdout);

            last_bytes = total_bytes;
            platform_interval_timer_reset_interval(&g_timer);
        }

        /* Sleep to avoid spinning at 100% CPU */
        sleep_ms(10);
    }

    /* Capture final stats BEFORE destroying session */
    double elapsed = platform_interval_timer_elapsed_sec(&g_timer);
    unsigned transfer_count = atomic_load(&g_transfer_count);
    unsigned long long total_counters = atomic_load(&g_total_counters_checked);
    unsigned counter_errors = atomic_load(&g_counter_errors);
    unsigned errors_logged = atomic_load(&g_errors_logged);

    struct fx3_drop_stats drops = {0};
    fx3_get_drop_stats(g_handle, &drops);
    unsigned long long total_bytes = drops.total_bytes;

    printf("\nShutting down...\n");

    /* Destroy acquisition session (cancels transfers, drains events) */
    fx3_acquisition_destroy(g_session);
    g_session = NULL;

    /* Stop firmware-side acquisition */
    fx3_stop_acquisition(g_handle);

    /* Print logged errors (only if counter validation enabled) */
    if (g_validate_mode && errors_logged > 0) {
        printf("\nFirst %u counter errors:\n", errors_logged);
        for (unsigned i = 0; i < errors_logged && i < ERROR_LOG_MAX; i++) {
            printf("  ERROR: transfer %u, offset %d: expected %u, got %u (gap: %d)\n",
                   g_error_log[i].transfer_num,
                   g_error_log[i].offset,
                   g_error_log[i].expected,
                   g_error_log[i].actual,
                   (int)(g_error_log[i].actual - g_error_log[i].expected));
        }
    }

    printf("\n");
    printf("====================== SUMMARY ======================\n");
    printf("  Duration:          %.2f seconds\n", elapsed);
    printf("  Transfers:         %u\n", transfer_count);
    printf("  Total bytes:       %.2f MB\n", total_bytes / 1e6);
    printf("  Average:           %.1f MB/s\n", elapsed > 0 ? (total_bytes / 1e6) / elapsed : 0);
    if (g_validate_mode) {
        printf("  Counters checked:  %llu\n", total_counters);
        printf("  Counter Errors:    %u\n", counter_errors);
        printf("  Last Counter:      %u\n", g_expected_counter > 0 ? g_expected_counter - 1 : 0);
    }
    printf("  FW Buffers:        %llu\n", (unsigned long long)drops.total_buffers);
    printf("  FW Bytes:          %.2f MB\n", drops.total_bytes / 1e6);
    printf("  FW Stalls:         %u\n", drops.stalls);
    printf("  FW GPIF Errors:    %u\n", drops.gpif_errors);
    printf("=====================================================\n");

    if (g_validate_mode) {
        if (counter_errors == 0 && transfer_count > 0) {
            printf("\nSUCCESS: All %llu counters validated correctly!\n", total_counters);
        } else if (counter_errors > 0) {
            printf("\nFAILURE: %u counter discontinuities detected!\n", counter_errors);
        }
    }

    /* Cleanup */
    fx3_close(g_handle);
    libusb_exit(g_usb_ctx);

    /* Return error code only if counter validation is enabled and failed */
    return (g_validate_mode && counter_errors > 0) ? 1 : 0;
}
