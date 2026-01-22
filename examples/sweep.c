/*
 * Bus Frequency Sweep Test
 *
 * Tests device connectivity and data transfer at various bus frequencies.
 * Cycles through frequencies and reports pass/fail along with measured data rates.
 *
 * Build:
 *   gcc -O2 -o sweep sweep.c -Ilib/include -Lbuild -lopenfx3 -lusb-1.0
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

#include "counter_validation.h"

//-----------------------------------------------------------------------------
// Configuration
//-----------------------------------------------------------------------------
#define TARGET_BYTES_DEFAULT     (1024 * 1024 * 32)   /* Default: collect 32 MiB per frequency */
#define NO_DATA_TIMEOUT_MS       500            /* Abort if no data received within this time */
#define MAX_TEST_TIMEOUT_MS      30000           /* Maximum time per test (safety limit) */
#define MAX_FREQUENCIES          256             /* Maximum number of test frequencies */

/* Default frequency sweep parameters (MHz) */
#define DEFAULT_FREQ_START       5
#define DEFAULT_FREQ_STOP        100
#define DEFAULT_FREQ_STEP        5

//#define ENABLE_COUNTER_DEBUG                /* Set to enable detailed counter error logging */

//-----------------------------------------------------------------------------
// Globals
//-----------------------------------------------------------------------------

static libusb_context *g_usb_ctx = NULL;
static libusb_device_handle *g_handle = NULL;
static volatile sig_atomic_t g_quit = 0;

/* Frequency sweep parameters */
static int g_freq_start = DEFAULT_FREQ_START;
static int g_freq_stop = DEFAULT_FREQ_STOP;
static int g_freq_step = DEFAULT_FREQ_STEP;
static int g_test_frequencies[MAX_FREQUENCIES];
static int g_num_frequencies = 0;

/* Target bytes per test (configurable via -b flag) */
static unsigned long long g_target_bytes = TARGET_BYTES_DEFAULT;

/* Validation mode: 0=none, 1=all counters, 2=headers only */
static int g_validate_mode = 0;

//-----------------------------------------------------------------------------
// Signal handler
//-----------------------------------------------------------------------------

static void signal_handler(int sig) {
    (void)sig;
    g_quit = 1;
}

//-----------------------------------------------------------------------------
// Test result tracking
//-----------------------------------------------------------------------------

typedef struct {
    int freq_mhz;
    uint32_t actual_freq_hz;  /* Actual frequency from firmware in Hz */
    uint32_t sys_clk_hz;      /* System clock frequency in Hz */
    uint16_t gpif_div;        /* GPIF divider register value */
    uint8_t pll_fbdiv;        /* PLL feedback divider (20-27) */
    bool passed;
    double data_rate_mbps;
    uint64_t bytes_received;
    unsigned counter_errors;
    unsigned long long counters_checked;
    uint32_t stall_count;
    const char *error_msg;
} test_result_t;

static test_result_t g_results[MAX_FREQUENCIES];

//-----------------------------------------------------------------------------
// Simple acquisition callback - count bytes and optionally validate counters
//-----------------------------------------------------------------------------

static atomic_ullong g_bytes_received;
static atomic_uint g_transfer_count;

/* Counter validation state */
static cv_state_t g_cv_state;

static int acquisition_callback(const uint8_t *data, int length, void *user_data) {
    (void)user_data;

    unsigned transfer_num = atomic_fetch_add(&g_transfer_count, 1) + 1;

    if (g_validate_mode == 1) {
        cv_validate_full(&g_cv_state, data, length, transfer_num);
    } else if (g_validate_mode == 2) {
        cv_validate_headers(&g_cv_state, data, length, transfer_num);
    }

    atomic_fetch_add(&g_bytes_received, (unsigned long long)length);
    return g_quit ? 1 : 0;
}


//-----------------------------------------------------------------------------
// Run test at a single frequency
//-----------------------------------------------------------------------------

static test_result_t test_frequency(int freq_mhz, int bus_width) {
    test_result_t result = {0};
    result.freq_mhz = freq_mhz;
    result.passed = false;

    /* Reset counters for this frequency test */
    atomic_store(&g_bytes_received, 0);
    atomic_store(&g_transfer_count, 0);

    /* Start firmware-side acquisition */
    struct fx3_acq_config acq_config = {
        .bus_width = bus_width,
        .clk_invert = 0,
        .internal_clk = 1,
        .clk_out = 1,
    };

    int ret = fx3_start_acquisition(g_handle, freq_mhz, &acq_config);
    if (ret != 0) {
        result.error_msg = "Failed to start acquisition";
        return result;
    }

    /* Query acquisition parameters from firmware */
    struct fx3_acquisition_params acq_params;
    if (fx3_acquisition_get_default_params(g_handle, &acq_params) != 0) {
        result.error_msg = "Failed to get DMA config";
        fx3_stop_acquisition(g_handle);
        return result;
    }

    /* Initialize/reset counter validation with firmware's DMA buffer size */
    cv_init(&g_cv_state, bus_width, acq_params.dma_buffer_size);

    /* Get actual bus frequency and clock config */
    struct fx3_acq_status acq_status;
    if (fx3_get_acq_status(g_handle, &acq_status) == 0) {
        result.actual_freq_hz = acq_status.bus_freq_hz;
        result.sys_clk_hz = acq_status.sys_clk_hz;
        result.gpif_div = acq_status.gpif_div;
        result.pll_fbdiv = acq_status.pll_fbdiv;
    }

    /* Wait for acquisition to become active */
    ret = fx3_wait_acquisition_ready(g_handle, 1000);
    if (ret != 0) {
        result.error_msg = "Timeout waiting for acquisition";
        fx3_stop_acquisition(g_handle);
        return result;
    }

    /* Create host-side acquisition session using firmware-derived parameters */
    fx3_acquisition_session_t *session = NULL;
    ret = fx3_acquisition_create(g_handle, g_usb_ctx,
                                 acq_params.transfer_size,
                                 acq_params.num_transfers,
                                 acq_params.timeout_ms,
                                 0,  /* chunk_size: 0 = whole transfer */
                                 acq_params.skip_bytes,
                                 acquisition_callback,
                                 NULL,
                                 &session);
    if (ret != 0) {
        result.error_msg = "Failed to create acquisition session";
        fx3_stop_acquisition(g_handle);
        return result;
    }

    /* Run until we collect target bytes (or timeout) */
    platform_timer_t timer;
    platform_timer_start(&timer);

    while (!g_quit) {
        uint32_t elapsed_ms = platform_timer_elapsed_ms(&timer);
        unsigned long long bytes_now = atomic_load(&g_bytes_received);

        /* Success: collected enough data */
        if (bytes_now >= g_target_bytes) {
            break;
        }

        /* Timeout: no data received */
        if (bytes_now == 0 && elapsed_ms >= NO_DATA_TIMEOUT_MS) {
            break;
        }

        /* Safety timeout */
        if (elapsed_ms >= MAX_TEST_TIMEOUT_MS) {
            break;
        }

        sleep_ms(1);
    }

    double elapsed_sec = platform_timer_elapsed_ms(&timer) / 1000.0;

    /* Capture results before cleanup */
    result.bytes_received = (uint64_t)atomic_load(&g_bytes_received);
    if (elapsed_sec > 0) {
        result.data_rate_mbps = (result.bytes_received / 1e6) / elapsed_sec;
    }

    result.counter_errors = cv_get_error_count(&g_cv_state);
    result.counters_checked = cv_get_counters_checked(&g_cv_state);

    struct fx3_drop_stats drop_stats;
    if (fx3_get_drop_stats(g_handle, &drop_stats) == 0) {
        result.stall_count = drop_stats.stalls;
    }

    /* Cleanup */
    fx3_acquisition_destroy(session);
    fx3_stop_acquisition(g_handle);

    /* Determine pass/fail */
    if (result.bytes_received >= g_target_bytes) {
        result.passed = true;
    } else if (result.bytes_received == 0) {
        result.error_msg = "No data received";
    } else {
        result.error_msg = "Insufficient data";
    }

    if (g_validate_mode && result.counters_checked > 0 && result.counter_errors > 0) {
        result.passed = false;
        result.error_msg = "Counter discontinuities";

#ifdef ENABLE_COUNTER_DEBUG
        /* Debug: print first few error entries */
        unsigned errors_logged = cv_get_errors_logged(&g_cv_state);
        if (errors_logged > 0) {
            printf("  [DEBUG %d MHz] First errors:\n", freq_mhz);
            for (unsigned i = 0; i < errors_logged && i < 4; i++) {
                printf("    xfer %u, offset %d: expected %u, got %u (gap %d)\n",
                       g_cv_state.error_log[i].transfer_num,
                       g_cv_state.error_log[i].offset,
                       g_cv_state.error_log[i].expected,
                       g_cv_state.error_log[i].actual,
                       (int)(g_cv_state.error_log[i].actual - g_cv_state.error_log[i].expected));
            }
        }
#endif
    }

    return result;
}

//-----------------------------------------------------------------------------
// Print results table
//-----------------------------------------------------------------------------

static void print_table_header(int bus_width) {
    printf("\n");
    printf("===========================================================================================================================\n");
    printf("                                          FX3 Bus Frequency Sweep Results\n");
    printf("                                               Bus Width: %d bits\n", bus_width);
    printf("===========================================================================================================================\n");
    printf("\n");

    if (g_validate_mode) {
        printf(" Target  |   Actual    | SysClk | FBDIV | DIV |  Status  | Theoretical  |   Measured   |  Efficiency | CntErr | Stalls\n");
        printf("---------+-------------+--------+-------+-----+----------+--------------+--------------+-------------+--------+--------\n");
    } else {
        printf(" Target  |   Actual    | SysClk | FBDIV | DIV |  Status  | Theoretical  |   Measured   |  Efficiency | Stalls\n");
        printf("---------+-------------+--------+-------+-----+----------+--------------+--------------+-------------+--------\n");
    }
}

static void print_table_row(test_result_t *r, int bus_width) {
    /* Actual frequency in MHz (with decimal for precision) */
    double actual_mhz = r->actual_freq_hz / 1000000.0;
    double sys_clk_mhz = r->sys_clk_hz / 1000000.0;

    /* Theoretical rate = actual_freq_hz * (bus_width / 8) bytes/sec -> MB/s */
    double theoretical_mbps = (r->actual_freq_hz / 1000000.0) * (bus_width / 8.0);
    double efficiency = (theoretical_mbps > 0) ? (r->data_rate_mbps / theoretical_mbps) * 100.0 : 0;

    if (g_validate_mode) {
        printf(" %3d MHz | %7.3f MHz | %5.1f  |   %2u  | %3u |  %s  | %7.2f MB/s | %7.2f MB/s |    %5.1f%% | %6u | %6u\n",
               r->freq_mhz,
               actual_mhz,
               sys_clk_mhz,
               r->pll_fbdiv,
               r->gpif_div,
               r->passed ? "PASS" : "FAIL",
               theoretical_mbps,
               r->data_rate_mbps,
               efficiency,
               r->counter_errors,
               r->stall_count);
    } else {
        printf(" %3d MHz | %7.3f MHz | %5.1f  |   %2u  | %3u |  %s  | %7.2f MB/s | %7.2f MB/s |    %5.1f%% | %6u\n",
               r->freq_mhz,
               actual_mhz,
               sys_clk_mhz,
               r->pll_fbdiv,
               r->gpif_div,
               r->passed ? "PASS" : "FAIL",
               theoretical_mbps,
               r->data_rate_mbps,
               efficiency,
               r->stall_count);
    }
}

static void print_table_footer(int passed, int failed, int total) {
    if (g_validate_mode) {
        printf("---------+-------------+--------+-------+-----+----------+--------------+--------------+-------------+--------+--------\n");
    } else {
        printf("---------+-------------+--------+-------+-----+----------+--------------+--------------+-------------+--------\n");
    }
    printf("\n");
    printf("Summary: %d passed, %d failed out of %d tests\n", passed, failed, total);
    printf("===========================================================================================================================\n");
}

//-----------------------------------------------------------------------------
// Build frequency list from start/stop/step
//-----------------------------------------------------------------------------

static void build_frequency_list(void)
{
    g_num_frequencies = 0;
    if (g_freq_step <= 0) {
        fprintf(stderr, "Error: step must be positive\n");
        return;
    }
    int step = (g_freq_start <= g_freq_stop) ? g_freq_step : -g_freq_step;
    for (int f = g_freq_start;
         (step > 0) ? (f <= g_freq_stop) : (f >= g_freq_stop);
         f += step)
    {
        if (g_num_frequencies >= MAX_FREQUENCIES)
            break;

        g_test_frequencies[g_num_frequencies++] = f;
    }
    /* Ensure stop is included if we didn't land on it */
    if (g_num_frequencies > 0 &&
        g_test_frequencies[g_num_frequencies - 1] != g_freq_stop &&
        g_num_frequencies < MAX_FREQUENCIES)
    {
        g_test_frequencies[g_num_frequencies++] = g_freq_stop;
    }
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(int argc, char **argv) {
    int ret;
    const char *fw_path = NULL;
    int bus_width = 32;
    int force_reload = 1;

    /* Parse arguments */
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-w") == 0 && i + 1 < argc) {
            bus_width = atoi(argv[++i]);
            if (bus_width != 8 && bus_width != 16 && bus_width != 24 && bus_width != 32) {
                fprintf(stderr, "Error: bus width must be 8, 16, 24, or 32\n");
                return 1;
            }
        } else if (strcmp(argv[i], "--validate-all") == 0) {
            g_validate_mode = 1;
        } else if (strcmp(argv[i], "--validate-headers") == 0) {
            g_validate_mode = 2;
        } else if (strcmp(argv[i], "--start") == 0 && i + 1 < argc) {
            g_freq_start = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--stop") == 0 && i + 1 < argc) {
            g_freq_stop = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--step") == 0 && i + 1 < argc) {
            g_freq_step = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--bytes") == 0) {
            if (i + 1 < argc) {
                g_target_bytes = strtoull(argv[++i], NULL, 0);
            }
        } else if (strcmp(argv[i], "-r") == 0 || strcmp(argv[i], "--reset") == 0) {
            force_reload = 1;
        } else if (strcmp(argv[i], "--firmware") == 0 && i + 1 < argc) {
            fw_path = argv[++i];
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            printf("Usage: %s [options]\n", argv[0]);
            printf("\nOptions:\n");
            printf("  -w <bits>           Bus width: 8, 16, 24, 32 (default: 32)\n");
            printf("  --validate-all      Validate every 32-bit counter in stream\n");
            printf("  --validate-headers  Validate only first counter per DMA buffer\n");
            printf("  -b, --bytes <n>     Bytes to collect per frequency (default: %d)\n", TARGET_BYTES_DEFAULT);
            printf("  --start <MHz>       Start frequency in MHz (default: %d)\n", DEFAULT_FREQ_START);
            printf("  --stop <MHz>        Stop frequency in MHz (default: %d)\n", DEFAULT_FREQ_STOP);
            printf("  --step <MHz>        Frequency step in MHz (default: %d)\n", DEFAULT_FREQ_STEP);
            printf("  -r, --reset         Reset device and reload firmware\n");
            printf("  --firmware <path>   Firmware file (default: embedded)\n");
            printf("  -h, --help          Show this help\n");
            printf("\nSweeps bus frequencies from start to stop and reports pass/fail results.\n");
            printf("If start > stop, sweep is descending.\n");
            return 0;
        } else {
            fprintf(stderr, "Error: unknown option '%s'\n", argv[i]);
            return 1;
        }
    }

    /* Build frequency list */
    build_frequency_list();
    if (g_num_frequencies == 0) {
        fprintf(stderr, "Error: no frequencies to test\n");
        return 1;
    }

    /* Setup signal handler */
    signal(SIGINT, signal_handler);
#ifndef _WIN32
    signal(SIGTERM, signal_handler);
#endif

    printf("FX3 Bus Frequency Sweep Test%s\n", g_validate_mode ? " (counter validation)" : "");
    printf("============================%s\n\n", g_validate_mode ? "=======================" : "");

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
        fprintf(stderr, "Failed to initialize device\n");
        libusb_exit(g_usb_ctx);
        return 1;
    }

    /* Get firmware version */
    uint8_t major, minor;
    if (fx3_get_firmware_version(g_handle, &major, &minor) == 0) {
        printf("Firmware version: %d.%d\n", major, minor);
    }

    printf("Bus width: %d bits\n", bus_width);
    printf("Frequency sweep: %d -> %d MHz (step %d)\n", g_freq_start, g_freq_stop, g_freq_step);
    if (g_target_bytes >= 1024 * 1024) {
        printf("Target bytes per test: %llu MB\n", g_target_bytes / (1024 * 1024));
    } else {
        printf("Target bytes per test: %llu KB\n", g_target_bytes / 1024);
    }

    print_table_header(bus_width);

    /* Run tests at each frequency, printing rows as we go */
    int passed = 0, failed = 0;
    for (int i = 0; i < g_num_frequencies && !g_quit; i++) {
        g_results[i] = test_frequency(g_test_frequencies[i], bus_width);
        print_table_row(&g_results[i], bus_width);

        if (g_results[i].passed) passed++;
        else failed++;
    }

    /* Print table footer */
    print_table_footer(passed, failed, g_num_frequencies);

    /* Cleanup */
    fx3_close(g_handle);
    libusb_exit(g_usb_ctx);

    /* Return non-zero if any test failed */
    for (int i = 0; i < g_num_frequencies; i++) {
        if (!g_results[i].passed) return 1;
    }
    return 0;
}
