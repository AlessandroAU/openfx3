/**
 * @file acquisition.c
 * @brief FX3 GPIF Continuous Streaming with Dual Socket Transfers
 *
 * Continuously samples data from GPIF bus into DMA buffers using two threads
 * that alternate to maintain seamless data flow to USB.
 *
 * Uses dual socket architecture where:
 * - Sockets 0-1 each have their own independent descriptor chains
 * - While one socket transfers data, the other loads its next descriptor
 * - This eliminates sample loss during thread/socket transitions
 *
 * Two state machine modes:
 * - Counter-based (states 0-2): For 8/16-bit bus, uses ADDR_CNT_HIT
 * - Watermark-based (states 3-4): For 32-bit bus, uses DMA_WM
 *
 * Assumes DMA is always ready (sufficient buffering). If DMA falls behind,
 * samples will be lost.
 */

#include <bsp/gpif.h>
#include <bsp/dma.h>
#include <bsp/usb.h>
#include <bsp/uart.h>
#include <bsp/util.h>
#include <bsp/irq.h>
#include <bsp/regaccess.h>
#include <bsp/dma_pool.h>
#include <rdb/uib.h>
#include <rdb/dma.h>
#include <rdb/vic.h>
#include <rdb/gpif.h>
#include <stdio.h>

/* Uncomment to enable UART debug output (costs ~1KB flash) */
// #define DEBUG_UART

#include "benchmark.h"
#include "command.h"

#define NUM_SOCKETS             FX3_DMA_POOL_SOCKETS
#define NUM_BUFFERS_PER_SOCKET  FX3_DMA_POOL_BUFFERS_PER_SOCKET
#define NUM_DMA_BUFFERS         FX3_DMA_POOL_COUNT
#define DMA_BUFFER_SIZE         FX3_DMA_POOL_BUFFER_SIZE

/* Stats counters - used by main.c */
volatile uint64_t acq_total_buffers = 0;  /* Buffers successfully produced by GPIF */
volatile uint32_t acq_stalls = 0;         /* STALL: socket waiting for descriptor (backpressure) */

/* Current bus configuration (set by start_acquisition) */
uint32_t acq_bus_freq_hz = 0;
uint32_t acq_sys_clk_hz = 0;
uint16_t acq_gpif_div = 0;
uint8_t acq_pll_fbdiv = 0;
uint8_t acq_bus_width = 0;

/*
 * GPIF DMA ISR - fires when producer sockets have problems
 */
static void Fx3GpifDmaIsr(void) __attribute__((isr("IRQ"), optimize("O3")));

static inline void __attribute__((optimize("O3"))) HandlePibSocket(uint32_t sck)
{
    const uint32_t intr = Fx3ReadReg32(sck + FX3_SCK_INTR);

    /* Count successful buffer commits */
    if (intr & FX3_SCK_INTR_PRODUCE_EVENT) {
        acq_total_buffers++;
        Fx3WriteReg32(sck + FX3_SCK_INTR, FX3_SCK_INTR_PRODUCE_EVENT);
    }

    /* Check for stall condition */
    if (intr & FX3_SCK_INTR_STALL) {
        acq_stalls++;
        Fx3WriteReg32(sck + FX3_SCK_INTR, FX3_SCK_INTR_STALL);
    }
}

static void Fx3GpifDmaIsr(void)
{
    for (unsigned i = 0; i < NUM_SOCKETS; i++)
        HandlePibSocket(FX3_PIB_DMA_SCK(i));

    /* EOI for VIC */
    Fx3WriteReg32(FX3_VIC_ADDRESS, 0);
}

static void EnablePibSocketAlerts(uint32_t sck)
{
    /* Clear any stale causes */
    Fx3WriteReg32(sck + FX3_SCK_INTR, ~0UL);

    /* Unmask produce event (for buffer counting) and stall condition */
    Fx3WriteReg32(sck + FX3_SCK_INTR_MASK,
        FX3_SCK_INTR_MASK_PRODUCE_EVENT |
        FX3_SCK_INTR_MASK_STALL);
}

static void EnableGpifDmaIrq(void)
{
    /* Install ISR into the VIC vector table */
    Fx3WriteReg32(FX3_VIC_VEC_ADDRESS + (FX3_IRQ_GPIF_DMA << 2),
                  (uint32_t)Fx3GpifDmaIsr);

    /* Enable the VIC line */
    Fx3SetReg32(FX3_VIC_INT_ENABLE, (1UL << FX3_IRQ_GPIF_DMA));
}

static void DisableGpifDmaIrq(void)
{
    Fx3ClearReg32(FX3_VIC_INT_ENABLE, (1UL << FX3_IRQ_GPIF_DMA));
}

/* DMA resources are shared via bsp/dma_pool.* */

/*
 * GPIF Waveform Functions
 *
 * Functions allow combining up to 4 lambda inputs (Fa, Fb, Fc, Fd) using
 * boolean logic to produce transition conditions f0 and f1.
 */
static const uint16_t functions[] = {
    [0] = 0U,                          /* Constant 0 (always false) */
    [1] = (uint16_t)~0U,               /* Constant 1 (always true) */
    [2] = FX3_GPIF_FUNCTION_Fa,        /* Pass through Fa */
    [3] = FX3_GPIF_FUNCTION_Fa & FX3_GPIF_FUNCTION_Fb,  /* Fa AND Fb */
};

/*
 * GPIF State Machine - Combined counter-based and watermark-based modes
 *
 * States 0-2: Counter-based mode for 8/16-bit bus
 *   State 0: Init - wait for DMA_RDY, go to state 1
 *   State 1: Thread 0 loop - sample+push+count, on DATA_CNT_HIT go to state 2
 *   State 2: Thread 1 loop - sample+push+count, on DATA_CNT_HIT go to state 1
 *   Counter uses RELOAD flag to auto-reset when limit is hit.
 *
 * States 3-4: Watermark-based mode for 32-bit bus
 *   State 3: Thread 0 - sample+push, on DMA_WM go to state 4
 *   State 4: Thread 1 - sample+push, on DMA_WM go to state 3
 *
 * Start at state 0 for 8/16-bit, state 3 for 32-bit.
 */
static const Fx3GpifWaveform_t waveforms[] = {
    /* ========== Counter-based mode (8/16-bit) ========== */

    /* State 0: Wait for DMA_RDY, then go to state 1
     * Counter is auto-loaded from registers when state machine starts */
    [0] = {
        .state = GPIF_STATE(0,
            FX3_GPIF_LAMBDA_INDEX_DMA_RDY, 0, 0, 0,
            2, 0,  /* f0 = Fa (DMA_RDY) */
            0, 0,  /* No sampling */
            0,     /* No beta - counter already loaded at SM start */
            0, 0),
        .left = 1, .right = 0
    },

    /* State 1: Thread 0 loop - sample+push+count, on DATA_CNT_HIT go to state 2 */
    [1] = {
        .state = GPIF_STATE(1,
            FX3_GPIF_LAMBDA_INDEX_DATA_CNT_HIT, 0, 0, 0,
            2, 0,  /* f0 = Fa (DATA_CNT_HIT) */
            FX3_GPIF_ALPHA_SAMPLE_DIN, FX3_GPIF_ALPHA_SAMPLE_DIN,
            FX3_GPIF_BETA_THREAD_0 | FX3_GPIF_BETA_WQ_PUSH | FX3_GPIF_BETA_COUNT_DATA,
            0, 0),
        .left = 2, .right = 1
    },

    /* State 2: Thread 1 loop - sample+push+count, on DATA_CNT_HIT go to state 1 */
    [2] = {
        .state = GPIF_STATE(2,
            FX3_GPIF_LAMBDA_INDEX_DATA_CNT_HIT, 0, 0, 0,
            2, 0,  /* f0 = Fa (DATA_CNT_HIT) */
            FX3_GPIF_ALPHA_SAMPLE_DIN, FX3_GPIF_ALPHA_SAMPLE_DIN,
            FX3_GPIF_BETA_THREAD_1 | FX3_GPIF_BETA_WQ_PUSH | FX3_GPIF_BETA_COUNT_DATA,
            0, 0),
        .left = 1, .right = 2
    },

    /* ========== Watermark-based mode (32-bit) ========== */

    /* State 3: Thread 0 - sample+push, on DMA_WM go to state 4 */
    [3] = {
        .state = GPIF_STATE(3,
            FX3_GPIF_LAMBDA_INDEX_DMA_WM, 0, 0, 0,
            2, 0,
            FX3_GPIF_ALPHA_SAMPLE_DIN, FX3_GPIF_ALPHA_SAMPLE_DIN,
            FX3_GPIF_BETA_THREAD_0 | FX3_GPIF_BETA_WQ_PUSH,
            0, 0),
        .left = 4, .right = 3
    },

    /* State 4: Thread 1 - sample+push, on DMA_WM go to state 3 */
    [4] = {
        .state = GPIF_STATE(4,
            FX3_GPIF_LAMBDA_INDEX_DMA_WM, 0, 0, 0,
            2, 0,
            FX3_GPIF_ALPHA_SAMPLE_DIN, FX3_GPIF_ALPHA_SAMPLE_DIN,
            FX3_GPIF_BETA_THREAD_1 | FX3_GPIF_BETA_WQ_PUSH,
            0, 0),
        .left = 3, .right = 4
    },
};

/* Starting state indices */
#define STATE_COUNTER_START     0   /* For 8/16-bit bus */
#define STATE_WATERMARK_START   3   /* For 32-bit bus */

#define WATERMARK_LEVEL    1   /* Trigger DMA_WM when buffer nearly full */
#define BURST_SIZE_LOG2    14  /* 2^14 = 16384 words (for watermark mode) */

/*
 * GPIF register configuration
 * Unspecified fields default to zero.
 */
static Fx3GpifRegisters_t gpif_registers = {
    .config = (FX3_GPIF_CONFIG_ENABLE |
               FX3_GPIF_CONFIG_THREAD_IN_STATE |  /* Thread selection from state machine */
               FX3_GPIF_CONFIG_SYNC |             /* Synchronous sampling mode */
               FX3_GPIF_CONFIG_CLK_SOURCE |       /* Internal clock (can be changed) */
               FX3_GPIF_CONFIG_CLK_OUT),          /* Output clock on CLK pin */

    .bus_config = (3UL << FX3_GPIF_BUS_CONFIG_BUS_WIDTH_SHIFT),  /* 32-bit bus */

    .ad_config = (FX3_GPIF_OEN_CFG_INPUT << FX3_GPIF_AD_CONFIG_A_OEN_CFG_SHIFT) |
                 (FX3_GPIF_OEN_CFG_INPUT << FX3_GPIF_AD_CONFIG_DQ_OEN_CFG_SHIFT),

    /* Thread config: ENABLE, burst size, thread N -> socket N, watermark level */
    .thread_config = {
        [0] = FX3_GPIF_THREAD_CONFIG_ENABLE
            | (BURST_SIZE_LOG2 << FX3_GPIF_THREAD_CONFIG_BURST_SIZE_SHIFT)
            | (0UL << FX3_GPIF_THREAD_CONFIG_THREAD_SOCK_SHIFT)
            | (WATERMARK_LEVEL << FX3_GPIF_THREAD_CONFIG_WATERMARK_SHIFT),

        [1] = FX3_GPIF_THREAD_CONFIG_ENABLE
            | (BURST_SIZE_LOG2 << FX3_GPIF_THREAD_CONFIG_BURST_SIZE_SHIFT)
            | (1UL << FX3_GPIF_THREAD_CONFIG_THREAD_SOCK_SHIFT)
            | (WATERMARK_LEVEL << FX3_GPIF_THREAD_CONFIG_WATERMARK_SHIFT),
    },
};

/*
 * Setup 2-socket descriptor chains for seamless transfers.
 *
 * Buffer allocation (with NUM_BUFFERS_PER_SOCKET=2, NUM_SOCKETS=2):
 *   Socket 0: buffers 0, 2
 *   Socket 1: buffers 1, 3
 *
 * Each socket has its own circular chain of descriptors.
 * The consumer (USB) sees all buffers in order: 0, 1, 2, 3, ...
 */
static void setup_descriptors(void)
{
    Fx3DmaPoolBuffer_t *buffers = Fx3DmaPoolBuffers();
    uint16_t *descriptors = Fx3DmaPoolDescriptors();

    for (unsigned i = 0; i < NUM_DMA_BUFFERS; i++) {
        unsigned socket = i % NUM_SOCKETS;

        /* Producer chain: each socket chains to its own next buffer (skip by NUM_SOCKETS) */
        unsigned prod_next = (i + NUM_SOCKETS) % NUM_DMA_BUFFERS;

        /* Consumer chain: all buffers chain sequentially */
        unsigned cons_next = (i + 1) % NUM_DMA_BUFFERS;

        Fx3DmaFillDescriptorThrough(
            FX3_PIB_DMA_SCK(socket),
            FX3_UIB_DMA_SCK(2),
            descriptors[i],
            (void*)buffers[i],
            DMA_BUFFER_SIZE,
            descriptors[prod_next],
            descriptors[cons_next]);
    }
}

void stop_acquisition(void)
{
    Fx3GpifStop();
    DisableGpifDmaIrq();

    for (unsigned i = 0; i < NUM_SOCKETS; i++)
        Fx3DmaAbortSocket(FX3_PIB_DMA_SCK(i));
    Fx3DmaAbortSocket(FX3_UIB_DMA_SCK(2));

    Fx3GpifPibStop();
    Fx3UsbFlushInEndpoint(2);
    Fx3UsbSetEpNak(2, 1);
    Fx3UsbResetEpDataToggle(2);

    Fx3DmaPoolRelease(FX3_DMA_POOL_OWNER_ACQUISITION);
}

uint8_t is_acquisition_active(void)
{
    return (Fx3GpifGetStat(NULL) == FX3_GPIF_RUNNING) ? 1 : 0;
}

uint32_t get_acq_drop_count(void)
{
    return acq_stalls;
}

uint64_t get_acq_total_bytes(void)
{
    return acq_total_buffers * DMA_BUFFER_SIZE;
}

void reset_drop_stats(void)
{
    acq_total_buffers = 0;
    acq_stalls = 0;
    gpif_error_count = 0;
}

void start_acquisition(uint16_t clock_divisor_x2, const struct acq_config *config)
{
    /* Stop any running benchmark or acquisition */
    if (is_benchmark_active()) {stop_benchmark();}
    if (is_acquisition_active()) {stop_acquisition();}

    Fx3DmaPoolInit();

    if (!Fx3DmaPoolAcquire(FX3_DMA_POOL_OWNER_ACQUISITION)) {
        return;
    }

    /* Configure bus width: (bits/8 - 1) for register value */
    gpif_registers.bus_config &= ~FX3_GPIF_BUS_CONFIG_BUS_WIDTH_MASK;
    gpif_registers.bus_config |= ((config->bus_width >> 3) - 1) << FX3_GPIF_BUS_CONFIG_BUS_WIDTH_SHIFT;

    /* Select state machine mode based on bus width */
    uint8_t start_state;
    if (config->bus_width == 32) {
        /* 32-bit: Use watermark-based mode (states 3-4) */
        start_state = STATE_WATERMARK_START;
        /* No data counter needed */
        gpif_registers.data_count_config = 0;
    } else {
        /* 8/16-bit: Use counter-based mode (states 0-2) */
        start_state = STATE_COUNTER_START;
        /* Set data counter: count up from 0 to limit with auto-reload
         * DATA_CNT_HIT fires when counter == limit, then counter reloads.
         * Each buffer needs exactly samples_per_buffer pushes.
         * limit = samples_per_buffer - 1 means counter goes 0..N-1 (N values) */
        uint32_t bytes_per_sample = config->bus_width >> 3;
        uint32_t samples_per_buffer = DMA_BUFFER_SIZE / bytes_per_sample;
        gpif_registers.data_count_config = FX3_GPIF_DATA_COUNT_CONFIG_ENABLE
                                         | FX3_GPIF_DATA_COUNT_CONFIG_RELOAD
                                         | FX3_GPIF_DATA_COUNT_CONFIG_DOWN_UP  /* Count up */
                                         | (1UL << FX3_GPIF_DATA_COUNT_CONFIG_INCREMENT_SHIFT);
        gpif_registers.data_count_reset = 0;  /* Start from 0 */
        gpif_registers.data_count_limit = samples_per_buffer - 1;
    }

#ifdef DEBUG_UART
    {
        char buf[80];
        snprintf(buf, sizeof(buf), "GPIF: bus_width=%u start_state=%u\n",
                 config->bus_width, start_state);
        Fx3UartTxString(buf);
    }
#endif

    /* Configure clock source, speed mode, polarity, and other options */
    gpif_registers.config &= ~(FX3_GPIF_CONFIG_CLK_SOURCE | FX3_GPIF_CONFIG_CLK_OUT |
                               FX3_GPIF_CONFIG_SYNC_SPEED | FX3_GPIF_CONFIG_CLK_INVERT |
                               FX3_GPIF_CONFIG_DDR_MODE | FX3_GPIF_CONFIG_ENDIAN);
    if (config->internal_clk) {
        gpif_registers.config |= FX3_GPIF_CONFIG_CLK_SOURCE;
        if (config->clk_out) {
            gpif_registers.config |= FX3_GPIF_CONFIG_CLK_OUT;
        }
    }
    /* SYNC_SPEED=1 for frequencies >50MHz (sys_clk ~480MHz, div_x2 < 10 means >48MHz) */
    if (clock_divisor_x2 < 10) {
        gpif_registers.config |= FX3_GPIF_CONFIG_SYNC_SPEED;
    }
    if (config->clk_invert) {
        gpif_registers.config |= FX3_GPIF_CONFIG_CLK_INVERT;
    }
    if (config->ddr) {
        gpif_registers.config |= FX3_GPIF_CONFIG_DDR_MODE;
    }
    if (config->endian) {
        gpif_registers.config |= FX3_GPIF_CONFIG_ENDIAN;
    }
    
    setup_descriptors();

    /* Start GPIF clock and configure state machine */
    Fx3GpifPibStart(clock_divisor_x2);
    Fx3GpifConfigure(waveforms, sizeof(waveforms)/sizeof(waveforms[0]),
                     functions, sizeof(functions)/sizeof(functions[0]),
                     &gpif_registers);

    uint16_t *descriptors = Fx3DmaPoolDescriptors();
    Fx3DmaStartConsumer(FX3_UIB_DMA_SCK(2), descriptors[0], 0, 0);

    for (unsigned i = 0; i < NUM_SOCKETS; i++)
        Fx3DmaStartProducer(FX3_PIB_DMA_SCK(i), descriptors[i], 0, 0);

    /* Enable USB endpoint - must be ready before GPIF pushes data */
    Fx3UsbSetEpNak(2, 0);

    EnableGpifDmaIrq();
    reset_drop_stats();

    /* Enable socket alerts (clears any pending interrupts) */
    for (unsigned i = 0; i < NUM_SOCKETS; i++)
        EnablePibSocketAlerts(FX3_PIB_DMA_SCK(i));

    /* Start GPIF - state selected based on bus width */
    Fx3GpifStart(start_state, 0);
}

void poll_acquisition(void)
{
#ifdef DEBUG_UART
    if (Fx3GpifGetStat(NULL) == FX3_GPIF_PAUSED) {
        Fx3UartTxString("GPIF paused\n");
    }

    char buf[96];
    sprintf(buf, "Acq Status: Buffers=%llu Bytes=%llu Stalls=%lu\n",
            acq_total_buffers, get_acq_total_bytes(), (unsigned long)acq_stalls);
    Fx3UartTxString(buf);
#endif
}
