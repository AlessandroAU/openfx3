/**
 * @file benchmark.c
 * @brief FX3 USB Benchmark Mode
 *
 * Continuously sends pre-filled DMA buffers to USB as fast as possible
 * without GPIF involvement. Used for measuring maximum USB throughput.
 */

#include <bsp/dma.h>
#include <bsp/usb.h>
#include <bsp/uart.h>
#include <bsp/regaccess.h>
#include <bsp/cache.h>
#include <bsp/irq.h>
#include <bsp/dma_pool.h>
#include <rdb/dma.h>
#include <rdb/vic.h>

#include "acquisition.h"

//65520

#define BENCHMARK_BUFFER_SIZE   FX3_DMA_POOL_BUFFER_SIZE
#define BENCHMARK_NUM_BUFFERS   12

/* Stats counter */
volatile uint64_t benchmark_total_buffers = 0;

/* Internal state */
static volatile uint8_t benchmark_active = 0;
static volatile uint32_t benchmark_counter = 0;

static void setup_descriptors_benchmark(void)
{
    Fx3DmaPoolBuffer_t *buffers = Fx3DmaPoolBuffers();
    uint16_t *descriptors = Fx3DmaPoolDescriptors();

    for (unsigned i = 0; i < BENCHMARK_NUM_BUFFERS; i++) {
        uint32_t *buf32 = (uint32_t *)buffers[i];
        buf32[0] = benchmark_counter++;
        for (unsigned j = 1; j < BENCHMARK_BUFFER_SIZE / 4; j++) {
            buf32[j] = 0xDEADBEEF;
        }

        unsigned next = (i + 1) % BENCHMARK_NUM_BUFFERS;
        Fx3DmaFillDescriptorRead(FX3_UIB_DMA_SCK(2), descriptors[i],
                                 (const volatile void *)buffers[i],
                                 BENCHMARK_BUFFER_SIZE, descriptors[next]);
    }

    Fx3CacheCleanDCache();
}

static void __attribute__((optimize("O3"))) refill_descriptors_benchmark(void)
{
    Fx3DmaPoolBuffer_t *buffers = Fx3DmaPoolBuffers();
    uint16_t *descriptors = Fx3DmaPoolDescriptors();

    for (unsigned i = 0; i < BENCHMARK_NUM_BUFFERS; i++) {
        volatile struct Fx3DmaDescriptor *desc = FX3_DMA_DESCRIPTOR(descriptors[i]);
        if (desc->dscr_size & FX3_DSCR_SIZE_BUFFER_OCCUPIED)
            continue;

        volatile uint32_t *buf32 = (volatile uint32_t *)buffers[i];
        buf32[0] = benchmark_counter++;
        Fx3CacheCleanDCacheEntry((void *)buf32);

        desc->dscr_size = (BENCHMARK_BUFFER_SIZE << FX3_DSCR_SIZE_BYTE_COUNT_SHIFT) |
                          ((BENCHMARK_BUFFER_SIZE + 15) & FX3_DSCR_SIZE_BUFFER_SIZE_MASK) |
                          FX3_DSCR_SIZE_BUFFER_OCCUPIED;
        Fx3CacheCleanDCacheEntry(desc);

        /* Commit this descriptor as produced by CPU */
        Fx3WriteReg32(FX3_UIB_DMA_SCK(2) + FX3_EVENT,
                      (descriptors[i] << FX3_EVENT_ACTIVE_DSCR_SHIFT));
    }
}

/*
 * USB DMA ISR for benchmark mode - re-commit buffer after it's consumed.
 *
 * Writes incrementing counter to first 4 bytes for validation, then re-commits.
 */
static void Fx3UsbBenchmarkIsr(void) __attribute__((isr("IRQ"), optimize("O3")));
static void Fx3UsbBenchmarkIsr(void)
{
    uint32_t intr = Fx3ReadReg32(FX3_UIB_DMA_SCK(2) + FX3_SCK_INTR);

    if (intr & FX3_SCK_INTR_CONSUME_EVENT) {
        benchmark_total_buffers++;

        refill_descriptors_benchmark();

        Fx3WriteReg32(FX3_UIB_DMA_SCK(2) + FX3_SCK_INTR, FX3_SCK_INTR_CONSUME_EVENT);
    }

    if (intr & ~FX3_SCK_INTR_CONSUME_EVENT) {
        Fx3WriteReg32(FX3_UIB_DMA_SCK(2) + FX3_SCK_INTR, intr & ~FX3_SCK_INTR_CONSUME_EVENT);
    }

    Fx3WriteReg32(FX3_VIC_ADDRESS, 0);
}

static void EnableUsbBenchmarkIrq(void)
{
    Fx3WriteReg32(FX3_UIB_DMA_SCK(2) + FX3_SCK_INTR, ~0UL);
    Fx3WriteReg32(FX3_UIB_DMA_SCK(2) + FX3_SCK_INTR_MASK,
                  FX3_SCK_INTR_MASK_CONSUME_EVENT);

    Fx3WriteReg32(FX3_VIC_VEC_ADDRESS + (FX3_IRQ_USB_DMA << 2),
                  (uint32_t)Fx3UsbBenchmarkIsr);
    Fx3SetReg32(FX3_VIC_INT_ENABLE, (1UL << FX3_IRQ_USB_DMA));
}

static void DisableUsbBenchmarkIrq(void)
{
    Fx3ClearReg32(FX3_VIC_INT_ENABLE, (1UL << FX3_IRQ_USB_DMA));
    Fx3WriteReg32(FX3_UIB_DMA_SCK(2) + FX3_SCK_INTR_MASK, 0);
}

void stop_benchmark(void)
{
    DisableUsbBenchmarkIrq();
    Fx3DmaAbortSocket(FX3_UIB_DMA_SCK(2));
    Fx3UsbFlushInEndpoint(2);
    Fx3UsbSetEpNak(2, 1);
    Fx3DmaPoolRelease(FX3_DMA_POOL_OWNER_BENCHMARK);
    benchmark_active = 0;
}

uint8_t is_benchmark_active(void)
{
    return benchmark_active;
}

void start_benchmark(void)
{
    Fx3DmaPoolInit();

    /* Stop any running benchmark or acquisition */
    if (is_benchmark_active()) {stop_benchmark();}
    if (is_acquisition_active()) {stop_acquisition();}

    if (!Fx3DmaPoolAcquire(FX3_DMA_POOL_OWNER_BENCHMARK)) {
        return;
    }

    /* Stop any running benchmark or acquisition */
    if (is_benchmark_active()) {stop_benchmark();}
    if (is_acquisition_active()) {stop_acquisition();}

    benchmark_total_buffers = 0;
    benchmark_active = 1;

    /* Setup descriptor ring with pre-filled data */
    setup_descriptors_benchmark();

    uint16_t *descriptors = Fx3DmaPoolDescriptors();

    /* Start USB consumer socket */
    Fx3DmaTransferStart(FX3_UIB_DMA_SCK(2), descriptors[0],
            FX3_SCK_STATUS_SUSP_TRANS	|
            FX3_SCK_STATUS_EN_CONS_EVENTS |
            FX3_SCK_STATUS_TRUNCATE |
            FX3_SCK_STATUS_AVL_ENABLE,
            0, 0);

    /* Enable USB endpoint */
    Fx3UsbSetEpNak(2, 0);

    /* Enable interrupt to re-commit buffer after consumption */
    EnableUsbBenchmarkIrq();

    /* Send initial produce events for all descriptors */
    for (unsigned i = 0; i < BENCHMARK_NUM_BUFFERS; i++) {
        Fx3WriteReg32(FX3_UIB_DMA_SCK(2) + FX3_EVENT,
                      (descriptors[i] << FX3_EVENT_ACTIVE_DSCR_SHIFT));
    }
}
