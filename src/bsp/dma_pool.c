#include <bsp/dma_pool.h>

#include <bsp/dma.h>
#include <bsp/irq.h>
#include <bsp/uart.h>

/* Uncomment to enable UART debug output (costs ~1KB flash) */
// #define DEBUG_UART

static Fx3DmaPoolOwner_t dma_pool_owner = FX3_DMA_POOL_OWNER_NONE;
static uint8_t dma_pool_initialized = 0;

static Fx3DmaPoolBuffer_t dma_pool_buffers[FX3_DMA_POOL_COUNT]
    __attribute__((aligned(FX3_DMA_POOL_BUFFER_SIZE)));

static uint16_t dma_pool_descriptors[FX3_DMA_POOL_COUNT];

void Fx3DmaPoolInit(void)
{
    if (dma_pool_initialized)
        return;

    for (unsigned i = 0; i < FX3_DMA_POOL_COUNT; i++) {
        uint16_t d = Fx3DmaAllocateDescriptor();
        if (!d) {
#ifdef DEBUG_UART
            Fx3UartTxString("ERROR: DMA descriptor allocation failed\n");
#endif
            break;
        }
        dma_pool_descriptors[i] = d;
    }

    dma_pool_initialized = 1;
}

int Fx3DmaPoolAcquire(Fx3DmaPoolOwner_t owner)
{
    int ok = 0;

    Fx3IrqDisableInterrupts();
    if ((dma_pool_owner == FX3_DMA_POOL_OWNER_NONE) || (dma_pool_owner == owner)) {
        dma_pool_owner = owner;
        ok = 1;
    }
    Fx3IrqEnableInterrupts();

    return ok;
}

void Fx3DmaPoolRelease(Fx3DmaPoolOwner_t owner)
{
    Fx3IrqDisableInterrupts();
    if (dma_pool_owner == owner)
        dma_pool_owner = FX3_DMA_POOL_OWNER_NONE;
    Fx3IrqEnableInterrupts();
}

Fx3DmaPoolBuffer_t *Fx3DmaPoolBuffers(void)
{
    return dma_pool_buffers;
}

uint16_t *Fx3DmaPoolDescriptors(void)
{
    return dma_pool_descriptors;
}

int Fx3DmaPoolBufferToIndex(uint32_t buffer_addr)
{
    uint32_t base = (uint32_t)dma_pool_buffers;
    if (buffer_addr < base)
        return -1;
    uint32_t offset = buffer_addr - base;
    unsigned idx = offset / FX3_DMA_POOL_BUFFER_SIZE;
    if (idx >= FX3_DMA_POOL_COUNT)
        return -1;
    return (int)idx;
}
