#ifndef BSP_DMA_POOL_H_
#define BSP_DMA_POOL_H_

#include <stdint.h>

/*
 * Shared DMA pool used by acquisition and benchmark.
 * Only one mode runs at a time, so buffers/descriptors can be reused.
 */
#define FX3_DMA_POOL_SOCKETS            2
#define FX3_DMA_POOL_BUFFERS_PER_SOCKET 5
#define FX3_DMA_POOL_COUNT              (FX3_DMA_POOL_SOCKETS * FX3_DMA_POOL_BUFFERS_PER_SOCKET)
#define FX3_DMA_POOL_BUFFER_SIZE        32768

typedef volatile uint8_t Fx3DmaPoolBuffer_t[FX3_DMA_POOL_BUFFER_SIZE];

typedef enum {
    FX3_DMA_POOL_OWNER_NONE = 0,
    FX3_DMA_POOL_OWNER_ACQUISITION = 1,
    FX3_DMA_POOL_OWNER_BENCHMARK = 2,
} Fx3DmaPoolOwner_t;

void Fx3DmaPoolInit(void);
int Fx3DmaPoolAcquire(Fx3DmaPoolOwner_t owner);
void Fx3DmaPoolRelease(Fx3DmaPoolOwner_t owner);

Fx3DmaPoolBuffer_t *Fx3DmaPoolBuffers(void);
uint16_t *Fx3DmaPoolDescriptors(void);
int Fx3DmaPoolBufferToIndex(uint32_t buffer_addr);

#endif /* BSP_DMA_POOL_H_ */
