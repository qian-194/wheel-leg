#ifndef MEMORY_REGIONS_H
#define MEMORY_REGIONS_H

#include <stddef.h>
#include <stdint.h>
#include "stm32h7xx_hal.h"

#define ITCM_FUNC     __attribute__((section(".itcm_text"), noinline))
#define DTCM_DATA     __attribute__((section(".dtcm_data")))
#define DTCM_BSS      __attribute__((section(".dtcm_bss")))
#define RAM_D2_DATA   __attribute__((section(".ram_d2")))
#define RAM_D3_DATA   __attribute__((section(".ram_d3")))
#define DMA_BUFFER    __attribute__((section(".dma_buffer"), aligned(32)))
#define CACHE_ALIGNED __attribute__((aligned(32)))

void MPU_Config(void);
void CPU_CACHE_Enable(void);

void Cache_CleanByAddr(const void *addr, size_t size);
void Cache_InvalidateByAddr(const void *addr, size_t size);
void Cache_CleanInvalidateByAddr(const void *addr, size_t size);

uint8_t Memory_IsDmaAccessible(const void *addr);

#endif /* MEMORY_REGIONS_H */
