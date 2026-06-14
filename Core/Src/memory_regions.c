#include "memory_regions.h"

#define H7_ITCM_BASE        0x00000000UL
#define H7_ITCM_SIZE        MPU_REGION_SIZE_64KB
#define H7_DTCM_BASE        0x20000000UL
#define H7_DTCM_END         0x20020000UL
#define H7_DTCM_SIZE        MPU_REGION_SIZE_128KB
#define H7_AXI_SRAM_BASE    0x24000000UL
#define H7_AXI_SRAM_SIZE    MPU_REGION_SIZE_128KB
#define H7_D2_SRAM_BASE     0x30000000UL
#define H7_D2_SRAM_SIZE     MPU_REGION_SIZE_32KB
#define H7_D3_SRAM_BASE     0x38000000UL
#define H7_D3_SRAM_SIZE     MPU_REGION_SIZE_16KB
#define H7_CACHE_LINE_SIZE  32UL

static void ConfigureMPURegion(uint32_t number,
                               uint32_t base_address,
                               uint32_t size,
                               uint32_t tex,
                               uint32_t cacheable,
                               uint32_t bufferable,
                               uint32_t shareable,
                               uint32_t instruction_access)
{
  MPU_Region_InitTypeDef mpu_init = {0};

  mpu_init.Enable = MPU_REGION_ENABLE;
  mpu_init.Number = number;
  mpu_init.BaseAddress = base_address;
  mpu_init.Size = size;
  mpu_init.SubRegionDisable = 0x00;
  mpu_init.TypeExtField = tex;
  mpu_init.AccessPermission = MPU_REGION_FULL_ACCESS;
  mpu_init.DisableExec = instruction_access;
  mpu_init.IsShareable = shareable;
  mpu_init.IsCacheable = cacheable;
  mpu_init.IsBufferable = bufferable;

  HAL_MPU_ConfigRegion(&mpu_init);
}

void MPU_Config(void)
{
  HAL_MPU_Disable();

  ConfigureMPURegion(MPU_REGION_NUMBER0,
                     H7_ITCM_BASE,
                     H7_ITCM_SIZE,
                     MPU_TEX_LEVEL0,
                     MPU_ACCESS_NOT_CACHEABLE,
                     MPU_ACCESS_NOT_BUFFERABLE,
                     MPU_ACCESS_NOT_SHAREABLE,
                     MPU_INSTRUCTION_ACCESS_ENABLE);

  ConfigureMPURegion(MPU_REGION_NUMBER1,
                     H7_DTCM_BASE,
                     H7_DTCM_SIZE,
                     MPU_TEX_LEVEL0,
                     MPU_ACCESS_NOT_CACHEABLE,
                     MPU_ACCESS_NOT_BUFFERABLE,
                     MPU_ACCESS_NOT_SHAREABLE,
                     MPU_INSTRUCTION_ACCESS_DISABLE);

  ConfigureMPURegion(MPU_REGION_NUMBER2,
                     H7_AXI_SRAM_BASE,
                     H7_AXI_SRAM_SIZE,
                     MPU_TEX_LEVEL0,
                     MPU_ACCESS_CACHEABLE,
                     MPU_ACCESS_BUFFERABLE,
                     MPU_ACCESS_NOT_SHAREABLE,
                     MPU_INSTRUCTION_ACCESS_DISABLE);

  ConfigureMPURegion(MPU_REGION_NUMBER3,
                     H7_D2_SRAM_BASE,
                     H7_D2_SRAM_SIZE,
                     MPU_TEX_LEVEL1,
                     MPU_ACCESS_NOT_CACHEABLE,
                     MPU_ACCESS_NOT_BUFFERABLE,
                     MPU_ACCESS_SHAREABLE,
                     MPU_INSTRUCTION_ACCESS_DISABLE);

  ConfigureMPURegion(MPU_REGION_NUMBER4,
                     H7_D3_SRAM_BASE,
                     H7_D3_SRAM_SIZE,
                     MPU_TEX_LEVEL1,
                     MPU_ACCESS_NOT_CACHEABLE,
                     MPU_ACCESS_NOT_BUFFERABLE,
                     MPU_ACCESS_SHAREABLE,
                     MPU_INSTRUCTION_ACCESS_DISABLE);

  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

void CPU_CACHE_Enable(void)
{
  SCB_EnableICache();
  SCB_EnableDCache();
}

static size_t GetAlignedCacheRange(const void *addr, size_t size, uintptr_t *aligned_start)
{
  uintptr_t start = (uintptr_t)addr;
  uintptr_t end = start + size;

  start &= ~(uintptr_t)(H7_CACHE_LINE_SIZE - 1UL);
  end = (end + H7_CACHE_LINE_SIZE - 1UL) & ~(uintptr_t)(H7_CACHE_LINE_SIZE - 1UL);

  *aligned_start = start;
  return (size_t)(end - start);
}

void Cache_CleanByAddr(const void *addr, size_t size)
{
  uintptr_t aligned_start;
  size_t aligned_size;

  if ((addr == NULL) || (size == 0U) || ((SCB->CCR & SCB_CCR_DC_Msk) == 0U))
  {
    return;
  }

  aligned_size = GetAlignedCacheRange(addr, size, &aligned_start);
  SCB_CleanDCache_by_Addr((uint32_t *)aligned_start, (int32_t)aligned_size);
}

void Cache_InvalidateByAddr(const void *addr, size_t size)
{
  uintptr_t aligned_start;
  size_t aligned_size;

  if ((addr == NULL) || (size == 0U) || ((SCB->CCR & SCB_CCR_DC_Msk) == 0U))
  {
    return;
  }

  aligned_size = GetAlignedCacheRange(addr, size, &aligned_start);
  SCB_InvalidateDCache_by_Addr((uint32_t *)aligned_start, (int32_t)aligned_size);
}

void Cache_CleanInvalidateByAddr(const void *addr, size_t size)
{
  uintptr_t aligned_start;
  size_t aligned_size;

  if ((addr == NULL) || (size == 0U) || ((SCB->CCR & SCB_CCR_DC_Msk) == 0U))
  {
    return;
  }

  aligned_size = GetAlignedCacheRange(addr, size, &aligned_start);
  SCB_CleanInvalidateDCache_by_Addr((uint32_t *)aligned_start, (int32_t)aligned_size);
}

uint8_t Memory_IsDmaAccessible(const void *addr)
{
  uintptr_t value = (uintptr_t)addr;

  if ((value < 0x00010000UL) || ((value >= H7_DTCM_BASE) && (value < H7_DTCM_END)))
  {
    return 0U;
  }

  return 1U;
}
