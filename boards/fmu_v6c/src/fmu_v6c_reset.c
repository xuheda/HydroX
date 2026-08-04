#include <nuttx/config.h>
#include <stdint.h>
#include "arm_internal.h"
#include "hardware/stm32_rcc.h"
#include "fmu_v6c.h"

static uint32_t g_reset_reason;

void fmu_v6c_capture_reset_reason(void)
{
  g_reset_reason = getreg32(STM32_RCC_RSR);
  putreg32(g_reset_reason | RCC_RSR_RMVF, STM32_RCC_RSR);
}

uint32_t fmu_v6c_reset_reason(void)
{
  return g_reset_reason;
}