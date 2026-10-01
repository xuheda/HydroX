#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <stdint.h>
#include "arm_internal.h"
#include "hardware/stm32_pwr.h"
#include "hardware/stm32_rcc.h"
#include "hardware/stm32_rtcc.h"
#include "fmu_v6c.h"

#define PX4_BOOT_RTC_SIGNATURE UINT32_C(0xb007b007)

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

void fmu_v6c_reboot_to_bootloader(void)
{
  /* PX4's resident STM32 bootloader consumes this signature from RTC BK0R,
   * clears it, and remains in the uploader instead of booting the app. */

  modifyreg32(STM32_PWR_CR1, 0, PWR_CR1_DBP);
  while ((getreg32(STM32_PWR_CR1) & PWR_CR1_DBP) == 0)
    {
    }
  putreg32(PX4_BOOT_RTC_SIGNATURE, STM32_RTC_BK0R);
  modifyreg32(STM32_PWR_CR1, PWR_CR1_DBP, 0);
  up_systemreset();
}
