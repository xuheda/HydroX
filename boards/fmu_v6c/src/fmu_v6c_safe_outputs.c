#include <nuttx/config.h>
#include <arch/board/board.h>
#include "stm32_gpio.h"
#include "fmu_v6c.h"

static volatile bool g_outputs_inhibited;

void fmu_v6c_inhibit_outputs_early(void)
{
  const uint32_t safe_pins[] =
  {
    GPIO_FMU_PWM1_SAFE, GPIO_FMU_PWM2_SAFE,
    GPIO_FMU_PWM3_SAFE, GPIO_FMU_PWM4_SAFE,
    GPIO_FMU_PWM5_SAFE, GPIO_FMU_PWM6_SAFE,
    GPIO_FMU_PWM7_SAFE, GPIO_FMU_PWM8_SAFE
  };

  g_outputs_inhibited = false;
  for (unsigned int i = 0; i < sizeof(safe_pins) / sizeof(safe_pins[0]); ++i)
    {
      if (stm32_configgpio(safe_pins[i]) < 0)
        {
          return;
        }
    }

  /* HITL defconfig does not enable PWM, DShot, CAN actuator or USART6/PX4IO. */
  g_outputs_inhibited = true;
}

bool fmu_v6c_outputs_are_inhibited(void)
{
  return g_outputs_inhibited;
}