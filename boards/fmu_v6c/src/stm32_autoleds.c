#include <nuttx/config.h>
#include <stdbool.h>
#include <arch/board/board.h>
#include "stm32_gpio.h"

#ifdef CONFIG_ARCH_LEDS
void board_autoled_initialize(void)
{
  stm32_configgpio(GPIO_nLED_RED);
  stm32_configgpio(GPIO_nLED_BLUE);
}

void board_autoled_on(int led)
{
  switch (led)
    {
      case LED_STARTED:
      case LED_HEAPALLOCATE:
      case LED_IRQSENABLED:
        stm32_gpiowrite(GPIO_nLED_BLUE, false);
        break;
      case LED_PANIC:
      case LED_ASSERTION:
        stm32_gpiowrite(GPIO_nLED_RED, false);
        break;
      default:
        break;
    }
}

void board_autoled_off(int led)
{
  switch (led)
    {
      case LED_STARTED:
      case LED_HEAPALLOCATE:
      case LED_IRQSENABLED:
        stm32_gpiowrite(GPIO_nLED_BLUE, true);
        break;
      case LED_PANIC:
      case LED_ASSERTION:
        stm32_gpiowrite(GPIO_nLED_RED, true);
        break;
      default:
        break;
    }
}
#endif