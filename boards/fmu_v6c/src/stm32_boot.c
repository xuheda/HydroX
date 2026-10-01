#include <nuttx/config.h>
#include <nuttx/board.h>
#include <arch/board/board.h>
#include "fmu_v6c.h"

void stm32_boardinitialize(void)
{
  /* These two operations happen before serial, timers or applications. */
  fmu_v6c_capture_reset_reason();
  fmu_v6c_inhibit_outputs_early();
#ifdef CONFIG_ARCH_LEDS
  board_autoled_initialize();
#endif
}

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
  (void)fmu_v6c_bringup();
}
#endif