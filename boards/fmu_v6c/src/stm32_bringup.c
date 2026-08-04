#include <nuttx/config.h>
#include <syslog.h>
#include <nuttx/fs/fs.h>
#ifdef CONFIG_CDCACM
#  include <nuttx/usb/cdcacm.h>
#endif
#include <arch/board/board.h>
#include "stm32_wdg.h"
#include "fmu_v6c.h"

int fmu_v6c_bringup(void)
{
  int ret = 0;

#ifdef CONFIG_FS_PROCFS
  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "FMUv6C: procfs mount failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_STM32_IWDG
  stm32_iwdginitialize("/dev/watchdog0", STM32_LSI_FREQUENCY);
#endif

#if defined(CONFIG_CDCACM) && !defined(CONFIG_CDCACM_CONSOLE) && \
    !defined(CONFIG_CDCACM_COMPOSITE)
  ret = cdcacm_initialize(0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "FMUv6C: CDC ACM initialization failed: %d\n", ret);
    }
#endif

  syslog(LOG_INFO, "FMUv6C: reset=0x%08lx outputs_inhibited=%d\n",
         (unsigned long)fmu_v6c_reset_reason(),
         fmu_v6c_outputs_are_inhibited() ? 1 : 0);
  return 0;
}