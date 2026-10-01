/****************************************************************************
 * HydroX FMUv6C USB device-mode board hooks.
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_USBDEV
#include <stdbool.h>
#include <nuttx/usb/usbdev.h>

void stm32_usbsuspend(struct usbdev_s *dev, bool resume)
{
  /* USB-C is a device-only diagnostics port with no switchable board power. */

  (void)dev;
  (void)resume;
}
#endif
