/**
 * main_stm32.cpp — STM32/Pixhawk 6C hardware entry point (stub implementation)
 *
 * Compilation target: hydrox_pixhawk6c (cross-compiled, -DSTM32H7xx, analogous to px4_fmu-v6c)
 * Porting note: Replace TcpTransport -> UartTransport, other logic is exactly the same.
 * TODO: Add FreeRTOS task, HAL initialization, watchdog.
 */
#ifdef STM32H7xx

#include "main.h" // CubeMX generated
#include "mavlink_hil.h"
#include "ekf.h"
#include "gnc/gnc_controller.h"
// uart_transport is provided during STM32 compilation

using namespace hydrox;

extern "C" void HydroX_main(void)
{
    // Create EKF + GNC objects identical to SITL, change Transport to UartTransport
    // (consistent with main_sitl.cpp logic, only Transport differs)
    for (;;)
    {
        HAL_Delay(1000); // TODO: Implement control loop
    }
}

#endif // STM32H7xx
