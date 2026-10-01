#include "hydrox/platform/nuttx/nuttx_clock.h"

#include <ctime>

namespace hydrox::platform::nuttx
{
    MonotonicTimeUs NuttxClock::now_us() const noexcept
    {
        timespec value{};
        if (clock_gettime(CLOCK_MONOTONIC, &value) != 0)
            return 0;

        return static_cast<MonotonicTimeUs>(value.tv_sec) * 1000000ULL +
               static_cast<MonotonicTimeUs>(value.tv_nsec / 1000);
    }
}
