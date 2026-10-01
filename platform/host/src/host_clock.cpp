#include "hydrox/platform/host/host_clock.h"

#include <chrono>

namespace hydrox::platform::host
{
    MonotonicTimeUs HostClock::now_us() const noexcept
    {
        const auto now = std::chrono::steady_clock::now().time_since_epoch();
        return static_cast<MonotonicTimeUs>(
            std::chrono::duration_cast<std::chrono::microseconds>(now).count());
    }
}
