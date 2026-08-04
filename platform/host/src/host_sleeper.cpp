#include "hydrox/platform/host/host_sleeper.h"

#include <chrono>
#include <thread>

namespace hydrox::platform::host
{
    void HostSleeper::sleep_for_us(MonotonicTimeUs duration_us) noexcept
    {
        std::this_thread::sleep_for(std::chrono::microseconds(duration_us));
    }

    void HostSleeper::sleep_until_us(MonotonicTimeUs deadline_us) noexcept
    {
        const std::chrono::steady_clock::time_point deadline{
            std::chrono::microseconds(deadline_us)};
        std::this_thread::sleep_until(deadline);
    }
}
