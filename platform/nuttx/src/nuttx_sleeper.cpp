#include "hydrox/platform/nuttx/nuttx_sleeper.h"

#include "hydrox/platform/nuttx/nuttx_clock.h"

#include <cerrno>
#include <ctime>

namespace hydrox::platform::nuttx
{
    void NuttxSleeper::sleep_for_us(MonotonicTimeUs duration_us) noexcept
    {
        timespec request{};
        request.tv_sec = static_cast<time_t>(duration_us / 1000000ULL);
        request.tv_nsec =
            static_cast<long>((duration_us % 1000000ULL) * 1000ULL);

        while (nanosleep(&request, &request) != 0 && errno == EINTR)
        {
        }
    }

    void NuttxSleeper::sleep_until_us(MonotonicTimeUs deadline_us) noexcept
    {
        NuttxClock clock;
        const MonotonicTimeUs now_us = clock.now_us();
        if (deadline_us > now_us)
            sleep_for_us(deadline_us - now_us);
    }
}
