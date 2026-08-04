#pragma once

#include "hydrox/platform/clock.h"
#include "hydrox/platform/sleeper.h"

#include <algorithm>
#include <cstdint>
#include <limits>

namespace hydrox::runtime
{
    struct PeriodicWakeup
    {
        platform::MonotonicTimeUs woke_at_us = 0;
        platform::MonotonicTimeUs scheduled_at_us = 0;
        platform::MonotonicTimeUs lateness_us = 0;
        uint32_t skipped_periods = 0;
    };

    /**
     * Drift-free periodic release scheduler.
     *
     * Deadlines advance from the previous scheduled release rather than the
     * actual wake time. If execution is late, missed periods are skipped
     * instead of causing a busy catch-up loop.
     */
    class PeriodicScheduler
    {
    public:
        PeriodicScheduler(
            const platform::Clock &clock,
            platform::Sleeper &sleeper,
            platform::MonotonicTimeUs period_us) noexcept
            : clock_(clock),
              sleeper_(sleeper),
              period_us_(std::max<platform::MonotonicTimeUs>(1, period_us))
        {
        }

        void reset() noexcept
        {
            reset(clock_.now_us());
        }

        void reset(platform::MonotonicTimeUs now_us) noexcept
        {
            next_release_us_ = saturating_add(now_us, period_us_);
            initialized_ = true;
        }

        PeriodicWakeup wait_next() noexcept
        {
            if (!initialized_)
                reset();

            const platform::MonotonicTimeUs scheduled_at_us = next_release_us_;
            sleeper_.sleep_until_us(scheduled_at_us);
            const platform::MonotonicTimeUs woke_at_us = clock_.now_us();

            PeriodicWakeup result;
            result.woke_at_us = woke_at_us;
            result.scheduled_at_us = scheduled_at_us;
            if (woke_at_us > scheduled_at_us)
            {
                result.lateness_us = woke_at_us - scheduled_at_us;
                const uint64_t skipped = result.lateness_us / period_us_;
                result.skipped_periods =
                    skipped > std::numeric_limits<uint32_t>::max()
                        ? std::numeric_limits<uint32_t>::max()
                        : static_cast<uint32_t>(skipped);
            }

            const uint64_t periods_to_advance =
                static_cast<uint64_t>(result.skipped_periods) + 1;
            next_release_us_ = saturating_add(
                scheduled_at_us,
                saturating_multiply(period_us_, periods_to_advance));
            return result;
        }

        platform::MonotonicTimeUs period_us() const noexcept
        {
            return period_us_;
        }

        platform::MonotonicTimeUs next_release_us() const noexcept
        {
            return next_release_us_;
        }

    private:
        static platform::MonotonicTimeUs saturating_add(
            platform::MonotonicTimeUs left,
            platform::MonotonicTimeUs right) noexcept
        {
            const auto maximum =
                std::numeric_limits<platform::MonotonicTimeUs>::max();
            return right > maximum - left ? maximum : left + right;
        }

        static platform::MonotonicTimeUs saturating_multiply(
            platform::MonotonicTimeUs left,
            uint64_t right) noexcept
        {
            const auto maximum =
                std::numeric_limits<platform::MonotonicTimeUs>::max();
            if (left != 0 && right > maximum / left)
                return maximum;
            return left * right;
        }

        const platform::Clock &clock_;
        platform::Sleeper &sleeper_;
        platform::MonotonicTimeUs period_us_;
        platform::MonotonicTimeUs next_release_us_ = 0;
        bool initialized_ = false;
    };
}
