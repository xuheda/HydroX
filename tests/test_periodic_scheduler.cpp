#include "hydrox/runtime/periodic_scheduler.h"

#include <cstdio>

namespace
{
    class FakeClock final : public hydrox::platform::Clock
    {
    public:
        hydrox::platform::MonotonicTimeUs now_us() const noexcept override
        {
            return now;
        }

        hydrox::platform::MonotonicTimeUs now = 0;
    };

    class FakeSleeper final : public hydrox::platform::Sleeper
    {
    public:
        explicit FakeSleeper(FakeClock &clock) : clock_(clock) {}

        void sleep_for_us(
            hydrox::platform::MonotonicTimeUs duration_us) noexcept override
        {
            clock_.now += duration_us;
        }

        void sleep_until_us(
            hydrox::platform::MonotonicTimeUs deadline_us) noexcept override
        {
            if (clock_.now < deadline_us)
                clock_.now = deadline_us;
            clock_.now += wakeup_overshoot_us;
        }

        hydrox::platform::MonotonicTimeUs wakeup_overshoot_us = 0;

    private:
        FakeClock &clock_;
    };

    int fail(const char *message)
    {
        std::fprintf(stderr, "FAIL: %s\n", message);
        return 1;
    }
}

int main()
{
    FakeClock clock;
    FakeSleeper sleeper(clock);
    clock.now = 1'000'000;

    hydrox::runtime::PeriodicScheduler scheduler(clock, sleeper, 10'000);
    scheduler.reset();
    const auto first = scheduler.wait_next();
    if (first.scheduled_at_us != 1'010'000 ||
        first.woke_at_us != 1'010'000 ||
        first.lateness_us != 0 ||
        first.skipped_periods != 0 ||
        scheduler.next_release_us() != 1'020'000)
    {
        return fail("on-time periodic release is incorrect");
    }

    sleeper.wakeup_overshoot_us = 25'000;
    const auto late = scheduler.wait_next();
    if (late.scheduled_at_us != 1'020'000 ||
        late.woke_at_us != 1'045'000 ||
        late.lateness_us != 25'000 ||
        late.skipped_periods != 2 ||
        scheduler.next_release_us() != 1'050'000)
    {
        return fail("late release did not skip missed periods");
    }

    sleeper.wakeup_overshoot_us = 0;
    const auto recovered = scheduler.wait_next();
    if (recovered.woke_at_us != 1'050'000 ||
        recovered.skipped_periods != 0)
    {
        return fail("scheduler did not recover after an overrun");
    }

    std::puts("PASS: periodic scheduler");
    return 0;
}
