#include "hydrox/platform/host/host_clock.h"
#include "hydrox/platform/host/host_critical_section.h"

#include <thread>
#include <vector>

int main()
{
    hydrox::platform::host::HostClock clock;
    const auto first = clock.now_us();
    const auto second = clock.now_us();
    if (second < first)
        return 1;

    hydrox::platform::host::HostCriticalSection section;
    int counter = 0;
    std::vector<std::thread> workers;
    for (int worker = 0; worker < 4; ++worker)
    {
        workers.emplace_back([&section, &counter]()
        {
            for (int i = 0; i < 1000; ++i)
            {
                hydrox::platform::CriticalSectionGuard guard(section);
                ++counter;
            }
        });
    }
    for (auto &worker : workers)
        worker.join();

    return counter == 4000 ? 0 : 2;
}
