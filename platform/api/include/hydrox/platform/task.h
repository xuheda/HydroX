#pragma once

#include <cstdint>

namespace hydrox::platform
{
    using TaskHandle = uint32_t;
    using TaskEntry = void (*)(void *);

    struct TaskConfig
    {
        const char *name = nullptr;
        uint8_t priority = 0;
        uint32_t stack_size_bytes = 0;
    };

    /**
     * Minimal task creation contract.
     *
     * Periodic timing belongs to the HydroX scheduler, not this platform API.
     */
    class TaskSpawner
    {
    public:
        virtual ~TaskSpawner() = default;

        virtual bool start(
            const TaskConfig &config,
            TaskEntry entry,
            void *argument,
            TaskHandle &handle) noexcept = 0;
    };
}
