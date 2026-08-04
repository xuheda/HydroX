#pragma once

#include "hydrox/platform/critical_section.h"

#include <cstdint>
#include <limits>
#include <type_traits>

namespace hydrox::runtime
{
    struct TopicCursor
    {
        uint64_t generation = 0;
    };

    /**
     * Fixed-memory latest-value topic.
     *
     * This is the initial HydroBus primitive for state-like data. A topic has
     * one logical publisher and any number of readers. Readers keep their own
     * cursor and can ask only for values newer than the last one consumed.
     */
    template <typename T>
    class LatestValueTopic
    {
        static_assert(std::is_copy_assignable<T>::value,
                      "HydroBus topic values must be copy assignable");

    public:
        explicit LatestValueTopic(platform::CriticalSection &section) noexcept
            : section_(section)
        {
        }

        void publish(const T &value) noexcept
        {
            platform::CriticalSectionGuard guard(section_);
            value_ = value;
            has_value_ = true;
            if (generation_ == std::numeric_limits<uint64_t>::max())
                generation_ = 1;
            else
                ++generation_;
        }

        bool read_latest(T &value, TopicCursor &cursor) const noexcept
        {
            platform::CriticalSectionGuard guard(section_);
            if (!has_value_)
                return false;

            value = value_;
            cursor.generation = generation_;
            return true;
        }

        bool read_if_new(T &value, TopicCursor &cursor) const noexcept
        {
            platform::CriticalSectionGuard guard(section_);
            if (!has_value_ || cursor.generation == generation_)
                return false;

            value = value_;
            cursor.generation = generation_;
            return true;
        }

    private:
        platform::CriticalSection &section_;
        T value_{};
        uint64_t generation_ = 0;
        bool has_value_ = false;
    };
}
