#include "hydrox/platform/critical_section.h"
#include "hydrox/runtime/latest_value_topic.h"
#include "hydrox/runtime/spsc_queue.h"

#define HYDROX_CHECK(condition) \
    do                           \
    {                            \
        if (!(condition))        \
            return __LINE__;     \
    } while (false)

namespace
{
    class TestCriticalSection final : public hydrox::platform::CriticalSection
    {
    public:
        void lock() noexcept override {}
        void unlock() noexcept override {}
    };

    struct Sample
    {
        int value = 0;
    };
}

int main()
{
    TestCriticalSection section;
    hydrox::runtime::LatestValueTopic<Sample> topic(section);
    hydrox::runtime::TopicCursor first_reader;
    hydrox::runtime::TopicCursor second_reader;
    Sample sample;

    HYDROX_CHECK(!topic.read_latest(sample, first_reader));
    topic.publish(Sample{42});
    HYDROX_CHECK(topic.read_if_new(sample, first_reader));
    HYDROX_CHECK(sample.value == 42);
    HYDROX_CHECK(!topic.read_if_new(sample, first_reader));
    HYDROX_CHECK(topic.read_if_new(sample, second_reader));

    hydrox::runtime::SpscQueue<int, 2> queue;
    HYDROX_CHECK(queue.capacity() == 2);
    HYDROX_CHECK(queue.try_push(10));
    HYDROX_CHECK(queue.try_push(20));
    HYDROX_CHECK(!queue.try_push(30));
    HYDROX_CHECK(queue.size_approx() == 2);

    int value = 0;
    HYDROX_CHECK(queue.try_pop(value) && value == 10);
    HYDROX_CHECK(queue.try_pop(value) && value == 20);
    HYDROX_CHECK(!queue.try_pop(value));
    return 0;
}

#undef HYDROX_CHECK
