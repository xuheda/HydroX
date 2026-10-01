#include "latest_value_mailbox.h"

#include <cstdint>
#include <cstdio>

namespace
{
    struct Value
    {
        int number = 0;
    };

    int fail(const char *message)
    {
        std::fprintf(stderr, "FAIL: %s\n", message);
        return 1;
    }
}

int main()
{
    hydrox::LatestValueMailbox<Value> mailbox;
    uint64_t sequence = 0;
    Value out;

    if (mailbox.try_take_if_new(sequence, out))
        return fail("empty mailbox produced a value");

    mailbox.publish(Value{1});
    mailbox.publish(Value{2});
    if (!mailbox.try_take_if_new(sequence, out) || out.number != 2)
        return fail("mailbox did not replace stale value");

    if (mailbox.try_take_if_new(sequence, out))
        return fail("same sequence was delivered twice");

    if (!mailbox.try_publish(Value{3}) ||
        !mailbox.try_take_if_new(sequence, out) ||
        out.number != 3)
    {
        return fail("try_publish value was not delivered");
    }

    return 0;
}
