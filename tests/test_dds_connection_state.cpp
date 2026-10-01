#include "dds_connection_state.h"

#include <chrono>
#include <cstdio>

namespace
{
    int fail(const char *message)
    {
        std::fprintf(stderr, "FAIL: %s\n", message);
        return 1;
    }
}

int main()
{
    using hydrox::sitl::DdsConnectionState;
    using hydrox::sitl::DdsConnectionStatus;
    using hydrox::sitl::DdsControlLinkState;

    DdsControlLinkState link;
    if (link.observe(DdsConnectionStatus{
            DdsConnectionState::CONNECTING, 0, 0, 1}))
    {
        return fail("initial connect was reported as a disconnect");
    }

    if (link.observe(DdsConnectionStatus{
            DdsConnectionState::CONNECTED, 1, 0, 0}))
    {
        return fail("first connection was reported as a disconnect");
    }
    if (!link.accepts_setpoint(1) || link.accepts_setpoint(2))
        return fail("session generation did not gate connected setpoints");

    // Simulate the control thread missing BACKOFF and CONNECTING because the
    // latest-value status mailbox was overwritten by a fast reconnect.
    if (!link.observe(DdsConnectionStatus{
            DdsConnectionState::CONNECTED, 2, 1, 0}))
    {
        return fail("skipped disconnect was not detected through its epoch");
    }
    if (link.accepts_setpoint(1) || !link.accepts_setpoint(2))
        return fail("reconnect accepted a stale-session setpoint");

    if (link.observe(DdsConnectionStatus{
            DdsConnectionState::CONNECTED, 2, 1, 0}))
    {
        return fail("same disconnect epoch invalidated commands twice");
    }

    if (!link.observe(DdsConnectionStatus{
            DdsConnectionState::BACKOFF, 2, 2, 1}))
    {
        return fail("explicit disconnect was not reported");
    }
    if (link.accepts_setpoint(2))
        return fail("disconnected link accepted a setpoint");

    using hydrox::sitl::dds_reconnect_delay;
    if (dds_reconnect_delay(1) != std::chrono::seconds(1) ||
        dds_reconnect_delay(2) != std::chrono::seconds(2) ||
        dds_reconnect_delay(3) != std::chrono::seconds(4) ||
        dds_reconnect_delay(4) != std::chrono::seconds(5) ||
        dds_reconnect_delay(100) != std::chrono::seconds(5))
    {
        return fail("reconnect backoff policy is incorrect");
    }

    return 0;
}
