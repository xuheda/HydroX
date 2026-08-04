#include "hydrox/platform/nuttx/nuttx_watchdog.h"
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <nuttx/timers/watchdog.h>

namespace hydrox::platform::nuttx
{
NuttxWatchdog::NuttxWatchdog(const char *device) noexcept : device_(device) {}

NuttxWatchdog::~NuttxWatchdog()
{
    if (fd_ >= 0)
        ::close(fd_);
}

bool NuttxWatchdog::start(uint32_t timeout_ms) noexcept
{
    if (running_)
        return true;
    if (fd_ < 0)
        fd_ = ::open(device_, O_RDONLY | O_CLOEXEC);
    if (fd_ < 0)
        return false;
    if (::ioctl(fd_, WDIOC_SETTIMEOUT,
                static_cast<unsigned long>(timeout_ms)) < 0 ||
        ::ioctl(fd_, WDIOC_START, 0UL) < 0)
    {
        ::close(fd_);
        fd_ = -1;
        return false;
    }
    running_ = true;
    return true;
}

void NuttxWatchdog::kick() noexcept
{
    if (running_ && ::ioctl(fd_, WDIOC_KEEPALIVE, 0UL) < 0)
        running_ = false;
}

bool NuttxWatchdog::is_running() const noexcept
{
    return running_;
}
}