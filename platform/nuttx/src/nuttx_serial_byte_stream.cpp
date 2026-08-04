#include "hydrox/platform/nuttx/nuttx_serial_byte_stream.h"
#include <cerrno>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace hydrox::platform::nuttx
{
namespace
{
    speed_t baud_flag(uint32_t baud) noexcept
    {
        switch (baud)
        {
#ifdef B1500000
        case 1500000: return B1500000;
#endif
#ifdef B921600
        case 921600: return B921600;
#endif
        case 460800: return B460800;
        case 230400: return B230400;
        case 115200: return B115200;
        case 57600: return B57600;
        default: return 0;
        }
    }

    IoStatus error_status() noexcept
    {
        return (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
                   ? IoStatus::WouldBlock
                   : IoStatus::Error;
    }
}

NuttxSerialByteStream::NuttxSerialByteStream(
    const char *device, uint32_t baud, bool hardware_flow_control) noexcept
    : device_(device), baud_(baud),
      hardware_flow_control_(hardware_flow_control)
{
}

NuttxSerialByteStream::~NuttxSerialByteStream()
{
    close();
}

bool NuttxSerialByteStream::open() noexcept
{
    if (fd_ >= 0)
        return true;
    fd_ = ::open(device_, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0)
        return false;
    if (!configure())
    {
        close();
        return false;
    }
    return true;
}

void NuttxSerialByteStream::close() noexcept
{
    if (fd_ >= 0)
        ::close(fd_);
    fd_ = -1;
}

bool NuttxSerialByteStream::is_open() const noexcept
{
    return fd_ >= 0;
}

bool NuttxSerialByteStream::configure() noexcept
{
    termios options{};
    if (tcgetattr(fd_, &options) < 0)
        return false;
    cfmakeraw(&options);
    const speed_t speed = baud_flag(baud_);
    if (speed == 0 || cfsetispeed(&options, speed) < 0 ||
        cfsetospeed(&options, speed) < 0)
        return false;
    options.c_cflag |= CLOCAL | CREAD;
#ifdef CRTSCTS
    if (hardware_flow_control_)
        options.c_cflag |= CRTSCTS;
    else
        options.c_cflag &= ~CRTSCTS;
#else
    if (hardware_flow_control_)
        return false;
#endif
    if (tcsetattr(fd_, TCSANOW, &options) < 0)
        return false;
    (void)tcflush(fd_, TCIOFLUSH);
    return true;
}

IoResult NuttxSerialByteStream::read(
    uint8_t *buffer, std::size_t capacity) noexcept
{
    if (fd_ < 0)
        return {0, IoStatus::Closed};
    const ssize_t count = ::read(fd_, buffer, capacity);
    if (count > 0)
        return {static_cast<std::size_t>(count), IoStatus::Ok};
    if (count == 0)
        return {0, IoStatus::WouldBlock};
    const IoStatus status = error_status();
    if (status == IoStatus::Error)
        close();
    return {0, status};
}

IoResult NuttxSerialByteStream::write(
    const uint8_t *data, std::size_t size) noexcept
{
    if (fd_ < 0)
        return {0, IoStatus::Closed};
    const ssize_t count = ::write(fd_, data, size);
    if (count > 0)
        return {static_cast<std::size_t>(count), IoStatus::Ok};
    if (count == 0)
        return {0, IoStatus::WouldBlock};
    const IoStatus status = error_status();
    if (status == IoStatus::Error)
        close();
    return {0, status};
}
}