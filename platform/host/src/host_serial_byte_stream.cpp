#include "hydrox/platform/host/host_serial_byte_stream.h"

#include <algorithm>
#include <climits>
#include <utility>

#ifdef _WIN32
#include <windows.h>
#else
#include <cerrno>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace hydrox::platform::host
{
    HostSerialByteStream::HostSerialByteStream(
        std::string device, uint32_t baud_rate)
        : device_(std::move(device)), baud_rate_(baud_rate)
    {
    }

    HostSerialByteStream::~HostSerialByteStream()
    {
        close();
    }

    bool HostSerialByteStream::open() noexcept
    {
        close();
#ifdef _WIN32
        std::string path = device_;
        if (path.rfind("\\\\.\\", 0) != 0)
            path = "\\\\.\\" + path;
        HANDLE handle = ::CreateFileA(
            path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr,
            OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
        if (handle == INVALID_HANDLE_VALUE)
            return false;

        DCB dcb{};
        dcb.DCBlength = sizeof(dcb);
        if (!::GetCommState(handle, &dcb))
        {
            ::CloseHandle(handle);
            return false;
        }
        dcb.BaudRate = baud_rate_;
        dcb.ByteSize = 8;
        dcb.Parity = NOPARITY;
        dcb.StopBits = ONESTOPBIT;
        dcb.fBinary = TRUE;
        dcb.fParity = FALSE;
        dcb.fOutxCtsFlow = FALSE;
        dcb.fOutxDsrFlow = FALSE;
        dcb.fOutX = FALSE;
        dcb.fInX = FALSE;
        dcb.fDtrControl = DTR_CONTROL_ENABLE;
        dcb.fRtsControl = RTS_CONTROL_ENABLE;
        if (!::SetCommState(handle, &dcb))
        {
            ::CloseHandle(handle);
            return false;
        }

        COMMTIMEOUTS timeouts{};
        timeouts.ReadIntervalTimeout = MAXDWORD;
        timeouts.ReadTotalTimeoutMultiplier = 0;
        timeouts.ReadTotalTimeoutConstant = 0;
        timeouts.WriteTotalTimeoutMultiplier = 0;
        timeouts.WriteTotalTimeoutConstant = 10;
        if (!::SetCommTimeouts(handle, &timeouts))
        {
            ::CloseHandle(handle);
            return false;
        }
        ::SetupComm(handle, 8192, 8192);
        native_handle_ = reinterpret_cast<std::intptr_t>(handle);
        return true;
#else
        const int fd = ::open(
            device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd < 0)
            return false;

        speed_t speed = B115200;
        switch (baud_rate_)
        {
        case 57600: speed = B57600; break;
        case 115200: speed = B115200; break;
#ifdef B230400
        case 230400: speed = B230400; break;
#endif
#ifdef B460800
        case 460800: speed = B460800; break;
#endif
#ifdef B921600
        case 921600: speed = B921600; break;
#endif
        default:
            ::close(fd);
            return false;
        }

        termios options{};
        if (::tcgetattr(fd, &options) != 0)
        {
            ::close(fd);
            return false;
        }
        ::cfmakeraw(&options);
        ::cfsetispeed(&options, speed);
        ::cfsetospeed(&options, speed);
        options.c_cflag |= CLOCAL | CREAD;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        options.c_cc[VMIN] = 0;
        options.c_cc[VTIME] = 0;
        if (::tcsetattr(fd, TCSANOW, &options) != 0)
        {
            ::close(fd);
            return false;
        }
        native_handle_ = fd;
        return true;
#endif
    }

    void HostSerialByteStream::close() noexcept
    {
        if (!is_open())
            return;
#ifdef _WIN32
        ::CloseHandle(reinterpret_cast<HANDLE>(native_handle_));
#else
        ::close(static_cast<int>(native_handle_));
#endif
        native_handle_ = -1;
    }

    bool HostSerialByteStream::is_open() const noexcept
    {
        return native_handle_ != -1;
    }

    IoResult HostSerialByteStream::read(
        uint8_t *buffer, std::size_t capacity) noexcept
    {
        if (!is_open())
            return {0, IoStatus::Closed};
        if (buffer == nullptr || capacity == 0)
            return {0, IoStatus::Error};
#ifdef _WIN32
        DWORD bytes = 0;
        const DWORD request = static_cast<DWORD>(
            std::min<std::size_t>(capacity, MAXDWORD));
        if (!::ReadFile(reinterpret_cast<HANDLE>(native_handle_),
                        buffer, request, &bytes, nullptr))
        {
            return {0, IoStatus::Error};
        }
        return bytes > 0 ? IoResult{bytes, IoStatus::Ok}
                         : IoResult{0, IoStatus::WouldBlock};
#else
        const ssize_t bytes = ::read(
            static_cast<int>(native_handle_), buffer, capacity);
        if (bytes > 0)
            return {static_cast<std::size_t>(bytes), IoStatus::Ok};
        if (bytes == 0 || errno == EAGAIN || errno == EWOULDBLOCK)
            return {0, IoStatus::WouldBlock};
        return {0, IoStatus::Error};
#endif
    }

    IoResult HostSerialByteStream::write(
        const uint8_t *data, std::size_t size) noexcept
    {
        if (!is_open())
            return {0, IoStatus::Closed};
        if (data == nullptr || size == 0)
            return {0, IoStatus::Error};
#ifdef _WIN32
        DWORD bytes = 0;
        const DWORD request = static_cast<DWORD>(
            std::min<std::size_t>(size, MAXDWORD));
        if (!::WriteFile(reinterpret_cast<HANDLE>(native_handle_),
                         data, request, &bytes, nullptr))
        {
            return {0, IoStatus::Error};
        }
        return bytes > 0 ? IoResult{bytes, IoStatus::Ok}
                         : IoResult{0, IoStatus::WouldBlock};
#else
        const ssize_t bytes = ::write(
            static_cast<int>(native_handle_), data, size);
        if (bytes > 0)
            return {static_cast<std::size_t>(bytes), IoStatus::Ok};
        if (errno == EAGAIN || errno == EWOULDBLOCK)
            return {0, IoStatus::WouldBlock};
        return {0, IoStatus::Error};
#endif
    }
} // namespace hydrox::platform::host
