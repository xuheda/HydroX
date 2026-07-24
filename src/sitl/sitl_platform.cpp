#include "sitl_platform.h"

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#else
#include <arpa/inet.h>
#include <cerrno>
#include <csignal>
#include <sys/socket.h>
#include <unistd.h>
#endif

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <thread>

namespace hydrox::sitl
{
namespace
{
    using namespace std::chrono_literals;

#ifdef _WIN32
    using Socket = SOCKET;
    constexpr Socket kInvalidSocket = INVALID_SOCKET;
    void close_socket(Socket socket) { ::closesocket(socket); }
#else
    using Socket = int;
    constexpr Socket kInvalidSocket = -1;
    void close_socket(Socket socket) { ::close(socket); }
#endif
}

NetworkRuntime::NetworkRuntime()
{
#ifdef _WIN32
    WSADATA data{};
    ready_ = ::WSAStartup(MAKEWORD(2, 2), &data) == 0;
#else
    ready_ = true;
#endif
}

NetworkRuntime::~NetworkRuntime()
{
#ifdef _WIN32
    if (ready_)
        ::WSACleanup();
#endif
}

bool NetworkRuntime::ready() const
{
    return ready_;
}

struct ParentProcessGuard::Impl
{
    explicit Impl(uint64_t pid) : parent_pid(pid) {}

    bool is_alive() const
    {
        if (parent_pid == 0)
            return true;

#ifdef _WIN32
        return parent_handle != nullptr &&
               ::WaitForSingleObject(parent_handle, 0) == WAIT_TIMEOUT;
#else
        if (direct_parent)
            return static_cast<uint64_t>(::getppid()) == parent_pid;
        if (::kill(static_cast<pid_t>(parent_pid), 0) == 0)
            return true;
        return errno == EPERM;
#endif
    }

    uint64_t parent_pid = 0;
    std::atomic<bool> stop_requested{false};
    std::thread watchdog;
#ifdef _WIN32
    HANDLE parent_handle = nullptr;
#else
    bool direct_parent = false;
#endif
};

ParentProcessGuard::ParentProcessGuard(uint64_t parent_pid)
    : impl_(std::make_unique<Impl>(parent_pid))
{
}

ParentProcessGuard::~ParentProcessGuard()
{
    impl_->stop_requested = true;
    if (impl_->watchdog.joinable())
        impl_->watchdog.join();
#ifdef _WIN32
    if (impl_->parent_handle != nullptr)
        ::CloseHandle(impl_->parent_handle);
#endif
}

bool ParentProcessGuard::arm()
{
    if (impl_->parent_pid == 0)
        return true;

#ifdef _WIN32
    impl_->parent_handle = ::OpenProcess(
        SYNCHRONIZE, FALSE, static_cast<DWORD>(impl_->parent_pid));
    if (impl_->parent_handle == nullptr)
    {
        std::fprintf(stderr,
                     "[FC] Parent guard failed to open PID %llu (Win32=%lu)\n",
                     static_cast<unsigned long long>(impl_->parent_pid),
                     static_cast<unsigned long>(::GetLastError()));
        return false;
    }
#else
    impl_->direct_parent =
        static_cast<uint64_t>(::getppid()) == impl_->parent_pid;
    if (!impl_->is_alive())
        return false;
#endif

    Impl *state = impl_.get();
    impl_->watchdog = std::thread([state]()
    {
        while (!state->stop_requested && state->is_alive())
            std::this_thread::sleep_for(100ms);

        if (!state->stop_requested)
        {
            std::fprintf(stderr,
                         "[FC] Parent PID %llu exited; terminating HydroX SITL.\n",
                         static_cast<unsigned long long>(state->parent_pid));
            std::fflush(nullptr);
            std::_Exit(0);
        }
    });
    return true;
}

bool ParentProcessGuard::is_parent_alive() const
{
    return impl_->is_alive();
}

struct UdpSender::Impl
{
    Socket socket = kInvalidSocket;
    sockaddr_in address{};
};

UdpSender::UdpSender(const std::string &host, uint16_t port, bool broadcast)
    : impl_(std::make_unique<Impl>())
{
    impl_->socket = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (impl_->socket == kInvalidSocket)
        return;

    if (broadcast)
    {
#ifdef _WIN32
        BOOL enabled = TRUE;
        ::setsockopt(impl_->socket, SOL_SOCKET, SO_BROADCAST,
                     reinterpret_cast<const char *>(&enabled), sizeof(enabled));
#else
        int enabled = 1;
        ::setsockopt(impl_->socket, SOL_SOCKET, SO_BROADCAST,
                     &enabled, sizeof(enabled));
#endif
    }

    impl_->address.sin_family = AF_INET;
    impl_->address.sin_port = htons(port);
    if (::inet_pton(AF_INET, host.c_str(), &impl_->address.sin_addr) != 1)
    {
        close_socket(impl_->socket);
        impl_->socket = kInvalidSocket;
    }
}

UdpSender::~UdpSender()
{
    if (impl_->socket != kInvalidSocket)
        close_socket(impl_->socket);
}

bool UdpSender::is_open() const
{
    return impl_->socket != kInvalidSocket;
}

bool UdpSender::send(const void *data, std::size_t size) const
{
    if (!is_open() || data == nullptr || size == 0)
        return false;

    const int sent = ::sendto(
        impl_->socket,
        reinterpret_cast<const char *>(data),
        static_cast<int>(size),
        0,
        reinterpret_cast<const sockaddr *>(&impl_->address),
        sizeof(impl_->address));
    return sent == static_cast<int>(size);
}

} // namespace hydrox::sitl
