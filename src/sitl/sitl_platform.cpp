#include "sitl_platform.h"

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#else
#include <arpa/inet.h>
#include <cerrno>
#include <csignal>
#include <fcntl.h>
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

bool replace_file_atomically(
    const std::string &source,
    const std::string &destination)
{
#ifdef _WIN32
    constexpr int kMaxAttempts = 8;
    for (int attempt = 0; attempt < kMaxAttempts; ++attempt)
    {
        if (::MoveFileExA(
                source.c_str(), destination.c_str(),
                MOVEFILE_REPLACE_EXISTING) != FALSE)
            return true;

        const DWORD error = ::GetLastError();
        if (error != ERROR_ACCESS_DENIED &&
            error != ERROR_SHARING_VIOLATION)
            return false;
        ::Sleep(1);
    }
    return false;
#else
    return std::rename(source.c_str(), destination.c_str()) == 0;
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
                         "[FC] Parent PID %llu exited; requesting HydroX SITL cleanup.\n",
                         static_cast<unsigned long long>(state->parent_pid));
            std::fflush(nullptr);
            // The main connection and control loops check is_parent_alive().
            // Let them unwind normally so asynchronous XLog writers drain,
            // close their footer, and persist their final status. A process-
            // wide _Exit here bypassed every stack destructor.
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
    sockaddr_in candidate_peer{};
    sockaddr_in accepted_peer{};
    bool candidate_peer_valid = false;
    bool accepted_peer_valid = false;
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
    if (impl_->socket == kInvalidSocket)
        return;

#ifdef _WIN32
    u_long nonblocking = 1;
    if (::ioctlsocket(impl_->socket, FIONBIO, &nonblocking) != 0)
#else
    const int flags = ::fcntl(impl_->socket, F_GETFL, 0);
    if (flags < 0 || ::fcntl(impl_->socket, F_SETFL, flags | O_NONBLOCK) != 0)
#endif
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

bool UdpSender::bind_local(const std::string &host, uint16_t port)
{
    if (!is_open() || port == 0)
        return false;

    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_port = htons(port);
    if (::inet_pton(AF_INET, host.c_str(), &local.sin_addr) != 1)
        return false;
    return ::bind(
               impl_->socket,
               reinterpret_cast<const sockaddr *>(&local),
               sizeof(local)) == 0;
}

bool UdpSender::send(const void *data, std::size_t size) const
{
    if (!is_open() || data == nullptr || size == 0)
        return false;

    const sockaddr_in &destination = impl_->accepted_peer_valid
                                         ? impl_->accepted_peer
                                         : impl_->address;
    const int sent = ::sendto(
        impl_->socket,
        reinterpret_cast<const char *>(data),
        static_cast<int>(size),
        0,
        reinterpret_cast<const sockaddr *>(&destination),
        sizeof(destination));
    return sent == static_cast<int>(size);
}

int UdpSender::receive(void *data, std::size_t capacity)
{
    if (!is_open() || data == nullptr || capacity == 0)
        return -1;

    sockaddr_in source{};
#ifdef _WIN32
    int source_size = sizeof(source);
#else
    socklen_t source_size = sizeof(source);
#endif
    const int received = ::recvfrom(
        impl_->socket,
        reinterpret_cast<char *>(data),
        static_cast<int>(capacity),
        0,
        reinterpret_cast<sockaddr *>(&source),
        &source_size);
    if (received >= 0)
    {
        impl_->candidate_peer = source;
        impl_->candidate_peer_valid = true;
        return received;
    }

#ifdef _WIN32
    const int error = ::WSAGetLastError();
    if (error == WSAEWOULDBLOCK)
        return 0;
#else
    if (errno == EAGAIN || errno == EWOULDBLOCK)
        return 0;
#endif
    return -1;
}

void UdpSender::accept_last_peer()
{
    if (!impl_->candidate_peer_valid)
        return;
    impl_->accepted_peer = impl_->candidate_peer;
    impl_->accepted_peer_valid = true;
}

bool UdpSender::has_peer() const
{
    return impl_->accepted_peer_valid;
}

} // namespace hydrox::sitl
