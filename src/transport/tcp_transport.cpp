/**
 * tcp_transport.cpp - TCP transport layer for HydroX HIL/SITL.
 *
 * Cross-platform: POSIX (Linux/macOS) and Windows (Winsock2)
 */
#include "tcp_transport.h"
#include <algorithm>
#include <climits>
#include <cstring>
#include <stdexcept>
#include <iostream>

#ifdef _WIN32
#pragma comment(lib, "ws2_32.lib")
#define close_socket closesocket
#define poll WSAPoll
#include <ws2tcpip.h>
static bool winsock_init()
{
    WSADATA wd;
    return WSAStartup(MAKEWORD(2, 2), &wd) == 0;
}
static bool _ws_init = winsock_init();
#else
#include <poll.h>
#include <fcntl.h>
#include <errno.h>
#define close_socket ::close
#endif

namespace hydrox
{

    TcpTransport::TcpTransport(const std::string &host, uint16_t port, bool client_mode)
        : _host(host), _port(port), _client_mode(client_mode) {}

    TcpTransport::~TcpTransport()
    {
        disconnect();
    }

    bool TcpTransport::connect()
    {
        _tx_sender.reset();
        _tx_backpressure_notices = 0;
        _last_send_error = 0;
        return _client_mode ? _connect_client() : _connect_server();
    }

    bool TcpTransport::_connect_server()
    {
        // Create listening socket
        _server = ::socket(AF_INET, SOCK_STREAM, 0);
        if (_server == INVALID_SOCKET_VAL)
            return false;

        int opt = 1;
#ifdef _WIN32
        setsockopt(_server, SOL_SOCKET, SO_REUSEADDR,
                   reinterpret_cast<const char *>(&opt), sizeof(opt));
#else
        setsockopt(_server, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
#endif

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(_port);
        addr.sin_addr.s_addr = INADDR_ANY;

        if (::bind(_server, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0)
        {
            _close_server();
            return false;
        }
        if (::listen(_server, 1) != 0)
        {
            _close_server();
            return false;
        }

        std::cout << "[TcpTransport] Waiting for HIL Bridge on port " << _port << "...\n";

        // Block and wait for a client connection
        sockaddr_in cli_addr{};
#ifdef _WIN32
        int cli_len = sizeof(cli_addr);
#else
        socklen_t cli_len = sizeof(cli_addr);
#endif
        _client = ::accept(_server, reinterpret_cast<sockaddr *>(&cli_addr), &cli_len);
        if (_client == INVALID_SOCKET_VAL)
        {
            _close_server();
            return false;
        }

        // Set non-blocking
#ifdef _WIN32
        u_long mode = 1;
        ioctlsocket(_client, FIONBIO, &mode);
#else
        int flags = fcntl(_client, F_GETFL, 0);
        fcntl(_client, F_SETFL, flags | O_NONBLOCK);
#endif

        std::cout << "[TcpTransport] Connected!\n";
        return true;
    }

    bool TcpTransport::_connect_client()
    {
        // Client mode: connect to the UE5 FOceanXCommBridge TCP server.
        _client = ::socket(AF_INET, SOCK_STREAM, 0);
        if (_client == INVALID_SOCKET_VAL)
            return false;

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(_port);
#ifdef _WIN32
        ::InetPtonA(AF_INET, _host.c_str(), &addr.sin_addr);
#else
        ::inet_pton(AF_INET, _host.c_str(), &addr.sin_addr);
#endif

        std::cout << "[TcpTransport] Connecting to " << _host << ":" << _port << "...\n";

        if (::connect(_client, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0)
        {
            _close_client();
            return false;
        }

        // Set non-blocking after connect
#ifdef _WIN32
        u_long mode = 1;
        ioctlsocket(_client, FIONBIO, &mode);
#else
        int flags = fcntl(_client, F_GETFL, 0);
        fcntl(_client, F_SETFL, flags | O_NONBLOCK);
#endif

        std::cout << "[TcpTransport] Connected to " << _host << ":" << _port << "\n";
        return true;
    }

    void TcpTransport::disconnect()
    {
        _close_client();
        _close_server();
    }

    int TcpTransport::read(uint8_t *buf, size_t len)
    {
        if (_client == INVALID_SOCKET_VAL)
            return -1;
        // The control loop calls read every tick, so this also progresses an
        // earlier partial write even if no new outbound frame is submitted.
        if (!_flush_pending_tx())
            return -1;
#ifdef _WIN32
        int n = ::recv(_client, reinterpret_cast<char *>(buf),
                       static_cast<int>(len), 0);
        if (n == SOCKET_ERROR)
        {
            int e = WSAGetLastError();
            if (e == WSAEWOULDBLOCK)
                return 0;
            std::fprintf(stderr, "[TcpTransport] read SOCKET_ERROR WSA=%d — closing\n", e);
            _close_client();
            return -1;
        }
#else
        int n = static_cast<int>(::recv(_client, buf, len, 0));
        if (n < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
                return 0;
            std::fprintf(stderr, "[TcpTransport] read errno=%d — closing\n", errno);
            _close_client();
            return -1;
        }
#endif
        if (n == 0)
        {
            std::fprintf(stderr, "[TcpTransport] read n=0 (peer closed gracefully) — closing\n");
            _close_client();
            return -1;
        }
        return n;
    }

    bool TcpTransport::write(const uint8_t *buf, size_t len)
    {
        if (_client == INVALID_SOCKET_VAL)
            return false;

        const auto now = detail::NonblockingFrameSender::Clock::now();
        const detail::FrameSendStatus status = _tx_sender.write_frame(
            buf, len, now,
            [this](const uint8_t *data, size_t size)
            { return _send_once(data, size); });

        if (status == detail::FrameSendStatus::Complete)
            return true;
        if (status == detail::FrameSendStatus::TailPending ||
            status == detail::FrameSendStatus::FrameDropped)
        {
            _note_backpressure(status);
            return true;
        }
        return _handle_send_failure(status);
    }

    detail::SendAttempt TcpTransport::_send_once(const uint8_t *buf, size_t len)
    {
        if (_client == INVALID_SOCKET_VAL || buf == nullptr || len == 0)
        {
            _last_send_error = 0;
            return detail::SendAttempt::fatal();
        }

#ifdef _WIN32
        const int chunk = static_cast<int>(std::min<size_t>(len, INT_MAX));
        int n = 0;
        do
        {
            n = ::send(_client, reinterpret_cast<const char *>(buf), chunk, 0);
            if (n == SOCKET_ERROR)
            {
                _last_send_error = WSAGetLastError();
            }
        } while (n == SOCKET_ERROR && _last_send_error == WSAEINTR);

        if (n > 0)
            return detail::SendAttempt::progress(static_cast<size_t>(n));
        if (n == SOCKET_ERROR &&
            (_last_send_error == WSAEWOULDBLOCK || _last_send_error == WSAENOBUFS))
        {
            return detail::SendAttempt::would_block();
        }
#else
        int send_flags = 0;
#ifdef MSG_NOSIGNAL
        send_flags |= MSG_NOSIGNAL;
#endif
        ssize_t n = 0;
        do
        {
            n = ::send(_client, buf, len, send_flags);
            if (n < 0)
                _last_send_error = errno;
        } while (n < 0 && _last_send_error == EINTR);

        if (n > 0)
            return detail::SendAttempt::progress(static_cast<size_t>(n));
        if (n < 0 && (_last_send_error == EAGAIN ||
                      _last_send_error == EWOULDBLOCK ||
                      _last_send_error == ENOBUFS))
        {
            return detail::SendAttempt::would_block();
        }
#endif

        _last_send_error = (n == 0) ? 0 : _last_send_error;
        return detail::SendAttempt::fatal();
    }

    bool TcpTransport::_flush_pending_tx()
    {
        const detail::FrameSendStatus status = _tx_sender.flush(
            detail::NonblockingFrameSender::Clock::now(),
            [this](const uint8_t *data, size_t size)
            { return _send_once(data, size); });

        if (status == detail::FrameSendStatus::Complete)
            return true;
        if (status == detail::FrameSendStatus::TailPending)
            return true;
        return _handle_send_failure(status);
    }

    bool TcpTransport::_handle_send_failure(detail::FrameSendStatus status)
    {
        if (status == detail::FrameSendStatus::TimedOut)
        {
            std::fprintf(stderr,
                         "[TcpTransport] write blocked for 500 ms — closing to discard stale stream state\n");
        }
        else
        {
#ifdef _WIN32
            std::fprintf(stderr,
                         "[TcpTransport] write SOCKET_ERROR WSA=%d — closing\n",
                         _last_send_error);
#else
            std::fprintf(stderr,
                         "[TcpTransport] write errno=%d — closing\n",
                         _last_send_error);
#endif
        }
        _close_client();
        return false;
    }

    void TcpTransport::_note_backpressure(detail::FrameSendStatus status)
    {
        ++_tx_backpressure_notices;
        if (_tx_backpressure_notices <= 5 || _tx_backpressure_notices % 1000 == 0)
        {
            if (status == detail::FrameSendStatus::TailPending)
            {
                std::fprintf(stderr,
                             "[TcpTransport] partial write — preserving %zu-byte frame tail\n",
                             _tx_sender.pending_bytes());
            }
            else
            {
                std::fprintf(stderr,
                             "[TcpTransport] send backpressure — dropped one untouched frame"
                             " (total=%llu)\n",
                             static_cast<unsigned long long>(_tx_sender.dropped_frames()));
            }
        }
    }

    void TcpTransport::_close_client()
    {
        if (_client != INVALID_SOCKET_VAL)
        {
            close_socket(_client);
            _client = INVALID_SOCKET_VAL;
        }
        // A reconnect starts a fresh byte stream. Never replay a tail that
        // belonged to the previous TCP connection.
        _tx_sender.reset();
    }
    void TcpTransport::_close_server()
    {
        if (_server != INVALID_SOCKET_VAL)
        {
            close_socket(_server);
            _server = INVALID_SOCKET_VAL;
        }
    }

} // namespace hydrox
