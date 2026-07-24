#pragma once
/**
 * tcp_transport.h — TCP socket transport (SITL)
 *
 * Two roles (selected by constructor):
 *   Server (client_mode=false, default): bind/listen/accept for embedded HIL
 *                                        layouts.
 *   Client (client_mode=true):           connect to UE5 FOceanXCommBridge; this
 *                                        is the current hydrox_sitl default.
 */
#include "transport.h"
#include "nonblocking_frame_sender.h"
#include <cstdint>
#include <string>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
using socket_t = SOCKET;
#define INVALID_SOCKET_VAL INVALID_SOCKET
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
using socket_t = int;
#define INVALID_SOCKET_VAL (-1)
#endif

namespace hydrox
{

    class TcpTransport : public Transport
    {
    public:
        /**
         * @param host        Server mode: bind address ("0.0.0.0").
         *                    Client mode: remote address to connect to.
         * @param port        Port number.
         * @param client_mode false = server (listen/accept), true = client (connect)
         */
        TcpTransport(const std::string &host = "0.0.0.0",
                     uint16_t port = 14600,
                     bool client_mode = false);
        ~TcpTransport() override;

        bool connect() override;
        void disconnect() override;
        int read(uint8_t *buf, size_t len) override;
        bool write(const uint8_t *buf, size_t len) override;
        bool is_connected() const override { return _client != INVALID_SOCKET_VAL; }

        size_t pending_write_bytes() const { return _tx_sender.pending_bytes(); }
        uint64_t dropped_write_frames() const { return _tx_sender.dropped_frames(); }

    private:
        std::string _host;
        uint16_t _port;
        bool _client_mode;
        socket_t _server = INVALID_SOCKET_VAL;
        socket_t _client = INVALID_SOCKET_VAL;
        detail::NonblockingFrameSender _tx_sender;
        int _last_send_error = 0;
        uint64_t _tx_backpressure_notices = 0;

        bool _connect_server();
        bool _connect_client();
        detail::SendAttempt _send_once(const uint8_t *buf, size_t len);
        bool _flush_pending_tx();
        bool _handle_send_failure(detail::FrameSendStatus status);
        void _note_backpressure(detail::FrameSendStatus status);
        void _close_client();
        void _close_server();
    };

} // namespace hydrox
