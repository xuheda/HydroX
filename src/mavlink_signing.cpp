#include "mavlink_signing.h"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <fstream>
#include <iterator>
#include <vector>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <bcrypt.h>
#else
#include <openssl/sha.h>
#endif

namespace hydrox
{
    namespace
    {
        constexpr int64_t kMavlinkEpochUnixSeconds = 1420070400LL;

        void set_error(std::string *out_error, const char *message)
        {
            if (out_error != nullptr)
                *out_error = message;
        }

        int hex_value(char c)
        {
            if (c >= '0' && c <= '9')
                return c - '0';
            if (c >= 'a' && c <= 'f')
                return c - 'a' + 10;
            if (c >= 'A' && c <= 'F')
                return c - 'A' + 10;
            return -1;
        }

        void write_timestamp_48_le(uint8_t *out, uint64_t timestamp)
        {
            for (int i = 0; i < 6; ++i)
                out[i] = static_cast<uint8_t>((timestamp >> (8 * i)) & 0xFFu);
        }
    } // namespace

    bool load_mavlink_signing_key_file(
        const std::string &path,
        std::array<uint8_t, MAVLINK_SIGNING_KEY_LEN> &out_key,
        std::string *out_error)
    {
        out_key.fill(0);
        std::ifstream input(path, std::ios::binary);
        if (!input)
        {
            set_error(out_error, "unable to open MAVLink signing key file");
            return false;
        }

        std::string text(
            (std::istreambuf_iterator<char>(input)),
            std::istreambuf_iterator<char>());
        text.erase(
            std::remove_if(
                text.begin(), text.end(),
                [](unsigned char c) { return std::isspace(c) != 0; }),
            text.end());
        if (text.size() != MAVLINK_SIGNING_KEY_LEN * 2u)
        {
            set_error(out_error, "MAVLink signing key must contain exactly 64 hexadecimal characters");
            return false;
        }

        for (size_t i = 0; i < out_key.size(); ++i)
        {
            const int high = hex_value(text[i * 2]);
            const int low = hex_value(text[i * 2 + 1]);
            if (high < 0 || low < 0)
            {
                out_key.fill(0);
                set_error(out_error, "MAVLink signing key contains non-hexadecimal characters");
                return false;
            }
            out_key[i] = static_cast<uint8_t>((high << 4) | low);
        }
        if (std::all_of(
                out_key.begin(), out_key.end(),
                [](uint8_t byte) { return byte == 0; }))
        {
            out_key.fill(0);
            set_error(out_error, "MAVLink signing key must not be all zero bytes");
            return false;
        }
        return true;
    }

    uint64_t mavlink_signing_timestamp_10us()
    {
        const auto now_us = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        constexpr int64_t epoch_us = kMavlinkEpochUnixSeconds * 1000000LL;
        if (now_us <= epoch_us)
            return 0;
        return static_cast<uint64_t>((now_us - epoch_us) / 10LL);
    }

    bool mavlink_signing_digest(
        const std::array<uint8_t, MAVLINK_SIGNING_KEY_LEN> &key,
        const uint8_t *packet_without_magic,
        size_t packet_len,
        uint8_t link_id,
        uint64_t timestamp_10us,
        std::array<uint8_t, 32> &out_digest)
    {
        if (packet_without_magic == nullptr || packet_len == 0)
            return false;

        uint8_t signing_suffix[7] = {};
        signing_suffix[0] = link_id;
        write_timestamp_48_le(signing_suffix + 1, timestamp_10us);

#ifdef _WIN32
        BCRYPT_ALG_HANDLE algorithm = nullptr;
        BCRYPT_HASH_HANDLE hash = nullptr;
        DWORD object_length = 0;
        DWORD result_length = 0;
        NTSTATUS status = BCryptOpenAlgorithmProvider(
            &algorithm, BCRYPT_SHA256_ALGORITHM, nullptr, 0);
        if (!BCRYPT_SUCCESS(status))
            return false;
        status = BCryptGetProperty(
            algorithm,
            BCRYPT_OBJECT_LENGTH,
            reinterpret_cast<PUCHAR>(&object_length),
            sizeof(object_length),
            &result_length,
            0);
        std::vector<uint8_t> hash_object(object_length);
        if (BCRYPT_SUCCESS(status))
        {
            status = BCryptCreateHash(
                algorithm,
                &hash,
                hash_object.data(),
                static_cast<ULONG>(hash_object.size()),
                nullptr,
                0,
                0);
        }
        if (BCRYPT_SUCCESS(status))
        {
            status = BCryptHashData(
                hash,
                const_cast<PUCHAR>(key.data()),
                static_cast<ULONG>(key.size()),
                0);
        }
        if (BCRYPT_SUCCESS(status))
        {
            status = BCryptHashData(
                hash,
                const_cast<PUCHAR>(packet_without_magic),
                static_cast<ULONG>(packet_len),
                0);
        }
        if (BCRYPT_SUCCESS(status))
            status = BCryptHashData(hash, signing_suffix, sizeof(signing_suffix), 0);
        if (BCRYPT_SUCCESS(status))
        {
            status = BCryptFinishHash(
                hash,
                out_digest.data(),
                static_cast<ULONG>(out_digest.size()),
                0);
        }
        if (hash != nullptr)
            BCryptDestroyHash(hash);
        BCryptCloseAlgorithmProvider(algorithm, 0);
        return BCRYPT_SUCCESS(status);
#else
        SHA256_CTX context;
        if (SHA256_Init(&context) != 1 ||
            SHA256_Update(&context, key.data(), key.size()) != 1 ||
            SHA256_Update(&context, packet_without_magic, packet_len) != 1 ||
            SHA256_Update(&context, signing_suffix, sizeof(signing_suffix)) != 1 ||
            SHA256_Final(out_digest.data(), &context) != 1)
        {
            return false;
        }
        return true;
#endif
    }

    bool mavlink_signature_equal_48(const uint8_t *lhs, const uint8_t *rhs)
    {
        if (lhs == nullptr || rhs == nullptr)
            return false;
        uint8_t difference = 0;
        for (size_t i = 0; i < 6; ++i)
            difference |= static_cast<uint8_t>(lhs[i] ^ rhs[i]);
        return difference == 0;
    }

} // namespace hydrox
