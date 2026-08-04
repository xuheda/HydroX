#pragma once

#include <string>
#include <string_view>

namespace hydrox::frame_contract
{
inline constexpr const char *kMapNed = "map_ned";

inline bool is_valid_frame_token(std::string_view token)
{
    if (token.empty())
        return false;
    for (const char ch : token)
    {
        const bool ascii_alphanumeric =
            (ch >= 'a' && ch <= 'z') ||
            (ch >= 'A' && ch <= 'Z') ||
            (ch >= '0' && ch <= '9');
        if (!ascii_alphanumeric && ch != '_')
            return false;
    }
    return true;
}

inline bool is_map_ned(std::string_view frame_id)
{
    return frame_id == kMapNed;
}

inline std::string body_frd(const std::string &vehicle)
{
    if (!is_valid_frame_token(vehicle))
        return {};
    return vehicle + "/base_link_frd";
}

inline std::string sensor_frd(
    const std::string &vehicle,
    const std::string &sensor)
{
    if (!is_valid_frame_token(vehicle) || !is_valid_frame_token(sensor))
        return {};
    return vehicle + "/" + sensor + "_frd";
}
} // namespace hydrox::frame_contract
