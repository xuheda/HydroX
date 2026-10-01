#pragma once

#include "hydrox/runtime/hitl_board.h"

#include <cstddef>
#include <cstdint>
#include <string>

namespace hydrox::runtime
{
    struct HitlProfileIdentity
    {
        const char *profile_id = nullptr;
        uint64_t fingerprint = 0;
        uint8_t mav_type = 0;
    };

    std::size_t compiled_hitl_profile_count() noexcept;
    bool compiled_hitl_profile_identity(
        std::size_t index, HitlProfileIdentity &identity) noexcept;

    /** Exact ID and fingerprint lookup; partial or fallback matching is forbidden. */
    bool load_compiled_hitl_profile(
        const char *profile_id,
        uint64_t fingerprint,
        uint32_t selection_nonce,
        HitlVehicleProfile &profile,
        std::string *error = nullptr);
} // namespace hydrox::runtime
