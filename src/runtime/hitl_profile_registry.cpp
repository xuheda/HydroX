#include "hydrox/runtime/hitl_profile_registry.h"

#include "hydrox/runtime/generated_fmuv6c_profiles.h"
#include "hydrox/runtime/hil_session_mapping.h"
#include "vehicle_bundle.h"

#include <cstring>
#include <utility>

namespace hydrox::runtime
{
namespace
{
    const GeneratedFmuv6cProfileDocument *find_document(
        const char *profile_id, uint64_t fingerprint) noexcept
    {
        if (profile_id == nullptr || profile_id[0] == '\0' || fingerprint == 0)
            return nullptr;
        for (const auto &document : kGeneratedFmuv6cProfiles)
        {
            if (document.fingerprint == fingerprint &&
                std::strcmp(document.profile_id, profile_id) == 0)
                return &document;
        }
        return nullptr;
    }

    void set_error(std::string *error, const std::string &message)
    {
        if (error != nullptr)
            *error = message;
    }

    bool load_document_control(
        const GeneratedFmuv6cProfileDocument &document,
        const std::string &json,
        ControlParameters &control,
        std::string *error)
    {
        VehicleBundle bundle = load_vehicle_bundle_json(
            json, "compiled:" + std::string(document.source_path), error);
        if (!bundle.valid || bundle.fingerprint != document.fingerprint || bundle.id != document.profile_id)
        {
            if (bundle.valid)
                set_error(error, "compiled VehicleBundle identity/fingerprint mismatch");
            return false;
        }
        control = std::move(bundle.control);
        return true;
    }
}

std::size_t compiled_hitl_profile_count() noexcept
{
    return kGeneratedFmuv6cProfiles.size();
}

bool compiled_hitl_profile_identity(
    std::size_t index, HitlProfileIdentity &identity) noexcept
{
    if (index >= kGeneratedFmuv6cProfiles.size())
        return false;
    const auto &document = kGeneratedFmuv6cProfiles[index];
    identity.profile_id = document.profile_id;
    identity.fingerprint = document.fingerprint;
    identity.mav_type = document.mav_type;
    return true;
}

bool load_compiled_hitl_profile(
    const char *profile_id,
    uint64_t fingerprint,
    uint32_t selection_nonce,
    HitlVehicleProfile &profile,
    std::string *error)
{
    const auto *document = find_document(profile_id, fingerprint);
    if (document == nullptr)
    {
        set_error(error, "profile ID/fingerprint is not present in firmware");
        return false;
    }

    const std::string json(document->json, document->json_size);
    if (!load_document_control(*document, json, profile.control, error))
        return false;

    profile.profile_id.fill('\0');
    const std::size_t length = std::strlen(document->profile_id);
    if (length == 0 || length >= profile.profile_id.size())
    {
        set_error(error, "compiled profile ID exceeds board contract");
        return false;
    }
    std::memcpy(profile.profile_id.data(), document->profile_id, length + 1);
    profile.bundle_fingerprint = document->fingerprint;
    profile.runtime.estimation_profile =
        estimation_profile_for(profile.control.vehicle_class);
    profile.runtime.motor = profile.control.motor;
    profile.runtime.safety_profile = safety::safety_profile_for(profile.control);
    // Shared runtime safety supervision is mandatory on every target.
    profile.sensors = SensorAdapter::Params(profile.runtime.estimation_profile);
    profile.session_config = HilSessionConfigV1{};
    apply_hil_session_config(
        profile.session_config, profile.runtime, profile.sensors);
    profile.session_digest.fill(0);
    profile.session_nonce = 0;
    profile.session_configured = false;
    profile.mav_type = document->mav_type;
    profile.selection_nonce = selection_nonce;

    const char *validation_error = nullptr;
    if (!validate_hitl_vehicle_profile(profile, validation_error))
    {
        set_error(error, validation_error != nullptr
                             ? validation_error
                             : "compiled profile failed HITL validation");
        return false;
    }
    if (error != nullptr)
        error->clear();
    return true;
}
} // namespace hydrox::runtime
