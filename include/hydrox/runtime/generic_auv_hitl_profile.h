#pragma once

#include <cstdint>
#include "hydrox/runtime/generated_fmuv6c_profiles.h"

namespace hydrox::runtime::generic_auv_hitl_profile
{
    // This metadata binds the compiled FMUv6C profile to its canonical bundle.
    // A host-side regression test fails if the bundle changes without updating
    // the compiled board profile and this deployment identifier together.
    inline constexpr char kBundlePath[] =
        "profiles/generic-auv-fin/vehicle-bundle.json";
    inline constexpr uint64_t kBundleFingerprint = kGeneratedFmuv6cProfiles[0].fingerprint;
}
