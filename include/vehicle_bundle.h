#pragma once

/**
 * vehicle_bundle.h — Versioned, simulator-independent control contracts.
 *
 * A VehicleBundle describes only the data HydroX consumes at runtime: the
 * controller family, control-oriented model, and logical actuator layout.
 * It deliberately contains no simulator asset path, hardware output mapping,
 * or full plant/digital-twin data.
 */

#include "control_parameters.h"
#include "gnc/thruster_allocator.h"

#include <string>
#include <vector>
#include <cstdint>

namespace hydrox
{
    enum class BundleIssueSeverity
    {
        Warning,
        Error,
    };

    struct BundleValidationIssue
    {
        BundleIssueSeverity severity = BundleIssueSeverity::Error;
        std::string field;
        std::string message;
    };

    /**
     * The runtime representation of one VehicleBundle JSON document.
     *
     * `control` includes the complete logical actuator layout in body-FRD.
     */
    struct VehicleBundle
    {
        bool valid = false;
        std::string schema_version;
        std::string id;
        std::string control_contract;
        std::string source_path;
        /** FNV-1a of the exact source bytes, used to match compiled HITL params. */
        uint64_t fingerprint = 0;
        ControlParameters control;
        size_t logical_actuator_count = 0;
        size_t allocation_rank = 0;
        std::vector<BundleValidationIssue> validation;
    };

    /** Load and validate one explicit VehicleBundle JSON file. */
    VehicleBundle load_vehicle_bundle(const std::string &path,
                                      std::string *error = nullptr);

    /**
     * Load and validate one bundle from an already-loaded JSON document.
     * FMUv6C uses this for generated immutable documents stored in Flash while
     * preserving the exact host-side parser, validation, and fingerprint.
     */
    VehicleBundle load_vehicle_bundle_json(
        const std::string &json,
        const std::string &source_name,
        std::string *error = nullptr);

    /** Re-run semantic checks after a caller applies a calibration overlay. */
    bool validate_vehicle_bundle(VehicleBundle &bundle,
                                 std::string *error = nullptr);
} // namespace hydrox
