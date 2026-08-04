#pragma once

/**
 * vehicle_bundle.h — Versioned, simulator-independent control contracts.
 *
 * A VehicleBundle describes only the data HydroX consumes at runtime: the
 * controller family, control-oriented model, and logical actuator layout.
 * It deliberately contains no simulator asset path, hardware output mapping,
 * or full plant/digital-twin data.
 */

#include "fossen_vehicle_params.h"
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
     * `control` keeps the existing controller/allocator API stable during the
     * migration away from legacy, simulator-shaped parameter files.  `thrusters`
     * is deliberately separate: it is a logical layout in body-FRD coordinates,
     * never a board pin or simulator asset reference.
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
        FossenControlParams control;
        std::vector<Thruster> thrusters;
        size_t logical_actuator_count = 0;
        size_t allocation_rank = 0;
        std::vector<BundleValidationIssue> validation;
    };

    /** Load and validate one explicit VehicleBundle JSON file. */
    VehicleBundle load_vehicle_bundle(const std::string &path,
                                      std::string *error = nullptr);

    /** Re-run semantic checks after a caller applies a calibration overlay. */
    bool validate_vehicle_bundle(VehicleBundle &bundle,
                                 std::string *error = nullptr);
} // namespace hydrox
