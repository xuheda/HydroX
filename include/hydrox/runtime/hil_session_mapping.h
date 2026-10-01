#pragma once

#include "hydrox/runtime/hil_runtime.h"
#include "hydrox/runtime/hil_session_config.h"
#include "sensor_adapter.h"

namespace hydrox::runtime
{
    /** Apply only Session-owned fields. Profile-owned and command-owned fields
     * are deliberately left unchanged. */
    void apply_hil_session_config(
        const HilSessionConfigV1 &session,
        HilRuntimeConfig &runtime,
        SensorAdapter::Params &sensors) noexcept;
} // namespace hydrox::runtime
