#pragma once

#include "hydrox/safety/safety_types.h"

namespace hydrox::safety
{
    /** Selects exactly one reference source authorized by VehicleSupervisor. */
    class CommandArbiter
    {
    public:
        AuthorizedReference select(const SupervisorOutput &supervisor,
                                   const CandidateSet &candidates,
                                   uint64_t now_us) const noexcept;

    private:
        static bool candidate_is_valid(const ControlCandidate &candidate,
                                       ControlSource expected_source,
                                       uint64_t now_us) noexcept;
    };
} // namespace hydrox::safety
