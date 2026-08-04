// Copyright (c) 2026 OceanX. Author: xuheda
#include "sitl/control_feedback.h"

#include <cstdio>

namespace
{
    int expect(bool condition, const char *message)
    {
        if (!condition)
        {
            std::fprintf(stderr, "FAIL: %s\n", message);
            return 1;
        }
        return 0;
    }
}

int main()
{
    using namespace hydrox;
    using namespace hydrox::sitl;

    int failures = 0;

    NavigationState estimated = NavigationState::zeros();
    estimated.eta[0] = 12.0;
    estimated.eta[2] = 4.0;
    estimated.nu[0] = 1.25;
    estimated.depth_m = 4.0;

    HilTruthStateMsg truth;
    truth.valid = true;
    truth.eta[0] = 99.0;
    truth.eta[2] = 8.0;
    truth.nu[0] = 3.5;

    const auto with_truth_present = select_control_feedback(
        estimated,
        truth,
        true,
        ControlFeedbackSource::EstimatedState);
    const auto without_truth_present = select_control_feedback(
        estimated,
        HilTruthStateMsg{},
        false,
        ControlFeedbackSource::EstimatedState);

    failures += expect(!with_truth_present.used_truth,
                       "default feedback never consumes valid truth");
    failures += expect(
        with_truth_present.state.eta.isApprox(
            without_truth_present.state.eta,
            0.0),
        "truth presence cannot change default control position or attitude");
    failures += expect(
        with_truth_present.state.nu.isApprox(
            without_truth_present.state.nu,
            0.0),
        "truth presence cannot change default control velocity");

    const auto explicit_debug = select_control_feedback(
        estimated,
        truth,
        true,
        ControlFeedbackSource::TruthDebug);
    failures += expect(explicit_debug.used_truth,
                       "explicit truth debug feedback consumes recent valid truth");
    failures += expect(explicit_debug.state.eta[0] == 99.0,
                       "truth debug replaces control position");
    failures += expect(explicit_debug.state.depth_m == 8.0,
                       "truth debug derives control depth from truth");

    const auto stale_debug = select_control_feedback(
        estimated,
        truth,
        false,
        ControlFeedbackSource::TruthDebug);
    failures += expect(!stale_debug.used_truth,
                       "truth debug rejects stale truth");
    failures += expect(stale_debug.state.eta[0] == estimated.eta[0],
                       "stale truth falls back to estimated state");

    if (failures == 0)
        std::printf("test_control_feedback: all checks passed\n");
    return failures == 0 ? 0 : 1;
}
