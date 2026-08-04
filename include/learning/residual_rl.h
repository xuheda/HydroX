#pragma once

// Safe integration boundary for a learned low-level residual policy.
// A policy may be backed by ONNX, TorchScript, or a remote inference process,
// but it never emits actuator commands directly. It only proposes a normalized
// correction to the controller's body wrench.
#include "gnc/control_interfaces.h"

#include <memory>

namespace hydrox::learning
{
    struct ResidualObservation
    {
        NavigationState state;
        GNCSetpoint setpoint;
        Wrench base_wrench = Wrench::Zero();
    };

    struct ResidualAction
    {
        // Per-axis normalized correction in [-1, 1].
        Wrench normalized = Wrench::Zero();
        double confidence = 0.0; // [0, 1], supplied by the model or an uncertainty estimator
        bool valid = false;
    };

    class IResidualPolicy
    {
    public:
        virtual ~IResidualPolicy() = default;
        virtual ResidualAction infer(const ResidualObservation &observation) = 0;
    };

    class NullResidualPolicy final : public IResidualPolicy
    {
    public:
        ResidualAction infer(const ResidualObservation &) override { return {}; }
    };

    class ResidualSafetyFilter
    {
    public:
        struct Params
        {
            bool enabled = false;
            double blend = 0.0;             // Global authority [0, 1]. Zero is a safe default.
            double min_confidence = 0.75;
            Wrench max_delta = Wrench::Zero(); // N / N m per axis; zero disables that axis.
            Wrench max_rate = Wrench::Zero();  // N/s / N m/s; zero permits an immediate correction.
        };

        explicit ResidualSafetyFilter(const Params &params = {}) : params_(params) {}

        void reset();
        Wrench apply(const Wrench &base, const ResidualAction &action, double dt);

    private:
        Params params_;
        Wrench previous_delta_ = Wrench::Zero();
    };

    class ResidualRlModule
    {
    public:
        explicit ResidualRlModule(const ResidualSafetyFilter::Params &params = {},
                                  std::unique_ptr<IResidualPolicy> policy = std::make_unique<NullResidualPolicy>());

        void reset();
        Wrench update(const ResidualObservation &observation, double dt);

        IResidualPolicy *policy() { return policy_.get(); }

    private:
        std::unique_ptr<IResidualPolicy> policy_;
        ResidualSafetyFilter safety_filter_;
    };
} // namespace hydrox::learning
