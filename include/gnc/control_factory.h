// Copyright (c) 2026 OceanX. Author: xuheda
#pragma once
/**
 * control_factory.h — build the {IController, IAllocator} stack for a vehicle.
 *
 * Selects the control archetype from the loaded vehicle params and assembles the
 * matching controller + allocator. This is the one place that knows the
 * vehicle->archetype mapping; everything downstream talks only the tau-contract.
 */
#include "gnc/control_interfaces.h"
#include "fossen_vehicle_params.h"
#include <memory>

namespace hydrox
{
    struct VehicleBundle;

    struct ControlStack
    {
        std::unique_ptr<IController> controller;
        std::unique_ptr<IAllocator> allocator;
    };

    ControlStack build_control_stack(const FossenControlParams &vp);
    ControlStack build_control_stack(const VehicleBundle &bundle);

} // namespace hydrox
