#include "mavlink_hil.h"

#include <array>
#include <cassert>
#include <cmath>

namespace
{
bool near(float lhs, float rhs, float tolerance = 1.0e-6f)
{
    return std::abs(lhs - rhs) <= tolerance;
}
}

int main()
{
    hydrox::MavlinkHIL codec;
    const std::array<float, 8> expected{
        0.1f, -0.2f, 0.3f, -0.4f, 0.5f, -0.6f, 0.7f, -0.8f};
    const auto bytes = codec.encode_hil_actuator_controls(
        expected, 123456u, 0x20u, 0x40000000u);
    const auto frames = codec.feed(bytes.data(), bytes.size());
    assert(frames.size() == 1);

    const auto decoded = codec.parse_hil_actuator_controls(frames.front());
    assert(decoded.valid);
    assert(decoded.time_usec == 123456u);
    assert(decoded.mode == 0x20u);
    assert(decoded.flags == 0x40000000u);
    for (std::size_t index = 0; index < expected.size(); ++index)
        assert(near(decoded.controls[index], expected[index]));
    for (std::size_t index = expected.size();
         index < decoded.controls.size(); ++index)
        assert(near(decoded.controls[index], 0.0f));
    return 0;
}
