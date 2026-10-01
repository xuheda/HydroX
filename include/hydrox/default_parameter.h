#pragma once

namespace hydrox::detail
{
    // GCC cannot bind `{}` to a nested Params reference while the enclosing
    // class is incomplete. A shared immutable object preserves zero-argument
    // construction without duplicating constructor implementations.
    template<typename T>
    const T &default_parameter()
    {
        static const T value{};
        return value;
    }
} // namespace hydrox::detail
