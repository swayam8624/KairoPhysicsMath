module;

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

export module Kairo.Foundation.PhysicsMath.Types;

import Kairo.Foundation.Math.Vector;
import Kairo.Foundation.Math.Matrix;
import Kairo.Foundation.Math.Quaternion;

export namespace kairo::foundation::physics
{
    using namespace kairo::foundation::math;

    using BodyID = std::uint32_t;
    using ColliderID = std::uint32_t;

    using Matrix3f = Mat3f;
    using Quaternionf = Quatf;

    inline constexpr BodyID InvalidBodyID =
        std::numeric_limits<BodyID>::max();

    inline constexpr ColliderID InvalidColliderID =
        std::numeric_limits<ColliderID>::max();

    inline constexpr float Pi =
        3.14159265358979323846f;

    inline constexpr float DefaultGravityMagnitude =
        9.80665f;

    inline constexpr Vec3f DefaultGravity =
        Vec3f{ 0.0f, -DefaultGravityMagnitude, 0.0f };

    enum class BodyType : std::uint8_t
    {
        Static,
        Kinematic,
        Dynamic
    };

    struct MotionState final
    {
        Vec3f Position = Vec3f::Zero();
        Quaternionf Rotation = Quaternionf::Identity();
        Vec3f LinearVelocity = Vec3f::Zero();
        Vec3f AngularVelocity = Vec3f::Zero();
    };

    /// Input: scalar value and label used in diagnostics.
    /// Output: throws when the value is NaN or infinity.
    /// Task: centralize physical scalar validation so math helpers fail before
    /// invalid state enters an engine/world integration step.
    inline void RequireFinite(
        float value,
        const char* name)
    {
        if (!std::isfinite(value))
        {
            throw std::invalid_argument(std::string(name) + " must be finite.");
        }
    }

    /// Input: scalar value and label used in diagnostics.
    /// Output: throws when the value is not finite or is negative.
    /// Task: validate quantities such as mass, density, radius, and friction
    /// coefficients that cannot be physically negative in this library.
    inline void RequireNonNegative(
        float value,
        const char* name)
    {
        RequireFinite(value, name);
        if (value < 0.0f)
        {
            throw std::invalid_argument(std::string(name) + " must be non-negative.");
        }
    }

    /// Input: scalar value and label used in diagnostics.
    /// Output: throws when the value is not finite or is not strictly positive.
    /// Task: validate dimensions and time steps that must divide or represent
    /// real volume/intervals.
    inline void RequirePositive(
        float value,
        const char* name)
    {
        RequireFinite(value, name);
        if (value <= 0.0f)
        {
            throw std::invalid_argument(std::string(name) + " must be positive.");
        }
    }

    /// Input: 3D vector and label used in diagnostics.
    /// Output: throws when any component is NaN or infinity.
    /// Task: keep force, impulse, velocity, and shape parameters deterministic.
    inline void RequireFinite(
        const Vec3f& value,
        const char* name)
    {
        RequireFinite(value.x, name);
        RequireFinite(value.y, name);
        RequireFinite(value.z, name);
    }

    /// Input: 3D vector and label used in diagnostics.
    /// Output: throws when any component is non-finite or negative.
    /// Task: validate half-extents and similar component-wise positive measures.
    inline void RequireNonNegativeComponents(
        const Vec3f& value,
        const char* name)
    {
        RequireFinite(value, name);
        if (value.x < 0.0f || value.y < 0.0f || value.z < 0.0f)
        {
            throw std::invalid_argument(std::string(name) + " components must be non-negative.");
        }
    }

    /// Input: 3D vector and label used in diagnostics.
    /// Output: throws when any component is non-finite or not strictly positive.
    /// Task: validate box half-extents for non-degenerate mass properties.
    inline void RequirePositiveComponents(
        const Vec3f& value,
        const char* name)
    {
        RequireFinite(value, name);
        if (value.x <= 0.0f || value.y <= 0.0f || value.z <= 0.0f)
        {
            throw std::invalid_argument(std::string(name) + " components must be positive.");
        }
    }
}
