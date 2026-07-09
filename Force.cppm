module;

#include <algorithm>

export module Kairo.Foundation.PhysicsMath.Force;

import Kairo.Foundation.Math.Vector;
import Kairo.Foundation.PhysicsMath.Types;

export namespace kairo::foundation::physics
{
    using namespace kairo::foundation::math;

    struct ForceAccumulation final
    {
        Vec3f Force = Vec3f::Zero();
        Vec3f Torque = Vec3f::Zero();
    };

    /// Input: force accumulator and world-space force.
    /// Output: accumulator with force added.
    /// Task: collect continuous linear forces before integration.
    inline void AddForce(
        ForceAccumulation& accumulation,
        const Vec3f& force)
    {
        RequireFinite(force, "force");
        accumulation.Force += force;
    }

    /// Input: force accumulator and world-space torque.
    /// Output: accumulator with torque added.
    /// Task: collect continuous angular forces before integration.
    inline void AddTorque(
        ForceAccumulation& accumulation,
        const Vec3f& torque)
    {
        RequireFinite(torque, "torque");
        accumulation.Torque += torque;
    }

    /// Input: force, application point, and center of mass in world space.
    /// Output: accumulator with force and torque added.
    /// Task: convert off-center force into both linear force and angular torque
    /// using `torque = cross(point - centerOfMass, force)`.
    inline void AddForceAtPoint(
        ForceAccumulation& accumulation,
        const Vec3f& force,
        const Vec3f& worldPoint,
        const Vec3f& worldCenterOfMass)
    {
        RequireFinite(force, "force");
        RequireFinite(worldPoint, "worldPoint");
        RequireFinite(worldCenterOfMass, "worldCenterOfMass");

        accumulation.Force += force;
        accumulation.Torque += Cross(worldPoint - worldCenterOfMass, force);
    }

    /// Input: force accumulator.
    /// Output: zeroed force and torque.
    /// Task: reset per-step force state after integration.
    inline void ClearForces(
        ForceAccumulation& accumulation) noexcept
    {
        accumulation = {};
    }

    /// Input: mass and gravity acceleration.
    /// Output: force vector `mass * gravity`.
    /// Task: provide a single convention for gravity as acceleration rather
    /// than an arbitrary world force.
    [[nodiscard]]
    inline Vec3f GravityForce(
        float mass,
        const Vec3f& gravity = DefaultGravity)
    {
        RequireNonNegative(mass, "mass");
        RequireFinite(gravity, "gravity");
        return gravity * mass;
    }

    /// Input: velocity and drag coefficient.
    /// Output: drag force opposing velocity.
    /// Task: model simple linear drag for demos and tests without fluid state.
    [[nodiscard]]
    inline Vec3f DragForce(
        const Vec3f& velocity,
        float linearDrag)
    {
        RequireFinite(velocity, "velocity");
        RequireNonNegative(linearDrag, "linearDrag");
        return velocity * -linearDrag;
    }

    /// Input: two endpoints, rest length, and spring stiffness.
    /// Output: spring force applied to point A toward/away from point B.
    /// Task: provide Hooke-law force math for later constraints and demos.
    [[nodiscard]]
    inline Vec3f SpringForce(
        const Vec3f& pointA,
        const Vec3f& pointB,
        float restLength,
        float stiffness)
    {
        RequireFinite(pointA, "pointA");
        RequireFinite(pointB, "pointB");
        RequireNonNegative(restLength, "restLength");
        RequireNonNegative(stiffness, "stiffness");

        const Vec3f delta =
            pointB - pointA;

        const float length =
            delta.Length();

        if (length <= 1.0e-6f)
        {
            return Vec3f::Zero();
        }

        return (delta / length) * ((length - restLength) * stiffness);
    }
}
