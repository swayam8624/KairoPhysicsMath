// TEACHING MODULE: Continuous forces enter an accumulator (N, N*m) and are integrated across
// dt; discrete impulses act immediately on velocity and are a different API.
// ForceAtPoint also produces torque r cross F about center of mass.
// PhysicsEngine decides when to clear accumulators and integrate.
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
    // TEACHING DERIVATION: AddForce
    // Add a world-space force vector; it has not yet changed velocity.
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
    // TEACHING DERIVATION: AddTorque
    // Add torque about center of mass to be integrated into angular acceleration.
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
    // TEACHING DERIVATION: AddForceAtPoint
    // Split the applied world force into translation F and torque (point-COM) cross F.
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
    // TEACHING DERIVATION: ClearForces
    // Zero accumulated force and torque after an integration interval, not persistent body state.
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
    // TEACHING DERIVATION: GravityForce
    // Compute F=m*g for dynamic mass; g is acceleration in m/s^2, not a force.
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
    // TEACHING DERIVATION: DragForce
    // Approximate dissipative force opposing motion; sign and coefficient convention
    // belong to this helper and should not be confused with fluid dynamics.
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
    // TEACHING DERIVATION: SpringForce
    // Apply Hooke-style force along the separation axis using extension from rest length.
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
