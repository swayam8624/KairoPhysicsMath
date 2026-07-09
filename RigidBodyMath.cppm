module;

export module Kairo.Foundation.PhysicsMath.RigidBodyMath;

import Kairo.Foundation.Math.Vector;
import Kairo.Foundation.Math.Matrix;
import Kairo.Foundation.PhysicsMath.Types;

export namespace kairo::foundation::physics
{
    using namespace kairo::foundation::math;

    /// Input: motion state and world-space point.
    /// Output: linear velocity of that point on the rigid body.
    /// Task: combine center-of-mass velocity with angular contribution for
    /// contact and constraint relative velocity calculations.
    [[nodiscard]]
    inline Vec3f VelocityAtPoint(
        const MotionState& state,
        const Vec3f& worldPoint)
    {
        RequireFinite(state.Position, "state.Position");
        RequireFinite(state.LinearVelocity, "state.LinearVelocity");
        RequireFinite(state.AngularVelocity, "state.AngularVelocity");
        RequireFinite(worldPoint, "worldPoint");

        return state.LinearVelocity +
            Cross(state.AngularVelocity, worldPoint - state.Position);
    }

    /// Input: velocity, inverse mass, and force integrated over dt.
    /// Output: updated velocity.
    /// Task: apply continuous force to linear velocity without owning a body.
    inline void ApplyForce(
        Vec3f& linearVelocity,
        float inverseMass,
        const Vec3f& force,
        float dt)
    {
        RequireFinite(linearVelocity, "linearVelocity");
        RequireNonNegative(inverseMass, "inverseMass");
        RequireFinite(force, "force");
        RequirePositive(dt, "dt");

        linearVelocity += force * (inverseMass * dt);
    }

    /// Input: angular velocity, inverse inertia, torque, and dt.
    /// Output: updated angular velocity.
    /// Task: apply continuous torque through world-space inverse inertia.
    inline void ApplyTorque(
        Vec3f& angularVelocity,
        const Matrix3f& worldInverseInertia,
        const Vec3f& torque,
        float dt)
    {
        RequireFinite(angularVelocity, "angularVelocity");
        RequireFinite(torque, "torque");
        RequirePositive(dt, "dt");

        angularVelocity += worldInverseInertia * torque * dt;
    }

    /// Input: velocity, inverse mass, and instantaneous impulse.
    /// Output: updated linear velocity.
    /// Task: apply collision/constraint impulses that change velocity instantly.
    inline void ApplyLinearImpulse(
        Vec3f& linearVelocity,
        float inverseMass,
        const Vec3f& impulse)
    {
        RequireFinite(linearVelocity, "linearVelocity");
        RequireNonNegative(inverseMass, "inverseMass");
        RequireFinite(impulse, "impulse");

        linearVelocity += impulse * inverseMass;
    }

    /// Input: angular velocity, inverse inertia, and angular impulse.
    /// Output: updated angular velocity.
    /// Task: apply direct angular impulse in world coordinates.
    inline void ApplyAngularImpulse(
        Vec3f& angularVelocity,
        const Matrix3f& worldInverseInertia,
        const Vec3f& angularImpulse)
    {
        RequireFinite(angularVelocity, "angularVelocity");
        RequireFinite(angularImpulse, "angularImpulse");

        angularVelocity += worldInverseInertia * angularImpulse;
    }

    /// Input: state, inverse mass/inertia, impulse, and point of application.
    /// Output: updated linear and angular velocities.
    /// Task: apply a contact impulse at a point using `r x impulse` torque arm.
    inline void ApplyImpulseAtPoint(
        MotionState& state,
        float inverseMass,
        const Matrix3f& worldInverseInertia,
        const Vec3f& impulse,
        const Vec3f& worldPoint)
    {
        RequireFinite(state.Position, "state.Position");
        RequireFinite(impulse, "impulse");
        RequireFinite(worldPoint, "worldPoint");

        ApplyLinearImpulse(state.LinearVelocity, inverseMass, impulse);
        ApplyAngularImpulse(
            state.AngularVelocity,
            worldInverseInertia,
            Cross(worldPoint - state.Position, impulse));
    }

    /// Input: mass and linear velocity.
    /// Output: linear momentum `m * v`.
    /// Task: expose a conserved quantity useful for tests and debug output.
    [[nodiscard]]
    inline Vec3f LinearMomentum(
        float mass,
        const Vec3f& linearVelocity)
    {
        RequireNonNegative(mass, "mass");
        RequireFinite(linearVelocity, "linearVelocity");
        return linearVelocity * mass;
    }

    /// Input: world inertia and angular velocity.
    /// Output: angular momentum `I * omega`.
    /// Task: expose rotational momentum for validation and diagnostics.
    [[nodiscard]]
    inline Vec3f AngularMomentum(
        const Matrix3f& worldInertia,
        const Vec3f& angularVelocity)
    {
        RequireFinite(angularVelocity, "angularVelocity");
        return worldInertia * angularVelocity;
    }

    /// Input: mass and velocity.
    /// Output: translational kinetic energy.
    /// Task: support energy checks for integrator and solver tests.
    [[nodiscard]]
    inline float LinearKineticEnergy(
        float mass,
        const Vec3f& linearVelocity)
    {
        RequireNonNegative(mass, "mass");
        RequireFinite(linearVelocity, "linearVelocity");
        return 0.5f * mass * Dot(linearVelocity, linearVelocity);
    }

    /// Input: world inertia and angular velocity.
    /// Output: rotational kinetic energy.
    /// Task: support angular energy diagnostics without engine state.
    [[nodiscard]]
    inline float AngularKineticEnergy(
        const Matrix3f& worldInertia,
        const Vec3f& angularVelocity)
    {
        RequireFinite(angularVelocity, "angularVelocity");
        return 0.5f * Dot(angularVelocity, worldInertia * angularVelocity);
    }
}
