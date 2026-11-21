// TEACHING MODULE: Rigidbody kinematics and conservation quantities, independent of collision
// shape ownership. MotionState is the common state record. Impulses change velocity
// directly, whereas forces/torques integrate over dt. The physics world controls
// when these helpers execute; KairoMath provides vectors, matrices and cross products.
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
    // TEACHING DERIVATION: VelocityAtPoint
    // At offset r from center of mass the point velocity is v_COM + omega cross r;
    // ContactSolver uses this when computing relative velocity at a collision.
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
    // TEACHING DERIVATION: ApplyForce
    // Continuous acceleration update: dv = (F/m)*dt. Static inverseMass 0 leaves velocity unchanged.
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
    // TEACHING DERIVATION: ApplyTorque
    // Angular acceleration update: dOmega = I_world^-1 * torque * dt.
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
    // TEACHING DERIVATION: ApplyLinearImpulse
    // A collision impulse J produces instant dv = J/m; no dt factor belongs here.
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
    // TEACHING DERIVATION: ApplyAngularImpulse
    // Angular impulse L changes angular velocity by I_world^-1 * L.
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
    // TEACHING DERIVATION: ApplyImpulseAtPoint
    // The same impulse changes linear motion and adds angular impulse (point-COM) cross J.
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
    // TEACHING DERIVATION: LinearMomentum
    // p=m*v supports momentum-conservation diagnostics for isolated systems.
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
    // TEACHING DERIVATION: AngularMomentum
    // L=I_world*omega; angular momentum depends on orientation through world inertia.
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
    // TEACHING DERIVATION: LinearKineticEnergy
    // Translational energy = 0.5*m*dot(v,v); useful to detect explosive solver work.
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
    // TEACHING DERIVATION: AngularKineticEnergy
    // Rotational energy = 0.5*dot(omega, I_world*omega).
    inline float AngularKineticEnergy(
        const Matrix3f& worldInertia,
        const Vec3f& angularVelocity)
    {
        RequireFinite(angularVelocity, "angularVelocity");
        return 0.5f * Dot(angularVelocity, worldInertia * angularVelocity);
    }
}
