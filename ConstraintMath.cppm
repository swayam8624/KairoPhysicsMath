module;

#include <algorithm>

export module Kairo.Foundation.PhysicsMath.ConstraintMath;

import Kairo.Foundation.Math.Vector;
import Kairo.Foundation.Math.Matrix;
import Kairo.Foundation.PhysicsMath.Types;

export namespace kairo::foundation::physics
{
    using namespace kairo::foundation::math;

    struct JacobianRow final
    {
        Vec3f LinearA = Vec3f::Zero();
        Vec3f AngularA = Vec3f::Zero();
        Vec3f LinearB = Vec3f::Zero();
        Vec3f AngularB = Vec3f::Zero();
    };

    /// Input: Jacobian row, inverse masses, and inverse inertia tensors.
    /// Output: scalar inverse effective mass denominator.
    /// Task: compute `J M^-1 J^T` for one constraint row.
    [[nodiscard]]
    inline float ConstraintDenominator(
        const JacobianRow& row,
        float inverseMassA,
        const Matrix3f& inverseInertiaA,
        float inverseMassB,
        const Matrix3f& inverseInertiaB)
    {
        RequireNonNegative(inverseMassA, "inverseMassA");
        RequireNonNegative(inverseMassB, "inverseMassB");

        return
            inverseMassA * Dot(row.LinearA, row.LinearA) +
            Dot(row.AngularA, inverseInertiaA * row.AngularA) +
            inverseMassB * Dot(row.LinearB, row.LinearB) +
            Dot(row.AngularB, inverseInertiaB * row.AngularB);
    }

    /// Input: constraint denominator.
    /// Output: effective mass scalar.
    /// Task: safely invert a constraint row denominator while preserving locked
    /// rows as zero effective mass.
    [[nodiscard]]
    inline float EffectiveMass(
        float denominator)
    {
        RequireNonNegative(denominator, "denominator");
        return denominator > 1.0e-6f ? 1.0f / denominator : 0.0f;
    }

    /// Input: positional error, Baumgarte factor, dt, and optional clamp.
    /// Output: velocity-level correction bias.
    /// Task: convert penetration/constraint error into a stable solver bias.
    [[nodiscard]]
    inline float BaumgarteBias(
        float error,
        float baumgarte,
        float dt,
        float maxBias = 1000.0f)
    {
        RequireFinite(error, "error");
        RequireNonNegative(baumgarte, "baumgarte");
        RequirePositive(dt, "dt");
        RequireNonNegative(maxBias, "maxBias");

        return std::clamp((baumgarte / dt) * error, -maxBias, maxBias);
    }

    /// Input: accumulated impulse, delta impulse, and min/max limits.
    /// Output: clamped delta impulse that should be applied this iteration.
    /// Task: support warm-started sequential impulse solvers without owning the
    /// solver loop or constraint storage.
    [[nodiscard]]
    inline float ClampAccumulatedImpulseDelta(
        float accumulatedImpulse,
        float deltaImpulse,
        float minImpulse,
        float maxImpulse)
    {
        RequireFinite(accumulatedImpulse, "accumulatedImpulse");
        RequireFinite(deltaImpulse, "deltaImpulse");
        RequireFinite(minImpulse, "minImpulse");
        RequireFinite(maxImpulse, "maxImpulse");

        const float previous =
            accumulatedImpulse;

        const float next =
            std::clamp(previous + deltaImpulse, minImpulse, maxImpulse);

        return next - previous;
    }
}
