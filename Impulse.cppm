module;

#include <algorithm>

export module Kairo.Foundation.PhysicsMath.Impulse;

import Kairo.Foundation.Math.Vector;
import Kairo.Foundation.Math.Matrix;
import Kairo.Foundation.PhysicsMath.Types;
import Kairo.Foundation.PhysicsMath.RigidBodyMath;

export namespace kairo::foundation::physics
{
    using namespace kairo::foundation::math;

    /// Input: inverse masses/inertias, contact offsets, and contact normal.
    /// Output: scalar effective mass denominator for normal impulse.
    /// Task: measure how much both bodies resist velocity change along a contact
    /// normal, including angular inertia from the contact arms.
    [[nodiscard]]
    inline float ContactNormalDenominator(
        float inverseMassA,
        const Matrix3f& inverseInertiaA,
        const Vec3f& rA,
        float inverseMassB,
        const Matrix3f& inverseInertiaB,
        const Vec3f& rB,
        const Vec3f& normal)
    {
        RequireNonNegative(inverseMassA, "inverseMassA");
        RequireNonNegative(inverseMassB, "inverseMassB");
        RequireFinite(rA, "rA");
        RequireFinite(rB, "rB");
        RequireFinite(normal, "normal");

        const Vec3f n =
            SafeNormalize(normal, Vec3f::Up());

        const Vec3f angularA =
            Cross(inverseInertiaA * Cross(rA, n), rA);

        const Vec3f angularB =
            Cross(inverseInertiaB * Cross(rB, n), rB);

        return inverseMassA + inverseMassB + Dot(n, angularA + angularB);
    }

    /// Input: relative normal velocity, restitution, and effective denominator.
    /// Output: non-negative normal impulse magnitude.
    /// Task: compute the scalar impulse used by sequential contact solvers.
    [[nodiscard]]
    inline float ComputeNormalImpulseMagnitude(
        float relativeNormalVelocity,
        float restitution,
        float denominator)
    {
        RequireFinite(relativeNormalVelocity, "relativeNormalVelocity");
        RequireNonNegative(restitution, "restitution");
        RequireNonNegative(denominator, "denominator");

        if (denominator <= 1.0e-6f || relativeNormalVelocity >= 0.0f)
        {
            return 0.0f;
        }

        return -(1.0f + restitution) * relativeNormalVelocity / denominator;
    }

    /// Input: tangent relative velocity, effective denominator, and friction cap.
    /// Output: tangent impulse vector clamped to Coulomb friction cone.
    /// Task: produce a stable friction impulse independent of solver ownership.
    [[nodiscard]]
    inline Vec3f ComputeFrictionImpulse(
        const Vec3f& relativeVelocity,
        const Vec3f& normal,
        float tangentDenominator,
        float maxFrictionImpulse)
    {
        RequireFinite(relativeVelocity, "relativeVelocity");
        RequireFinite(normal, "normal");
        RequireNonNegative(tangentDenominator, "tangentDenominator");
        RequireNonNegative(maxFrictionImpulse, "maxFrictionImpulse");

        const Vec3f n =
            SafeNormalize(normal, Vec3f::Up());

        const Vec3f tangentVelocity =
            relativeVelocity - n * Dot(relativeVelocity, n);

        const float tangentSpeed =
            tangentVelocity.Length();

        if (tangentSpeed <= 1.0e-6f || tangentDenominator <= 1.0e-6f)
        {
            return Vec3f::Zero();
        }

        const Vec3f tangent =
            tangentVelocity / tangentSpeed;

        const float unclampedMagnitude =
            -tangentSpeed / tangentDenominator;

        const float clampedMagnitude =
            std::clamp(unclampedMagnitude, -maxFrictionImpulse, maxFrictionImpulse);

        return tangent * clampedMagnitude;
    }
}
