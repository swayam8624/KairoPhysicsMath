module;

#include <algorithm>
#include <cmath>

export module Kairo.Foundation.PhysicsMath.InertiaTensor;

import Kairo.Foundation.Math.Vector;
import Kairo.Foundation.Math.Matrix;
import Kairo.Foundation.Math.Quaternion;
import Kairo.Foundation.PhysicsMath.Types;

export namespace kairo::foundation::physics
{
    using namespace kairo::foundation::math;

    /// Input: diagonal inertia components.
    /// Output: 3x3 diagonal inertia tensor.
    /// Task: create local inertia tensors without repeating row-major zeros.
    [[nodiscard]]
    inline Matrix3f DiagonalInertia(
        float ixx,
        float iyy,
        float izz)
    {
        RequireNonNegative(ixx, "ixx");
        RequireNonNegative(iyy, "iyy");
        RequireNonNegative(izz, "izz");

        return Matrix3f
        {
            ixx, 0.0f, 0.0f,
            0.0f, iyy, 0.0f,
            0.0f, 0.0f, izz
        };
    }

    /// Input: inertia tensor assumed diagonal in local principal axes.
    /// Output: inverse tensor with zero preserved for locked/static axes.
    /// Task: avoid calling a generic inverse on common diagonal inertia tensors
    /// where zero means infinite inertia, not a numerical accident.
    [[nodiscard]]
    inline Matrix3f InverseDiagonalInertia(
        const Matrix3f& inertia)
    {
        return DiagonalInertia(
            inertia(0, 0) > 0.0f ? 1.0f / inertia(0, 0) : 0.0f,
            inertia(1, 1) > 0.0f ? 1.0f / inertia(1, 1) : 0.0f,
            inertia(2, 2) > 0.0f ? 1.0f / inertia(2, 2) : 0.0f);
    }

    /// Input: sphere radius and mass.
    /// Output: local inertia tensor about the sphere center.
    /// Task: support rotational response for uniform solid spheres.
    [[nodiscard]]
    inline Matrix3f InertiaTensorSphere(
        float radius,
        float mass)
    {
        RequirePositive(radius, "radius");
        RequireNonNegative(mass, "mass");

        const float value =
            0.4f * mass * radius * radius;

        return DiagonalInertia(value, value, value);
    }

    /// Input: box half-extents and mass.
    /// Output: local inertia tensor about the box center.
    /// Task: support uniform solid boxes with Kairo's half-extent convention.
    [[nodiscard]]
    inline Matrix3f InertiaTensorBox(
        const Vec3f& halfExtents,
        float mass)
    {
        RequirePositiveComponents(halfExtents, "halfExtents");
        RequireNonNegative(mass, "mass");

        const float x2 =
            halfExtents.x * halfExtents.x;

        const float y2 =
            halfExtents.y * halfExtents.y;

        const float z2 =
            halfExtents.z * halfExtents.z;

        const float scale =
            mass / 3.0f;

        return DiagonalInertia(
            scale * (y2 + z2),
            scale * (x2 + z2),
            scale * (x2 + y2));
    }

    /// Input: capsule radius, cylinder height, and mass.
    /// Output: approximate local inertia tensor for a Y-axis capsule.
    /// Task: provide stable V1 rotational mass for capsule bodies. The model
    /// treats the capsule as a cylinder plus spherical cap mass and is intended
    /// for engine gameplay stability, not CAD-grade analytic precision.
    [[nodiscard]]
    inline Matrix3f InertiaTensorCapsule(
        float radius,
        float height,
        float mass)
    {
        RequirePositive(radius, "radius");
        RequireNonNegative(height, "height");
        RequireNonNegative(mass, "mass");

        const float cylinderVolume =
            Pi * radius * radius * height;

        const float sphereVolume =
            (4.0f / 3.0f) * Pi * radius * radius * radius;

        const float totalVolume =
            cylinderVolume + sphereVolume;

        if (mass == 0.0f || totalVolume == 0.0f)
        {
            return Matrix3f::Zero();
        }

        const float cylinderMass =
            mass * (cylinderVolume / totalVolume);

        const float sphereMass =
            mass - cylinderMass;

        const float iAxis =
            0.5f * cylinderMass * radius * radius +
            0.4f * sphereMass * radius * radius;

        const float iSide =
            (cylinderMass * (3.0f * radius * radius + height * height) / 12.0f) +
            0.4f * sphereMass * radius * radius +
            sphereMass * (height * height * 0.25f);

        return DiagonalInertia(iSide, iAxis, iSide);
    }

    /// Input: local inertia tensor about center of mass, mass, and local offset.
    /// Output: inertia tensor shifted to a parallel axis.
    /// Task: combine child shapes whose centers of mass are offset from the
    /// aggregate body center.
    [[nodiscard]]
    inline Matrix3f ParallelAxisTheorem(
        const Matrix3f& localInertia,
        float mass,
        const Vec3f& offset)
    {
        RequireNonNegative(mass, "mass");
        RequireFinite(offset, "offset");

        const float d2 =
            Dot(offset, offset);

        return localInertia +
            mass * ((d2 * Matrix3f::Identity()) - OuterProduct(offset, offset));
    }

    /// Input: local inverse inertia and current orientation.
    /// Output: inverse inertia tensor in world space.
    /// Task: transform angular impulses/torques from local principal axes into
    /// world coordinates using `R * I^-1 * R^T`.
    [[nodiscard]]
    inline Matrix3f InverseInertiaWorld(
        const Matrix3f& localInverseInertia,
        const Quaternionf& rotation)
    {
        const Matrix3f r =
            ToMatrix3(rotation);

        return r * localInverseInertia * Transpose(r);
    }
}
