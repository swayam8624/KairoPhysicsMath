module;

#include <vector>

export module Kairo.Foundation.PhysicsMath.MassProperties;

import Kairo.Foundation.Math.Vector;
import Kairo.Foundation.Math.Matrix;
import Kairo.Foundation.PhysicsMath.Types;
import Kairo.Foundation.PhysicsMath.InertiaTensor;

export namespace kairo::foundation::physics
{
    using namespace kairo::foundation::math;

    struct MassProperties final
    {
        float Mass = 0.0f;
        float InverseMass = 0.0f;
        Vec3f LocalCenterOfMass = Vec3f::Zero();
        Matrix3f LocalInertiaTensor = Matrix3f::Zero();
        Matrix3f LocalInverseInertiaTensor = Matrix3f::Zero();
    };

    /// Input: mass value.
    /// Output: reciprocal mass, or zero when mass is zero/static.
    /// Task: encode the physics convention that zero inverse mass means an
    /// object does not respond to linear impulses.
    [[nodiscard]]
    inline float InverseMassFor(
        float mass)
    {
        RequireNonNegative(mass, "mass");
        return mass > 0.0f ? 1.0f / mass : 0.0f;
    }

    /// Input: mass, center of mass, and local inertia tensor.
    /// Output: complete mass properties with inverse values computed.
    /// Task: provide one validated construction path for custom collider shapes.
    [[nodiscard]]
    inline MassProperties MakeMassProperties(
        float mass,
        const Vec3f& localCenterOfMass,
        const Matrix3f& localInertiaTensor)
    {
        RequireNonNegative(mass, "mass");
        RequireFinite(localCenterOfMass, "localCenterOfMass");

        return
        {
            mass,
            InverseMassFor(mass),
            localCenterOfMass,
            mass > 0.0f ? localInertiaTensor : Matrix3f::Zero(),
            mass > 0.0f ? InverseDiagonalInertia(localInertiaTensor) : Matrix3f::Zero()
        };
    }

    /// Input: none.
    /// Output: zero mass and zero inverse inertia.
    /// Task: represent static/infinite-mass bodies without special-case numbers.
    [[nodiscard]]
    inline MassProperties StaticMassProperties()
    {
        return {};
    }

    /// Input: sphere radius and density.
    /// Output: mass properties for a uniform solid sphere centered at origin.
    /// Task: convert authoring-friendly density into solver-friendly mass and
    /// inertia data.
    [[nodiscard]]
    inline MassProperties SphereMassProperties(
        float radius,
        float density)
    {
        RequirePositive(radius, "radius");
        RequireNonNegative(density, "density");

        const float volume =
            (4.0f / 3.0f) * Pi * radius * radius * radius;

        const float mass =
            density * volume;

        return MakeMassProperties(
            mass,
            Vec3f::Zero(),
            InertiaTensorSphere(radius, mass));
    }

    /// Input: box half-extents and density.
    /// Output: mass properties for a uniform solid box centered at origin.
    /// Task: follow the same half-extent convention used by Kairo geometry code.
    [[nodiscard]]
    inline MassProperties BoxMassProperties(
        const Vec3f& halfExtents,
        float density)
    {
        RequirePositiveComponents(halfExtents, "halfExtents");
        RequireNonNegative(density, "density");

        const float volume =
            8.0f * halfExtents.x * halfExtents.y * halfExtents.z;

        const float mass =
            density * volume;

        return MakeMassProperties(
            mass,
            Vec3f::Zero(),
            InertiaTensorBox(halfExtents, mass));
    }

    /// Input: capsule radius, cylinder height, and density.
    /// Output: mass properties for a Y-axis capsule centered at origin.
    /// Task: create the capsule mass convention that future collider code can
    /// reuse without embedding formulas in the physics engine.
    [[nodiscard]]
    inline MassProperties CapsuleMassProperties(
        float radius,
        float height,
        float density)
    {
        RequirePositive(radius, "radius");
        RequireNonNegative(height, "height");
        RequireNonNegative(density, "density");

        const float cylinderVolume =
            Pi * radius * radius * height;

        const float sphereVolume =
            (4.0f / 3.0f) * Pi * radius * radius * radius;

        const float mass =
            density * (cylinderVolume + sphereVolume);

        return MakeMassProperties(
            mass,
            Vec3f::Zero(),
            InertiaTensorCapsule(radius, height, mass));
    }

    /// Input: child mass properties expressed in the same local frame.
    /// Output: aggregate mass properties about the combined center of mass.
    /// Task: support compound rigid bodies before the engine has collider
    /// ownership or a broadphase/narrowphase layer.
    [[nodiscard]]
    inline MassProperties CombineMassProperties(
        const std::vector<MassProperties>& parts)
    {
        float totalMass = 0.0f;
        Vec3f weightedCenter = Vec3f::Zero();

        for (const MassProperties& part : parts)
        {
            RequireNonNegative(part.Mass, "part.Mass");
            totalMass += part.Mass;
            weightedCenter += part.LocalCenterOfMass * part.Mass;
        }

        if (totalMass == 0.0f)
        {
            return StaticMassProperties();
        }

        const Vec3f center =
            weightedCenter / totalMass;

        Matrix3f inertia =
            Matrix3f::Zero();

        for (const MassProperties& part : parts)
        {
            const Vec3f offset =
                part.LocalCenterOfMass - center;

            inertia +=
                ParallelAxisTheorem(
                    part.LocalInertiaTensor,
                    part.Mass,
                    offset);
        }

        return MakeMassProperties(totalMass, center, inertia);
    }
}
