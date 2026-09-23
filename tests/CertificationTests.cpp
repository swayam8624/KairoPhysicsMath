#include <catch2/catch_test_macros.hpp>

#include <cmath>

import Kairo.Foundation.PhysicsMath;
import Kairo.Foundation.Math;

using namespace kairo::foundation::physics;
using namespace kairo::foundation::math;

TEST_CASE("Certification: primitive mass properties remain finite across scale range", "[PhysicsMath][Certification]")
{
    for (int index = 1; index <= 512; ++index)
    {
        const float radius = 0.01f + static_cast<float>(index) * 0.01f;
        const float density = 0.1f + static_cast<float>(index % 31) * 0.25f;

        const MassProperties sphere = SphereMassProperties(radius, density);
        REQUIRE(std::isfinite(sphere.Mass));
        REQUIRE(sphere.Mass > 0.0f);
        REQUIRE(sphere.InverseMass > 0.0f);
        REQUIRE(std::abs((sphere.Mass * sphere.InverseMass) - 1.0f) < 2.0e-5f);
        REQUIRE(sphere.LocalInertiaTensor(0, 0) > 0.0f);
        REQUIRE(sphere.LocalInertiaTensor(1, 1) > 0.0f);
        REQUIRE(sphere.LocalInertiaTensor(2, 2) > 0.0f);
    }
}

TEST_CASE("Certification: angular integration maintains normalized orientation", "[PhysicsMath][Certification]")
{
    Quaternionf orientation = Quaternionf::Identity();
    const Vec3f angularVelocity{ 0.7f, -1.1f, 0.35f };
    constexpr float dt = 1.0f / 240.0f;

    for (int step = 0; step < 20000; ++step)
    {
        orientation = IntegrateAngularVelocity(orientation, angularVelocity, dt);
        REQUIRE(orientation.IsNormalized(2.0e-4f));
    }
}

TEST_CASE("Certification: center-of-mass impulses obey linear impulse relation", "[PhysicsMath][Certification]")
{
    for (int index = 1; index <= 256; ++index)
    {
        MotionState state;
        const float inverseMass = 1.0f / (0.5f + static_cast<float>(index) * 0.02f);
        const Vec3f impulse{
            static_cast<float>((index % 11) - 5) * 0.2f,
            static_cast<float>((index % 7) - 3) * 0.3f,
            static_cast<float>((index % 5) - 2) * 0.4f
        };

        ApplyImpulseAtPoint(
            state,
            inverseMass,
            Matrix3f::Identity(),
            impulse,
            Vec3f::Zero());

        const Vec3f expected = impulse * inverseMass;
        REQUIRE(NearlyEqual(state.LinearVelocity, expected, 1.0e-5f));
        REQUIRE(NearlyEqual(state.AngularVelocity, Vec3f::Zero(), 1.0e-6f));
    }
}
