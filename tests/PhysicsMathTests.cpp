#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <stdexcept>

import Kairo.Foundation.PhysicsMath;
import Kairo.Foundation.Math.Vector;
import Kairo.Foundation.Math.Matrix;
import Kairo.Foundation.Math.Quaternion;

using namespace kairo::foundation::physics;
using namespace kairo::foundation::math;

namespace
{
    void RequireVecNear(
        const Vec3f& actual,
        const Vec3f& expected,
        float tolerance = 1.0e-5f)
    {
        REQUIRE(actual.x == Catch::Approx(expected.x).margin(tolerance));
        REQUIRE(actual.y == Catch::Approx(expected.y).margin(tolerance));
        REQUIRE(actual.z == Catch::Approx(expected.z).margin(tolerance));
    }
}

TEST_CASE("Mass properties compute sphere box capsule and static bodies", "[PhysicsMath][Mass]")
{
    const MassProperties sphere =
        SphereMassProperties(1.0f, 2.0f);

    REQUIRE(sphere.Mass == Catch::Approx((8.0f / 3.0f) * Pi));
    REQUIRE(sphere.InverseMass == Catch::Approx(1.0f / sphere.Mass));
    REQUIRE(sphere.LocalInertiaTensor(0, 0) == Catch::Approx(0.4f * sphere.Mass));

    const MassProperties box =
        BoxMassProperties(Vec3f{ 1.0f, 2.0f, 3.0f }, 0.5f);

    REQUIRE(box.Mass == Catch::Approx(24.0f));
    REQUIRE(box.LocalInertiaTensor(0, 0) == Catch::Approx((box.Mass / 3.0f) * (4.0f + 9.0f)));

    const MassProperties capsule =
        CapsuleMassProperties(0.5f, 2.0f, 1.25f);

    REQUIRE(capsule.Mass > 0.0f);
    REQUIRE(capsule.LocalInertiaTensor(0, 0) > capsule.LocalInertiaTensor(1, 1));

    const MassProperties stat =
        StaticMassProperties();

    REQUIRE(stat.Mass == 0.0f);
    REQUIRE(stat.InverseMass == 0.0f);
    REQUIRE(stat.LocalInverseInertiaTensor(0, 0) == 0.0f);
}

TEST_CASE("Inertia tensors transform and shift correctly", "[PhysicsMath][Inertia]")
{
    const Matrix3f localInverse =
        DiagonalInertia(1.0f, 2.0f, 3.0f);

    const Matrix3f identityWorld =
        InverseInertiaWorld(localInverse, Quaternionf::Identity());

    REQUIRE(identityWorld(0, 0) == Catch::Approx(1.0f));
    REQUIRE(identityWorld(1, 1) == Catch::Approx(2.0f));
    REQUIRE(identityWorld(2, 2) == Catch::Approx(3.0f));

    const Matrix3f rotated =
        InverseInertiaWorld(localInverse, AxisAngle(Vec3f::Forward(), Pi * 0.5f));

    REQUIRE(rotated(0, 0) == Catch::Approx(2.0f).margin(1.0e-5f));
    REQUIRE(rotated(1, 1) == Catch::Approx(1.0f).margin(1.0e-5f));
    REQUIRE(rotated(2, 2) == Catch::Approx(3.0f).margin(1.0e-5f));

    const Matrix3f shifted =
        ParallelAxisTheorem(Matrix3f::Zero(), 2.0f, Vec3f{ 1.0f, 0.0f, 0.0f });

    REQUIRE(shifted(0, 0) == Catch::Approx(0.0f));
    REQUIRE(shifted(1, 1) == Catch::Approx(2.0f));
    REQUIRE(shifted(2, 2) == Catch::Approx(2.0f));
}

TEST_CASE("Forces and rigid body math update velocities and diagnostics", "[PhysicsMath][RigidBody]")
{
    ForceAccumulation accumulation;
    AddForceAtPoint(
        accumulation,
        Vec3f{ 0.0f, 10.0f, 0.0f },
        Vec3f{ 1.0f, 0.0f, 0.0f },
        Vec3f::Zero());

    RequireVecNear(accumulation.Force, Vec3f{ 0.0f, 10.0f, 0.0f });
    RequireVecNear(accumulation.Torque, Vec3f{ 0.0f, 0.0f, 10.0f });

    MotionState state;
    state.LinearVelocity = Vec3f{ 1.0f, 0.0f, 0.0f };
    state.AngularVelocity = Vec3f{ 0.0f, 0.0f, 2.0f };

    RequireVecNear(
        VelocityAtPoint(state, Vec3f{ 0.0f, 1.0f, 0.0f }),
        Vec3f{ -1.0f, 0.0f, 0.0f });

    ApplyImpulseAtPoint(
        state,
        0.5f,
        Matrix3f::Identity(),
        Vec3f{ 0.0f, 2.0f, 0.0f },
        Vec3f{ 1.0f, 0.0f, 0.0f });

    RequireVecNear(state.LinearVelocity, Vec3f{ 1.0f, 1.0f, 0.0f });
    RequireVecNear(state.AngularVelocity, Vec3f{ 0.0f, 0.0f, 4.0f });

    REQUIRE(LinearKineticEnergy(2.0f, Vec3f{ 3.0f, 0.0f, 0.0f }) == Catch::Approx(9.0f));
    RequireVecNear(LinearMomentum(2.0f, Vec3f{ 1.0f, 2.0f, 3.0f }), Vec3f{ 2.0f, 4.0f, 6.0f });
}

TEST_CASE("Integrators advance deterministic motion", "[PhysicsMath][Integrators]")
{
    MotionState state;
    state.Position = Vec3f::Zero();
    state.LinearVelocity = Vec3f{ 1.0f, 0.0f, 0.0f };

    const MotionState semi =
        SemiImplicitEuler(state, Vec3f{ 0.0f, -10.0f, 0.0f }, Vec3f::Zero(), 0.1f);

    RequireVecNear(semi.LinearVelocity, Vec3f{ 1.0f, -1.0f, 0.0f });
    RequireVecNear(semi.Position, Vec3f{ 0.1f, -0.1f, 0.0f });

    const Vec3f verlet =
        VerletPosition(Vec3f{ 1.0f, 0.0f, 0.0f }, Vec3f::Zero(), Vec3f{ 0.0f, 2.0f, 0.0f }, 0.5f);

    RequireVecNear(verlet, Vec3f{ 2.0f, 0.5f, 0.0f });

    const Quaternionf rotated =
        IntegrateAngularVelocity(Quaternionf::Identity(), Vec3f{ 0.0f, 0.0f, Pi }, 0.1f);

    REQUIRE(rotated.IsNormalized(1.0e-5f));

    const float rk4 =
        RK4(1.0f, 0.1f, [](float y) { return y; });

    REQUIRE(rk4 == Catch::Approx(std::exp(0.1f)).margin(1.0e-5f));
}

TEST_CASE("Impulse and constraint helpers produce solver scalars", "[PhysicsMath][SolverMath]")
{
    const float denominator =
        ContactNormalDenominator(
            1.0f,
            Matrix3f::Identity(),
            Vec3f{ 0.0f, 1.0f, 0.0f },
            1.0f,
            Matrix3f::Identity(),
            Vec3f{ 0.0f, -1.0f, 0.0f },
            Vec3f{ 1.0f, 0.0f, 0.0f });

    REQUIRE(denominator == Catch::Approx(4.0f));

    REQUIRE(ComputeNormalImpulseMagnitude(-2.0f, 0.5f, denominator) == Catch::Approx(0.75f));

    const Vec3f friction =
        ComputeFrictionImpulse(
            Vec3f{ 3.0f, -1.0f, 0.0f },
            Vec3f::Up(),
            2.0f,
            1.0f);

    RequireVecNear(friction, Vec3f{ -1.0f, 0.0f, 0.0f });

    const JacobianRow row
    {
        Vec3f{ 1.0f, 0.0f, 0.0f },
        Vec3f::Zero(),
        Vec3f{ -1.0f, 0.0f, 0.0f },
        Vec3f::Zero()
    };

    REQUIRE(ConstraintDenominator(row, 0.5f, Matrix3f::Identity(), 0.5f, Matrix3f::Identity()) == Catch::Approx(1.0f));
    REQUIRE(EffectiveMass(4.0f) == Catch::Approx(0.25f));
    REQUIRE(BaumgarteBias(0.1f, 0.2f, 0.02f) == Catch::Approx(1.0f));
    REQUIRE(ClampAccumulatedImpulseDelta(0.8f, 0.5f, 0.0f, 1.0f) == Catch::Approx(0.2f));
}

TEST_CASE("Contacts store validated manifold data", "[PhysicsMath][Contact]")
{
    ContactManifold manifold =
        MakeContactManifold(3, 7);

    manifold.Points.push_back(
        MakeContactPoint(
            Vec3f{ 1.0f, 2.0f, 3.0f },
            Vec3f{ 0.0f, 5.0f, 0.0f },
            0.25f,
            0.5f,
            -0.1f));

    REQUIRE(manifold.A == 3);
    REQUIRE(manifold.B == 7);
    REQUIRE(manifold.Points.size() == 1);
    RequireVecNear(manifold.Points[0].Normal, Vec3f::Up());
}

TEST_CASE("Invalid physical inputs throw clearly", "[PhysicsMath][Validation]")
{
    Vec3f velocity = Vec3f::Zero();

    REQUIRE_THROWS_AS(SphereMassProperties(-1.0f, 1.0f), std::invalid_argument);
    REQUIRE_THROWS_AS(BoxMassProperties(Vec3f{ 1.0f, 0.0f, 1.0f }, 1.0f), std::invalid_argument);
    REQUIRE_THROWS_AS(ApplyForce(velocity, 1.0f, Vec3f::Zero(), 0.0f), std::invalid_argument);
    REQUIRE_THROWS_AS(BaumgarteBias(1.0f, 0.2f, 0.0f), std::invalid_argument);
    REQUIRE_THROWS_AS(MakeContactPoint(Vec3f::Zero(), Vec3f::Up(), -0.1f), std::invalid_argument);
}
