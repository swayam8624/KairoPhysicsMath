// TEACHING MODULE: Numerical integration approximates differential equations at discrete dt.
// The listed choices differ in stability and accuracy, not by magically solving
// contacts: PhysicsWorld must supply forces, constraints and a fixed timestep.
// Angular velocity uses rad/s, orientations are unit quaternions.
// TEACHING MAP: Time-integration math advances physical state under forces. KairoPhysicsEngine owns contacts, bodies and fixed-step scheduling; integration alone does not resolve collisions or constraint stability.
module;

export module Kairo.Foundation.PhysicsMath.Integrators;

import Kairo.Foundation.Math.Vector;
import Kairo.Foundation.Math.Quaternion;
import Kairo.Foundation.PhysicsMath.Types;

export namespace kairo::foundation::physics
{
    using namespace kairo::foundation::math;

    /// Input: current orientation, angular velocity in radians/sec, and dt.
    /// Output: normalized orientation advanced by angular velocity.
    /// Task: integrate rigid-body orientation robustly and keep unit quaternion
    /// drift from accumulating across simulation steps.
    [[nodiscard]]
    // TEACHING DERIVATION: IntegrateAngularVelocity
    // Use quaternion derivative 0.5*(omega_quaternion*rotation), then normalize;
    // first-order integration is not exact under rapidly varying angular speed.
    inline Quaternionf IntegrateAngularVelocity(
        const Quaternionf& rotation,
        const Vec3f& angularVelocity,
        float dt)
    {
        RequireFinite(angularVelocity, "angularVelocity");
        RequirePositive(dt, "dt");

        const Quaternionf omega
        {
            angularVelocity,
            0.0f
        };

        const Quaternionf derivative =
            omega * rotation * 0.5f;

        return (rotation + derivative * dt).Normalized();
    }

    /// Input: position, velocity, acceleration, and dt.
    /// Output: position advanced by explicit Euler.
    /// Task: expose the simplest integrator for comparison and education.
    [[nodiscard]]
    // TEACHING DERIVATION: ExplicitEulerPosition
    // x_next = x + v_current*dt; acceleration is not an input despite an older
    // nearby doc phrase suggesting otherwise.
    inline Vec3f ExplicitEulerPosition(
        const Vec3f& position,
        const Vec3f& velocity,
        float dt)
    {
        RequireFinite(position, "position");
        RequireFinite(velocity, "velocity");
        RequirePositive(dt, "dt");
        return position + velocity * dt;
    }

    /// Input: motion state, linear/angular acceleration, and dt.
    /// Output: state advanced by explicit Euler.
    /// Task: provide a baseline integrator where position uses pre-step velocity.
    [[nodiscard]]
    inline MotionState ExplicitEuler(
        MotionState state,
        const Vec3f& linearAcceleration,
        const Vec3f& angularAcceleration,
        float dt)
    {
        RequireFinite(linearAcceleration, "linearAcceleration");
        RequireFinite(angularAcceleration, "angularAcceleration");
        RequirePositive(dt, "dt");

        state.Position += state.LinearVelocity * dt;
        state.Rotation = IntegrateAngularVelocity(state.Rotation, state.AngularVelocity, dt);
        state.LinearVelocity += linearAcceleration * dt;
        state.AngularVelocity += angularAcceleration * dt;
        return state;
    }

    /// Input: motion state, linear/angular acceleration, and dt.
    /// Output: state advanced by semi-implicit Euler.
    /// Task: use the stable default for future physics engine V1 where velocity
    /// is updated before position.
    [[nodiscard]]
    // TEACHING DERIVATION: SemiImplicitEuler
    // Update velocities first, then pose using new velocity; frequently more
    // stable for real-time forces than explicit Euler at the same fixed dt.
    inline MotionState SemiImplicitEuler(
        MotionState state,
        const Vec3f& linearAcceleration,
        const Vec3f& angularAcceleration,
        float dt)
    {
        RequireFinite(linearAcceleration, "linearAcceleration");
        RequireFinite(angularAcceleration, "angularAcceleration");
        RequirePositive(dt, "dt");

        state.LinearVelocity += linearAcceleration * dt;
        state.AngularVelocity += angularAcceleration * dt;
        state.Position += state.LinearVelocity * dt;
        state.Rotation = IntegrateAngularVelocity(state.Rotation, state.AngularVelocity, dt);
        return state;
    }

    /// Input: current position, previous position, acceleration, and dt.
    /// Output: next Verlet position.
    /// Task: support simple position-based demos without requiring velocity
    /// storage in the integrator call.
    [[nodiscard]]
    // TEACHING DERIVATION: VerletPosition
    // x_next = 2*x - x_previous + a*dt^2; previous position encodes velocity
    // implicitly, so variable dt needs extra treatment.
    inline Vec3f VerletPosition(
        const Vec3f& position,
        const Vec3f& previousPosition,
        const Vec3f& acceleration,
        float dt)
    {
        RequireFinite(position, "position");
        RequireFinite(previousPosition, "previousPosition");
        RequireFinite(acceleration, "acceleration");
        RequirePositive(dt, "dt");

        return position + (position - previousPosition) + acceleration * (dt * dt);
    }

    /// Input: scalar state y, derivative function f(y), and dt.
    /// Output: midpoint RK2 result.
    /// Task: provide a deterministic small ODE helper for springs/constraints.
    template<typename Derivative>
    [[nodiscard]]
    // TEACHING DERIVATION: RK2
    // For scalar autonomous y'=f(y), evaluate derivative at midpoint using k1,
    // then update from k2. The helper does not accept explicit time t.
    float RK2(
        float y,
        float dt,
        Derivative derivative)
    {
        RequireFinite(y, "y");
        RequirePositive(dt, "dt");

        const float k1 =
            derivative(y);

        const float k2 =
            derivative(y + k1 * dt * 0.5f);

        return y + k2 * dt;
    }

    /// Input: scalar state y, derivative function f(y), and dt.
    /// Output: fourth-order Runge-Kutta result.
    /// Task: provide a higher-accuracy reference integrator for tests and tools.
    template<typename Derivative>
    [[nodiscard]]
    // TEACHING DERIVATION: RK4
    // For scalar autonomous y'=f(y), blend four derivative evaluations with
    // 1:2:2:1 weights; accuracy assumes smooth dynamics and fixed dt.
    float RK4(
        float y,
        float dt,
        Derivative derivative)
    {
        RequireFinite(y, "y");
        RequirePositive(dt, "dt");

        const float k1 = derivative(y);
        const float k2 = derivative(y + k1 * dt * 0.5f);
        const float k3 = derivative(y + k2 * dt * 0.5f);
        const float k4 = derivative(y + k3 * dt);

        return y + (dt / 6.0f) * (k1 + 2.0f * k2 + 2.0f * k3 + k4);
    }
}
