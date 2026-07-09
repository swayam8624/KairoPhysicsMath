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
