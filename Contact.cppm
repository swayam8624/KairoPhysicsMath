module;

#include <vector>

export module Kairo.Foundation.PhysicsMath.Contact;

import Kairo.Foundation.Math.Vector;
import Kairo.Foundation.PhysicsMath.Types;

export namespace kairo::foundation::physics
{
    using namespace kairo::foundation::math;

    struct ContactPoint final
    {
        Vec3f Position = Vec3f::Zero();
        Vec3f Normal = Vec3f::Up();
        float PenetrationDepth = 0.0f;
        float NormalImpulse = 0.0f;
        float TangentImpulse = 0.0f;
    };

    struct ContactManifold final
    {
        BodyID BodyA = InvalidBodyID;
        BodyID BodyB = InvalidBodyID;
        ColliderID ColliderA = InvalidColliderID;
        ColliderID ColliderB = InvalidColliderID;
        bool IsTrigger = false;
        std::vector<ContactPoint> Points;
    };

    /// Input: position, normal, penetration, and optional warm-start impulses.
    /// Output: validated contact point.
    /// Task: provide a narrowphase-to-solver data record without depending on a
    /// concrete collision detector.
    [[nodiscard]]
    inline ContactPoint MakeContactPoint(
        const Vec3f& position,
        const Vec3f& normal,
        float penetrationDepth,
        float normalImpulse = 0.0f,
        float tangentImpulse = 0.0f)
    {
        RequireFinite(position, "position");
        RequireFinite(normal, "normal");
        RequireNonNegative(penetrationDepth, "penetrationDepth");
        RequireNonNegative(normalImpulse, "normalImpulse");
        RequireFinite(tangentImpulse, "tangentImpulse");

        return
        {
            position,
            SafeNormalize(normal, Vec3f::Up()),
            penetrationDepth,
            normalImpulse,
            tangentImpulse
        };
    }

    /// Input: two body ids and optional collider ids.
    /// Output: empty contact manifold for that exact contact pair.
    /// Task: make body and collider ownership explicit without creating a
    /// physics world. Collider ids matter because one body can own multiple
    /// colliders with different materials or filtering.
    [[nodiscard]]
    inline ContactManifold MakeContactManifold(
        BodyID bodyA,
        BodyID bodyB,
        ColliderID colliderA = InvalidColliderID,
        ColliderID colliderB = InvalidColliderID,
        bool isTrigger = false)
    {
        return { bodyA, bodyB, colliderA, colliderB, isTrigger, {} };
    }
}
