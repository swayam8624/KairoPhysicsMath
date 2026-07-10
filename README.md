# KairoPhysicsMath

`KairoPhysicsMath` is the physics-math foundation layer for the Kairo engine
roadmap. It exists before `KairoPhysics` so rigid-body formulas, units,
integration helpers, impulses, contacts, and constraint math can be tested in
isolation.

```text
KairoMath -> KairoPhysicsMath -> KairoPhysics later
```

This repo is not a physics world. It does not perform collision detection,
broadphase, narrowphase, world stepping, rendering, sandbox visualization, or
editor behavior.

## Build

```bash
cd /Users/swayamsingal/Desktop/Programming/Kairo/Foundation/KairoPhysicsMath
cmake -S . -B build -G Ninja -DCMAKE_CXX_COMPILER=/opt/homebrew/opt/llvm/bin/clang++
cmake --build build
ctest --test-dir build --output-on-failure
```

Dependency resolution prefers local Kairo repos first:

```text
KAIRO_MATH_SOURCE_DIR
../KairoMath
https://github.com/swayam8624/KairoMath.git
```

No outside math library is used.

## Conventions

```text
Units: SI-style meters, kilograms, seconds
Angles: radians
Coordinates: right-handed, matching KairoMath/KairoGeometry
Capsules: local Y axis, centered at local origin
Boxes: half-extents
Inertia tensors: local tensors are about local center of mass
World inverse inertia: R * I_local^-1 * R^T
Static bodies: zero mass inverse, zero inverse inertia
```

Public APIs use KairoMath types:

```cpp
Vec3f
Quatf
Mat3f
```

The module also exports compatibility aliases:

```cpp
using Matrix3f = Mat3f;
using Quaternionf = Quatf;
```

## Modules

```text
PhysicsTypes      IDs, body type, motion state, validation helpers
MassProperties    mass, inverse mass, center of mass, shape mass properties
InertiaTensor     primitive inertia, inverse world inertia, parallel-axis theorem
Force             force accumulation, gravity, drag, spring forces
RigidBodyMath     velocity at point, impulses, momentum, kinetic energy
Impulse           normal/friction impulse scalar helpers
Integrators       explicit Euler, semi-implicit Euler, Verlet, RK2, RK4
Contact           contact point and manifold data records with body/collider ids and trigger state
ConstraintMath    Jacobian rows, effective mass, Baumgarte, warm-start clamps
```

Use the umbrella module for normal consumers:

```cpp
import Kairo.Foundation.PhysicsMath;
```

## Examples

Mass properties:

```cpp
MassProperties ball =
    SphereMassProperties(0.5f, 1200.0f);

Matrix3f invWorld =
    InverseInertiaWorld(
        ball.LocalInverseInertiaTensor,
        Quaternionf::Identity());
```

Force accumulation:

```cpp
ForceAccumulation forces;
AddForce(forces, GravityForce(ball.Mass));
AddForceAtPoint(
    forces,
    Vec3f{ 10.0f, 0.0f, 0.0f },
    Vec3f{ 0.0f, 1.0f, 0.0f },
    Vec3f::Zero());
```

Impulse response:

```cpp
MotionState state;
ApplyImpulseAtPoint(
    state,
    ball.InverseMass,
    invWorld,
    Vec3f{ 0.0f, 4.0f, 0.0f },
    Vec3f{ 0.5f, 0.0f, 0.0f });
```

Contact records:

```cpp
ContactManifold manifold =
    MakeContactManifold(
        bodyA,
        bodyB,
        colliderA,
        colliderB,
        true); // trigger/sensor contact
```

`ContactManifold` deliberately stores both body ids and collider ids. A body can
own multiple colliders with different materials, filters, and trigger behavior,
so solver and event systems must not guess the collider from the body alone.

Semi-implicit integration:

```cpp
MotionState next =
    SemiImplicitEuler(
        state,
        forces.Force * ball.InverseMass,
        invWorld * forces.Torque,
        1.0f / 60.0f);
```

## Validation

Functions throw `std::invalid_argument` for invalid physical inputs such as:

```text
negative mass or density
non-positive radius or box half-extents
non-positive dt
NaN or infinite force, velocity, position, impulse, or scalar settings
negative penetration depth
negative cached normal impulse
```

## Not In This Repo

The following belong in `KairoPhysics` or later sandbox/viewer repos:

```text
RigidBody ownership
Collider ownership
PhysicsWorld::Step
Broadphase and narrowphase
Collision detection
Sequential impulse solver loop
Dynamic AABB tree integration
Terminal or GLFW visualization
Ray-traced physics frame export
ImGui/editor UI
```
