// TEACHING ENTRY POINT: `import Kairo.Foundation.PhysicsMath;` exposes
// stateless physical formulas, typed records and validation helpers. It is not
// a world simulation and does not own bodies, broadphase proxies or events.
//
// Follow the data flow: Types defines MotionState and units; MassProperties
// and InertiaTensor define mass and rotational resistance; Force and
// RigidBodyMath update forces/momentum; Impulse/Contact/ConstraintMath provide
// contact- and joint-solver ingredients; Integrators advance state over dt.
//
// Math conventions are inherited from KairoMath (meters, kg, seconds, radians,
// column-vector transforms). World-space inverse inertia is rotated from
// local COM space; static bodies use zero inverse mass/inertia. These details
// affect KairoPhysicsEngine's ContactSolver and PhysicsWorld.
//
// C++23 `export import` re-exports each narrower module for consumers.
// A narrow `import Kairo.Foundation.PhysicsMath.Integrators;` is preferable
// when only a small API surface is needed. See docs/SOURCE_READING_ORDER.md.

export module Kairo.Foundation.PhysicsMath;

export import Kairo.Foundation.PhysicsMath.Types;
export import Kairo.Foundation.PhysicsMath.InertiaTensor;
export import Kairo.Foundation.PhysicsMath.MassProperties;
export import Kairo.Foundation.PhysicsMath.Force;
export import Kairo.Foundation.PhysicsMath.RigidBodyMath;
export import Kairo.Foundation.PhysicsMath.Impulse;
export import Kairo.Foundation.PhysicsMath.Integrators;
export import Kairo.Foundation.PhysicsMath.Contact;
export import Kairo.Foundation.PhysicsMath.ConstraintMath;
