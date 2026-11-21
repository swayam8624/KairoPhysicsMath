// TEACHING MAP — KairoPhysicsMath.cppm
// Umbrella C++23 physics mathematics API for motion, mass, impulses, contacts and
// constraints. KairoPhysicsEngine imports it but owns the actual simulation world, stepping,
// callbacks and solver state.
// Read READING_ORDER.md and KairoPhysicsEngine for the caller flow.

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
