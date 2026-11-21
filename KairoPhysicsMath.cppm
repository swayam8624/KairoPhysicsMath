// TEACHING NOTE:
// Umbrella module reexports mass/inertia, forces, impulses, contacts and integration; the simulation
// world lives in KairoPhysicsEngine.
// See docs/SOURCE_READING_ORDER.md for related modules and tests.

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
