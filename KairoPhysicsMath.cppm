// TEACHING MODULE: This C++23 umbrella is deliberately declarative: `export import`
// re-exports the nine narrower physics math modules. Callers can use the full
// `import Kairo.Foundation.PhysicsMath;` surface or only a needed component.
// KairoPhysicsEngine supplies world ownership, contact detection, stepping and
// solver iteration; no such behavior is hidden inside this umbrella.

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
