# KairoPhysicsMath — source reading order and API notebook

Understand physical units, mass and inertia before using forces, impulses, constraints or integration. These routines supply the mathematical layer of KairoPhysicsEngine; they are not themselves a complete rigid-body world or renderer.

**Before reading:** [README.md](README.md) for setup, [CMakeLists.txt](CMakeLists.txt) if present for module boundaries, [LICENSE](LICENSE) for original-code reuse. Read each exported/public interface, implementation, its imported sibling APIs and matching tests. The sequence below links **all tracked implementation and test source files** in a suggested conceptual order; it is not an execution trace or a claim of complete verification.

## Core API and interaction questions

API contracts: body mass/inertia, world versus body coordinates, time-step units, force versus impulse, contact normals, numerical integrators and constraint Jacobians.

## Ordered source files (12)

1. [`KairoPhysicsMath.cppm`](KairoPhysicsMath.cppm) — identify this file's public imports/includes, state ownership, exported functions, edge cases, and call sites in the next files or tests.
2. [`PhysicsTypes.cppm`](PhysicsTypes.cppm) — identify this file's public imports/includes, state ownership, exported functions, edge cases, and call sites in the next files or tests.
3. [`MassProperties.cppm`](MassProperties.cppm) — identify this file's public imports/includes, state ownership, exported functions, edge cases, and call sites in the next files or tests.
4. [`InertiaTensor.cppm`](InertiaTensor.cppm) — identify this file's public imports/includes, state ownership, exported functions, edge cases, and call sites in the next files or tests.
5. [`RigidBodyMath.cppm`](RigidBodyMath.cppm) — identify this file's public imports/includes, state ownership, exported functions, edge cases, and call sites in the next files or tests.
6. [`Force.cppm`](Force.cppm) — identify this file's public imports/includes, state ownership, exported functions, edge cases, and call sites in the next files or tests.
7. [`Impulse.cppm`](Impulse.cppm) — identify this file's public imports/includes, state ownership, exported functions, edge cases, and call sites in the next files or tests.
8. [`Contact.cppm`](Contact.cppm) — identify this file's public imports/includes, state ownership, exported functions, edge cases, and call sites in the next files or tests.
9. [`ConstraintMath.cppm`](ConstraintMath.cppm) — identify this file's public imports/includes, state ownership, exported functions, edge cases, and call sites in the next files or tests.
10. [`Integrators.cppm`](Integrators.cppm) — identify this file's public imports/includes, state ownership, exported functions, edge cases, and call sites in the next files or tests.
11. [`tests/CertificationTests.cpp`](tests/CertificationTests.cpp) — identify this file's public imports/includes, state ownership, exported functions, edge cases, and call sites in the next files or tests.
12. [`tests/PhysicsMathTests.cpp`](tests/PhysicsMathTests.cpp) — identify this file's public imports/includes, state ownership, exported functions, edge cases, and call sites in the next files or tests.

## Use it as a learning exercise

Choose one API, write its preconditions/postconditions and units, trace the caller in the next layer, and create a minimal deterministic test for the happy case, an invalid-input case and a precision/boundary case. Comment each function's **reason, invariant, output and interaction with other files**, not merely its syntax. Original code is [MIT-licensed](LICENSE) for studying and further development; external dependencies retain their own licenses. Do not infer a finished product or a passed native build from this inventory.
