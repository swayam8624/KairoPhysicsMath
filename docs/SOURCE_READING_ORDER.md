# Teaching source map — KairoPhysicsMath

Dimensioned dynamics and constraint math below the game-facing PhysicsEngine.

Follow source links sequentially. For each API/function, identify its input contract, owning data structure, called modules, failure modes, outputs, test and downstream consumers.

## Read in this order

The route below progresses from physical quantities to a simulation step.
It is a learning order, not an assertion about CMake's build order.

### Stage 1 — Units, state and geometry-derived inertia

1. [`CMakeLists.txt`](../CMakeLists.txt): compiler/modules, external KairoMath dependency, test targets.
2. [`KairoPhysicsMath.cppm`](../KairoPhysicsMath.cppm): C++23 public umbrella and exported module boundaries.
3. [`PhysicsTypes.cppm`](../PhysicsTypes.cppm): body identity, motion state, validation, units and finite-value contracts.
4. [`MassProperties.cppm`](../MassProperties.cppm): mass, center of mass, inverse mass and shape-specific properties.
5. [`InertiaTensor.cppm`](../InertiaTensor.cppm): inertia about COM, local/world frames and parallel-axis theorem.

**Check:** distinguish an object's mass from its resistance to rotation.
A zero inverse mass identifies an immovable body for solver calculations;
never substitute a zero forward mass into an ordinary division.

### Stage 2 — Applying forces and impulses

6. [`Force.cppm`](../Force.cppm): force/torque accumulation, gravity, springs and drag.
7. [`RigidBodyMath.cppm`](../RigidBodyMath.cppm): velocity at a point, linear/angular momentum and energy.
8. [`Impulse.cppm`](../Impulse.cppm): normal and friction impulse coefficients.
9. [`Contact.cppm`](../Contact.cppm): per-body/collider contact records and trigger distinctions.
10. [`ConstraintMath.cppm`](../ConstraintMath.cppm): Jacobian rows, effective mass, corrective bias and impulse clamping.

**Check:** `F = m*a` is a time-accumulated force law, whereas an impulse
changes momentum immediately. At offset `r` from COM, torque from a force
is `r × F`; a point impulse similarly affects angular velocity. A trigger
contact may be reported without physical blocking.

### Stage 3 — Integration and repeatable verification

11. [`Integrators.cppm`](../Integrators.cppm): explicit/semi-implicit Euler, Verlet and RK helpers.
12. [`tests/PhysicsMathTests.cpp`](../tests/PhysicsMathTests.cpp): behavior assertions and numerical edge cases.
13. [`tests/CertificationTests.cpp`](../tests/CertificationTests.cpp): certification scenarios.

**Check:** Euler uses the pre-step velocity for position; semi-implicit
Euler updates velocity first. Position Verlet encodes motion in the previous
position and assumes a compatible timestep convention. Higher-order ODE
helpers do not themselves solve contacts, friction or constraints.

### Stage 4 — Understand the engine boundary

Next use the [KairoPhysicsEngine guide](https://github.com/swayam8624/KairoPhysicsEngine/blob/docs/teaching-walkthrough-20261121/docs/SOURCE_READING_ORDER.md).
There, `PhysicsWorld` controls fixed-step scheduling and owns body/contact
state; `Broadphase` discovers possible overlap pairs; `Narrowphase`
generates actual contact geometry and `ContactSolver` applies the math from
this repository. Do not confuse a helper function with a complete solver.

## How this repository interacts with others

Math vectors/matrices feed physical state; PhysicsEngine combines these operations into broadphase/narrowphase/world stepping. Numerical integrators are not the same as collision handling.

## Verification and study rules

- Read the public C++23 module/entrypoint first, then its implementation and tests.
- Explain invariants, ownership, error states, concurrency or numerical assumptions in comments, not trivial syntax.
- A passing CI job covers only tests actually run on that commit/host; source presence is not runtime certification.
- Check [STATUS.md](../STATUS.md) and [STATUS.yaml](../STATUS.yaml) for current explicit verification limitations.
- Original work may be modified and extended under the repository license; third-party content retains upstream terms.


## Complete source-file inventory

After the curated teaching path, follow this deterministic inventory of first-party source, scripts, shaders, and tests. Grouping is a suggested learning order, not an indication that every function is already annotated. Vendored dependencies, ROMs, binaries, data, media, and generated evidence are excluded.

### Entry points and build (1)

1. [`CMakeLists.txt`](../CMakeLists.txt)

### Interfaces and modules (10)

1. [`ConstraintMath.cppm`](../ConstraintMath.cppm)
2. [`Contact.cppm`](../Contact.cppm)
3. [`Force.cppm`](../Force.cppm)
4. [`Impulse.cppm`](../Impulse.cppm)
5. [`InertiaTensor.cppm`](../InertiaTensor.cppm)
6. [`Integrators.cppm`](../Integrators.cppm)
7. [`KairoPhysicsMath.cppm`](../KairoPhysicsMath.cppm)
8. [`MassProperties.cppm`](../MassProperties.cppm)
9. [`PhysicsTypes.cppm`](../PhysicsTypes.cppm)
10. [`RigidBodyMath.cppm`](../RigidBodyMath.cppm)

### Verification and tests (2)

1. [`tests/CertificationTests.cpp`](../tests/CertificationTests.cpp)
2. [`tests/PhysicsMathTests.cpp`](../tests/PhysicsMathTests.cpp)

Snapshot totals: 13 first-party source/build/test files, 20 tracked files overall. Re-audit when files are renamed or added.
