# Teaching source map — KairoPhysicsMath

Dimensioned dynamics and constraint math below the game-facing PhysicsEngine.

Follow source links sequentially. For each API/function, identify its input contract, owning data structure, called modules, failure modes, outputs, test and downstream consumers.

## Read in this order

1. [`CMakeLists.txt`](../CMakeLists.txt)
2. [`KairoPhysicsMath.cppm`](../KairoPhysicsMath.cppm)
3. [`PhysicsTypes.cppm`](../PhysicsTypes.cppm)
4. [`MassProperties.cppm`](../MassProperties.cppm)
5. [`InertiaTensor.cppm`](../InertiaTensor.cppm)
6. [`Force.cppm`](../Force.cppm)
7. [`Impulse.cppm`](../Impulse.cppm)
8. [`RigidBodyMath.cppm`](../RigidBodyMath.cppm)
9. [`Integrators.cppm`](../Integrators.cppm)
10. [`Contact.cppm`](../Contact.cppm)
11. [`ConstraintMath.cppm`](../ConstraintMath.cppm)
12. [`tests`](../tests)

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
