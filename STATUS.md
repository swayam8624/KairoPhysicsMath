# KairoPhysicsMath Status

Wave: A — foundation certification  
Frozen v1 target: 95/100  
Source gate: complete  
Execution gate: `cmake --build <build> && ctest --test-dir <build> --output-on-failure`

## Frozen v1 scope

PhysicsMath v1 owns units/conventions, primitive mass and inertia math, force accumulation, impulses, rigid-body diagnostics, deterministic integration helpers, contact records and reusable constraint scalar/Jacobian math. World ownership, collision detection and solver iteration stay in KairoPhysicsEngine.

## 95 exit evidence

- Complete intended math modules are exported through one umbrella module.
- Existing tests cover mass/inertia, forces, impulses, integrators, contacts, constraints and invalid physical inputs.
- Certification adds scale-range mass invariants, 20,000-step quaternion normalization stress, and center-of-mass impulse invariants.
- No external physics/math package is required.

## Verification policy

95 is the frozen-scope score; exact-SHA CTest evidence is still required before a release is labeled verified.
