# KizuriPhysics — Architecture

This document explains how a step is executed and why each subsystem is built
the way it is.

## 1. Coordinate and storage conventions

* Right-handed, **Y-up**, metres and seconds.
* `Transform = { Vec3 position; Quat rotation; }` — rotation is applied about
  the body origin; the centre of mass is a separate offset.
* `Vec3` is 16-byte aligned so it can be loaded into SSE/NEON registers
  directly; the solver also provides a scalar path that the compiler
  auto-vectorises.
* Bodies are addressed by `BodyID` (24-bit index + 8-bit sequence). The
  sequence guards against the ABA problem when a slot is recycled.

## 2. Broad phase — dynamic AABB tree

`DynamicAABBTree` is a binary BVH of fat AABBs (tight bounds + 0.1 m margin +
per-step displacement). Each leaf corresponds to one body proxy.

* **Insertion** picks the sibling that minimises the surface-area cost, then
  walks up refitting bounds and rotating to keep the tree balanced.
* **MoveProxy** is cheap: if the new tight AABB is still inside the fat AABB the
  leaf is *not* re-inserted. Only escapes trigger a remove/insert.
* **Pair generation** (`QueryPairs`) is an overlap-pruned self-intersection
  traversal. Node pairs whose fat bounds do not overlap are discarded, and the
  larger node is split to keep the stack balanced. This makes pair generation
  `O(n log n + k)` in the number of overlapping pairs `k`.
* **Layer filtering** rejects a pair when
  `(layerA & maskB) == 0 || (layerB & maskA) == 0`.

The tree stores a growable stack in `QueryPairs` (never a fixed buffer) so
large scenes cannot silently drop pairs.

## 3. Narrow phase — GJK, EPA, manifolds

### Convex vs convex

1. **GJK** (`Collision/GJK.cpp`) computes the closest points between two convex
   shapes using the Minkowski difference and a simplex that is reduced to its
   supporting feature each iteration. It returns either the separation distance
   and witness points, or a report that the shapes intersect.
2. **EPA** (`Collision/EPA.cpp`) expands an initial polytope that strictly
   contains the origin to find the minimum penetration depth and normal. The
   initial polytope is seeded from the GJK simplex and completed with support
   samples; a brute-force search over axis + diagonal samples guarantees a
   strictly-containing tetrahedron even for symmetric/axis-aligned overlaps
   (the case where the origin lies exactly on the GJK simplex).
3. **Manifold generation** then produces the contact points:
   * *Polyhedral pairs* (Box, ConvexHull): the reference face of one shape is
     clipped against the side planes of the incident face of the other
     (Sutherland–Hodgman), yielding up to 4 stable points.
   * *Smooth convex pairs* (sphere, capsule, cylinder): the EPA witness points
     give a single contact.

### Special cases and non-convex shapes

* **Sphere** and **Plane** have analytic fast paths.
* **TriangleMesh** and **HeightField** iterate candidate triangles and run a
  convex-vs-triangle clip (or a closest-point test for spheres/capsules).
* **Compound** shapes recurse into their children and merge manifolds.

## 4. Solver — sequential impulse with split impulse

`ContactSolver` builds one `ContactConstraint` per body pair, each holding up to
`kMaxManifoldPoints` point constraints.

Per point it precomputes:

* the world contact basis `{n, t1, t2}`,
* the effective mass along each axis (`EffectiveMass` accounts for both linear
  and angular terms),
* the restitution target from the approach velocity,
* the position bias from penetration.

The velocity solve then runs `velocityIterations` sweeps of
**friction → normal** impulses, clamping the accumulated normal impulse to
`>= 0` and the friction impulse to `±μ·Pₙ` (Coulomb cone). Accumulated impulses
are warm-started from the previous frame.

**Position correction** uses split impulse: a second set of *pseudo-velocities*
is solved against the position bias and integrated into the positions after the
real motion. Because the pseudo-velocities never feed back into the real
velocities, correcting penetration does not add energy — the reason a stack of
boxes settles instead of buzzing.

If `useVelocityBias` is enabled the solver instead folds the position bias into
the velocity solve (classic Baumgarte). Split impulse is the default.

## 5. Sleeping and islands

At the end of a step, contacts and joints are used to union bodies into
**islands**. A body accumulates a sleep timer while its linear and angular
speeds stay below the thresholds; an island is put to sleep only when *every*
dynamic body in it has exceeded `sleepTimeThreshold`. Sleeping bodies are
removed from integration and from broad-phase updates, and are woken by new
contacts or by an explicit impulse.

## 6. Determinism

The engine uses no wall-clock time and no thread-order-dependent accumulation
inside the solver (the narrow phase writes to per-pair slots, so its result is
order-independent). `PhysicsWorld::ComputeStateHash()` folds every body's
position, rotation and velocity into a 64-bit hash for lockstep verification.

## 7. Threading

`JobSystem` provides an MPMC queue, `ParallelFor` and job dependencies. The
narrow phase fans out over broad-phase pairs when the pair count exceeds a
threshold; the solve is currently single-threaded (parallel islands are on the
roadmap).
