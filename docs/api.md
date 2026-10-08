# KizuriPhysics — API reference

Everything lives in the `kizuri` namespace and is reachable through
`#include "Kizuri/Kizuri.h"`.

## World

```cpp
class PhysicsWorld {
public:
    WorldSettings&       GetSettings();
    const WorldSettings& GetSettings() const;

    void  SetGravity(const Vec3& g);
    Vec3  GetGravity() const;

    // Bodies
    BodyID CreateBody(const BodySettings& settings);
    void   DestroyBody(BodyID id);
    Body*  GetBody(BodyID id);
    const Body* GetBody(BodyID id) const;

    // Constraints
    void AddConstraint(std::unique_ptr<Constraint> c);
    void RemoveAllConstraints();
    u32  GetNumConstraints() const;

    // Simulation
    void Step(Real dt);        // honours useFixedTimestep / maxSubSteps
    void StepFixed(Real dt);   // exactly one step

    // Queries
    RayCastResult   RayCast(const Ray&, Real maxDist = BIG, u32 mask = ~0u) const;
    u32             RayCastAll(const Ray&, Real maxDist, RayCastResult* out, u32 max,
                               u32 mask = ~0u) const;
    ShapeCastResult CastShape(const Shape&, const Transform& start, const Vec3& dir,
                              Real maxDist, u32 mask = ~0u) const;
    u32             QueryAABB(const AABB&, BodyID* out, u32 max, u32 mask = ~0u) const;
    BodyID          QueryPoint(const Vec3&, u32 mask = ~0u) const;

    // Introspection
    const WorldStats&   GetStats() const;
    const BroadPhase&   GetBroadPhase() const;
    const BodyManager&  GetBodyManager() const;
    u64                 ComputeStateHash() const;   // determinism / lockstep

    void DrawDebug(DebugRenderer&, bool shapes = true,
                   bool contacts = false, bool aabbs = false) const;
};
```

### WorldSettings

| Field | Default | Meaning |
|---|---|---|
| `gravity` | `(0,-9.81,0)` | Global acceleration. |
| `solver` | see below | Solver tuning. |
| `allowSleeping` | `true` | Enable island sleeping. |
| `sleepTimeThreshold` | `0.5` s | Time below threshold before sleeping. |
| `sleepLinearThreshold` | `0.05` m/s | Linear sleep speed. |
| `sleepAngularThreshold` | `0.1` rad/s | Angular sleep speed. |
| `useFixedTimestep` | `true` | Sub-divide `Step(dt)` into fixed steps. |
| `fixedTimestep` | `1/60` s | Fixed step length. |
| `maxSubSteps` | `4` | Spiral-of-death guard. |
| `useMultithreading` | `true` | Parallel narrow phase. |

### SolverSettings

| Field | Default | Meaning |
|---|---|---|
| `velocityIterations` | `16` | Impulse iterations. |
| `positionIterations` | `6` | Split-impulse iterations. |
| `baumgarte` | `0.2` | Position error correction factor. |
| `penetrationSlop` | `0.005` m | Allowed penetration. |
| `restitutionThreshold` | `1.0` m/s | Min approach speed for bounce. |
| `maxBiasVelocity` | `3.0` | Cap on correction velocity. |
| `warmStartFactor` | `1.0` | Warm-start scaling. |
| `useVelocityBias` | `false` | `false` = split impulse (recommended); `true` = Baumgarte in the velocity solve. |

## Bodies

```cpp
BodyID id = world.CreateBody(BodySettings{
    .shape        = MakeRef<BoxShape>(Vec3(0.5f)),
    .position     = Vec3(0, 5, 0),
    .motionType   = MotionType::Dynamic,   // Static | Kinematic | Dynamic
    .friction     = 0.6f,
    .restitution  = 0.0f,
    .density      = 1000.0f,
});
```

`Body` exposes `GetPosition/SetPosition`, `GetRotation/SetRotation`,
`GetTransform/SetTransform`, `GetLinearVelocity/SetLinearVelocity`,
`GetAngularVelocity/SetAngularVelocity`, `GetMass`, `GetShape`, friction /
restitution accessors, `IsDynamic/IsStatic/IsKinematic`, `IsActive`, and
`ApplyForce` / `ApplyImpulse` style helpers.

`BodyID` packs a 24-bit index and an 8-bit sequence; `Pack()` / `Unpack()` are
used internally by the broad phase.

## Shapes

All shapes derive from `Shape` (ref-counted, immutable). Create them with
`MakeRef<T>(...)`:

| Shape | Construction |
|---|---|
| `SphereShape` | `MakeRef<SphereShape>(radius)` |
| `BoxShape` | `MakeRef<BoxShape>(halfExtent)` |
| `CapsuleShape` | `MakeRef<CapsuleShape>(halfHeight, radius)` |
| `CylinderShape` | `MakeRef<CylinderShape>(halfHeight, radius)` |
| `ConvexHullShape` | `ConvexHullShape::Create(points, count)` |
| `MeshShape` | `MeshShape::Create(vertices, count, indices, count)` |
| `CompoundShape` | `MakeRef<CompoundShape>()` then `AddChild(transform, shape)` |
| `HeightFieldShape` | `HeightFieldShape::Create(heights, nx, nz, scale)` |
| `PlaneShape` | `MakeRef<PlaneShape>(Plane(normal, distance))` |

Common `Shape` API: `GetType()`, `GetName()`, `GetLocalBounds()`,
`GetWorldBounds(transform)`, `GetVolume()`, `GetMassProperties(density)`,
`GetSupport(dir)`, `IsConvex()`, `RayCastLocal(ray, maxFraction, outNormal)`,
`ContainsPoint(point)`.

## Constraints (joints)

```cpp
world.AddConstraint(std::make_unique<PointConstraint>(bodyA, bodyB, worldAnchor));
world.AddConstraint(std::make_unique<FixedConstraint>(bodyA, bodyB, worldAnchor));
world.AddConstraint(std::make_unique<DistanceConstraint>(bodyA, bodyB, anchorA, anchorB, distance));
world.AddConstraint(std::make_unique<HingeConstraint>(bodyA, bodyB, worldAnchor, axisA, axisB));
world.AddConstraint(std::make_unique<SliderConstraint>(bodyA, bodyB, worldAnchor, worldAxis));
world.AddConstraint(std::make_unique<SixDOFConstraint>(bodyA, bodyB, worldAnchor, worldRotation));
```

`HingeConstraint` and `SliderConstraint` support `SetLimits(lo, hi)` and
`SetMotor(targetVelocity, maxTorqueOrForce)`. `SixDOFConstraint` exposes
`SetLinearLimits(axis, lo, hi)` / `SetAngularLimits(axis, lo, hi)` with
`axis ∈ {0,1,2}`.

## Queries

```cpp
struct RayCastResult {
    BodyID body;
    Real   fraction = 1;
    Vec3   position, normal;
    bool   hit = false;
};

struct ShapeCastResult {
    BodyID body;
    Real   fraction = 1;
    Vec3   normal, point;
    bool   hit = false;
};
```

```cpp
Ray ray{ Vec3(0, 10, 0), Vec3(0, -1, 0) };     // origin, direction
RayCastResult hit = world.RayCast(ray, 100.0f);
```

## Debug rendering

Implement `DebugRenderer` and pass it to `world.DrawDebug(...)`:

```cpp
class DebugRenderer {
    virtual void DrawLine(const Vec3& from, const Vec3& to, Color color) = 0;
    virtual void DrawTriangle(const Vec3& a, const Vec3& b, const Vec3& c, Color color) = 0;
    virtual void DrawSphere(const Vec3& center, Real radius, Color color) = 0;
    virtual void DrawBox(const Vec3& center, const Vec3& halfExtent,
                         const Quat& rotation, Color color) = 0;
};
```

## Error handling

Invalid inputs are guarded by `KZ_ASSERT`. `BodyID::IsInvalid()` reports a
failed body creation. Shapes are immutable and ref-counted, so a shape must not
be mutated after it is attached to a body.
