# KizuriPhysics — Referência da API

Tudo vive no namespace `kizuri` e é acessível via
`#include "Kizuri/Kizuri.h"`.

## Mundo

```cpp
class PhysicsWorld {
public:
    WorldSettings&       GetSettings();
    const WorldSettings& GetSettings() const;

    void  SetGravity(const Vec3& g);
    Vec3  GetGravity() const;

    // Corpos
    BodyID CreateBody(const BodySettings& settings);
    void   DestroyBody(BodyID id);
    void   Clear();                                  // remove todo corpo + cache
    void   SetBodyActive(BodyID id, bool active);    // acorda / adormece um corpo
    Body*  GetBody(BodyID id);
    const Body* GetBody(BodyID id) const;

    // Restrições
    void AddConstraint(std::unique_ptr<Constraint> c);
    void RemoveAllConstraints();
    u32  GetNumConstraints() const;

    // Simulação
    void Step(Real dt);        // respeita useFixedTimestep / maxSubSteps
    void StepFixed(Real dt);   // exatamente um passo

    // Consultas
    RayCastResult   RayCast(const Ray&, Real maxDist = BIG, u32 mask = ~0u) const;
    u32             RayCastAll(const Ray&, Real maxDist, RayCastResult* out, u32 max,
                               u32 mask = ~0u) const;
    ShapeCastResult CastShape(const Shape&, const Transform& start, const Vec3& dir,
                              Real maxDist, u32 mask = ~0u) const;
    u32             CastShapeAll(const Shape&, const Transform& start, const Vec3& dir,
                                 Real maxDist, ShapeCastResult* out, u32 max,
                                 u32 mask = ~0u) const;
    u32             QueryAABB(const AABB&, BodyID* out, u32 max, u32 mask = ~0u) const;
    BodyID          QueryPoint(const Vec3&, u32 mask = ~0u) const;

    // Introspecção
    const WorldStats&   GetStats() const;
    const BroadPhase&   GetBroadPhase() const;
    const BodyManager&  GetBodyManager() const;
    u64                 ComputeStateHash() const;   // determinismo / lockstep

    void DrawDebug(DebugRenderer&, bool shapes = true,
                   bool contacts = false, bool aabbs = false) const;
};
```

### WorldSettings

| Campo | Padrão | Significado |
|---|---|---|
| `gravity` | `(0,-9.81,0)` | Aceleração global. |
| `solver` | veja abaixo | Ajustes do solver. |
| `allowSleeping` | `true` | Habilita o sono por ilhas. |
| `sleepTimeThreshold` | `0.5` s | Tempo abaixo do limite antes de dormir. |
| `sleepLinearThreshold` | `0.05` m/s | Velocidade linear de sono. |
| `sleepAngularThreshold` | `0.1` rad/s | Velocidade angular de sono. |
| `useFixedTimestep` | `true` | Subdivide `Step(dt)` em passos fixos. |
| `fixedTimestep` | `1/60` s | Duração do passo fixo. |
| `maxSubSteps` | `4` | Proteção contra espiral da morte. |
| `useMultithreading` | `true` | Fase estreita paralela. |

### SolverSettings

| Campo | Padrão | Significado |
|---|---|---|
| `velocityIterations` | `16` | Iterações de impulso. |
| `positionIterations` | `6` | Iterações de split impulse. |
| `baumgarte` | `0.2` | Fator de correção do erro de posição. |
| `penetrationSlop` | `0.005` m | Penetração permitida. |
| `restitutionThreshold` | `1.0` m/s | Velocidade mínima de aproximação para quicar. |
| `maxBiasVelocity` | `3.0` | Teto da velocidade de correção. |
| `warmStartFactor` | `1.0` | Escala do warm start. |
| `useVelocityBias` | `false` | `false` = split impulse (recomendado); `true` = Baumgarte no solve de velocidade. |
| `useSIMDSolver` | `true` | Usa os lotes de contato SoA 4-wide. |

### WorldSettings — CCD

| Campo | Padrão | Significado |
|---|---|---|
| `useCCD` | `true` | Habilita a colisão contínua por linear cast. |
| `ccdMaxPenetration` | `0.01` m | Para a esta distância antes da superfície. |

O CCD por corpo é selecionado com `BodySettings::motionQuality`
(`MotionQuality::Discrete` ou `MotionQuality::LinearCast`) e ajustado com
`BodySettings::ccdMotionThreshold` (pula o cast abaixo deste movimento por
passo).

## Corpos

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

`Body` expõe `GetPosition/SetPosition`, `GetRotation/SetRotation`,
`GetTransform/SetTransform`, `GetLinearVelocity/SetLinearVelocity`,
`GetAngularVelocity/SetAngularVelocity`, `GetMass`, `GetShape`, acessores de
atrito / restituição, `IsDynamic/IsStatic/IsKinematic`, `IsActive`, e helpers no
estilo `ApplyForce` / `ApplyImpulse`.

`BodyID` empacota um índice de 24 bits e uma sequência de 8 bits; `Pack()` /
`Unpack()` são usados internamente pela fase ampla.

## Formas

Todas as formas derivam de `Shape` (com contagem de referência, imutável).
Crie-as com `MakeRef<T>(...)`:

| Forma | Construção |
|---|---|
| `SphereShape` | `MakeRef<SphereShape>(radius)` |
| `BoxShape` | `MakeRef<BoxShape>(halfExtent)` |
| `CapsuleShape` | `MakeRef<CapsuleShape>(halfHeight, radius)` |
| `CylinderShape` | `MakeRef<CylinderShape>(halfHeight, radius)` |
| `ConvexHullShape` | `ConvexHullShape::Create(points, count)` |
| `MeshShape` | `MeshShape::Create(vertices, count, indices, count)` |
| `CompoundShape` | `MakeRef<CompoundShape>()` e depois `AddChild(transform, shape)` |
| `HeightFieldShape` | `HeightFieldShape::Create(heights, nx, nz, scale)` |
| `PlaneShape` | `MakeRef<PlaneShape>(Plane(normal, distance))` |

API comum de `Shape`: `GetType()`, `GetName()`, `GetLocalBounds()`,
`GetWorldBounds(transform)`, `GetVolume()`, `GetMassProperties(density)`,
`GetSupport(dir)`, `IsConvex()`, `RayCastLocal(ray, maxFraction, outNormal)`,
`ContainsPoint(point)`.

## Restrições (juntas)

```cpp
world.AddConstraint(std::make_unique<PointConstraint>(bodyA, bodyB, worldAnchor));
world.AddConstraint(std::make_unique<FixedConstraint>(bodyA, bodyB, worldAnchor));
world.AddConstraint(std::make_unique<DistanceConstraint>(bodyA, bodyB, anchorA, anchorB, distance));
world.AddConstraint(std::make_unique<HingeConstraint>(bodyA, bodyB, worldAnchor, axisA, axisB));
world.AddConstraint(std::make_unique<SliderConstraint>(bodyA, bodyB, worldAnchor, worldAxis));
world.AddConstraint(std::make_unique<SixDOFConstraint>(bodyA, bodyB, worldAnchor, worldRotation));
```

`HingeConstraint` e `SliderConstraint` suportam `SetLimits(lo, hi)` e
`SetMotor(targetVelocity, maxTorqueOrForce)`. `SixDOFConstraint` expõe
`SetLinearLimits(axis, lo, hi)` / `SetAngularLimits(axis, lo, hi)` com
`axis ∈ {0,1,2}`.

## Consultas

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
Ray ray{ Vec3(0, 10, 0), Vec3(0, -1, 0) };     // origem, direção
RayCastResult hit = world.RayCast(ray, 100.0f);
```

## Renderização de depuração

Implemente `DebugRenderer` e passe-o a `world.DrawDebug(...)`:

```cpp
class DebugRenderer {
    virtual void DrawLine(const Vec3& from, const Vec3& to, Color color) = 0;
    virtual void DrawTriangle(const Vec3& a, const Vec3& b, const Vec3& c, Color color) = 0;
    virtual void DrawSphere(const Vec3& center, Real radius, Color color) = 0;
    virtual void DrawBox(const Vec3& center, const Vec3& halfExtent,
                         const Quat& rotation, Color color) = 0;
};
```

## Controlador de personagem

```cpp
#include "Kizuri/Kizuri.h"

CharacterSettings cs;
cs.radius       = 0.3f;
cs.height       = 1.8f;      // altura total da cápsula (>= 2 * radius)
cs.maxSlopeAngle = 50.0f;    // graus
cs.stepHeight   = 0.4f;
cs.skin         = 0.02f;
cs.gravity      = Vec3(0, -9.81f, 0);

CharacterController character(world, cs);
character.SetPosition(Vec3(0, 2, 0));
character.SetVelocity(Vec3(0, 0, 4));   // intenção horizontal, m/s
character.Update(1.0f / 60.0f);
character.Jump(5.0f);                   // apenas quando no chão

bool grounded = character.IsGrounded();
Vec3 pos      = character.GetPosition();
Vec3 vel      = character.GetVelocity();
```

`Update(dt)` executa collide-and-slide contra o mundo, resolve a subida de
degrau e adere ao chão quando está apoiado. O controlador é cinemático e nunca
empurra os corpos que toca.

## Veículo

```cpp
VehicleSettings vs;
vs.forward     = Vec3(0, 0, 1);
vs.maxSteerAngle = 0.5f;     // rad
vs.engineForce = 4000.0f;
vs.brakeForce  = 2000.0f;

Vehicle vehicle(world, chassisBodyID, vs);

WheelSettings wheel;
wheel.localPosition          = Vec3(0.8f, -0.3f, 1.2f);  // espaço do chassis
wheel.radius                 = 0.35f;
wheel.suspensionRestLength   = 0.35f;
wheel.suspensionStiffness    = 30000.0f;
wheel.suspensionDamping      = 2000.0f;
wheel.maxSuspensionForce     = 100000.0f;
wheel.friction               = 1.0f;
wheel.steerable              = true;
wheel.driven                 = true;
u32 fl = vehicle.AddWheel(wheel);

vehicle.SetThrottle(1.0f);   // -1..1
vehicle.SetSteer(0.4f);      // -1..1 (escalado por maxSteerAngle)
vehicle.SetBrake(0.0f);      // 0..1

// A cada quadro, antes de world.Step():
vehicle.Update(1.0f / 60.0f);
world.Step(1.0f / 60.0f);

bool grounded = vehicle.IsWheelGrounded(fl);
Real compression = vehicle.GetSuspensionCompression(fl);
Transform wheelT = vehicle.GetWheelTransform(fl);
```

## Corpos macios

```cpp
SoftBodySettings ss;
ss.gravity          = Vec3(0, -9.81f, 0);
ss.damping          = 0.01f;
ss.solverIterations = 8;

SoftBody cloth(world, ss);
cloth.CreateCloth(Vec3(0, 3, 0), /*largura*/ 1.0f, /*altura*/ 1.0f,
                  /*resX*/ 6, /*resY*/ 6);

SoftBody box(world, ss);
box.CreateBox(Vec3(0, 3, 0), Vec3(0.5f), /*resolução*/ 3);

// A cada quadro:
cloth.Update(1.0f / 60.0f);
box.Update(1.0f / 60.0f);

u32 n = box.GetNumParticles();
Vec3 p = box.GetParticlePosition(0);
```

As partículas colidem com o mundo rígido (formas esfera / caixa / plano) via um
push-out por distância assinada.

## Serialização

```cpp
#include "Kizuri/Kizuri.h"

// Em memória
Vector<u8> bytes;
SerializeWorld(world, bytes);
DeserializeWorld(world, bytes.Data(), bytes.Size());

// Para / de um arquivo
SaveWorldToFile(world, "scene.kzp");
LoadWorldFromFile(world, "scene.kzp");
```

O fluxo é versionado (magic `KZP1`). Cada corpo é reconstruído via `CreateBody`,
então os IDs **não** são preservados — resolva referências externas pela ordem
dos slots após carregar.

## Tratamento de erros

Entradas inválidas são protegidas por `KZ_ASSERT`. `BodyID::IsInvalid()` reporta
uma criação de corpo que falhou. As formas são imutáveis e com contagem de
referência, então uma forma não deve ser mutada depois de anexada a um corpo.
