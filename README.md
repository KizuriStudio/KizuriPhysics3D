# KizuriPhysics

Um **motor de física de corpos rígidos 3D** em tempo real, orientado à produção,
em C++20 moderno, arquitetado no espírito do
[Jolt Physics](https://github.com/jrouwe/JoltPhysics): dinâmica determinística
por impulsos sequenciais, fase estreita convexa com GJK/EPA, fase ampla com
árvore AABB dinâmica, sono por ilhas e uma API limpa, header-first.

```
KizuriPhysics/
├── include/Kizuri/      Headers públicos (toda a API)
├── src/                 Implementação
├── tests/               45 testes unitários + de integração autocontidos
├── examples/            Exemplos executáveis + benchmark
├── CMakeLists.txt       Build (biblioteca + testes + exemplos)
└── docs/                Notas de arquitetura e API
```

---

## Destaques

| Área | O que está implementado |
|---|---|
| **Matemática** | `Vec3`, `Vec4`, `Quat`, `Mat3`, `Mat4`, `Transform`, `AABB`. Alinhamento de 16 bytes amigável a SIMD, operações enxutas, `GetBasis`, slerp, decomposição twist/swing. |
| **Memória** | Alocadores linear, de pilha, de pool e free-list atrás de uma única interface `Allocator`. |
| **Jobs** | Fila de trabalho MPMC, `ParallelFor`, dependências de jobs — usados na fase estreita paralela. |
| **Formas** | Sphere, Box, Capsule, Cylinder, ConvexHull (QuickHull incremental), TriangleMesh (BVH), Compound, HeightField, Plane. |
| **Propriedades de massa** | Analíticas para primitivas; decomposição tetraédrica para hulls/malhas (volume, centroide, tensor de inércia). |
| **Fase ampla** | Árvore AABB dinâmica, fat bounds, movimento incremental, consulta de auto-interseção podada por sobreposição, filtragem por camada. |
| **Fase estreita** | GJK + EPA para convexo-convexo, recorte de faces estilo SAT para poliedros (até 8 pontos de contato), caminhos analíticos rápidos para esfera/plano, iteração de triângulos para malha/heightfield, recursão de compound. |
| **Solver** | Impulso sequencial (Gauss-Seidel projetado), warm starting, atrito de Coulomb em 2 eixos, restituição, correção de posição por **split impulse** (sem injeção de energia). |
| **Solver paralelo** | **Ilhas** por union-find construídas a cada passo; ilhas independentes são resolvidas em paralelo pelo sistema de jobs. |
| **Solver SIMD** | Lote de contatos **SoA** 4-wide (`ContactSolverSIMD`) sobre pares de corpos disjuntos, controlado por `SolverSettings::useSIMDSolver`. |
| **Dinâmica** | Corpos Static / Kinematic / Dynamic, amortecimento, fator de gravidade, limites de velocidade, sono por ilhas. |
| **Restrições** | Point, Distance, Fixed, Hinge (limites + motor), Slider (limites + motor), Six-DOF. |
| **Colisão contínua** | CCD por `MotionQuality::LinearCast` — o movimento varrido é limitado para que corpos rápidos não atravessem. |
| **Personagem** | `CharacterController` — collide-and-slide, subida de degrau, aderência ao chão, filtro de rampa caminhável. |
| **Veículo** | `Vehicle` com rodas por raycast — suspensão por roda (mola/amortecedor), força do motor, freios, direção, atrito do pneu. |
| **Corpos macios** | `SoftBody` por position-based dynamics — lattices de partículas, restrições de distância, pano, colisão com o mundo. |
| **Serialização** | Binário versionado `SerializeWorld` / `DeserializeWorld` (+ helpers de arquivo) com estado completo dos corpos e reconstrução das formas. |
| **Consultas** | Ray cast, ray cast all, shape cast convexo (avanço conservativo), consulta AABB, consulta por ponto. |
| **Determinismo** | `ComputeStateHash()` para verificação lockstep / replay. |
| **Depuração** | Interface `DebugRenderer` + `PhysicsWorld::DrawDebug()` (formas, contatos, AABBs). |

---

## Começo rápido

```cpp
#include "Kizuri/Kizuri.h"
using namespace kizuri;

int main() {
    PhysicsWorld world;
    world.SetGravity(Vec3(0, -9.81f, 0));

    // Plano de chão estático.
    BodySettings ground;
    ground.shape = MakeRef<PlaneShape>(Plane(Vec3::UnitY(), 0));
    ground.motionType = MotionType::Static;
    ground.friction = 0.8f;
    world.CreateBody(ground);

    // Esfera dinâmica.
    BodySettings ball;
    ball.shape = MakeRef<SphereShape>(0.5f);
    ball.position = Vec3(0, 5, 0);
    ball.motionType = MotionType::Dynamic;
    BodyID id = world.CreateBody(ball);

    for (int i = 0; i < 300; ++i) {
        world.Step(1.0f / 60.0f);
        const Body* b = world.GetBody(id);
        // ... leia b->GetPosition(), b->GetRotation(), velocidades ...
    }
}
```

### Compilação

**CMake (recomendado)**

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build          # roda os 45 testes
./build/examples/kizuri_HelloWorld
```

**g++ puro (sem CMake)**

```bash
# Compila os fontes da biblioteca + seu programa.
g++ -std=c++20 -O2 -Iinclude -pthread \
    $(find src -name '*.cpp') my_app.cpp -o my_app
```

---

## Arquitetura

O pipeline do passo espelha a ordenação do Jolt:

```
StepFixed(dt)
 ├─ 1. Atualiza fase ampla      (move proxies, refaz pares sobrepostos)
 ├─ 2. Fase estreita            (GJK/EPA + manifolds, paralelo sobre os pares)
 ├─ 3. Constrói restrições      (warm-start dos impulsos do cache de manifold)
 ├─ 4. Integra velocidades      (gravidade, forças, amortecimento, limites)
 ├─ 5. Constrói ilhas           (union-find sobre corpos dinâmicos ativos)
 ├─ 6. Resolve contatos + juntas(ilhas em paralelo; lotes SIMD 4-wide)
 ├─ 7. Clamp de CCD + integra posições
 ├─ 8. Atualiza inércia / sono  (por ilha)
 └─ 9. Guarda impulsos p/ o próximo quadro
```

Decisões de projeto principais:

* **Handles, não ponteiros.** `BodyID` empacota um índice de 24 bits com um
  número de sequência de 8 bits, então um slot destruído/reutilizado nunca pode
  ser acessado por um handle obsoleto.
* **Formas imutáveis com contagem de referência.** Uma `Shape` é compartilhada
  entre corpos e nunca é mutada; `Ref<T>` suporta conversão
  `Ref<Derived> -> Ref<Base>`.
* **Warm starting entre quadros.** Um manifold em cache por hash guarda os
  impulsos por ponto, casados com os contatos do próximo quadro por proximidade.
* **Split impulse.** A penetração é removida com pseudo-velocidades integradas
  nas posições *após* o movimento real, de modo que a correção de posição nunca
  injeta energia cinética (é o que mantém as pilhas calmas).
* **Fat AABBs + árvore incremental.** Os corpos só reentram na fase ampla quando
  saem dos fat bounds, mantendo o custo em regime quase nulo.

Veja [`docs/architecture.md`](docs/architecture.md) para o passo a passo completo
e [`docs/api.md`](docs/api.md) para a superfície da API.

---

## Desempenho medido

Solver single-thread, `-O2`, Linux x86-64 (GCC 12). Cena: `N` caixas dinâmicas
jogadas num contêiner com paredes, `dt = 1/60 s`, 300 passos, 8 iterações de
velocidade / 3 de posição. Os tempos são a média ao longo de todo o assentamento
(pior caso, pilha densa).

| Corpos | passo médio | fase ampla | fase estreita | solve | folga de FPS |
|-------:|------------:|-----------:|--------------:|------:|-------------:|
| 1 000  | 8,6 ms | 0,44 ms | 1,7 ms | 4,2 ms | ~116 |
| 2 000  | 16,3 ms | 0,90 ms | 4,1 ms | 7,8 ms | ~61 |

Rode você mesmo:

```bash
./build/examples/kizuri_Benchmark 2000 300
```

---

## Cobertura de recursos

Todos os sistemas de paridade com o Jolt estão implementados, testados e
exercitados por um exemplo:

- [x] **Solver paralelo por ilhas** — `PhysicsWorld::BuildIslands` / `SolveIsland`,
      distribuído via `JobSystem::ParallelFor`.
- [x] **Solver de contatos SIMD** — lotes SoA 4-wide (`ContactSolverSIMD`).
- [x] **Detecção de colisão contínua** — `MotionQuality::LinearCast`.
- [x] **Controlador de personagem** — `CharacterController` (`examples/Character.cpp`).
- [x] **Veículo com rodas** — `Vehicle` (`examples/Vehicle.cpp`).
- [x] **Corpos macios / pano** — `SoftBody` (`examples/SoftBody.cpp`).
- [x] **Serialização** — `SerializeWorld` / `DeserializeWorld`.

Rode-os:

```bash
./build/examples/kizuri_Character
./build/examples/kizuri_Vehicle
./build/examples/kizuri_SoftBody
./build/examples/kizuri_CCD
```

---

## Testes

```bash
./build/tests/kizuri_tests
```

45 testes / 177 verificações cobrindo matemática, formas e propriedades de
massa, GJK/EPA, geração de manifolds, pares da fase ampla, colisões
plano/esfera/caixa, empilhamento, sono, determinismo, ray casts, shape casts,
CCD, o controlador de personagem, o veículo, corpos macios e a serialização do
mundo.

## Licença

MIT — veja [`LICENSE`](LICENSE).
