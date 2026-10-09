# KizuriPhysics

<<<<<<< HEAD
Um **motor de física 3D de corpos rígidos em tempo real**, orientado para produção e desenvolvido em C++20 moderno, com arquitetura inspirada no [Jolt Physics](https://github.com/jrouwe/JoltPhysics): dinâmica determinística por impulsos sequenciais, uma fase estreita de colisão GJK/EPA para formas convexas, uma fase ampla baseada em árvore AABB dinâmica, suspensão baseada em ilhas e uma API limpa, com foco em cabeçalhos.

```text
=======
Um **motor de física de corpos rígidos 3D** em tempo real, orientado à produção,
em C++20 moderno, arquitetado no espírito do
[Jolt Physics](https://github.com/jrouwe/JoltPhysics): dinâmica determinística
por impulsos sequenciais, fase estreita convexa com GJK/EPA, fase ampla com
árvore AABB dinâmica, sono por ilhas e uma API limpa, header-first.

```
>>>>>>> cd163a1 (v0.1.0 adicionados bagulhos novos)
KizuriPhysics/
├── include/Kizuri/      Headers públicos (toda a API)
├── src/                 Implementação
<<<<<<< HEAD
├── tests/               35 testes unitários e de integração independentes
├── examples/            Exemplos executáveis e benchmark
├── CMakeLists.txt       Compilação (biblioteca + testes + exemplos)
└── docs/                Documentação da arquitetura e da API
=======
├── tests/               45 testes unitários + de integração autocontidos
├── examples/            Exemplos executáveis + benchmark
├── CMakeLists.txt       Build (biblioteca + testes + exemplos)
└── docs/                Notas de arquitetura e API
>>>>>>> cd163a1 (v0.1.0 adicionados bagulhos novos)
```

---

## Destaques

| Área | O que está implementado |
|---|---|
<<<<<<< HEAD
| **Matemática** | `Vec3`, `Vec4`, `Quat`, `Mat3`, `Mat4`, `Transform`, `AABB`. Alinhamento de 16 bytes favorável a SIMD, operações com poucas ramificações, `GetBasis`, interpolação esférica (slerp) e decomposição de torção e balanço (twist/swing). |
| **Memória** | Alocadores linear, de pilha, de pool e de lista livre, por meio de uma única interface `Allocator`. |
| **Tarefas** | Fila de trabalho MPMC, `ParallelFor` e dependências entre tarefas — utilizados na fase estreita de colisão paralela. |
| **Formas** | Esfera, caixa, cápsula, cilindro, envoltória convexa (`ConvexHull`, com QuickHull incremental), malha triangular (`TriangleMesh`, com BVH), formas compostas (`Compound`), campo de altura (`HeightField`) e plano. |
| **Propriedades de massa** | Cálculo analítico para formas primitivas; decomposição tetraédrica para envoltórias e malhas (volume, centroide e tensor de inércia). |
| **Fase ampla** | Árvore AABB dinâmica, limites ampliados, movimentação incremental, consulta de auto-interseção com eliminação de sobreposições e filtragem por camadas de objetos. |
| **Fase estreita** | GJK + EPA para colisões entre formas convexas, recorte de faces baseado em SAT para poliedros (até 8 pontos de contato), caminhos analíticos rápidos para esfera/plano, iteração por triângulos para malhas e campos de altura e recursão para formas compostas. |
| **Solver** | Impulsos sequenciais (Gauss-Seidel Projetado), inicialização com impulsos anteriores (*warm starting*), atrito de Coulomb em dois eixos, restituição e correção de posição por **impulso dividido (*split-impulse*)**, sem injeção de energia cinética. |
| **Dinâmica** | Corpos estáticos, cinemáticos e dinâmicos, amortecimento, fator de gravidade, limites de velocidade e suspensão baseada em ilhas. |
| **Restrições** | Ponto, distância, fixa, dobradiça (com limites e motor), deslizante (com limites e motor) e seis graus de liberdade (Six-DOF). |
| **Consultas** | Lançamento de raios, lançamento de todos os raios, lançamento de formas convexas (*shape cast*, com avanço conservador), consulta AABB e consulta por ponto. |
| **Determinismo** | `ComputeStateHash()` para verificação de simulações sincronizadas (*lockstep*) e reprodução de simulações (*replay*). |
| **Depuração** | Interface `DebugRenderer` e `PhysicsWorld::DrawDebug()` (formas, contatos e AABBs). |

---

## Início rápido
=======
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
>>>>>>> cd163a1 (v0.1.0 adicionados bagulhos novos)

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

<<<<<<< HEAD
**G++ diretamente (sem necessidade de CMake)**

```bash
# Compilar os arquivos-fonte da biblioteca + seu programa.
g++ -std=c++20 -O2 -Iinclude -pthread \
    $(find src -name '*.cpp') meu_app.cpp -o meu_app
=======
**g++ puro (sem CMake)**

```bash
# Compila os fontes da biblioteca + seu programa.
g++ -std=c++20 -O2 -Iinclude -pthread \
    $(find src -name '*.cpp') my_app.cpp -o my_app
>>>>>>> cd163a1 (v0.1.0 adicionados bagulhos novos)
```

---

## Arquitetura

<<<<<<< HEAD
O fluxo de execução segue uma ordem semelhante à do Jolt:

```text
StepFixed(dt)
 ├─ 1. Atualização da fase ampla (mover proxies e reconstruir pares sobrepostos)
 ├─ 2. Fase estreita (GJK/EPA + conjuntos de contatos, paralelizados entre pares)
 ├─ 3. Construção das restrições de contato (impulsos anteriores do cache de contatos)
 ├─ 4. Integração das velocidades (gravidade, forças, amortecimento e limites)
 ├─ 5. Resolução de contatos e juntas (correção de posição por split-impulse)
 ├─ 6. Integração das posições
 ├─ 7. Atualização da inércia e suspensão (ilhas com union-find)
 └─ 8. Armazenamento dos impulsos para o próximo quadro
```

### Principais decisões de projeto

- **Identificadores em vez de ponteiros.** `BodyID` armazena um índice de 24 bits e um número de sequência de 8 bits. Assim, um identificador antigo não consegue acessar uma posição que foi liberada e reutilizada por outro corpo.

- **Formas imutáveis com contagem de referências.** Uma `Shape` pode ser compartilhada entre vários corpos e nunca é modificada. `Ref<T>` permite conversões de `Ref<Derived>` para `Ref<Base>`.

- **Inicialização com impulsos anteriores entre quadros.** Um conjunto de contatos armazenado em cache por hash mantém os impulsos de cada ponto, associando-os aos contatos do quadro seguinte com base na proximidade.

- **Impulso dividido (*split-impulse*).** A penetração é corrigida usando pseudovelocidades integradas às posições *depois* do movimento real, evitando que a correção de posição injete energia cinética e ajudando a manter pilhas de objetos estáveis.

- **AABBs ampliadas + árvore incremental.** Os corpos só precisam voltar à fase ampla quando saem dos limites ampliados, mantendo o custo dessa etapa muito baixo em situações estáveis.

Consulte [`docs/architecture.md`](docs/architecture.md) para a explicação completa da arquitetura e [`docs/api.md`](docs/api.md) para conhecer a API.
=======
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
>>>>>>> cd163a1 (v0.1.0 adicionados bagulhos novos)

---

## Desempenho medido

<<<<<<< HEAD
Solver de uma única thread, compilado com `-O2`, em Linux x86-64 (GCC 12).

Cenário: `N` caixas dinâmicas caindo dentro de um recipiente com paredes, `dt = 1/60 s`, 300 etapas, 8 iterações de velocidade e 3 iterações de posição. Os tempos representam a média durante toda a simulação até a estabilização, considerando o pior caso: uma pilha densa de objetos.

| Corpos | Tempo médio por etapa | Fase ampla | Fase estreita | Resolução | Margem de FPS |
|---:|---:|---:|---:|---:|---:|
| 1.000 | 8,6 ms | 0,44 ms | 1,7 ms | 4,2 ms | ~116 |
| 2.000 | 16,3 ms | 0,90 ms | 4,1 ms | 7,8 ms | ~61 |

Execute o benchmark por conta própria:
=======
Solver single-thread, `-O2`, Linux x86-64 (GCC 12). Cena: `N` caixas dinâmicas
jogadas num contêiner com paredes, `dt = 1/60 s`, 300 passos, 8 iterações de
velocidade / 3 de posição. Os tempos são a média ao longo de todo o assentamento
(pior caso, pilha densa).

| Corpos | passo médio | fase ampla | fase estreita | solve | folga de FPS |
|-------:|------------:|-----------:|--------------:|------:|-------------:|
| 1 000  | 8,6 ms | 0,44 ms | 1,7 ms | 4,2 ms | ~116 |
| 2 000  | 16,3 ms | 0,90 ms | 4,1 ms | 7,8 ms | ~61 |

Rode você mesmo:
>>>>>>> cd163a1 (v0.1.0 adicionados bagulhos novos)

```bash
./build/examples/kizuri_Benchmark 2000 300
```

---

<<<<<<< HEAD
## Roteiro de desenvolvimento (em direção à compatibilidade completa com o Jolt)

Estes sistemas ainda não estão implementados; a arquitetura permite incorporá-los futuramente:

- [ ] **Solver paralelo por ilhas** — resolver ilhas independentes em threads de trabalho (o sistema de tarefas e o construtor de ilhas já existem).
- [ ] **Solver de contatos com SIMD** — processamento em lotes de quatro elementos (*4-wide SoA*) para as linhas de contato.
- [ ] **Detecção contínua de colisões (CCD)** — lançamento linear de formas para corpos que se movem rapidamente.
- [ ] **Controlador de personagens (`CharacterVirtual`) e veículos com rodas.**
- [ ] **Corpos deformáveis, tecidos e ragdolls.**
- [ ] **Serialização do estado do mundo físico.**
=======
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
>>>>>>> cd163a1 (v0.1.0 adicionados bagulhos novos)

---

## Testes

```bash
./build/tests/kizuri_tests
```

<<<<<<< HEAD
35 testes e 141 verificações que abrangem matemática, formas e propriedades de massa, GJK/EPA, geração de conjuntos de contatos, pares da fase ampla, colisões entre planos, esferas e caixas, empilhamento de objetos, suspensão, determinismo, lançamento de raios e lançamento de formas.

## Licença

MIT — consulte o arquivo [`LICENSE`](LICENSE).
=======
45 testes / 177 verificações cobrindo matemática, formas e propriedades de
massa, GJK/EPA, geração de manifolds, pares da fase ampla, colisões
plano/esfera/caixa, empilhamento, sono, determinismo, ray casts, shape casts,
CCD, o controlador de personagem, o veículo, corpos macios e a serialização do
mundo.

## Licença

MIT — veja [`LICENSE`](LICENSE).
>>>>>>> cd163a1 (v0.1.0 adicionados bagulhos novos)
