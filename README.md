KizuriPhysics

Um motor de física 3D de corpos rígidos em tempo real, orientado para produção e desenvolvido em C++20 moderno, com arquitetura inspirada no "Jolt Physics" (https://github.com/jrouwe/JoltPhysics): dinâmica determinística por impulsos sequenciais, uma fase estreita de colisão GJK/EPA para formas convexas, uma árvore AABB dinâmica na fase ampla, suspensão de corpos baseada em ilhas e uma API limpa, com foco em cabeçalhos.

KizuriPhysics/
├── include/Kizuri/      Cabeçalhos públicos (toda a API)
├── src/                 Implementação
├── tests/               35 testes unitários e de integração independentes
├── examples/            Exemplos executáveis e benchmark
├── CMakeLists.txt       Compilação (biblioteca + testes + exemplos)
└── docs/                Documentação da arquitetura e da API

---

Destaques

Área| O que está implementado
Matemática| "Vec3", "Vec4", "Quat", "Mat3", "Mat4", "Transform", "AABB". Alinhamento de 16 bytes otimizado para SIMD, operações com poucas ramificações, "GetBasis", interpolação esférica (slerp) e decomposição de torção e balanço (twist/swing).
Memória| Alocadores linear, de pilha, de pool e de lista livre, por meio de uma única interface "Allocator".
Tarefas| Fila de trabalho MPMC, "ParallelFor" e dependências entre tarefas — utilizados na fase estreita de colisão paralela.
Formas| Esfera, caixa, cápsula, cilindro, envoltória convexa ("ConvexHull", com QuickHull incremental), malha triangular ("TriangleMesh", com BVH), formas compostas ("Compound"), campo de altura ("HeightField") e plano.
Propriedades de massa| Cálculo analítico para formas primitivas; decomposição tetraédrica para envoltórias e malhas (volume, centroide e tensor de inércia).
Fase ampla| Árvore AABB dinâmica, limites ampliados, movimentação incremental, consulta de auto-interseção com eliminação de sobreposições e filtragem por camadas de objetos.
Fase estreita| GJK + EPA para colisões entre formas convexas, recorte de faces baseado em SAT para poliedros (até 8 pontos de contato), caminhos analíticos rápidos para esfera/plano, iteração por triângulos para malhas e campos de altura e recursão para formas compostas.
Solver| Impulsos sequenciais (Gauss-Seidel Projetado), inicialização com impulsos anteriores (warm starting), atrito de Coulomb em dois eixos, restituição e correção de posição por impulso dividido (split-impulse), sem injeção de energia cinética.
Dinâmica| Corpos estáticos, cinemáticos e dinâmicos, amortecimento, fator de gravidade, limites de velocidade e suspensão baseada em ilhas.
Restrições| Ponto, distância, fixa, dobradiça (com limites e motor), deslizante (com limites e motor) e seis graus de liberdade (Six-DOF).
Consultas| Lançamento de raios, lançamento de todos os raios, lançamento de formas convexas (shape cast, com avanço conservador), consulta AABB e consulta por ponto.
Determinismo| "ComputeStateHash()" para verificação de simulações sincronizadas (lockstep) e reprodução de simulações (replay).
Depuração| Interface "DebugRenderer" + "PhysicsWorld::DrawDebug()" (formas, contatos e AABBs).

---

Início rápido

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
        // ... ler b->GetPosition(), b->GetRotation(), velocidades etc. ...
    }
}

Compilação

CMake (recomendado)

cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build          # executar os 34 testes
./build/examples/kizuri_HelloWorld

G++ diretamente (sem necessidade de CMake)

# Compilar os arquivos-fonte da biblioteca + seu programa.
g++ -std=c++20 -O2 -Iinclude -pthread \
    $(find src -name '*.cpp') meu_app.cpp -o meu_app

---

Arquitetura

O fluxo de execução segue uma ordem semelhante à do Jolt:

StepFixed(dt)
 ├─ 1. Atualização da fase ampla (mover proxies e reconstruir pares sobrepostos)
 ├─ 2. Fase estreita (GJK/EPA + conjuntos de contatos, paralelizados entre pares)
 ├─ 3. Construção das restrições de contato (impulsos anteriores do cache de contatos)
 ├─ 4. Integração das velocidades (gravidade, forças, amortecimento e limites)
 ├─ 5. Resolução de contatos e juntas (correção de posição por split-impulse)
 ├─ 6. Integração das posições
 ├─ 7. Atualização da inércia e suspensão (ilhas com union-find)
 └─ 8. Armazenamento dos impulsos para o próximo quadro

Principais decisões de projeto

* Identificadores em vez de ponteiros. "BodyID" armazena um índice de 24 bits e um número de sequência de 8 bits, impedindo que um identificador antigo acesse uma posição de memória liberada e reutilizada.

* Formas imutáveis com contagem de referências. Uma "Shape" é compartilhada entre corpos e nunca é modificada; "Ref<T>" permite conversões de "Ref<Derived>" para "Ref<Base>".

* Inicialização com impulsos anteriores entre quadros. Um conjunto de contatos armazenado em cache por hash mantém os impulsos de cada ponto, associando-os aos contatos do quadro seguinte com base na proximidade.

* Impulso dividido (split-impulse). A penetração é corrigida usando pseudovelocidades integradas às posições depois do movimento real, evitando que a correção de posição injete energia cinética e ajudando a manter pilhas de objetos estáveis.

* AABBs ampliadas + árvore incremental. Os corpos só precisam voltar à fase ampla quando saem dos limites ampliados, mantendo o custo dessa etapa muito baixo em situações estáveis.

Consulte ""docs/architecture.md"" (docs/architecture.md) para a explicação completa da arquitetura e ""docs/api.md"" (docs/api.md) para conhecer a API.

---

Desempenho medido

Solver de uma única thread, compilado com "-O2", em Linux x86-64 (GCC 12). Cenário: "N" caixas dinâmicas caindo em um recipiente com paredes, "dt = 1/60 s", 300 etapas, 8 iterações de velocidade e 3 iterações de posição. Os tempos representam a média durante toda a simulação até a estabilização, considerando o pior caso: uma pilha densa de objetos.

Corpos| Tempo médio por etapa| Fase ampla| Fase estreita| Resolução| Margem de FPS
1.000| 8,6 ms| 0,44 ms| 1,7 ms| 4,2 ms| ~116
2.000| 16,3 ms| 0,90 ms| 4,1 ms| 7,8 ms| ~61

Execute o benchmark por conta própria:

./build/examples/kizuri_Benchmark 2000 300

---

Roteiro de desenvolvimento (em direção à compatibilidade completa com o Jolt)

Estes sistemas ainda não estão implementados; a arquitetura permite incorporá-los futuramente:

- [ ] Solver paralelo por ilhas — resolver ilhas independentes em threads de trabalho (o sistema de tarefas e o construtor de ilhas já existem).
- [ ] Solver de contatos com SIMD — processamento em lotes de quatro elementos (4-wide SoA) para as linhas de contato.
- [ ] Detecção contínua de colisões (CCD) — lançamento linear de formas para corpos que se movem rapidamente.
- [ ] Controlador de personagens ("CharacterVirtual") e veículos com rodas.
- [ ] Corpos deformáveis, tecidos e ragdolls.
- [ ] Serialização do estado do mundo físico.

---

Testes

./build/tests/kizuri_tests

35 testes e 141 verificações que abrangem matemática, formas e propriedades de massa, GJK/EPA, geração de conjuntos de contatos, pares da fase ampla, colisões entre planos, esferas e caixas, empilhamento de objetos, suspensão, determinismo, lançamento de raios e lançamento de formas.

Licença

MIT — consulte o arquivo ""LICENSE"" (LICENSE).
