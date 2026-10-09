# KizuriPhysics — Arquitetura

Este documento explica como um passo é executado e por que cada subsistema foi
construído do jeito que é.

## 1. Convenções de coordenadas e armazenamento

* Destro, **Y para cima**, metros e segundos.
* `Transform = { Vec3 position; Quat rotation; }` — a rotação é aplicada em
  torno da origem do corpo; o centro de massa é um deslocamento separado.
* `Vec3` é alinhado em 16 bytes para poder ser carregado diretamente em
  registradores SSE/NEON; o solver também oferece um caminho escalar que o
  compilador auto-vetoriza.
* Os corpos são endereçados por `BodyID` (índice de 24 bits + sequência de 8
  bits). A sequência protege contra o problema ABA quando um slot é reciclado.

## 2. Fase ampla — árvore AABB dinâmica

`DynamicAABBTree` é uma BVH binária de fat AABBs (limites justos + margem de
0,1 m + deslocamento por passo). Cada folha corresponde a um proxy de corpo.

* **Inserção** escolhe o irmão que minimiza o custo de área de superfície e
  depois sobe reajustando limites e rotacionando para manter a árvore
  balanceada.
* **MoveProxy** é barato: se a nova AABB justa ainda está dentro da fat AABB, a
  folha *não* é reinserida. Só as saídas disparam remoção/inserção.
* **Geração de pares** (`QueryPairs`) é uma travessia de auto-interseção podada
  por sobreposição. Pares de nós cujos fat bounds não se sobrepõem são
  descartados, e o nó maior é dividido para manter a pilha balanceada. Isso torna
  a geração de pares `O(n log n + k)` no número `k` de pares sobrepostos.
* **Filtragem por camada** rejeita um par quando
  `(layerA & maskB) == 0 || (layerB & maskA) == 0`.

A árvore usa uma pilha expansível em `QueryPairs` (nunca um buffer fixo), de
modo que cenas grandes não podem descartar pares silenciosamente.

## 3. Fase estreita — GJK, EPA, manifolds

### Convexo vs convexo

1. **GJK** (`Collision/GJK.cpp`) calcula os pontos mais próximos entre duas
   formas convexas usando a diferença de Minkowski e um simplex reduzido à sua
   feature de suporte a cada iteração. Ele retorna a distância de separação e os
   pontos testemunha, ou um relatório de que as formas se intersectam.
2. **EPA** (`Collision/EPA.cpp`) expande um politopo inicial que contém
   estritamente a origem para achar a profundidade e a normal de penetração
   mínimas. O politopo inicial é semeado a partir do simplex do GJK e completado
   com amostras de suporte; uma busca por força bruta sobre eixos + amostras
   diagonais garante um tetraedro estritamente contido mesmo para sobreposições
   simétricas/alinhadas aos eixos (o caso em que a origem está exatamente sobre
   o simplex do GJK).
3. **Geração de manifold** então produz os pontos de contato:
   * *Pares poliédricos* (Box, ConvexHull): a face de referência de uma forma é
     recortada contra os planos laterais da face incidente da outra
     (Sutherland–Hodgman), produzindo até 4 pontos estáveis.
   * *Pares convexos suaves* (esfera, cápsula, cilindro): os pontos testemunha
     do EPA dão um único contato.

### Casos especiais e formas não convexas

* **Esfera** e **Plano** têm caminhos analíticos rápidos.
* **TriangleMesh** e **HeightField** iteram triângulos candidatos e executam um
  recorte convexo-vs-triângulo (ou um teste de ponto mais próximo para
  esferas/cápsulas).
* Formas **Compound** recursam nos filhos e mesclam os manifolds.

## 4. Solver — impulso sequencial com split impulse

`ContactSolver` constrói um `ContactConstraint` por par de corpos, cada um
segurando até `kMaxManifoldPoints` restrições de ponto.

Por ponto ele pré-calcula:

* a base de contato no mundo `{n, t1, t2}`,
* a massa efetiva ao longo de cada eixo (`EffectiveMass` considera tanto os
  termos lineares quanto os angulares),
* o alvo de restituição a partir da velocidade de aproximação,
* o viés de posição a partir da penetração.

O solve de velocidade então executa `velocityIterations` varreduras de impulsos
**atrito → normal**, limitando o impulso normal acumulado a `>= 0` e o impulso
de atrito a `±μ·Pₙ` (cone de Coulomb). Os impulsos acumulados recebem warm start
do quadro anterior.

**A correção de posição** usa split impulse: um segundo conjunto de
*pseudo-velocidades* é resolvido contra o viés de posição e integrado nas
posições depois do movimento real. Como as pseudo-velocidades nunca realimentam
as velocidades reais, corrigir a penetração não adiciona energia — o motivo pelo
qual uma pilha de caixas assenta em vez de vibrar.

Se `useVelocityBias` estiver habilitado, o solver em vez disso dobra o viés de
posição no solve de velocidade (Baumgarte clássico). Split impulse é o padrão.

## 5. Ilhas, sono e o solve paralelo

Ao final de um passo, contatos e juntas são usados para unir corpos em **ilhas**
com uma passagem de union-find sobre os corpos dinâmicos ativos. Um corpo acumula
um temporizador de sono enquanto suas velocidades linear e angular ficam abaixo
dos limites; uma ilha só é posta para dormir quando *todos* os corpos dinâmicos
nela ultrapassaram `sleepTimeThreshold`. Corpos dormindo saem da integração e das
atualizações da fase ampla, e são acordados por novos contatos ou por um impulso
explícito.

A mesma decomposição em ilhas impulsiona o paralelismo. `BuildIslands()` atribui
cada contato e junta à ilha dos seus corpos, então as ilhas são disjuntas e podem
ser resolvidas concorrentemente. `StepFixed` então chama
`JobSystem::ParallelFor` sobre a lista de ilhas, e cada worker roda `SolveIsland`
(passe de velocidade) seguido de `SolveIslandPositions` (passe de split impulse).

**Segurança de corrida.** Dois corpos nunca compartilham uma ilha, então uma
ilha possui seus corpos com exclusividade. Corpos estáticos e cinemáticos são
*compartilhados* por muitas ilhas, então `ApplyImpulsePair` nunca escreve num
corpo não dinâmico — uma ilha só muta os próprios corpos dinâmicos. O
`ContactConstraint` guarda os flags `dynamicA` / `dynamicB` pelo mesmo motivo.

### Lotes de contato SIMD

Dentro do passe de velocidade, `ContactSolverSIMD::SolveVelocityBatchSIMD`
reúne até quatro pontos de contato cujos corpos são dois a dois disjuntos num
layout SoA 4-wide (normais, tangentes, massas efetivas, impulsos acumulados) e os
resolve num único lote de lanes SSE. Pontos que não podem ser agrupados caem no
caminho escalar, então o resultado é idêntico com ou sem `useSIMDSolver`.

## 6. Determinismo

O motor não usa tempo de relógio e não há acumulação dependente da ordem das
threads dentro do solver (a fase estreita escreve em slots por par, então seu
resultado é independente de ordem). `PhysicsWorld::ComputeStateHash()` dobra a
posição, a rotação e a velocidade de cada corpo num hash de 64 bits para
verificação lockstep.

## 7. Detecção de colisão contínua

Um corpo com `motionQuality = MotionQuality::LinearCast` tem seu movimento
varrido limitado em `ClampMotionCCD()` durante a integração de posições. A
varredura linear do corpo ao longo do passo é lançada contra o mundo; quando um
impacto é encontrado dentro do passo, o corpo avança apenas até o contato menos
`ccdMaxPenetration`, e a componente da velocidade ao longo da normal da
superfície é removida. Corpos discretos pulam o teste por completo, então o CCD
não custa nada para objetos lentos.

## 8. Controlador de personagem

`CharacterController` é uma cápsula cinemática dirigida por *collide-and-slide*:
uma cápsula de cast encolhida (raio reduzido por uma skin) é varrida ao longo do
deslocamento desejado, e o movimento restante é projetado no plano de contato e
lançado de novo, até um pequeno orçamento de iterações. O contato com o chão é
detectado por `CastShapeAll` filtrado para normais mais inclinadas que o limite
de rampa caminhável; um pequeno auxílio de subida de degrau permite que o
controlador suba degraus cuja altura esteja dentro de `maxStepHeight`. O
controlador nunca escreve nos corpos que toca.

## 9. Veículo com rodas

`Vehicle` é um veículo por raycast. Cada roda guarda um ponto de fixação local e
uma suspensão (comprimento de repouso, rigidez, amortecimento, curso máximo).
Cada `Update(dt)`:

1. lança um raio do ponto de fixação ao longo do eixo "para baixo" do chassis;
2. converte a distância do impacto em compressão da suspensão e aplica uma força
   de mola/amortecedor no contato;
3. decompõe a velocidade do ponto de contato em componentes frontal e lateral;
4. aplica força de motor/freio ao longo do eixo frontal (esterçado) da roda e
   uma força de atrito lateral limitada por Coulomb.

A força da suspensão é limitada a `maxSuspensionForce`, e as forças do pneu são
limitadas a `μ·N`, então o veículo não consegue gerar tração que não tem.

## 10. Corpos macios

`SoftBody` é um sistema de position-based dynamics (PBD). As partículas carregam
massa inversa; as arestas carregam comprimentos de repouso. `Update(dt)` prevê as
posições sob gravidade e amortecimento, depois itera a projeção das restrições de
distância (ponderada pela massa inversa) `solverIterations` vezes, resolve as
colisões das partículas contra o mundo rígido e, por fim, deriva as velocidades a
partir do delta de posição. Um helper de lattice (`CreateBox`) e um helper de
pano (`CreateCloth`, linha superior fixada) constroem as topologias comuns. A
colisão das partículas usa a distância assinada da forma e seu gradiente por
diferenças finitas para empurrar a partícula ao longo da normal da superfície.

## 11. Serialização

`SerializeWorld` escreve um fluxo binário versionado (magic `KZP1`, versão 1):
para cada corpo escreve o tipo e os parâmetros da forma, o estado completo de
`BodySettings` (transform, velocidades, tipo/qualidade de movimento, atrito,
restituição, amortecimento, fator de gravidade, limites de velocidade,
camada/máscara, flags) e o flag de ativo. `DeserializeWorld` reconstrói cada
corpo via `CreateBody`, recriando primitivas, planos e convex hulls. O formato é
little-endian e dependente da arquitetura para `Real`.

## 12. Threading

`JobSystem` oferece uma fila MPMC, `ParallelFor` e dependências de jobs. A fase
estreita se distribui sobre os pares da fase ampla quando a contagem de pares
ultrapassa um limite, e o solve por ilhas se distribui sobre as ilhas. Como as
ilhas são disjuntas e corpos estáticos nunca são escritos, o solve paralelo é
determinístico no sentido de que dois workers nunca tocam a mesma memória.
