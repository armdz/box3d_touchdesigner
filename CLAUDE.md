# box3d-touchdesigner

Operadores nativos de TouchDesigner (C++ custom operators, Windows x64 y macOS
arm64/x86_64) que exponen el motor de física **Box3D** de Erin Catto dentro de TD, con UX
estilo Bullet Solver.

**Leer `PLAN.md` antes de trabajar acá**: tiene el contexto completo, las decisiones de
arquitectura (por qué CHOP y no POP, composición sin cables vía registro global, un
operador por DLL sobre un core compartido), el contrato de atributos del Spawn SOP y el
estado fase por fase. `README.md` es la doc pública en inglés (build, instalación, uso).

Estado: Fases 0–5 (joints) completas. Nodos: **Box3D Solver CHOP** (mundo/step/Collision
SOP/Allow Sleep), **Box3D Body SOP** (un body: hull/primitiva/Mesh estático/Compound de
hulls; salida = geometría transformada, con overlays de pivot y collider), **Box3D
Instances CHOP** (masas para instancing, count variable en vivo), **Box3D Joint CHOP**
(hasta 8 pares + series, estilo Constant CHOP), **Box3D Set Joint SOP** (pivote como
atributos afín-invariantes en la cadena SOP), **Box3D Debug SOP** (wireframe del mundo de
colisión real + joints, colores por estado). Todo sobre `Box3DCore.dll` (registro de
mundos, box3d estático adentro). Pendiente: eventos de contacto, Force CHOP, filtrado de
colisiones (categorías/máscaras), sensores, COMP wrappers .tox.

Update 2026-07:

- Solver quedó **world-only** (sin spawn/demo propio; su salida CHOP de transform queda en cero por compatibilidad).
- Se agregó contenedor estático opcional (4 paredes) y parámetros `Workers`, `External Accel`, `Max Steps / Cook`,
  `Allow Sleep` (toggle world-level, aplica en vivo vía `b3World_EnableSleeping`; apagarlo despierta todo — útil si
  una isla quedó dormida y parece "congelada").
- `setGroup()` del core hace updates locales: material se aplica en caliente sobre los shapes existentes; cambios de
  pose mueven solo static/kinematic (los dinámicos solo toman pose de spawn al crearse o en un rebuild); cambios
  estructurales (shape/size/type/count) se parchean **POR ÍNDICE**: los bodies cuyo def no cambió conservan su body
  vivo intacto (ni se recrean), los índices cambiados se recrean individualmente (preservando estado dinámico finito
  si el spawn pose no cambió), y crecer/achicar el count solo spawnea/destruye la cola — un Spawn SOP estilo emisor
  puede variar su cantidad de puntos SIN resetear lo que ya simula. Identidad de body = índice de punto (mantener
  orden estable upstream; agregar puntos al final). Excepción: si hay shape Mesh (4) involucrado cae al recreate
  completo del grupo (los b3MeshData se poolean por grupo sin tracking por body).
- Kinematic updates usan `b3Body_SetTargetTransform(...)` para contactos más estables con dinámicos. `advance()`
  re-targetea los kinematics antes de cada step (box3d nunca frena kinematics solo; sin esto derivan al infinito
  cuando la animación upstream se detiene). El re-target va con `wake=false` para dejar dormir a los kinematics
  quietos, PERO despierta explícitamente al body si está dormido y el target se movió (comparación pos/quat con
  epsilon y signo de cuaternión alineado) — sin eso, un kinematic dormido nunca retomaba la animación upstream y
  su isla entera quedaba congelada. Deltas negativos (scrub hacia atrás) no drenan el acumulador.
- Body SOP intenta seguir animación rígida upstream (Transform SOP) sin rebuild global.
- El param `Solver` de todos los clientes (Body/Instances/Joint/Debug) tiene default `box3dsolver1`: un nodo
  creado junto a un solver con nombre default se bindea solo (path relativo al mismo COMP). No hay
  auto-descubrimiento real: el SDK no puede enumerar la escena, y bindear por Registry sin el param rompería
  el orden de cook (la referencia del parámetro ES la dependencia que hace que el mundo stepee antes que los
  actores).
- **cookEveryFrame + cookOnStart + heartbeat**: Solver, Body SOP, Instances CHOP y Joint CHOP usan
  `cookEveryFrame` (no `IfAsked`) y `customInfo.cookOnStart = true` — existir en el mundo físico no puede
  depender de que algo consuma el output (el Joint CHOP típicamente no está cableado a nada: con IfAsked no
  cookeaba nunca al cargar la escena y "a veces los joints no andaban"). Con cook garantizado, "no cookear"
  significa bypass/cook-flag-off/COMP deshabilitado, y el core lo usa como heartbeat: `setGroupPath` y
  `setJointNodeList` (llamados cada cook) marcan `lastTouch`/`jointTouch` con `advanceCounter`, y `advance()`
  remueve grupos/joints sin touch por más de 4 advances → **bypassear un nodo lo saca del mundo en ~4 frames**
  (al des-bypassear se re-registra; los dinámicos respawnean con su pose de spawn). El SDK no expone el estado
  de bypass — TD simplemente deja de llamar execute() — por eso heartbeat y no notificación. Bypassear el
  Solver pausa el mundo entero (advance no corre, nada se remueve). Debug/SetJoint quedan IfAsked (no
  registran estado).
- Body SOP tiene shape **`Mesh (Static)`** (core shape 4): malla de triángulos exacta (cóncavas OK — terrenos,
  tubos), solo para static/kinematic (box3d no soporta meshes en dinámicos; un dinámico con Mesh se fuerza a
  static con warning; para dinámicos cóncavos: hull o composición). Los `b3MeshData*` viven por grupo
  (`Group::meshes`, box3d los referencia sin copiar) y se destruyen junto con los bodies; grupos borrados con
  rebuild pendiente van a `orphanMeshes` hasta el próximo `destroyWorld`. Sin frame-following para meshes:
  geometría animada upstream recrea el grupo por cook (aceptable para estáticos). El `Collision SOP` del Solver
  sigue existiendo para un mesh de mundo único. Todos los meshes se crean con `weldVertices=true` (tol 1e-4):
  la adyacencia de aristas de box3d va por índice compartido y la geo de TD suele traer puntos duplicados
  (Facet/merges) — sin weld quedan grietas de contacto en las aristas por donde los bodies se escapan.
- **Los contactos de mesh en box3d son one-sided** (culling de back-side por winding en triangle_manifold.c) y
  TD no garantiza winding — un Torus directo no colisionaba. `extractOrientedTriangles()` (common/TDB3Mesh.h,
  compartido por Body SOP mesh y Collision SOP del Solver) fan-triangula y flipea cada triángulo para que la
  cara que sombrea (normales point o vertex del input) sea la que colisiona; sin normales, el winding pasa tal
  cual. Cualquier polígono sirve (quads etc., el SDK ya convierte todo a Polygon).
- El Body SOP y el Set Joint SOP preservan UVs (`setTexCoord/s`): Body maneja uv point y vertex (vertex expande
  puntos por vértice, mismo mecanismo que las normales vertex/primitive; normales point también se preservan en
  el path expandido); Set Joint solo uv/N de punto (mantiene topología compartida — ponerlo antes del Facet).
- Body SOP shape **`Compound (Hulls)`** (core shape 5): un hull convexo por isla de conectividad del input
  (union-find sobre índices de punto compartidos por polígonos; puntos sueltos se ignoran), todas las piezas
  como shapes múltiples del MISMO body — cóncavo y **dinámico OK** (masa/inercia de todas las piezas; es la
  composición convexa estándar de la industria, no hace falta `b3_compoundShape`). `SpawnBody.hullPieceCounts`
  delimita las piezas dentro de `hullPoints`. Máx 64 piezas (el resto se mergea en la última, con warning);
  buffers de `b3Body_GetShapes` subidos a 64 en material/debug. Sin frame-following (como mesh): identidad de
  orientación al spawn. Piezas con <4 puntos se saltean; sin piezas usables cae a box.
- Toggle `CCD (Bullet)` en Body SOP e Instances CHOP (y atributo por punto `bullet` en el Spawn SOP):
  `b3BodyDef.isBullet` para dinámicos rápidos (CCD contra kinematic/dynamic; contra estáticos ya es automático
  vía `enableContinuous` del mundo, on por default). Cambiarlo recrea el body (está en `sameShapeAndType`).
- **Box3D Debug SOP** (`Box3ddebug`, DLL propia): debug draw estilo Bullet. `SolverCore::getDebugWireframe()`
  lee las shapes VIVAS de vuelta desde box3d (`b3Shape_GetType/GetSphere/GetCapsule/GetHull/GetMesh` +
  accessors de half-edges/triángulos) — o sea hulls post-simplificación de budget y meshes post-weld, en la
  pose viva de cada body — y devuelve segmentos world-space + color RGB por segmento (dinámico despierto
  verde / dormido azul / kinematic naranja / static gris / ground-walls-collision-mesh gris oscuro / joints
  amarillo con cruces en anchors). El core trackea `worldStaticBodies` (ground/paredes/mesh del mundo) solo
  para esto. El SOP saca line prims con `Cd`; uso pensado: standalone, Display ON + Render OFF (el SDK no
  permite dibujar en el viewer sin outputearlo — no hay guides custom ni flags custom junto a
  Compare/Template/Render). Aristas de mesh compartidas se dibujan una vez (filtro por orden de índices;
  asume mallas soldadas cerradas). El Body SOP tiene además `Show Collision Shape`: overlay del wireframe de
  SU collider dentro del propio output (via `getGroupWireframe(groupKey, ...)`), sin nodo aparte — para ver
  el mundo completo (instancias/joints/estáticos) sigue estando el Debug SOP, porque los CHOPs no pueden
  emitir geometría.
- Instances CHOP ahora publica `sx sy sz` además de `tx ty tz rx ry rz`, con escala saneada para evitar 0/negativos.
- Spawn SOP acepta aliases de size (`size0..2`, `sizex/y/z`, `sx/y/z`) y `rx/ry/rz` como fallback de orientación inicial.
- Instances agrega presets de material (`Custom`, `Soft`, `Medium`, `Bouncy`) para defaults rápidos.
- **Joints (Fase 5)**: la vía principal es el **Box3D Joint CHOP** (DLL nueva, mismo patrón que los bodies:
  referencia al Solver por path, se registra en el core con su opId). Params: Type (distance/spherical/revolute/
  weld), `Joints` (1–8, habilita filas de pares en la página `Bodies` estilo Constant CHOP — el SDK C++ NO tiene
  parámetros secuenciales reales, es un pool fijo con enablePar; slot 1 conserva los nombres legacy
  `Bodya/Idxa/Bodyb/Idxb`, slots 2+ son `Bodya2`, ...), Body A/B por slot (path o nombre de un Body SOP /
  Instances CHOP; B vacío = anclado al mundo), `Count` (convierte cada par en serie de índices para cadenas sobre
  Instances), y página Dynamics (spring hertz/damping, límites en grados, cono, motor, collide — compartida por
  todos los joints del nodo). El pivote vive en el Body SOP con `Joint` / `Joint Pivot` + `Show Joint Pivot`
  (en página `Joint` propia; se auto-grisan cuando el input trae atributos del Set Joint SOP — NO quitarlos:
  son la única vía de pivote para bodies sin Set Joint upstream y para primitivas sin input); el
  joint solo conecta Body A/B. Salida CHOP: `ax..bz`, `active`, un sample por joint (orden: slot-major, serie
  minor). El param `Pairs` (lista de texto `A>B; C>;`) se eliminó a favor de los slots. Hubo un **Box3D Joint
  SOP** equivalente (un joint por nodo, salida = línea entre anchors); se eliminó por redundante — el CHOP es
  superconjunto. OJO: la "vía scriptable" por `Joints DAT` en el Solver que describían versiones anteriores de
  este documento NO está implementada en el código actual (quedó como idea/pendiente; el Solver no tiene página
  Joints ni parámetro DAT). El core registra el opPath
  de cada grupo (los clientes lo re-registran cada cook) y resuelve referencias lazy; `syncJoints()` destruye y
  recrea todo ante cualquier cambio de specs/grupos/mundo (barato a estas escalas; 1 frame de latencia al crear).
  Joints anclados al mundo usan un static body oculto sin shape. Los joints entre dos bodies son **pivot-a-pivot** (estilo Bullet): cada
  frame local se deriva del pivote mundial de SU body, y el solver junta ambos pivotes al crear el joint aunque
  los bodies spawneen separados o desalineados en X/Z (antes se usaba el punto medio compartido, que congelaba
  la separación de spawn como offset rígido dentro del constraint). Auto Length de distance = distancia real
  entre pivotes (antes daba 0 entre dos bodies porque ambos anchors caían en el punto medio).
- Instances CHOP: toggle `Extra Channels` agrega `vx vy vz wx wy wz awake` (w en deg/s). Info CHOP del Body SOP
  ahora también expone `vx..wz awake`.
- **Patrón de actores con COMPs nativos** (via `getParObject`, que da la matriz world de cualquier Object COMP —
  el SDK NO permite leer parámetros de otros nodos, así que no se pueden consumir los Actor/Constraint COMP de
  Bullet): Body SOP tiene `Joint` / `Joint Pivot` + `Show Joint Pivot` (pivot local visible en preview); el Joint CHOP solo conecta Body A/B.
  Convención de matriz TD: `[fila][col]`, traslación en
  `[0..2][3]`, ejes en columnas.
- **Set Joint SOP → Transform SOP → Body SOP funciona**: el Set Joint escribe `joint_pivot` en espacio objeto
  (`joint_pivot_space=0`) más una codificación afín-invariante `joint_ref` (int×4: índices de 4 puntos de
  referencia no coplanares; -1 en slots sin usar) + `joint_ref_w` (float×3: coeficientes tal que
  pivot = p0 + a·e1 + b·e2 + c·e3). El Body SOP reconstruye el pivote desde los puntos YA transformados, le
  resta el centroide actual y lo rota al frame de spawn del body (`R(def.q)^T`, el mismo frame de los hull
  points — antes se pasaba sin rotar y el core lo interpretaba en el frame equivocado; también había una
  doble resta de centroide). El pivote local se snapea (1e-4) contra el valor anterior para no disparar
  `jointsDirty` (re-sync total de joints) por ruido float en cada cook con animación upstream.

- **Robustez ante inputs degenerados** (auditoría 2026-07): el core sanitiza en los choke points — spawn
  pose/quat/material no finitos se reemplazan por defaults en `createBodyFromDef`, `setBodyTransformFromDef`,
  `retargetKinematicBodies` y `applyMaterialToBody` (los atributos por punto del Spawn SOP bypassean los clamps
  de parámetros; un NaN de posición envenena el mundo y no sana solo); el preserve de dinámicos al recrear
  grupos NUNCA preserva pose/velocidad no finita (respawnea — antes un body NaN revivía NaN para siempre). El
  Body SOP valida el cache del hull contra la nube viva CADA cook (diagonal del bbox vs `myInputRefDiag`,
  invariante bajo movimiento rígido): un input que colapsó (scale por cero/negativo) y volvió fuerza rebuild
  de referencia y de grupo sin depender del gating por cook-count — antes quedaba pegado hasta reconectar el
  wire. Set Joint no escribe atributos con 0 puntos.

Build:

```
cmake -B build
cmake --build build --config Release
```

box3d se resuelve solo: usa `../box3d` si existe, si no FetchContent pineado
(`BOX3D_GIT_TAG`). Instalar: cerrar TD y correr `install_plugin.bat` (Windows) o
`install_plugin.sh` (macOS).

Gotchas clave: box3d root CMake fuerza /MT y los plugins TD necesitan /MD (por eso se
agrega `box3d/src` directo, nunca su CMakeLists raíz); un operador custom por DLL (límite
de TD); timestep fijo 60 Hz con acumulador; las mallas de colisión son referenciadas (no
copiadas) por box3d — el core maneja el lifetime. En macOS: `Box3DTDCore.h` exporta con
`__attribute__((visibility("default")))` en vez de `__declspec`; los bundles `.plugin`
enlazan `libBox3DCore.dylib` con rpath `@loader_path/../../../` (no el patrón
Contents/Frameworks de Derivative — ver PLAN.md, rompería el registro compartido);
`CMAKE_OSX_DEPLOYMENT_TARGET` fijo en 13.0. Detalle completo en PLAN.md.
