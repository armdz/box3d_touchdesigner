# box3d-touchdesigner

Operadores nativos de TouchDesigner (C++ custom operators, Windows x64 y macOS
arm64/x86_64) que exponen el motor de física **Box3D** de Erin Catto dentro de TD, con UX
estilo Bullet Solver.

**Leer `PLAN.md` antes de trabajar acá**: tiene el contexto completo, las decisiones de
arquitectura (por qué CHOP y no POP, composición sin cables vía registro global, el split
en 3 DLLs), el contrato de atributos del Spawn SOP y el estado fase por fase. `README.md`
es la doc pública en inglés (build, instalación, uso).

Estado: Fases 0–4b completas. Nodos: **Box3D Solver CHOP** (mundo/step/Collision SOP),
**Box3D Body SOP** (un body: hull del input o primitiva; salida = geometría transformada),
**Box3D Instances CHOP** (masas para instancing). Todo sobre `Box3DCore.dll` (registro de
mundos, box3d estático adentro). Pendiente: COMP wrappers .tox, eventos de contacto,
Fase 5 (joints, fuerzas, kinematic targets).

Update 2026-07:

- Solver quedó **world-only** (sin spawn/demo propio; su salida CHOP de transform queda en cero por compatibilidad).
- Se agregó contenedor estático opcional (4 paredes) y parámetros `Workers`, `External Accel`, `Max Steps / Cook`,
  `Allow Sleep` (toggle world-level, aplica en vivo vía `b3World_EnableSleeping`; apagarlo despierta todo — útil si
  una isla quedó dormida y parece "congelada").
- `setGroup()` del core hace updates locales: material se aplica en caliente sobre los shapes existentes; cambios de
  pose mueven solo static/kinematic (los dinámicos solo toman pose de spawn al crearse o en un rebuild); cambios de
  shape/size/type/count recrean solo ese grupo, preservando pose y velocidad simuladas de los dinámicos cuyo spawn
  no cambió (sin reset global del mundo).
- Kinematic updates usan `b3Body_SetTargetTransform(...)` para contactos más estables con dinámicos. `advance()`
  re-targetea los kinematics antes de cada step (box3d nunca frena kinematics solo; sin esto derivan al infinito
  cuando la animación upstream se detiene). El re-target va con `wake=false` para dejar dormir a los kinematics
  quietos, PERO despierta explícitamente al body si está dormido y el target se movió (comparación pos/quat con
  epsilon y signo de cuaternión alineado) — sin eso, un kinematic dormido nunca retomaba la animación upstream y
  su isla entera quedaba congelada. Deltas negativos (scrub hacia atrás) no drenan el acumulador.
- Body SOP intenta seguir animación rígida upstream (Transform SOP) sin rebuild global.
- Body SOP tiene shape **`Mesh (Static)`** (core shape 4): malla de triángulos exacta (cóncavas OK — terrenos,
  tubos), solo para static/kinematic (box3d no soporta meshes en dinámicos; un dinámico con Mesh se fuerza a
  static con warning; para dinámicos cóncavos: hull o composición). Los `b3MeshData*` viven por grupo
  (`Group::meshes`, box3d los referencia sin copiar) y se destruyen junto con los bodies; grupos borrados con
  rebuild pendiente van a `orphanMeshes` hasta el próximo `destroyWorld`. Sin frame-following para meshes:
  geometría animada upstream recrea el grupo por cook (aceptable para estáticos). El `Collision SOP` del Solver
  sigue existiendo para un mesh de mundo único. Todos los meshes se crean con `weldVertices=true` (tol 1e-4):
  la adyacencia de aristas de box3d va por índice compartido y la geo de TD suele traer puntos duplicados
  (Facet/merges) — sin weld quedan grietas de contacto en las aristas por donde los bodies se escapan.
- Toggle `CCD (Bullet)` en Body SOP e Instances CHOP (y atributo por punto `bullet` en el Spawn SOP):
  `b3BodyDef.isBullet` para dinámicos rápidos (CCD contra kinematic/dynamic; contra estáticos ya es automático
  vía `enableContinuous` del mundo, on por default). Cambiarlo recrea el body (está en `sameShapeAndType`).
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
  todos los joints del nodo). El pivote vive en el Body SOP con `Joint` / `Joint Pivot` + `Show Joint Pivot`; el
  joint solo conecta Body A/B. Salida CHOP: `ax..bz`, `active`, un sample por joint (orden: slot-major, serie
  minor). El param `Pairs` (lista de texto `A>B; C>;`) se eliminó a favor de los slots. Hubo un **Box3D Joint
  SOP** equivalente (un joint por nodo, salida = línea entre anchors); se eliminó por redundante — el CHOP es
  superconjunto. Vía alternativa scriptable: el
  Solver tiene página `Joints` con un `Joints DAT` (tabla con header; mismas columnas semánticas: `type`, `bodya`/
  `bodyb`, `idxa`/`idxb`, `anchorx/y/z`, `axisx/y/z`, `hertz`, `damping`, `length`, `minlength`, `maxlength`,
  `lower`/`upper` (grados), `cone` (grados), `motorspeed`, `maxmotortorque`, `collide`). El core registra el opPath
  de cada grupo (los clientes lo re-registran cada cook) y resuelve referencias lazy; `syncJoints()` destruye y
  recrea todo ante cualquier cambio de specs/grupos/mundo (barato a estas escalas; 1 frame de latencia al crear).
  Joints anclados al mundo usan un static body oculto sin shape. Info CHOP del Solver: `joint_count` (vivos) y
  `joint_rows` (specs, DAT + nodos). Los joints entre dos bodies son **pivot-a-pivot** (estilo Bullet): cada
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
