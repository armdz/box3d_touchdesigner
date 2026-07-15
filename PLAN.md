# Box3D → TouchDesigner: Plugin nativo

Contexto completo del proyecto de integración. Leer esto antes de tocar este repo.

## Estado actual (2026-07)

Este documento contiene historia por fases y partes ya no reflejan exactamente el runtime actual.
Usar este bloque como fuente de verdad rápida:

- **Solver world-only**: ya no hace spawn/demo propio ni entrega transforms de actores.
  Los cuerpos los aportan `Box3dbody` y `Box3dinstances`.
- **Mundo**: gravedad/substeps, piso opcional, contenedor estático opcional (4 paredes), Collision SOP estática,
  y parámetros `Workers`, `External Accel`, `Max Steps / Cook`.
- **Actualización de grupos**:
  - pose-only: update en caliente,
  - cambios incompatibles (shape/material/count/hull): recreate local del grupo,
  - evitar reset global del mundo salvo cambios de world/collision mesh.
- **Kinematic**: updates de pose con `b3Body_SetTargetTransform(...)` para colisiones más estables con dinámicos.
- **Body SOP**: seguimiento de animación rígida upstream para minimizar rebuild global; mantiene salida SOP transformada.
- **Instances CHOP**: salida `tx ty tz rx ry rz sx sy sz` (escala saneada para evitar 0/negativos en render).
- **Defaults de spawn/instances**: tamaño unitario (`1,1,1`), más defaults de `restitution` y `type`.
- **Fuerzas / atractor (Box3D Force CHOP)**: `ForceField` en el core (attractor/repulsor/wind/vortex, strength/radius/falloff/mass-indep/target group) aplicado a bodies dinámicos antes de cada step; registrado por nodo con heartbeat como joints. **Box3D Force CHOP** (`Box3dforce`): posición por param o un Points SOP (multi-atractor), target all o un grupo por path.
- **Box contenedor (colisión hacia adentro)**: shape 6 del core (`hollow box`) — 6 (o 5) slabs finas con `b3MakeOffsetBoxHull` como shapes múltiples de un body, la cavidad colisiona hacia adentro; expuesto en el Body SOP como `Box (Container, Inward)` con `Wall Thickness` + `Open Top`. Static/kinematic/dynamic.
- **Material presets (Instances)**: `Custom`, `Soft`, `Medium`, `Bouncy` para fallback rápido de material.
- **Ragdoll**: `JointSpec.pivotMode` cableado (0 pivot-a-pivot, 1 pivote de A, 2 pivote de B —
  convención ragdoll: el hueso hijo lleva el ancla —, 3 ancla mundial explícita), expuesto como menú
  `Pivot` + `Anchor` en el Joint CHOP; **Box3D Ragdoll SOP** (`Box3dragdoll`): humanoide completo en un
  nodo — 11 cápsulas + 10 joints con límites anatómicos, salida geometría simulada o puntos con
  `orient`/`scale`/`bone` para instancing.
- **Eventos de contacto**: el core captura begin/end/hit de `b3World_GetContactEvents` por advance;
  **Box3D Contacts CHOP** los emite (un sample por evento, con punto/normal/velocidad e Info DAT con paths),
  Instances CHOP tiene toggle `Contact Channels` (`touching impulse hitspeed` por instancia), el Info CHOP del
  Body SOP expone lo mismo y el Solver ganó `Hit Speed Threshold`.

## Objetivo

Crear operadores nativos de TouchDesigner (C++ custom operators) que expongan el motor de
física **Box3D** (este repo: motor 3D de Erin Catto, C17, MIT) dentro de TD, imitando la UX
del sistema Bullet integrado de TD: un *Solver* que es dueño del mundo y *Actors* que son
los cuerpos.

**Restricción clave del SDK de TD**: no se pueden crear COMPs nativos. Las familias
disponibles son CHOP / SOP / TOP / DAT / POP. La UX estilo Bullet se logra con:

1. Operadores C++ que hacen el trabajo (mundo, bodies, transforms de salida).
2. COMP wrappers (.tox) con parámetros custom para la experiencia de usuario final.
3. Un registro global de mundos dentro de la DLL (singleton) para que varios nodos
   compartan un mundo referenciándolo por nombre.

## Material de referencia (clonado dentro del checkout de box3d en `../box3d/`, no trackeado)

- `CustomOperatorSamples/` — ejemplos oficiales de Derivative por familia (CHOP/SOP/TOP/DAT).
  - El patrón base está en `CHOP/BasicGeneratorCHOP/`: entry points C
    (`FillCHOPPluginInfo`, `CreateCHOPInstance`, `DestroyCHOPInstance`), clase que hereda
    `CHOP_CPlusPlusBase`, parámetros vía `OP_ParameterManager`.
  - Cada ejemplo trae su copia de los headers del SDK (`CHOP_CPlusPlusBase.h`,
    `CPlusPlus_Common.h`). API version CHOP = **9**.
- `SimpleShapesPOP/` — ejemplo oficial de POP (Point Operators, TD 2025+, API POP v1).
  Interesante para una futura salida de geometría/puntos en GPU, no para la fase inicial.

Los headers del SDK usados por nuestro plugin están copiados en `sdk/`
(licencia Shared Use de Derivative — solo usables con TouchDesigner).

## Hechos del SDK de TD que ya verificamos

- Un plugin es una DLL (`MODULE`) con `PREFIX ""`. TD la carga desde
  `Documentos/Derivative/Plugins` (custom op global) o vía el parámetro *Plugin Path* de un
  CPlusPlus CHOP.
- `OP_Inputs::getTimeInfo()` da `OP_TimeInfo` con `deltaMS` / `deltaFrames` / `rate` —
  base para el acumulador de timestep fijo.
- `CHOP_GeneralInfo::cookEveryFrameIfAsked = true` para que la simulación avance cada frame
  mientras alguien use la salida.
- Salida CHOP: `getOutputInfo()` fija `numChannels`/`numSamples`; `execute()` llena
  `output->channels[canal][sample]`. Un sample por body ⇒ compatible con instancing de
  Geometry COMP (canales `tx ty tz qx qy qz qw`, rotación como "Rotate Quat").
- Parámetros: `setupParameters()` con `appendXYZ/appendFloat/appendInt/appendToggle/appendPulse`;
  pulses llegan por `pulsePressed(const char* name, ...)`.
- Los nombres de parámetro empiezan con mayúscula y siguen en minúscula (`Boxcount`).
  Igual el `opType` del nodo (`Box3dsolver`).

## Hechos de Box3D que importan para la integración

- API C pura con handles opacos por valor (`b3WorldId`, `b3BodyId`, `b3ShapeId`,
  `b3JointId`). Patrón: `b3Default*Def()` → ajustar → `b3Create*()`. Ids nulos con
  `B3_IS_NULL(id)` / `b3_nullWorldId` (`include/box3d/id.h`).
- `b3World_Step(world, dt, subSteps)` con **dt fijo** (1/60, 4 substeps recomendado) y
  **bloquea** hasta terminar. En TD: acumulador de tiempo fijo dentro del cook, con tope de
  steps por cook para evitar la espiral de la muerte.
- Threading: `b3WorldDef.workerCount` (>1 activa multithreading con scheduler interno si no
  se dan callbacks `enqueueTask`/`finishTask`). Fase 1 usa workerCount=1.
- Shapes: sphere, capsule, convex hull (`b3CreateHull(points, n, maxVerts)` — copiado al
  crear el shape), mesh de triángulos (`b3CreateMesh`, **solo bodies estáticos**, box3d
  guarda REFERENCIA ⇒ el plugin debe mantener vivo el `b3MeshData*`), height field y
  compound (solo estáticos).
- Joints: spherical, revolute, prismatic, distance, motor, weld, wheel, parallel, filter.
- Eventos por step: `b3World_GetContactEvents/GetBodyEvents/GetSensorEvents/GetJointEvents`
  (datos transitorios, copiar en el cook).
- Kinematic bodies + `b3Body_SetTargetTransform(body, target, dt, wake)` = ideal para
  mover cuerpos desde CHOPs (tracking/mocap).
- Sin up-vector propio; convención Y-up igual que TD. Unidades en metros
  (`b3SetLengthUnitsPerMeter` si hiciera falta otra escala).
- Hasta 128 mundos independientes por proceso.

## Gotcha de build #0: soporte macOS (2026-07)

El proyecto compila también en macOS (arm64/x86_64, universal opcional con
`-DCMAKE_OSX_ARCHITECTURES="arm64;x86_64"`). Puntos verificados en este entorno (build real
con clang, no solo lectura de código):

- `common/Box3DTDCore.h` usaba `__declspec(dllexport/dllimport)` sin guardia `_WIN32` — no
  compila con clang fuera de Windows. Fix: macro cross-platform, `__attribute__((visibility("default")))`
  en no-Windows.
- Los operadores son bundles `.plugin` (`BUNDLE TRUE`, `BUNDLE_EXTENSION "plugin"`, ya
  estaba en el CMakeLists). Cada bundle enlaza `libBox3DCore.dylib` vía `@rpath`.
- **rpath**: el patrón "oficial" de Derivative (embeber la dylib en
  `Contents/Frameworks/` de cada bundle + `@loader_path/../Frameworks`) NO sirve acá:
  cada bundle tendría su PROPIA copia de la dylib → 3 imágenes cargadas distintas → 3
  `Registry` estáticos distintos → se rompe el registro global compartido entre Solver/
  Body/Instances (la razón de ser de `Box3DCore`). Se usa en cambio
  `INSTALL_RPATH "@loader_path/../../../"` + `BUILD_WITH_INSTALL_RPATH TRUE` para que los
  3 bundles y `libBox3DCore.dylib` compartan una sola carpeta (igual que `Box3DCore.dll`
  al lado de los plugins en Windows) y dyld cargue una única instancia.
- **Deployment target**: sin fijar, un Xcode/CLT nuevo compila con el SDK más reciente
  (visto: minos 26.0) y el binario de TD instalado reporta minos 13.0 — un plugin más
  nuevo que TD no cargaría en macOS viejos que TD sigue soportando. Fijado
  `CMAKE_OSX_DEPLOYMENT_TARGET "13.0"` antes del `project()`.
- Carpeta de instalación real (confirmada leyendo la doc offline embebida en
  `TouchDesigner.app/Contents/Resources/tfs/.../Custom_Operators.htm`, y el string
  literal `/Derivative/TouchDesigner099` dentro de `libUT.dylib`):
  `~/Library/Application Support/Derivative/TouchDesigner099/Plugins` — el `099` es fijo,
  no depende de la versión de TD instalada. `install_plugin.sh` la usa.
- box3d ya resuelve SIMD portable en arm64 (fallback fuera de x86 SSE2) sin cambios de
  nuestra parte; compiló limpio en Apple Silicon nativo y en build universal.

## Gotcha de build #1: runtime de MSVC

El `CMakeLists.txt` raíz de box3d fuerza runtime **estático** (`/MT`,
`CMAKE_MSVC_RUNTIME_LIBRARY MultiThreaded...` en la línea ~28). Una DLL de plugin de TD debe
usar `/MD`. Solución usada: `CMakeLists.txt (raíz de este repo)` es un proyecto top-level propio
que fija `CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL"` y hace
`add_subdirectory(../src box3d)` **directo a src/** (se salta el CMakeLists raíz y sus
opciones de samples/sanitizers). box3d queda como lib estática con /MD linkeada dentro de la
DLL del plugin. C17 lo fija el propio src/CMakeLists.txt.

## Decisión: CHOP como columna vertebral, POP pospuesto

Analizamos usar POPs (API v1, TD 2025 experimental, ejemplo en `SimpleShapesPOP/`).
Conclusión: **no para el solver**. Box3D es 100% CPU; un POP custom generador llena
buffers CPU que TD sube a GPU — mismo trabajo que el CHOP con mucho más boilerplate y
una API experimental que cambia entre builds. El volumen de datos (10 floats/body) es
trivial. POP queda como *salida opcional* de Fase 5 (bodies como nube de puntos con
atributos para post-proceso GPU, contactos como puntos), reutilizando el registro de
mundos. Leer inputs POP requiere readback GPU→CPU (ok en spawn, malo por-frame).

## Decisión: composición de escena sin cables (estilo Bullet)

Sin COMPs custom, la forma de agregar bodies de nodos separados NO es cablear
Body→Merge→Solver (los cables CHOP solo llevan canales numéricos; la geometría no pasa).
El patrón elegido, igual que el Bullet real de TD: **registro global de mundos en la DLL +
parámetro path**. Un nodo `Box3dbody` (familia SOP: input = su geometría de colisión,
output = la geometría transformada) referencia al solver con un parámetro CHOP-path; leerlo
vía `getParCHOP` crea la dependencia de cook, así TD steppea el solver antes de cookear los
bodies. Cero cables entre bodies y solver. Esto es Fase 4.

## Contrato de atributos del Spawn SOP (Fase 2)

El solver tiene un parámetro **Spawn SOP** (`getParSOP`, lectura CPU directa, sin
readback). Cada punto = un body. Atributos custom por punto, todos opcionales (float o
int); si faltan, se usan los parámetros de la página Bodies del nodo:

| Atributo | Comps | Significado |
|---|---|---|
| `shape` | 1 | 0=box, 1=sphere, 2=capsule |
| `size` | 1-3 | tamaños TOTALES; box: x,y,z · sphere: x=diámetro · capsule: x=diámetro, y=alto total (eje Y) |
| `size0/size1/size2` | 1 c/u | alias de tamaño split |
| `sizex/sizey/sizez` | 1 c/u | alias de tamaño split |
| `sx/sy/sz` | 1 c/u | alias de tamaño split |
| `density` | 1 | densidad de masa |
| `friction` | 1 | fricción Coulomb |
| `restitution` | 1 | rebote |
| `type` | 1 | 0=static, 1=kinematic, 2=dynamic (default) |
| `orient` | 4 | cuaternión inicial x y z w |
| `rx/ry/rz` | 1 c/u | rotación inicial en grados (fallback cuando no hay `orient`) |

Si `size` existe pero con menos de 3 componentes, y/z heredan de x. Si no existe `size`,
se aceptan aliases split (`size0..2`, `sizex/y/z`, `sx/y/z`) con la misma herencia.
Sin Spawn SOP el nodo cae al modo demo (grilla de cajas, params "Demo *"). Cambios en el SOP
re-crean el mundo si "Reset On Input Change" está activo (detección por `opId` + `totalCooks`).

## Plan por fases

- **Fase 0 — Esqueleto** ✅: repo con CMake propio (box3d /MD + DLL de plugin),
  headers del SDK en `sdk/`.
- **Fase 1 — Box3D Solver CHOP (juguete)** ✅ (esta fase): un CHOP `Box3dsolver` que crea
  suelo estático + N cajas dinámicas. Parámetros: Simulate, Reset (pulse), Gravity (xyz),
  Substeps, Boxcount, Boxsize, Spawnheight, Groundsize. Salida: 6 canales
  `tx ty tz rx ry rz`, un sample por body → instancing (rx/ry/rz en grados,
  orden de rotación XYZ de TD, para los campos RX/RY/RZ del instancing; el quat queda
  disponible para quien lo quiera). Timestep fijo 60 Hz con acumulador sobre `deltaMS`.
  Spawn determinista (grilla con jitter por índice, sin rand).
- **Fase 2 — Bodies definidos por el usuario** ✅: parámetro Spawn SOP en el solver, cada
  punto = un body con los atributos de la tabla de arriba; páginas de parámetros Solver
  (Simulate/Reset/Gravity/Substeps/Ground) y Bodies (Spawn SOP, defaults, demo). Ground
  plane ahora opcional (toggle).
- **Fase 3 — Colisión estática desde SOPs** ✅: parámetro Collision SOP (página Solver).
  Polígonos fan-triangulados → `b3CreateMesh` (allocation autocontenida; el `b3MeshData*`
  vive en el plugin y se destruye DESPUÉS del mundo porque el shape lo referencia).
  `identifyEdges` activado para mejor calidad en bordes. Convivencia con Ground Plane
  (toggle). Cambios del Collision SOP disparan rebuild bajo el mismo "Reset On Input
  Change".
- **Fase 4a — Multi-nodo estilo Bullet** ✅: arquitectura de 3 DLLs (TD permite UN operador
  custom por DLL): `Box3DCore.dll` (box3d estático adentro + `tdb3::Registry` global de
  `SolverCore` keyed por opId del solver + toda la lógica de mundo/grupos/step/rebuild) y
  los plugins `Box3DSolverCHOP.dll` / `Box3DBodiesCHOP.dll` que la comparten (debe ir junto
  a los plugins en la carpeta Plugins). El nodo **Box3D Bodies** referencia al solver con
  un parámetro CHOP-path (`getParCHOP` crea la dependencia de cook ⇒ el solver steppea
  primero), registra su grupo (Spawn SOP + defaults propios) y saca los transforms SOLO de
  sus bodies → instancing por grupo. El solver saca los suyos (Spawn SOP propio o demo grid
  solo cuando está solo). Cualquier cambio de grupo → rebuild completo determinista (orden
  por opId). Grupo nuevo se reporta en pose de spawn hasta el próximo cook del solver
  (1 frame de latencia). Los plugins NO llaman funciones link-level de box3d (no se
  exportan del core); solo math inline de los headers.
  Falta de 4: COMP wrappers .tox. (Eventos de contacto ✅ — ver Fase 6.)
- **Fase 4b — Box3D Body SOP (actor individual)** ✅: nodo `Box3dbody` (familia SOP, API
  v3). UN body por nodo: input SOP opcional = su geometría; Shape = Input Hull (hull
  convexo de los puntos del input, `b3CreateHull` con budget 64 vértices, origen del body
  = centroide, SpawnBody.shape=3 + hullPoints en el core) o Box/Sphere/Capsule (en el
  centroide del input, o en el parámetro Position si no hay input — en ese caso genera un
  mesh de preview de la primitiva). Salida SOP = geometría transformada por la física cada
  frame (sin instancing); el transform también sale por Info CHOP (tx..rz). Params: Solver
  (CHOP path), Body Type (dynamic/kinematic/static), Density/Friction/Restitution, Reset
  On Input Change. El nodo de grupos se renombró a **Box3D Instances** (`Box3dinstances`).
  Familia de nodos actual: Solver CHOP (mundo) · Body SOP (actor individual) · Instances
  CHOP (masas para instancing).
- **Fase 5 — Features**: joints entre actors ✅ (**Box3D Joint CHOP**: uno o varios joints
  por nodo — `Count` para series, `Pairs` para listas explícitas —,
  distance/spherical/revolute/weld, bodies por path/nombre + índice; el pivote se
  define en cada Body SOP con `Joint` / `Joint Pivot` o upstream con el **Set Joint SOP**;
  salida CHOP `ax..bz active`; alternativa scriptable: Joints DAT en el Solver; resync
  automático ante rebuilds — ver CLAUDE.md. El Joint SOP original se eliminó por
  redundante), fuerzas/viento/explosiones desde CHOPs (pendiente),
  kinematic targets ✅ (Box3D uses target transforms internally for kinematic bodies),
  sensores, hulls convexos desde SOPs
  para dinámicos, workerCount > 1 ✅
  (`Workers`), posible salida POP. Canales extra de estado (velocidades/awake) ✅ en
  Instances CHOP (toggle) e Info CHOP del Body SOP. Familia de nodos: Solver CHOP ·
  Body SOP · Instances CHOP · Joint CHOP · Set Joint SOP.
- **Fase 6 — Eventos de contacto** ✅: el core habilita `enableContactEvents` +
  `enableHitEvents` en todos los shapes de grupos (flags OR-eados por par, así que pares
  contra los estáticos del mundo también reportan), etiqueta cada body con
  (groupKey, índice) empaquetado en el userData, y después de cada `b3World_Step` traduce
  `b3World_GetContactEvents` a un buffer de `ContactEvent` {kind begin/end/hit, grupo+índice
  de A y B, punto, normal A→B, approach speed} que vive exactamente un advance (se limpia al
  inicio del siguiente, también en pausa). API: `contactEventCount`/`getContactEvents`,
  `getGroupContactStates` (por body: touching count vía `b3Body_GetContactData`, impulso
  normal sumado, hit speed máximo del último advance), `getGroupPathByKey`,
  `findGroupKeyByPath`. Consumidores: **Box3D Contacts CHOP** (`Box3dcontacts`, DLL nueva:
  un sample por evento, canales `active kind idxa idxb worlda worldb px py pz nx ny nz
  speed`; toggles por tipo de evento; `Body Filter` normaliza el body filtrado al lado A;
  Info DAT con paths de nodos por evento; Info CHOP con counts), toggle **Contact
  Channels** en Instances CHOP (`touching impulse hitspeed` por instancia), Info CHOP del
  Body SOP (idem, canales 13–15) y `contact_events` en el Info CHOP del Solver. El Solver
  ganó `Hit Speed Threshold` (m/s, live via `b3World_SetHitEventThreshold`). Pendiente:
  sensores (`isSensor`/`enableSensorEvents`), filtrado de colisiones (categorías/máscaras).

## Layout del repo

```
box3d-touchdesigner/
  PLAN.md                  ← este documento
  CLAUDE.md                ← estado vivo del proyecto (leer primero)
  CMakeLists.txt           ← proyecto top-level (fija /MD, agrega box3d/src directo)
  sdk/                     ← headers del SDK de TD (CHOP, SOP, comunes)
  common/                  ← Box3DTDCore (DLL core compartida) + helpers header-only
  Box3DSolverCHOP/         ← mundo/step/Collision SOP
  Box3DBodySOP/            ← un body (hull/box/sphere/capsule/mesh/compound)
  Box3DBodiesCHOP/         ← Instances CHOP (grupo de bodies para instancing)
  Box3DJointCHOP/          ← joints (hasta 8 pares + series)
  Box3DSetJointSOP/        ← pivote de joint como atributos en la cadena SOP
  Box3DDebugSOP/           ← debug draw del mundo de colisión
  Box3DContactsCHOP/       ← eventos de colisión (begin/end/hit)
  Box3DRagdollSOP/         ← ragdoll humanoide de un nodo (11 huesos + 10 joints)
  TD-Examples/             ← escenas .toe de ejemplo
  tox/                     ← COMP wrappers (WIP)
  build/                   ← build dir de CMake (no trackear)
  plugin/                  ← DLLs resultantes (copiar a Documentos/Derivative/Plugins)
```

## Cómo compilar

```
cmake -B build
cmake --build build --config Release
```

Resultado: `plugin/Box3DCore.dll` + una DLL por operador. Para instalar ejecutar
`install_plugin.bat` (copia las DLLs a
`%USERPROFILE%/Documents/Derivative/Plugins`; TD debe estar cerrado porque bloquea la DLL
cargada). `build_and_install.bat` hace build + install en un paso.

## Verificación en TD (Fase 1)

1. Colocar el nodo (custom op `Box3dsolver` / Box3D Solver).
2. Conectarlo a un Geometry COMP instanciando una Box SOP: Instance OP = el CHOP,
   TX/TY/TZ = `tx ty tz`, Rotate Quat = `qx qy qz qw`, escala = parámetro Boxsize.
3. Play: las cajas caen y apilan sobre el suelo invisible (plano y=0). Reset las respawnea.
