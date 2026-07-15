# box3d-touchdesigner

Operadores nativos de TouchDesigner (C++ custom operators, Windows x64 y macOS
arm64/x86_64) que exponen el motor de física **Box3D** de Erin Catto dentro de TD, con UX
estilo Bullet Solver.

**Leer `PLAN.md` antes de trabajar acá**: tiene el contexto completo, las decisiones de
arquitectura (por qué CHOP y no POP, composición sin cables vía registro global, un
operador por DLL sobre un core compartido), el contrato de atributos del Spawn SOP y el
estado fase por fase. `README.md` es la doc pública en inglés (build, instalación, uso).

Estado: Fases 0–6 (joints, eventos de contacto) completas. Nodos: **Box3D Solver CHOP** (mundo/step/Collision
SOP/Allow Sleep), **Box3D Body SOP** (un body: hull/primitiva/Mesh estático/Compound de
hulls; salida = geometría transformada, con overlays de pivot y collider), **Box3D
Instances CHOP** (masas para instancing, count variable en vivo), **Box3D Joint CHOP**
(hasta 8 pares + series, estilo Constant CHOP), **Box3D Set Joint SOP** (pivote como
atributos afín-invariantes en la cadena SOP), **Box3D Debug SOP** (wireframe del mundo de
colisión real + joints, colores por estado), **Box3D Contacts CHOP** (eventos de
colisión: un sample por evento begin/end/hit con punto, normal y velocidad de impacto), **Box3D Force CHOP** (campos de fuerza: atractor/repulsor/viento/vórtice sobre las instancias),
**Box3D Ragdoll SOP** (humanoide articulado completo en un nodo: 11 cápsulas + 10 joints
con límites anatómicos; salida geometría simulada o puntos para instancing).
Todo sobre `Box3DCore.dll` (registro de mundos, box3d estático adentro). Pendiente:
filtrado de colisiones (categorías/máscaras), sensores, COMP wrappers .tox.

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
- El param `Solver` de todos los clientes (Body/Instances/Joint/Debug) tiene default `Box3dsolver1`: un nodo
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
- **UVs en geometría generada** (fix): las primitivas del Body SOP (box/sphere/capsule) y el shape container
  no emitían coordenadas de textura — solo el passthrough de input las tenía. Ahora: box/container se emiten
  como 24 puntos por caja (`appendBoxWithUV`, 6 caras × 4, normal flat + uv 0..1); sphere/capsule llevan uv
  lat-long en su teselación. TexCoord = 3 floats (u,v,w) por capa; `setTexCoord(&tc,1,idx)`.
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

- **Eventos de contacto (Fase 6)**: el core habilita `enableContactEvents`/`enableHitEvents` en todos los shapes
  de grupos (los flags se OR-ean por par ⇒ contra ground/paredes/Collision-mesh también reporta) y etiqueta cada
  body con (groupKey, índice) empaquetado en `b3Body_SetUserData` (índice+1 en los 32 bits bajos para que un ref
  real nunca sea null; userData null = "world"). Tras cada `b3World_Step`, `captureContactEvents()` traduce
  `b3World_GetContactEvents` a `ContactEvent {kind 0 begin/1 end/2 hit, grupo+índice A y B, punto, normal A→B,
  approach speed}`; el buffer vive exactamente un `advance()` (se limpia al inicio del siguiente, aún en pausa,
  para que un evento nunca se reporte dos veces; tope 4096/advance). Begin events sacan punto/normal del manifold
  vivo si sobrevivió al step; end events pueden traer shapes ya destruidos (se validan y caen a "world"). API:
  `contactEventCount`/`getContactEvents`, `getGroupContactStates` (por body: `touching` = contactos con manifold
  activo vía `b3Body_GetContactData`, `impulse` = suma de totalNormalImpulse, `hitSpeed` = máximo del último
  advance), `getGroupPathByKey`, `findGroupKeyByPath`. Consumidores: **Box3D Contacts CHOP** (`Box3dcontacts`,
  DLL nueva, cookEveryFrame — los eventos duran 1 frame): un sample por evento, canales `active kind idxa idxb
  worlda worldb px py pz nx ny nz speed`, toggles Begin/End/Hit, `Body Filter` (path/nombre de un Body SOP o
  Instances CHOP; filtra y normaliza ese body al lado A flipeando la normal, así `idxa` indexa directo sus
  instancias), Info DAT con paths por evento e Info CHOP con counts. Instances CHOP: toggle `Contact Channels`
  agrega `touching impulse hitspeed` por instancia (después del bloque activo, o sea el offset depende de Extra
  Channels). Info CHOP del Body SOP: canales 13–15 idem. Solver: param `Hit Speed Threshold` (m/s, live vía
  `b3World_SetHitEventThreshold`, default 1.0 — bajar para hits más sensibles) y canal info `contact_events`.
  Uso típico de visualización: hitspeed como trigger de flash/audio (dura 1 frame), el Contacts CHOP como fuente
  de partículas/decals en el punto de impacto (px..pz + normal), `touching` para tintar instancias en reposo.

- **Ragdoll (pivotMode + Ragdoll SOP)**: `JointSpec.pivotMode` ahora se respeta en `createJointFromSpec`:
  0 = pivot-a-pivot (comportamiento histórico), 1 = el pivote de A ancla ambos, 2 = el pivote de B ancla
  ambos (convención ragdoll: el hueso HIJO lleva el ancla y el pivote del padre no se consulta — así un
  body con dos articulaciones (muslo: cadera+rodilla) funciona con UN pivote), 3 = ancla mundial explícita
  (`anchorX/Y/Z`). En el Joint CHOP: menú `Pivot` (each/bodya/bodyb/anchor) + param `Anchor` (se habilita
  solo en modo anchor), página Joint. **Box3D Ragdoll SOP** (`Box3dragdoll`, DLL nueva): humanoide de un
  nodo — tabla fija de 11 huesos cápsula (pelvis, spine, head, uarm/farm L+R, thigh/shin L+R; proporciones
  × param `Height`, grosor × `Bulk`) en T-pose mirando +Z, registrados como UN grupo (`setGroup`) y 10
  `JointSpec` propios (`setJointNodeList` con el mismo opId; specs referencian el opPath propio con índices
  de hueso, pivotMode=2): esféricos con cono+twist (columna 20°, cuello 40°, hombros 85°, caderas 60°) y
  revolutas con límites (codos/rodillas -2°..150°, ejes espejados por lado; rodillas eje +X ⇒ el talón va
  hacia atrás). `Stiffness`/`Damping` = spring hertz en todos los joints (tono muscular; 0 = flojo),
  `Anatomical Limits` toggle, `collideConnected` false. Pivotes locales invariantes al yaw global
  (`Rotate Y` rota centros, quats y ejes de joints). Salida: menú `Output` — Capsules (malla lat-long por
  hueso transformada por la sim, lista para render) o Points (un punto por hueso + atributos `orient`
  float4, `scale` float3, `bone` int para instancing desde SOP). Position/Rotate solo aplican al spawn
  (los dinámicos ignoran cambios de pose en vivo) — Reset respawnea. Mismo patrón cliente que Body SOP:
  cookEveryFrame + cookOnStart + heartbeats (`setGroupPath` y `setJointNodeList` cada cook).
  **Info DAT para retargeting a Mixamo/3D**: expone tabla (header + 11 filas) `name tx ty tz rx ry rz`
  (rotación euler XYZ en grados, convención TD vía `boneQuatToEulerXYZ` local = misma que `quatToEuler
  XYZDegrees` del Instances CHOP; local para no arrastrar headers de CHOP a un SOP)
  con la pose mundial viva por hueso, cacheada cada cook en `myBoneXforms` (desde `getGroupTransforms`)
  porque el DAT se consulta fuera de `execute`. Param `Bone Naming` (box3d / **mixamo**): en modo mixamo
  la columna name usa `mixamorig:Hips/Spine/Head/LeftArm/LeftForeArm/RightArm/RightForeArm/LeftUpLeg/LeftLeg/
  RightUpLeg/RightLeg` (índice-matched a `kBones`), para mapear por nombre a un esqueleto Mixamo importado.
  Los SOP no llevan atributos string por punto ⇒ los nombres van por el DAT (las transforms también quedan
  como atributos de punto `orient`/`P` en modo Points).

- **Box contenedor / colisión hacia adentro (shape 6)**: nuevo core shape `6` (hollow box) en `createBodyFromDef`:
  seis (o cinco si `openTop`) slabs finas construidas con `b3MakeOffsetBoxHull(whx,why,whz,offset)` como shapes
  MÚLTIPLES de un mismo body (como el compound), insetadas contra la superficie externa (offset = h - ht por cara).
  Como cada slab es un convexo sólido, un objeto DENTRO de la cavidad colisiona contra las caras internas y queda
  contenido, mientras el conjunto puede ser static/kinematic/DINÁMICO (una caja móvil que lleva su contenido) —
  a diferencia del Container world-level del Solver (4 paredes, estático, axis-aligned). `SpawnBody` ganó
  `wallThickness` (default 0.1, clamp a 0.98× del semilado para que no se crucen) y `openTop` (deja abierta la
  cara +Y para tirar cosas adentro); ambos en `sameShapeAndType`. Expuesto en el **Body SOP** como
  `Box (Container, Inward)` (menú shape índice 6 → core 6 vía `menuShapeToCoreShape`) con params `Wall Thickness`
  + `Open Top` (auto-grisados salvo shape container). `outputPrimitiveMesh` dibuja las 6/5 paredes reales (helper
  `emitBox` con base-index de `getNumPoints`) así se ve la caja y su hueco; Show Collision y Debug SOP ya las
  muestran (leen shapes vivas). Con input, el collider sigue siendo el hollow box al centroide (como box/sphere/
  capsule); sin input genera la geometría. Disponible también por atributo de punto `shape=6` en el Spawn SOP
  (Instances), con wallThickness/openTop en sus defaults.

- **Fuerzas / atractor (ForceField + Box3D Force CHOP)**: `ForceField` en el core {type 0 attractor/1 repulsor/
  2 wind/3 vortex, px/py/pz punto, dx/dy/dz dir o eje, strength, radius (0=∞), falloff (0 none/1 linear/2 inv²),
  useMass, targetGroup}. `setForceNodeList(ownerKey, fields)`/`removeForceNode` — mismo heartbeat que joints
  (forceTouch, se remueve a los ~4 advances sin cook ⇒ bypassear apaga la fuerza). `applyForceFields()` corre
  en el loop de step ANTES de cada `b3World_Step` (box3d limpia fuerzas acumuladas post-step ⇒ hay que
  re-aplicar cada step); para cada field itera los bodies del targetGroup (o todos los grupos), solo dinámicos,
  y `b3Body_ApplyForceToCenter`. Por default trata strength como ACELERACIÓN (F = masa × a) ⇒ todos los bodies
  se mueven igual sin importar la masa (como gravedad); `useMass` (toggle "Mass Independent" OFF) lo aplica como
  fuerza cruda. Attractor tira al punto, repulsor empuja, wind es constante e ignora posición/radio, vortex es
  tangente al eje (cross(axis, radial) normalizado). Sanea NaN antes de aplicar. Los ForceField NO tienen
  recursos box3d ⇒ sin cleanup en destroyWorld; resuelven bodies por lookup de grupo al aplicar (grupo ausente
  = skip). **Box3D Force CHOP** (`Box3dforce`, DLL nueva): Type, Position (o **Points SOP** = un field por punto,
  multi-atractor), Direction/Axis, Strength (negativo invierte attract/repel), Radius, Falloff, Mass Independent,
  Bodies (path/nombre → `findGroupKeyByPath`, vacío = todos). Salida: un sample por field con tx ty tz strength.
  cookEveryFrame + heartbeat. Cierra el pendiente "Force CHOP".

- **Update de springs en vivo (setJointNodeList)**: `sameJointStructure(a,b)` = igual que `sameJointSpec` pero
  ignora hertz/dampingRatio/length y exige paridad de spring-enabled (`(a.hertz>0)==(b.hertz>0)`). Cuando la
  lista nueva difiere de la actual SOLO en esos params de resorte (misma estructura, mismos ids vivos, tipos
  distance/spherical/revolute), `setJointNodeList` llama `applyLiveSpringParams` (`b3DistanceJoint_SetSpringHertz/
  SetSpringDampingRatio/SetLength`, y equivalentes spherical/revolute) sobre los joints vivos y actualiza el spec
  guardado — SIN `jointsDirty`/`syncJoints` (que destruye y recrea todo). Esto es lo que evita que arrastrar
  los sliders de spring de la página Dynamics del Joint CHOP recreen todos los joints por cook (y sirve para
  cualquier cliente que actualice muchos resortes por frame). Cruzar el umbral hertz 0↔>0 sí recrea (cambia
  enableSpring). Weld cae a recreate.

Update 2026-07 (Mixamo ragdoll texturado + skinning en POP/GPU):

- **Box3D Mixamo Ragdoll SOP** (`Box3dmixamoragdoll`, `Box3DMixamoRagdollSOP/`): ragdoll de la
  tabla de esqueleto **horneada de `samples/Ch36_nonPBR.fbx`** (rig Mixamo de 175 cm, prefijo
  `mixamorig1:`) — 13 cápsulas + 12 joints anatómicos, más un **retarget rígido de los 65 huesos
  Mixamo** (posición Y rotación de bind pose horneadas; brazos/piernas/manos NO son identidad).
  Info DAT `name tx ty tz rx ry rz` con nombres `mixamorig1:*` (param `Bone Prefix`) para manejar
  un esqueleto importado. **Además tiene un INPUT opcional de mesh**: auto-skinea cada vértice al
  cuerpo más cercano y saca el mesh deformado (auto-fit de escala; sale en espacio físico,
  alineado con la colisión — mover con `Position`). `Count`/`Spacing` para N instancias en grilla.
- **LÍMITE del SOP con texturas**: el mesh Mixamo capturado (`pCaptPath[65]`, `pCaptData`, punto
  `pCapt`) trae `uv`/`N` como atributos de **vértice en un Vertex Buffer Object** que la API C++
  de SOP (`getTextures`/`getNormals`) **NO expone** → el deform en SOP pierde la textura. No hay
  Convert/Facet que lo destrabe. Por eso el camino texturado es por **POP** (leen atributos de
  vértice en GPU).
- **Box3D Skin GPU POP** (`Box3dskin`, `Box3DSkinPOP/`): es el ragdoll Mixamo pero como POP que
  **deforma el mesh preservando las UVs**. Param `Solver`, input 0 = mesh, input 1 (opc) = spawn
  points (una instancia por punto; si no, `Count`/`Spacing`). Reusa la física del header
  compartido **`common/Box3DMixamoRig.h`** (`build`/`bindSkin`/`deform`). Forwardea `uv` (busca
  `uv`/`Tex`/`map1`, punto o vértice) + `N` + topología COMPLETA (triángulos **y** quads; copiar
  todo el `POP_TopologyInfo` + índice, si no los quads quedan como huecos). Instancing: n·N puntos,
  atributos de vértice replicados en el **mismo orden reordenado** que el index buffer (si no, se
  rompen las texturas). Deform en **CPU** (~40 instancias @ 1 substep). Gotcha: en POP usar
  `getParDouble(name)` que devuelve valor, NO la sobrecarga `(name,&v)` (no puebla).
- **Box3D Skin CUDA POP** (`Box3dskincuda`, `Box3DSkinCudaPOP/`): idéntico pero el deform LBS corre
  en un **kernel CUDA** (`SkinCuda.cu`), escala a muchísimas más instancias. El POP CPU queda como
  fallback sin CUDA. **Se compila con `build_skincuda.bat` (nvcc directo, como el ClothPOP)** — NO
  por CMake (falta la integración CUDA de VS y Program Files pide admin); enlaza
  `build\Release\Box3DCore.lib`, así que correr `cmake --build build --config Release` primero.
- Normales durante el movimiento: los POPs forwardean las originales; recalcular con un **Normal
  POP aguas abajo** (más correcto que rotarlas). Ver la memoria del proyecto para el detalle de
  todo el camino y decisiones.

- **Box3D Instances POP** (`Box3dinstancespop`, `Box3DInstancesPOP/`): la versión **POP** del
  Instances CHOP para live performance (todo GPU/POP, sin round-trip de canales; comparte Solver =
  mismo mundo box3d ⇒ colisiona gratis con los ragdolls Mixamo / Skin POP). Input 0 = spawn points
  (un cuerpo rígido por punto, count variable en vivo); salida = nube de N puntos con atributos de
  punto `P` (pos), **`rot` (float3 = rx ry rz Euler XYZ grados, formato TouchDesigner — NO
  quaternion)**, `scale` (float3, saneada y espejada según shape como el CHOP: esfera uniforme,
  cápsula redonda en XZ) y `v` (float3 velocidad, qualifier Direction). Atributos por punto del
  input (todos opcionales, pisan los defaults del nodo): `scale`/`size`, `shape` (0 box/1 sphere/2
  capsule), `density`, `friction`, `restitution`, `type` (0/1/2), `alive` (0 static/1 dynamic —
  pisa a `type`), `bullet` (CCD), `orient` (float4 quat) o `rot` (float3 Euler XYZ deg). Params:
  Solver (default `Box3dsolver1`), Reset, Default Shape/Size/Density/Friction/Restitution/Type,
  CCD. Mismo patrón cliente que el Skin POP: `cookEveryFrame` + `cookOnStart` + heartbeat
  (`setGroupPath` cada cook ⇒ bypassear lo saca del mundo en ~4 advances). Detección de cambio de
  grupo por `defaults != last || input.opId != last || input.totalCooks != last` (reusa el parcheo
  por índice de `setGroup`: agregar/sacar puntos al final NO resetea lo que ya simula, identidad =
  índice de punto). Sin CUDA (solo saca transforms, no deform por vértice) ⇒ compila por **CMake**
  ungated como el Skin POP, NO por `build_skincuda.bat`. Conversiones euler↔quat locales en el cpp
  (mismo álgebra/rotate-order que `tdb3::quatToEulerXYZDegrees` — el POP no puede incluir
  `TDB3Common.h` porque arrastra el header CHOP, que no está en `sdk_pop`). Shapes hull/mesh/compound
  (3/4/5) no soportados acá (necesitan hullPoints por punto, que una nube POP no trae).

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
