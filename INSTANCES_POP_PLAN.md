# Plan — Box3D Instances POP (rigid-body instancing en GPU)

> **ESTADO: IMPLEMENTADO** (`Box3DInstancesPOP/`, opType `Box3dinstancespop`,
> label "Box3D Instances POP", icon `INP`). Registrado en `CMakeLists.txt` (bloque
> POP ungated, como el Skin POP). Build:
> `cmake --build build --config Release --target Box3DInstancesPOP`. Instalar:
> copiar `plugin\Box3DInstancesPOP.dll` a `Documents\Derivative\Plugins\` (cerrar
> TD). Salida (atributos de punto): `P` (float3), **`rot` (float3 = rx ry rz Euler
> XYZ grados, formato TouchDesigner)**, `scale` (float3), `v` (float3 velocidad).
> Input 0 = spawn points; atributos por punto opcionales: `scale`/`size`, `shape`
> (0/1/2), `density`, `friction`, `restitution`, `type`, `alive` (0 static/1
> dynamic, pisa a `type`), `bullet`, `orient` (float4) o `rot` (float3 Euler).
> Notas de diseño abajo (referencia histórica).

Handoff para sesión nueva. Objetivo: una versión **POP** del Box3D Instances (que
hoy es CHOP), para **live performance** — todo GPU/POP, sin round-trip de canales,
que **interactúa con los ragdolls Mixamo** (mismo Solver = mismo mundo box3d).

## Qué hace
- **Input** = spawn points (POP) con posiciones (`P`) y, opcional, atributos por
  punto: `scale`/`size`, `shape` (0 box / 1 esfera / 2 cápsula / 3 hull), `mass`
  (o `density`), `alive` (0 = estático/congelado, 1 = dinámico).
- Registra **un cuerpo rígido por punto** en el Solver (count variable en vivo).
- **Output** = nube de N puntos POP, cada uno en la pose viva del cuerpo, con
  atributos `P` (pos), `orient` (quaternion float4), `scale` (float3), `v`
  (velocidad float3). Se instancia geometría en GPU desde esos puntos.
- Comparte Solver con el Skin CUDA POP / ragdolls → **colisionan gratis** (no hay
  filtrado de colisiones todavía; todos chocan con todos).

## No necesita CUDA
Solo saca N puntos con transforms (barato) — NO hay deform por vértice. Así que es
**CPU + buffers CPU**, se compila por **CMake** como el `Box3DSkinPOP` (ungated, sin
nvcc). Nada de `build_skincuda.bat` acá.

## Cómo construirlo (patrones a copiar)
1. **Estructura POP + registro en el Solver**: copiar de `Box3DSkinPOP/Box3DSkinPOP.cpp`
   (el CPU, no el CUDA):
   - `getParCHOP("Solver")` → `Registry::find(opId)` → `SolverCore*` (mismo patrón).
   - `core->setGroup(myOpId, defs)` en cambio; `core->setGroupPath` + (si hubiera
     joints) heartbeat cada cook. Acá NO hay joints — solo `setGroupPath` de heartbeat.
   - Reset pulse → `removeGroup`.
   - `cookEveryFrame = true`.
2. **Contrato de atributos del spawn** (shape/size/mass/etc.): mirar el
   **Box3D Instances CHOP** = carpeta `Box3DBodiesCHOP/` (`Box3DBodiesCHOP.cpp`) — de
   ahí sale cómo arma `std::vector<tdb3::SpawnBody>` desde puntos (shape, size, density,
   bullet, jointEnabled, etc.). Reusar esa lógica de "punto → SpawnBody".
3. **Leer atributos del input POP**: como en `Box3DSkinCudaPOP` (input 1 spawn points):
   `in->getAttribute(POP_AttributeClass::Point, "scale"/"alive"/..., nullptr)` →
   `getBuffer(CPU)` → leer. `getPointInfo` → count. OJO lifetime del `OP_SmartRef`
   (copiar a vector mientras el ref vive).
4. **Poses vivas**: `core->getGroupStates(myOpId, states, count)` → `tdb3::BodyState`
   `{px py pz, qx qy qz qw, vx vy vz, wx wy wz, awake}` (ver `common/Box3DTDCore.h`).
   Usar pos+quat+vel para los atributos de salida.
5. **Salida POP** (buffers CPU): P (n*3), `orient` (n*4), `scale` (n*3), `v` (n*3), +
   `POP_PointInfo` con numPoints=n. Sin topología (es una nube de puntos para
   instancing). Ver el bloque de `setAttribute`/`setInfoBuffers` en el Skin POP; acá es
   más simple (sin index buffer). `POP_SetBufferInfo sinfo;` y `output->setAttribute(...)`.
6. **alive → tipo de cuerpo**: `alive?2(dynamic):0(static)` en `SpawnBody.type`; flip
   0→1 recrea el cuerpo dinámico en spawn (mismo truco que el ragdoll). Incluir
   scale/alive/offsets en la detección de cambio para el rebuild.
7. **CMakeLists**: agregar `add_library(Box3DInstancesPOP MODULE ...)` ungated (como el
   bloque de `Box3DSkinPOP` en `CMakeLists.txt`, ~línea 130): include `sdk_pop` + `common`
   + su dir, link `Box3DCore`. Build: `cmake --build build --config Release --target
   Box3DInstancesPOP`. Instalar: copiar `plugin\Box3DInstancesPOP.dll` a
   `Documents\Derivative\Plugins\` (cerrar TD si está cargado).

## Escala variable en vivo
El count del input cambia el número de cuerpos: reusar el parcheo por índice de
`setGroup` del core (agregar/sacar puntos al final NO resetea los que ya simulan;
identidad = índice de punto, mantener orden estable). Ver notas de `setGroup` en
`CLAUDE.md`.

## Interacción con los ragdolls
Cero código extra: mismo `Solver` param → mismo mundo → box3d colisiona las cápsulas
del ragdoll contra estos cuerpos rígidos. Se pueden usar **Force CHOP** (viento/atractor)
y **Contacts CHOP** (golpes) también.

## Contexto / memoria
Todo el proyecto del ragdoll texturado + POPs está en la memoria del proyecto
(`memory/mixamo-ragdoll-deform.md`) y documentado en `CLAUDE.md` (sección "Update 2026-07
(Mixamo ragdoll texturado + skinning en POP/GPU)"). Gotchas clave que aplican acá:
- **POP getParDouble**: usar `getParDouble(name)` que devuelve valor, NO `(name,&v)`.
- Lifetime de `OP_SmartRef<POP_Buffer>` al leer atributos de input.
- Referencia del Solver por CHOP param crea la dependencia de cook (steppea antes).

Nodos de referencia: `Box3DSkinPOP` (POP + solver), `Box3DBodiesCHOP` (spawn→SpawnBody),
`Box3DSkinCudaPOP` (lectura de atributos de input POP). Core API: `common/Box3DTDCore.h`.
