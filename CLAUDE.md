# box3d-touchdesigner

Operadores nativos de TouchDesigner (C++ custom operators, Windows x64) que exponen el
motor de física **Box3D** de Erin Catto dentro de TD, con UX estilo Bullet Solver.

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
- Se agregó contenedor estático opcional (4 paredes) y parámetros `Workers`, `External Accel`, `Max Steps / Cook`.
- `setGroup()` del core hace updates locales: si cambia solo pose, mueve en caliente; si cambia shape/material/count,
  recrea solo ese grupo (sin reset global del mundo).
- Kinematic updates usan `b3Body_SetTargetTransform(...)` para contactos más estables con dinámicos.
- Body SOP intenta seguir animación rígida upstream (Transform SOP) sin rebuild global.
- Instances CHOP ahora publica `sx sy sz` además de `tx ty tz rx ry rz`, con escala saneada para evitar 0/negativos.
- Spawn SOP acepta aliases de size (`size0..2`, `sizex/y/z`, `sx/y/z`) y `rx/ry/rz` como fallback de orientación inicial.
- Instances agrega presets de material (`Custom`, `Soft`, `Medium`, `Bouncy`) para defaults rápidos.

Build:

```
cmake -B build
cmake --build build --config Release
```

box3d se resuelve solo: usa `../box3d` si existe, si no FetchContent pineado
(`BOX3D_GIT_TAG`). Instalar: cerrar TD y correr `install_plugin.bat`.

Gotchas clave: box3d root CMake fuerza /MT y los plugins TD necesitan /MD (por eso se
agrega `box3d/src` directo, nunca su CMakeLists raíz); un operador custom por DLL (límite
de TD); timestep fijo 60 Hz con acumulador; las mallas de colisión son referenciadas (no
copiadas) por box3d — el core maneja el lifetime.
