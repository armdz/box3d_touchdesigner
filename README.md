# Box3D for TouchDesigner

Native TouchDesigner custom operators exposing [Box3D](https://box2d.org) — Erin Catto's
3D rigid-body physics engine — inside TD, modeled after the workflow of TD's built-in
Bullet solver: a solver node owns the world, actor nodes contribute bodies, no wiring
between them.

> Agent/contributor context, design decisions and the phase roadmap live in
> [PLAN.md](PLAN.md) (Spanish). Read it before changing this folder.

## Node family

| Node | Family | Role |
|---|---|---|
| **Box3D Solver** (`Box3dsolver`) | CHOP | Owns one world: gravity, sub-steps, optional ground plane, optional static container walls, optional static **Collision SOP** (triangle mesh). Steps the simulation once per frame. It does not spawn bodies; body nodes feed the world. Output channels are compatibility zeros. |
| **Box3D Body** (`Box3dbody`) | SOP | ONE rigid body. Wire geometry in and pick a shape: **Input Hull** (convex hull of the input points), or Box/Sphere/Capsule. Output is the input geometry transformed by the simulation every frame — wire it straight to a render. Body transform also on its Info CHOP. For rigid upstream SOP animation (Translate/Rotate/Transform SOP), the body follows pose updates without rebuilding the whole world. |
| **Box3D Instances** (`Box3dinstances`) | CHOP | A group of bodies for instancing: each point of its Spawn SOP spawns one body (per-point attributes below). Outputs `tx ty tz rx ry rz`, one sample per body — feed Geometry COMP instancing (RX/RY/RZ are degrees, TD rotate order XYZ). |

Body and Instances nodes bind to a solver through their **Solver** path parameter.
Reading the solver creates the cook dependency, so TD always steps the world before the
actors cook — no wires needed. All groups interact in the same world.

### Spawn SOP per-point attributes (all optional, float or int)

| Attribute | Comps | Meaning |
|---|---|---|
| `shape` | 1 | 0=box, 1=sphere, 2=capsule |
| `size` | 1–3 | FULL sizes; box: x,y,z · sphere: x=diameter · capsule: x=diameter, y=total height (Y axis) |
| `density` | 1 | mass density |
| `friction` | 1 | Coulomb friction |
| `restitution` | 1 | bounciness |
| `type` | 1 | 0=static, 1=kinematic, 2=dynamic (default) |
| `orient` | 4 | initial rotation quaternion x y z w |

Missing attributes fall back to the node's default parameters.

Notes:

- `size` is interpreted as full extents/diameter, and current defaults are unit-sized (`1,1,1`).
- Body/Instances updates are automatic. There is no "Reset On Input Change" toggle for body spawn groups.

## Wrapper TOX generation

You can auto-generate basic wrapper `.tox` components from TouchDesigner with:

- Script: `tools/td/generate_tox_wrappers.py`
- Output folder: `tox/`

How to run:

1. Open a `.toe` located inside this repo (for example `samples/test.toe`).
2. Create a Text DAT and paste the script contents, or run it from disk.
3. Run the script (Alt+R in the Text DAT).

Expected output files:

- `tox/Box3D_Solver.tox`
- `tox/Box3D_Body.tox`
- `tox/Box3D_Instances.tox`

These wrappers contain preconfigured CPlusPlus operators pointing to the DLLs in
`plugin/`.

If you run this script while TouchDesigner has an old plugin loaded from a locked path,
close and reopen TD before exporting/using the wrappers.

## Building (Windows x64)

Requirements: Visual Studio 2022 (MSVC), CMake ≥ 3.22, git.

```
cmake -B build
cmake --build build --config Release
```

Box3D sources are resolved automatically: a sibling `../box3d` checkout is used if
present, otherwise CMake fetches the engine from GitHub pinned to a known-good commit
(`BOX3D_GIT_TAG`). You can also point at any checkout with
`-DBOX3D_SOURCE_DIR=<path>`.

Output goes to `plugin/`:

- `Box3DCore.dll` — shared core: the Box3D engine (statically linked inside) plus the
  world registry that the operator DLLs share. Must sit next to the plugin DLLs.
- `Box3DSolverCHOP.dll`, `Box3DBodySOP.dll`, `Box3DBodiesCHOP.dll` — one custom operator
  per DLL (a TouchDesigner constraint).

## Installing

Close TouchDesigner (it locks loaded DLLs), then run:

```
install_plugin.bat
```

This copies every DLL from `plugin/` to
`%USERPROFILE%\Documents\Derivative\Plugins`. Reopen TD and the operators appear in the
OP Create dialog (Custom family).

## Publishing a GitHub release (DLLs)

This repo includes an automated release workflow at `.github/workflows/release.yml`.

How it works:

- Trigger: push a tag that starts with `v` (example: `v0.2.0`).
- CI builds the project in `Release` on `windows-latest`.
- CI validates expected outputs in `plugin/`.
- CI packages DLLs + `install_plugin.bat` + `README.md` into a zip.
- CI publishes a GitHub Release with the zip and a `.sha256` checksum file.

Recommended command sequence:

```bash
git add .
git commit -m "Prepare release v0.2.0"
git tag v0.2.0
git push origin main
git push origin v0.2.0
```

Resulting release artifact name:

- `box3d-touchdesigner-vX.Y.Z-windows-x64.zip`

If you trigger it manually (`workflow_dispatch`), it still builds and uploads an artifact
using `manual-<run_number>` as version suffix.

## Quick start

1. Drop a **Box3D Solver** CHOP and configure world settings (gravity/ground/container/collision).
2. Drop a Box SOP, deform it, wire it into a **Box3D Body** SOP, set its Solver parameter
   to the solver node, Shape = Input Hull, and wire the Body to a Geometry COMP. Play.
3. For crowds: make a Grid SOP, point a **Box3D Instances** CHOP at it (Spawn SOP) and at
   the solver, then instance a Geometry COMP from its channels
   (TX/TY/TZ ← `tx ty tz`, RX/RY/RZ ← `rx ry rz`).

## Behavior updates

- Solver is world-only: no solver-owned demo spawn, no solver-owned actor output.
- Group updates are local when possible:
  - pose-only updates move existing bodies in-place,
  - incompatible body changes (shape/material/count/hull) recreate only that group.
- Kinematic bodies are driven with target transforms (not pure teleports), improving
  contacts against dynamic bodies when externally animated.

## Repo layout

```
repo root
  README.md              this file
  PLAN.md                project context, decisions, roadmap (for humans and AI agents)
  CMakeLists.txt         standalone top-level project (do NOT build via the box3d root)
  common/                Box3DCore shared DLL sources + header-only TD helpers
  Box3DSolverCHOP/       solver operator
  Box3DBodySOP/          single-body operator
  Box3DBodiesCHOP/       instances operator
  sdk/                   TouchDesigner CPlusPlus SDK headers (Derivative Shared Use License)
  install_plugin.bat     copies built DLLs into the TD Plugins folder
  build/                 CMake build dir (gitignored)
  plugin/                built DLLs (gitignored)
```

## Build notes / gotchas

- **MSVC runtime**: box3d's own root CMake forces the static runtime (`/MT`); TD plugin
  DLLs must use the dynamic runtime (`/MD`). That is why this project adds box3d's
  `src/` directory directly and never includes box3d's root CMakeLists.
- **Fixed timestep**: the world steps at a fixed 60 Hz with an accumulator over TD's cook
  delta (max 4 steps per cook), as Box3D recommends.
- **Mesh lifetime**: Box3D references (does not copy) collision mesh data; the core keeps
  it alive and destroys it only after the world.
- Simulation is CPU (Box3D). The Solver `Workers` parameter maps to Box3D worker count.

## Performance tips

- Start with `Sub Steps = 2..4` and increase only if needed.
- Increase `Workers` on CPUs with real performance cores for collision-heavy scenes.
- For large crowds, prefer primitive colliders (box/sphere/capsule) over dense hulls.

## Licenses

- Box3D: MIT (c) Erin Catto — this folder builds it from the repository sources.
- `sdk/` headers: Derivative Inc. Shared Use License — usable only with TouchDesigner.
- Plugin code in this folder: MIT unless noted otherwise.
