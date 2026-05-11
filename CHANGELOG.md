# Changelog

All notable changes are documented here.

## [0.9.1] — 2026-05-07

### SVJ target: 0.97

The converter now targets SVJ standard **v0.97**. The schema enum no longer
accepts `"0.95"`; valid values are `"0.96"` and `"0.97"`. All existing physics
output is fully backward-compatible — no fields removed or renamed.

### New: KN5 → GLB conversion (`kn5_reader.py`)

A new module, `kn5_reader.py`, parses the Assetto Corsa binary `.kn5` 3-D
model format and exports it as a **GLB** (binary glTF) file in the
**SAE J670** coordinate frame (X-forward, Y-right, Z-down) — the same
frame used for all physics data.

Key details:

- Pure-Python parser; no external C tools required.  Uses `pygltflib>=1.15`
  (added to `requirements.txt`).
- Reads node tree, geometry (positions, normals, UVs, indices), textures,
  and materials.  Supports KN5 versions 5 and 6.
- Coordinate transform matches `ac_to_svj()` exactly:
  `SAE = (AC.Z, AC.X, −AC.Y)`.  Triangle winding is reversed to preserve
  front-face orientation when switching from left-handed (AC) to
  right-handed (SAE J670).
- Node-name heuristic maps AC node names (`BODY`, `WHEEL_LF`, `UPRIGHT_LF`, …)
  to SVJ body IDs (`chassis`, `upright_fl`, …).
- DDS textures are converted to PNG on the fly via Pillow before embedding.
- KN5 materials are mapped to glTF PBR:
  `ksAmbient/ksDiffuse` → `baseColorFactor`, `ksSpecular` → `roughnessFactor`.
- `find_car_kn5()` locates the primary LOD (largest `.kn5` whose stem matches
  the car folder name; falls back to any `.kn5` in the car root).

Public API:

```python
from kn5_reader import parse_kn5, scan_kn5_nodes, map_ac_nodes_to_svj, \
                       kn5_to_glb, find_car_kn5
```

### New: `assets.meshes` + `visual` bindings (SVJ 0.97)

When a KN5 file is detected alongside the car data, `build_svj()` now emits:

- **`assets.meshes`** — manifest block referencing the GLB URI
  (`meshes/<car>.glb`).
- **`chassis.visual`** — `{mesh_ref, node: "SVJ::body::chassis"}`.
- **`suspension.<corner>.visual`** — per-corner upright bindings using
  `SVJ::body::upright_fl/fr/rl/rr` naming.

All fields are optional in the schema; if no KN5 is present, the SVJ is
still fully valid.

### Gradio UI changes

- **Tab 2 (single car)**: new checkbox *"Convert KN5 → GLB (embeds SAE J670
  3D mesh)"*.  When checked, the output file changes from `.svj.json` to a
  `.zip` containing both the SVJ JSON and the `meshes/*.glb` file.
- **Tab 3 (batch)**: new checkbox *"Convert KN5 → GLB (SAE J670 3D mesh per
  car)"*.  When checked, each car's subfolder in the output ZIP gains a
  `meshes/<car>.glb` entry (when a KN5 file is available).

### Other

- `_metadata.version` bumped to `"0.97"`.
- `pygltflib>=1.15` added to `requirements.txt`.
- README: SVJ version reference updated to 0.97.

---

## [0.9.0-beta] — 2026-05-02

Initial public beta release.

### Suspension geometry
- All six AC suspension `TYPE` codes covered: MacPherson, double-wishbone,
  multi-link, solid-axle, trailing-arm, semi-trailing-arm.
- Correct lateral coordinate convention for chassis-side pickups (`WBCAR_*`,
  `LINK_*_CAR`, `PANHARD_CAR`, `WATTS_CAR`, `TRAIL_CAR`, `SEMI_TRAIL_CAR`):
  the AC X component is an inboard-positive offset from the wheel centre —
  the same convention as `WBTYRE_*`. Verified against MX-5 ND (DWB V-shape,
  strut inclination 14.9° on Audi TT Cup, F2004 keel geometry).
- Correct vertical (Z) reference: `chassis_z_offset = -rolling_radius`.

### Powertrain & vehicle
- Real `power.lut` / `coast_curve.lut` parsing.
- Turbos, BOV, engine damage, driver aids (autoblip, autoclutch, ABS, TC).
- Wheelbase from `suspensions.ini [BASIC]`; CG from `CG_LOCATION` fraction.

### Tyres
- Pacejka MF 5.2 fitter per axle/compound with R² / RMSE metrics.
- Multi-compound `[FRONT_N]/[REAR_N]`, thermal blocks, pressure model,
  relaxation length, rolling resistance, camber thrust, speed sensitivity.

### Aero
- `aerodynamics.components[]` per SVJ 0.95; wing AoA-CL/CD LUTs; DRS.

### UI (Gradio)
- Tab 1: upload files. Tab 2: single car directory. Tab 4: Tire Lab.
- Tab 3 (batch): scan folder → tick-list of cars → convert selected.
  Output ZIP named `ac_svj_batch_<date>_conv<ver>_svj<ver>.zip`.

### Quality
- `smoke_test.py` with 62 structural checks including DWB V-shape geometry.
- `examples/mx5_nd_club.svj.json` reference output from real Kunos car data.
