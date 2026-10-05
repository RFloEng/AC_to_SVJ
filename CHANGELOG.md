# Changelog

All notable changes are documented here.

## [Unreleased]

### KN5 → GLB exporter (`kn5_reader.py`)

- **Variants** — runtime-variant meshes are now *dropped* (subtree and all)
  instead of being made transparent. Rules are exact: a node is a blur/damage
  variant when a name token is exactly `blur` or `damage` (so `RIM_BLUR_LF`,
  `BODY_DAMAGE` go; `UNDAMAGED_PANEL`, `BENTLEY_BADGE` stay). The low-res
  half of an in-file LOD pair (`COCKPIT_LR`, `STEER_LR`) is dropped **only when
  its `_HR` twin exists**, so `_LR` = Left Rear (`WHEEL_LR`, `SUSP_LR`, …) is
  never touched. New `keep_variants` option on `kn5_to_glb` /
  `kn5_all_lods_to_glbs` and `--keep-variants` on the `kn5_reader.py` CLI.
  Ideas adapted from semiloker/assetto-corsa-gltf (MIT) — see
  `THIRD_PARTY_NOTICES.md`.
- **Livery paint** — the GLB now opens in the colour the game would show. A
  flat `txDetail` map (with `useDetail` set) is the paint on many Kunos cars:
  AC multiplies it into a shared grey diffuse as `diffuse * detail * 2`, so it
  is baked into `baseColorFactor` (doubled in gamma space, clamped, then
  linearised). Base materials resolve against AC's default skin (the first;
  `default_skin="first"|<name>|"none"`), and every skin is still a
  `KHR_materials_variants` entry, now including skins that differ only by
  detail colour. A flat detail map is no longer also exported as AO; patterned
  detail maps keep the `TEXCOORD_1` AO path.
  `list_skins(kn5)` / `python kn5_reader.py car.kn5 --list-skins` print each
  skin and the colour it paints (bodywork first, rims/glass ranked after).
  `kn5_reader.py` CLI is now argparse-based (`--skin`, `--no-skins`,
  `--scan-nodes`, `--keep-variants`, `--list-skins`).
- **Materials** — roughness now comes from the Blinn exponent and intensity
  (`sqrt(2 / (ksSpecularEXP * ksSpecular + 2))`, clamped) instead of
  `1 - ksSpecular`. `fresnelMaxLevel` maps to `KHR_materials_specular`
  (`specularFactor`); materials with `sunSpecular` (car paint) get
  `KHR_materials_clearcoat` from `sunSpecular` / `sunSpecularEXP`. Not done:
  per-pixel roughness from the `txMaps` texture.
- **Robustness** — texture-table entries that differ only in case are folded
  (largest blob wins) and material slots re-pointed, so exports no longer name
  an image that exists only under another capitalisation. Car folder, `skins/`,
  skin texture and LOD file lookups are case-insensitive (Linux/macOS).
  Meshes that use vertex index 65535 (more than 65,535 vertices) are written
  with `UNSIGNED_INT` indices — 65535 is the reserved restart value for
  `UNSIGNED_SHORT` and made such files invalid.
- New `test_kn5_export.py` (synthetic KN5 encoder; no AC content), run first by
  `smoke_test.py`.

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

### Desktop GUI (tkinter)

- Standalone two-tab tkinter application (`gui.py`); no browser required.
- **Batch tab**: scan a `cars/` folder, tick-list selection, **▶ Convert
  selected** and **■ Stop** button, colour-coded log pane with **Save log**
  export.
- Output written to a folder (not a ZIP): one subfolder per car with
  `{model}.svj.json`, `conversion_log.txt`, Pacejka PNG plots, and
  `meshes/{model}.glb` when KN5→GLB is enabled.
- New checkbox *"Convert KN5 → GLB"* in the Batch tab.
- **Tire Lab tab**: three input modes (car folder, bare `tyres.ini`, manual
  parameters); displays five MF 5.2 / MF 6.2 plots plus copy-ready JSON
  blocks.

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

### UI (tkinter)
- Standalone two-tab tkinter desktop app (`gui.py`).
- **Batch tab**: scan folder → tick-list of cars → convert selected.
  Output folder named `ac_svj_batch_<date>_conv<ver>_svj<ver>/`.
- **Tire Lab tab**: standalone MF 5.2 / MF 6.2 bench.

### Quality
- `smoke_test.py` with 62 structural checks including DWB V-shape geometry.
- `examples/mx5_nd_club.svj.json` reference output from real Kunos car data.
