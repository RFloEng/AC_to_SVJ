# SVJ → AC Converter  ·  v0.1.0

> **Part of [AC_to_SVJ](https://github.com/RFloEng/AC_to_SVJ)**, in this folder (`svj_to_ac/`);
> it started as the separate repository `RFloEng/SVJ_to_AC`. It reads the AC modules one
> folder up (`ac_parsers`, `tire_lab`, ...) for its tests.

Inverse of [AC_to_SVJ](https://github.com/RFloEng/AC_to_SVJ). Takes an
[SVJ v0.95 standard vehicle JSON](https://github.com/RFloEng/SVJ-standard-vehicle-json)
-- SAE J670 axes, SI units -- and produces a complete Assetto Corsa
`content/cars/<model>/` folder that Content Manager can install, plus an
optional FBX skeleton that KsEditor can compile to `.kn5`.

## Install

```bash
pip install -r requirements.txt
python svj_to_ac.py
```

Opens the Gradio UI at `http://127.0.0.1:7860`.

## Four tabs

**1 - Upload SVJ** -- drop a single `.svj.json`, get back a Content
Manager-ready zip.

**2 - Single SVJ** -- point at an SVJ file on disk; useful while
iterating on a hand-edited fixture.

**3 - Batch SVJ folder** -- point at a folder of `.svj.json` files,
get a single zip named `svj_ac_batch_<YYYYMMDD>_conv0_1.zip` containing
one `content/cars/<model>/` per car plus a `batch_summary.csv`.

**4 - FBX Lab** -- generate a Z-up, cm-units ASCII FBX 2014 skeleton
from SVJ dimensions (or manually entered wheelbase / track / overall
L*W*H / tyre width), with the dummies KsEditor needs (`WHEEL_LF/RF/LR/RR`,
`SUSP_LF/RF/LR/RR`, `COCKPIT_HR`, `STEER_HR`, `BODY` cube, four wheel
cylinders).

## Coordinate system

Forward (per AC_to_SVJ): `svj = (ac.z, ac.x, -ac.y)`.
Inverse (this project): `ac = (svj.y, -svj.z, svj.x)`.

All suspension pickup keys share an *inboard-positive from the wheel
centre* convention. When writing AC: `ac_x = half_track - abs(svj_lateral)`.
Vertical shift: `chassis_z_offset = -rolling_radius`.

Implemented once in `coords.py`. Enforced by a 100-point random-cloud
round-trip test (max error < 1e-9).

## Suspension type coverage

| AC `TYPE` | SVJ `system_type` | Pickup keys emitted |
|---|---|---|
| 0 / STRUT | `macpherson` | `WBCAR_BOTTOM_*`, `WBTYRE_BOTTOM_*`, `STRUT_CAR/TYRE` |
| 1 / DWB | `double_wishbone` | upper + lower wishbone, tie rod |
| 2 / MULTILINK | `multi_link` | upper/lower lateral + toe link |
| 3 / AXLE | `solid_axle` | trailing links + Panhard **xor** Watts |
| 4 / TRAILING_ARM | `trailing_arm` | `TRAIL_CAR/TYRE` |
| 5 / SEMI_TRAILING | `semi_trailing_arm` | `SEMI_TRAIL_CAR/TYRE` |

## Fit quality (round-trip via AC_to_SVJ parsers)

| File | Round-trip path | Result |
|---|---|---|
| `car.ini` | `_ac_native` pass-through | dict-equal |
| `suspensions.ini` (DWB + multi-link) | `_ac_native` | per-axle `parse_suspension_axle` equal |
| `tyres.ini` | `_source_params` fast path | byte-faithful |
| `engine.ini` + `power.lut` | `_ac_native` + LUT cache | dict-equal + `parse_lut` equal |
| `drivetrain.ini` | `_ac_native` | `parse_drivetrain` semantic-equal |
| `aero.ini` | `_ac_native` | `parse_aero` semantic-equal |
| `brakes.ini` | `_ac_native` | `parse_brakes` semantic-equal |
| `setup.ini` | `_ac_native` | dict-equal |
| `ctrl_abs.ini` / `ctrl_tc.ini` | `_ac_native` | dict-equal |
| `ui_car.json` | canonical SVJ | Kunos required keys present |

Pickup-point accuracy on `mx5_nd_club.svj.json` (full SVJ -> AC math
through `coords.py`, no `_ac_native` shortcut): every front-axle pickup
matches its independently-computed reference to **< 1 mm**.

Reproducible zip: two consecutive `pack_car.convert()` runs produce a
**byte-identical** archive (SHA-256 equal).

## Non-AC SVJ support

The converter works on any SVJ v0.95 source, not just files produced
by AC_to_SVJ. SVJs without an `x_assettocorsa` extension are read
through their canonical SVJ v0.95 per-corner blocks
(`tests/test_non_ac_source.py` enforces this end-to-end):

| SVJ canonical field | AC output |
|---|---|
| `suspension.<corner>.spring.rate` | `[FRONT/REAR].SPRING_RATE` |
| `suspension.<corner>.spring.rod_length` | `[FRONT/REAR].ROD_LENGTH` |
| `suspension.<corner>.damper.bump_curve` | `[FRONT/REAR].BUMP` (median force/velocity) |
| `suspension.<corner>.damper.rebound_curve` | `[FRONT/REAR].REBOUND` (same reduction) |
| `suspension.<corner>.damper.bumpstops.up/down` | `[FRONT/REAR].BUMPSTOP_UP / BUMPSTOP_DN` |
| `suspension.<corner>.arb.bar_rate` | `[FRONT/REAR].ARB` |
| `suspension.<corner>.alignment.camber` (rad) | `[FRONT/REAR].CAMBER` |
| `suspension.<corner>.alignment.toe` (rad) | `[FRONT/REAR].TOE_OUT` |
| `suspension.<corner>.brake.max_torque_nm` | `[FRONT/REAR].MAX_TORQUE` in brakes.ini |
| `suspension.<corner>.brake.disc.outer_diameter` | `[FRONT/REAR].DISC_RADIUS` in brakes.ini |
| `suspension.<corner>.wheel.mass` | `[FRONT/REAR].WHEEL_MASS` |
| `suspension.<corner>.wheel.rim_diameter` | `[FRONT/REAR].RIM_RADIUS` |
| `suspension.<corner>.static_setup.ride_height` | `[FRONT/REAR].BASEY` (magnitude) |
| `aerodynamics.components[].position` (SVJ axes) | `[WING_N].POSITION` via coords transform |
| `aerodynamics.components[].position_ac` | `[WING_N].POSITION` verbatim |

What still requires `x_assettocorsa` data (no canonical SVJ home in
v0.95):

- `setup.ini` adjustable ranges (the in-game garage will be empty).
- `electronics.ini` and `ctrl_*.ini` (ABS / TC / ESP, turbo controllers).
- `car.ini` cosmetic blocks: `[CONTROLS]`, `[PIT_STOP]`, `[TAGS]`,
  `[FUELTANK].POSITION`. Functional `[BASIC]` fields (mass, wheelbase,
  CG height) still work from canonical SVJ.
- `[DRS]` aero block.

A non-AC SVJ that carries the full canonical schema produces a
**drivable** car: correct mass, springs, dampers, ARB, alignment,
brakes, geometry, and tyres via Pacejka inverse. Cosmetic and sim-aid
fields default sensibly and are easy to tune in Content Manager.

## Files

| File | Purpose |
|---|---|
| `svj_to_ac.py` | Gradio UI + orchestrator |
| `svj_loader.py` | SVJ JSON loader + soft validator + dataclass tree |
| `coords.py` | SVJ <-> AC axis transforms (single source) |
| `units.py` | SI <-> AC unit conversions (single source) |
| `ac_writers/` | One writer per AC `.ini` file + shared formatting primitives |
| `lut_writer.py` | Two-column AC LUT writer |
| `pacejka_inverse.py` | MF 5.2 -> AC tyre parameter fitter |
| `fbx_skeleton.py` | Optional ASCII FBX 2014 generator |
| `pack_car.py` | Folder assembler + reproducible zipper |
| `smoke_test.py` | Single-file regression suite (53 checks) |
| `tests/` | Pytest suite (181 tests across 11 files) |
| `examples/` | `mx5_nd_club.svj.json` canonical fixture |
| `DEVELOPMENT.md` / `ROADMAP.md` / `CHANGELOG.md` | Docs |

## What this project does not do

- Write `.kn5` binaries directly. KsEditor must compile the FBX.
- Encrypt `data/` back into `data.acd`. Open the car in Content
  Manager -> Tools -> Pack Data if you want a shipped encrypted build.
- Generate skins, sound banks, driver models, or LODs beyond `LOD_0`.

## License

[MIT](LICENSE) -- https://github.com/RFloEng/SVJ_to_AC

Intended for use with AC car data you have legally obtained.
