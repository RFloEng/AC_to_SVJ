# SVJ_to_AC — Project specification

Self-contained spec. Reading this plus `05_PITFALLS.md` is enough to build the project without internet access, provided you also have a local clone of `AC_to_SVJ` for the parser ground truth.

## 1. Goal

Inverse of AC_to_SVJ. Input: one SVJ v0.95 JSON. Output: a Content Manager-installable `content/cars/<model>/` folder, with the option to also produce an FBX skeleton that KsEditor can compile to kn5.

## 2. Project layout

```
SVJ_to_AC/
├── svj_to_ac.py           # Gradio UI + orchestrator (entry point)
├── svj_loader.py          # JSON load + schema validation
├── coords.py              # SVJ ⇄ AC axis transform (one place only)
├── units.py               # SI ⇄ AC unit conversion
├── ac_writers/
│   ├── __init__.py
│   ├── car_ini.py
│   ├── suspensions_ini.py
│   ├── tyres_ini.py
│   ├── engine_ini.py
│   ├── drivetrain_ini.py
│   ├── aero_ini.py
│   ├── brakes_ini.py
│   ├── electronics_ini.py
│   ├── setup_ini.py
│   ├── ctrl_files.py       # all ctrl_*.ini + matching LUTs
│   └── ui_car_json.py
├── lut_writer.py          # AC two-column LUT format
├── pacejka_inverse.py     # MF 5.2 → AC tyre params (uses _ac_native if present)
├── fbx_skeleton.py        # Optional: minimal FBX from SVJ dimensions
├── pack_car.py            # Assemble content/cars/<model>/ folder, zip it
├── smoke_test.py          # pytest regression suite
├── test_car/              # Synthetic fixture mirroring AC_to_SVJ/test_car/
├── examples/              # Sample SVJ files for batch testing
├── requirements.txt
├── README.md
├── DEVELOPMENT.md
└── LICENSE                # MIT
```

## 3. Inputs

- **SVJ v0.95 JSON** as defined by https://github.com/RFloEng/SVJ-standard-vehicle-json.
- Optionally an `_ac_native` extension block carrying the original AC tyre parameters that AC_to_SVJ saw (proposal in stage 4: AC_to_SVJ should write this block; if absent, SVJ_to_AC falls back to a Pacejka-coefficient → AC-parameter fitter).

## 4. Outputs

```
content/cars/<model>/
├── data/
│   ├── car.ini
│   ├── suspensions.ini
│   ├── tyres.ini
│   ├── engine.ini
│   ├── power.lut
│   ├── drivetrain.ini
│   ├── aero.ini
│   ├── brakes.ini
│   ├── electronics.ini
│   ├── setup.ini
│   ├── ctrl_turbo0.ini    # plus matching .lut files
│   ├── ctrl_abs.ini
│   └── ctrl_*.ini ...
├── ui/
│   ├── ui_car.json
│   ├── badge.png          # 64×64 placeholder
│   └── preview.jpg        # 1022×575 placeholder
├── sfx/
│   └── GUIDs.txt          # stub pointing at a Kunos default
├── <model>.fbx            # optional, stage 7
└── (kn5 produced by user via KsEditor)
```

Final deliverable from CLI/UI: a `<model>.zip` of the `content/cars/<model>/` folder.

## 5. Coordinate inverse

AC: `(X-right, Y-up, Z-forward)` — left-handed, Y-up.
SVJ / SAE J670: `(X-forward, Y-right, Z-down)`.

Forward (from AC_to_SVJ README): `svj = (ac.z, ac.x, -ac.y)`.
Inverse: `ac = (svj.y, -svj.z, svj.x)`.

Implement once in `coords.py` as `svj_to_ac_point(p)` and `svj_to_ac_vector(v)` (same transform for both, no translation component since SVJ pickup points are already car-relative).

### Suspension pickups

Per AC_to_SVJ README, the AC X of `WBCAR_*`, `WBTYRE_*`, `LINK_*_CAR`, `PANHARD_CAR`, `WATTS_CAR`, `STRUT_CAR/TYRE`, `TRAIL_CAR`, `SEMI_TRAIL_CAR` is **inboard-positive from the wheel centre**, not the centreline.

- `absolute_lateral = half_track − ac_x` → so when writing AC: `ac_x = half_track − abs(svj_lateral)`.
- `chassis_z_offset = −rolling_radius` → when writing AC: take `rolling_radius` from the tyre section, negate it.

`half_track` and `rolling_radius` must be resolved per axle **before** any pickup-writing happens. Cache them on a small `AxleContext` object.

## 6. Suspension types

| AC `TYPE` | SVJ `system_type` | Pickup keys to emit |
| --- | --- | --- |
| 0 / STRUT | `macpherson` | `WBCAR_BOTTOM_*`, `WBTYRE_BOTTOM_*`, `STRUT_CAR`, `STRUT_TYRE` |
| 1 / DWB | `double_wishbone` | `WBCAR_TOP_*`, `WBCAR_BOTTOM_*`, `WBTYRE_TOP_*`, `WBTYRE_BOTTOM_*`, tie rod pair |
| 2 / MULTILINK | `multi_link` | upper/lower lateral + toe link key pairs |
| 3 / AXLE | `solid_axle` | `LINK_*_CAR/TYRE`, `PANHARD_CAR/AXLE` **xor** `WATTS_CAR/AXLE` |
| 4 / TRAILING_ARM | `trailing_arm` | `TRAIL_CAR`, `TRAIL_TYRE` |
| 5 / SEMI_TRAILING | `semi_trailing_arm` | `SEMI_TRAIL_CAR`, `SEMI_TRAIL_TYRE` |

If SVJ `solid_axle` has both Panhard and Watts populated, prefer Panhard and warn.

## 7. Unit table

| Quantity | SVJ unit | AC unit | Conversion |
| --- | --- | --- | --- |
| Length | m | m (mostly) — INI scalars | identity |
| Mass | kg | kg | identity |
| Inertia | kg·m² | kg·m² | identity |
| Force | N | N | identity |
| Torque | N·m | N·m | identity |
| Angle, camber/toe | rad | **degrees** | `rad → deg` |
| Angle, steering rack | rad | rad | identity |
| Angular velocity (engine) | rad/s | **RPM** | `rad/s × 60/(2π)` |
| Power | W | kW (display only) | identity in INI, convert for `ui_car.json` |
| Pressure (tyres) | Pa | psi | `Pa × 0.000145038` |
| Temperature | K | °C | `K − 273.15` |

Centralise in `units.py`. Every writer imports from there.

## 8. Per-file mapping (top level)

| AC file | SVJ source section(s) | Notes |
| --- | --- | --- |
| `car.ini` | `chassis`, `aero` (frontal area), `metadata.model_id` | `[BASIC] GRAPHICS_OFFSET`, `INERTIA`, `TOTALMASS`, `SCREEN_NAME` |
| `suspensions.ini` | `suspension.axles[]`, `chassis.dimensions` | The big one. Per-axle `system_type` → `[FRONT]/[REAR]` blocks with all pickup keys |
| `tyres.ini` | `tyres.compounds[]` | One section per compound; use `_ac_native` if present, else run `pacejka_inverse.py` |
| `engine.ini` + `power.lut` | `powertrain.engine` | LUT is RPM,Nm two-column |
| `drivetrain.ini` | `powertrain.drivetrain` | layout (FWD/RWD/AWD), gears[], diff |
| `aero.ini` | `aero.wings[]`, `aero.drag` | One `[WING_N]` per element |
| `brakes.ini` | `brakes` | `[FRONT]/[REAR]` torque, bias |
| `electronics.ini` | `electronics` | TC, ESP, ABS toggles + presets |
| `setup.ini` | `setup_defaults` | Min/max/default per adjustable |
| `ctrl_*.ini` | `controllers[]` | Each has matching `.lut`; turbo, ABS, traction, etc. |
| `ui/ui_car.json` | `metadata` | Kunos-canonical fields only; extras go under `specs.*` |

Authoritative key list per file comes from `AC_to_SVJ/ac_parsers.py` — every key the parser reads is a key the writer must emit (when source data exists).

## 9. Pacejka inverse

AC's tyre model uses parametric forms (`LOAD_CURVE`, `DX_REF`, `DY_REF`, `PRESSURE_*`, `CAMBER_GAIN`, etc.). AC_to_SVJ fits MF 5.2 from those.

Going back is underdetermined. Strategy:
1. If `tyres.compounds[i]._ac_native` exists, use it verbatim. **This is the default path.**
2. Otherwise, run `scipy.optimize.least_squares` to find AC parameters whose MF 5.2 fit matches the SVJ coefficients at standard load points (Fz = 1500, 3000, 4500, 6000 N) for both Fx and Fy. Warn in `conversion_log.txt` that the result is approximate.
3. The fit reuses the forward model from `AC_to_SVJ/tire_lab.py` — import it as a library, do not duplicate.

**Action item for forward project**: AC_to_SVJ should be patched (separate PR) to embed `_ac_native` blocks during forward conversion. Until then, expect approximate tyres.

## 10. FBX skeleton (optional, stage 7)

When user ticks "generate placeholder model", produce an ASCII FBX 2014 with:
- Z-up, units cm (scale SI m × 100).
- One root null `Car`.
- Wheel dummies: `WHEEL_LF`, `WHEEL_RF`, `WHEEL_LR`, `WHEEL_RR` — positioned from SVJ wheelbase + half-track + rolling radius.
- Suspension dummies: `SUSP_LF/RF/LR/RR` at the same positions as wheels (KsEditor binds them).
- Cockpit dummies: `COCKPIT_HR`, `STEER_HR` — placed at driver location from SVJ if present, else 0.4 m left of centre, 0.7 m up, 0.3 m back from front axle.
- One cube `BODY` sized to SVJ overall_length × width × height, with a default Kunos-grey material.
- Cylinder mesh per `WHEEL_*` sized to tyre width + rolling radius.

Skip skinning, materials beyond defaults, LODs. KsEditor will compile, the result drives, it looks like a soapbox. That is fine.

Also emit `lods.ini` with a single LOD_0 entry, and `persistence.ksanim` placeholder (empty animation).

## 11. Acceptance criteria

A run is successful when:
1. `pytest smoke_test.py` passes — full round-trip on `test_car/` with parsed-value diff < 1e-6 on all scalars, < 1 mm on all 3D points.
2. Round-trip on `examples/ks_mazda_mx5_nd.svj.json` produces a folder that Content Manager imports without error.
3. Re-running AC_to_SVJ on the output yields an SVJ whose diff with the original input is limited to the Pacejka block (and zero diff if `_ac_native` was used).
4. If FBX skeleton was generated: KsEditor opens the FBX and a "Save kn5" succeeds without errors.
5. The Gradio UI runs `python svj_to_ac.py` and serves on `127.0.0.1:7860` with the four tabs functional.

## 12. Out of scope

- Writing kn5 binary directly.
- Skins, brake disc textures, headlight animations.
- Encrypting `data/` back into `data.acd` (Content Manager does this).
- Sound (`sfx/` beyond a stub `GUIDs.txt`).
- Driver model FBX.

## 13. Licence

MIT, identical to AC_to_SVJ. Intended for AC car data the user has legally obtained.
