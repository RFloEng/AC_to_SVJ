# SVJ_to_AC — Development notes

Inverse of [AC_to_SVJ](https://github.com/RFloEng/AC_to_SVJ). Reads SVJ
v0.95 JSON and emits a Content Manager-ready `content/cars/<model>/`
folder, plus an optional ASCII FBX 2014 skeleton that KsEditor can
compile into a `.kn5`.

## Data flow

```
SVJ JSON  (SAE J670, SI units)
   |
   v
svj_loader.load        --> SvjDoc (typed dataclass tree)
   |
   v
coords + units         --> pure transforms, single source of truth
   |
   v
ac_writers.*           --> one writer per AC data file
   |  car_ini, suspensions_ini, tyres_ini, engine_ini,
   |  drivetrain_ini, aero_ini, brakes_ini, electronics_ini,
   |  setup_ini, ctrl_files, ui_car_json
   v
pack_car.assemble      --> content/cars/<model>/ tree
   |
   v
pack_car.zip_folder    --> <model>.zip  (byte-deterministic)
```

Optional FBX side branch:

```
SvjDoc --> fbx_skeleton.write --> car.fbx + lods.ini + persistence.ksanim
```

## Coordinate conventions

| Frame | Axes | Used by |
|---|---|---|
| SVJ / SAE J670 | X-forward, Y-right, Z-down | Source SVJ JSON, metres |
| AC | X-right, Y-up, Z-forward | All `.ini` writers, metres |
| FBX (Z-up) | X-right, Y-forward, Z-up | `fbx_skeleton.py` only, **centimetres** |

The forward AC -> SVJ transform (per AC_to_SVJ README) is
`svj = (ac.z, ac.x, -ac.y)`. This project implements the inverse
`ac = (svj.y, -svj.z, svj.x)` exactly once, inside `coords.py`.

### Suspension pickups

AC's pickup-coordinate X is **inboard-positive from the wheel centre**,
not the car centreline. Convert via
`coords.pickup_lateral_to_ac(svj_lateral, half_track)` =
`half_track - abs(svj_lateral)`. AC pickup Z is offset by
`-rolling_radius` via `coords.pickup_z_to_ac(rolling_radius)`.

Worked example (from `ai_handoff/05_PITFALLS.md`):
- SVJ outer upper wishbone at `(1.4, 0.75, 0.35)`, half-track 0.78
- AC equivalent: `(0.03, -0.35, 1.4)`

The `tests/test_coords.py` battery includes this exact case.

## Unit table

Every conversion lives in `units.py`. `smoke_test.py` step 9 and
`tests/test_no_inline_units.py` enforce the rule with a repo-wide grep.

| Quantity | SVJ unit | AC unit | Function |
|---|---|---|---|
| Length | m | m | `m_to_m` (identity) |
| Mass | kg | kg | `kg_to_kg` (identity) |
| Inertia | kg.m^2 | kg.m^2 | `inertia_to_ac` (identity) |
| Force | N | N | `n_to_n` (identity) |
| Torque | N.m | N.m | `nm_to_nm` (identity) |
| Angle, camber/toe | rad | deg | `rad_to_deg` |
| Angle, steering rack | rad | rad | `rad_to_rad` (identity) |
| Engine speed | rad/s | RPM | `radps_to_rpm` |
| Power | W | kW | `w_to_kw` |
| Tyre pressure | Pa | psi | `pa_to_psi` |
| Temperature | K | degC | `k_to_c` |

Every conversion ships with its inverse (`deg_to_rad`, `rpm_to_radps`,
`psi_to_pa`, `kw_to_w`, `c_to_k`).

## AC-native pass-through

Every writer accepts an `x_assettocorsa._ac_native["<file>"]` block
carrying the **pre-parsed AC INI sections** as a `dict[section, dict[key, str]]`.
When present, the writer reproduces the source INI byte-faithfully
(modulo float formatting normalisation). When absent, writers fall
back to canonical SVJ fields plus reasonable defaults for AC-specific
data SVJ doesn't carry (e.g. brake disc inertia).

This extends spec §9's tyre-only `_ac_native` proposal to every INI
file, making round-trips lossless when AC_to_SVJ embeds the data.

For tyres specifically, the fast path reads
`tires.sets[*].models.pacejka_mf52._source_params` -- AC_to_SVJ
already embeds these from v0.99 onward.

## Module layout

```
svj_to_ac.py            Gradio UI + main()
svj_loader.py           load() + validate() + SvjDoc dataclasses
coords.py               SVJ <-> AC axis transforms
units.py                SI <-> AC unit conversions
lut_writer.py           AC two-column LUT writer
pacejka_inverse.py      MF 5.2 -> AC tyre param fitter
fbx_skeleton.py         ASCII FBX 2014 generator
pack_car.py             folder assembler + reproducible zipper
smoke_test.py           single-file regression suite (no pytest)

ac_writers/
   __init__.py          format_value / format_vec3 / format_section primitives
   _axle_context.py     AxleContext + resolve_axle
   car_ini.py
   suspensions_ini.py
   tyres_ini.py
   engine_ini.py
   drivetrain_ini.py
   aero_ini.py
   brakes_ini.py
   electronics_ini.py
   setup_ini.py
   ctrl_files.py
   ui_car_json.py

tests/                  pytest suite (163 tests across 9 files)
   conftest.py          adds the repository root (AC_to_SVJ's modules) to sys.path
   test_svj_loader.py
   test_coords.py
   test_units.py
   test_no_inline_units.py
   test_writers_basic.py
   test_writers_full.py
   test_suspensions.py
   test_pack.py
   test_fbx.py
   test_ui.py

examples/               mx5_nd_club.svj.json (canonical fixture)
test_car/               synthetic fixture (mirrors AC_to_SVJ/test_car/)
assets/                 badge_placeholder.png, preview_placeholder.jpg
ai_handoff/             original AI handoff pack (spec, build plan, pitfalls)
```

## Style

- Small, focused modules. Pure functions where practical.
- All paths via `pathlib.Path`. No `os.path.join`, no `str.replace`.
- INI writers use LF line endings (`newline="\n"`) and UTF-8 without BOM.
- `lut_writer` produces two-column LUTs with a trailing newline.
- Tests use `pytest`; commit-blocking lint via `ruff` (`pre-commit`).
- Single source of truth: `coords.py` for transforms, `units.py` for
  conversions, `ac_writers/__init__.py` for INI formatting primitives.

## Running tests

```
# Quick smoke test (no pytest required).
python smoke_test.py

# Full pytest suite.
python -m pytest -v
```

On macOS/Linux with the workspace mounted via Synology Drive or
similar, pytest needs a non-mount temp directory:

```
python -m pytest -v --basetemp=/tmp/pytest-tmp
```

## Author

RFloEng -- https://github.com/RFloEng
