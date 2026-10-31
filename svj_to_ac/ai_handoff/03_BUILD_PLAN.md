# SVJ_to_AC — Build plan

Ten sequenced stages. Each stage block contains a goal, files in scope, acceptance test, and a ready-to-paste prompt for the AI. Run in order; do not advance past a failing acceptance test.

---

## Stage 1 — Skeleton

**Goal**: Project layout and tooling exist. Empty modules importable. README and licence in place.

**Files**: full tree from `02_PROJECT_SPEC.md §2`, all `.py` files containing only docstring + `pass`, `requirements.txt`, `README.md`, `LICENSE`, `DEVELOPMENT.md`, `.gitignore`.

**Acceptance**:
```
pip install -r requirements.txt && python -c "import svj_to_ac, coords, units, svj_loader"
```

**Prompt**:
> Create the project skeleton exactly as defined in `02_PROJECT_SPEC.md §2`. All Python modules contain a one-line docstring plus `pass`. `requirements.txt` lists gradio, numpy, scipy, matplotlib, Pillow. `README.md` mirrors the AC_to_SVJ README structure (title, install, tabs, files, license sections) but marks the project as "beta 0.1, work in progress". `LICENSE` is MIT with my name (ask if unknown). Commit as `stage 1: skeleton`.

---

## Stage 2 — SVJ loader + schema

**Goal**: Load and validate SVJ v0.95 JSON. Fail loudly on missing required sections. Round-trip preserves structure.

**Files**: `svj_loader.py`, `tests/test_svj_loader.py`, `examples/ks_mazda_mx5_nd.svj.json` (copy from AC_to_SVJ/examples/).

**Acceptance**:
```
pytest tests/test_svj_loader.py -v
```
Must validate the example and reject a fixture with `chassis` removed.

**Prompt**:
> Implement `svj_loader.py` with `load(path) -> SvjDoc` and `validate(doc) -> list[Error]`. `SvjDoc` is a dataclass tree mirroring the SVJ v0.95 sections (metadata, chassis, suspension, tyres, powertrain, aero, brakes, electronics, setup_defaults, controllers). Required sections raise on missing; optional ones default to None. Add `tests/test_svj_loader.py` covering the happy path against `examples/ks_mazda_mx5_nd.svj.json` and the failure mode for a stripped fixture. Commit as `stage 2: svj loader`.

---

## Stage 3 — Coords and units

**Goal**: One transform, one unit module, both fully tested.

**Files**: `coords.py`, `units.py`, `tests/test_coords.py`, `tests/test_units.py`.

**Acceptance**:
```
pytest tests/test_coords.py tests/test_units.py -v
```
Round-trip a 100-point random cloud through `svj_to_ac → ac_to_svj` (use AC_to_SVJ's `coords` if exposed, else inline the forward transform in the test) with max error < 1e-9.

**Prompt**:
> Implement `coords.py` exposing `svj_to_ac_point(p)`, `svj_to_ac_vector(v)`, `ac_to_svj_point(p)` (for round-trip tests), and `pickup_lateral_to_ac(svj_lateral, half_track) -> float` and `pickup_z_to_ac(rolling_radius) -> float` as per `02_PROJECT_SPEC.md §5`. Implement `units.py` with one function per row of §7's unit table. No unit conversion may appear outside this module — enforce via a smoke grep test if convenient. Add tests covering all conversions including degenerate values (0, negative, very large). Commit as `stage 3: coords and units`.

---

## Stage 4 — INI writer scaffolding + first three files

**Goal**: The simplest three writers landed and round-trip clean: `car.ini`, `aero.ini`, `brakes.ini`. Establish the writer pattern others will follow.

**Files**: `ac_writers/__init__.py`, `ac_writers/car_ini.py`, `ac_writers/aero_ini.py`, `ac_writers/brakes_ini.py`, `lut_writer.py`, `tests/test_writers_basic.py`.

**Acceptance**:
```
pytest tests/test_writers_basic.py -v
```
For each writer: write from SVJ fixture → parse with AC_to_SVJ's `ac_parsers.py` → assert deep-equal vs original AC fixture in `AC_to_SVJ/test_car/data/`.

**Prompt**:
> Establish the writer pattern: every writer is a module exposing `write(doc: SvjDoc, out_dir: Path) -> Path` and a private `_render(doc) -> str`. Implement `ac_writers/car_ini.py`, `aero_ini.py`, `brakes_ini.py`, and `lut_writer.py` (two-column AC LUT format with trailing newline). Tests must round-trip through `AC_to_SVJ/ac_parsers.py` — import it as `from ac_parsers import parse_car_ini` etc. with `AC_to_SVJ` on `sys.path` from a conftest fixture. Commit as `stage 4: first writers`.

**Re-read before this stage**: `05_PITFALLS.md` §"Units", §"INI formatting quirks".

---

## Stage 5 — Suspensions writer

**Goal**: The hardest writer. Handles all six AC suspension types, all pickup-point conventions, half-track and rolling-radius resolution.

**Files**: `ac_writers/suspensions_ini.py`, `ac_writers/_axle_context.py`, `tests/test_suspensions.py`.

**Acceptance**:
```
pytest tests/test_suspensions.py -v
```
Round-trip through `ac_parsers.parse_suspensions_ini` for all six suspension types. Per-pickup 3D-point error < 1 mm. Also test the `solid_axle` Panhard-vs-Watts disambiguation warning.

**Prompt**:
> Implement `ac_writers/suspensions_ini.py` per `02_PROJECT_SPEC.md §6`. Add `_axle_context.py` exposing `AxleContext(half_track, rolling_radius, system_type)` resolved per axle before any pickup is written. Cover all six suspension types. Use `coords.pickup_lateral_to_ac` and `pickup_z_to_ac` everywhere — never inline the maths. Tests must round-trip through `ac_parsers.parse_suspensions_ini` for synthetic fixtures of all six types (build them in the test from real-world geometry — borrow numbers from `AC_to_SVJ/test_car/`). Commit as `stage 5: suspensions writer`.

**Re-read before this stage**: `05_PITFALLS.md` §"Coordinate sign mistakes", §"Suspension type 3 ambiguity".

---

## Stage 6 — Remaining INI writers

**Goal**: All other INI files land. Engine, drivetrain, electronics, setup, controllers, tyres (with Pacejka inverse).

**Files**: `ac_writers/engine_ini.py`, `drivetrain_ini.py`, `electronics_ini.py`, `setup_ini.py`, `ctrl_files.py`, `tyres_ini.py`, `pacejka_inverse.py`, `tests/test_writers_full.py`.

**Acceptance**:
```
pytest tests/test_writers_full.py -v
```
Round-trip every file through its `ac_parsers` counterpart. For tyres: with `_ac_native` present, expect deep-equal; without, expect R² ≥ 0.98 on Fx and Fy at standard load points vs the original AC parameters.

**Prompt**:
> Implement the remaining writers. `engine_ini.py` produces both the INI and the matching `power.lut` via `lut_writer`. `ctrl_files.py` iterates `doc.controllers[]` emitting one `ctrl_<name>.ini` plus the matching `.lut` per controller. `pacejka_inverse.py` checks `compound._ac_native` first; if missing, uses `scipy.optimize.least_squares` against the forward model imported from `AC_to_SVJ.tire_lab`. Log a warning to `conversion_log.txt` when the fallback runs. Tests must use both code paths. Commit as `stage 6: remaining writers`.

**Re-read before this stage**: `05_PITFALLS.md` §"Pacejka inverse", §"ctrl LUT references".

---

## Stage 7 — UI JSON + folder packager

**Goal**: `ui_car.json`, `sfx/GUIDs.txt` stub, placeholder badge/preview images, full folder assembly, zip output.

**Files**: `ac_writers/ui_car_json.py`, `pack_car.py`, `assets/badge_placeholder.png`, `assets/preview_placeholder.jpg`, `tests/test_pack.py`.

**Acceptance**:
```
pytest tests/test_pack.py -v
```
After a full conversion, the resulting folder has all 14 minimum required files; `ui_car.json` validates against Kunos schema fields (name, brand, year, class, specs.{}); the zip is reproducible (same input → byte-identical output).

**Prompt**:
> Implement `ui_car_json.py` (Kunos-canonical keys only, extras under `specs.`), `pack_car.py` (assembles `content/cars/<model>/`, copies placeholder badge.png and preview.jpg, emits sfx/GUIDs.txt stub referencing `event:/cars/kunos_default/engine_int`, then zips the folder reproducibly with sorted entries and fixed mtime). Add `tests/test_pack.py`. Commit as `stage 7: ui and packager`.

---

## Stage 8 — Optional FBX skeleton

**Goal**: Generate a minimal FBX that KsEditor accepts.

**Files**: `fbx_skeleton.py`, `tests/test_fbx.py`.

**Acceptance**: FBX file opens in KsEditor without errors and a kn5 export succeeds. Test programmatically by parsing the generated FBX with `fbx-python-binding` if available, otherwise sanity-check via grep for required dummy names and Z-up axis.

**Prompt**:
> Implement `fbx_skeleton.py` producing ASCII FBX 2014 per `02_PROJECT_SPEC.md §10`. Use a string template — do not introduce a binding library. Required dummies: WHEEL_LF/RF/LR/RR, SUSP_LF/RF/LR/RR, COCKPIT_HR, STEER_HR, BODY mesh, four wheel cylinder meshes. Z-up, cm units. Add `tests/test_fbx.py` asserting file structure (header version, axis, all dummy names present, unit scale = 1.0 for cm). Commit as `stage 8: fbx skeleton`.

**Re-read before this stage**: `05_PITFALLS.md` §"KsEditor FBX requirements".

---

## Stage 9 — Gradio UI

**Goal**: Four-tab UI mirroring AC_to_SVJ: upload SVJ, single car directory, batch, FBX lab.

**Files**: `svj_to_ac.py` (full implementation).

**Acceptance**: `python svj_to_ac.py` serves on `127.0.0.1:7860` with four working tabs. Each tab end-to-end converts a sample SVJ and produces a downloadable zip.

**Prompt**:
> Implement `svj_to_ac.py` as a single-file Gradio app, four tabs (Upload, Single SVJ, Batch SVJ folder, FBX Lab). Mirror AC_to_SVJ's UI conventions: file drop + path input + Convert button + progress + downloadable zip + log textbox. Batch outputs `svj_ac_batch_<date>_conv<ver>.zip`. The FBX Lab tab generates a placeholder model from manually entered wheelbase/track/etc. or from a loaded SVJ. Commit as `stage 9: gradio ui`.

---

## Stage 10 — Smoke test and docs

**Goal**: Full regression suite, DEVELOPMENT.md, ROADMAP.md, CHANGELOG.md matching AC_to_SVJ's docs.

**Files**: `smoke_test.py`, `DEVELOPMENT.md`, `ROADMAP.md`, `CHANGELOG.md`, updated `README.md`.

**Acceptance**:
```
python smoke_test.py
```
All checks pass: round-trip every fixture under `test_car/` and `examples/`, count parity with AC_to_SVJ's smoke test where applicable.

**Prompt**:
> Finalise the project. Implement `smoke_test.py` as a single-file regression runner (do not require pytest) mirroring `AC_to_SVJ/smoke_test.py`'s structure: numbered checks, pass/fail line per check, exit code reflects total. Cover loader, every writer, full round-trip via AC_to_SVJ. Write `DEVELOPMENT.md` (coordinate conventions, unit table, data flow diagram in ASCII), `ROADMAP.md` (next steps: full kn5 writer research, skin templating, sound integration), `CHANGELOG.md` (start at 0.1.0). Update `README.md` to remove "work in progress" and add a "Fit quality" table mirroring AC_to_SVJ's. Commit as `stage 10: smoke test and docs`. Tag as `v0.1.0`.
