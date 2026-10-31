# Changelog

All notable changes to SVJ_to_AC will be documented in this file.
Format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [0.1.0] — 2026-05-17

First release. Project produces Content Manager-ready
`content/cars/<model>/` folders from SVJ v0.95 JSON input.

### Added

- **Loader (`svj_loader.py`)** -- `load(path) -> SvjDoc` and
  `validate(doc) -> list[SvjError]`. SvjDoc dataclass tree mirrors
  the real SVJ schema (vehicle_info, chassis, suspension, tires,
  powertrain, brakes, optional metadata / aerodynamics /
  x_assettocorsa).
- **Coordinate transforms (`coords.py`)** -- single source of truth
  for SVJ <-> AC axis flips, plus AC pickup-point conventions
  (inboard-from-wheel-centre X, `-rolling_radius` Z offset).
- **Unit conversions (`units.py`)** -- single source of truth for
  every Pa/psi, rad/deg, rad/s/RPM, K/degC, W/kW conversion.
- **INI writers** for the full Kunos data file set: `car.ini`,
  `suspensions.ini` (all six AC TYPE values 0..5),
  `tyres.ini`, `engine.ini` (+ companion `power.lut`,
  `coast_curve.lut`), `drivetrain.ini`, `aero.ini`, `brakes.ini`,
  `electronics.ini`, `setup.ini`, `ctrl_*.ini` (with paired
  `.lut` companions per the LUT-reference pitfall).
- **`_ac_native` pass-through** -- every writer accepts a
  pre-parsed AC INI section dict for byte-faithful round-trips.
- **Pacejka inverse (`pacejka_inverse.py`)** -- fast path reads
  `models.pacejka_mf52._source_params` verbatim; slow path runs
  `scipy.optimize.least_squares` against the `tire_lab.py` forward
  model with WARNING-level logging.
- **UI JSON writer** for `ui/ui_car.json` with Kunos-canonical
  fields and `specs.*` extras.
- **Folder packager (`pack_car.py`)** -- assemble the full tree,
  emit reproducible byte-identical zips (sorted entries, fixed
  `ZipInfo.date_time`).
- **FBX skeleton (`fbx_skeleton.py`)** -- ASCII FBX 2014 with
  Z-up axis (UpAxis=1, FrontAxis=2, CoordAxis=0), UnitScaleFactor=1,
  required dummies (`WHEEL_LF/RF/LR/RR`, `SUSP_*`, `COCKPIT_HR`,
  `STEER_HR`), `BODY` cube, four `WHEEL_*_MESH` cylinders, default
  phong material. Companion `lods.ini` + `persistence.ksanim`.
- **Gradio UI (`svj_to_ac.py`)** -- four tabs: Upload SVJ, Single
  SVJ (path), Batch SVJ folder (combined zip
  `svj_ac_batch_<YYYYMMDD>_conv0_1.zip` + `batch_summary.csv`),
  FBX Lab (SVJ or manual dims).
- **Smoke test (`smoke_test.py`)** -- single-file no-pytest runner,
  53 numbered checks, exit code reflects failures.
- **Pytest suite** -- 163 tests across 10 test files; full
  round-trip via `AC_to_SVJ/ac_parsers.py` ground truth.
- **Guard rails** -- `tests/test_no_inline_units.py` and
  `smoke_test.py` step 9 grep for inline conversion constants
  anywhere outside `units.py`.

### Notes

- Targets SVJ v0.95 specifically (key set follows
  `AC_to_SVJ/examples/mx5_nd_club.svj.json`).
- Requires Python 3.10+. Dependencies: `gradio`, `numpy`, `scipy`,
  `matplotlib`, `Pillow`.
- Does not write `.kn5` binaries (use KsEditor) or `data.acd`
  (use Content Manager -> Tools -> Pack Data).
