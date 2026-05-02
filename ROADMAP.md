# Roadmap

This is a high-level view of planned improvements. For the detailed Pacejka
extension spec (combined slip, camber, pressure/temperature scaling, etc.)
see `TIRE_LAB_ROADMAP.md`.

---

## Tyre model — near term

The current fitter covers pure-slip MF 5.2 (lateral and longitudinal) at a
single operating point. The most impactful near-term additions, in priority
order:

1. **Combined-slip G-functions** (`Gxα`, `Gyκ`) — needed to describe
   trail-braking and throttle-on-exit behaviour. Derivable from the existing
   AC forward model with no new source fields.
2. **Camber thrust** — wire in `CAMBER_GAIN` / `DCAMBER_0/1`; emit a sampled
   μ-vs-γ curve alongside the MF coefficients so consumers that don't run MF
   can still use it.
3. **Speed-sensitivity curve** — `SPEED_SENSITIVITY` is already parsed; emit
   a μ-scale LUT over a 20–250 kph grid.
4. **Pressure and temperature scaling curves** — `PRESSURE_FLEX_GAIN`,
   `OPTIMAL_TEMPERATURE`, thermal block; emit sampled LUTs so consumers
   can apply corrections without re-running the full AC model.

See `TIRE_LAB_ROADMAP.md` §13 for the recommended implementation order and
§14 for the `TireStateCube` refactor that should precede items 4+.

---

## Suspension geometry — near term

- **Roll-centre and instant-centre calculation** — derive IC from the
  upper/lower arm lines in front view; emit `roll_centre_height` per corner.
  Useful for setup tools and as a sanity-check on the hardpoint geometry.
- **Kingpin geometry** — compute scrub radius, mechanical trail, and caster
  trail from the ball joint and spindle hardpoints; emit under
  `topology.upright.geometry`.
- **Anti-dive / anti-squat percentages** — from wishbone angles relative to
  the force line; straightforward once ICs are available.

---

## Data coverage

- **`setup.ini` range binding** — map adjustment ranges onto the SVJ
  `static_setup` fields so consumers know the legal min/max for each
  parameter (spring rate, ride height, damper, camber, toe, ARB).
- **Transmission ratios from `drivetrain.ini`** — individual gear ratios are
  already in the SVJ; add `efficiency` and `shift_delay` where present.
- **Fuel map** — `fuel_consumption.lut` if the mod ships it.

---

## Output formats

- **`.tir` export** — a proper MSC Adams / IPG CarMaker tyre file alongside
  the SVJ. The pure-slip fit already provides enough coefficients for a
  minimal `.tir`; combined-slip (item 1 above) fills the rest.
  See `TIRE_LAB_ROADMAP.md` §12.
- **Raw sweep `.npz` sidecar** — dump the full `Fz × SA × SR × γ` sweep so
  downstream tools can re-fit to any model (MF 6.1, Fiala, neural net)
  without re-running AC. See `TIRE_LAB_ROADMAP.md` §9.

---

## UI / workflow

- **Per-car progress bar in batch** — replace the single spinner with a
  per-car status row in the checkbox list (✓ / ✗ / in-progress).
- **Re-run single car from batch results** — click a car name in the output
  log to open its SVJ in tab 2 without re-running the whole batch.
- **Diff view** — when re-converting a car that already has an SVJ in the
  output folder, surface a concise diff of what changed.

---

## Code quality

- **`TireStateCube` refactor** — before adding more than one environmental
  axis to the tyre sweep, refactor `tire_lab.sweep()` to return a typed
  n-D array with axis metadata. See `TIRE_LAB_ROADMAP.md` §14.
- **Typed SVJ builder** — replace the current dict-building in `build_svj()`
  with dataclasses or a lightweight schema so field names are checked at
  definition time.
- **CI smoke test** — add a GitHub Actions workflow that runs `smoke_test.py`
  on every push and pull request.
