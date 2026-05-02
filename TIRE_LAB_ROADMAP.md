# Tire Lab — Future-Patches Roadmap

This document is a self-contained spec for extending `tire_lab.py` beyond
the single-operating-point MF 5.2 pure-slip fit that ships in v0.99. It's
written so a future contributor — human or AI — can pick any patch, read
this page, and have everything they need to implement it without re-deriving
context.

**Canonical input file:** `tire_lab.py`
**Canonical schema file:** `converter.py::build_svj_pacejka_block()` and
whatever it delegates to.
**Test fixture:** `test_car/data/tyres.ini` (synthetic MX-5, both axles).
**Regression baseline:** every patch must keep `smoke_test.py` green and
`examples/mx5_nd_club.svj.json` regenerated.

---

## 0. Current state (v0.99)

`tire_lab.sweep(p)` walks `Fz × SA` and `Fz × SR` independently at a single
operating point: `P = PRESSURE_IDEAL`, `T = optimum`, `γ = 0`, `v ≈ 0`,
fresh tread. `_fit_lat` and `_fit_long` fit MF 5.2 pure-slip only. The
emitted SVJ block already carries:

```
tires.sets.<compound>.models.pacejka_mf52.{
  version, reference_load_N,
  pure_slip_lateral  { pCy1, pDy1, pDy2, pEy1, pEy2, pKy1, pKy2, _metrics },
  pure_slip_longitudinal { pCx1, pDx1, pDx2, pEx1..4, pKx1..3, _metrics },
  _source, _source_params, _note
}
```

The parser (`ac_parsers.py::parse_tyre_section`) already reads these fields
but **does not** forward them into the bench:
`PRESSURE_IDEAL`, `PRESSURE_D_GAIN`, `PRESSURE_FLEX_GAIN`,
`DCAMBER_0`, `DCAMBER_1`, `SPEED_SENSITIVITY`, `RELAXATION_LENGTH`,
`ROLLING_RESISTANCE_0/1/2`, the `THERMAL_FRONT_N` block
(`OPTIMAL_TEMPERATURE`, `CORE_TRANSFER`, `SURFACE_TRANSFER`, `ROLLING_K`).

Every patch below is fundamentally "wire one of these into the sweep, emit a
new SVJ sub-block." None of them require changing the overall SVJ container
or breaking consumers of the existing `pacejka_mf52` block.

---

## 1. SVJ format support — yes, with one recommended versioning bump

**Can SVJ 0.95 hold everything proposed here? Yes.** The format is a
permissive JSON tree. The existing `tires.sets.<compound>` block has the
expected slots already named in the `_note` ("combined-slip + thermal +
transient terms deferred"), and the top-level `_metadata.version` plus the
`x_assettocorsa.*` escape hatch mean you can add fields freely without
breaking consumers that only care about what they already read.

**Recommended extension points (all additive, all backwards-compatible):**

```
tires.sets.<compound>.models.pacejka_mf52.{
  pure_slip_lateral          (already there)
  pure_slip_longitudinal     (already there)
  combined_slip              ← new in §2
  transient                  ← new in §4
  camber                     ← new in §3
  scaling_curves.{pressure, temperature, speed, wear}   ← new in §5–§8
  raw_sweep_ref              ← new in §9 (path to sidecar .npz)
}
tires.sets.<compound>.thermal_model        ← new in §10
tires.sets.<compound>.pressure_dynamics    ← new in §11
tires.sets.<compound>.data_origin.{source, ac_keys, confidence}
  ← align with SVJ 0.95 aero convention
```

**Versioning guidance.** None of these require `SVJ_VERSION > 0.95` — they
all land inside `tires.*` as new optional keys. If we later add the Tier 3
patches (thermal ODE, pressure dynamics), consider bumping `SVJ_VERSION`
to 0.96 and documenting the new blocks in the upstream
`SVJ-standard-vehicle-json` spec. Until then, emit them, mark them
optional, and add a `tires.schema_extensions: ["combined_slip", ...]` list
for consumers that want to opt in.

**What format would NOT support:** anything that requires dropping or
renaming a key already in the spec. Don't. Add, don't rewrite.

---

## 2. Tier 1 — Combined-slip fit (G_xα, G_yκ)

**Why:** without this, a consumer can't tell what the tire does under
trail-braking or throttle-on exit — the biggest gap in the current fit.

**AC source:** no new fields. Use the same `ACTyreParams` that already drives
`ac_fy` and `ac_fx`. Combined behavior is implicit in those forward models.

**Sweep change:** add a third pass to `sweep()`:

```python
# Pure-slip sweeps (existing) produce data["lat"], data["long"].
# New: combined-slip cube.
SA_grid = [0.02, 0.05, 0.10, 0.15, 0.20]           # rad
SR_grid = [-0.10, -0.05, 0.0, 0.05, 0.10]
Fz_grid = [0.5*Fz0, Fz0, 1.5*Fz0]
cube = []
for Fz in Fz_grid:
  for SA in SA_grid:
    for SR in SR_grid:
      Fy = ac_fy_combined(SA, SR, Fz, γ=0, p)
      Fx = ac_fx_combined(SA, SR, Fz, γ=0, p)
      cube.append({"Fz":Fz, "SA":SA, "SR":SR, "Fy":Fy, "Fx":Fx})
data["combined"] = cube
```

`ac_fy_combined` / `ac_fx_combined` need the AC combined-slip forward model.
Reference: Pacejka §4.3.2 friction-ellipse reduction; the AC docs (or
`Content Manager`'s tire preview source) show the actual reduction AC uses
(typically a normalized friction-ellipse `sqrt(SA²/SA_peak² + SR²/SR_peak²)`
that scales peak μ).

**Fit change:** add `_fit_combined(cube)` that fits MF 5.2 G-functions:

```
rBx1, rBx2, rBx3                (longitudinal weighting vs SA)
rCx1
rBy1, rBy2, rBy3                (lateral weighting vs SR)
rCy1
```

Use `scipy.optimize.least_squares` with the existing residual-function style.

**SVJ output shape:**

```json
"combined_slip": {
  "version": "MF 5.2",
  "Gxa": { "rBx1": ..., "rBx2": ..., "rBx3": ..., "rCx1": ... },
  "Gyk": { "rBy1": ..., "rBy2": ..., "rBy3": ..., "rCy1": ... },
  "_metrics": { "r2_Fx": 0.97, "r2_Fy": 0.97, "n_points": 75 }
}
```

**Verification:** add a smoke_test assertion: `r2_Fx >= 0.95`, `r2_Fy >= 0.95`
on the synthetic MX-5. Re-run `examples/` regen.

---

## 3. Tier 1 — Camber thrust scaling

**Why:** `CAMBER_GAIN` and `DCAMBER_0/1` are among the most setup-sensitive
knobs in AC. Currently ignored.

**AC source:** `CAMBER` exponent in `tyres.ini`, `DCAMBER_0`, `DCAMBER_1`.
Parser reads all three.

**Sweep change:** add

```python
CAMBER_grid = [-0.07, -0.035, -0.017, 0.0, 0.017, 0.035, 0.07]  # rad
camber_samples = []
for γ in CAMBER_grid:
  for Fz in Fz_grid:
    Fy = ac_fy(SA=0.05, Fz, γ, p)
    camber_samples.append({"γ":γ, "Fz":Fz, "Fy_ratio": Fy / Fy_at_γ0})
data["camber"] = camber_samples
```

**Fit change:** fit either
1. a quadratic `μ_scale(γ) = 1 + a·γ + b·γ²`, or
2. MF 5.2 native `pKy6, pKy7, pEy3, pEy4` (preferred if you're already
   solving within MF).

**SVJ output:**

```json
"camber": {
  "version": "MF 5.2" | "polynomial",
  "coeffs": { "pKy6": ..., "pKy7": ..., "pEy3": ..., "pEy4": ... },
  "curve": [ {"gamma_rad": -0.07, "mu_scale": 0.93}, ..., {"gamma_rad": 0.07, "mu_scale": 0.93} ],
  "_metrics": { "r2": 0.98, "n_points": 21 }
}
```

Emit both the MF coefficients AND the sampled LUT — downstream consumers
that aren't running MF can still use the LUT directly.

**Verification:** assert curve symmetry (|μ_scale(γ) − μ_scale(−γ)| < 0.02),
R² ≥ 0.95.

---

## 4. Tier 1 — Speed-sensitivity curve

**Why:** `SPEED_SENSITIVITY` linearly drops μ with speed in AC. Currently
parsed and passed through but never emitted as a consumer-friendly curve.

**AC source:** `SPEED_SENSITIVITY` scalar + optional `_speed_grip.lut`.

**Sweep change:**

```python
v_grid_kph = [20, 40, 60, 80, 120, 180, 250]
speed_samples = []
for v in v_grid_kph:
  Fy = ac_fy_at_speed(SA=0.1, Fz=Fz0, γ=0, v_kph=v, p)
  speed_samples.append({"v_kph": v, "mu_scale": Fy / Fy_at_v0})
```

**Fit change:** no MF change needed; emit a 1-D LUT directly.

**SVJ output:**

```json
"scaling_curves": {
  "speed": {
    "variable": "v_kph",
    "curve": [ [20, 1.00], [60, 0.99], [120, 0.97], [180, 0.94], [250, 0.89] ],
    "_source": "SPEED_SENSITIVITY + _speed_grip.lut"
  }
}
```

**Verification:** assert monotone-decreasing, curve[0] ≈ 1.0 within 1%.

---

## 5. Tier 2 — Pressure scaling curve

**AC source:** `PRESSURE_IDEAL`, `PRESSURE_FLEX_GAIN`, `DY0`/`DY1`,
`DX0`/`DX1`. Parser already reads them.

**Sweep change:**

```python
P_ratio_grid = [0.7, 0.8, 0.9, 0.95, 1.0, 1.05, 1.1, 1.2, 1.3]
pressure_samples = []
for r in P_ratio_grid:
  Fy = ac_fy_at_pressure(SA=0.1, Fz=Fz0, p_now=p_ideal*r, params)
  Fx = ac_fx_at_pressure(SR=0.08, Fz=Fz0, p_now=p_ideal*r, params)
  pressure_samples.append({"p_ratio":r, "mu_lat_scale":..., "mu_long_scale":...})
```

**SVJ output:**

```json
"scaling_curves": {
  "pressure": {
    "variable": "p_ratio",   // P_actual / P_ideal
    "p_ideal_pa": 186160,
    "lateral":      [[0.7, 0.88], [1.0, 1.00], [1.3, 0.91]],
    "longitudinal": [[0.7, 0.90], [1.0, 1.00], [1.3, 0.93]],
    "_source": "PRESSURE_IDEAL + PRESSURE_FLEX_GAIN sweep"
  }
}
```

**Verification:** peak μ occurs at p_ratio ≈ 1.0 ± 0.05.

---

## 6. Tier 2 — Temperature scaling curve

**AC source:** `OPTIMAL_TEMPERATURE`, `THERMAL_*` block,
`_grip_temp.lut` if present.

**Sweep change:** ~ §5 but on temperature axis
(40 / 60 / 80 / 100 / 120 / 140 °C core).

**SVJ output:**

```json
"scaling_curves": {
  "temperature": {
    "variable": "T_core_C",
    "T_optimum_C": 85,
    "lateral":      [[40, 0.70], [85, 1.00], [140, 0.88]],
    "longitudinal": [[40, 0.72], [85, 1.00], [140, 0.90]],
    "_source": "OPTIMAL_TEMPERATURE + THERMAL_*"
  }
}
```

**Verification:** peak at T_optimum ± 3 °C; μ at 40 °C < 0.85.

---

## 7. Tier 2 — Transient (relaxation length, first-order lag)

**AC source:** `RELAXATION_LENGTH` (parser already reads this).

**Sweep change:** run a step-input test:

```python
# SA jumps 0 -> 3° at t=0, integrate first-order lag dFy/dt = (Fy_ss - Fy)/τ
# where τ = σ_α / v.
```

Either take AC's value at face value (σ_α = RELAXATION_LENGTH) or fit a
transient step response to verify.

**SVJ output:**

```json
"transient": {
  "sigma_alpha_m": 0.35,
  "sigma_kappa_m": 0.18,
  "_source": "RELAXATION_LENGTH (direct) + fitted step response",
  "_verified": true
}
```

---

## 8. Tier 2 — Wear curve

**AC source:** `WEAR_CURVE` LUT, `MIN_GRIP`, `WEAR` scalar.

**Sweep:**

```python
wear_grid = [0, 0.1, 0.25, 0.5, 0.75, 0.9, 1.0]   # 0 = fresh, 1 = fully worn
for w in wear_grid:
  apply wear state, sweep μ at reference point, record mu_scale.
```

**SVJ output:**

```json
"scaling_curves": {
  "wear": {
    "variable": "wear_fraction",
    "curve": [[0.0, 1.00], [0.5, 0.95], [1.0, 0.78]],
    "wear_per_km": 0.015,
    "_source": "WEAR_CURVE + MIN_GRIP"
  }
}
```

---

## 9. Tier 2 — Raw sweep dump (provenance)

**Why:** anyone downstream can re-fit to MF 6.2, Fiala, MF-Swift, or a
neural surrogate without re-running AC.

**Implementation:** after every sweep, dump
`data["combined"] + data["pressure"] + data["temperature"] + ...` to an
`.npz` sidecar.

**Naming:** `<car>_<compound>_tire_sweep.npz`, next to the SVJ.

**SVJ reference:**

```json
"raw_sweep_ref": {
  "relative_path": "mx5_nd_club_front_tire_sweep.npz",
  "axes": ["Fz_N", "SA_rad", "SR", "gamma_rad", "v_kph", "p_pa", "T_core_C", "wear"],
  "shape": [3, 7, 7, 7, 5, 9, 7, 7],
  "byte_size": 98304,
  "_note": "Raw AC forward-model samples. Load with numpy.load(). Re-fittable to any tire model."
}
```

Pros: ~100 KB per compound, trivial to zip with the SVJ.
Cons: the sweep cube is wide; plan the axis set carefully to stay under 1 MB.

---

## 10. Tier 3 — Dynamic thermal model

**AC source:** `CORE_TRANSFER`, `SURFACE_TRANSFER`, `ROLLING_K`, track-heat
curves.

**Sketch:** two-node model (tread/carcass) with heat generation from sliding
power `P_gen = μ·Fz·|v_slip|`, conduction between nodes, cooling to air
and to track. Emit ODE constants in SVJ:

```json
"thermal_model": {
  "nodes": ["tread", "carcass"],
  "heat_capacities_J_per_K": {"tread": 5400, "carcass": 8200},
  "heat_transfer": {"tread_to_carcass": 120, "tread_to_track": 85, "carcass_to_air": 15},
  "T_ambient_C": 20,
  "T_track_C": 30,
  "_source": "THERMAL_FRONT_N block"
}
```

---

## 11. Tier 3 — Pressure dynamics (hot-pressure model)

**AC source:** `PRESSURE_D_GAIN`.

```json
"pressure_dynamics": {
  "p_cold_pa": 186160,
  "rise_per_K": 0.85,    // pa per °C of T_core
  "_source": "PRESSURE_D_GAIN"
}
```

---

## 12. Tier 3 — Full MF 6.1 / .tir export

Emit a proper `<car>_<compound>.tir` file alongside the SVJ so consumers
that use MSC Adams, IPG CarMaker, VI-CarRealTime, or Simulink can load it
directly. The sweep data from §2–§8 is enough to populate it.

Reference: MSC MF-Tyre & MF-Swift User Manual, `.tir` section for the full
parameter list. Expect ~200 parameters; most default to sensible values
when the corresponding AC field is absent.

---

## 13. Suggested implementation order

1. §2 Combined-slip (Tier 1)
2. §4 Speed sensitivity (Tier 1)
3. §3 Camber thrust (Tier 1)
4. §9 Raw sweep dump (Tier 2) ← do this before §5–§8 so the extra axes land
   in the dump for free.
5. §5 Pressure scaling (Tier 2)
6. §6 Temperature scaling (Tier 2)
7. §7 Transient (Tier 2)
8. §8 Wear (Tier 2)
9. §10–§12 Tier 3 only if a downstream consumer asks.

---

## 14. Refactor prerequisite: TireStateCube

Before adding more than one environmental axis, refactor `sweep()` to
return a single `TireStateCube` object (n-D numpy array + axis metadata)
rather than a dict of flat lists. Otherwise each new axis doubles the dict
bookkeeping in `_fit_*`.

Proposed dataclass:

```python
@dataclass
class TireStateCube:
    Fy: np.ndarray                 # shape (nFz, nSA, nSR, nγ, nv, nP, nT, nwear)
    Fx: np.ndarray                 # same shape
    Mz: Optional[np.ndarray] = None
    axes: dict[str, np.ndarray] = field(default_factory=dict)
    meta: dict = field(default_factory=dict)   # fit hints, source params
```

Fitters become `fit_pure_slip_lateral(cube, axis_slice={"γ":0, "v":0, ...})`.

Estimated effort: ~50 LOC once, saves ~100 LOC across §2–§8.

---

## 15. Pick-up checklist for a future session

Starting a new tire-lab patch session? In order:

1. Read `CHANGELOG.md` top entry — is there a tire work-in-progress note?
2. Read this file, skim §0–§1.
3. `python smoke_test.py` — must exit 0 before touching anything.
4. Pick a patch from §13 ordering.
5. Read that §'s AC source, sweep change, fit change, and SVJ output
   shape. Don't invent a new schema; emit under the existing
   `tires.sets.<compound>.models.pacejka_mf52.*` tree unless §1 says
   otherwise.
6. Extend `sweep()` first, make the new samples show up in `data`.
7. Extend the fitter.
8. Wire into `build_svj_pacejka_block()` under the right sub-key.
9. Add a smoke_test assertion for the R² / curve shape.
10. Regenerate `examples/mx5_nd_club.svj.json`.
11. Add a CHANGELOG entry.
12. If you add a new top-level key (§10, §11, §12), also update
    `DEVELOPMENT.md` §7 "SVJ 0.95 conventions" and this file's §1.
