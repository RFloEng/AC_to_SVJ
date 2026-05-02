# Development & Handoff Notes

This document is for future contributors — human or AI — picking up the
AC → SVJ Converter mid-stream. It captures the architecture, conventions,
and the small-but-painful pitfalls that have eaten time before.

---

## 1. Architecture map

```
converter.py       Main app. Gradio UI + SVJ assembly.
                   Public surface:
                     - convert_uploaded_files()
                     - convert_single_car_dir()
                     - convert_batch_collection()
                     - build_corner()        ← per-corner topology
                     - build_svj()           ← top-level SVJ assembly
                   Constants:
                     - CONV_VER  = "0.99"
                     - SVJ_VERSION = "0.95"

ac_parsers.py      AC ini/lut readers. Source of truth for which ini sections
                   are recognised. HARDPOINT_KEYS is the canonical list of
                   suspension pickup keys consumed by build_corner.

acd_reader.py      Car-data loader. Reads unpacked `data/` for any car
                   that has it; for cars with only an encrypted
                   `data.acd` archive returns an empty ini map and a
                   clear "needs unpack" note (callers surface that to
                   the user).

tire_lab.py        AC tire forward model + Pacejka MF 5.2 fitter +
                   matplotlib plot generation. The Tire Lab Gradio tab in
                   converter.py just wires this up.

smoke_test.py      End-to-end smoke test against test_car/. If a real_car/
                   folder exists alongside the repo root, additional real-car
                   asserts run automatically.

test_car/          Synthetic Mazda MX-5 ND2 with every recognised AC field
                   populated. CRITICAL: the WBTYRE_* values here match the
                   v0.99 wheel-space convention. Don't ever copy values from
                   an older test_car or older zip — they are stale.

examples/          Reference SVJ outputs.
```

## 2. Coordinate convention (read this first)

AC: **(X-right, Y-up, Z-forward)**.
SVJ / SAE J670: **(X-forward, Y-right, Z-down)**.

Transform applied to every point: `svj = (ac.z, ac.x, -ac.y)`.

Two reference frames in `suspensions.ini`:

- **Wheel-centre-relative, inboard-positive**: `WBCAR_*`, `LINK_*_CAR`,
  `PANHARD_CAR`, `WATTS_CAR`, `TRAIL_CAR`, `SEMI_TRAIL_CAR`, **and also**
  `WBTYRE_*`, `STRUT_TYRE`. **All** chassis- and wheel-space pickup keys
  share the same lateral (X in AC) convention: the value is an
  *inboard-positive* distance from the wheel centre, not from the car
  centreline. A value of `0.38` means "0.38 m inboard of the wheel
  centre"; the absolute lateral position in vehicle frame is therefore
  `half_track − X`. The converter applies this via `wheel_y − ac.x × y_sign`.

  After axis conversion the longitudinal (SVJ X) and vertical (SVJ Z)
  components are shifted as usual: X by the axle position (`0` front,
  `−wheelbase` rear), Z by `-(rolling_radius + BASEY)` per axle — see
  "Chassis-frame Z origin" below.

> **Historical note** — prior to v0.99.10, `WBCAR_*` was (incorrectly)
> treated as a chassis-centreline absolute for the lateral axis, while
> `WBTYRE_*` was already wheel-centre-relative. The discrepancy produced
> inverted V-shape geometry (upper inboard closer to centreline than
> lower) and exaggerated strut inclination angles (≈36° vs the correct
> ≈15° for the Audi TT Cup). The fix was verified against:
> - MX-5 ND: upper inboard 0.365 m from centreline (wider) vs lower
>   0.293 m — correct DWB V-shape, lower arm 355 mm vs upper 228 mm.
> - Audi TT Cup: strut inclination 14.9° (was 36.3°).
> - Ferrari F2004: `WBCAR_BOTTOM_X = 0.7827 > half_track = 0.735` — only
>   valid under wheel-centre interpretation (F1 keel pushrod geometry).

### Chassis-frame Z origin (the BASEY trap — fixed in v0.99.1)

The chassis-space frame in AC is **not** ground-relative — it's
CG-relative. Kunos' own template comment is canonical:

    BASEY = Distance of CG from the centre of the wheel
    Front Wheel Radius + BASEY = front CoG (height above ground)

So a `WBCAR_BOTTOM_FRONT` Y of `-0.0696` (typical for a stock car) means
"7 cm below the CG", *not* "7 cm below the ground". To express that in
the SVJ ground-relative vehicle frame:

    svj.z = -(rolling_radius + BASEY) + (-ac.y)
          = chassis_z_offset - ac.y

`build_corner` precomputes `chassis_z_offset = -(rolling_radius + BASEY)`
once per axle and passes it to `ac_hp_to_vehicle()`. v0.99 shipped
without the offset, putting every chassis-side pickup ~10–20 cm too low
on stock Kunos cars (and below the ground plane on cars where
`WBCAR_BOTTOM` Y < 0). If `BASEY` is missing from the parsed dict the
offset falls back to 0 (matches v0.99 behaviour — predictable, even if
wrong).

## 3. Suspension `TYPE` codes

| AC `TYPE` | sys_type string         | Notes                                       |
|-----------|-------------------------|---------------------------------------------|
| `0`/STRUT | `macpherson`            | strut link from `STRUT_CAR`/`STRUT_TYRE`    |
| `1`/DWB   | `double_wishbone`       | upper/lower wishbone + steering tie rod     |
| `2`/MULTILINK | `multi_link`        | upper/lower lateral + toe (LINK_0..2)       |
| `3`/AXLE  | `solid_axle`            | trailing links + Panhard or Watts           |
| `4`/TRAILING_ARM | `trailing_arm`   | upper/lower arm + optional `trailing_arm`   |
| `5`/SEMI_TRAILING | `semi_trailing_arm` | + optional `semi_trailing_arm`         |

The classifier in `ac_parsers.py` accepts both numeric codes and string
aliases. **Don't** quietly fall through to `macpherson` for unknown values
— if you see one, log a warning.

## 4. Build-a-corner data flow

`build_corner(corner_id, susp_axle, …)` is the single entry point. It returns
a dict with at minimum:

```python
{
  "topology": {
    "links": [ {"name": ..., "type": ..., "inboard_points": [...], ...}, ... ],
    "upright": {
      "hardpoints": { "name": [x,y,z], ... },
      ...
    },
    "spring_damper": {...},
    ...
  },
  "wheel": {...},
  "brake": {...},
  ...
}
```

All link names are stable identifiers — downstream consumers (and tests)
match on them. Don't rename without bumping the SVJ version and updating
`examples/`.

## 5. Atomic writes (Synology drive race condition)

The repo lives on a Synology-synced drive on at least one developer's
machine. Naive `Path.write_text()` followed by a quick re-read can return
a truncated file because the sync agent is mid-flush. Mitigation:

```python
import os, tempfile
from pathlib import Path

def atomic_write(path: Path, text: str):
    with tempfile.NamedTemporaryFile(
        mode="w", encoding="utf-8", dir=str(path.parent),
        prefix=".patch_", suffix=".tmp", delete=False,
    ) as tmp:
        tmp.write(text)
        tmp_name = tmp.name
    os.replace(tmp_name, path)
```

Use this for any patch script that re-reads the file it just wrote.

## 6. Testing

```bash
python smoke_test.py
```

Should exit 0. Drop a `real_car/` folder beside the repo root with an
unpacked `data/` directory to enable extra real-car checks.

Per-suspension audit fixtures (in `outputs/` during development; not part of
the released repo):

- `audit_solid_axle.py`
- `audit_multilink.py`
- `audit_trailing_arms.py`

Each fires `build_corner` with synthetic axle dicts and prints the resulting
link names + hardpoints. Useful for verifying that a topology branch lights
up under each combination of optional pickups.

## 7. SVJ 0.95 conventions

- `aerodynamics.components[]` is an array; each component has
  `data_origin: {source, ac_keys, confidence}`.
  - `source`: `"ac_native"` (parsed from a real AC field), `"derived"` (computed
    from AC fields), or `"synthetic"` (placeholder for missing data).
  - `ac_keys`: list of AC ini keys consulted.
  - `confidence`: `"high" | "medium" | "low"`.
- Wing AoA-CL/AoA-CD curves go under `coefficient_curves`, x-axis in float
  degrees (NOT radians).
- `wheel_center` Z is measured from the **reference plane (ground)**, not
  the hub. Update if you backport from older code.

## 8. Common pitfalls

### 8.1 Coordinate sign on `WBTYRE_*`
`WBTYRE_*` X is an inboard *magnitude* — same value applies to both sides
of the car. Don't multiply by `y_sign` again before mirroring; that flips
the inboard ball joint to the outside of the wheel.

### 8.2 Bash heredoc + `\!r` in f-strings
A double-quoted bash heredoc will history-expand `\!r`, producing
`SyntaxError: f-string expression part cannot include a backslash`. Use
single-quoted heredoc delimiters: `<<'PYEOF'`.

### 8.3 Truncated files on Synology
See §5. Symptom: a script you Edit'd disappears mid-line on next read. Fix:
re-write through the atomic-write helper.

### 8.4 Stale `test_car/` data
Earlier zips shipped with WBTYRE values from before the v0.97 wheel-space
fix (`0.050, 0.370, 0.000`). The current v0.99 values are
`0.1875, 0.095, 0.000` (front) and `0.2125, 0.075, 0.000` (rear). Always
copy `test_car/` from the latest source folder, never from an older
release zip.

### 8.5 Multi-link rear "fell through to wishbone"
Pre-v0.99, a multi-link rear with `LINK_0..2` keys silently rendered as
upper/lower wishbones because no branch consumed the `LINK_*` keys.
`build_corner` now has explicit branches per `sys_type`; if you add a new
suspension type, follow the same branch pattern.

### 8.6 Cars that ship only with `data.acd`
`acd_reader.open_car` does not unpack the encrypted `data.acd` archive —
it just detects it and returns an empty ini map plus a clear
"needs unpack" `source_note`. Callers (`convert_single_dir`,
`convert_batch`) surface that to the user with a Content Manager
unpack instruction; in batch mode every skipped car is also written to
a `skipped.txt` file inside the output ZIP.

## 9. Releasing

1. Bump `CONV_VER` in `converter.py` and `SVJ_VERSION` if the schema moved.
2. Run `python smoke_test.py` against `test_car/` — must exit 0.
3. Regenerate `examples/mx5_nd_club.svj.json` from `test_car/`.
4. Update `CHANGELOG.md` with a new `## [vX.Y] — date` block (Added /
   Fixed / Validated subsections).
5. Add a "What's new in vX.Y" section near the top of `README.md`.
6. Commit, tag (`vX.Y`), push.
7. GitHub release notes: copy the new CHANGELOG section verbatim.

## 10. AI session handoff checklist

If you're an AI picking up mid-stream:

1. Read `CHANGELOG.md` from the top — what landed in the most recent
   version.
2. Read this file end-to-end.
3. Open `converter.py` and confirm `CONV_VER`, `SVJ_VERSION`.
4. `grep -n "sys_type ==" converter.py` to see which suspension branches
   exist; cross-check against the table in §3.
5. Run `smoke_test.py` against `test_car/` to confirm a green baseline
   before changing anything.
6. If the 