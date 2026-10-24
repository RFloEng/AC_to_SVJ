"""
AC → SVJ Converter  v0.99
SVJ spec:       https://github.com/RFloEng/SVJ-standard-vehicle-json  (v0.97 target)
Converter repo: https://github.com/RFloEng/AC_to_SVJ

SVJ 0.97 target update (converter v0.9.1)
-----------------------------------------
 • `_metadata.version` bumped to "0.97". The schema enum no longer accepts
   "0.95"; valid values are "0.96" and "0.97". All existing physics output
   is fully backward-compatible — no fields removed or renamed.
 • `_metadata.coordinate_system` now accepts either a named string
   ("SAE_J670", "ISO_8855", "OpenDRIVE") or a custom axis-spec object
   {up, forward, handedness}. This converter continues to emit "SAE_J670"
   which remains valid under the new oneOf schema.
 • `_metadata.units` now accepts "SI" | "meters" | "millimeters" (glTF
   pipeline convenience). This converter continues to emit "SI".
 • New optional `assets.meshes` block and `visual` bindings on chassis /
   suspension corners (glTF node references). Not emitted here — AC has no
   glTF mesh source; consumers can add them post-conversion.

v0.99 highlights (vs v0.98)
---------------------------
 • SVJ target bumped to 0.95. Aerodynamics block restructured to match
   the 0.95 schema: legacy `coefficients` kept for back-compat, but new
   values also land in `global_coefficients`; top-level `wings[]`/`drs`
   are gone — wings become entries in `components[]` with `type:"wing"`,
   per-component `reference.frontal_area`, and LUT curves reshaped into
   `aero_map` objects; DRS lives under `active_systems.drs` with its
   `ACTIVE_SPEED_THRESHOLD` converted from km/h to m/s. A synthetic
   `{name:"body", type:"fixed"}` component carries the aero.ini BODY
   coefficients so Σ components = global_coefficients per §13.5.
 • `_metadata.data_origin` added (0.95 provenance block). `type:"simulation"`,
   `detail` derived from open_car's source_note, `confidence` = medium for
   unpacked data/.
 • Wing `POSITION` now transformed from AC axes to SAE J670 before
   emission (was leaking AC-frame coords into aerodynamics.wings in v0.98).
 • Wing `ANGLE` emitted as degrees (AC's aero.ini convention is degrees).
 • `WIND_MULT` and `DAMAGE_K` carried through to `x_assettocorsa.aero`
   (were parsed but dropped in v0.98).

v0.97 carryover
---------------
 • Suspension hardpoint frames fixed: WBCAR_* are treated as chassis-space
   absolutes, WBTYRE_* and STRUT_TYRE as wheel-space offsets added to the
   wheel centre. In v0.96 both were treated as absolutes, which collapsed
   upright ball joints near the car centreline.
 • STRUT_TYRE + STRUT_CAR now produce a dedicated `strut` link with a
   `damper_outboard` hardpoint (useful for MacPherson axles).

v0.96 carryover
---------------
 • Full engine parsing: power.lut & coast_curve.lut are now read for real,
   turbos, BOV, dynamic turbo, damage, throttle response, fuel consumption k
 • Full drivetrain: shift timings, autoclutch, autoblip, downshift profiler,
   differential type, damage — all carried into SVJ / x_assettocorsa
 • Suspension geometry thaw: WBCAR_*, WBTYRE_*, TIE_ROD_* hardpoints are
   parsed and transformed to SAE J670 when present
 • Bumpstops, progressive spring factor, rod length now populated
 • Tyres: multi-compound [FRONT_N]/[REAR_N] sets, thermal, relaxation length,
   rolling resistance, pressure model, camber thrust, speed sensitivity
 • Brakes: handbrake, cockpit bias adjust, thermal pads
 • Aero: wing sections incl. .lut curves, DRS, wind multiplier
 • Electronics: ctrl_ABS.ini, ctrl_TC.ini, ctrl_turbo*.ini, electric_controllers.ini
 • setup.ini → x_assettocorsa.setup_ranges (adjustability bounds)
 • car.ini: [CONTROLS], [INFO], [FUELTANK], [PIT_STOP], [RULES]
 • Damage: all 5 AC damage zones collected in x_assettocorsa.damage

Run:
    pip install -r requirements.txt
    python converter.py
"""

from __future__ import annotations

import json
import math
import re
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

import numpy as np

from ac_parsers import (
    parse_ini, parse_lut,
    parse_engine, parse_drivetrain, parse_suspension_axle,
    parse_damper_tables, parse_bumpstop_tables, parse_inline_lut,
    parse_brakes, parse_aero, parse_car_extras,
    detect_tyre_compounds, parse_tyre_extras, parse_tyre_thermal,
    parse_controller_ini, parse_setup_ranges,
    _parse_vec3, _f, _i, _m, _b, _first, _resolve_lut,
)
from svj_migrate import migrate as migrate_suspension
from susp_geometry import corner_geometry
from svj_migrate.naming import canonical_part_id
from tire_lab import (
    ACTyreParams, parse_tyre_section, run_bench,
    build_svj_pacejka_block, build_svj_mf62_block,
    build_svj_pacejka_blocks, BenchResult,
)
from ac_parsers import _parse_vec3 as _vec3
try:
    from kn5_reader import (scan_kn5_nodes, map_ac_nodes_to_svj,
                            map_ac_nodes_to_svj_parts,
                            find_car_kn5, find_car_kn5_lods, resolve_car_kn5,
                            kn5_to_glb, kn5_all_lods_to_glbs, describe_moving_parts)
    _KN5_AVAILABLE = True
except ImportError:
    _KN5_AVAILABLE = False

# ─── Constants ────────────────────────────────────────────────────────────────

SVJ_VERSION = "0.100"
SVJ_SPEC    = "SVJ"
CONV_VER    = "0.9.1"
REPO_URL    = "https://github.com/RFloEng/SVJ-standard-vehicle-json"
CONV_REPO   = "https://github.com/RFloEng/AC_to_SVJ"

CORE_INI_FILES = [
    "car.ini", "engine.ini", "drivetrain.ini", "suspensions.ini",
    "tyres.ini", "brakes.ini", "aero.ini",
    "dampers.ini", "bumpstops.ini",          # CSP extended physics (optional)
]
EXTRA_INI_FILES = [
    "wing.ini", "setup.ini", "fuel_cons.ini",
    "ctrl_abs.ini", "ctrl_tc.ini", "ctrl_turbo0.ini", "ctrl_turbo1.ini",
    "electric_controllers.ini", "ers.ini",
]

SUSP_TYPE_MAP = {
    # AC accepts BOTH numeric codes (0..5) and string aliases. Kunos/mod cars
    # in the wild mix the two conventions freely, sometimes within one file.
    "0": "macpherson", "1": "double_wishbone", "2": "multi_link",
    "3": "solid_axle", "4": "trailing_arm", "5": "semi_trailing_arm",
    "STRUT": "macpherson", "MACPHERSON": "macpherson",
    "DWB": "double_wishbone", "DOUBLE_WISHBONE": "double_wishbone",
    "MULTILINK": "multi_link", "MULTI_LINK": "multi_link",
    "AXLE": "solid_axle", "SOLID_AXLE": "solid_axle", "LIVE_AXLE": "solid_axle",
    "TRAILING": "trailing_arm", "TRAILING_ARM": "trailing_arm",
    "SEMI_TRAILING": "semi_trailing_arm", "SEMITRAILING": "semi_trailing_arm",
}
DRIVE_LAYOUT_MAP = {"RWD": "FR", "FWD": "FF", "AWD": "AWD", "AWD2": "AWD", "4WD": "4WD"}
DIFF_TYPE_MAP = {
    "LSD": "lsd_clutch", "SALISBURY": "lsd_clutch",
    "CLUTCH": "lsd_clutch", "OPEN": "open", "SPOOL": "spool",
    "TORSEN": "lsd_torsen", "VISCOUS": "lsd_viscous",
}


def susp_type(raw) -> str:
    """Accept either a numeric code ('0'..'5') or a string alias ('STRUT',
    'DWB', 'MULTILINK', …). Falls back to 'custom' rather than silently
    defaulting to macpherson."""
    if raw is None:
        return "custom"
    key = str(raw).strip().upper()
    return SUSP_TYPE_MAP.get(key, "custom")


def drive_layout(raw: str) -> str:
    return DRIVE_LAYOUT_MAP.get((raw or "").upper().strip(), "FR")


# Rear-engined road cars whose weight split alone looks like a mid-engine car.
_RR_NAME_HINT = re.compile(
    r"porsche.*(911|930|964|993|996|997|991|992)|ruf|singer|fiat[ _]?126|a110",
    re.IGNORECASE)


def infer_engine_layout(layout: str, front_weight: float, car_key: str = "") -> tuple:
    """Refine AC's RWD (AC has no engine position) into FR / MR / RR.

    Heuristic from the static front weight fraction (suspensions.ini CG_LOCATION):
    a rear-driven car with < 46 % of its weight on the front axle has its engine
    behind the driver (MR); a name hint promotes known rear-engined families to RR.
    Returns (layout, source) where source is "weight", "name" or None when unchanged.
    """
    if layout != "FR" or not (0.0 < front_weight < 1.0):
        return layout, None
    if front_weight >= 0.46:
        return "FR", None
    if _RR_NAME_HINT.search(car_key or ""):
        return "RR", "name"
    return "MR", "weight"


_DRIVE_TYPE_BY_LAYOUT = {"FR": "RWD", "MR": "RWD", "RR": "RWD", "FF": "FWD"}


def drive_type(layout: str) -> Optional[str]:
    """vehicle_info.drive_type (FWD/RWD/AWD/4WD) from a layout code (FR/MR/RR/FF)."""
    if layout in ("FWD", "RWD", "AWD", "4WD"):
        return layout
    return _DRIVE_TYPE_BY_LAYOUT.get(layout)


# ─── Coordinate transform ────────────────────────────────────────────────────

def ac_to_svj(ac_x: float, ac_y: float, ac_z: float) -> list[float]:
    """AC (X-right, Y-up, Z-forward) → SAE J670 (X-fwd, Y-right, Z-down)."""
    return [round(ac_z, 4), round(ac_x, 4), round(-ac_y, 4)]


def ac_vec_to_svj(v: Optional[list[float]]) -> Optional[list[float]]:
    if v is None or len(v) < 3:
        return None
    return ac_to_svj(v[0], v[1], v[2])


def ac_hp_to_vehicle(ac_vec: Optional[list[float]], axle_x: float,
                     y_sign: int,
                     chassis_z_offset: float = 0.0,
                     wheel_y: float = 0.0) -> Optional[list[float]]:
    """
    AC *chassis-space* hardpoint (WBCAR_*, LINK_*_CAR, PANHARD_*, WATTS_*,
    TRAIL_*, SEMI_TRAIL_*, STRUT_CAR) → SVJ vehicle coords.

    Lateral convention (verified against MacPherson strut inclination):
    AC stores the X component of every chassis-space pickup as an
    *inboard-positive* offset from the wheel centre — the same convention
    as WBTYRE_*/STRUT_TYRE.  A value of 0.38 m means "0.38 m toward the
    car centreline from the wheel centre", NOT "0.38 m from the centreline".
    The correct lateral position in vehicle frame is therefore:

        svj_Y = (half_track − AC_X) × y_sign
              = wheel_y − AC_X × y_sign          (wheel_y already carries sign)

    Evidence: the Audi TT Cup MacPherson strut has STRUT_CAR X = 0.254 m,
    half-track = 0.81 m.  Treating it as centreline-absolute puts the top
    mount 505 mm inboard of the bottom (36° inclination — impossible).
    Treating it as wheel-relative gives 182 mm / 14.8° — correct for a strut.

    `wheel_y` = y_sign × track/2 (signed; +ve for right, −ve for left).
    All callers inside build_corner pass this.  Default 0.0 retained for
    any external caller that has not yet been updated (produces old behaviour
    for that caller only).

    The full transform:
      1. swap AC axes to J670 (ac.Z→svj.X, ac.X→svj.Y, −ac.Y→svj.Z),
      2. lateral Y: wheel_y − ac.X × y_sign   (inboard-positive → absolute),
      3. shift X by axle position (0 front, −wheelbase rear),
      4. shift Z by chassis_z_offset (wheel-centre-relative → ground-relative).
    """
    if ac_vec is None:
        return None
    svj = ac_to_svj(ac_vec[0], ac_vec[1], ac_vec[2])
    svj[0] = round(svj[0] + axle_x, 4)
    svj[1] = round(wheel_y - svj[1] * y_sign, 4)
    svj[2] = round(svj[2] + chassis_z_offset, 4)
    return svj


def ac_wheelspace_to_vehicle(ac_vec: Optional[list[float]],
                             wheel_center: list[float],
                             y_sign: int) -> Optional[list[float]]:
    """
    AC *wheel-space* hardpoint (WBTYRE_*, STRUT_TYRE) → SVJ vehicle coords.

    AC expresses upright-side hardpoints as small offsets from the wheel
    centre. In `suspensions.ini` the X component is stored *inboard-positive*
    (magnitude of displacement toward the chassis centreline), so the same
    number describes both sides of the car. Upper ball joints are therefore
    placed *inside* the wheel, not outside it.

    Transform:
      1. swap AC axes to J670 (ac_x → svj_y, ac_y → -svj_z, ac_z → svj_x),
      2. subtract the lateral offset in the wheel's outboard direction —
         i.e. toward the car centreline — via `-offset_y * y_sign`,
      3. add the forward (X) and vertical (Z) offsets directly to the
         wheel centre.
    """
    if ac_vec is None:
        return None
    offset = ac_to_svj(ac_vec[0], ac_vec[1], ac_vec[2])
    return [
        round(wheel_center[0] + offset[0], 4),
        round(wheel_center[1] - offset[1] * y_sign, 4),
        round(wheel_center[2] + offset[2], 4),
    ]


# ─── Damper / torque synth (fallback when .lut is absent) ────────────────────

def _setup_default(setup: dict, name: str) -> Optional[int]:
    """DEFAULT click of a setup.ini entry, or None."""
    try:
        return int(float(str((setup.get(name) or {}).get("DEFAULT", "")).split()[0]))
    except (ValueError, IndexError):
        return None


def _merge_slow_fast(slow: Optional[list], fast: Optional[list]) -> list:
    """Slow-speed table, continued by the fast-speed table beyond its last speed."""
    pts = [[abs(float(s)), abs(float(f))] for s, f in (slow or [])]
    if fast:
        last = pts[-1][0] if pts else -1.0
        pts += [[abs(float(s)), abs(float(f))] for s, f in fast if abs(float(s)) > last]
    pts.sort(key=lambda p: p[0])
    if pts and pts[0][0] > 0:
        pts.insert(0, [0.0, 0.0])
    return pts


def _csp_physics_flags(parsed: dict) -> dict:
    """The CSP extended-physics switches of a car, as written: every ``[_EXTENSION*]``
    section of the data files, and the ``EXT_*`` keys of an engine's turbo sections.
    Kept under ``x_assettocorsa.csp_physics`` so no option is lost."""
    out: dict = {}
    for fname in ("car.ini", "suspensions.ini", "tyres.ini", "brakes.ini", "drivetrain.ini",
                  "engine.ini", "electronics.ini", "aero.ini"):
        sections: dict = {}
        for sec, keys in (parsed.get(fname) or {}).items():
            name = sec.strip().upper()
            if name.startswith("_EXTENSION"):
                sections[name] = {k: v for k, v in keys.items()}
            else:
                ext = {k: v for k, v in keys.items() if k.upper().startswith("EXT_")}
                if ext:
                    sections[name] = ext
        if sections:
            out[fname] = sections
    return out


def _rounded(d: dict, nd: int) -> dict:
    """A copy of `d` with its floats (and lists of floats) rounded."""
    return {k: ([round(x, nd) for x in v] if isinstance(v, list)
                else round(v, nd) if isinstance(v, float) else v) for k, v in d.items()}


def _interp(curve: list, x: float) -> float:
    """Linear interpolation in a sorted ``[[x, y], ...]`` table (clamped at the ends)."""
    if x <= curve[0][0]:
        return curve[0][1]
    for (x0, y0), (x1, y1) in zip(curve, curve[1:]):
        if x <= x1:
            return y0 if x1 == x0 else y0 + (y1 - y0) * (x - x0) / (x1 - x0)
    return curve[-1][1]


def _attach_cosmic(sa_f: dict, sa_r: dict, parsed: dict, log: list) -> None:
    """CSP COSMIC axles: reduce the joint model to links, and the coilovers to a wheel
    rate (their RATE is a component rate, wheel rate = rate x motion ratio squared)."""
    from cosmic import reduce_cosmic
    ini = parsed.get("suspensions.ini") or {}
    for axle, sa in (("FRONT", sa_f), ("REAR", sa_r)):
        try:
            res = reduce_cosmic(ini, axle)
        except Exception as ex:                                  # noqa: BLE001
            log.append(f"⚠ COSMIC {axle.lower()} not converted: {ex}")
            continue
        if res is None:
            continue
        sa["cosmic"] = res
        sprung = [c for c in res["coilovers"] if c["rate"] > 0]
        if sprung and all(c["wheel_rate"] for c in sprung):
            sa["spring_rate"] = round(sum(c["wheel_rate"] for c in sprung)
                                      + (res["extra_wheel_rate"] or 0.0), 1)
        elif res["extra_wheel_rate"] and not sprung:
            sa["spring_rate"] = res["extra_wheel_rate"]
        log.append(f"✓ COSMIC suspension ({axle.lower()}): {res['system_type']}, "
                   f"{len(res['links'])} link(s), {len(res['coilovers'])} coilover(s)"
                   + (f"; {len(res['notes'])} part(s) not converted" if res["notes"] else ""))


def _attach_arb(sa_f: dict, sa_r: dict, parsed: dict, log: list) -> None:
    """Anti-roll bars live in the global ``[ARB]`` section of suspensions.ini
    (``FRONT`` / ``REAR``; with CSP ``EXTEND=1`` also ``FRONT_MOTION_RATIO`` /
    ``REAR_MOTION_RATIO``), not in the axle sections."""
    arb = (parsed.get("suspensions.ini") or {}).get("ARB") or {}
    got = []
    for axle, sa in (("FRONT", sa_f), ("REAR", sa_r)):
        k = _f(arb.get(axle), 0.0)
        if k > 0:
            sa["arb_rate"] = k
            got.append(axle.lower())
        mr = _f(arb.get(f"{axle}_MOTION_RATIO"), 0.0)
        if mr > 0:
            sa["arb_motion_ratio"] = mr
    if got:
        log.append(f"✓ Anti-roll bar ({', '.join(got)}): [ARB] of suspensions.ini")


def _attach_csp_tables(sa_f: dict, sa_r: dict, parsed: dict, data_dir,
                       log: list) -> None:
    """Put CSP damper / bump-stop tables on the parsed axle dicts (``damper_tables``,
    ``bumpstop_table``) so ``build_corner`` can use them."""
    setup = parsed.get("setup.ini", {})
    dt = parse_damper_tables(parsed.get("dampers.ini", {}), data_dir)
    bt = parse_bumpstop_tables(parsed.get("bumpstops.ini", {}), data_dir)
    for axle, sa, corners in (("FRONT", sa_f, ("LF", "RF")), ("REAR", sa_r, ("LR", "RR"))):
        # ── motion-ratio table: deflection from the zero point | ratio ───────
        if sa.get("motion_ratio_lut"):
            mrt = _resolve_lut(data_dir, sa["motion_ratio_lut"])
            if mrt and len(mrt) >= 2:
                mrt.sort(key=lambda p: p[0])
                if all(p[1] > 0 for p in mrt):
                    sa["motion_ratio_table"] = mrt
                    log.append(f"✓ Motion-ratio table ({axle.lower()}): "
                               f"{len(mrt)} pts, {_interp(mrt, 0.0):.3f} at the zero point")
        # ── damper tables ────────────────────────────────────────────────────
        ent = (dt.get("axles") or {}).get(axle)
        if ent:
            t = ent["tables"]
            sfx = f"_C{ent['coilover']}" if ent["coilover"] is not None else ""
            idx_b = idx_r = None
            for c in corners:
                idx_b = idx_b if idx_b is not None else _setup_default(setup, f"DAMPER_BUMP_{c}{sfx}")
                idx_r = idx_r if idx_r is not None else _setup_default(setup, f"DAMPER_REBOUND_{c}{sfx}")

            def pick(kind: str, want: Optional[int]):
                d = t.get(kind) or {}
                if not d:
                    return None, None
                i = want if want in d else min(d)
                return d[i], i

            bs, ib = pick("bump_slow", idx_b)
            rs, ir = pick("rebound_slow", idx_r)
            bf, _ = pick("bump_fast", ib)
            rf, _ = pick("rebound_fast", ir)
            bump = _merge_slow_fast(bs, bf)
            reb = _merge_slow_fast(rs, rf)
            if len(bump) >= 2 and len(reb) >= 2:
                sa["damper_tables"] = {
                    "bump": bump, "rebound": reb, "bump_index": ib, "rebound_index": ir,
                    "coilover": ent["coilover"],
                    "settings": {k: sorted(v) for k, v in t.items()},
                }
                n = max(len(v) for v in t.values())
                log.append(f"✓ Damper tables ({axle.lower()}): dampers.ini, "
                           f"setting {ib}/{ir} of {n}")
        # ── bump-stop table (dampers.ini sibling, or the older BUMP_STOP_LUT) ──
        be = bt.get(axle)
        if be:
            i = be["default"] if be["default"] in be["luts"] else min(be["luts"])
            sa["bumpstop_table"] = {"curve": be["luts"][i], "stack": be["stack"],
                                    "index": i, "source": "bumpstops.ini"}
        elif sa.get("bump_stop_lut") and data_dir:
            lut = _resolve_lut(data_dir, sa["bump_stop_lut"])
            # a force table starts at 0 | 0 and grows with the deflection
            if lut and len(lut) >= 2 and lut[0][0] >= 0 and lut[0][1] == 0                     and all(b[0] > a_[0] for a_, b in zip(lut, lut[1:])):
                sa["bumpstop_table"] = {"curve": lut, "stack": 1, "index": 0,
                                        "source": sa["bump_stop_lut"]}
        if sa.get("bumpstop_table"):
            log.append(f"✓ Bump-stop table ({axle.lower()}): {sa['bumpstop_table']['source']}")


def make_damper_curve(rate: float, factor: float = 1.0) -> list:
    r = rate * factor
    return [
        [0.000, 0],
        [0.025, round(r * 0.25 * 0.025, 1)],
        [0.050, round(r * 0.50 * 0.05, 1)],
        [0.100, round(r * 0.80 * 0.10, 1)],
        [0.200, round(r * 1.00 * 0.20, 1)],
        [0.500, round(r * 1.10 * 0.50, 1)],
    ]


def synth_torque_curve(idle_rpm: float, max_rpm: float,
                       cm_bhp: Optional[float]) -> list:
    if cm_bhp and cm_bhp > 0:
        peak_rpm    = round(max_rpm * 0.75 / 100) * 100
        peak_torque = round(cm_bhp * 745.7 / (peak_rpm * math.pi / 30), 1)
        return [
            [round(idle_rpm),      round(peak_torque * 0.55, 1)],
            [round(idle_rpm * 2),  round(peak_torque * 0.72, 1)],
            [round(max_rpm * 0.4), round(peak_torque * 0.88, 1)],
            [round(max_rpm * 0.6), round(peak_torque * 0.98, 1)],
            [round(peak_rpm),      peak_torque],
            [round(max_rpm * 0.9), round(peak_torque * 0.90, 1)],
            [round(max_rpm),       round(peak_torque * 0.75, 1)],
        ]
    return [[round(idle_rpm), 80.0], [round(max_rpm), 60.0]]


# ─── Corner builder (now consumes parsed suspension-axle block) ──────────────

def build_corner(corner_id: str, susp_axle: dict,
                 wheelbase: float, track_front: float, track_rear: float,
                 disc_inertia, disc_radius, max_torque, brake_bias,
                 tire_set_ref: str, rolling_radius: float = 0.306,
                 cg_height: float = 0.42,
                 tire_sets: Optional[dict] = None) -> dict:
    """
    corner_id : "FL" | "FR" | "RL" | "RR"
    SVJ hardpoints are expressed in VEHICLE coordinates (J670 origin at the
    front-axle mid-plane, X forward, Y right, Z down).

    AC suspensions.ini gives points in each axle's local frame with the right
    side positive; we mirror Y for left and offset X by −wheelbase for the
    rear axle.
    """
    is_front = corner_id.startswith("F")
    is_right = corner_id.endswith("R")
    y_sign   = 1 if is_right else -1
    axle_x   = 0.0 if is_front else -float(wheelbase)
    track    = float(track_front) if is_front else float(track_rear)
    wheel_y  = round(y_sign * track / 2.0, 4)

    sys_type = susp_type(susp_axle.get("type", "0"))
    cos = susp_axle.get("cosmic")
    if cos:
        sys_type = cos["system_type"]
    spring_k = susp_axle.get("spring_rate") or (25000.0 if is_front else 22000.0)
    bump_raw = susp_axle.get("bump_rate")    or (2500.0  if is_front else 2200.0)
    reb_raw  = susp_axle.get("rebound_rate") or (3000.0  if is_front else 2800.0)
    arb_k    = susp_axle.get("arb_rate")     or (8000.0  if is_front else 6000.0)
    ride_h   = susp_axle.get("ride_height") or 0.05
    camber   = susp_axle.get("camber")      if susp_axle.get("camber") is not None else -0.02
    toe      = susp_axle.get("toe")         if susp_axle.get("toe") is not None else 0.0
    caster   = susp_axle.get("caster")      if is_front else None
    kpi      = susp_axle.get("kpi")         if is_front else None

    disc_mass = None
    if disc_inertia is not None and disc_radius and disc_radius > 0:
        disc_mass = round(2 * disc_inertia / disc_radius ** 2, 2)

    disc_obj = {"type": "vented",
                "outer_diameter": round((disc_radius or 0.28) * 2, 4)}
    if disc_radius:
        disc_obj["effective_radius"] = round(disc_radius * 0.85, 4)
    if disc_mass is not None:
        disc_obj["mass"] = disc_mass
    if disc_inertia is not None:
        disc_obj["rotational_inertia"] = round(disc_inertia, 5)

    # ── Hardpoints in VEHICLE frame ──────────────────────────────────────────
    hp_ac = susp_axle.get("hardpoints_ac") or {}

    # Wheel centre in vehicle coords. SVJ J670 convention (matching every
    # reference example in svjrepo/{examples,templates}) puts the origin at
    # the ground contact patch — so wheel_center.Z = -rolling_radius
    # (Z-down: negative Z is UP, positive is DOWN). All wheel-space offsets
    # (WBTYRE_*, STRUT_TYRE, spring/damper outboards, etc.) cascade from
    # this value and get the same global shift.
    wheel_center = [round(axle_x, 4), wheel_y, round(-float(rolling_radius), 4)]

    # AC chassis-space pickups (WBCAR_*, LINK_*_CAR, PANHARD_*, WATTS_*,
    # TRAIL_*, SEMI_TRAIL_*) — Kunos's documentation says these live in
    # a chassis frame whose vertical origin is the centre of gravity,
    # but in practice (verified across 60+ mods including the Caterham
    # 165 LHD where CG=0.42 lifted chassis pivots ~17 cm too high) AC
    # mod authors place WBCAR values relative to the WHEEL CENTRE.
    # Anchoring at -rolling_radius gives physically correct geometry:
    # lower wishbone inboard ~14 cm above ground vs ~13 cm at the BJ,
    # i.e. roughly level. Anchoring at -cg_height makes the lower
    # wishbone slope up from wheel to chassis (lower-wishbone chassis
    # pivot ends up *above* the lower ball joint) which is the
    # "upper/lower swap" symptom the user reported in the viewer.
    chassis_z_offset = -round(float(rolling_radius), 4)
    _ = cg_height  # not used for hardpoint placement; feeds the anti-dive / anti-squat estimate

    hp_svj: dict = {"wheel_center": wheel_center}
    # WBTYRE_* / STRUT_TYRE are wheel-space offsets in AC, so they must be
    # added to the wheel centre (after axis swap + Y mirror). WBCAR_* are
    # chassis-space absolutes.
    for src, dst in [("WBTYRE_TOP",    "upper_ball_joint"),
                     ("WBTYRE_BOTTOM", "lower_ball_joint"),
                     ("WBTYRE_STEER", "steering_tie_rod_end")]:
        v = ac_wheelspace_to_vehicle(hp_ac.get(src), wheel_center, y_sign)
        if v:
            hp_svj[dst] = v
    strut_tyre = ac_wheelspace_to_vehicle(hp_ac.get("STRUT_TYRE"),
                                          wheel_center, y_sign)
    if strut_tyre:
        hp_svj["damper_outboard"] = strut_tyre
        # On a true MacPherson axle the strut bottom IS the upper attachment
        # of the upright (there is no separate upper ball joint). Expose it
        # as a dedicated `strut_outboard` hardpoint so 3D viewers that walk
        # the upright hardpoints can render the strut as a visible member;
        # `damper_outboard` is retained for back-compat with v0.99 consumers.
        if sys_type == "macpherson":
            hp_svj["strut_outboard"] = strut_tyre
            hp_svj["strut_lower"] = strut_tyre        # lower clamp on the upright (spec 0.99.4)

    # Control-arm inboard points in vehicle frame (chassis-space).
    # `chassis_z_offset` lifts each WBCAR pickup from CG-relative to
    # ground-relative; see ac_hp_to_vehicle docstring.
    upper_front = ac_hp_to_vehicle(hp_ac.get("WBCAR_TOP_FRONT"),    axle_x, y_sign, chassis_z_offset, wheel_y)
    upper_rear  = ac_hp_to_vehicle(hp_ac.get("WBCAR_TOP_REAR"),     axle_x, y_sign, chassis_z_offset, wheel_y)
    lower_front = ac_hp_to_vehicle(hp_ac.get("WBCAR_BOTTOM_FRONT"), axle_x, y_sign, chassis_z_offset, wheel_y)
    lower_rear  = ac_hp_to_vehicle(hp_ac.get("WBCAR_BOTTOM_REAR"),  axle_x, y_sign, chassis_z_offset, wheel_y)
    tie_chassis = ac_hp_to_vehicle(hp_ac.get("WBCAR_STEER")
                                   or hp_ac.get("WBCAR_TIE"),         axle_x, y_sign, chassis_z_offset, wheel_y)
    strut_car   = ac_hp_to_vehicle(hp_ac.get("STRUT_CAR"),           axle_x, y_sign, chassis_z_offset, wheel_y)
    if strut_car and "strut_lower" in hp_svj:
        hp_svj["strut_top"] = strut_car          # body-side end of the strut (spec 0.99.4)

    links: list = []
    is_axle = sys_type == "solid_axle"
    # AC type STRUT has no upper arm: a WBCAR_TOP_* left in the file is unused
    # boilerplate, and an upper arm on a MacPherson over-constrains the corner.
    if upper_front and upper_rear and not is_axle and sys_type != "macpherson":
        links.append({
            # "wishbone" is reserved for true double-A-arm geometry; multi-link
            # and MacPherson equivalents use "control_arm" (matches svjrepo
            # reference templates).
            "name": ("upper_wishbone" if sys_type == "double_wishbone" else "trailing_arm_upper" if sys_type == "trailing_arm" else "semi_trailing_arm_upper" if sys_type == "semi_trailing_arm" else "upper_control_arm"),
            "type": "arm",
            "inboard_points": [upper_front, upper_rear],
            "outboard_ref": "hardpoints.upper_ball_joint",
        })
    if is_axle and upper_front and upper_rear and hp_svj.get("upper_ball_joint"):
        # AC type AXLE reuses the two-pivot WBCAR_* sets as the axle's locating links.
        # SVJ solid_axle wants one rod per link (§9.2.1): chassis end = midpoint of
        # the two pivots, axle end = the matching WBTYRE_* point.
        for nm, a, b, hp in (("upper_link", upper_front, upper_rear, "upper_link_end"),
                             ("lower_link", lower_front, lower_rear, "lower_link_end")):
            if not (a and b):
                continue
            hp_svj[hp] = hp_svj["upper_ball_joint" if nm == "upper_link" else "lower_ball_joint"]
            links.append({
                "name": nm, "type": "rod",
                "inboard_points": [[round((a[i] + b[i]) / 2.0, 4) for i in range(3)]],
                "outboard_ref": f"hardpoints.{hp}",
                "_est": True,
                "_est_note": "AC TYPE=AXLE: two-pivot WBCAR_* pickups collapsed to one "
                             "rod at their midpoint (solid_axle, spec 9.2.1)",
            })
    elif lower_front and lower_rear and not is_axle:
        links.append({
            "name": ("lower_wishbone" if sys_type == "double_wishbone" else "trailing_arm_lower" if sys_type == "trailing_arm" else "semi_trailing_arm_lower" if sys_type == "semi_trailing_arm" else "lower_control_arm"),
            "type": "arm",
            "inboard_points": [lower_front, lower_rear],
            "outboard_ref": "hardpoints.lower_ball_joint",
        })
    if tie_chassis and hp_svj.get("steering_tie_rod_end") and not is_axle:
        # AC's WBCAR_STEER / WBTYRE_STEER define the toe-control link's
        # inboard / outboard pickups. On the front axle this IS the steering
        # tie rod; on the rear it's a passive toe link (no steering box on
        # the rear axle of any AC car). Rename the link AND its outboard
        # hardpoint on the rear so SVJ consumers don't think the rear steers.
        if is_front:
            links.append({
                "name": "steering_tie_rod",
                "type": "rod",
                "inboard_points": [tie_chassis],
                "outboard_ref": "hardpoints.steering_tie_rod_end",
            })
        else:
            # Re-key the outboard hardpoint as `toe_link_outer` (keep
            # `steering_tie_rod_end` too as a back-compat alias for v0.99
            # consumers, both at the same coordinates).
            tle = hp_svj["steering_tie_rod_end"]
            hp_svj["toe_link_outer"] = tle
            links.append({
                "name": "toe_link",
                "type": "rod",
                "inboard_points": [tie_chassis],
                "outboard_ref": "hardpoints.toe_link_outer",
            })
    if strut_car and hp_svj.get("damper_outboard"):
        # AC STRUT_* pickup represents the strut tower → upright mount. On a
        # true MacPherson axle this IS the strut, so emit it under a clearer
        # name and reference the dedicated `strut_outboard` hardpoint. On
        # non-strut axles (rare — copy-pasted boilerplate) we keep the
        # legacy `damper_link` name and `damper_outboard` ref.
        if sys_type == "macpherson":
            links.append({
                "name": "strut",
                "type": "strut",
                "inboard_points": [strut_car],
                "outboard_ref": "hardpoints.strut_lower",
            })
        # On any other axle STRUT_* is not a locating member (a damper does not
        # locate the wheel): it stays out of `links`, kept under damper.x_ac_strut.

    # ── Solid-axle / live-axle links (TYPE=3) ────────────────────────────────
    # AC solid-axle cars use LINK_N_CAR / LINK_N_AXLE pairs for trailing and
    # diagonal links, plus PANHARD_* or WATTS_* for lateral location. These
    # points live in the axle-local chassis frame, so ac_hp_to_vehicle
    # (same transform as WBCAR_*) gives the correct vehicle coordinates.
    if sys_type == "solid_axle":
        for n in range(4):
            car_ac  = hp_ac.get(f"LINK_{n}_CAR")
            axle_ac = hp_ac.get(f"LINK_{n}_AXLE")
            if car_ac is None or axle_ac is None:
                continue
            car_v  = ac_hp_to_vehicle(car_ac,  axle_x, y_sign, chassis_z_offset, wheel_y)
            axle_v = ac_hp_to_vehicle(axle_ac, axle_x, y_sign, chassis_z_offset, wheel_y)
            hp_svj[f"axle_link_{n}_outer"] = axle_v
            links.append({
                "name": f"trailing_link_{n}",
                "type": "rod",
                "inboard_points": [car_v],
                "outboard_ref": f"hardpoints.axle_link_{n}_outer",
                "_est_note": "AC LINK_{n}_CAR/AXLE pair — trailing or "
                             "diagonal link; role depends on car".format(n=n),
            })
        pan_car  = hp_ac.get("PANHARD_CAR")
        pan_axle = hp_ac.get("PANHARD_AXLE")
        if pan_car and pan_axle:
            pc = ac_hp_to_vehicle(pan_car,  axle_x, y_sign, chassis_z_offset, wheel_y)
            pa = ac_hp_to_vehicle(pan_axle, axle_x, y_sign, chassis_z_offset, wheel_y)
            hp_svj["panhard_axle_end"] = pa
            links.append({
                "name": "panhard_rod",
                "type": "rod",
                "inboard_points": [pc],
                "outboard_ref": "hardpoints.panhard_axle_end",
                "_est_note": "Panhard rod is a single member shared between "
                             "L/R corners — emitted on both for schema symmetry.",
            })
        wt_car  = hp_ac.get("WATTS_CAR")
        wt_axle = hp_ac.get("WATTS_AXLE")
        if wt_car and wt_axle:
            wc_v = ac_hp_to_vehicle(wt_car,  axle_x, y_sign, chassis_z_offset, wheel_y)
            wa_v = ac_hp_to_vehicle(wt_axle, axle_x, y_sign, chassis_z_offset, wheel_y)
            hp_svj["watts_axle_end"] = wa_v
            links.append({
                "name": "watts_link",
                "type": "rod",
                "inboard_points": [wc_v],
                "outboard_ref": "hardpoints.watts_axle_end",
                "_est_note": "Watts-linkage lateral-location rod.",
            })

    # ── Trailing-arm / Semi-trailing-arm explicit pickups (TYPE=4 / TYPE=5) ──
    # Optional: some AC mods expose a dedicated single pivot pair instead of
    # (or in addition to) the WBCAR_TOP/BOTTOM pair. If present, emit the arm
    # as a discrete link; the WBCAR-derived upper/lower arms still provide
    # structural pickups for the upright.
    if sys_type == "trailing_arm":
        tc = hp_ac.get("TRAIL_CAR")
        ta = hp_ac.get("TRAIL_AXLE")
        if tc and ta:
            tc_v = ac_hp_to_vehicle(tc, axle_x, y_sign, chassis_z_offset, wheel_y)
            ta_v = ac_hp_to_vehicle(ta, axle_x, y_sign, chassis_z_offset, wheel_y)
            hp_svj["trailing_arm_pivot"] = tc_v
            hp_svj["trailing_arm_outer"] = ta_v
            links.append({
                "name": "trailing_arm",
                "type": "arm",
                "inboard_points": [tc_v],
                "outboard_ref": "hardpoints.trailing_arm_outer",
                "_est_note": "AC TRAIL_CAR/AXLE pair — single-member trailing arm.",
            })
    if sys_type == "semi_trailing_arm":
        sc = hp_ac.get("SEMI_TRAIL_CAR")
        sa = hp_ac.get("SEMI_TRAIL_AXLE")
        if sc and sa:
            sc_v = ac_hp_to_vehicle(sc, axle_x, y_sign, chassis_z_offset, wheel_y)
            sa_v = ac_hp_to_vehicle(sa, axle_x, y_sign, chassis_z_offset, wheel_y)
            hp_svj["semi_trailing_pivot"] = sc_v
            hp_svj["semi_trailing_outer"] = sa_v
            links.append({
                "name": "semi_trailing_arm",
                "type": "arm",
                "inboard_points": [sc_v],
                "outboard_ref": "hardpoints.semi_trailing_outer",
                "_est_note": "AC SEMI_TRAIL_CAR/AXLE pair — diagonal pivot single arm.",
            })

    # ── Multi-link synthesis (TYPE=2, rear only) ─────────────────────────────
    # AC only models two pickups per arm; a "true" multi-link has independent
    # toe, upper-lateral and lower-lateral rods. Synthesize those as `_est`
    # placeholders so downstream SVJ consumers see a complete multi-link
    # topology. All offsets follow the MX-5 reference template pattern.
    if sys_type == "multi_link" and not is_front:
        upper_bj = hp_svj.get("upper_ball_joint")
        lower_bj = hp_svj.get("lower_ball_joint")
        if upper_bj and lower_bj:
            # Extra outboard hardpoints (all _est in the upright node).
            upper_lateral_end = [
                round(upper_bj[0] - 0.04, 4),
                round(upper_bj[1] + 0.01 * y_sign, 4),
                round(upper_bj[2] + 0.02, 4),
            ]
            lower_lateral_end = [
                round(lower_bj[0] + 0.04, 4),
                round(lower_bj[1] - 0.01 * y_sign, 4),
                round(lower_bj[2] - 0.02, 4),
            ]
            toe_link_end = [
                round(wheel_center[0] + 0.08, 4),
                round(wheel_center[1] + 0.19 * -y_sign, 4),
                round(wheel_center[2] + 0.14, 4),
            ]
            hp_svj["upper_lateral_end"] = upper_lateral_end
            hp_svj["lower_lateral_end"] = lower_lateral_end
            hp_svj["toe_link_end"]      = toe_link_end
            # CV-joint outer is coincident with wheel centre in reference templates.
            hp_svj["cv_joint_outer"]    = list(wheel_center)

            # Extra link inboards: reuse the rear chassis pickup of the
            # respective arm, offset slightly inboard for the lateral rods;
            # toe-link inboard sits forward and far inboard of the wheel.
            if upper_rear:
                ul_inboard = [
                    round(upper_rear[0] - 0.04, 4),
                    round(upper_rear[1] + 0.02 * -y_sign, 4),
                    round(upper_rear[2] + 0.02, 4),
                ]
                links.append({
                    "name": "upper_lateral_link",
                    "type": "rod",
                    "inboard_points": [ul_inboard],
                    "outboard_ref": "hardpoints.upper_lateral_end",
                    "_est": True,
                    "_est_note": "Synthesised from AC upper WBCAR rear "
                                 "pickup — AC models multi-link as two "
                                 "2-pickup arms; real multi-link splits this "
                                 "into independent rods.",
                })
            if lower_rear:
                ll_inboard = [
                    round(lower_rear[0] + 0.04, 4),
                    round(lower_rear[1] - 0.02 * -y_sign, 4),
                    round(lower_rear[2] - 0.02, 4),
                ]
                links.append({
                    "name": "lower_lateral_link",
                    "type": "rod",
                    "inboard_points": [ll_inboard],
                    "outboard_ref": "hardpoints.lower_lateral_end",
                    "_est": True,
                    "_est_note": "Synthesised from AC lower WBCAR rear pickup.",
                })
            # Toe link: inboard fixed-guess forward of axle, far inboard.
            toe_inboard = [
                round(wheel_center[0] + 0.08, 4),
                round(wheel_center[1] + 0.58 * -y_sign, 4),
                round(wheel_center[2] + 0.14, 4),
            ]
            links.append({
                "name": "toe_link",
                "type": "rod",
                "inboard_points": [toe_inboard],
                "outboard_ref": "hardpoints.toe_link_end",
                "_est": True,
                "_est_note": "Synthesised toe link — AC suspension.ini does "
                             "not model a rear toe link; placeholder location.",
            })

    # ── COSMIC: links reduced from the joint model ───────────────────────────
    cos_coil = None
    if cos:
        _v = lambda p: ac_wheelspace_to_vehicle(p, wheel_center, y_sign)
        for L in cos["links"]:
            hp_svj[L["hardpoint"]] = _v(L["end"])
            links.append({"name": L["name"], "type": L["type"],
                          "inboard_points": [_v(p) for p in L["inboard"]],
                          "outboard_ref": f"hardpoints.{L['hardpoint']}"})
            if L["type"] == "strut":
                hp_svj["strut_top"] = _v(L["inboard"][0])
                hp_svj["strut_outboard"] = hp_svj["damper_outboard"] = _v(L["end"])
        # the coilover that carries the damper: the one the damper table names,
        # else the one with the highest rate
        _want = (susp_axle.get("damper_tables") or {}).get("coilover")
        _cs = cos["coilovers"]
        cos_coil = next((c for c in _cs if c["index"] == _want), None) \
            or (max(_cs, key=lambda c: c["rate"]) if _cs else None)
        if cos_coil and not cos_coil["motion_ratio"]:
            cos_coil = None

    # ── Spring / damper: vertical at the wheel ───────────────────────────────
    # AC uses a wheel-rate spring model without modelled pickup points, so the
    # SVJ representation is a purely vertical axis through the wheel centre:
    #   inboard  = wheel_center + (0, 0, -tower_height)    (top of strut)
    #   outboard = wheel_center                            (hub)
    # Motion ratio is 1:1 (AC stores wheel rate directly).
    tower_h = 0.35 if is_front else 0.30  # metres, representative
    spring_inboard  = [wheel_center[0], wheel_center[1],
                       round(wheel_center[2] - tower_h, 4)]
    spring_outboard = wheel_center

    # MacPherson: the strut IS the damper, so its real mounts (STRUT_CAR -> STRUT_TYRE)
    # give the axis and the motion ratio (|strut_z| / strut length).
    strut_mr = None
    if cos_coil:
        spring_inboard = ac_wheelspace_to_vehicle(cos_coil["inboard"], wheel_center, y_sign)
        spring_outboard = ac_wheelspace_to_vehicle(cos_coil["outboard"], wheel_center, y_sign)
        strut_mr = cos_coil["motion_ratio"]
    elif sys_type == "macpherson" and strut_car and strut_tyre:
        _dx = [strut_car[i] - strut_tyre[i] for i in range(3)]
        _len = math.sqrt(sum(c * c for c in _dx))
        if _len > 0.05:
            _mr = abs(_dx[2]) / _len
            if 0.5 <= _mr <= 1.0:
                strut_mr = round(_mr, 4)
                spring_inboard, spring_outboard = list(strut_car), list(strut_tyre)

    spring = {
        "type":          "coilover",
        "rate":          round(spring_k, 1),
        "motion_ratio":  strut_mr or 1.0,
        "axis":          "strut" if strut_mr else "vertical",
        "inboard_mount":  spring_inboard,
        "outboard_mount": spring_outboard,
        "_note": ("AC spring rate is already wheel-rate; mounts are the strut's "
                  "STRUT_CAR / STRUT_TYRE, motion ratio from the strut inclination"
                  if strut_mr else
                  "AC spring rate is already wheel-rate — axis taken as vertical "
                  "through the wheel centre (AC does not model pickup points)"),
    }
    if susp_axle.get("progressive"):
        spring["progressive_factor"] = susp_axle["progressive"]
    if susp_axle.get("rod_length"):
        spring["rod_length"] = susp_axle["rod_length"]

    _tab = susp_axle.get("damper_tables")
    _bump_c = make_damper_curve(bump_raw)
    _reb_c = make_damper_curve(reb_raw, factor=1.5)
    if _tab:
        # real force-against-speed tables (CSP): speed in m/s, force in N, sign dropped
        _bump_c = [[round(v, 5), round(f, 2)] for v, f in _tab["bump"]]
        _reb_c = [[round(v, 5), round(f, 2)] for v, f in _tab["rebound"]]
    elif strut_mr:
        # AC damping is given at the wheel; the curves are on the damper: v*MR, F/MR
        _bump_c = [[round(v * strut_mr, 5), round(f / strut_mr, 2)] for v, f in _bump_c]
        _reb_c = [[round(v * strut_mr, 5), round(f / strut_mr, 2)] for v, f in _reb_c]
    damper = {
        "type":           "monotube",
        "bump_curve":     _bump_c,
        "rebound_curve":  _reb_c,
        "motion_ratio":   strut_mr or 1.0,
        "axis":           "strut" if strut_mr else "vertical",
        "inboard_mount":  spring_inboard,
        "outboard_mount": spring_outboard,
        "_est":           True,
        "_note": ("AC damping is specified at wheel; curves synthesised from AC scalar "
                  "bump/rebound and converted to the strut axis (v*MR, F/MR)"
                  if strut_mr else
                  "AC damping is specified at wheel — vertical axis at wheel centre, "
                  "1:1 motion ratio. Curves synthesised from AC scalar bump/rebound."),
    }
    if _tab:
        damper["_est"] = False
        damper["_note"] = ("Force-against-speed tables from dampers.ini (CSP), setting "
                           f"{_tab['bump_index']}/{_tab['rebound_index']}; "
                           + ("component-level (COSMIC coilover): motion ratio not derived"
                              if _tab.get("coilover") is not None else
                              "wheel level (no motion-ratio table)"))
        damper["reference"] = "damper" if _tab.get("coilover") is not None else "wheel"
        if damper["reference"] == "wheel":
            damper["motion_ratio"] = 1.0
        damper["x_settings"] = _tab["settings"]
        if susp_axle.get("motion_ratio_table"):
            # CSP applies the motion ratio to the damper tables: they are damper level
            damper["reference"] = "damper"
            damper["_note"] = ("Force-against-speed tables from dampers.ini (CSP), setting "
                               f"{_tab['bump_index']}/{_tab['rebound_index']}; damper level, "
                               "converted to the wheel with the motion-ratio table")
    if strut_car and strut_tyre and sys_type != "macpherson":
        damper["x_ac_strut"] = {"car": strut_car, "tyre": strut_tyre,
                                "_note": "STRUT_* on a non-strut axle: not a locating link"}
    bump = susp_axle.get("bumpstop")
    if bump:
        damper["bumpstops"] = bump

    geometry_note = None
    if not links:
        geometry_note = ("No usable hardpoints in suspensions.ini (type "
                         f"{susp_axle.get('type')}) - links left empty; spring/damper only")
    elif is_front and sys_type in ("double_wishbone", "macpherson") \
            and not any(L["name"] == "steering_tie_rod" for L in links):
        geometry_note = ("No steering tie rod: WBCAR_STEER / WBTYRE_STEER missing or "
                         "unreadable in suspensions.ini")
    elif not is_front and sys_type == "macpherson" \
            and not any(L["name"] == "toe_link" for L in links):
        geometry_note = "No toe link: WBCAR_STEER / WBTYRE_STEER missing in suspensions.ini"

    # Resolve every link's `outboard_ref` to a literal `outboard_point`
    # alongside it, so SVJ viewers that don't walk reference strings still
    # render the link correctly. Refs of the form "hardpoints.<name>" look
    # up the upright hardpoint we just built.
    for L in links:
        ref = L.get("outboard_ref")
        if ref and ref.startswith("hardpoints.") and "outboard_point" not in L:
            hp_name = ref.split(".", 1)[1]
            target  = hp_svj.get(hp_name)
            if target is not None:
                L["outboard_point"] = list(target)

    if cos and cos["notes"]:
        geometry_note = ((geometry_note + "; ") if geometry_note else "") + \
            "COSMIC, not converted: " + "; ".join(cos["notes"])
    topology = {
        "system_type": sys_type,
        "upright": {
            "id": f"upright_{corner_id}",
            "hardpoints": hp_svj,
        },
        "links": links,
    }
    if is_axle:
        # spec 9.2.4: a solid axle shares one rigid body between both corners
        topology["axle_body"] = {
            "id": "axle_front" if is_front else "axle_rear",
            "position": [wheel_center[0], 0.0, wheel_center[2]],
        }
        if any(L["name"] in ("panhard_rod", "watts_link") for L in links):
            topology["lateral_location"] = ("watts_link" if any(
                L["name"] == "watts_link" for L in links) else "panhard_rod")
    if geometry_note:
        topology["_note"] = geometry_note

    _tw = ((tire_sets or {}).get(tire_set_ref, {}).get("rim", {}) or {}).get("width_nominal")
    _est_rim_w = round(float(_tw) * 0.8, 3) if _tw else 0.18
    _arb = {"bar_id": "arb_front" if is_front else "arb_rear",
            "bar_rate": round(arb_k, 1)}
    if susp_axle.get("arb_rate"):
        _arb["_note"] = ("Stiffness as written in the [ARB] section of suspensions.ini "
                         "(AC / CSP document it in N/m); not converted to Nm/rad")
        if susp_axle.get("arb_motion_ratio"):
            _arb["motion_ratio"] = susp_axle["arb_motion_ratio"]
    else:
        _arb["_est"] = True                      # no [ARB] values: placeholder
    corner = {
        "position":        wheel_center,   # corner location in vehicle frame
        "topology":        topology,
        "wheel": {
            "rim_diameter": round(
                (tire_sets or {}).get(tire_set_ref, {}).get("rim", {}).get("diameter", 0.432),
                4),
            # AC has no rim width; the schema requires one, so estimate 0.8 x the tyre
            # section width (typical for road and race tyres) and flag it.
            "rim_width":     _est_rim_w,
            "_est_rim_width": True,
            "set_ref":      tire_set_ref,
        },
        "spring": spring,
        "damper": damper,
        "arb": _arb,
        "alignment": {
            "camber": round(camber, 5),
            "toe":    round(toe, 5),
        },
        "brake": {"disc": disc_obj,
                  "max_torque_nm": round(max_torque, 1)},
        "tire": {"set_ref": tire_set_ref},
        # BASEY is the wheel-centre offset from the physics origin, not a ride height:
        # only a positive value is one.
        "static_setup": {"ride_height": round(ride_h, 4)} if ride_h > 0 else {},
    }
    if caster is not None:
        corner["alignment"]["caster"] = round(caster, 5)
    if kpi is not None:
        corner["alignment"]["kpi"] = round(kpi, 5)

    _geo = corner_geometry(topology, wheel_center, y_sign, is_front,
                           float(wheelbase), float(cg_height), float(brake_bias))
    if _geo:
        _align = corner["alignment"]
        _kp = _geo.pop("kingpin", None)
        if _kp:
            _align["caster_trail"] = round(_kp["mechanical_trail"], 5)
            _align["scrub_radius"] = round(_kp["scrub_radius"], 5)
        _align["x_geometry"] = _rounded(_geo, 4)
        if _kp:
            _align["x_geometry"]["kingpin"] = _rounded(_kp, 5)
        _align["x_geometry_note"] = (
            "Derived from the hardpoints at the static position (front/side-view instant "
            "centres, outboard brakes); an estimate, not a value from AC")

    _mrt = susp_axle.get("motion_ratio_table")
    if _mrt:
        _mr0 = round(_interp(_mrt, 0.0), 4)          # at the zero point (static)
        _curve = [[round(d, 5), round(r, 4)] for d, r in _mrt]
        corner["spring"]["motion_ratio"] = _mr0       # geometry; `rate` stays the wheel rate
        corner["spring"]["x_motion_ratio_curve"] = _curve
        corner["damper"]["x_motion_ratio_curve"] = _curve
        if corner["damper"].get("reference") == "damper":
            corner["damper"]["motion_ratio"] = _mr0
        corner["static_setup"]["x_motion_ratio_note"] = (
            "x_motion_ratio_curve: deflection from the zero point (m) | ratio, from "
            "MOTION_RATIO (CSP DWB2)")
    _bt = susp_axle.get("bumpstop_table")
    if _bt:
        # CSP: with these tables PACKER_RANGE is the bump-stop gap (corner springs)
        _pr = susp_axle.get("packer_range")
        _gap = _pr if (_pr and 0 < _pr < 0.5) else (susp_axle.get("bumpstop") or {}).get("up")
        _curve = []
        for d, f in _bt["curve"]:
            pt = [round(d, 5), round(f * _bt["stack"], 2)]
            if _curve and pt[0] <= _curve[-1][0]:       # a curve must rise in x: AC tables
                continue                                # may repeat their last point
            _curve.append(pt)
        if _gap is not None and 0 < _gap < 0.5:
            # deflection past the gap -> force (the table starts at 0 | 0)
            corner["bump_stop"] = {"gap": round(_gap, 4), "type": "rubber",
                                   "rate_curve": _curve}
        else:
            corner["static_setup"]["x_bump_stop_curve"] = _curve   # no usable gap
    wm = susp_axle.get("unsprung_mass")
    if wm:
        corner["wheel"]["mass"] = round(wm / 2, 2)
    if susp_axle.get("tyre_offset"):
        corner["static_setup"]["track_offset"] = susp_axle["tyre_offset"]
    return corner


# ─── Pacejka fit per axle (per compound when multi) ──────────────────────────

def fit_tyre_axle_section(
        tyres_ini_dict: dict, section: str, axle: str, log: list,
        data_dir: Optional[Path] = None, plots: bool = True,
) -> tuple[Optional[dict], Optional[dict], Optional[BenchResult]]:
    """
    Returns (mf52_block, mf62_block, BenchResult).
    Both blocks are None when the section has no usable AC parameters.

    data_dir — car's data/ directory; required to resolve DY_CURVE / DX_CURVE
               LUT files for advanced CSP tyres.  Pass None to skip LUT reading.
    """
    p = parse_tyre_section(tyres_ini_dict, section=section, axle=axle,
                           data_dir=data_dir)
    if p.source == "defaults":
        log.append(f"  ⚠ {axle} [{section}]: no usable params — Pacejka blocks omitted")
        return None, None, None
    result = run_bench(p, plots)
    log.append(
        f"  ✓ {axle} [{section}]: MF5.2/MF6.2 fit from {p.source}  "
        f"lat R²={result.fit_lateral['r2']:.4f}  "
        f"long R²={result.fit_longitudinal['r2']:.4f}"
    )
    mf52, mf62 = build_svj_pacejka_blocks(result)
    return mf52, mf62, result


# ─── Main SVJ builder ────────────────────────────────────────────────────────

def build_svj(ini_files: dict, cm_meta: Optional[dict] = None,
              lut_files: Optional[dict] = None,
              data_dir: Optional[Path] = None,
              ctrl_files: Optional[dict] = None,
              glb_output_dir: Optional[Path] = None,
              include_skins: bool = True,
              kn5_override: Optional[Path] = None,
              meshes_subdir: str = "meshes",
              apply_graphics_pitch: bool = False,
              include_damage_textures: bool = False,
              highest_lod_only: bool = False,
              include_plots: bool = True,
              include_animations: bool = True,
              unified_suspension: bool = True,
              ) -> tuple[dict, list[str], dict]:
    """
    Returns (svj_dict, log_lines, bench_results_by_axle_compound).

    ``include_plots=False`` skips rendering the tire comparison PNGs (the slowest part
    when no GLB is made); the SVJ is identical, the bench results carry empty PNGs.

    ``unified_suspension`` (default) writes the suspension as bodies, joints and elements
    plus ``wheel_stations`` (SVJ 0.100, spec 25); False keeps the deprecated corner-based
    ``suspension`` section.

    Parameters
    ----------
    ini_files      : dict[str, str]   filename (lower) → text
    cm_meta        : dict             parsed ui_car.json
    lut_files      : dict[str, list]  optional pre-parsed LUTs (by filename)
    data_dir       : Path             optional; used by engine parser to resolve .lut
    ctrl_files     : dict[str, str]   controller ini texts (ctrl_ABS.ini, ctrl_TC.ini, …)
    glb_output_dir : Path             optional; when set, kn5_to_glb() writes
                                      <glb_output_dir>/meshes/<car>.glb alongside
                                      the JSON so callers can package or serve it.
    """
    log: list[str] = []
    ctrl_files = ctrl_files or {}

    # Surface how the data was sourced (unpacked data/, upload, or
    # "needs unpack" if only an encrypted archive was found)
    if cm_meta and cm_meta.get("_data_source"):
        log.append(f"✓ Source: {cm_meta['_data_source']}")

    # Parse every ini we have
    parsed: dict[str, dict] = {}
    for name, text in ini_files.items():
        try:
            parsed[name] = parse_ini(text)
        except Exception as ex:
            log.append(f"⚠ Failed to parse {name}: {ex}")

    c  = parsed.get("car.ini", {});            cB = c.get("BASIC", {})
    cW = c.get("WEIGHT", {});                  cF = c.get("FUEL", {})
    e  = parsed.get("engine.ini", {})
    d  = parsed.get("drivetrain.ini", {})
    s  = parsed.get("suspensions.ini", {});    sf = s.get("FRONT", {}); sr = s.get("REAR", {})
    sB = s.get("BASIC", {})          # ← Kunos keeps WHEELBASE/CG_LOCATION here
    t  = parsed.get("tyres.ini", {})
    b  = parsed.get("brakes.ini", {})
    a  = parsed.get("aero.ini", {})
    w  = parsed.get("wing.ini", {})

    # Content Manager meta
    cm       = cm_meta or {}
    cm_specs = cm.get("specs", {})

    import re
    # CM specs values look like "133bhp/6000rpm", "166Nm/4500rpm", "1095kg",
    # "0-100: 5,8s", "240km/h". The previous version stripped EVERYTHING
    # except digits and dots, which concatenated multi-number strings into
    # nonsense (e.g. "133bhp/6000rpm" → "1336000" → 1.33 million bhp,
    # which then made synth_torque_curve emit ~1.4 MN·m of peak torque).
    # Match the FIRST numeric token only; accept comma OR dot as decimal
    # separator (European locales use comma, e.g. "5,8s" → 5.8 s).
    _CM_NUM_RE = re.compile(r"-?\d+(?:[.,]\d+)?")
    def parse_cm_num(key: str) -> Optional[float]:
        raw = cm_specs.get(key)
        if raw is None:
            return None
        m = _CM_NUM_RE.search(str(raw))
        if not m:
            return None
        return float(m.group(0).replace(",", "."))

    car_name  = cm.get("name") or cB.get("SCREEN_NAME") or cB.get("NAME") or "Unknown"
    brand     = cm.get("brand") or cB.get("BRAND") or ""
    # CM often stores the full "Brand Model Year" string in name; strip the leading
    # brand prefix so vehicle_info.model doesn't duplicate the make field.
    _model = car_name
    if brand and _model.lower().startswith(brand.lower()):
        _model = _model[len(brand):].strip()
    year      = cm.get("year")
    car_class = cm.get("class")
    tags      = cm.get("tags", [])
    desc      = cm.get("description", "")
    cm_bhp    = parse_cm_num("bhp")

    log.append(f"✓ Car: {(brand + ' ') if brand else ''}{_model}")
    if cm:
        log.append("✓ Content Manager metadata loaded")

    cm_weight_str = cm_specs.get("weight", "")
    cm_weight = parse_cm_num("weight")      # None for "-", "" or text without a number
    total_mass = _f(cW.get("MINIMUM"), cm_weight or 1200.0)

    # ── Suspension parsed blocks ─────────────────────────────────────────────
    sa_f = parse_suspension_axle(sf)
    sa_r = parse_suspension_axle(sr)
    _attach_csp_tables(sa_f, sa_r, parsed, data_dir, log)
    _attach_arb(sa_f, sa_r, parsed, log)
    _attach_cosmic(sa_f, sa_r, parsed, log)

    wm_f = sa_f.get("unsprung_mass")
    wm_r = sa_r.get("unsprung_mass")
    log.append(f"✓ Mass: {total_mass} kg"
               + (f", unsprung F:{wm_f} R:{wm_r} kg/axle" if wm_f else " (no unsprung data)"))

    rear_bias = _f(d.get("TRACTION", {}).get("REAR_BIAS")
                   or d.get("TRACTION", {}).get("REARBIASRATIO"), 0.50)
    # WHEELBASE can live in suspensions.ini [BASIC] (Kunos) or car.ini [BASIC]
    # (some mods). Try suspensions first, then car.ini, then fall back to 2.6.
    wheelbase_raw = sB.get("WHEELBASE") or cB.get("WHEELBASE")
    wheelbase = _f(wheelbase_raw, 2.6)
    # CG_LOCATION in suspensions.ini [BASIC] is the front weight distribution
    # (fraction). If set, it overrides drivetrain.ini REAR_BIAS.
    cg_loc = _f(sB.get("CG_LOCATION"), -1.0)
    if cg_loc > 0.0:
        rear_bias = 1.0 - cg_loc
    # The CG sits (1 - front_weight) = rear_weight x wheelbase behind the front axle:
    # the more weight on the front, the closer the CG is to the front axle. (This
    # was mirrored - front_weight x wheelbase - which put the CG of a front-heavy
    # car near the rear axle. Verified against the KN5 + GRAPHICS_OFFSET geometry of
    # installed cars: front axle -> CG = (1 - CG_LOCATION) x wheelbase.)
    cg_x = -round(wheelbase * rear_bias, 3)
    cg_z = -round(_f(cB.get("CG_HEIGHT") or cB.get("CGHEIGHT"), 0.42), 3)
    cg_derived = False   # True once the height comes from the car's own data (below)
    _cg_explicit = bool(cB.get("CG_HEIGHT") or cB.get("CGHEIGHT"))
    # car.ini [BASIC] GRAPHICS_OFFSET / GRAPHICS_PITCH_ROTATION: how AC maps the
    # KN5 model into its physics frame (origin at the CG).
    _gfx_off   = _vec3(cB.get("GRAPHICS_OFFSET"))
    _gfx_pitch = _f(cB.get("GRAPHICS_PITCH_ROTATION"), 0.0)
    log.append(f"✓ CG: [{cg_x}, 0.0, {cg_z}] m  (wheelbase {wheelbase} m, rear bias {rear_bias:.1%})")

    # ── Engine (NEW: full parse) ─────────────────────────────────────────────
    eng = parse_engine(e, data_dir)
    if eng["source_flags"]["power_lut"]:
        log.append(f"✓ Engine: power.lut loaded ({len(eng['torque_curve'])} pts)")
    else:
        eng["torque_curve"] = synth_torque_curve(eng["idle_rpm"], eng["max_rpm"], cm_bhp)
        log.append(f"⚠ Engine: no power.lut, torque curve synthesised"
                   f" (from {'CM bhp' if cm_bhp else 'flat defaults'})")
    if eng["source_flags"]["coast_lut"]:
        log.append(f"✓ Engine: coast_curve.lut loaded ({len(eng['coast_curve'])} pts)")
    elif eng["coast_curve"]:
        log.append(f"✓ Engine: coast curve modelled from COAST_REF")
    if eng["turbos"]:
        log.append(f"✓ Turbos: {len(eng['turbos'])} stage(s)")
    if eng["damage"]:
        log.append(f"✓ Engine damage block captured ({len(eng['damage'])} fields)")
    log.append(f"✓ Engine: {int(eng['idle_rpm'])}–{int(eng['max_rpm'])} rpm,"
               f" inertia {eng['inertia']} kg·m²")

    # ── Drivetrain (NEW: full parse) ─────────────────────────────────────────
    dt = parse_drivetrain(d)
    layout = drive_layout(dt["layout_raw"])
    layout, _layout_src = infer_engine_layout(layout, 1.0 - rear_bias, f"{car_name} {cB.get('SCREEN_NAME', '')}")
    if _layout_src:
        log.append(f"✓ Engine layout inferred as {layout} from {_layout_src}"
                   f" (front weight {1.0 - rear_bias:.1%}; AC has no engine position)")
    diff_type_raw = dt["differential"]["type"]
    diff_type = DIFF_TYPE_MAP.get(diff_type_raw, "lsd_clutch")
    diff_loc  = "rear" if layout in ("FR", "MR", "RR") else "front"
    log.append(f"✓ Gearbox: {dt['gearbox']['count']}sp,"
               f" final {dt['gearbox'].get('final_drive', 3.5):.3f},"
               f" layout {layout}, diff {diff_type}")
    if dt.get("autoblip"):
        log.append("✓ Driver aids: autoblip")
    if dt.get("autoclutch"):
        log.append("✓ Driver aids: autoclutch")

    # ── Brakes (NEW: full parse) ─────────────────────────────────────────────
    br = parse_brakes(b)
    disc_in_f    = _m(br["front"].get("DISC_INERTIA"))
    disc_in_r    = _m(br["rear"].get("DISC_INERTIA"))
    disc_rad_f   = _m(br["front"].get("DISC_RADIUS") or br["front"].get("DISC_R"))
    disc_rad_r   = _m(br["rear"].get("DISC_RADIUS") or br["rear"].get("DISC_R"))
    brake_torq_f = _f(br["front"].get("MAX_TORQUE") or br["front"].get("TORQUE"), 3500.0)
    brake_torq_r = _f(br["rear"].get("MAX_TORQUE") or br["rear"].get("TORQUE"), 2500.0)
    brake_bias   = br.get("bias", 0.6)
    if disc_in_f:
        log.append(f"✓ Brake discs: F inertia={disc_in_f} r={disc_rad_f}m,"
                   f" R inertia={disc_in_r} r={disc_rad_r}m")
    if br.get("handbrake"):
        log.append("✓ Handbrake parsed")

    # ── Tires: multi-compound + Pacejka fit each ─────────────────────────────
    compounds = detect_tyre_compounds(t) if t else []
    tire_sets: dict[str, dict] = {}
    bench_results: dict[str, BenchResult] = {}
    pacejka_front = None
    pacejka_rear  = None

    def tire_set_name(section: str, suffix: str, tw, asp, rim_r):
        rim_in = round(rim_r * 2 / 0.0254) - 1  # Kunos: RIM_RADIUS=(nominal_in+1)*0.0254/2
        label = suffix if suffix else ""
        return f"tire_{section.lower()}{label}_{round(tw*1000)}_{round(asp*100)}r{rim_in}"

    def tire_set_obj(tw, asp, rim_rv, label, compound_suffix):
        rim_in  = round(rim_rv * 2 / 0.0254) - 1  # Kunos +1 inch convention
        rim_d   = round(rim_in * 0.0254, 4)           # nominal diameter (m)
        overall = round(rim_d + 2 * tw * asp, 4)
        size_code = f"{round(tw*1000)}/{round(asp*100)}R{rim_in}"
        dims = {
            "section_width":     round(tw, 4),
            "aspect_ratio":      round(asp, 3),
            "rim_diameter_code": rim_in,
            "overall_diameter":  overall,
            "section_height":    round(tw * asp, 4),
        }
        out = {
            "description":    f"AC-derived tire {size_code} ({label}{compound_suffix})",
            "size_code":      size_code,
            "source":         "estimated",
            "dimensions":     dims,
            "rim": {"diameter": round(rim_in * 0.0254, 4),
                    "width_nominal": round(tw, 4)},
            "pressure": 210000.0,
            "_est": True,
            "_note": "Tire dimensions from tyres.ini. See pacejka (MF62) and pacejka_mf52 for force models.",
        }
        # The schema takes a size code like 205/55R16 and an aspect ratio up to 1; a tyre
        # whose sidewall is taller than it is wide (the data of some mods) gets neither,
        # the heights stay in `dimensions`.
        if not re.fullmatch(r"\d{3}/\d{2}R\d{2}", size_code):
            del out["size_code"]
        if asp > 1.0:
            del dims["aspect_ratio"]
        if not 10 <= rim_in <= 30:                  # not a rim size: bad or odd data
            del dims["rim_diameter_code"]
        return out

    log.append("─── Tire Lab (tyres.ini → Pacejka MF 5.2) ──────────────")
    ts_front_default = "tire_front_default"
    ts_rear_default  = "tire_rear_default"
    rolling_radius_f = 0.306
    rolling_radius_r = 0.306

    if not compounds:
        log.append("  ⚠ No tyre sections found")
    for pair_idx, (f_sec, r_sec) in enumerate(compounds):
        suffix = f_sec.replace("FRONT", "").lower()  # "" or "_0", "_1"…
        tf = t.get(f_sec, {})
        tr = t.get(r_sec, {})

        tw_f  = _f(tf.get("WIDTH") or tf.get("TYRE_WIDTH"),  0.205)
        rad_f = _f(tf.get("RADIUS") or tf.get("TYRE_RADIUS"), 0.306)
        rim_f = _f(tf.get("RIM_RADIUS"), 0.191)
        tw_r  = _f(tr.get("WIDTH") or tr.get("TYRE_WIDTH"),  0.205)
        rad_r = _f(tr.get("RADIUS") or tr.get("TYRE_RADIUS"), 0.306)
        rim_r = _f(tr.get("RIM_RADIUS"), 0.191)
        asp_f = round((rad_f - (rim_f - 0.0127)) / tw_f, 3) if tw_f > 0 else 0.45
        asp_r = round((rad_r - (rim_r - 0.0127)) / tw_r, 3) if tw_r > 0 else 0.45

        name_f = tire_set_name("front", suffix, tw_f, asp_f, rim_f)
        name_r = tire_set_name("rear",  suffix, tw_r, asp_r, rim_r)

        # Pacejka fit — returns (mf52, mf62, BenchResult)
        pf52, pf62, br_f = fit_tyre_axle_section(t, f_sec, f"front{suffix}", log, data_dir,
                                                     include_plots)
        pr52, pr62, br_r = fit_tyre_axle_section(t, r_sec, f"rear{suffix}",  log, data_dir,
                                                     include_plots)
        if pair_idx == 0:
            pacejka_front    = pf62  # MF62 is now the primary reference
            pacejka_rear     = pr62
            ts_front_default = name_f
            ts_rear_default  = name_r
            rolling_radius_f = rad_f
            rolling_radius_r = rad_r
        if br_f: bench_results[f"front{suffix}"] = br_f
        if br_r: bench_results[f"rear{suffix}"]  = br_r

        front_obj = tire_set_obj(tw_f, asp_f, rim_f, "front", suffix)
        rear_obj  = tire_set_obj(tw_r, asp_r, rim_r, "rear",  suffix)

        # Thermal + extras
        thermal = parse_tyre_thermal(t, suffix)
        if thermal:
            front_obj["thermal"] = thermal.get("front")
            rear_obj["thermal"]  = thermal.get("rear")

        xtra_f = parse_tyre_extras(tf)
        xtra_r = parse_tyre_extras(tr)
        if xtra_f: front_obj.update(xtra_f)
        if xtra_r: rear_obj.update(xtra_r)

        # Embed both Pacejka models.
        # SVJ schema-standard key: `pacejka` → MF62 (richer model, includes
        # combined-slip weights and AC camber mapping via pVy3).
        # Additional key: `pacejka_mf52` → MF52 pure-slip subset for
        # simulators that only support the older format.
        if pf62:
            front_obj["pacejka"]      = pf62
            front_obj["pacejka_mf52"] = pf52
        if pr62:
            rear_obj["pacejka"]      = pr62
            rear_obj["pacejka_mf52"] = pr52

        tire_sets[name_f] = front_obj
        if name_r != name_f:
            tire_sets[name_r] = rear_obj

    if not tire_sets:
        tire_sets = {
            ts_front_default: {
                "description": "placeholder — no tyres.ini sections found",
                "_est": True,
            },
        }
    log.append(f"  ({len(compounds)} compound pair(s), {len(tire_sets)} tire sets emitted)")

    # ── Aero + wings + DRS (SVJ 0.95: components[] + global_coefficients) ───
    ae = parse_aero(a, w, data_dir)
    if ae["wings"]:
        log.append(f"✓ Aero: {len(ae['wings'])} wing element(s) parsed")
    if ae.get("drs"):
        log.append("✓ DRS parsed")

    def _lut_to_aero_map(lut, coeff_name: str) -> Optional[dict]:
        """AC LUT (list of [aoa_deg, coeff]) → SVJ 0.95 aero_map (1D); None if empty.

        The schema wants two points at least: a one-point LUT (a constant in AC) is
        written as the same value at x and x + 1, which is the same curve."""
        vals = [{"x": float(p[0]), "value": float(p[1])} for p in (lut or [])
                if isinstance(p, (list, tuple)) and len(p) >= 2]
        if not vals:
            return None
        note = "x values are degrees (AC convention)."
        if len(vals) == 1:
            vals.append({"x": vals[0]["x"] + 1.0, "value": vals[0]["value"]})
            note += " One-point curve in AC (constant): the point is repeated at x + 1."
        return {
            "type": "1d",
            "x_axis": "pitch_angle",
            "values": vals,
            "interpolation": "linear",
            "extrapolation": "clamp",
            "_note": note,
        }

    # Build per-component list. Body component absorbs global BODY coefficients
    # *minus* the wing contributions so Σ components = global (§13.5).
    aero_components: list[dict] = []
    wing_cl_sum = 0.0
    wing_cd_sum = 0.0
    drs_wing_ref = None
    for i, wing in enumerate(ae.get("wings", []) or []):
        wc = {
            "name": (wing.get("name") or f"wing_{i}").strip() or f"wing_{i}",
            "type": "wing",
        }
        pos_ac = wing.get("position_ac")
        if pos_ac and len(pos_ac) == 3:
            wc["position"] = list(ac_vec_to_svj(pos_ac))
        chord = wing.get("chord")
        span  = wing.get("span")
        if chord and span:
            try:
                wc["reference"] = {"frontal_area": round(float(chord) * float(span), 4)}
            except (TypeError, ValueError):
                pass
        angle_raw = wing.get("angle")
        if angle_raw is not None:
            # AC aero.ini ANGLE is in degrees (distinct from suspensions.ini
            # angle conventions). Emit raw — no rad→deg conversion.
            try:
                wc["adjustment_range"] = {"neutral_deg": round(float(angle_raw), 2)}
            except (TypeError, ValueError):
                pass
        cl_gain = wing.get("cl_gain")
        cd_gain = wing.get("cd_gain")
        coeffs: dict = {}
        if cl_gain is not None:
            try:
                coeffs["Cl"] = round(float(cl_gain), 4)
                wing_cl_sum += float(cl_gain)
            except (TypeError, ValueError):
                pass
        if cd_gain is not None:
            try:
                coeffs["Cd"] = round(float(cd_gain), 4)
                wing_cd_sum += float(cd_gain)
            except (TypeError, ValueError):
                pass
        if coeffs:
            wc["coefficients"] = coeffs
        _amap = _lut_to_aero_map(wing.get("aoa_cl_curve"), "Cl")
        if _amap:
            wc.setdefault("maps", {})["Cl_vs_angle"] = _amap
        _amap = _lut_to_aero_map(wing.get("aoa_cd_curve"), "Cd")
        if _amap:
            wc.setdefault("maps", {})["Cd_vs_angle"] = _amap
        aero_components.append(wc)
        if drs_wing_ref is None and ae.get("drs") is not None:
            drs_idx = ae["drs"].get("wing_index")
            if drs_idx is not None:
                try:
                    if int(drs_idx) == i:
                        drs_wing_ref = wc["name"]
                except (TypeError, ValueError):
                    pass

    # Synthetic body component. Cd/Cl assigned so Σ components = global.
    try:
        body_cd = round(float(ae["cd"]) - wing_cd_sum, 4)
    except (TypeError, ValueError):
        body_cd = ae.get("cd")
    try:
        body_cl = round(float(ae["cl_front"]) + float(ae["cl_rear"]) - wing_cl_sum, 4)
    except (TypeError, ValueError):
        body_cl = None
    body_comp: dict = {
        "name": "body",
        "type": "fixed",
        "coefficients": {},
    }
    if body_cd is not None:
        body_comp["coefficients"]["Cd"] = body_cd
    if body_cl is not None:
        body_comp["coefficients"]["Cl"] = body_cl
    # Body first so Σ order matches spec example.
    aero_components.insert(0, body_comp)

    # active_systems.drs — AC's ACTIVE_SPEED_THRESHOLD is km/h, SVJ wants m/s.
    active_systems: dict = {}
    if ae.get("drs") is not None:
        drs_src = ae["drs"]
        drs_out: dict = {
            "enabled": bool(drs_src.get("enabled")) if drs_src.get("enabled") is not None else False,
        }
        if drs_wing_ref:
            drs_out["component_ref"] = drs_wing_ref
        thr_kmh = drs_src.get("zone_speed_threshold")
        if thr_kmh is not None:
            try:
                drs_out["activation_conditions"] = {
                    "speed_min": round(float(thr_kmh) * (1000.0 / 3600.0), 3),
                }
            except (TypeError, ValueError):
                pass
        active_systems["drs"] = drs_out

    # ── car.ini extras ───────────────────────────────────────────────────────
    car_extras = parse_car_extras(c)

    fuel_default = _f(cF.get("FUEL") or cB.get("FUEL"), 50.0)
    fuel_max     = _f(cF.get("MAX_FUEL") or cF.get("MAX"), 60.0)

    log.append(f"✓ Suspension: F={susp_type(sf.get('TYPE','0'))}  R={susp_type(sr.get('TYPE','0'))}")
    if sa_f.get("hardpoints_ac") or sa_r.get("hardpoints_ac"):
        log.append(f"✓ Suspension hardpoints: "
                   f"F={len(sa_f.get('hardpoints_ac') or {})} pts, "
                   f"R={len(sa_r.get('hardpoints_ac') or {})} pts")
    else:
        log.append("⚠ Suspension hardpoints: none found (WBCAR_* / WBTYRE_* keys absent)")

    unsprung: dict = {}
    if wm_f: unsprung["FL"] = unsprung["FR"] = round(wm_f / 2, 2)
    if wm_r: unsprung["RL"] = unsprung["RR"] = round(wm_r / 2, 2)

    track_f = _f(sf.get("TRACK") or sf.get("TRACK_WIDTH") or sf.get("FRONT_TRACK"), 1.5)
    track_r = _f(sr.get("TRACK") or sr.get("TRACK_WIDTH") or sr.get("REAR_TRACK"),  1.5)
    if not track_f > 0:               # TRACK=0: a car with one front wheel; the schema needs > 0
        track_f = 0.01
        log.append("⚠ Front track is 0 (one front wheel): written as 0.01 m, the schema needs a positive value")

    # ── Electronics (controllers) ────────────────────────────────────────────
    electronics: dict = {}
    for key, text in ctrl_files.items():
        try:
            electronics[key] = parse_controller_ini(text)
            log.append(f"✓ Controller parsed: {key} ({len(electronics[key].get('controllers', []))} curves)")
        except Exception as ex:
            log.append(f"⚠ Controller parse failed for {key}: {ex}")

    # setup ranges
    setup_ranges = None
    if ini_files.get("setup.ini"):
        try:
            setup_ranges = parse_setup_ranges(ini_files["setup.ini"])
            log.append(f"✓ setup.ini: {len(setup_ranges)} adjustment range block(s)")
        except Exception as ex:
            log.append(f"⚠ setup.ini parse failed: {ex}")

    # ── Assemble SVJ ─────────────────────────────────────────────────────────
    gearbox_out = {
        "type":           "manual",
        "ratios":         dt["gearbox"]["ratios"] or [3.0, 2.1, 1.5, 1.1, 0.9, 0.76],
        "final_drive":    dt["gearbox"].get("final_drive", 3.5),
    }
    for k_from, k_to in [("change_up_s", "shift_up_time"),
                         ("change_down_s", "shift_down_time"),
                         ("auto_cutoff_s", "auto_cutoff_time"),
                         ("supports_shifter", "h_pattern_supported"),
                         ("inertia", "inertia")]:
        if dt["gearbox"].get(k_from) is not None:
            gearbox_out[k_to] = dt["gearbox"][k_from]

    if dt["gearbox"].get("reverse_ratio"):
        gearbox_out["reverse_ratios"] = [dt["gearbox"]["reverse_ratio"]]
    _diff_ctl = _load_diff_controllers(ini_files, data_dir, log)
    _diffs, _tc = _build_driveline(dt, layout, diff_type, diff_loc, _diff_ctl)
    _clutch = ({"max_torque": dt["clutch"]["max_torque"]}
               if dt.get("clutch", {}).get("max_torque") else None)
    if layout in ("AWD", "4WD") and _tc is None:
        log.append("⚠ AWD car without an [AWD]/[AWD2] section: single differential "
                   "emitted from [DIFFERENTIAL]")
    elif _tc is not None:
        _fs = _tc.get("torque_split")
        log.append(f"✓ Driveline: {_tc['type']} transfer case"
                   + (f", {_fs[0]:.0%} front" if _fs else "")
                   + f", {len(_diffs)} axle differential(s)")

    # data_origin (SVJ 0.95 §1) — confidence reflects how the AC data
    # reached us: unpacked data/ is verbatim from the car; uploads are
    # less verifiable so we drop a notch.
    _source_note = (cm_meta.get("_data_source") if cm_meta else None) or "unknown"
    if "unpacked data/" in _source_note:
        _origin_confidence = "medium"
    else:
        _origin_confidence = "low"

    # ── CG height ────────────────────────────────────────────────────────────
    # car.ini has no CG height, so this used to be the constant 0.42 m for every
    # car.  AC defines it through BASEY (suspensions.ini, per axle): the physics
    # origin is the CG and the wheel centre sits BASEY from it, so the CG is
    # (tyre radius - BASEY) above the ground at that axle.  The two axles can
    # differ (the car sits with some rake), so interpolate at the CG position,
    # i.e. weight each axle by the OTHER axle's share: front share = cg_loc.
    if _cg_explicit:
        cg_derived = True            # the car states it: the most trustworthy source
        log.append(f"✓ CG height {-cg_z} m from car.ini")
    elif sa_f.get("basey") is not None and sa_r.get("basey") is not None:
        _fw = 1.0 - rear_bias
        _h_f = rolling_radius_f - sa_f["basey"]
        _h_r = rolling_radius_r - sa_r["basey"]
        cg_z = -round(_fw * _h_f + (1.0 - _fw) * _h_r, 3)
        cg_derived = True
        log.append(f"✓ CG height {-cg_z} m from tyre radius - BASEY "
                   f"(F {_h_f:.3f} / R {_h_r:.3f} m at the axles)")
    else:
        log.append(f"⚠ CG height {-cg_z} m is a default: no CG_HEIGHT in car.ini and "
                   f"no BASEY in suspensions.ini")

    svj: dict = {
        "_metadata": {
            "specification":        SVJ_SPEC,
            "version":              SVJ_VERSION,
            "coordinate_system":    "SAE_J670",
            "units":                "SI",
            "alignment_convention": "relative_to_centerline",
            "source_format":        "AC",
            "author":               f"ac_to_svj_converter v{CONV_VER}",
            "created":              datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
            "description":          f"Converted from Assetto Corsa{(' — ' + desc) if desc else ''}",
            "x_svj_repo":           REPO_URL,
                "x_converter_repo":     CONV_REPO,
            "data_origin": {
                "type":       "simulation",
                "detail":     f"Assetto Corsa — {_source_note}",
                "confidence": _origin_confidence,
            },
        },
        "vehicle_info": {
            "make": brand or None, "model": _model, "year": year or None,
            "variant": car_class or None, "drive_type": drive_type(layout),
        },
        "chassis": {
            "mass_total":               total_mass,
            "mass_unsprung_per_corner": unsprung or None,
            "wheelbase":                wheelbase,
            "track_front":  track_f,
            "track_rear":   track_r,
            "center_of_gravity": [cg_x, 0.0, cg_z],
            "fuel_capacity": fuel_max,
            "fuel_initial":  fuel_default,
        },
        "suspension": {
            "FL": build_corner("FL", sa_f, wheelbase, track_f, track_r,
                               disc_in_f, disc_rad_f, brake_torq_f, brake_bias, ts_front_default,
                               rolling_radius=rolling_radius_f, cg_height=-cg_z, tire_sets=tire_sets),
            "FR": build_corner("FR", sa_f, wheelbase, track_f, track_r,
                               disc_in_f, disc_rad_f, brake_torq_f, brake_bias, ts_front_default,
                               rolling_radius=rolling_radius_f, cg_height=-cg_z, tire_sets=tire_sets),
            "RL": build_corner("RL", sa_r, wheelbase, track_f, track_r,
                               disc_in_r, disc_rad_r, brake_torq_r, brake_bias, ts_rear_default,
                               rolling_radius=rolling_radius_r, cg_height=-cg_z, tire_sets=tire_sets),
            "RR": build_corner("RR", sa_r, wheelbase, track_f, track_r,
                               disc_in_r, disc_rad_r, brake_torq_r, brake_bias, ts_rear_default,
                               rolling_radius=rolling_radius_r, cg_height=-cg_z, tire_sets=tire_sets),
        },
        "tires": {"sets": tire_sets},
        "brakes": {
            "bias":       brake_bias,
            "bias_type":  "adjustable" if br.get("cockpit_adj") else "fixed",
            "bias_step":  br.get("bias_step"),
            "bias_min":   br.get("bias_min"),
            "bias_max":   br.get("bias_max"),
            "handbrake":  br.get("handbrake"),
        },
        "powertrain": {
            "layout": layout,
            "engine": {
                "idle_rpm":             eng["idle_rpm"],
                "max_rpm":              eng["max_rpm"],
                "inertia":              eng["inertia"],
                "limiter_hz":           eng.get("limiter_hz"),
                "altitude_sensitivity": eng.get("altitude_sensitivity"),
                "throttle_response":    eng.get("throttle_response"),
                "fuel_consumption_k":   eng.get("fuel_consumption_k"),
                "torque_curve":         eng["torque_curve"],
                "coast_curve":          eng.get("coast_curve"),
                "turbos":               eng["turbos"] or None,
                "bov":                  eng.get("bov"),
                "_source": {
                    "power_lut":  eng["source_flags"]["power_lut"],
                    "coast_lut":  eng["source_flags"]["coast_lut"],
                    "version":    eng.get("version"),
                },
            },
            "gearbox": gearbox_out,
            "differentials": _diffs,
        },
        "aerodynamics": {
            "reference": {
                "frontal_area":     ae["frontal_area"],
                "air_density":      1.225,
                "reference_length": wheelbase,
            },
            # SVJ 0.95 canonical key:
            "global_coefficients": {
                "Cd":        round(float(ae["cd"]), 4),
                "Cl":        round(float(ae["cl_front"]) + float(ae["cl_rear"]), 4),
                "Cl_front":  round(float(ae["cl_front"]), 4),
                "Cl_rear":   round(float(ae["cl_rear"]), 4),
            },
            # Legacy 0.94 key — kept so downstream tooling that hasn't moved
            # to 0.95 still finds Cd/Cl where it used to live.
            "coefficients": {
                "Cd":        round(float(ae["cd"]), 4),
                "Cl":        round(float(ae["cl_front"]) + float(ae["cl_rear"]), 4),
                "Cl_front":  round(float(ae["cl_front"]), 4),
                "Cl_rear":   round(float(ae["cl_rear"]), 4),
                "_note":     "Legacy 0.94 key — see global_coefficients for 0.95.",
            },
            "components": aero_components,
            "active_systems": (active_systems or None),
        },
    }

    if _clutch:
        svj["powertrain"]["clutch"] = _clutch
    if _tc:
        svj["powertrain"]["transfer_case"] = _tc

    # ── x_assettocorsa extension block ───────────────────────────────────────
    cm_ext: dict = {}
    for k, getter in [
        ("power_bhp",     lambda: parse_cm_num("bhp")),
        ("torque_nm",     lambda: parse_cm_num("torque")),
        ("top_speed_kmh", lambda: parse_cm_num("topspeed")),
        ("accel_0_100_s", lambda: parse_cm_num("acceleration")),
        ("power_to_weight", lambda: cm_specs.get("pwratio")),
        ("tags",          lambda: tags or None),
    ]:
        v = getter()
        if v is not None:
            cm_ext[k] = v

    damage_all: dict = {}
    if eng.get("damage"):
        damage_all["engine"] = eng["damage"]
    if dt.get("damage"):
        damage_all["drivetrain"] = dt["damage"]

    driver_aids = {}
    if dt.get("autoclutch"):     driver_aids["autoclutch"]     = dt["autoclutch"]
    if dt.get("autoblip"):       driver_aids["autoblip"]       = dt["autoblip"]
    if dt.get("shift_profiler"): driver_aids["shift_profiler"] = dt["shift_profiler"]

    # x_assettocorsa.aero — preserve AC-specific aero knobs that don't have a
    # home in the SVJ 0.95 aero schema (wind multiplier, damage coefficient,
    # and CSP active-aero wing.ini blob).
    x_ac_aero: dict = {}
    if ae.get("wind_mult") is not None:
        x_ac_aero["wind_mult"] = ae["wind_mult"]
    if ae.get("damage_k") is not None:
        x_ac_aero["damage_k"] = ae["damage_k"]
    if ae.get("active_aero"):
        x_ac_aero["active_aero_raw"] = ae["active_aero"]

    # Drivetrain details with no SVJ home: raw AWD values, shift automation, the
    # extra [DIFFERENTIAL] keys and the AWD centre-diff controller (with its LUTs).
    x_ac_dt: dict = {}
    if dt.get("awd"):                 x_ac_dt["awd"] = dt["awd"]
    if dt.get("auto_shifter"):        x_ac_dt["auto_shifter"] = dt["auto_shifter"]
    if dt.get("downshift_protection"): x_ac_dt["downshift_protection"] = dt["downshift_protection"]
    if dt.get("differential_extra"):  x_ac_dt["differential_extra"] = dt["differential_extra"]
    if _diff_ctl:
        x_ac_dt["controllers"] = _diff_ctl

    x_ac: dict = {}
    if x_ac_dt:          x_ac["drivetrain"]      = x_ac_dt
    if cm_ext:           x_ac["cm_specs"]        = cm_ext
    if damage_all:       x_ac["damage"]          = damage_all
    if driver_aids:      x_ac["driver_aids"]     = driver_aids
    if electronics:      x_ac["electronics"]     = electronics
    if setup_ranges:     x_ac["setup_ranges"]    = setup_ranges
    if x_ac_aero:        x_ac["aero"]            = x_ac_aero
    if eng.get("dynamic_turbo"):
        x_ac["dynamic_turbo"] = eng["dynamic_turbo"]
    if car_extras.get("info"):    x_ac["info"]     = car_extras["info"]
    if car_extras.get("controls"):x_ac["controls"] = car_extras["controls"]
    if car_extras.get("rules"):   x_ac["rules"]    = car_extras["rules"]
    if car_extras.get("pit_stop"):x_ac["pit_stop"] = car_extras["pit_stop"]
    if car_extras.get("fuel_tank_position_ac"):
        x_ac["fuel_tank_position"] = ac_vec_to_svj(car_extras["fuel_tank_position_ac"])
    ac_versions = {
        "engine_version":     eng.get("version"),
        "drivetrain_version": dt.get("version"),
    }
    ac_versions = {k: v for k, v in ac_versions.items() if v is not None}
    if ac_versions:
        x_ac["versions"] = ac_versions

    _csp = _csp_physics_flags(parsed)
    if _csp:
        x_ac["csp_physics"] = _csp

    if _gfx_off is not None:
        x_ac["graphics"] = {"offset": _gfx_off, "pitch_rotation_deg": _gfx_pitch}
    if x_ac:
        svj["x_assettocorsa"] = x_ac

    # ── KN5 visual bindings (SVJ 0.97 assets.meshes + visual) ────────────────
    # When a KN5 file is present alongside the car data, scan its node tree
    # and emit the assets.meshes manifest plus visual bindings on chassis and
    # each suspension corner.  The GLB lives at  meshes/<car_name>.glb  (the
    # conventional output path when kn5_to_glb() is used from the CLI or
    # Tire Lab tab).  LOD B/C/D GLBs (when present) are also exported.
    # All fields are optional in the schema — if no KN5 is found the block is
    # omitted and the SVJ is still fully valid.
    if _KN5_AVAILABLE and data_dir is not None:
        car_path = data_dir.parent
        try:
            kn5_lods = find_car_kn5_lods(car_path, kn5_override)  # [("A", path), ...]
            if highest_lod_only:
                kn5_lods = kn5_lods[:1]              # LOD A only
            _choice = resolve_car_kn5(car_path, kn5_override)
        except ValueError as _e:
            kn5_lods, _choice = [], None
            log.append(f"⚠ KN5 override rejected: {_e}")
        if _choice is not None and _choice.skipped_encrypted:
            log.append("ℹ KN5: " + _choice.message.replace("\n", "\n    "))
        if not kn5_lods:
            log.append(f"ℹ No usable KN5 in {car_path} — visual bindings and GLB skipped"
                       + (f"\n    {_choice.message}" if _choice is not None else ""))
        else:
            kn5_path = kn5_lods[0][1]            # LOD A is always first
            try:
                node_names = scan_kn5_nodes(kn5_path)
                # GLBs live in <meshes_subdir>/ next to the JSON, or directly
                # beside it when meshes_subdir is "" (URIs are relative to the JSON).
                _mp        = f"{meshes_subdir}/" if meshes_subdir else ""
                glb_uri    = f"{_mp}{kn5_path.stem}.glb"
                mesh_id    = _mesh_slug(kn5_path.stem)

                # Build assets.meshes list — LOD A first, then B/C/D entries
                mesh_entries = [
                    {
                        "id":          mesh_id,
                        "uri":         glb_uri,
                        "description": f"Converted from {kn5_path.name} (LOD A)",
                    }
                ]
                for lod_label, lod_path in kn5_lods[1:]:
                    lod_id  = _mesh_slug(lod_path.stem)
                    lod_uri = f"{_mp}{lod_path.stem}.glb"
                    mesh_entries.append({
                        "id":          lod_id,
                        "uri":         lod_uri,
                        "description": f"Converted from {lod_path.name} (LOD {lod_label})",
                    })

                svj["assets"] = {"meshes": mesh_entries}

                # Visual bindings (SVJ v0.99.2 categories).  Every binding points
                # at LOD A and names the glTF node the exporter will create
                # (SVJ::<category>::<id>); a part is bound only when the KN5 has
                # a node for it AND the SVJ output has the part to carry it.
                node_rename: dict = {}
                for part in map_ac_nodes_to_svj_parts(node_names):
                    vis = {"mesh_ref": mesh_id, "node": part["node"]}
                    st = part["station"]
                    corner = svj.get("suspension", {}).get(st) if st else None
                    carrier = None
                    if part["part"] == "chassis":
                        carrier = svj["chassis"]
                    elif corner is not None:
                        if part["part"] == "upright":
                            carrier = corner
                        elif part["part"] == "wheel":
                            carrier = corner.setdefault("wheel", {})
                        elif part["part"] == "disc":
                            disc = (corner.get("brake") or {}).get("disc")
                            carrier = disc if isinstance(disc, dict) else None
                    if carrier is None:
                        continue
                    carrier["visual"] = vis
                    node_rename[part["ac_name"]] = part["node"]

                lod_labels = [lbl for lbl, _ in kn5_lods]
                log.append(
                    f"✓ KN5 visual bindings: {len(node_rename)} parts bound "
                    f"from {kn5_path.name} (LODs: {', '.join(lod_labels)}) → {glb_uri}"
                )

                # Exact model placement from GRAPHICS_OFFSET; None = fall back to
                # putting the tyres on the ground.
                # GRAPHICS_PITCH_ROTATION (positive = nose down, degrees, about the CG;
                # sign and unit fitted on ~700 installed cars) is OFF by default: it
                # compensates the RAKE of AC's physics frame (body tilted against the
                # ground), whereas the SVJ frame is level, and fleet data shows the
                # un-rotated model matches the SVJ wheel positions best (see CHANGELOG).
                # Opt in with apply_graphics_pitch=True; values beyond +-6 deg are ignored.
                _pitch = (_gfx_pitch if apply_graphics_pitch and abs(_gfx_pitch) <= 6.0
                          else 0.0)
                _placement = _visual_placement(svj, _gfx_off, cg_derived, _pitch)
                if _placement and _placement["translation"] is not None:
                    log.append(f"✓ Mesh placement: GRAPHICS_OFFSET {_gfx_off} + derived CG "
                               f"(physics frame)")
                elif _placement:
                    log.append("ℹ Mesh placement: wheel centres matched to the physics "
                               "(GRAPHICS_OFFSET or a known CG height is missing)")
                else:
                    log.append("ℹ Mesh placement: tyre-contact fallback (no wheel centres)")
                if _gfx_pitch and _placement and _placement.get("pitch_deg"):
                    log.append(f"✓ GRAPHICS_PITCH_ROTATION {_gfx_pitch}° applied "
                               f"(positive = nose down, about the CG)")
                elif _gfx_pitch:
                    log.append(f"ℹ GRAPHICS_PITCH_ROTATION {_gfx_pitch}° recorded in "
                               f"x_assettocorsa.graphics, not applied to the mesh (the SVJ "
                               f"frame is level; AC's pitch compensates its physics rake)")

                # Doors, wipers, shifter: the joints of animations/*.ksanim become movable
                # parts (SVJ 24): a body, a joint, a pose.  The GLB carries the animation.
                if include_animations:
                    try:
                        _moving = describe_moving_parts(kn5_path, car_path,
                                                        placement=_placement)
                        if _moving:
                            svj.setdefault("x_assettocorsa", {})["moving_parts"] = {
                                "frame": "kn5_model",
                                "note": ("pivot_model/axis_model: KN5 model axes "
                                         "(left-handed, Y up), before GRAPHICS_OFFSET; "
                                         "rest pose = frame 0; fps assumed"),
                                "parts": _moving,
                            }
                            _nb, _nj, _ren = movable_parts_to_svj(svj, _moving, mesh_id,
                                                                  bound=set(node_rename))
                            node_rename.update(_ren)
                            log.append(f"✓ Moving parts: {len(_moving)} animated node(s); "
                                       f"{_nb} movable part(s), {_nj} joint(s) → "
                                       f"constraints / poses")
                    except Exception as _mv_err:                # noqa: BLE001
                        log.append(f"⚠ Moving parts skipped: {_mv_err}")

                # ── GLB export (only when caller supplies an output dir) ───
                if glb_output_dir is not None:
                    try:
                        _reports: dict = {}
                        _glb_dir = (glb_output_dir / meshes_subdir
                                    if meshes_subdir else glb_output_dir)
                        exported = kn5_all_lods_to_glbs(
                            car_path, _glb_dir, include_skins=include_skins,
                            node_names=node_rename, kn5_override=kn5_override,
                            placement=_placement, reports=_reports,
                            include_damage_textures=include_damage_textures,
                            highest_lod_only=highest_lod_only,
                            include_animations=include_animations)
                        for lbl, out_p in exported.items():
                            log.append(f"✓ GLB LOD {lbl} written → {_mp}{out_p.name}")
                        _ra = _reports.get("A") or {}
                        if _ra.get("bbox_min"):
                            log.append(f"  mesh: lowest point y={_ra['bbox_min'][1]:+.3f} m, "
                                       f"height {_ra['size'][1]:.3f} m, "
                                       f"length {_ra['size'][2]:.3f} m")
                        if _ra.get("placement_mode") == "physics":
                            _wd = _ra.get("wheel_deltas") or {}
                            _worst = max((abs(v) for d in _wd.values() for v in d),
                                         default=0.0)
                            log.append(
                                f"  wheels placed from physics: "
                                f"{', '.join(_ra['wheels_moved']) or 'none'}"
                                + (f" (model was off by up to {_worst * 100:.1f} cm)"
                                   if _wd else ""))
                            for _st, _why in _ra.get("wheels_skipped", []):
                                log.append(f"  ⚠ wheel {_st} not repositioned: {_why}")
                    except Exception as _glb_err:
                        log.append(f"⚠ GLB export failed: {_glb_err}")

            except Exception as _kn5_err:
                log.append(f"⚠ KN5 scan skipped ({kn5_path.name}): {_kn5_err}")

    if unified_suspension:
        try:
            svj, _notes = migrate_suspension(svj)
            log.append(f"✓ Suspension written as bodies, joints and elements "
                       f"({len(svj.get('wheel_stations', []))} wheel stations, "
                       f"{len(svj.get('constraints', []))} constraints; spec 25)")
        except Exception as _mig_err:                           # noqa: BLE001
            log.append(f"⚠ Unified suspension skipped, corner section kept: {_mig_err}")

    log.append(f"✓ SVJ v{SVJ_VERSION} output built (converter v{CONV_VER})")
    return svj, log, bench_results


# ─── Helpers ────────────────────────────────────────────────────────────────

def _clean(obj):
    """Recursively strip Nones and convert non-finite floats (NaN, inf) to
    None so the JSON output is strict-spec compliant. Python's default
    `json.dumps` emits the literal token `NaN` which most JSON parsers
    (browsers, the SVJ reference viewer, etc.) reject — that surfaced as
    a "Invalid JSON: Unexpected token 'N'" error in the SVJ viewer when
    a Pacejka fit returned r²=NaN on a degenerate axle."""
    import math
    if isinstance(obj, dict):
        return {k: _clean(v) for k, v in obj.items() if v is not None}
    if isinstance(obj, list):
        return [_clean(i) for i in obj]
    if isinstance(obj, float) and not math.isfinite(obj):
        return None
    return obj


def read_car_directory(car_path: Path) -> tuple[dict, Optional[dict], dict, Path]:
    """Load every ini/ctrl/lut file an AC car exposes.

    Strategy:
      1. If the car has an unpacked `data/` folder (mods) → use it directly.
      2. If only `data.acd` exists (stock Kunos cars or unpatched mods) →
         the loader returns an empty ini map and a clear "needs unpack"
         note. Caller should surface this to the user (in batch mode log
         it, then skip the car) rather than emit a silent empty SVJ.
      3. Still honour EXTRA_INI_FILES that might sit at the car root.

    Returns (ini_files, cm_meta, ctrl_files, data_dir, source_note).
    """
    from acd_reader import open_car as _open_car

    ini_files: dict = {}
    ctrl_files: dict = {}

    # Load whatever the car directory exposes (unpacked data/, or empty
    # with a "needs unpack" note if only an encrypted archive is there).
    loaded, data_dir, source_note = _open_car(car_path)
    # Keep only the ini files we care about from the core list…
    for fname in CORE_INI_FILES:
        if fname in loaded:
            ini_files[fname] = loaded[fname]
    # AWD centre-diff controllers (ctr_awd_center_lock.ini): kept beside the core
    # files; build_svj reads them for x_assettocorsa.drivetrain.
    for fname, text in loaded.items():
        if fname.lower().startswith("ctr_") and fname.lower().endswith(".ini"):
            ini_files[fname] = text
    # …plus any ctrl_*.ini / electric_controllers.ini from the same source.
    for fname, text in loaded.items():
        low = fname.lower()
        if low.startswith("ctrl_") and low.endswith(".ini"):
            ctrl_files[fname] = text
        elif low == "electric_controllers.ini":
            ctrl_files[fname] = text

    # Step 3: anything in EXTRA_INI_FILES might also sit at the car root
    # (older mod packaging). Only add if not already loaded.
    for fname in EXTRA_INI_FILES:
        if fname in ini_files:
            continue
        for cand in (data_dir / fname, car_path / fname):
            if cand.is_file():
                try:
                    ini_files[fname] = cand.read_text(encoding="utf-8", errors="replace")
                except OSError:
                    pass
                break

    cm_meta: Optional[dict] = None
    ui_json = car_path / "ui" / "ui_car.json"
    if ui_json.is_file():
        try:
            cm_meta = json.loads(ui_json.read_text(encoding="utf-8", errors="replace"))
        except (json.JSONDecodeError, OSError):
            pass

    # Attach source note to cm_meta so build_svj can surface it in its log.
    cm_meta = cm_meta or {}
    cm_meta.setdefault("_data_source", source_note)
    return ini_files, cm_meta, ctrl_files, data_dir


_FS_INVALID = re.compile(r'[<>:"/\\|?*\x00-\x1f]')
_FS_RESERVED = frozenset({"con", "prn", "aux", "nul",
                          *(f"com{i}" for i in range(1, 10)),
                          *(f"lpt{i}" for i in range(1, 10))})


# ─── Movable parts (SVJ 24) ─────────────────────────────────────────────────

_SIDE_TOKENS = {"lf": "fl", "fl": "fl", "rf": "fr", "fr": "fr", "lr": "rl", "lb": "rl",
                "rl": "rl", "rr": "rr", "rb": "rr", "l": "l", "left": "l", "r": "r",
                "right": "r"}


def _mesh_slug(stem: str) -> str:
    """Mesh id for assets.meshes (the schema allows ``[a-z0-9_]``)."""
    return re.sub(r"[^a-z0-9_]", "_", stem.lower()) or "mesh"


def _part_tokens(node: str) -> tuple:
    """(name tokens without the side token, side token or "") of an AC node name."""
    toks = [t for t in re.split(r"[^a-z0-9]+", node.lower()) if t]
    side = ""
    for i, t in enumerate(toks):
        if t in _SIDE_TOKENS and i > 0:
            side = _SIDE_TOKENS[t]
            toks.pop(i)
            break
    return toks, side


def _part_id(node: str, taken: set, canonical: bool = True,
             side: Optional[str] = None) -> str:
    """Body id for an AC node: lowercase, ``[a-z0-9_]``, side token moved to the end
    (``DOOR_LF`` -> ``door_fl``), unique within ``taken``.  ``side`` replaces the side
    read from the name."""
    toks, name_side = _part_tokens(node)
    side = name_side if side is None else side
    base = "_".join(toks + ([side] if side else [])) or "part"
    if canonical:
        base = canonical_part_id(base) or base      # hatch -> tailgate, bonnet -> hood
    if base[0].isdigit():
        base = "part_" + base
    pid, n = base, 2
    while pid in taken:
        pid, n = f"{base}_{n}", n + 1
    taken.add(pid)
    return pid


def _z_to_quat(axis) -> list:
    """Unit quaternion (x, y, z, w) turning +Z onto ``axis``."""
    ax = np.asarray(axis, dtype="f8")
    ax = ax / np.linalg.norm(ax)
    c = np.cross([0.0, 0.0, 1.0], ax)
    w = 1.0 + float(ax[2])
    if w < 1e-9:                                     # axis = -Z
        return [1.0, 0.0, 0.0, 0.0]
    q = np.array([c[0], c[1], c[2], w])
    q /= np.linalg.norm(q)
    return [round(float(v), 7) for v in q]


# Suspension, wheel and steering animations are the suspension travelling, not parts
# that open or move on their own: the suspension model covers them.
_NOT_MOVABLE = re.compile(r"^(anim_)?(susp|hub|wheel|steer|upright|disc|caliper|brake)",
                          re.I)


# parts that come in front / rear x left / right
_FOUR_CORNER = {"door", "window", "seat"}
# parts whose pose is "open" (spec 24.2)
_OPENS = {"door", "hood", "trunk", "tailgate", "fuel_flap", "charge_flap", "sunroof"}


def _same_movement(a: dict, b: dict) -> bool:
    """True when two animated nodes are one movement: same animation and joint, the same
    axis direction and angle (or travel), pivots within 15 cm."""
    if a["animation"] != b["animation"] or a["joint"] != b["joint"]:
        return False
    ea, eb = ((a.get("angle_rad"), b.get("angle_rad")) if a["joint"] == "revolute"
              else (a.get("travel_m"), b.get("travel_m")))
    if abs(ea - eb) > 0.01 * max(abs(ea), abs(eb)):
        return False
    ua, ub = np.asarray(a["axis_svj"], "f8"), np.asarray(b["axis_svj"], "f8")
    if float(ua @ ub) / (np.linalg.norm(ua) * np.linalg.norm(ub)) < 0.999:
        return False
    return float(np.linalg.norm(np.asarray(a["pivot_svj"]) - np.asarray(b["pivot_svj"]))) <= 0.15


def _member_id(node: str, main_node: str, main_pid: str, taken: set) -> str:
    """Id of a node that moves with `main_node` (`DOOR_L001` with `DOOR_L` = `door_fl`):
    the main id plus what the node name adds to it (`door_fl_001`)."""
    main = set(_part_tokens(main_node)[0]) | set(_SIDE_TOKENS)
    extra = []
    for t in re.split(r"[^a-z0-9]+", node.lower()):
        m = re.fullmatch(r"(?:" + "|".join(sorted(_SIDE_TOKENS, key=len, reverse=True))
                         + r")(\d+)", t)
        t = m.group(1) if m else t                    # `l001`: the number after the side
        if t and t not in main:
            extra.append(t)
    base = "_".join([main_pid] + extra)
    pid, n = base, 2
    while pid in taken:
        pid, n = f"{base}_{n}", n + 1
    taken.add(pid)
    return pid


def movable_parts_to_svj(svj: dict, parts: list, mesh_id: str,
                         bound: Optional[set] = None) -> tuple:
    """Write doors, flaps, wipers ... as SVJ 24 movable parts.

    One ``mass_bodies`` entry per hinge or slide (``x_derived``, mass 0: AC carries no
    masses), a marker on the chassis and on the body at the pivot, a ``joint`` constraint
    (``revolute`` / ``translational``, +Z = the axis, range from the rest pose to the
    animation's end) and one pose per animation.  Parts with no SVJ-frame pivot or of
    ``general`` type, suspension / wheel nodes and nodes in ``bound`` (already carrying
    another visual binding) are left to ``x_assettocorsa.moving_parts``.  Returns
    ``(bodies, joints, {AC node name: glTF node name})``.
    """
    chassis = svj.setdefault("chassis", {})
    bodies = chassis.setdefault("mass_bodies", [])
    markers = chassis.setdefault("markers", {})
    cons = svj.setdefault("constraints", [])
    taken = {b["id"] for b in bodies} | {c["id"] for c in cons}
    rename: dict = {}
    nb = nj = 0

    # An animation that turns a hub, wheel or steering node is the suspension (or the
    # steering with its driver rig) travelling: every node it moves, arms and springs
    # included, is covered by the suspension model and is not a part that opens.
    susp_anims = {m["animation"] for m in parts if _NOT_MOVABLE.match(m["node"])}

    # The movable parts: revolute / prismatic joints with a pivot in the SVJ frame.
    cand, done = [], set()
    for m in parts:
        piv, ax = m.get("pivot_svj"), m.get("axis_svj")
        if (m["joint"] not in ("revolute", "prismatic") or not piv or not ax
                or m["animation"] in susp_anims
                or m["node"] in done or m["node"] in (bound or ())
                or _NOT_MOVABLE.match(m["node"])):
            continue
        if not (m.get("angle_rad") if m["joint"] == "revolute" else m.get("travel_m")):
            continue
        done.add(m["node"])
        cand.append(m)

    # A door is often several KN5 nodes (shell, trim, glass) that one animation turns
    # together about the same axis.  Each keeps its own body; the one with the shortest
    # name is the door itself and gives the others their ids (`door_fl_001`).
    groups: list = []
    for m in sorted(cand, key=lambda q: (len(q["node"]), q["node"])):
        for g in groups:
            if _same_movement(g[0], m):
                g.append(m)
                break
        else:
            groups.append([m])

    # Doors, windows and seats take front / rear and left / right from where they hinge
    # (the standard's `door_fl`): left is -Y, front is the more forward hinge of its side.
    sides: dict = {}
    by_side: dict = {}
    for g in groups:
        toks, _ = _part_tokens(g[0]["node"])
        base = canonical_part_id("_".join(toks)) or "_".join(toks)
        if base in _FOUR_CORNER:
            lr = "l" if g[0]["pivot_svj"][1] < 0 else "r"
            by_side.setdefault((base, lr), []).append(g)
    for (base, lr), gs in by_side.items():
        gs.sort(key=lambda g: -g[0]["pivot_svj"][0])
        for i, g in enumerate(gs[:2]):
            sides[id(g)] = ("f", "r")[i] + lr

    poses: dict = {}
    for g in groups:
        main_pid = None
        for m in g:
            piv, ax = m["pivot_svj"], m["axis_svj"]
            rev = m["joint"] == "revolute"
            extent = m.get("angle_rad") if rev else m.get("travel_m")
            if main_pid is None:
                pid = main_pid = _part_id(m["node"], taken, side=sides.get(id(g)))
            else:
                # trim, glass, handles of the same part: own body, named after the part
                pid = _member_id(m["node"], g[0]["node"], main_pid, taken)
            jid = ("hinge_" if rev else "slide_") + pid
            taken.add(jid)
            frame = {"position": list(piv), "orientation": _z_to_quat(ax)}
            markers[f"{pid}_hinge"] = dict(frame)
            glb_node = f"SVJ::body::{pid}"
            bodies.append({
                "id": pid, "name": m["node"], "category": "attachment",
                "description": (f"Animated node {m['node']} of the AC model "
                                f"({m['animation']}.ksanim); AC gives no mass or inertia."),
                "mass": 0, "position": list(piv), "parent": "chassis", "x_derived": True,
                "markers": {"hinge": dict(frame)},
                "visual": {"mesh_ref": mesh_id, "node": glb_node},
            })
            cons.append({
                "id": jid, "type": "joint",
                "joint_type": "revolute" if rev else "translational",
                "body_a": "chassis", "marker_a": f"{pid}_hinge",
                "body_b": pid, "marker_b": "hinge",
                "range": {"min": 0.0, "max": round(float(extent), 5)},
            })
            poses.setdefault(m["animation"], []).append((pid, jid, round(float(extent), 5)))
            rename[m["node"]] = glb_node
            nb += 1
            nj += 1
    if not nb:
        for k in ("mass_bodies", "markers"):
            if not chassis.get(k):
                chassis.pop(k, None)
        if not cons:
            svj.pop("constraints", None)
        return 0, 0, {}
    pl = svj.setdefault("poses", [{"id": "closed"}])
    for anim, items in poses.items():
        # a door animation also turns its handle / arm: the pose is named after the one
        # openable part in it
        opens = [it[0] for it in items
                 if re.sub(r"_(fl|fr|rl|rr|l|r)$", "", it[0]) in _OPENS]
        if len(opens) == 1:
            name = f"{opens[0]}_open"
        else:
            name = re.sub(r"^car_", "", anim.lower())
        name = re.sub(r"[^a-z0-9_]+", "_", name).strip("_") or "pose"
        pid_, n_ = name, 2
        while pid_ in {p["id"] for p in pl}:
            pid_, n_ = f"{name}_{n_}", n_ + 1
        pl.append({"id": pid_, "values": {jid: ext for _, jid, ext in items}})
    return nb, nj, rename


def _visual_placement(svj: dict, gfx_off: Optional[list],
                      cg_derived: bool, pitch_deg: float = 0.0) -> Optional[dict]:
    """
    Exact placement of the KN5 model in the SVJ frame, from car.ini.

    AC maps the model into its physics frame with ``GRAPHICS_OFFSET`` (origin at
    the centre of gravity).  The SVJ frame has its origin on the ground under the
    front axle, so the model->SVJ shift in AC axes is::

        Tx = 0                      (the lateral offset is not applied)
        Ty = GRAPHICS_OFFSET.y + CG height
        Tz = GRAPHICS_OFFSET.z - (CG to front axle distance)

    and each wheel centre is placed where the SVJ puts it (one tyre radius above
    ground).  Returns None when the CG height was not derived from the car's own
    data (the shift would rest on a guess) or GRAPHICS_OFFSET is missing; the
    ``translation`` is then None and the exporter works it out so the wheel
    nodes' centres match the physics wheel centres.  None only when the SVJ has no
    wheel centres at all.
    """
    cg = (svj.get("chassis") or {}).get("center_of_gravity")
    use_offset = (gfx_off is not None and cg_derived and cg is not None
                  and len(cg) >= 3)
    centers: dict = {}
    for st in ("FL", "FR", "RL", "RR"):
        hp = ((((svj.get("suspension") or {}).get(st) or {}).get("topology") or {})
              .get("upright") or {}).get("hardpoints", {}).get("wheel_center")
        if hp and len(hp) == 3:
            centers[st] = (-float(hp[2]), float(hp[0]))   # (height, forward of front axle)
    if not centers:
        return None
    translation = None
    if use_offset:
        h_cg, d_f = -float(cg[2]), -float(cg[0])
        # The body is pitched about the CG (origin of the physics frame) AFTER the
        # offset: p' = Rx(pitch) (p + offset), then moved to the SVJ frame.  Positive
        # GRAPHICS_PITCH_ROTATION (degrees) lowers the nose: y' = y cos - z sin.
        th = math.radians(pitch_deg)
        oy, oz = float(gfx_off[1]), float(gfx_off[2])
        translation = (0.0,
                       oy * math.cos(th) - oz * math.sin(th) + h_cg,
                       oz * math.cos(th) + oy * math.sin(th) - d_f)
    return {"translation": translation, "wheel_centers": centers, "tolerance": 0.15,
            "pitch_deg": pitch_deg if translation is not None else 0.0}


def _load_diff_controllers(ini_files: dict, data_dir: Optional[Path],
                           log: Optional[list] = None) -> dict:
    """
    AWD centre-diff controllers (``ctr_*.ini``) parsed, with the LUT each controller
    points at read into ``controller["lut"]`` ([[x, y], ...]).  ``{file: parsed}``.
    """
    out: dict = {}
    for name, text in ini_files.items():
        if not name.lower().startswith("ctr_"):
            continue
        try:
            ctl = parse_controller_ini(text)
            for c in ctl.get("controllers", []):
                lut = _resolve_lut(data_dir, c.get("lut_raw")) if data_dir else None
                if lut:
                    c["lut"] = lut
            out[name] = ctl
            if log is not None:
                log.append(f"✓ Drivetrain controller kept: {name} "
                           f"({len(ctl.get('controllers', []))} curves)")
        except Exception as ex:                                   # noqa: BLE001
            if log is not None:
                log.append(f"⚠ {name} not parsed: {ex}")
    return out


_COMBINE = {"MULT": "multiply", "ADD": "add"}


def _lock_control(controllers: dict) -> Optional[dict]:
    """
    A differential's dynamic lock, in the shape proposed for the SVJ standard
    (``proposals/svj_differential_control_proposal.md``): an ordered list of stages,
    each reading an input, mapping it through a curve and combining the result.
    Emitted as ``lock_control`` (SVJ v0.99.3).  AC's ``FILTER`` is a smoothing
    coefficient, not a time constant in seconds, so it is kept as ``x_filter`` rather
    than guessed into ``filter_time_constant``.
    The AC semantics of the combination are taken from the controller files as
    written (INPUT, COMBINATOR, LUT, FILTER, UP_LIMIT, DOWN_LIMIT); ``target`` is the
    differential's lock.
    """
    stages: list = []
    sources: list = []
    for name, ctl in controllers.items():
        for c in ctl.get("controllers", []):
            st: dict = {"input": str(c.get("input") or "").lower(),
                        "combine": _COMBINE.get(str(c.get("combinator") or "").upper(),
                                                str(c.get("combinator") or "").lower())}
            if not c.get("lut") or len(c["lut"]) < 2:
                continue                      # a stage needs a curve (curve2d: >= 2 points)
            st["curve"] = c["lut"]
            for src, dst in (("filter", "x_filter"), ("down_limit", "min"), ("up_limit", "max")):
                if c.get(src) is not None:
                    st[dst] = c[src]
            stages.append(st)
        sources.append(name)
    if not stages:
        return None
    return {"target": "lock", "stages": stages, "_source": sources}


def _build_driveline(dt: dict, layout: str, diff_type: str,
                     diff_loc: str,
                     controllers: Optional[dict] = None) -> tuple[list, Optional[dict]]:
    """
    ``(differentials, transfer_case)`` from the parsed drivetrain.ini.

    AWD (``[AWD]``): a front and a rear differential from ``FRONT_DIFF_*`` /
    ``REAR_DIFF_*`` and a ``full_time`` transfer case whose ``torque_split`` is
    ``[front_share, 1 - front_share]`` and whose ``center_diff`` carries
    ``CENTRE_DIFF_*``.  ``TYPE=AWD2`` (``[AWD2]``) has no fixed split: a torque-limited
    centre coupling, so ``on_demand`` with ``max_torque`` / ``ramp_torque`` on the
    centre diff and no ``torque_split``.  Without AWD data (or for RWD / FWD) the
    single ``[DIFFERENTIAL]`` differential is emitted as before.
    """
    final = dt["gearbox"].get("final_drive", 3.5)

    def diff(loc: str, vals: dict) -> dict:
        d = {"id": f"diff_{loc}", "location": loc, "type": diff_type, "final_drive": final}
        if vals.get("preload") is not None:
            d["preload"] = max(float(vals["preload"]), 0.0)
        for src, dst in (("power", "lock_power"), ("coast", "lock_coast")):
            if vals.get(src) is not None:
                d[dst] = min(max(float(vals[src]), 0.0), 1.0)
        return d

    awd = dt.get("awd")
    if awd and (awd.get("front") or awd.get("rear")):
        diffs = [diff(loc, awd[loc]) for loc in ("front", "rear") if awd.get(loc)]
        tc: dict = {"type": "on_demand" if awd["model"] == "awd2" else "full_time"}
        fs = awd.get("front_share")
        if fs is not None:
            fs = min(max(float(fs), 0.0), 1.0)
            tc["torque_split"] = [round(fs, 4), round(1.0 - fs, 4)]
        cen = awd.get("centre") or {}
        if any(cen.get(k) is not None for k in ("power", "coast", "preload")):
            cd = {"type": diff_type, "location": "center"}
            if cen.get("preload") is not None:
                cd["preload"] = max(float(cen["preload"]), 0.0)
            for src, dst in (("power", "lock_power"), ("coast", "lock_coast")):
                if cen.get(src) is not None:
                    cd[dst] = min(max(float(cen[src]), 0.0), 1.0)
            tc["center_diff"] = cd
        # AWD2: a torque-limited centre coupling with no fixed split
        # (transfer_case.coupling, SVJ v0.99.3; max_torque is required there).
        _cpl = {k: cen[k] for k in ("ramp_torque", "max_torque") if cen.get(k) is not None}
        if _cpl.get("max_torque", 0) > 0:
            if _cpl.get("ramp_torque", 0) > _cpl["max_torque"]:
                _cpl.pop("ramp_torque")           # schema: ramp_torque <= max_torque
            tc["coupling"] = _cpl
        elif _cpl:
            tc["x_coupling"] = _cpl
        ctl = _lock_control(controllers or {})
        if ctl:
            if "center_diff" in tc:
                tc["center_diff"]["lock_control"] = ctl
            else:
                tc["x_lock_control"] = ctl
        return diffs, tc
    return [diff(diff_loc, dt["differential"])], None


def _fs_safe(name: str) -> str:
    """Lowercase, spaces -> '_', and characters Windows forbids in file names
    (< > : " / \\ | ? * and control chars) removed; trailing dots/spaces trimmed;
    reserved device names (CON, NUL, COM1, ...) rejected. May return ''."""
    stem = _FS_INVALID.sub("", str(name)).strip().lower().replace(" ", "_")[:40].rstrip(" .")
    return "" if stem.split(".")[0] in _FS_RESERVED else stem


def _car_stem(svj: dict, fallback: str) -> str:
    """File-name stem for a car's outputs, from its model name. The display name
    can hold anything (e.g. ``3.2 (E36) "Regional Rally"``), so it is made safe
    for use as a file name; names that were already safe are unchanged."""
    name = svj.get("vehicle_info", {}).get("model") or fallback
    return _fs_safe(name) or _fs_safe(fallback) or "car"


if __name__ == "__main__":
    import subprocess, sys
    subprocess.run([sys.executable, str(Path(__file__).with_name("gui.py"))])
