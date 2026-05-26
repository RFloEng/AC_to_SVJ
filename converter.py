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
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

from ac_parsers import (
    parse_ini, parse_lut,
    parse_engine, parse_drivetrain, parse_suspension_axle,
    parse_brakes, parse_aero, parse_car_extras,
    detect_tyre_compounds, parse_tyre_extras, parse_tyre_thermal,
    parse_controller_ini, parse_setup_ranges,
    _parse_vec3, _f, _i, _m, _b, _first,
)
from tire_lab import (
    ACTyreParams, parse_tyre_section, run_bench,
    build_svj_pacejka_block, build_svj_mf62_block,
    build_svj_pacejka_blocks, BenchResult,
)
try:
    from kn5_reader import (scan_kn5_nodes, map_ac_nodes_to_svj,
                            find_car_kn5, find_car_kn5_lods,
                            kn5_to_glb, kn5_all_lods_to_glbs)
    _KN5_AVAILABLE = True
except ImportError:
    _KN5_AVAILABLE = False

# ─── Constants ────────────────────────────────────────────────────────────────

SVJ_VERSION = "0.97"
SVJ_SPEC    = "SVJ"
CONV_VER    = "0.9.1"
REPO_URL    = "https://github.com/RFloEng/SVJ-standard-vehicle-json"
CONV_REPO   = "https://github.com/RFloEng/AC_to_SVJ"

CORE_INI_FILES = [
    "car.ini", "engine.ini", "drivetrain.ini", "suspensions.ini",
    "tyres.ini", "brakes.ini", "aero.ini",
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
DRIVE_LAYOUT_MAP = {"RWD": "FR", "FWD": "FF", "AWD": "AWD", "4WD": "4WD"}
DIFF_TYPE_MAP = {
    "LSD": "lsd_clutch", "SALISBURY": "lsd_clutch",
    "CLUTCH": "lsd_clutch", "OPEN": "open", "SPOOL": "spool",
    "TORSEN": "torsen", "VISCOUS": "viscous",
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
                 cg_height: float = 0.42) -> dict:
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
    _ = cg_height  # kwarg kept for API compatibility; not used today

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

    links: list = []
    if upper_front and upper_rear:
        links.append({
            # "wishbone" is reserved for true double-A-arm geometry; multi-link
            # and MacPherson equivalents use "control_arm" (matches svjrepo
            # reference templates).
            "name": ("upper_wishbone" if sys_type == "double_wishbone" else "trailing_arm_upper" if sys_type == "trailing_arm" else "semi_trailing_arm_upper" if sys_type == "semi_trailing_arm" else "upper_control_arm"),
            "type": "arm",
            "inboard_points": [upper_front, upper_rear],
            "outboard_ref": "hardpoints.upper_ball_joint",
        })
    if lower_front and lower_rear:
        links.append({
            "name": ("lower_wishbone" if sys_type == "double_wishbone" else "trailing_arm_lower" if sys_type == "trailing_arm" else "semi_trailing_arm_lower" if sys_type == "semi_trailing_arm" else "lower_control_arm"),
            "type": "arm",
            "inboard_points": [lower_front, lower_rear],
            "outboard_ref": "hardpoints.lower_ball_joint",
        })
    if tie_chassis and hp_svj.get("steering_tie_rod_end"):
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
                "type": "rod",
                "inboard_points": [strut_car],
                "outboard_ref": "hardpoints.strut_outboard",
            })
        else:
            links.append({
                "name": "damper_link",
                "type": "rod",
                "inboard_points": [strut_car],
                "outboard_ref": "hardpoints.damper_outboard",
            })

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

    spring = {
        "type":          "coilover",
        "rate":          round(spring_k, 1),
        "motion_ratio":  1.0,
        "axis":          "vertical",
        "inboard_point":  spring_inboard,
        "outboard_point": spring_outboard,
        "_note": "AC spring rate is already wheel-rate — axis taken as vertical "
                 "through the wheel centre (AC does not model pickup points)",
    }
    if susp_axle.get("progressive"):
        spring["progressive_factor"] = susp_axle["progressive"]
    if susp_axle.get("rod_length"):
        spring["rod_length"] = susp_axle["rod_length"]

    damper = {
        "type":           "monotube",
        "bump_curve":     make_damper_curve(bump_raw),
        "rebound_curve":  make_damper_curve(reb_raw, factor=1.5),
        "motion_ratio":   1.0,
        "axis":           "vertical",
        "inboard_point":  spring_inboard,
        "outboard_point": spring_outboard,
        "_est":           True,
        "_note": "AC damping is specified at wheel — vertical axis at wheel centre, "
                 "1:1 motion ratio. Curves synthesised from AC scalar bump/rebound.",
    }
    bump = susp_axle.get("bumpstop")
    if bump:
        damper["bumpstops"] = bump

    geometry_note = None
    if not links:
        geometry_note = "Hardpoints not found in suspensions.ini — using spring/damper only"

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

    topology = {
        "system_type": sys_type,
        "upright": {
            "id": f"upright_{corner_id}",
            "hardpoints": hp_svj,
        },
        "links": links if links else [{
            "name": "spring_damper_only",
            "type": "virtual",
            "inboard_points": [spring_inboard],
            "outboard_ref": "hardpoints.wheel_center",
            "outboard_point": list(hp_svj["wheel_center"]),
            "_note": "No AC hardpoints — vertical spring/damper axis only",
        }],
    }
    if geometry_note:
        topology["_note"] = geometry_note

    corner = {
        "position":        wheel_center,   # corner location in vehicle frame
        "topology":        topology,
        "wheel": {
            "rim_diameter": round(0.432, 4),
            "rim_width":    0.20,
            "set_ref":      tire_set_ref,
        },
        "spring": spring,
        "damper": damper,
        "arb": {
            "bar_id":   "arb_front" if is_front else "arb_rear",
            "bar_rate": round(arb_k, 1),
            "_est":     True,
        },
        "alignment": {
            "camber": round(camber, 5),
            "toe":    round(toe, 5),
        },
        "brake": {"disc": disc_obj,
                  "max_torque_nm": round(max_torque, 1)},
        "tire": {"set_ref": tire_set_ref},
        "static_setup": {"ride_height": round(ride_h, 4)},
    }
    if caster is not None:
        corner["alignment"]["caster"] = round(caster, 5)
    if kpi is not None:
        corner["alignment"]["kpi"] = round(kpi, 5)

    wm = susp_axle.get("unsprung_mass")
    if wm:
        corner["wheel"]["mass"] = round(wm / 2, 2)
    if susp_axle.get("tyre_offset"):
        corner["static_setup"]["track_offset"] = susp_axle["tyre_offset"]
    return corner


# ─── Pacejka fit per axle (per compound when multi) ──────────────────────────

def fit_tyre_axle_section(
        tyres_ini_dict: dict, section: str, axle: str, log: list,
) -> tuple[Optional[dict], Optional[dict], Optional[BenchResult]]:
    """
    Returns (mf52_block, mf62_block, BenchResult).
    Both blocks are None when the section has no usable AC parameters.
    """
    p = parse_tyre_section(tyres_ini_dict, section=section, axle=axle)
    if p.source == "defaults":
        log.append(f"  ⚠ {axle} [{section}]: no usable params — Pacejka blocks omitted")
        return None, None, None
    result = run_bench(p)
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
              ) -> tuple[dict, list[str], dict]:
    """
    Returns (svj_dict, log_lines, bench_results_by_axle_compound).

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
    cm_weight = float(re.sub(r"[^0-9.]", "", cm_weight_str)) if cm_weight_str else None
    total_mass = _f(cW.get("MINIMUM"), cm_weight or 1200.0)

    # ── Suspension parsed blocks ─────────────────────────────────────────────
    sa_f = parse_suspension_axle(sf)
    sa_r = parse_suspension_axle(sr)

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
    cg_x = -round(wheelbase * (1.0 - rear_bias), 3)
    cg_z = -round(_f(cB.get("CG_HEIGHT") or cB.get("CGHEIGHT"), 0.42), 3)
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
        return {
            "description":    f"AC-derived tire {round(tw*1000)}/{round(asp*100)}R{rim_in} "
                              f"({label}{compound_suffix})",
            "size_code":      f"{round(tw*1000)}/{round(asp*100)}R{rim_in}",
            "source":         "estimated",
            "dimensions": {
                "section_width":     round(tw, 4),
                "aspect_ratio":      round(asp, 3),
                "rim_diameter_code": rim_in,
                "overall_diameter":  overall,
                "section_height":    round(tw * asp, 4),
            },
            "rim": {"diameter": round(rim_in * 0.0254, 4),
                    "width_nominal": round(tw, 4)},
            "pressure": 210000.0,
            "_est": True,
            "_note": "Tire dimensions from tyres.ini. See pacejka (MF62) and pacejka_mf52 for force models.",
        }

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
        pf52, pf62, br_f = fit_tyre_axle_section(t, f_sec, f"front{suffix}", log)
        pr52, pr62, br_r = fit_tyre_axle_section(t, r_sec, f"rear{suffix}",  log)
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

    def _lut_to_aero_map(lut, coeff_name: str) -> dict:
        """AC LUT (list of [aoa_deg, coeff]) → SVJ 0.95 aero_map (1D)."""
        return {
            "type": "1d",
            "x_axis": "pitch_angle",
            "values": [{"x": float(p[0]), "value": float(p[1])} for p in (lut or [])
                       if isinstance(p, (list, tuple)) and len(p) >= 2],
            "interpolation": "linear",
            "extrapolation": "clamp",
            "_note": "x values are degrees (AC convention).",
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
        if wing.get("aoa_cl_curve"):
            wc.setdefault("maps", {})["Cl_vs_angle"] = _lut_to_aero_map(
                wing["aoa_cl_curve"], "Cl")
        if wing.get("aoa_cd_curve"):
            wc.setdefault("maps", {})["Cd_vs_angle"] = _lut_to_aero_map(
                wing["aoa_cd_curve"], "Cd")
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

    # data_origin (SVJ 0.95 §1) — confidence reflects how the AC data
    # reached us: unpacked data/ is verbatim from the car; uploads are
    # less verifiable so we drop a notch.
    _source_note = (cm_meta.get("_data_source") if cm_meta else None) or "unknown"
    if "unpacked data/" in _source_note:
        _origin_confidence = "medium"
    else:
        _origin_confidence = "low"

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
            "variant": car_class or None, "drive_type": layout,
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
                               rolling_radius=rolling_radius_f, cg_height=-cg_z),
            "FR": build_corner("FR", sa_f, wheelbase, track_f, track_r,
                               disc_in_f, disc_rad_f, brake_torq_f, brake_bias, ts_front_default,
                               rolling_radius=rolling_radius_f, cg_height=-cg_z),
            "RL": build_corner("RL", sa_r, wheelbase, track_f, track_r,
                               disc_in_r, disc_rad_r, brake_torq_r, brake_bias, ts_rear_default,
                               rolling_radius=rolling_radius_r, cg_height=-cg_z),
            "RR": build_corner("RR", sa_r, wheelbase, track_f, track_r,
                               disc_in_r, disc_rad_r, brake_torq_r, brake_bias, ts_rear_default,
                               rolling_radius=rolling_radius_r, cg_height=-cg_z),
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
            "differentials": [{
                "id":          f"diff_{diff_loc}",
                "location":    diff_loc,
                "type":        diff_type,
                "final_drive": dt["gearbox"].get("final_drive", 3.5),
                "preload":     dt["differential"]["preload"],
                "lock_power":  dt["differential"]["power"],
                "lock_coast":  dt["differential"]["coast"],
            }],
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

    x_ac: dict = {}
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
        kn5_lods = find_car_kn5_lods(car_path)   # [("A", path), ("B", path), ...]
        if not kn5_lods:
            log.append(f"ℹ KN5 not found in {car_path} — visual bindings and GLB skipped")
        else:
            kn5_path = kn5_lods[0][1]            # LOD A is always first
            try:
                node_names = scan_kn5_nodes(kn5_path)
                body_map   = map_ac_nodes_to_svj(node_names)  # {svj_id: ac_name}
                glb_uri    = f"meshes/{kn5_path.stem}.glb"
                mesh_id    = kn5_path.stem.lower().replace("-", "_").replace(" ", "_")

                # Build assets.meshes list — LOD A first, then B/C/D entries
                mesh_entries = [
                    {
                        "id":          mesh_id,
                        "uri":         glb_uri,
                        "description": f"Converted from {kn5_path.name} (LOD A)",
                    }
                ]
                for lod_label, lod_path in kn5_lods[1:]:
                    lod_id  = lod_path.stem.lower().replace("-", "_").replace(" ", "_")
                    lod_uri = f"meshes/{lod_path.stem}.glb"
                    mesh_entries.append({
                        "id":          lod_id,
                        "uri":         lod_uri,
                        "description": f"Converted from {lod_path.name} (LOD {lod_label})",
                        "lod":         lod_label,
                    })

                svj["assets"] = {"meshes": mesh_entries}

                # chassis visual binding (always points at LOD A mesh)
                if "chassis" in body_map:
                    svj["chassis"]["visual"] = {
                        "mesh_ref": mesh_id,
                        "node":     f"SVJ::body::chassis",
                    }

                # suspension corner visual bindings (upright per corner)
                _CORNER_MAP = {
                    "FL": "upright_fl",
                    "FR": "upright_fr",
                    "RL": "upright_rl",
                    "RR": "upright_rr",
                }
                for corner, svj_id in _CORNER_MAP.items():
                    if svj_id in body_map and corner in svj.get("suspension", {}):
                        svj["suspension"][corner]["visual"] = {
                            "mesh_ref": mesh_id,
                            "node":     f"SVJ::body::{svj_id}",
                        }

                lod_labels = [lbl for lbl, _ in kn5_lods]
                log.append(
                    f"✓ KN5 visual bindings: {len(body_map)} nodes mapped "
                    f"from {kn5_path.name} (LODs: {', '.join(lod_labels)}) → {glb_uri}"
                )

                # ── GLB export (only when caller supplies an output dir) ───
                if glb_output_dir is not None:
                    try:
                        _glb_dir = glb_output_dir / "meshes"
                        exported = kn5_all_lods_to_glbs(
                            car_path, _glb_dir, include_skins=include_skins)
                        for lbl, out_p in exported.items():
                            log.append(f"✓ GLB LOD {lbl} written → meshes/{out_p.name}")
                    except Exception as _glb_err:
                        log.append(f"⚠ GLB export failed: {_glb_err}")

            except Exception as _kn5_err:
                log.append(f"⚠ KN5 scan skipped ({kn5_path.name}): {_kn5_err}")

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


def _car_stem(svj: dict, fallback: str) -> str:
    name = svj.get("vehicle_info", {}).get("model") or fallback
    return name.lower().replace(" ", "_")[:40]


if __name__ == "__main__":
    import subprocess, sys
    subprocess.run([sys.executable, str(Path(__file__).with_name("gui.py"))])
