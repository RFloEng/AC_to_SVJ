"""
AC → SVJ Converter  v0.99
Sub-module: AC-specific parsers for .ini, .lut, turbos, controllers, etc.

These helpers keep converter.py focused on SVJ assembly while isolating the
(often messy) AC file-format details here.
"""
from __future__ import annotations

import re
from pathlib import Path
from typing import Optional


# ─── low-level parse helpers ──────────────────────────────────────────────────

def parse_ini(text: str) -> dict:
    """AC .ini → dict[SECTION][KEY]=str.  Keys upper-cased, `;` is comment."""
    obj: dict = {}
    section: Optional[str] = None
    for raw in text.splitlines():
        line = raw.strip()
        if not line or line[0] in (";", "/", "#"):
            continue
        m = re.match(r"^\[([^\]]+)\]", line)
        if m:
            section = m.group(1).upper()
            obj.setdefault(section, {})
            continue
        if section and "=" in line:
            k, _, v = line.partition("=")
            # Strip trailing comments — `;` (Kunos convention) and `//` (some mods).
            obj[section][k.strip().upper()] = v.split(";")[0].split("//")[0].strip()
    return obj


def parse_lut(text: str) -> list[list[float]]:
    """
    AC .lut → list of [x, y] pairs.  Handles `|`, tabs, whitespace delimiters,
    comments with `;`, blank lines, and trailing commas.
    """
    pairs: list[list[float]] = []
    for raw in text.splitlines():
        line = raw.strip()
        if not line or line[0] in (";", "/", "#"):
            continue
        # strip trailing comment
        line = line.split(";")[0].strip()
        if not line:
            continue
        # unify delimiters
        line = line.replace("|", " ").replace("\t", " ").replace(",", " ")
        parts = [p for p in line.split(" ") if p]
        if len(parts) < 2:
            continue
        try:
            pairs.append([float(parts[0]), float(parts[1])])
        except ValueError:
            continue
    return pairs


def _f(v, default: float = 0.0) -> float:
    try:
        return float(v)
    except Exception:
        return default


def _i(v, default: int = 0) -> int:
    try:
        return int(float(v))
    except Exception:
        return default


def _m(v) -> Optional[float]:
    try:
        return float(v)
    except Exception:
        return None


def _b(v) -> Optional[bool]:
    if v is None:
        return None
    s = str(v).strip().upper()
    if s in ("1", "TRUE", "YES", "ON"):
        return True
    if s in ("0", "FALSE", "NO", "OFF"):
        return False
    return None


def _first(*vals, default=None):
    for v in vals:
        r = _m(v)
        if r is not None:
            return r
    return default


# ─── engine.ini parsing (power.lut, coast, turbos, damage, throttle) ──────────

def _resolve_lut(data_dir: Path, ref: Optional[str]) -> Optional[list[list[float]]]:
    """Locate and read a .lut file referenced from an ini header."""
    if not ref or not data_dir:
        return None
    ref = ref.strip().strip('"').strip("'")
    candidate = data_dir / ref
    if not candidate.is_file():
        # AC sometimes stores extension-less references
        if (data_dir / (ref + ".lut")).is_file():
            candidate = data_dir / (ref + ".lut")
        else:
            return None
    try:
        return parse_lut(candidate.read_text(encoding="utf-8", errors="replace"))
    except OSError:
        return None


def parse_engine(ini: dict, data_dir: Optional[Path]) -> dict:
    """
    Parse engine.ini into a structured dict.
    Returns keys: version, idle_rpm, max_rpm, inertia, limiter_hz,
      altitude_sensitivity, torque_curve, coast_curve, coast_ref,
      turbos, bov, dynamic_turbo, throttle_response, damage,
      fuel_consumption_k, source_flags
    """
    hdr = ini.get("HEADER", {})
    eD = ini.get("ENGINE_DATA", {})
    d = ini.get("DAMAGE", {})
    tr = ini.get("THROTTLE_RESPONSE", {})
    bov = ini.get("BOV", ini.get("BLOWOFF", {}))
    dyn = ini.get("DYNAMIC_TURBO", {})
    coast_ref = ini.get("COAST_REF", ini.get("COAST_REFERENCE", {}))
    fuelcons = ini.get("FUEL_CONSUMPTION", ini.get("CONSUMPTION", {}))

    out: dict = {
        "version":          hdr.get("VERSION"),
        "idle_rpm":         _f(eD.get("MINIMUM"), 800.0),
        "max_rpm":          _f(eD.get("LIMITER"), 7000.0),
        "inertia":          _f(eD.get("INERTIA"), 0.09),
        "limiter_hz":       _m(eD.get("LIMITER_HZ")),
        "altitude_sensitivity": _m(eD.get("ALTITUDE_SENSITIVITY")),
        "throttle_response":_m(tr.get("CURVE") or eD.get("THROTTLE_RESPONSE")),
        "fuel_consumption_k": _m(fuelcons.get("K") or fuelcons.get("RATE")),
        "torque_curve":     None,
        "coast_curve":      None,
        "turbos":           [],
        "bov":              None,
        "dynamic_turbo":    None,
        "damage":           {},
        "source_flags":     {"power_lut": False, "coast_lut": False},
    }

    # Power & coast LUTs
    pc_ref = hdr.get("POWER_CURVE")
    cc_ref = hdr.get("COAST_CURVE")
    if data_dir:
        pc = _resolve_lut(data_dir, pc_ref) if pc_ref else None
        cc = _resolve_lut(data_dir, cc_ref) if cc_ref else None
        if pc:
            out["torque_curve"] = pc
            out["source_flags"]["power_lut"] = True
        if cc:
            out["coast_curve"] = cc
            out["source_flags"]["coast_lut"] = True

    # Inline coast reference (fallback)
    if out["coast_curve"] is None and coast_ref:
        rpm = _m(coast_ref.get("RPM"))
        nm = _m(coast_ref.get("TORQUE"))
        non_rpm = _m(coast_ref.get("NON_LINEARITY"))
        if rpm and nm:
            # model a simple exponent curve: T_coast(w) = -Nm * (w/Rpm)^(1+k)
            k = non_rpm or 0.0
            max_rpm = out["max_rpm"]
            pts = []
            for r in [0.0, out["idle_rpm"], 0.25 * max_rpm, 0.5 * max_rpm,
                      0.75 * max_rpm, max_rpm]:
                ratio = (r / rpm) if rpm else 0.0
                pts.append([round(r), round(-nm * (ratio ** (1 + k)), 2)])
            out["coast_curve"] = pts

    # Turbos
    for i in range(8):
        sec = ini.get(f"TURBO_{i}")
        if not sec:
            continue
        turbo = {
            "id":                f"turbo_{i}",
            "lag_up":            _m(sec.get("LAG_UP")),
            "lag_down":          _m(sec.get("LAG_DN") or sec.get("LAG_DOWN")),
            "max_boost":         _m(sec.get("MAX_BOOST")),
            "wastegate":         _m(sec.get("WASTEGATE")),
            "display_max_boost": _m(sec.get("DISPLAY_MAX_BOOST") or sec.get("MAX_BOOST")),
            "reference_rpm":     _m(sec.get("REFERENCE_RPM")),
            "gamma":             _m(sec.get("GAMMA")),
            "cockpit_adjustable": _b(sec.get("COCKPIT_ADJUSTABLE")),
        }
        # drop None
        turbo = {k: v for k, v in turbo.items() if v is not None}
        out["turbos"].append(turbo)

    # BOV
    if bov:
        out["bov"] = {
            "pressure_threshold": _m(bov.get("PRESSURE_THRESHOLD")),
            "closing_time":       _m(bov.get("CLOSING_TIME")),
        }
        out["bov"] = {k: v for k, v in out["bov"].items() if v is not None}
        if not out["bov"]:
            out["bov"] = None

    # Dynamic turbo (CSP)
    if dyn:
        out["dynamic_turbo"] = {k.lower(): _m(v) for k, v in dyn.items()
                                if _m(v) is not None}

    # Damage
    if d:
        out["damage"] = {
            "turbo_boost_threshold": _m(d.get("TURBO_BOOST_THRESHOLD")),
            "turbo_damage_k":        _m(d.get("TURBO_DAMAGE_K")),
            "rpm_threshold":         _m(d.get("RPM_THRESHOLD")),
            "rpm_damage_k":          _m(d.get("RPM_DAMAGE_K")),
        }
        out["damage"] = {k: v for k, v in out["damage"].items() if v is not None}

    return out


# ─── drivetrain.ini parsing (beyond ratios) ───────────────────────────────────

def parse_drivetrain(ini: dict) -> dict:
    hdr = ini.get("HEADER", {})
    trac = ini.get("TRACTION", {})
    gbx = ini.get("GEARBOX", {})
    grs = ini.get("GEARS", {})
    diff = ini.get("DIFFERENTIAL", {})
    autoc = ini.get("AUTOCLUTCH", ini.get("AUTO_CLUTCH", {}))
    autob = ini.get("AUTOBLIP", {})
    dsp = ini.get("DOWNSHIFT_PROFILER", {})
    dmg = ini.get("DAMAGE", {})

    # Ratios
    count = _i(grs.get("COUNT"), 6)
    ratios: list[float] = []
    for i in range(1, count + 1):
        r = _m(grs.get(f"GEAR_{i}") or grs.get(f"RATIO_{i}"))
        if r:
            ratios.append(round(r, 4))

    out = {
        "version":     hdr.get("VERSION"),
        "layout_raw":  (trac.get("TYPE") or "RWD").upper().strip(),
        "rear_bias":   _m(trac.get("REAR_BIAS") or trac.get("REARBIASRATIO")),
        "gearbox": {
            "ratios":             ratios,
            "final_drive":        _f(grs.get("FINAL") or grs.get("FINAL_RATIO"), 3.5),
            "count":              count,
            "change_up_s":        _m(gbx.get("CHANGE_UP_TIME")),
            "change_down_s":      _m(gbx.get("CHANGE_DN_TIME") or gbx.get("CHANGE_DOWN_TIME")),
            "auto_cutoff_s":      _m(gbx.get("AUTO_CUTOFF_TIME")),
            "supports_shifter":   _b(gbx.get("SUPPORTS_SHIFTER")),
            "valid_shift_window": _m(gbx.get("VALID_SHIFT_RPM_WINDOW")),
            "controls_gain":      _m(gbx.get("CONTROLS_WINDOW_GAIN")),
            "inertia":            _m(gbx.get("INERTIA")),
        },
        "differential": {
            "type":    (diff.get("TYPE") or "LSD").upper(),
            "power":   _f(diff.get("POWER"), 0.5),
            "coast":   _f(diff.get("COAST"), 0.3),
            "preload": _f(diff.get("PRELOAD"), 50.0),
        },
        "autoclutch":     {k.lower(): _m(v) or _b(v) or v for k, v in autoc.items()} if autoc else None,
        "autoblip":       {k.lower(): _m(v) or _b(v) or v for k, v in autob.items()} if autob else None,
        "shift_profiler": {k.lower(): _m(v) or v for k, v in dsp.items()} if dsp else None,
        "damage": {
            "clutch_torque": _m(dmg.get("CLUTCH_TORQUE")),
            "rpm_threshold": _m(dmg.get("RPM_THRESHOLD")),
            "gear_damage":   _m(dmg.get("GEAR_DAMAGE_RATE")),
        } if dmg else {},
    }
    # Strip Nones from gearbox
    out["gearbox"] = {k: v for k, v in out["gearbox"].items() if v is not None}
    if out["damage"]:
        out["damage"] = {k: v for k, v in out["damage"].items() if v is not None}
    return out


# ─── suspensions.ini — geometry & bumpstops ───────────────────────────────────

HARDPOINT_KEYS = [
    # Double-wishbone / multi-link / MacPherson
    "WBCAR_TOP_FRONT", "WBCAR_TOP_REAR",
    "WBCAR_BOTTOM_FRONT", "WBCAR_BOTTOM_REAR",
    "WBCAR_STEER", "WBCAR_TIE",
    "WBTYRE_TOP", "WBTYRE_BOTTOM", "WBTYRE_STEER",
    "TIE_ROD_FRONT", "TIE_ROD_REAR",
    "TRACK_ROD", "STRUT_CAR", "STRUT_TYRE",
    "ROD_INBOARD", "ROD_OUTBOARD",
    # Solid-axle / live-axle trailing + lateral-location links
    "LINK_0_CAR", "LINK_0_AXLE",
    "LINK_1_CAR", "LINK_1_AXLE",
    "LINK_2_CAR", "LINK_2_AXLE",
    "LINK_3_CAR", "LINK_3_AXLE",
    "PANHARD_CAR", "PANHARD_AXLE",
    "WATTS_CAR",  "WATTS_AXLE",
    # Trailing-arm / semi-trailing-arm (rare in stock AC, present in mods)
    "TRAIL_CAR", "TRAIL_AXLE",
    "SEMI_TRAIL_CAR", "SEMI_TRAIL_AXLE",
]


def _parse_vec3(s: Optional[str]) -> Optional[list[float]]:
    """AC geometry values are comma or pipe separated triples, AC axes."""
    if not s:
        return None
    s = s.replace("|", ",").replace(";", ",")
    parts = [p.strip() for p in s.split(",") if p.strip()]
    try:
        if len(parts) >= 3:
            return [float(parts[0]), float(parts[1]), float(parts[2])]
    except ValueError:
        return None
    return None


def parse_suspension_axle(sec: dict) -> dict:
    """
    Extract geometry + bumpstops + damping characteristics for one axle.
    Returns dict with: hardpoints{AC axes}, bumpstops, rod_length, progressive,
    unsprung_mass, rates.
    """
    hp: dict[str, list[float]] = {}
    for k in HARDPOINT_KEYS:
        v = _parse_vec3(sec.get(k))
        if v is not None:
            hp[k] = v

    bumpstop = {
        "rate":  _m(sec.get("BUMPSTOP_RATE")),
        "up":    _m(sec.get("BUMPSTOP_UP")),
        "down":  _m(sec.get("BUMPSTOP_DN") or sec.get("BUMPSTOP_DOWN")),
    }
    bumpstop = {k: v for k, v in bumpstop.items() if v is not None}

    out = {
        "hardpoints_ac":  hp or None,  # AC axes, caller transforms
        "bumpstop":       bumpstop or None,
        "rod_length":     _m(sec.get("ROD_LENGTH")),
        "progressive":    _m(sec.get("PROGRESSIVE_SPRING_FACTOR")),
        "unsprung_mass":  _first(sec.get("WHEEL_MASS"), sec.get("HUB_MASS"),
                                 sec.get("UNSPRUNG_MASS"), default=None),
        "spring_rate":    _m(sec.get("SPRING_RATE") or sec.get("SPRING")),
        "bump_rate":      _m(sec.get("BUMP") or sec.get("DAMP_BUMP")),
        "rebound_rate":   _m(sec.get("REBOUND") or sec.get("DAMP_REBOUND")),
        "arb_rate":       _m(sec.get("ARB") or sec.get("ANTI_ROLL_K")),
        # AC's BASEY is *not* ride height — Kunos defines it as
        # "Distance of CG from the centre of the wheel; FrontWheelRadius+BASEY=front CoG".
        # We expose it as `basey` so the geometry transform can lift WBCAR_*
        # pickups into ground-relative SVJ coords. `ride_height` only falls back
        # to BASEY when no explicit RIDE_HEIGHT is present (and is then a poor
        # proxy — preserved for backward compat with v0.99-and-earlier consumers).
        "basey":          _m(sec.get("BASEY")),
        "ride_height":    _first(sec.get("RIDE_HEIGHT"), sec.get("BASEY"),
                                 default=None),
        "camber":         _first(sec.get("CAMBER"), sec.get("STATIC_CAMBER"),
                                 default=None),
        "toe":            _first(sec.get("TOE"), sec.get("TOE_OUT"),
                                 sec.get("STATIC_TOE"), default=None),
        "caster":         _m(sec.get("CASTER")),
        "kpi":            _m(sec.get("KINGPIN")),
        "type":           sec.get("TYPE") or "0",
        "tyre_offset":    _m(sec.get("TYRE_STATIC_OFFSET")),
    }
    return out


# ─── brakes.ini additions ─────────────────────────────────────────────────────

def parse_brakes(ini: dict) -> dict:
    f = ini.get("FRONT", {})
    r = ini.get("REAR", ini.get("BRAKES", {}))
    b = ini.get("BRAKES", {})
    hb = ini.get("HANDBRAKE", {})
    return {
        "front":  {k: _m(v) for k, v in f.items() if _m(v) is not None},
        "rear":   {k: _m(v) for k, v in r.items() if _m(v) is not None},
        "bias":           _f(b.get("BIAS") or f.get("BIAS"), 0.6),
        "cockpit_adj":    _b(b.get("COCKPIT_ADJUSTABLE")),
        "bias_step":      _m(b.get("BIAS_STEP")),
        "bias_min":       _m(b.get("BIAS_MIN")),
        "bias_max":       _m(b.get("BIAS_MAX")),
        "handbrake": {
            "max_torque": _m(hb.get("MAX_TORQUE")),
            "ratio":      _m(hb.get("RATIO")),
            "curve":      hb.get("CURVE"),
        } if hb else None,
    }


# ─── aero.ini / wing.ini additions ────────────────────────────────────────────

def parse_aero(ini_aero: dict, ini_wing: Optional[dict], data_dir: Optional[Path]) -> dict:
    body = ini_aero.get("BODY", ini_aero.get("AERO", {}))
    drs  = ini_aero.get("DRS", {})
    out = {
        "frontal_area": _f(body.get("FRONTAL_AREA") or body.get("AREA"), 1.8),
        "cd":           _f(body.get("CD") or body.get("CD0"), 0.32),
        "cl_front":     _f(body.get("FRONT") or body.get("CL_FRONT"), 0.0),
        "cl_rear":      _f(body.get("REAR") or body.get("CL_REAR"), 0.0),
        "wind_mult":    _m(body.get("WIND_MULT")),
        "damage_k":     _m(body.get("DAMAGE_K")),
        "wings":        [],
        "drs":          None,
    }
    # Wings from aero.ini
    for i in range(16):
        sec = ini_aero.get(f"WING_{i}")
        if not sec:
            continue
        w = {
            "id":      f"wing_{i}",
            "name":    sec.get("NAME"),
            "position_ac": _parse_vec3(sec.get("POSITION")),
            "chord":   _m(sec.get("CHORD")),
            "span":    _m(sec.get("SPAN")),
            "angle":   _m(sec.get("ANGLE")),
            "cl_gain": _m(sec.get("CL_GAIN")),
            "cd_gain": _m(sec.get("CD_GAIN")),
        }
        lut_cl = sec.get("LUT_AOA_CL")
        lut_cd = sec.get("LUT_AOA_CD")
        if data_dir and lut_cl:
            w["aoa_cl_curve"] = _resolve_lut(data_dir, lut_cl)
        if data_dir and lut_cd:
            w["aoa_cd_curve"] = _resolve_lut(data_dir, lut_cd)
        w = {k: v for k, v in w.items() if v is not None}
        out["wings"].append(w)

    # Separate wing.ini (CSP active aero)
    if ini_wing:
        out["active_aero"] = {k.lower(): dict(v) for k, v in ini_wing.items()}

    if drs:
        out["drs"] = {
            "enabled": _b(drs.get("ENABLED")),
            "wing_index": _m(drs.get("WING_INDEX")),
            "zone_speed_threshold": _m(drs.get("ACTIVE_SPEED_THRESHOLD")),
        }
        out["drs"] = {k: v for k, v in out["drs"].items() if v is not None}
    return out


# ─── tyres.ini additions (multi-compound, thermal, relaxation) ───────────────

COMPOUND_SUFFIXES = ["", "_1", "_2", "_3", "_4", "_5", "_6", "_7"]


def detect_tyre_compounds(ini: dict) -> list[tuple[str, str]]:
    """
    AC supports multiple compounds as [FRONT_0]/[REAR_0], [FRONT_1]/[REAR_1]…
    Returns list of (front_section_name, rear_section_name) pairs actually
    present.  If only the plain [FRONT]/[REAR] exist, returns one pair.
    """
    pairs: list[tuple[str, str]] = []
    for i in range(8):
        fn = f"FRONT_{i}"
        rn = f"REAR_{i}"
        if fn in ini or rn in ini:
            pairs.append((fn if fn in ini else "FRONT",
                          rn if rn in ini else "REAR"))
    if not pairs and ("FRONT" in ini or "REAR" in ini):
        pairs.append(("FRONT", "REAR"))
    return pairs


def parse_tyre_extras(sec: dict) -> dict:
    """Capture tyres.ini fields beyond the Pacejka fit."""
    out = {
        "relaxation_length_m": _m(sec.get("RELAXATION_LENGTH")),
        "rolling_resistance": {
            "k0": _m(sec.get("ROLLING_RESISTANCE_0")),
            "k1": _m(sec.get("ROLLING_RESISTANCE_1")),
            "k2": _m(sec.get("ROLLING_RESISTANCE_SLIP") or sec.get("ROLLING_RESISTANCE_2")),
        },
        "speed_sensitivity": _m(sec.get("SPEED_SENSITIVITY")),
        "dcamber_0":         _m(sec.get("DCAMBER_0")),
        "dcamber_1":         _m(sec.get("DCAMBER_1")),
        "friction_limit_angle": _m(sec.get("FRICTION_LIMIT_ANGLE")),
        "pressure_model": {
            "static_psi":        _m(sec.get("PRESSURE_STATIC")),
            "ideal_psi":         _m(sec.get("PRESSURE_IDEAL")),
            "d_gain":            _m(sec.get("PRESSURE_D_GAIN")),
            "flex_gain":         _m(sec.get("PRESSURE_FLEX_GAIN")),
        },
    }
    # clean nested Nones
    out["rolling_resistance"] = {k: v for k, v in out["rolling_resistance"].items() if v is not None}
    if not out["rolling_resistance"]:
        out["rolling_resistance"] = None
    out["pressure_model"] = {k: v for k, v in out["pressure_model"].items() if v is not None}
    if not out["pressure_model"]:
        out["pressure_model"] = None
    return {k: v for k, v in out.items() if v is not None}


def parse_tyre_thermal(ini: dict, compound_suffix: str) -> Optional[dict]:
    """THERMAL_FRONT_N / THERMAL_REAR_N blocks."""
    out: dict = {}
    for side in ("FRONT", "REAR"):
        name = f"THERMAL_{side}{compound_suffix}"
        sec = ini.get(name) or ini.get(f"THERMAL_{side}")
        if not sec:
            continue
        out[side.lower()] = {
            "performance_curve": sec.get("PERFORMANCE_CURVE"),
            "core_transfer":     _m(sec.get("CORE_TRANSFER")),
            "surface_transfer":  _m(sec.get("SURFACE_TRANSFER")),
            "patch_transfer":    _m(sec.get("PATCH_TRANSFER")),
            "friction_k":        _m(sec.get("FRICTION_K")),
            "rolling_k":         _m(sec.get("ROLLING_K")),
            "ideal_temp_c":      _m(sec.get("IDEAL_TEMP")),
        }
        out[side.lower()] = {k: v for k, v in out[side.lower()].items() if v is not None}
    return out or None


# ─── car.ini additions (controls, info, pit, fuel tank) ───────────────────────

def parse_car_extras(ini: dict) -> dict:
    info = ini.get("INFO", {})
    ctrl = ini.get("CONTROLS", {})
    ft = ini.get("FUELTANK", {})
    rules = ini.get("RULES", {})
    pit = ini.get("PIT_STOP", {})
    return {
        "info":  {k.lower(): v for k, v in info.items()} if info else None,
        "controls": {
            "ffb_mult":     _m(ctrl.get("FFMULT")),
            "steer_lock":   _m(ctrl.get("STEER_LOCK")),
            "steer_ratio":  _m(ctrl.get("STEER_RATIO")),
            "steer_assist": _m(ctrl.get("STEER_ASSIST")),
        } if ctrl else None,
        "fuel_tank_position_ac": _parse_vec3(ft.get("POSITION")) if ft else None,
        "rules": {k.lower(): _m(v) or v for k, v in rules.items()} if rules else None,
        "pit_stop": {
            "tyre_change_time": _m(pit.get("TYRE_CHANGE_TIME_SEC")),
            "fuel_rate":        _m(pit.get("FUEL_LITER_SEC")),
            "engine_fix_rate":  _m(pit.get("ENGINE_REPAIR_RATE")),
            "body_fix_rate":    _m(pit.get("BODY_REPAIR_RATE")),
            "suspension_fix":   _m(pit.get("SUSPENSION_REPAIR_TIME")),
        } if pit else None,
    }


# ─── ctrl_*.ini electronics ───────────────────────────────────────────────────

def parse_controller_ini(text: str) -> dict:
    """
    AC controller files have a tabular control-curve syntax:

    [HEADER]
    VERSION=1
    [CONTROLLER_0]
    INPUT=RPM
    COMBINATOR=ADD
    LUT=(|500=0.00|4000=1.00|7000=0.50|)
    FILTER=0.95
    UP_LIMIT=1
    DOWN_LIMIT=0

    Returns a list of such controllers for use as an opaque block.
    """
    parsed = parse_ini(text)
    hdr = parsed.get("HEADER", {})
    out = {"version": hdr.get("VERSION"), "controllers": []}
    for i in range(16):
        sec = parsed.get(f"CONTROLLER_{i}")
        if not sec:
            continue
        c = {
            "id":         f"ctrl_{i}",
            "input":      sec.get("INPUT"),
            "combinator": sec.get("COMBINATOR"),
            "filter":     _m(sec.get("FILTER")),
            "up_limit":   _m(sec.get("UP_LIMIT")),
            "down_limit": _m(sec.get("DOWN_LIMIT")),
            "lut_raw":    sec.get("LUT"),
        }
        # Parse inline LUT of form (|x=y|x=y|)
        raw = (sec.get("LUT") or "")
        if raw and raw.strip().startswith("("):
            pairs = []
            for token in raw.strip("()").split("|"):
                if "=" in token:
                    xs, ys = token.split("=")
                    try:
                        pairs.append([float(xs), float(ys)])
                    except ValueError:
                        pass
            if pairs:
                c["lut"] = pairs
        out["controllers"].append({k: v for k, v in c.items() if v is not None})
    return out


def parse_setup_ranges(text: str) -> dict:
    """
    AC setup.ini defines adjustability RANGES, not current values. Each section
    has MIN, MAX, STEP, DEFAULT.  We keep the raw mapping for transparency.
    """
    parsed = parse_ini(text)
    return {sec_name.lower(): dict(sec) for sec_name, sec in parsed.items()
            if any(k in sec for k in ("MIN", "MAX", "STEP", "DEFAULT"))}
