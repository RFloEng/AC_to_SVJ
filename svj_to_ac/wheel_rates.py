"""Wheel-level rates from the way the standard (and other simulators) state springs, dampers and bars.

Assetto Corsa wants everything at the wheel: SPRING_RATE in N/m of wheel travel, damper slopes
in N*s/m of wheel speed, ARB in N/m.  An SVJ file may give them in any of these forms
(spec 9.3, 24.5, 25.3):

* a **wheel rate** (``reference: "wheel"``, or the legacy ``suspension.spring.rate``): used as it is;
* an **element rate** (``reference: "element"``, the default of a unified file): the coil's own N/m,
  brought to the wheel by ``rate_wheel = rate_element * MR**2`` with MR the element's length change per
  unit of vertical wheel travel.  MR is the file's own ``motion_ratio`` when it has one, otherwise it is
  measured on the mechanism with the standard's loop-closure solver (the spec's own estimate, section 25.8),
  otherwise a strut's vertical share, otherwise 1 (and the result says so);
* a **torsional** element (a torsion bar, ``kind: "torsional"``, N*m/rad): MR is the vertical speed of the
  wheel centre per unit of rotation, ``(axis x lever) . up``;
* a **force-length curve** (``rate_curve``, non-linear): the local slope at the reference pose, as AC has
  one linear rate per axle (plus a progression factor it does not get from us);
* several springs in **parallel** on one wheel (a coil and a torsion bar, a main and an assist spring):
  their wheel rates add; a helper spring that is slack at the reference pose adds nothing.

Dampers: ``bump_curve`` / ``rebound_curve`` (force against speed) or the scalar ``damping`` (N*s/m of
the element, so ``c_wheel = damping * MR**2``).  Anti-roll bars: ``wheel_rate_equiv`` when the file has
it, else ``bar_rate`` (N*m/rad) over a lever arm squared; an Assetto Corsa file stores the AC value
itself in ``bar_rate`` and says so in its ``_note``.

Every function returns the number and a list of notes; nothing here raises on odd data.
"""
from __future__ import annotations

import math
from typing import Optional, Sequence

MR_MIN, MR_MAX = 0.08, 3.0          # outside this a measured ratio is a solver or data artefact
MAX_TORSIONAL_BAR_RATE = 15000.0    # N*m/rad: above it a bar_rate is not a torsional rate
DEFAULT_ARB_LEVER = 0.25            # m: used when a file gives a bar's torsional rate and no lever
_STOP_TYPES = ("rubber", "bump", "bumpstop", "bump_stop", "rebound", "packer", "helper")
_AUX_PREFIX = ("bump_stop", "rebound_stop", "stop_spring", "helper", "packer")


# --------------------------------------------------------------------------- #
# Curves                                                                      #
# --------------------------------------------------------------------------- #


def _pts(curve) -> list[tuple[float, float]]:
    out = []
    for p in curve or []:
        if isinstance(p, (list, tuple)) and len(p) >= 2:
            try:
                out.append((float(p[0]), float(p[1])))
            except (TypeError, ValueError):
                pass
    return sorted(out)


def curve_slope(curve, x: float) -> Optional[float]:
    """dF/dx of a piecewise-linear curve at x (central, across the two segments around x)."""
    pts = _pts(curve)
    if len(pts) < 2:
        return None
    x = min(max(x, pts[0][0]), pts[-1][0])
    lo = max((i for i, p in enumerate(pts) if p[0] <= x), default=0)
    a, b = max(lo - 1, 0), min(lo + 1, len(pts) - 1)
    if pts[b][0] - pts[a][0] < 1e-12:
        return None
    return (pts[b][1] - pts[a][1]) / (pts[b][0] - pts[a][0])


def curve_value(curve, x: float) -> Optional[float]:
    pts = _pts(curve)
    if not pts or not pts[0][0] <= x <= pts[-1][0]:
        return None
    for (x0, f0), (x1, f1) in zip(pts, pts[1:]):
        if x0 <= x <= x1:
            return f0 if x1 - x0 < 1e-12 else f0 + (f1 - f0) * (x - x0) / (x1 - x0)
    return pts[-1][1]


def is_auxiliary(e: dict) -> bool:
    """A bump / rebound stop, helper spring or packer: not the spring that carries the car."""
    if str(e.get("id", "")).lower().startswith(_AUX_PREFIX):
        return True
    return str(e.get("spring_type", "")).lower() in _STOP_TYPES


# --------------------------------------------------------------------------- #
# Motion ratio                                                                #
# --------------------------------------------------------------------------- #


def geometric_motion_ratio(data: dict, station_id: str, element_id: str,
                           delta: float = 0.005) -> Optional[float]:
    """|d(element length) / d(wheel travel)| by solving the station at +-delta of bump travel with the
    standard's loop-closure solver; None when the mechanism does not close or the value is absurd."""
    try:
        from svj_migrate import loop_solver as ls
        res = ls.sweep(data, station_id, [-delta, delta])
    except Exception:                                   # SolveError, missing joints, odd files
        return None
    if len(res) != 2 or not all(r.get("converged") for r in res):
        return None
    lens = [r.get("element_lengths", {}).get(element_id) for r in res]
    if None in lens:
        return None
    mr = abs(lens[1] - lens[0]) / (2 * delta)
    return mr if MR_MIN <= mr <= MR_MAX else None


def torsional_motion_ratio(axis_point: Sequence[float], axis_dir: Sequence[float],
                           wheel_centre: Sequence[float], up: Sequence[float]) -> Optional[float]:
    """Vertical speed of the wheel centre per radian about the axis: |(axis x lever) . up|."""
    n = math.sqrt(sum(c * c for c in axis_dir))
    if n < 1e-9:
        return None
    a = [c / n for c in axis_dir]
    lever = [wheel_centre[i] - axis_point[i] for i in range(3)]
    cx = [a[1] * lever[2] - a[2] * lever[1], a[2] * lever[0] - a[0] * lever[2], a[0] * lever[1] - a[1] * lever[0]]
    mr = abs(sum(cx[i] * up[i] for i in range(3)))
    return mr if mr > 0.02 else None


# --------------------------------------------------------------------------- #
# Springs                                                                     #
# --------------------------------------------------------------------------- #


def spring_wheel_rate(e: dict, *, mr: Optional[float], length: Optional[float] = None,
                      travel_ref: float = 0.0) -> tuple[Optional[float], list[str]]:
    """Wheel rate (N/m) of one spring element of a unified file, and notes.

    ``mr`` is the motion ratio already resolved for the element (None: unknown, taken as 1 and flagged).
    ``length`` is the element's length at the reference pose (for the slope of a force-length curve)."""
    notes: list[str] = []
    ident = e.get("id", "spring")
    wheel_ref = e.get("reference") == "wheel"
    k: Optional[float] = None
    if e.get("rate_curve"):
        if wheel_ref:
            # quoted at the wheel as [travel, force]: the secant over the middle of the curve
            pts = _pts(e["rate_curve"])
            mid = pts[len(pts) // 2][0] if pts else 0.0
            s = curve_slope(e["rate_curve"], mid)
        else:
            s = curve_slope(e["rate_curve"], length if length is not None else 0.0)
        if s is not None:
            k = abs(s)
            notes.append(f"{ident}: non-linear spring, the slope at the reference pose is used as the linear rate")
    elif isinstance(e.get("rate"), (int, float)) and e["rate"] > 0:
        k = float(e["rate"])
    if k is None:
        return None, notes
    if wheel_ref:
        return k, notes
    if mr is None:
        notes.append(f"{ident}: referenced to the element and no motion ratio could be found: its rate is used as the wheel rate")
        return k, notes
    return k * mr * mr, notes


# --------------------------------------------------------------------------- #
# Dampers                                                                     #
# --------------------------------------------------------------------------- #


def damper_wheel_curves(e: dict, mr: Optional[float]) -> tuple[dict, list[str]]:
    """``{"bump_curve": ..., "rebound_curve": ...}`` of a damper at the wheel ([speed, force] pairs), from its
    curves or from the scalar ``damping``; element data is brought to the wheel (v / MR, F * MR)."""
    notes: list[str] = []
    out: dict = {}
    wheel_ref = e.get("reference") == "wheel"
    m = 1.0 if (wheel_ref or mr is None) else mr
    if mr is None and not wheel_ref:
        notes.append(f"{e.get('id', 'damper')}: referenced to the element and no motion ratio could be found: used at the wheel")
    for name in ("bump_curve", "rebound_curve"):
        if e.get(name):
            out[name] = [[v / m, f * m] for v, f in _pts(e[name])]
    if not out and isinstance(e.get("damping"), (int, float)) and e["damping"] > 0:
        c = float(e["damping"]) * (1.0 if wheel_ref else m * m)
        out["bump_curve"] = [[0.0, 0.0], [1.0, c]]
        out["rebound_curve"] = [[0.0, 0.0], [1.0, c]]
        notes.append(f"{e.get('id', 'damper')}: a single damping coefficient is used for bump and rebound")
    return out, notes


# --------------------------------------------------------------------------- #
# Stops                                                                       #
# --------------------------------------------------------------------------- #


def stop_from_curve(curve, length: Optional[float], mr: float) -> Optional[dict]:
    """A stop given as a force-length curve with a slack range around the reference length: the gaps
    to the bump (shorter) and rebound (longer) engagement at the wheel and the rate beyond them."""
    pts = _pts(curve)
    if len(pts) < 3 or length is None:
        return None
    zero = [x for x, f in pts if abs(f) < 1e-9]
    if not zero:
        return None
    lo, hi = min(zero), max(zero)
    if not lo - 1e-9 <= length <= hi + 1e-9:
        return None
    out: dict = {"up": round(max(length - lo, 0.0) / mr, 5), "down": round(max(hi - length, 0.0) / mr, 5)}
    below = [(x, f) for x, f in pts if x < lo and f > 0]
    if below:
        x, f = max(below, key=lambda p: p[1])
        out["rate"] = round(abs(f / (lo - x)) * mr * mr, 1) if lo - x > 1e-9 else None
        if out["rate"] is None:
            del out["rate"]
    return out


# --------------------------------------------------------------------------- #
# Anti-roll bars                                                              #
# --------------------------------------------------------------------------- #


def arb_wheel_rate(bar: dict, ac_origin: bool = False) -> tuple[Optional[float], list[str]]:
    """The value AC's ``[ARB]`` wants (N/m at the wheel) from a bar of an SVJ file."""
    notes: list[str] = []
    if isinstance(bar.get("wheel_rate_equiv"), (int, float)) and bar["wheel_rate_equiv"] > 0:
        return float(bar["wheel_rate_equiv"]), notes
    br = bar.get("bar_rate")
    if not isinstance(br, (int, float)) or br <= 0:
        return None, notes
    if ac_origin or "[arb]" in str(bar.get("_note", "")).lower():
        return float(br), notes                                 # the AC value, written there by AC_to_SVJ (older files)
    lever = bar.get("lever_arm")
    if (not isinstance(lever, (int, float)) or lever <= 0) and br > MAX_TORSIONAL_BAR_RATE:
        # no real bar is stiffer than this in N*m/rad: the number is a rate at the wheel (older exporters)
        notes.append(f"{bar.get('id', 'arb')}: bar_rate {br:g} is too stiff for N*m/rad; taken as N/m at the wheel")
        return float(br), notes
    lever = bar.get("lever_arm")
    if not isinstance(lever, (int, float)) or lever <= 0:
        lever = DEFAULT_ARB_LEVER
        notes.append(f"{bar.get('id', 'arb')}: torsional rate with no lever arm or wheel_rate_equiv; "
                     f"a {DEFAULT_ARB_LEVER} m lever was assumed")
    mr = bar.get("motion_ratio")
    mr = float(mr) if isinstance(mr, (int, float)) and mr > 0 else 1.0
    return float(br) / (lever * lever) * mr * mr, notes


def legacy_spring_wheel_rate(sp: dict) -> tuple[Optional[float], list[str]]:
    """Wheel rate of a v0.95-0.99 ``suspension.<corner>.spring`` block (spec 9.3): ``rate`` is the wheel
    rate; a ``rate_curve`` ([displacement, force], compression positive) takes precedence; a torsion bar or
    rubber-torsion spring with ``torsional_rate`` and ``arm_length`` is that rate over the arm squared;
    an air spring's ``effective_area`` and ``nominal_pressure`` alone do not give a rate."""
    notes: list[str] = []
    if sp.get("rate_curve"):
        pts = _pts(sp["rate_curve"])
        if len(pts) >= 2:
            mid = pts[len(pts) // 2][0]
            s = curve_slope(sp["rate_curve"], mid)
            if s:
                notes.append("non-linear spring: the slope of the middle of the curve is used as the linear rate")
                return abs(s), notes
    if isinstance(sp.get("rate"), (int, float)) and sp["rate"] > 0:
        return float(sp["rate"]), notes
    tr, arm = sp.get("torsional_rate"), sp.get("arm_length")
    if isinstance(tr, (int, float)) and tr > 0 and isinstance(arm, (int, float)) and arm > 0:
        notes.append("torsional spring: wheel rate = torsional_rate / arm_length^2")
        return float(tr) / (arm * arm), notes
    if sp.get("effective_area") or sp.get("hydropneumatic"):
        notes.append("gas spring with no linear rate: AC's default spring rate is kept")
    return None, notes
