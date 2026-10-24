"""
Side-view suspension geometry that the SVJ standard's solver does not give: the side-view
instant centre and the anti-dive / anti-squat it implies.

(Caster, kingpin inclination, scrub radius, caster trail and roll centre are not derived
here: the standard's own loop-closure solver, tools/loop_solver.py, computes them from the
mechanism, so a file carries no second, slightly different set.)

Works in the SVJ vehicle frame of `converter.build_corner` (X forward, Y right, Z down,
ground at z = 0) from the hardpoints of one corner at the static position.  Estimates: the
usual textbook simplifications (brakes outboard; a wishbone's side-view line is the axis
through its two inboard pivots).
"""
from __future__ import annotations

import math
from typing import Optional, Sequence

_EPS = 1e-9
_MAX_ANTI = 200.0      # beyond this (%) the instant centre is too close to the wheel to mean anything


def _intersect(p1, d1, p2, d2) -> Optional[tuple]:
    """Intersection of two 2D lines (point + direction); None when parallel."""
    cross = d1[0] * d2[1] - d1[1] * d2[0]
    n = math.hypot(*d1) * math.hypot(*d2)
    if n < _EPS or abs(cross) < 1e-6 * n:
        return None
    t = ((p2[0] - p1[0]) * d2[1] - (p2[1] - p1[1]) * d2[0]) / cross
    return (p1[0] + t * d1[0], p1[1] + t * d1[1])


def _xz(p):
    return (p[0], p[2])


def _arm_axis(arm: dict) -> Optional[tuple]:
    """Side-view line through the two inboard pivots of an arm."""
    inb = arm.get("inboard_points") or []
    if len(inb) < 2:
        return None
    a, b = _xz(inb[0]), _xz(inb[1])
    if math.hypot(b[0] - a[0], b[1] - a[1]) < _EPS:
        return None
    return a, (b[0] - a[0], b[1] - a[1])


def _strut_perpendicular(hp: dict) -> Optional[tuple]:
    """MacPherson upper constraint: the line through the strut top, perpendicular to the
    strut axis."""
    top, low = hp.get("strut_top"), hp.get("strut_lower")
    if not top or not low:
        return None
    a, b = _xz(top), _xz(low)
    d = (b[0] - a[0], b[1] - a[1])
    if math.hypot(*d) < _EPS:
        return None
    return a, (-d[1], d[0])


def side_view_ic(topo: dict) -> Optional[tuple]:
    hp = topo["upright"]["hardpoints"]
    arms = {L["name"]: L for L in topo.get("links", []) if L.get("type") == "arm"}
    lower = next((arms[n] for n in ("lower_wishbone", "lower_control_arm") if n in arms), None)
    upper = next((arms[n] for n in ("upper_wishbone", "upper_control_arm") if n in arms), None)
    lo = _arm_axis(lower) if lower else None
    if upper:
        up = _arm_axis(upper)
    elif topo.get("system_type") == "macpherson":
        up = _strut_perpendicular(hp)
    else:
        up = None
    if not up or not lo:
        return None
    return _intersect(*up, *lo)


def anti_geometry(ic: Optional[Sequence[float]], wheel_x: float, wheelbase: float,
                  cg_height: float, is_front: bool, front_brake: float) -> Optional[dict]:
    """Anti-dive (front, braking) or anti-squat (rear, driving), in percent.

    Front: tan(angle of the contact-patch -> IC line) x wheelbase / CG height x front
    brake share; 100 % means the line reaches the CG height at the CG.  Rear: the same
    with the drive force going through the contact patch (all of it on this axle).
    """
    if ic is None or cg_height <= 0 or wheelbase <= 0:
        return None
    up = -ic[1]                                     # IC height above the ground
    run = (wheel_x - ic[0]) if is_front else (ic[0] - wheel_x)   # IC behind (front) / ahead (rear)
    if abs(run) < 1e-3:
        return None
    pct = (up / run) * (wheelbase / cg_height) * 100.0
    if abs(pct) > _MAX_ANTI:
        return None
    if is_front:
        return {"anti_dive_percent": pct * front_brake}
    return {"anti_squat_percent": pct}


def corner_geometry(topo: dict, wheel_center: Sequence[float], is_front: bool,
                    wheelbase: float, cg_height: float, front_brake: float) -> dict:
    """The side-view numbers of one corner.  Keys that cannot be computed are left out."""
    out: dict = {}
    if topo.get("system_type") not in ("double_wishbone", "macpherson", "multi_link"):
        return out
    ic = side_view_ic(topo)
    if ic:
        out["side_view_ic"] = list(ic)
        a = anti_geometry(ic, wheel_center[0], wheelbase, cg_height, is_front, front_brake)
        if a:
            out.update(a)
    return out
