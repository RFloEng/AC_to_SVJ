"""
Static suspension geometry derived from a corner's hardpoints.

Everything works in the SVJ vehicle frame used by `converter.build_corner`
(X forward, Y right, Z down, origin on the ground at the front axle, so the
ground is z = 0 and a point above it has a negative z).  The inputs are the
hardpoints of one corner at the static position; nothing is simulated.

Results are estimates: they inherit the accuracy of the hardpoints (AC mods
give them in an axle-local frame) and use the usual textbook simplifications
(an arm is projected as the line from its ball joint to the middle of its two
inboard pivots; brakes are taken as outboard).
"""
from __future__ import annotations

import math
from typing import Optional, Sequence

_EPS = 1e-9
_MAX_RC = 1.5          # a roll centre farther than this (m) from the ground is a degenerate fit


def _intersect(p1, d1, p2, d2) -> Optional[tuple]:
    """Intersection of two 2D lines (point + direction); None when parallel."""
    cross = d1[0] * d2[1] - d1[1] * d2[0]
    n = math.hypot(*d1) * math.hypot(*d2)
    if n < _EPS or abs(cross) < 1e-6 * n:
        return None
    t = ((p2[0] - p1[0]) * d2[1] - (p2[1] - p1[1]) * d2[0]) / cross
    return (p1[0] + t * d1[0], p1[1] + t * d1[1])


def _proj(p, axes):
    return (p[axes[0]], p[axes[1]])


def _line_through(a, b):
    return a, (b[0] - a[0], b[1] - a[1])


def _arm_front(arm: dict, axes) -> Optional[tuple]:
    """Line of a wishbone / control arm in the plane `axes`: ball joint to the
    middle of the inboard pivots (front view)."""
    out = arm.get("outboard_point")
    inb = arm.get("inboard_points") or []
    if not out or not inb:
        return None
    mid = [sum(p[i] for p in inb) / len(inb) for i in range(3)]
    return _line_through(_proj(out, axes), _proj(mid, axes))


def _arm_axis(arm: dict, axes) -> Optional[tuple]:
    """Line through the two inboard pivots, i.e. the axis the arm rotates about
    (side view)."""
    inb = arm.get("inboard_points") or []
    if len(inb) < 2:
        return None
    a, b = _proj(inb[0], axes), _proj(inb[1], axes)
    if math.hypot(b[0] - a[0], b[1] - a[1]) < _EPS:
        return None
    return _line_through(a, b)


def _strut_perpendicular(hp: dict, axes) -> Optional[tuple]:
    """MacPherson upper constraint: the line through the strut top, perpendicular
    to the strut axis."""
    top, low = hp.get("strut_top"), hp.get("strut_lower")
    if not top or not low:
        return None
    a, b = _proj(top, axes), _proj(low, axes)
    d = (b[0] - a[0], b[1] - a[1])
    if math.hypot(*d) < _EPS:
        return None
    return a, (-d[1], d[0])


def _upper_lower(topo: dict, axes, kind: str):
    """(upper_line, lower_line) for the front (`kind` = "front") or side view."""
    hp = topo["upright"]["hardpoints"]
    arms = {L["name"]: L for L in topo.get("links", []) if L.get("type") == "arm"}
    lower = next((arms[n] for n in ("lower_wishbone", "lower_control_arm") if n in arms), None)
    upper = next((arms[n] for n in ("upper_wishbone", "upper_control_arm") if n in arms), None)
    fn = _arm_front if kind == "front" else _arm_axis
    lo = fn(lower, axes) if lower else None
    if upper:
        up = fn(upper, axes)
    elif topo.get("system_type") == "macpherson":
        up = _strut_perpendicular(hp, axes)
    else:
        up = None
    return up, lo


def roll_centre(topo: dict, wheel_y: float) -> Optional[dict]:
    """Roll-centre height and front-view instant centre of one corner.

    Returns {"height", "instant_centre": [y, z] | None} (height in m above the
    ground), or None when the corner has no usable arms.
    """
    up, lo = _upper_lower(topo, (1, 2), "front")
    if not up or not lo:
        return None
    ic = _intersect(*up, *lo)
    contact = (wheel_y, 0.0)
    if ic is None:                                  # parallel arms: IC at infinity
        d = lo[1]
    else:
        d = (ic[0] - contact[0], ic[1] - contact[1])
    if abs(d[0]) < _EPS:                            # line is vertical: never meets the centreline
        return None
    z = d[1] * (0.0 - contact[0]) / d[0]
    height = -z
    if abs(height) > _MAX_RC:
        return None
    return {"height": height,
            "instant_centre": [ic[0], ic[1]] if ic else None}


def side_view_ic(topo: dict) -> Optional[tuple]:
    up, lo = _upper_lower(topo, (0, 2), "side")
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
    if is_front:
        return {"anti_dive_percent": pct * front_brake}
    return {"anti_squat_percent": pct}


def kingpin(topo: dict, wheel_xy: Sequence[float], y_sign: int) -> Optional[dict]:
    """Steering-axis geometry of a front corner (lower ball joint to the upper ball
    joint, or to the strut top on a MacPherson axle).

    caster_angle / kingpin_inclination in rad (positive: axis leans back / in at the
    top); mechanical_trail positive when the axis meets the ground ahead of the contact
    patch; scrub_radius positive when it meets the ground inboard of it.
    """
    hp = topo["upright"]["hardpoints"]
    low = hp.get("lower_ball_joint")
    up = hp.get("upper_ball_joint") if topo.get("system_type") != "macpherson" \
        else hp.get("strut_top")
    if not low or not up:
        return None
    dz = low[2] - up[2]                              # > 0 when the upper point is higher
    if dz < 0.02:
        return None
    t = -low[2] / (up[2] - low[2])                   # ground (z = 0) on the axis line
    gx = low[0] + t * (up[0] - low[0])
    gy = low[1] + t * (up[1] - low[1])
    return {
        "caster_angle": math.atan2(low[0] - up[0], dz),
        "kingpin_inclination": math.atan2(y_sign * (low[1] - up[1]), dz),
        "mechanical_trail": gx - wheel_xy[0],
        "scrub_radius": y_sign * (wheel_xy[1] - gy),
    }


def corner_geometry(topo: dict, wheel_center: Sequence[float], y_sign: int, is_front: bool,
                    wheelbase: float, cg_height: float, front_brake: float) -> dict:
    """All the derived numbers of one corner.  Keys that cannot be computed are left out."""
    out: dict = {}
    if topo.get("system_type") not in ("double_wishbone", "macpherson", "multi_link"):
        return out
    rc = roll_centre(topo, wheel_center[1])
    if rc:
        out["roll_centre_height"] = rc["height"]
        if rc["instant_centre"]:
            out["front_view_ic"] = rc["instant_centre"]
    ic = side_view_ic(topo)
    if ic:
        out["side_view_ic"] = list(ic)
        a = anti_geometry(ic, wheel_center[0], wheelbase, cg_height, is_front, front_brake)
        if a:
            out.update(a)
    if is_front:
        k = kingpin(topo, wheel_center, y_sign)
        if k:
            out["kingpin"] = k
    return out
