"""Corner-shaped view of a unified suspension (SVJ 0.100+, spec 25).

The suspension of an SVJ 0.100 file is bodies (``chassis.mass_bodies``), ``joint``
constraints and ``spring`` / ``damper`` elements plus ``wheel_stations[]``; the
``suspension`` section of v0.95 .. v0.99 (one block per corner with ``topology``,
``spring``, ``damper``, ``arb`` ...) is deprecated.  The AC writers work from the
corner blocks, so this module derives them from the bodies and joints:

* the **upright** is the station's ``upright_body``; a joint between it and another body
  is one of its connections (the wheel's hinge is not);
* a **link** is a body joined to the upright (its outboard point is that joint's frame)
  and to the chassis or the steering rack (its inboard points are those joints' frames);
  two inboard points make an ``arm`` (front point first), one a ``rod``;
* a body joined to the upright by a ``translational`` joint is the **strut**: its inboard
  point is the joint to the chassis, its outboard point the slide;
* the corner's ``spring`` / ``damper`` come from the elements on the upright, brought to
  the wheel with the motion ratio when they are referenced to the element;
* ``arb`` from ``anti_roll_bars[]``; ``alignment``, ``wheel``, ``tire``, ``brake`` and
  ``static_setup`` are the station's own.

Everything is in the vehicle frame at the reference pose (spec 24.1).  Only what the AC
writers read is built; a station that cannot be read is left out and reported in
``notes``.
"""
from __future__ import annotations

import copy
import re
from typing import Any

import wheel_rates as wr

CORNERS = ("FL", "FR", "RL", "RR")
_SIDE = re.compile(r"_(fl|fr|rl|rr|a\d+[lrc]|[lr])(_\w+)?$")


def is_unified(data: dict) -> bool:
    return bool(data.get("wheel_stations")) and "suspension" not in data


def _strip_side(body_id: str) -> str:
    base = _SIDE.sub("", body_id.lower())
    return base or body_id.lower()


def _corner_key(station: dict, stations: list) -> str | None:
    sid = str(station.get("id", "")).upper()
    if sid in CORNERS:
        return sid
    side = {"L": "L", "R": "R"}.get(str(station.get("side", "")).upper())
    axle = str(station.get("axle", "")).upper()
    if side and axle:
        axles = sorted({str(s.get("axle", "")).upper() for s in stations if s.get("axle")},
                       key=lambda a: (len(a), a))
        if axle in axles[:2]:
            return ("F" if axles.index(axle) == 0 else "R") + side
    return None


ARM_HALF = 0.15      # m: half the pickup spacing of an arm that the file gives as one hinge


def _z_axis(q) -> list:
    """Unit z axis of a quaternion (x, y, z, w); the joint axis of a frame."""
    if not (isinstance(q, (list, tuple)) and len(q) == 4):
        return [0.0, 0.0, 1.0]
    x, y, z, w = (float(v) for v in q)
    return [2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y)]


class _Doc:
    def __init__(self, data: dict):
        chassis = data.get("chassis") or {}
        self.bodies = {b["id"]: b for b in chassis.get("mass_bodies", []) if "id" in b}
        self.chassis_markers = chassis.get("markers") or {}
        self.joints = [c for c in data.get("constraints", []) if c.get("type") == "joint"]
        self.elements = [c for c in data.get("constraints", [])
                         if c.get("type") in ("spring", "damper")]
        self.root_names = {"chassis", "ground"}
        self.data = data
        self.stations = data.get("wheel_stations") or []
        self.other_uprights = {x for st in self.stations for x in (st.get("upright_body"), st.get("wheel_body"))}

    def marker(self, body: str, name: str):
        mk = (self.chassis_markers if body == "chassis"
              else (self.bodies.get(body, {}).get("markers") or {})).get(name)
        return None if mk is None else _pos(mk)

    def frame(self, c: dict, side: str):
        """Position of side "a" / "b" of a constraint: an inline frame, else a marker."""
        fr = c.get("frame_" + side)
        if fr is not None:
            return _pos(fr)
        mk = c.get("marker_" + side) or c.get("marker")
        body = c.get("body_" + side)
        return self.marker(body, mk) if mk else None


def _pos(obj) -> list | None:
    if isinstance(obj, dict):
        obj = obj.get("position")
    if isinstance(obj, (list, tuple)) and len(obj) == 3:
        return [float(v) for v in obj]
    return None


def _joint_between(doc: _Doc, a: str, b: str):
    return [j for j in doc.joints
            if {j.get("body_a"), j.get("body_b")} == {a, b}]


def _end_on(doc: _Doc, j: dict, body: str):
    """Position of joint `j` on the side of `body`."""
    return doc.frame(j, "a" if j.get("body_a") == body else "b")


def _station_corner(doc: _Doc, st: dict, notes: list) -> dict | None:
    up = st.get("upright_body")
    if up not in doc.bodies:
        notes.append(f"station {st.get('id')}: upright body {up!r} not found")
        return None
    wheel_body = st.get("wheel_body")
    wc = (doc.marker(up, st.get("wheel_center_marker") or "wheel_center")
          or (_pos(doc.bodies[wheel_body]) if wheel_body in doc.bodies else None)
          or _pos(doc.bodies[up]))
    hardpoints: dict = {"wheel_center": wc}
    links: list = []

    for j in [j for j in doc.joints if up in (j.get("body_a"), j.get("body_b"))]:
        other = j["body_b"] if j.get("body_a") == up else j["body_a"]
        if other == wheel_body or other in doc.root_names:   # the wheel's hinge; a direct hold
            continue
        here = _end_on(doc, j, up)
        name = _strip_side(other)
        link: dict = {"name": name, "outboard_point": here, "_body": other}
        inb = []
        for k in doc.joints:
            if k is j or other not in (k.get("body_a"), k.get("body_b")):
                continue
            far = k["body_b"] if k.get("body_a") == other else k["body_a"]
            if far == up:
                continue
            p = _end_on(doc, k, other)
            if not p:
                continue
            if k.get("joint_type") == "revolute" and far in doc.root_names:
                # an arm hinged about one axis (a single revolute joint): its pickups are two
                # points of that axis, ARM_HALF either side of the joint frame
                fr = k.get("frame_b" if k.get("body_a") == other else "frame_a") or {}
                ax = _z_axis(fr.get("orientation"))
                inb.extend([[p[i] + ax[i] * ARM_HALF for i in range(3)],
                            [p[i] - ax[i] * ARM_HALF for i in range(3)]])
                link["_est"] = True
            else:
                inb.append(p)
        inb.sort(key=lambda p: -p[0])                     # front point first
        link["inboard_points"] = inb
        if j.get("joint_type") == "translational":
            link["type"] = "strut"
            if "strut" not in name:
                link["name"] = "strut"
        else:
            link["type"] = "arm" if len(inb) >= 2 else "rod"
        links.append(link)

        low = link["name"]
        if "wishbone" in low or "control_arm" in low or "arm" in low or "link" in low:
            if "lower" in low and here:
                hardpoints["lower_ball_joint"] = here
            elif "upper" in low and here:
                hardpoints["upper_ball_joint"] = here
        if ("tie_rod" in low or "steering" in low) and here:
            hardpoints["steering_tie_rod_end"] = here
        if link["type"] == "strut" and here:
            hardpoints["strut_outboard"] = here

    corner: dict = {"position": wc,
                    "topology": {"system_type": st.get("topology_hint") or "double_wishbone",
                                 "upright": {"id": up, "hardpoints": hardpoints},
                                 "links": [{k: v for k, v in l.items() if k != "_body"}
                                           for l in links]}}
    for key in ("wheel", "tire", "brake", "alignment", "static_setup", "compliance_summary"):
        if st.get(key) is not None:
            corner[key] = copy.deepcopy(st[key])
    return corner


def _reach(doc: _Doc, up: str, wheel_body, depth: int = 3) -> set:
    """Bodies that move with one wheel's suspension: joined to the upright, to those, and so on, up to
    ``depth`` joints; the chassis, the wheel and any other station's upright or wheel are borders."""
    seen, frontier = {up}, {up}
    stop = doc.root_names | {wheel_body} | (doc.other_uprights - {up})
    for _ in range(depth):
        nxt = set()
        for j in doc.joints:
            a, b = j.get("body_a"), j.get("body_b")
            for x, y in ((a, b), (b, a)):
                if x in frontier and y not in seen and y not in stop and y in doc.bodies:
                    nxt.add(y)
        seen |= nxt
        frontier = nxt
    return seen


def _station_elements(doc: _Doc, st: dict, kind: str, main_only: bool = True) -> list:
    """The spring / damper elements of one wheel: one end on a body that moves with it, the other on the
    chassis or on another such body.  Stops, helper springs and packers are left out of ``main_only``."""
    up = st["upright_body"]
    moving = _reach(doc, up, st.get("wheel_body"))
    out = []
    for e in doc.elements:
        if e.get("type") != kind:
            continue
        a, b = e.get("body_a"), e.get("body_b")
        if not ((a in moving or b in moving) and (a in moving or a in doc.root_names)
                and (b in moving or b in doc.root_names)):
            continue
        if main_only and wr.is_auxiliary(e):
            continue
        out.append(e)
    return out


def _length(e: dict, doc: _Doc | None = None) -> float | None:
    a, b = doc.frame(e, "a"), doc.frame(e, "b")
    if a and b:
        return sum((x - y) ** 2 for x, y in zip(a, b)) ** 0.5
    return None


def _motion_ratio(e: dict, doc: _Doc, st: dict, strut: bool, wc=None) -> tuple[float | None, str]:
    """Motion ratio of one element: ``(value, source)``; the value is None when nothing could be found.
    A ratio the file gives wins, then the loop solver (the spec's estimate, 25.8), then a strut's vertical
    share of its axis."""
    conv = e.get("x_converted_from_wheel_rate") or {}
    if isinstance(conv.get("motion_ratio"), (int, float)) and conv["motion_ratio"] > 0:
        return float(conv["motion_ratio"]), "file"
    if e.get("reference") == "wheel":
        return 1.0, "wheel"
    mr = e.get("motion_ratio")
    if isinstance(mr, (int, float)) and mr > 0:
        return float(mr), "file"
    if e.get("kind") == "torsional":
        a = doc.frame(e, "a")
        axis = _z_axis((e.get("frame_a") or {}).get("orientation"))
        if a and wc:
            from svj_migrate.frame import frame_of
            up = [0.0, 0.0, 1.0 if frame_of(doc.data) == "ISO_8855" else -1.0]
            v = wr.torsional_motion_ratio(a, axis, wc, up)
            if v:
                return v, "torsion"
        return None, "none"
    v = wr.geometric_motion_ratio(doc.data, st.get("id"), e.get("id"))
    if v:
        return v, "solver"
    if strut:
        a, b = doc.frame(e, "a"), doc.frame(e, "b")
        n = _length(e, doc)
        if a and b and n and n > 0.05:
            return max(0.4, min(1.0, abs(b[2] - a[2]) / n)), "strut"
    return None, "none"


def _stops(doc: _Doc, st: dict, strut: bool, notes: list) -> dict:
    """Bump and rebound stops of a corner, as AC states them: {"up": travel at which the bump stop engages,
    "down": the rebound stop's, "rate": N/m}, at the wheel."""
    out: dict = {}
    for e in _station_elements(doc, st, "spring", main_only=False):
        eid = str(e.get("id", "")).lower()
        if not wr.is_auxiliary(e) or eid.startswith(("helper", "packer")):
            continue
        mr, _ = _motion_ratio(e, doc, st, strut)
        mr = mr or 1.0
        length = _length(e, doc)
        kind = "up" if eid.startswith("bump_stop") else "down" if eid.startswith("rebound_stop") else None
        if kind is None:                                # one element that is both stops (a force-length curve)
            got = wr.stop_from_curve(e.get("rate_curve"), length, mr)
            for k, v in (got or {}).items():
                out.setdefault(k, v)
            continue
        free = e.get("free_length")
        if isinstance(free, (int, float)) and length is not None:
            gap = (length - free) if kind == "up" else (free - length)
            out[kind] = round(max(gap, 0.0) / mr, 5)
        pts = [p for p in (e.get("rate_curve") or []) if abs(p[0]) > 1e-9 and p[1]]
        if pts and "rate" not in out:
            p = max(pts, key=lambda q: abs(q[1]))
            out["rate"] = round(abs(p[1] / p[0]) * mr * mr, 1)
    return out


def normalize(data: dict) -> tuple[dict, list]:
    """Return ``(data, notes)``: a copy of a unified file with the ``suspension`` section
    derived; any other file is returned as it is."""
    if not is_unified(data):
        return data, []
    notes: list = []
    doc = _Doc(data)
    stations = data["wheel_stations"]
    susp: dict = {}
    arb_by_station: dict = {}
    for bar in data.get("anti_roll_bars", []):
        for s in bar.get("stations", []):
            arb_by_station[s] = bar
    for st in stations:
        key = _corner_key(st, stations)
        if key is None or key in susp:
            continue
        corner = _station_corner(doc, st, notes)
        if corner is None:
            continue
        up = st["upright_body"]
        strut = str(st.get("topology_hint") or "") == "macpherson"
        wc = ((corner.get("topology") or {}).get("upright") or {}).get("hardpoints", {}).get("wheel_center")
        stops = _stops(doc, st, strut, notes)
        total, used = 0.0, []
        for sp in _station_elements(doc, st, "spring"):
            mr, src = _motion_ratio(sp, doc, st, strut, wc)
            if src in ("solver", "strut", "torsion"):
                notes.append(f"{key}: {sp.get('id')} motion ratio {mr:.3f} " + {
                    "solver": "measured on the mechanism with the loop solver",
                    "strut": "estimated from the strut's inclination",
                    "torsion": "from the torsion bar's lever"}[src])
            k, nn = wr.spring_wheel_rate(sp, mr=mr, length=_length(sp, doc))
            notes.extend(f"{key}: {n}" for n in nn)
            if k:
                total += k
                used.append(sp)
        if used:
            corner["spring"] = {"rate": round(total, 3),
                                **({"rod_length": used[0]["rod_length"]} if "rod_length" in used[0] else {})}
            if len(used) > 1:
                notes.append(f"{key}: {len(used)} springs in parallel on the wheel: their wheel rates are added")
        dampers = _station_elements(doc, st, "damper")
        if dampers:
            mr, _ = _motion_ratio(dampers[0], doc, st, strut, wc)
            curves, nn = wr.damper_wheel_curves(dampers[0], mr)
            notes.extend(f"{key}: {n}" for n in nn)
            d: dict = dict(curves)
            if stops:
                d["bumpstops"] = stops
            corner["damper"] = d
        elif stops:
            corner["damper"] = {"bumpstops": stops}
        bar = arb_by_station.get(st.get("id"))
        if bar:
            rate, nn = wr.arb_wheel_rate(bar, ac_origin="x_assettocorsa" in data)
            notes.extend(nn)
            if rate:
                corner["arb"] = {"bar_rate": round(rate, 3)}
        susp[key] = corner
    out = dict(data)
    out["suspension"] = susp
    return out, notes
