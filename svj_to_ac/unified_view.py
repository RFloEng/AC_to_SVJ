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


class _Doc:
    def __init__(self, data: dict):
        chassis = data.get("chassis") or {}
        self.bodies = {b["id"]: b for b in chassis.get("mass_bodies", []) if "id" in b}
        self.chassis_markers = chassis.get("markers") or {}
        self.joints = [c for c in data.get("constraints", []) if c.get("type") == "joint"]
        self.elements = [c for c in data.get("constraints", [])
                         if c.get("type") in ("spring", "damper")]
        self.root_names = {"chassis", "ground"}

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
            if p:
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


def _wheel_side(doc: _Doc, up: str, kind: str):
    """The spring / damper element on an upright."""
    for e in doc.elements:
        if e.get("type") == kind and up in (e.get("body_a"), e.get("body_b")):
            return e
    return None


def _motion_ratio(e: dict) -> tuple:
    """(motion ratio, guessed): element travel / wheel travel.  An element that was converted
    from a value quoted at the wheel carries the ratio it was converted with."""
    conv = e.get("x_converted_from_wheel_rate") or {}
    if isinstance(conv.get("motion_ratio"), (int, float)) and conv["motion_ratio"] > 0:
        return float(conv["motion_ratio"]), False
    if e.get("reference") == "wheel":
        return 1.0, False
    mr = e.get("motion_ratio")
    if isinstance(mr, (int, float)) and mr > 0:
        return float(mr), False
    return 1.0, True


def _stops(doc: _Doc, up: str) -> dict:
    """Bump and rebound stops of a corner (spring elements `bump_stop_*` / `rebound_stop_*`
    on the upright): {"up": gap, "down": gap, "rate": N/m}."""
    out: dict = {}
    for e in doc.elements:
        eid = str(e.get("id", ""))
        kind = "up" if eid.startswith("bump_stop") else "down" if eid.startswith("rebound_stop") else None
        if kind is None or up not in (e.get("body_a"), e.get("body_b")):
            continue
        free = e.get("free_length")
        curve = e.get("rate_curve") or []
        if isinstance(free, (int, float)):
            out[kind] = round(abs(0.30 - free), 5)           # the migration's free length is 0.30 - gap
            if kind == "down":
                out[kind] = round(abs(free - 0.30), 5)
        pts = [p for p in curve if abs(p[0]) > 1e-9]
        if pts and "rate" not in out:
            p = max(pts, key=lambda q: abs(q[1]))
            if p[0] and p[1]:
                out["rate"] = round(abs(p[1] / p[0]), 1)
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
        sp, dp = _wheel_side(doc, up, "spring"), _wheel_side(doc, up, "damper")
        stops = _stops(doc, up)
        if sp:
            mr, guessed = _motion_ratio(sp)
            conv = sp.get("x_converted_from_wheel_rate") or {}
            rate = conv.get("wheel_rate") or (sp.get("rate") or 0.0) * mr * mr
            corner["spring"] = {"rate": rate, **({"rod_length": sp["rod_length"]}
                                                  if "rod_length" in sp else {})}
            if guessed:
                notes.append(f"{key}: spring {sp.get('id')} is referenced to the element and "
                             "has no motion ratio: its rate is used as the wheel rate")
        if dp:
            mr, _ = _motion_ratio(dp)
            d: dict = {}
            for name in ("bump_curve", "rebound_curve"):
                if dp.get(name):                        # element -> wheel: v / MR, F * MR
                    d[name] = [[v / mr, f * mr] for v, f in dp[name]]
            if stops:
                d["bumpstops"] = stops
            corner["damper"] = d
        if stops and "damper" not in corner:
            corner["damper"] = {"bumpstops": stops}
        bar = arb_by_station.get(st.get("id"))
        if bar and bar.get("bar_rate"):
            corner["arb"] = {"bar_rate": bar["bar_rate"]}
        susp[key] = corner
    out = dict(data)
    out["suspension"] = susp
    return out, notes
