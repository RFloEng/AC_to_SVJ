"""
Reduce a CSP COSMIC suspension (``TYPE=COSMIC``) to the links an SVJ corner needs.

COSMIC describes a corner as a small multibody model: rigid bodies, distance joints
(``DJn``), ball joints (``Jn``), hinges (``HJn``), sliders and a steer joint, with
springs / dampers between bodies.  SVJ describes it as links from the chassis to the
upright.  The reduction recognises the patterns the installed cars use:

* **adjuster pairs** - a slider plus a distance joint with the same two bodies and the
  same points is a setup adjuster (camber, caster, toe): the two bodies are one rigid
  body.  A slider with a spring / damper between the same two bodies is a bushing,
  also treated as rigid;
* **rods** - a distance joint (or the steer joint) from the chassis to the hub;
  two rods ending at the same hub point are one A-arm;
* **A-arm** - a body held to the chassis by two ball joints (its pivot axis) and
  joined to the hub by a third;
* **strut** - a body held to the chassis by one ball joint and sliding on the hub.

Anything else is reported in ``notes`` and left out, never guessed.  Positions are the
AC values (relative to the wheel centre: x inboard-positive, y up, z forward), so the
caller converts them like the other suspension hardpoints.
"""
from __future__ import annotations

import math
import re
from typing import Optional

_TOL = 1e-3


def _vec(v) -> Optional[list]:
    if v is None:
        return None
    try:
        p = [float(x) for x in str(v).replace(";", ",").split(",")[:3]]
    except ValueError:
        return None
    return p if len(p) == 3 else None


def _close(a, b) -> bool:
    return a is not None and b is not None and all(abs(x - y) < _TOL for x, y in zip(a, b))


def _sub(a, b):
    return [a[i] - b[i] for i in range(3)]


def _dot(a, b) -> float:
    return sum(a[i] * b[i] for i in range(3))


def _cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def _norm(a) -> float:
    return math.sqrt(_dot(a, a))


class _Joint:
    def __init__(self, kind, name, a, b, pa, pb):
        self.kind, self.name, self.a, self.b, self.pa, self.pb = kind, name, a, b, pa, pb


class _Groups:
    """Union-find over body names."""

    def __init__(self):
        self.p: dict = {}

    def find(self, x):
        self.p.setdefault(x, x)
        while self.p[x] != x:
            self.p[x] = self.p[self.p[x]]
            x = self.p[x]
        return x

    def union(self, a, b):
        self.p[self.find(a)] = self.find(b)


def _joints(sec: dict, left: bool) -> list:
    out = []

    def parity_ok(prefix):
        raw = sec.get(f"{prefix}_PARITY")
        try:
            par = int(float(str(raw).split()[0])) if raw is not None else -1
        except ValueError:
            par = -1
        return par == -1 or (par == 0 and left) or (par == 1 and not left)

    for key, val in sec.items():
        m = re.match(r"^(DJ|HJ|J)(\d+)$|^(SLIDER)_(\d+)$|^(STEER_JOINT)_(\d+)$", key)
        if not m:
            continue
        if m.group(1):
            kind, n = m.group(1), m.group(2)
            prefix = f"{kind}{n}"
        elif m.group(3):
            kind, n = "SLIDER", m.group(4)
            prefix = f"SLIDER_{n}"
        else:
            kind, n = "STEER", m.group(6)
            prefix = f"STEER_JOINT_{n}"
        if not parity_ok(prefix):
            continue
        a = str(sec.get(f"{prefix}_BODY_A", "CHASSIS")).strip()
        b = str(sec.get(f"{prefix}_BODY_B", "HUB_L")).strip()
        if kind == "J":
            pa = pb = _vec(sec.get(f"{prefix}_POS"))
        elif kind == "HJ":
            pa = _vec(sec.get(f"{prefix}_POS_A") or sec.get(f"{prefix}_POS0"))
            pb = _vec(sec.get(f"{prefix}_POS_B") or sec.get(f"{prefix}_POS1"))
        else:
            pa, pb = _vec(sec.get(f"{prefix}_POS_A")), _vec(sec.get(f"{prefix}_POS_B"))
        out.append(_Joint(kind, str(val).strip(), a, b, pa, pb))
    return out


def _elements(ini: dict, axle: str, left: bool) -> list:
    out = []
    for name, sec in ini.items():
        m = re.match(rf"^{axle}_(COILOVER|SPRING|DAMPER)_(\d+)$", name.strip(), re.IGNORECASE)
        if not m:
            continue
        try:
            par = int(float(str(sec.get("PARITY", -1)).split()[0]))
        except ValueError:
            par = -1
        if not (par == -1 or (par == 0 and left) or (par == 1 and not left)):
            continue
        try:
            rate = float(str(sec.get("RATE", 0)).split()[0])
        except ValueError:
            rate = 0.0
        out.append({"kind": m.group(1).upper(), "index": int(m.group(2)),
                    "a": str(sec.get("BODY_A", "CHASSIS")).strip(),
                    "b": str(sec.get("BODY_B", "HUB_L")).strip(),
                    "pa": _vec(sec.get("POS_A")), "pb": _vec(sec.get("POS_B")),
                    "rate": rate})
    return sorted(out, key=lambda e: (e["kind"], e["index"]))


def _slug(name: str) -> str:
    return re.sub(r"[^a-z0-9]+", "_", name.lower()).strip("_") or "link"


def reduce_cosmic(ini: dict, axle: str, left: bool = True) -> Optional[dict]:
    """
    ``ini`` is the parsed suspensions.ini, ``axle`` ``"FRONT"`` or ``"REAR"``.  Returns
    None when the axle is not COSMIC, else::

        {"system_type", "links": [{"name", "type", "inboard": [[x,y,z]..], "end": [x,y,z],
                                   "hardpoint"}], "strut_lower": [..] | None,
         "coilovers": [{"index", "inboard", "outboard", "rate", "motion_ratio",
                        "wheel_rate"}], "extra_wheel_rate": float | None,
         "notes": [str]}
    """
    sec = ini.get(axle)
    if not sec or str(sec.get("TYPE", "")).strip().upper() != "COSMIC":
        return None
    joints = _joints(sec, left)
    elems = _elements(ini, axle, left)
    notes: list = []
    g = _Groups()
    for j in joints:
        g.find(j.a), g.find(j.b)
    g.find("CHASSIS"), g.find("HUB_L")

    # adjusters (slider + distance joint on the same pair and points) and bushings
    pair = lambda x, y: frozenset((x, y))
    dj_pairs = [(pair(j.a, j.b), j) for j in joints if j.kind == "DJ"]
    el_pairs = [pair(e["a"], e["b"]) for e in elems if e["kind"] in ("SPRING", "DAMPER")]
    merged = set()
    for s in joints:
        if s.kind != "SLIDER":
            continue
        pr = pair(s.a, s.b)
        same_dj = [d for p, d in dj_pairs if p == pr and
                   ((_close(d.pa, s.pa) and _close(d.pb, s.pb)) or
                    (_close(d.pa, s.pb) and _close(d.pb, s.pa)))]
        if same_dj or (pr in el_pairs and "HUB_L" not in pr):
            g.union(s.a, s.b)
            merged.add(id(s))
            merged.update(id(d) for d in same_dj)
    chassis, hub = g.find("CHASSIS"), g.find("HUB_L")

    def side(j: _Joint, grp):
        """(position on the `grp` side, position on the other side, other group)."""
        ga, gb = g.find(j.a), g.find(j.b)
        if ga == grp:
            return j.pa, j.pb, gb
        return j.pb, j.pa, ga

    dj_to_chassis: dict = {}                # body group -> [(distance joint, point on the body)]
    rods: list = []
    bodies: dict = {}                      # group -> {"ball_c": [...], "ball_h": ..., "slide_h": ...}
    steer_name = None
    for j in joints:
        if id(j) in merged:
            continue
        ga, gb = g.find(j.a), g.find(j.b)
        if ga == gb:
            continue
        if hub in (ga, gb):
            mine, other, og = side(j, hub)             # mine = hub-side point
            if og == chassis:
                if j.kind in ("DJ", "STEER") and other is not None and mine is not None:
                    rods.append({"name": j.name, "steer": j.kind == "STEER",
                                 "inboard": other, "end": mine})
                else:
                    notes.append(f"{j.kind} {j.name}: chassis-to-hub {j.kind} not converted")
            else:
                b = bodies.setdefault(og, {"ball_c": [], "ball_h": None, "slide_h": None,
                                           "name": j.name})
                if j.kind in ("J", "HJ"):
                    b["ball_h"] = (j.name, mine)
                elif j.kind == "SLIDER":
                    b["slide_h"] = (j.name, mine)
                else:
                    notes.append(f"{j.kind} {j.name}: body-to-hub {j.kind} not converted")
        elif chassis in (ga, gb) and j.kind in ("J", "HJ"):
            _mine, _other, og = side(j, chassis)
            pos = j.pa if j.pa is not None else j.pb
            bodies.setdefault(og, {"ball_c": [], "ball_h": None, "slide_h": None,
                                   "name": j.name})["ball_c"].append(pos)
        elif chassis in (ga, gb) and j.kind == "DJ":
            mine, _other, og = side(j, chassis)           # point on the body's side
            dj_to_chassis.setdefault(og, []).append((j, _other))
        else:
            notes.append(f"{j.kind} {j.name}: joint between {j.a} and {j.b} not converted")

    # three or more distance joints from the chassis meeting at one point on a body are a
    # ball joint at that point (adjustable by the lengths of the three rods)
    for og, items in dj_to_chassis.items():
        pts = [p for _j, p in items if p is not None]
        if len(items) >= 3 and pts and all(_close(p, pts[0]) for p in pts):
            bodies.setdefault(og, {"ball_c": [], "ball_h": None, "slide_h": None,
                                   "name": items[0][0].name})["ball_c"].append(pts[0])
        else:
            for j, _p in items:
                notes.append(f"DJ {j.name}: chassis-to-body distance joint not converted")

    links: list = []
    arms: list = []
    strut_lower = None
    pivots: dict = {}                      # group -> (axis point, axis direction, ball-joint point)
    for og, b in bodies.items():
        if len(b["ball_c"]) == 2 and b["ball_h"]:
            p0, p1 = b["ball_c"]
            arms.append({"name": _slug(b["ball_h"][0]), "inboard": [p0, p1],
                         "end": b["ball_h"][1]})
            d = _sub(p1, p0)
            if _norm(d) > 1e-6:
                pivots[og] = (p0, [x / _norm(d) for x in d], b["ball_h"][1])
        elif len(b["ball_c"]) == 1 and b["slide_h"]:
            strut_lower = b["slide_h"][1]
            links.append({"name": "strut", "type": "strut", "inboard": [b["ball_c"][0]],
                          "end": strut_lower, "hardpoint": "strut_lower"})
        else:
            notes.append(f"body near {b['name']}: attachment pattern not recognised")

    # rods ending at the same hub point form an A-arm
    used = set()
    for i, r in enumerate(rods):
        if i in used or r["steer"]:
            continue
        mates = [k for k in range(i + 1, len(rods))
                 if k not in used and not rods[k]["steer"] and _close(rods[k]["end"], r["end"])]
        if mates:
            k = mates[0]
            used.update((i, k))
            arms.append({"name": _slug(r["name"]) + "_arm",
                         "inboard": [r["inboard"], rods[k]["inboard"]], "end": r["end"]})
    for i, r in enumerate(rods):
        if i in used:
            continue
        if r["steer"]:
            links.append({"name": "steering_tie_rod" if axle == "FRONT" else "toe_link",
                          "type": "rod", "inboard": [r["inboard"]], "end": r["end"],
                          "hardpoint": "steering_tie_rod_end" if axle == "FRONT" else "toe_link_outer"})
        else:
            links.append({"name": _slug(r["name"]), "type": "rod", "inboard": [r["inboard"]],
                          "end": r["end"], "hardpoint": _slug(r["name"]) + "_end"})

    # name the arms: with two, the higher one (AC y = up) is the upper
    arms.sort(key=lambda a: -a["end"][1])
    n_arm = len(arms)
    for k, a in enumerate(arms):
        if n_arm == 2:
            role = "upper" if k == 0 else "lower"
            a["name"], a["hardpoint"] = f"{role}_wishbone", f"{role}_ball_joint"
        elif n_arm == 1 and strut_lower is not None:
            a["name"], a["hardpoint"] = "lower_control_arm", "lower_ball_joint"
        else:
            a["name"] = a["name"]
            a["hardpoint"] = a["name"] + "_end"
        a["type"] = "arm"
        links.insert(k, a)

    n_rod = sum(1 for l in links if l["type"] == "rod" and l["name"] not in
                ("steering_tie_rod", "toe_link"))
    if strut_lower is not None:
        system = "macpherson"
    elif n_arm == 2:
        system = "double_wishbone"
    elif n_arm + n_rod >= 3:
        system = "multi_link"
    else:
        system = "custom"
        if links:
            notes.append("link set not recognised as a standard suspension type")

    # spring / damper: every coilover, each with its motion ratio; extra vertical
    # "virtual" springs add wheel rate
    strut_groups = {og for og, b in bodies.items()
                    if len(b["ball_c"]) == 1 and b["slide_h"]}
    coils: list = []
    for e in elems:
        if e["kind"] != "COILOVER" or e["pa"] is None or e["pb"] is None:
            continue
        ga, gb = g.find(e["a"]), g.find(e["b"])
        if ga == chassis:
            inb, outb, mover = e["pa"], e["pb"], gb
        elif gb == chassis:
            inb, outb, mover = e["pb"], e["pa"], ga
        elif gb == hub and ga in strut_groups:       # strut body -> hub: the strut axis
            inb, outb, mover = e["pa"], e["pb"], hub
        elif ga == hub and gb in strut_groups:
            inb, outb, mover = e["pb"], e["pa"], hub
        else:
            notes.append(f"coilover {e['index']}: not between the chassis and a corner body")
            continue
        vec = _sub(inb, outb)
        length = _norm(vec)
        mr = None
        if length > 0.05:
            if mover == hub:
                mr = abs(vec[1]) / length
            elif mover in pivots:
                p0, n, bj = pivots[mover]
                u = [x / length for x in vec]
                rm = _sub(outb, p0)
                rm = [rm[i] - _dot(rm, n) * n[i] for i in range(3)]
                rb = _sub(bj, p0)
                rb = [rb[i] - _dot(rb, n) * n[i] for i in range(3)]
                dL = abs(_dot(u, _cross(n, rm)))
                dz = abs(_cross(n, rb)[1])
                if dz > 1e-4:
                    mr = dL / dz
        if mr is not None and not 0.2 <= mr <= 1.5:
            notes.append(f"coilover {e['index']}: motion ratio {mr:.2f} out of range, ignored")
            mr = None
        coils.append({"inboard": inb, "outboard": outb, "rate": e["rate"], "index": e["index"],
                      "motion_ratio": round(mr, 4) if mr else None,
                      "wheel_rate": round(e["rate"] * mr * mr, 1) if mr else None})
    extra = 0.0
    for e in elems:
        # a one-metre vertical spring from the chassis to the hub is a pure wheel rate
        if e["kind"] == "SPRING" and e["pa"] == [0.0, 1.0, 0.0] and e["pb"] == [0.0, 0.0, 0.0]:
            extra += e["rate"]
    return {"system_type": system, "links": links, "strut_lower": strut_lower,
            "coilovers": coils, "extra_wheel_rate": extra or None, "notes": notes}
