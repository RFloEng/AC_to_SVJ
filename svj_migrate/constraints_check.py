# Vendored from RFloEng/SVJ-standard-vehicle-json, tools/constraints_check.py (commit 4cb783c, Apache-2.0).
# Only the import lines were changed.
"""
SVJ Constraints Checker and Kinematics (v0.100)
===============================================
Cross-reference rules for movable parts (spec §24, proposals/svj_articulated_parts_proposal.md):
`constraints[]` (joint / spring / actuator / latch / coupling / contact / damper), `poses[]`,
`wheel_stations[]`, `anti_roll_bars[]` and `chassis.markers`, plus the forward
kinematics that turn a pose into rigid-body transforms, element lengths and a composite CG.

Used automatically by tools/validate.py; can also run standalone.

svj-py ships an identical copy as svj/constraints.py — keep the two files in sync
(tests/test_vehicle.py::test_constraints_checker_in_sync enforces it).

Usage:
    python tools/constraints_check.py <file.svj.json> [file2 ...]
    python tools/constraints_check.py --poses <file.svj.json>     # CG and element lengths per pose
"""

import json
import math
import re
import sys

ROOTS = ("chassis", "ground")
DOF = {"revolute": 1, "translational": 1, "cylindrical": 2, "spherical": 3,
       "universal": 2, "planar": 3, "fixed": 0}
AXIS_JOINTS = {"revolute", "translational", "cylindrical", "planar"}   # Z axis of the frame is meaningful
COINCIDE_TOL = 1e-3        # m, the two frames of a joint at the reference pose
AXIS_TOL = 1e-4            # 1 - |cos| between the two Z axes
QUAT_ERROR = 1e-2
QUAT_WARN = 1e-3
LEN_TOL = 1e-6



# ── naming vocabulary (docs/naming_convention.md, "Movable parts and attachments") ─────
_SIDE_SUFFIX = {"fl": "fl", "lf": "fl", "fr": "fr", "rf": "fr", "rl": "rl", "lr": "rl", "lb": "rl", "rr": "rr", "rb": "rr",
                "l": "l", "r": "r", "left": "l", "right": "r"}
_SIDE_PREFIX = {"front_left": "fl", "front_right": "fr", "rear_left": "rl", "rear_right": "rr", "left": "l", "right": "r",
                "fl": "fl", "fr": "fr", "rl": "rl", "rr": "rr"}
PART_CANONICAL = {"door", "hood", "trunk", "tailgate", "fuel_flap", "charge_flap", "mirror", "wiper", "sunroof", "cab", "tipper_body", "loader_boom", "loader_bucket", "boom", "stick", "bucket", "swing_frame", "hitch_lift_arm", "hitch_lower_link", "hitch_top_link", "hitch_drawbar", "implement"}
PART_ALIASES = {
    "car_door": "door",
    "bonnet": "hood",
    "engine_hood": "hood",
    "engine_bonnet": "hood",
    "boot": "trunk",
    "boot_lid": "trunk",
    "trunk_lid": "trunk",
    "deck_lid": "trunk",
    "rear_lid": "trunk",
    "liftgate": "tailgate",
    "lift_gate": "tailgate",
    "hatch": "tailgate",
    "rear_hatch": "tailgate",
    "hatchback": "tailgate",
    "tail_gate": "tailgate",
    "fuel_door": "fuel_flap",
    "fuel_filler_flap": "fuel_flap",
    "filler_flap": "fuel_flap",
    "fuel_cap_door": "fuel_flap",
    "gas_door": "fuel_flap",
    "charge_port_door": "charge_flap",
    "charge_door": "charge_flap",
    "charging_flap": "charge_flap",
    "side_mirror": "mirror",
    "wing_mirror": "mirror",
    "door_mirror": "mirror",
    "windscreen_wiper": "wiper",
    "windshield_wiper": "wiper",
    "sun_roof": "sunroof",
    "moonroof": "sunroof",
    "cabin": "cab",
    "driver_cab": "cab",
    "cab_shell": "cab",
    "truck_cab": "cab",
    "tipper": "tipper_body",
    "dump_body": "tipper_body",
    "dump_box": "tipper_body",
    "dumper_body": "tipper_body",
    "tipper_box": "tipper_body",
    "front_loader_boom": "loader_boom",
    "loader_arm": "loader_boom",
    "front_loader_arm": "loader_boom",
    "front_loader_bucket": "loader_bucket",
    "loader_shovel": "loader_bucket",
    "front_bucket": "loader_bucket",
    "crane_boom": "boom",
    "excavator_boom": "boom",
    "dipper": "stick",
    "dipper_stick": "stick",
    "dipper_arm": "stick",
    "excavator_bucket": "bucket",
    "turret": "swing_frame",
    "upper_structure": "swing_frame",
    "superstructure": "swing_frame",
    "lift_arm": "hitch_lift_arm",
    "rockshaft_arm": "hitch_lift_arm",
    "lower_link": "hitch_lower_link",
    "lift_link": "hitch_lower_link",
    "three_point_lower_link": "hitch_lower_link",
    "top_link": "hitch_top_link",
    "upper_link": "hitch_top_link",
    "third_link": "hitch_top_link",
    "drawbar": "hitch_drawbar",
    "draw_bar": "hitch_drawbar",
    "tool": "implement",
    "working_implement": "implement",
}
_ACTUATOR_ALIAS_SUFFIX = ("_ram", "_piston")

# suspension, steering and brake parts (docs/naming_convention.md, "Canonical part names"): a body that is one of
# these keeps the canonical name; the mechanism is defined by the components, never by the name
_SUSPENSION_VOCAB = {
    "upright": ("knuckle", "hub_carrier", "spindle", "stub_axle"), "hub": ("hub_flange", "wheel_hub"),
    "upper_wishbone": ("upper_arm", "uca", "wishbone_upper", "a_arm_top"), "lower_wishbone": ("lower_arm", "lca", "wishbone_lower", "a_arm_bottom"),
    "upper_link_front": ("upper_fore",), "upper_link_rear": ("upper_aft",), "lower_link_front": ("lower_fore",), "lower_link_rear": ("lower_aft",),
    "trailing_arm": ("longitudinal_arm", "radius_arm"), "semi_trailing_arm": ("sta", "diagonal_arm"), "leading_arm": ("front_trailing_arm",),
    "tie_rod": ("track_rod", "steering_rod", "steering_link"), "toe_link": ("toe_rod", "toe_control_arm"), "camber_link": ("camber_rod",),
    "drag_link": (), "center_link": ("relay_rod", "centre_link"), "pitman_arm": ("sector_arm",), "idler_arm": ("idle_arm",),
    "pushrod": ("push_rod",), "pullrod": ("pull_rod",), "rocker": ("bellcrank", "rocker_arm"),
    "strut": ("macpherson_strut", "damper_strut"), "torque_rod": ("radius_rod", "torque_arm"), "panhard_rod": ("track_bar", "lateral_rod"),
    "watts_link_rod": ("watts_arm",), "watts_pivot": ("watts_centre",), "axle_body": ("axle_housing", "de_dion_tube", "beam_axle"),
    "spring": ("coil", "air_spring"), "leaf_spring": (), "damper": ("shock", "shock_absorber"),
    "arb": ("sway_bar", "stabiliser_bar", "torsion_bar_arb"), "drop_link": ("arb_link", "end_link"),
    "wheel": ("rim", "tire", "tyre"), "disc": ("rotor", "brake_disc", "drum"), "caliper": ("brake_caliper", "brake_cylinder"),
    "rack": ("steering_gear", "steering_box"), "column": ("steering_shaft",), "steering_wheel": ("handwheel",),
}
for _canon, _als in _SUSPENSION_VOCAB.items():
    PART_CANONICAL.add(_canon)
    for _al in _als:
        PART_ALIASES[_al] = _canon


def canonical_part_id(pid):
    """The canonical spelling of a movable-part id, or None when the id is not a known part.

    Normalises case and separators, moves a side prefix (`front_left_door`) to a suffix, maps known
    aliases (`bonnet` -> `hood`, `hatch` -> `tailgate`) and keeps the side (`door_lf` -> `door_fl`).
    """
    if not isinstance(pid, str):
        return None
    base = "_".join(pid.strip().lower().replace("-", "_").split())
    side = None
    for pre in sorted(_SIDE_PREFIX, key=len, reverse=True):
        if base.startswith(pre + "_"):
            side, base = _SIDE_PREFIX[pre], base[len(pre) + 1:]
            break
    else:
        head, _, tail = base.rpartition("_")
        if head and tail in _SIDE_SUFFIX:
            base, side = head, _SIDE_SUFFIX[tail]
        elif head and re.fullmatch(r"a\d{1,2}[lrc]", tail):          # multi-axle station suffix (a1l, a3r, a2c)
            base, side = head, tail
    base = PART_ALIASES.get(base, base)
    if base not in PART_CANONICAL:
        return None
    return base + ("_" + side if side else "")


# ── small vector / matrix helpers (3x3 row-major tuples) ──────────────────────
def _mm(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)) for i in range(3))


def _mv(a, v):
    return tuple(sum(a[i][k] * v[k] for k in range(3)) for i in range(3))


def _tr(a):
    return tuple(tuple(a[j][i] for j in range(3)) for i in range(3))


def _add(a, b):
    return tuple(x + y for x, y in zip(a, b))


def _sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


I3 = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
IDENT = (I3, (0.0, 0.0, 0.0))


def quat_to_mat(q):
    x, y, z, w = (float(v) for v in q)
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return ((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
            (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
            (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)))


def rot_axis(u, angle):
    """Rodrigues rotation about unit vector u."""
    n = math.sqrt(sum(c * c for c in u))
    if n == 0 or angle == 0:
        return I3
    ux, uy, uz = (c / n for c in u)
    c, s = math.cos(angle), math.sin(angle)
    k = 1 - c
    return ((c + ux * ux * k, ux * uy * k - uz * s, ux * uz * k + uy * s),
            (uy * ux * k + uz * s, c + uy * uy * k, uy * uz * k - ux * s),
            (uz * ux * k - uy * s, uz * uy * k + ux * s, c + uz * uz * k))


def compose(a, b):
    """a ∘ b: apply b first, then a."""
    return (_mm(a[0], b[0]), _add(_mv(a[0], b[1]), a[1]))


def apply(t, p):
    return _add(_mv(t[0], p), t[1])


def is_identity(t, eps=1e-12):
    return all(abs(t[0][i][j] - I3[i][j]) < eps for i in range(3) for j in range(3)) and all(abs(v) < eps for v in t[1])


# ── document access ───────────────────────────────────────────────────────────
def _bodies(doc):
    ch = doc.get("chassis") if isinstance(doc.get("chassis"), dict) else {}
    return {b.get("id"): b for b in (ch.get("mass_bodies") or []) if isinstance(b, dict)}


def _marker(doc, body_id, name):
    if body_id == "chassis":
        src = (doc.get("chassis") or {}).get("markers")
    elif body_id == "ground":
        src = None
    else:
        src = (_bodies(doc).get(body_id) or {}).get("markers")
    m = src.get(name) if isinstance(src, dict) else None
    return m if isinstance(m, dict) and isinstance(m.get("position"), list) else None


def _frame(doc, c, side):
    """(position, orientation) of one side of a constraint, or None. side is 'a' or 'b'."""
    fr = c.get("frame_" + side)
    if isinstance(fr, dict) and isinstance(fr.get("position"), list):
        return tuple(fr["position"]), fr.get("orientation") or [0, 0, 0, 1]
    name = c.get("marker_" + side)
    if name is not None:
        m = _marker(doc, c.get("body_" + side), name)
        if m:
            return tuple(m["position"]), m.get("orientation") or [0, 0, 0, 1]
    return None


def _constraints(doc, kind=None):
    out = [c for c in (doc.get("constraints") or []) if isinstance(c, dict)]
    return [c for c in out if kind is None or c.get("type") == kind]


def joint_dof(joint_type):
    return DOF.get(joint_type)


# ── forward kinematics ────────────────────────────────────────────────────────
def _local_motion(joint_type, q):
    """Rigid motion (R, t) of body_b in the marker-A frame for coordinates q."""
    q = list(q) + [0.0] * (DOF.get(joint_type, 0) - len(q))
    if joint_type == "revolute":
        return rot_axis((0, 0, 1), q[0]), (0.0, 0.0, 0.0)
    if joint_type == "translational":
        return I3, (0.0, 0.0, q[0])
    if joint_type == "cylindrical":
        return rot_axis((0, 0, 1), q[0]), (0.0, 0.0, q[1])
    if joint_type == "spherical":
        ang = math.sqrt(q[0] ** 2 + q[1] ** 2 + q[2] ** 2)
        return (rot_axis(q[:3], ang) if ang else I3), (0.0, 0.0, 0.0)
    if joint_type == "universal":
        return _mm(rot_axis((0, 1, 0), q[1]), rot_axis((1, 0, 0), q[0])), (0.0, 0.0, 0.0)
    if joint_type == "planar":
        return rot_axis((0, 0, 1), q[2]), (q[0], q[1], 0.0)
    return I3, (0.0, 0.0, 0.0)


def _world_motion(frame, joint_type, q):
    """The joint motion expressed in reference-world coordinates."""
    if not any(abs(v) > 0 for v in q):
        return IDENT                                  # zero coordinates: exactly the reference pose
    p, orient = frame
    rm = quat_to_mat(orient)
    rl, tl = _local_motion(joint_type, q)
    rw = _mm(_mm(rm, rl), _tr(rm))
    tw = _sub(_add(tuple(p), _mv(rm, tl)), _mv(rw, tuple(p)))
    return rw, tw


def pose_values(doc, pose=None):
    """Resolve a pose (None, a pose id, or a {joint_id: value} mapping) to {joint_id: [coords]}."""
    if pose is None:
        return {}
    if isinstance(pose, str):
        for p in doc.get("poses") or []:
            if isinstance(p, dict) and p.get("id") == pose:
                pose = p.get("values") or {}
                break
        else:
            raise KeyError(f"no pose '{pose}'")
    vals = {k: (list(v) if isinstance(v, (list, tuple)) else [v]) for k, v in pose.items()}
    return apply_couplings(doc, vals)


def apply_couplings(doc, vals):
    """Fill in coordinates fixed by `coupling` constraints (q_b = ratio * q_a).

    A coupled joint the pose does not name takes its value from the joint it is coupled to,
    in either direction, and chains of couplings propagate. Values the pose names are kept.
    """
    cps = _constraints(doc, "coupling")
    if not cps:
        return vals
    joints = {c.get("id"): c for c in _constraints(doc, "joint")}

    def put(jid, i, x):
        dof = DOF.get((joints.get(jid) or {}).get("joint_type"), 0)
        v = list(vals.get(jid) or [])
        v += [0.0] * (dof - len(v))
        if i < len(v):
            v[i] = x
            vals[jid] = v

    explicit = set(vals)
    for _ in range(len(cps) + 1):
        changed = False
        for c in cps:
            ja, jb, r = c.get("joint_a"), c.get("joint_b"), c.get("ratio")
            ia, ib = c.get("coordinate_a", 0) or 0, c.get("coordinate_b", 0) or 0
            if ja not in joints or jb not in joints or not isinstance(r, (int, float)) or r == 0:
                continue
            if ja in vals and jb not in vals and ia < len(vals[ja]):
                put(jb, ib, r * vals[ja][ia]); changed = True
            elif jb in vals and ja not in vals and ib < len(vals[jb]):
                put(ja, ia, vals[jb][ib] / r); changed = True
        if not changed:
            break
    return vals


def body_transforms(doc, pose=None):
    """{body_id: (R, t)} rigid transforms from the reference pose to `pose` (reference-world coordinates)."""
    vals = pose_values(doc, pose)
    bodies = _bodies(doc)
    joints = _constraints(doc, "joint")
    X = {r: IDENT for r in ROOTS}
    pending = list(joints)
    progress = True
    while pending and progress:
        progress = False
        for c in list(pending):
            a, b = c.get("body_a"), c.get("body_b")
            if a in X:
                pending.remove(c)
                progress = True
                if b in X or b not in bodies:
                    continue                      # loop or unknown body: not solved here
                fr = _frame(doc, c, "a")
                if fr is None:
                    continue
                m = _world_motion(fr, c.get("joint_type"), vals.get(c.get("id"), []))
                X[b] = compose(X[a], m)
    memo = dict(X)

    def resolve(bid, seen=()):
        if bid in memo:
            return memo[bid]
        par = (bodies.get(bid) or {}).get("parent")
        if par in memo or (par in bodies and par not in seen and par != bid):
            memo[bid] = resolve(par, seen + (bid,)) if par not in memo else memo[par]
        else:
            memo[bid] = IDENT
        return memo[bid]

    for bid in bodies:
        resolve(bid)
    return memo


def element_lengths(doc, pose=None):
    """{constraint_id: length_m} of every spring and actuator in `pose`."""
    X = body_transforms(doc, pose)
    out = {}
    for c in _constraints(doc):
        if c.get("type") not in ("spring", "actuator", "damper") or c.get("kind") == "torsional":
            continue
        fa, fb = _frame(doc, c, "a"), _frame(doc, c, "b")
        if fa is None or fb is None:
            continue
        pa = apply(X.get(c.get("body_a"), IDENT), fa[0])
        pb = apply(X.get(c.get("body_b"), IDENT), fb[0])
        out[c.get("id")] = math.dist(pa, pb)
    return out


def element_angles(doc, pose=None):
    """{constraint_id: angle_rad} of every torsional spring and damper in `pose`: the rotation of frame B relative to
    frame A about their shared z axis (a torsion bar, an anti-roll bar half pair, a twist beam)."""
    X = body_transforms(doc, pose)
    out = {}
    for c in _constraints(doc):
        if c.get("type") not in ("spring", "damper") or c.get("kind") != "torsional":
            continue
        fa, fb = _frame(doc, c, "a"), _frame(doc, c, "b")
        if fa is None or fb is None:
            continue
        Ra = _mm(X.get(c.get("body_a"), IDENT)[0], quat_to_mat(fa[1]))
        Rb = _mm(X.get(c.get("body_b"), IDENT)[0], quat_to_mat(fb[1]))
        rel = _mm(_tr(Ra), Rb)                                   # B in A's frame; the free rotation is about z
        out[c.get("id")] = math.atan2(rel[1][0], rel[0][0])
    return out


def pose_cg(doc, pose=None):
    """Composite CG for `pose`: {'cg': [x,y,z], 'delta': [dx,dy,dz], 'mass': M}.

    The file's `center_of_gravity` is the reference-pose CG; each body's displacement
    (carried by its joint chain, or by its welded parent) shifts it by m_i·Δp_i / M.
    """
    ch = doc.get("chassis") or {}
    bodies = _bodies(doc)
    X = body_transforms(doc, pose)
    total = ch.get("mass_total")
    if not isinstance(total, (int, float)) or total <= 0:
        total = sum(b.get("mass", 0) for b in bodies.values()) or 1.0
    d = [0.0, 0.0, 0.0]
    for bid, b in bodies.items():
        p, m = b.get("position"), b.get("mass")
        if not isinstance(p, list) or len(p) != 3 or not isinstance(m, (int, float)):
            continue
        moved = apply(X.get(bid, IDENT), p)
        for i in range(3):
            d[i] += m * (moved[i] - p[i]) / total
    cg0 = ch.get("center_of_gravity") or [0.0, 0.0, 0.0]
    return {"cg": [cg0[i] + d[i] for i in range(3)], "delta": d, "mass": total}


def _inertia_mat(obj):
    """Full 3x3 inertia matrix from an `inertia` value (spec §7.1): the object form or a 3x3 array. None if unusable."""
    if (isinstance(obj, (list, tuple)) and len(obj) == 3
            and all(isinstance(r, (list, tuple)) and len(r) == 3 and all(isinstance(v, (int, float)) and not isinstance(v, bool) for v in r) for r in obj)):
        return tuple(tuple(float(v) for v in r) for r in obj)
    if not isinstance(obj, dict) or not all(isinstance(obj.get(k), (int, float)) for k in ("Ixx", "Iyy", "Izz")):
        return None
    g = lambda k: float(obj.get(k, 0.0) or 0.0)
    return ((g("Ixx"), -g("Ixy"), -g("Ixz")), (-g("Ixy"), g("Iyy"), -g("Iyz")), (-g("Ixz"), -g("Iyz"), g("Izz")))


def body_inertia(body):
    """A mass body's inertia matrix in the vehicle axes (applies `inertia_orientation`), or None."""
    m = _inertia_mat(body.get("inertia"))
    if m is None:
        return None
    q = body.get("inertia_orientation")
    if isinstance(q, list) and len(q) == 4:
        r = quat_to_mat(q)
        m = _mm(_mm(r, m), _tr(r))
    return m


def inertia_matrix(value):
    """Public helper: an `inertia` value (object or matrix) as a 3x3 list of lists, or None."""
    m = _inertia_mat(value)
    return [list(r) for r in m] if m else None


def _inertia_obj(m):
    return {"Ixx": m[0][0], "Iyy": m[1][1], "Izz": m[2][2], "Ixy": -m[0][1], "Ixz": -m[0][2], "Iyz": -m[1][2]}


def _steiner(mass, d):
    """Parallel-axis term m·(|d|²·1 − d dᵀ)."""
    dd = sum(c * c for c in d)
    return tuple(tuple(mass * ((dd if i == j else 0.0) - d[i] * d[j]) for j in range(3)) for i in range(3))


def _madd(a, b, sign=1.0):
    return tuple(tuple(a[i][j] + sign * b[i][j] for j in range(3)) for i in range(3))


def pose_inertia(doc, pose=None, form="object"):
    """Composite inertia tensor about the composite CG in `pose`, in vehicle axes (spec §7.1 object).

    Starts from the file's composite `chassis.inertia` (reference pose) and, for every body the pose
    moves, swaps its reference contribution (own tensor + parallel-axis term) for the rotated tensor
    at its new position, then shifts the reference point to the new CG. Returns None when the
    composite inertia is not given. A body without `inertia` counts as a point mass. Bodies may give
    `inertia` as an object or a 3x3 matrix and an `inertia_orientation`. `form="matrix"` returns the
    3x3 tensor (list of lists) instead of the §7.1 object.
    """
    ch = doc.get("chassis") or {}
    i_ref = _inertia_mat(ch.get("inertia"))
    if i_ref is None:
        return None
    bodies = _bodies(doc)
    X = body_transforms(doc, pose)
    r = pose_cg(doc, pose)
    cg0 = tuple(ch.get("center_of_gravity") or (0.0, 0.0, 0.0))
    total = r["mass"]
    acc = i_ref
    for bid, b in bodies.items():
        p, m = b.get("position"), b.get("mass")
        t = X.get(bid, IDENT)
        if is_identity(t) or not isinstance(p, list) or len(p) != 3 or not isinstance(m, (int, float)):
            continue
        ib = body_inertia(b) or ((0.0,) * 3,) * 3
        d0 = _sub(tuple(p), cg0)
        d1 = _sub(apply(t, tuple(p)), cg0)
        rot = t[0]
        acc = _madd(acc, _madd(ib, _steiner(m, d0)), -1.0)
        acc = _madd(acc, _madd(_mm(_mm(rot, ib), _tr(rot)), _steiner(m, d1)))
    res = _madd(acc, _steiner(total, tuple(r["delta"])), -1.0)
    return [list(row) for row in res] if form == "matrix" else _inertia_obj(res)


def actuator_force_limits(c):
    """(max_push_N, max_pull_N) of an actuator from `max_force` or `bore_diameter` × `max_pressure`; None if unknown."""
    if isinstance(c.get("max_force"), (int, float)):
        return c["max_force"], c["max_force"]
    bore, rod, pr = c.get("bore_diameter"), c.get("rod_diameter", 0.0), c.get("max_pressure")
    if isinstance(bore, (int, float)) and isinstance(pr, (int, float)):
        a_push = math.pi / 4 * bore * bore
        a_pull = math.pi / 4 * max(bore * bore - (rod or 0.0) ** 2, 0.0)
        return pr * a_push, pr * a_pull
    return None


def moving_bodies(doc, include_derived=True):
    """Bodies a joint can move, plus bodies welded to them through `parent`.

    With include_derived=False only joints that are not marked `x_derived` count (generated joints are exempt
    from the mass and inertia rules)."""
    bodies = _bodies(doc)
    moving = {c.get("body_b") for c in _constraints(doc, "joint")
              if c.get("body_b") in bodies and (include_derived or not c.get("x_derived"))}
    grew = True
    while grew:
        grew = False
        for bid, b in bodies.items():
            if bid not in moving and b.get("parent") in moving:
                moving.add(bid)
                grew = True
    return moving


# ── contact geometry ──────────────────────────────────────────────────────────
_CONTACT_PAIRS = {frozenset(("point", "plane")), frozenset(("sphere", "plane")), frozenset(("box", "plane")), frozenset(("cylinder", "plane")),
                  frozenset(("sphere",)), frozenset(("point", "sphere")), frozenset(("point", "box")), frozenset(("sphere", "box"))}


def contact_pair_supported(ta, tb):
    """True when the distance between the two primitive types is defined (spec §24.6.3)."""
    return frozenset((ta, tb)) in _CONTACT_PAIRS


def _shape_issue(shape):
    """Problem with a contact shape's dimensions, or None."""
    t = shape.get("type")
    pos = lambda k: isinstance(shape.get(k), (int, float)) and shape[k] > 0
    if t == "sphere" and not pos("radius"):
        return "a sphere needs a positive radius"
    if t == "cylinder" and not (pos("radius") and pos("length")):
        return "a cylinder needs a positive radius and length"
    if t == "box" and not (isinstance(shape.get("size"), list) and len(shape["size"]) == 3 and all(isinstance(x, (int, float)) and x > 0 for x in shape["size"])):
        return "a box needs a size of three positive lengths"
    return None


def _world_frame(doc, c, side, X):
    fr = _frame(doc, c, side)
    if fr is None:
        return None
    t = X.get(c.get("body_" + side), IDENT)
    return apply(t, fr[0]), _mm(t[0], quat_to_mat(fr[1]))


def _sd_box(local, half):
    q = [abs(local[i]) - half[i] for i in range(3)]
    return math.sqrt(sum(max(v, 0.0) ** 2 for v in q)) + min(max(q), 0.0)


def contact_distance(doc, c, pose=None):
    """Signed distance (m) between the two shapes of a `contact` constraint in `pose`; None if undefined.

    Positive = apart, zero = touching, negative = interpenetrating. The plane is a half-space whose free
    side is its frame's +Z; boxes and cylinders are centred on their frame.
    """
    X = body_transforms(doc, pose)
    fa, fb = _world_frame(doc, c, "a", X), _world_frame(doc, c, "b", X)
    sa, sb = c.get("shape_a") or {}, c.get("shape_b") or {}
    if fa is None or fb is None or not contact_pair_supported(sa.get("type"), sb.get("type")):
        return None
    ta, tb = sa.get("type"), sb.get("type")
    # normalise the order: plane second; point first; sphere before box
    order = {"plane": 3, "box": 2, "cylinder": 2, "sphere": 1, "point": 0}
    if order[ta] > order[tb]:
        (fa, sa, ta), (fb, sb, tb) = (fb, sb, tb), (fa, sa, ta)
    (pa, Ra), (pb, Rb) = fa, fb
    if tb == "plane":
        n = [Rb[i][2] for i in range(3)]
        h = lambda x: sum(n[i] * (x[i] - pb[i]) for i in range(3))
        if ta == "point":
            return h(pa)
        if ta == "sphere":
            return h(pa) - sa["radius"]
        if ta == "box":
            half = [x / 2 for x in sa["size"]]
            return min(h([pa[i] + sum(Ra[i][k] * sg[k] * half[k] for k in range(3)) for i in range(3)])
                       for sg in ((x, y, z) for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)))
        axis = [Ra[i][2] for i in range(3)]                                   # cylinder
        na = sum(n[i] * axis[i] for i in range(3))
        return h(pa) - sa["length"] / 2 * abs(na) - sa["radius"] * math.sqrt(max(0.0, 1 - na * na))
    if ta in ("point", "sphere") and tb == "sphere":
        return math.dist(pa, pb) - (sa.get("radius", 0.0) + sb["radius"])
    if tb == "box":                                                             # point or sphere against a box
        local = _mv(_tr(Rb), _sub(pa, pb))
        return _sd_box(local, [x / 2 for x in sb["size"]]) - (sa.get("radius", 0.0) if ta == "sphere" else 0.0)
    return None


# ── checker ───────────────────────────────────────────────────────────────────
def _quat_issue(path, q, errors, warnings):
    if not (isinstance(q, list) and len(q) == 4 and all(isinstance(x, (int, float)) for x in q)):
        return
    n = math.sqrt(sum(x * x for x in q))
    dev = abs(n - 1.0)
    if dev > QUAT_ERROR:
        errors.append(f"{path}: quaternion {q} is not a unit quaternion (norm {n:.4f})")
    elif dev > QUAT_WARN:
        warnings.append(f"{path}: quaternion norm {n:.4f} deviates from 1 (renormalise)")


def _range_lists(rng, dof, where, errors):
    """Normalise a joint `range` to (mins, maxs) lists of length dof (None = free), or None."""
    if not isinstance(rng, dict):
        return None
    out = []
    for key in ("min", "max"):
        v = rng.get(key)
        if v is None:
            vals = [None] * dof
        elif isinstance(v, (int, float)) and not isinstance(v, bool):
            vals = [v] + [None] * (dof - 1) if dof == 1 else None
        elif isinstance(v, list):
            vals = v
        else:
            vals = None
        if vals is None or len(vals) != dof:
            errors.append(f"{where}: range.{key} must have {dof} value(s) for this joint type")
            return None
        out.append(vals)
    return out[0], out[1]


def _z_axis(orient):
    return _mv(quat_to_mat(orient), (0.0, 0.0, 1.0))


def check(doc):
    errors, warnings = [], []
    cons = _constraints(doc)
    poses = [p for p in (doc.get("poses") or []) if isinstance(p, dict)]
    bodies = _bodies(doc)

    # ── inertia given as a matrix, or in a rotated frame (any mass body) ───────
    for bid, b in bodies.items():
        iv = b.get("inertia")
        where = f"mass_body '{bid}'"
        if isinstance(iv, list):
            m = _inertia_mat(iv)
            if m is None:
                errors.append(f"{where}: inertia matrix must be 3x3 numbers")
                continue
            scale = max(abs(v) for r in m for v in r) or 1.0
            if any(abs(m[i][j] - m[j][i]) > 1e-6 * scale for i in range(3) for j in range(i + 1, 3)):
                errors.append(f"{where}: inertia matrix is not symmetric")
        if b.get("inertia_orientation") is not None:
            if iv is None:
                warnings.append(f"{where}: inertia_orientation without inertia")
            _quat_issue(f"{where}.inertia_orientation", b["inertia_orientation"], errors, warnings)

    # ── naming advisories: a known alias where the canonical part name exists ───
    for bid in bodies:
        canon = canonical_part_id(bid)
        if canon is not None and canon != bid:
            warnings.append(f"mass_body '{bid}': use the canonical part name '{canon}' (docs/naming_convention.md)")
    for c in cons:
        cid = c.get("id")
        if c.get("type") == "actuator" and isinstance(cid, str):
            for suf in _ACTUATOR_ALIAS_SUFFIX:
                if cid.endswith(suf):
                    warnings.append(f"constraints '{cid}': name hydraulic actuators '<part>_cylinder', not '{suf}'")

    stations = [x for x in (doc.get("wheel_stations") or []) if isinstance(x, dict)]
    bars = [x for x in (doc.get("anti_roll_bars") or []) if isinstance(x, dict)]
    if not cons and not poses and not stations and not bars:
        return errors, warnings

    # ── wheel stations and anti-roll bars (spec 25) ────────────────────────────
    joint_children = {}
    for c in cons:
        if c.get("type") == "joint":
            joint_children.setdefault(c.get("body_b"), []).append(c)
    seen_st = set()
    for i, st in enumerate(stations):
        sid = st.get("id")
        where = f"wheel_stations[{i}] '{sid}'"
        if sid in seen_st:
            errors.append(f"{where}: duplicate station id")
        seen_st.add(sid)
        for key in ("wheel_body", "upright_body"):
            if st.get(key) is not None and st[key] not in bodies:
                errors.append(f"{where}: {key} '{st[key]}' is not a chassis.mass_bodies id")
        wb, ub = st.get("wheel_body"), st.get("upright_body")
        if wb in bodies and ub in bodies:
            if not any(j.get("body_a") == ub for j in joint_children.get(wb, [])):
                warnings.append(f"{where}: no joint connects wheel_body '{wb}' to upright_body '{ub}'")
        sj = st.get("steering_joint")
        if sj is not None and not any(c.get("id") == sj and c.get("type") == "joint" for c in cons):
            errors.append(f"{where}: steering_joint '{sj}' is not a joint id")
        wcm = st.get("wheel_center_marker", "wheel_center")
        if ub in bodies and st.get("wheel_center_marker") is not None and wcm not in (bodies[ub].get("markers") or {}):
            errors.append(f"{where}: wheel_center_marker '{wcm}' is not a marker of upright_body '{ub}'")
        sid_base = sid.rsplit(".", 1)[-1] if isinstance(sid, str) and doc.get("x_assembled") else sid       # `unit.A1L` in a composite
        m = re.fullmatch(r"(?:(FL|FR|RL|RR)|A(\d{1,2})([LRC]))", sid_base) if isinstance(sid_base, str) else None
        if m:
            axle, side = ("A1" if m.group(1) in ("FL", "FR") else "A2", m.group(1)[1]) if m.group(1) else ("A" + m.group(2), m.group(3))
            if st.get("axle") is not None and st["axle"] != axle:
                errors.append(f"{where}: axle '{st['axle']}' does not match the station id (expected {axle})")
            if st.get("side") is not None and st["side"] != side:
                errors.append(f"{where}: side '{st['side']}' does not match the station id (expected {side})")
    seen_bar = set()
    for i, bar in enumerate(bars):
        where = f"anti_roll_bars[{i}] '{bar.get('id')}'"
        if bar.get("id") in seen_bar:
            errors.append(f"{where}: duplicate bar id")
        seen_bar.add(bar.get("id"))
        for sid in bar.get("stations") or []:
            if sid not in seen_st:
                errors.append(f"{where}: station '{sid}' is not in wheel_stations")

    if not cons and not poses:
        return errors, warnings

    valid_bodies = set(bodies) | set(ROOTS)
    for bid in bodies:
        if isinstance(bid, str) and "." in bid and not doc.get("x_assembled"):
            warnings.append(f"chassis.mass_bodies '{bid}': '.' is reserved for qualified ids of attached units")

    # ── per constraint ─────────────────────────────────────────────────────
    seen_ids, joints, linkers, contacts = {}, [], [], []
    for i, c in enumerate(cons):
        cid = c.get("id")
        where = f"constraints[{i}] '{cid}'"
        if cid in seen_ids:
            errors.append(f"{where}: duplicate constraint id")
        seen_ids[cid] = c
        if c.get("type") in ("latch", "coupling"):
            linkers.append((i, c))
            continue
        a, b = c.get("body_a"), c.get("body_b")
        ok = True
        for side, bid in (("a", a), ("b", b)):
            if bid not in valid_bodies:
                errors.append(f"{where}: body_{side} '{bid}' is not a mass_bodies id, 'chassis' or 'ground'")
                ok = False
        if ok and a == b:
            errors.append(f"{where}: body_a and body_b are the same body")
            ok = False
        frames = {}
        for side in ("a", "b"):
            if c.get("frame_" + side) is None and c.get("marker_" + side) is None:
                errors.append(f"{where}: needs marker_{side} or frame_{side}")
                ok = False
                continue
            fr = _frame(doc, c, side)
            if fr is None:
                if c.get("marker_" + side) is not None and c.get("body_" + side) in valid_bodies:
                    errors.append(f"{where}: marker '{c.get('marker_' + side)}' not found on body '{c.get('body_' + side)}'")
                ok = False
            else:
                frames[side] = fr
                if c.get("frame_" + side) is not None:
                    _quat_issue(f"{where}.frame_{side}.orientation", fr[1], errors, warnings)
        t = c.get("type")
        if t == "joint":
            joints.append(c)
            jt = c.get("joint_type")
            dof = DOF.get(jt)
            rl = _range_lists(c.get("range"), dof, where, errors) if dof is not None and c.get("range") is not None else None
            if rl:
                mins, maxs = rl
                for k in range(dof):
                    lo, hi = mins[k], maxs[k]
                    if lo is not None and hi is not None and lo > hi:
                        errors.append(f"{where}: range.min ({lo}) is greater than range.max ({hi})")
                    elif (lo is not None and lo > 0) or (hi is not None and hi < 0):
                        errors.append(f"{where}: the reference pose (0) lies outside the range [{lo}, {hi}]")
            drv = c.get("drive")
            if isinstance(drv, dict):
                if dof is not None and dof != 1:
                    errors.append(f"{where}: drive is defined for one-coordinate joints (this one has {dof})")
                mode = drv.get("mode")
                if mode in ("position", "velocity", "force") and not isinstance(drv.get("target"), (int, float)):
                    errors.append(f"{where}: drive mode '{mode}' needs a target")
                if mode in ("position", "velocity", "force") and not isinstance(drv.get("max_force"), (int, float)):
                    warnings.append(f"{where}: a powered drive without max_force has an unknown capability")
                if mode == "position" and rl and isinstance(drv.get("target"), (int, float)):
                    lo, hi = rl[0][0], rl[1][0]
                    if (lo is not None and drv["target"] < lo - 1e-9) or (hi is not None and drv["target"] > hi + 1e-9):
                        errors.append(f"{where}: drive target {drv['target']} is outside the joint range [{lo}, {hi}]")
            if ok and len(frames) == 2:
                pa, pb = frames["a"][0], frames["b"][0]
                if math.dist(pa, pb) > COINCIDE_TOL:
                    errors.append(f"{where}: the two frames are {math.dist(pa, pb):.4f} m apart at the reference pose (must coincide)")
                if jt in AXIS_JOINTS:
                    dot = abs(sum(x * y for x, y in zip(_z_axis(frames["a"][1]), _z_axis(frames["b"][1]))))
                    if 1.0 - dot > AXIS_TOL:
                        errors.append(f"{where}: the Z axes of the two frames are not parallel at the reference pose")
        elif t == "contact":
            sa, sb = c.get("shape_a"), c.get("shape_b")
            if not isinstance(sa, dict) or not isinstance(sb, dict):
                errors.append(f"{where}: a contact needs shape_a and shape_b")
            else:
                for side, sh in (("a", sa), ("b", sb)):
                    issue = _shape_issue(sh)
                    if issue:
                        errors.append(f"{where}.shape_{side}: {issue}")
                if sa.get("type") and sb.get("type") and not contact_pair_supported(sa["type"], sb["type"]):
                    errors.append(f"{where}: the shape pair {sa['type']} / {sb['type']} is not supported (spec §24.6.3)")
            if not isinstance((c.get("law") or {}).get("stiffness"), (int, float)):
                errors.append(f"{where}: a contact needs law.stiffness")
            contacts.append((i, c, ok))
        elif t == "damper":
            for key in ("bump_curve", "rebound_curve"):
                xs = [p[0] for p in (c.get(key) or []) if isinstance(p, list) and p]
                if any(y <= x for x, y in zip(xs, xs[1:])):
                    errors.append(f"{where}: {key} velocities must be strictly increasing")
            if (c.get("bump_curve") is None) != (c.get("rebound_curve") is None):
                errors.append(f"{where}: give both bump_curve and rebound_curve, or a linear damping")
            if ok and len(frames) == 2 and c.get("kind") != "torsional" and c.get("reference") != "wheel" and math.dist(frames["a"][0], frames["b"][0]) < COINCIDE_TOL:
                warnings.append(f"{where}: both end frames coincide in the reference pose (zero-length element)")
            if t == "damper" and ok and len(frames) == 2 and c.get("kind") == "torsional":
                dot = abs(sum(x * y for x, y in zip(_z_axis(frames["a"][1]), _z_axis(frames["b"][1]))))
                if 1.0 - dot > AXIS_TOL:
                    errors.append(f"{where}: the Z axes of the two frames are not parallel at the reference pose")
        elif t in ("spring", "actuator"):
            if t == "spring" and c.get("rate") is not None and c.get("rate_curve") is not None:
                errors.append(f"{where}: give either rate or rate_curve, not both")
            if t == "spring" and isinstance(c.get("rate_curve"), list):
                xs = [p[0] for p in c["rate_curve"] if isinstance(p, list) and p]
                if any(y <= x for x, y in zip(xs, xs[1:])):
                    errors.append(f"{where}: rate_curve x values must be strictly increasing")
            if t == "actuator":
                st = c.get("stroke") or {}
                if isinstance(st.get("min"), (int, float)) and isinstance(st.get("max"), (int, float)) and st["min"] > st["max"]:
                    errors.append(f"{where}: stroke.min ({st['min']}) is greater than stroke.max ({st['max']})")
            if t == "spring" and ok and len(frames) == 2 and c.get("kind") == "torsional":
                dot = abs(sum(x * y for x, y in zip(_z_axis(frames["a"][1]), _z_axis(frames["b"][1]))))
                if 1.0 - dot > AXIS_TOL:
                    errors.append(f"{where}: the Z axes of the two frames are not parallel at the reference pose")
            elif ok and len(frames) == 2 and c.get("reference") != "wheel" and math.dist(frames["a"][0], frames["b"][0]) < COINCIDE_TOL:
                warnings.append(f"{where}: both end frames coincide in the reference pose (zero-length element)")

    # ── graph: reachability, parents, loops ────────────────────────────────
    reached = set(ROOTS)
    progress = True
    while progress:
        progress = False
        for c in joints:
            if c.get("body_a") in reached and c.get("body_b") not in reached and c.get("body_b") in bodies:
                reached.add(c["body_b"])
                progress = True
    child_count = {}
    for c in joints:
        b = c.get("body_b")
        if not c.get("x_derived"):                # a derived suspension is a closed loop by nature
            child_count[b] = child_count.get(b, 0) + 1
        derived_b = bool((bodies.get(b) or {}).get("x_derived") or c.get("x_derived"))
        if c.get("body_a") in valid_bodies and b in bodies and c.get("body_a") not in reached:
            (warnings if c.get("x_derived") else errors).append(
                f"constraints '{c.get('id')}': body '{c.get('body_a')}' is not connected to the chassis or ground through joints")
        par = (bodies.get(b) or {}).get("parent")
        if par is not None and par != c.get("body_a") and not derived_b:
            warnings.append(f"constraints '{c.get('id')}': body '{b}' has parent '{par}' but is jointed to '{c.get('body_a')}'")
    for b, n in child_count.items():
        if n > 1 and not (bodies.get(b) or {}).get("x_derived"):
            warnings.append(f"body '{b}' is the child of {n} joints (closed loop / over-constraint; poses give independent coordinates)")

    # ── physical completeness of moving bodies ──────────────────────────────
    for bid in sorted(moving_bodies(doc, include_derived=False)):
        b = bodies[bid]
        where = f"mass_body '{bid}'"
        if not isinstance(b.get("mass"), (int, float)) or b["mass"] <= 0:
            if b.get("x_derived"):
                continue                         # generated bodies (mass 0 allowed, §7.2) are exempt
            errors.append(f"{where}: a moving body needs a positive mass")
        im = body_inertia(b)
        if im is None:
            if b.get("x_derived"):
                continue                      # derived bodies cannot carry a tensor (link inertia is a scalar)
            warnings.append(f"{where}: moves but has no inertia tensor; it cannot be simulated as a rigid body (treated as a point mass)")
            continue
        d = (im[0][0], im[1][1], im[2][2])
        det3 = (im[0][0] * (im[1][1] * im[2][2] - im[1][2] * im[2][1])
                - im[0][1] * (im[1][0] * im[2][2] - im[1][2] * im[2][0])
                + im[0][2] * (im[1][0] * im[2][1] - im[1][1] * im[2][0]))
        minors = (im[0][0], im[0][0] * im[1][1] - im[0][1] * im[1][0], det3)
        tri = d[0] + d[1] >= d[2] - 1e-9 and d[1] + d[2] >= d[0] - 1e-9 and d[0] + d[2] >= d[1] - 1e-9
        if not all(v > 0 for v in d) or not all(m > 0 for m in minors) or not tri:
            warnings.append(f"{where}: the inertia tensor is not physically possible (needs a positive-definite tensor satisfying the triangle inequalities)")
    for c in _constraints(doc, "actuator"):
        has_geometry = c.get("bore_diameter") is not None or c.get("max_force") is not None
        if not has_geometry:
            warnings.append(f"constraints '{c.get('id')}': actuator has neither max_force nor bore_diameter/max_pressure, so its force capability is unknown")
        elif c.get("max_force") is None and c.get("max_pressure") is None:
            warnings.append(f"constraints '{c.get('id')}': bore_diameter given without max_pressure or max_force")
        rod, bore = c.get("rod_diameter"), c.get("bore_diameter")
        if isinstance(rod, (int, float)) and isinstance(bore, (int, float)) and rod >= bore:
            errors.append(f"constraints '{c.get('id')}': rod_diameter must be smaller than bore_diameter")

    # ── poses ──────────────────────────────────────────────────────────────
    jmap = {c.get("id"): c for c in joints}

    def jrange(j, k):
        dof = DOF.get(j.get("joint_type"), 0)
        rl = _range_lists(j.get("range"), dof, "", []) if j.get("range") is not None and dof else None
        return (rl[0][k], rl[1][k]) if rl and k < dof else (None, None)

    couplings = []
    driven = {}
    for i, c in linkers:
        where = f"constraints[{i}] '{c.get('id')}'"
        if c.get("type") == "latch":
            j = jmap.get(c.get("joint_ref"))
            if j is None:
                errors.append(f"{where}: joint_ref '{c.get('joint_ref')}' is not a joint id")
                continue
            k = c.get("coordinate", 0) or 0
            if k >= DOF.get(j.get("joint_type"), 0):
                errors.append(f"{where}: coordinate {k} does not exist on a {j.get('joint_type')} joint")
                continue
            lo, hi = jrange(j, k)
            pos = c.get("position")
            if isinstance(pos, (int, float)) and ((lo is not None and pos < lo - 1e-9) or (hi is not None and pos > hi + 1e-9)):
                errors.append(f"{where}: position {pos} is outside the joint range [{lo}, {hi}]")
            if c.get("state") == "latched" and isinstance(pos, (int, float)) and abs(pos) > 1e-6:
                errors.append(f"{where}: a latch that is 'latched' at the reference pose must engage at position 0, not {pos}")
        else:
            ja, jb = jmap.get(c.get("joint_a")), jmap.get(c.get("joint_b"))
            r = c.get("ratio")
            bad = False
            for name, j in (("joint_a", ja), ("joint_b", jb)):
                if j is None:
                    errors.append(f"{where}: {name} '{c.get(name)}' is not a joint id")
                    bad = True
            if bad:
                continue
            if c.get("joint_a") == c.get("joint_b") and (c.get("coordinate_a", 0) or 0) == (c.get("coordinate_b", 0) or 0):
                errors.append(f"{where}: a coupling needs two different joint coordinates")
                continue
            for name, j, key in (("joint_a", ja, "coordinate_a"), ("joint_b", jb, "coordinate_b")):
                if (c.get(key, 0) or 0) >= DOF.get(j.get("joint_type"), 0):
                    errors.append(f"{where}: {key} {c.get(key, 0) or 0} does not exist on a {j.get('joint_type')} joint")
                    bad = True
            if bad:
                continue
            if not isinstance(r, (int, float)) or r == 0:
                errors.append(f"{where}: ratio must be a non-zero number")
                continue
            ia, ib = c.get("coordinate_a", 0) or 0, c.get("coordinate_b", 0) or 0
            couplings.append((c, ia, ib, r))
            key = (c.get("joint_b"), ib)
            driven[key] = driven.get(key, 0) + 1
            la, ha = jrange(ja, ia)
            lb, hb = jrange(jb, ib)
            img = [None if v is None else v * r for v in (la, ha)]
            if r < 0:
                img.reverse()
            if (lb is not None and img[0] is not None and img[0] < lb - 1e-9) or (hb is not None and img[1] is not None and img[1] > hb + 1e-9):
                warnings.append(f"{where}: the range of '{c.get('joint_a')}' maps outside the range of '{c.get('joint_b')}' through ratio {r}")
    for (jid, k), n in driven.items():
        if n > 1:
            warnings.append(f"joint '{jid}' coordinate {k} is driven by {n} couplings (over-constrained)")

    seen_poses = set()
    settings = {s.get("name") for comp in ((doc.get("aerodynamics") or {}).get("components") or []) if isinstance(comp, dict)
                for s in (comp.get("settings") or []) if isinstance(s, dict)}
    pose_ok = {}
    for i, p in enumerate(poses):
        pid = p.get("id")
        where = f"poses[{i}] '{pid}'"
        if pid in seen_poses:
            errors.append(f"{where}: duplicate pose id")
        seen_poses.add(pid)
        good = True
        for jid, v in (p.get("values") or {}).items():
            j = jmap.get(jid)
            if j is None:
                errors.append(f"{where}: '{jid}' is not a joint id")
                good = False
                continue
            dof = DOF.get(j.get("joint_type"))
            vec = v if isinstance(v, list) else [v]
            if dof is None or len(vec) != dof or not all(isinstance(x, (int, float)) and not isinstance(x, bool) for x in vec):
                errors.append(f"{where}: '{jid}' needs {dof} coordinate(s) for a {j.get('joint_type')} joint")
                good = False
                continue
            rl = _range_lists(j.get("range"), dof, where, [])
            if rl:
                for k in range(dof):
                    lo, hi = rl[0][k], rl[1][k]
                    if (lo is not None and vec[k] < lo - 1e-9) or (hi is not None and vec[k] > hi + 1e-9):
                        errors.append(f"{where}: '{jid}' value {vec[k]} is outside its range [{lo}, {hi}]")
        if good and couplings:
            vals_p = {k: (list(v) if isinstance(v, (list, tuple)) else [v]) for k, v in (p.get("values") or {}).items()}
            given = set(vals_p)
            derived = apply_couplings(doc, {k: list(v) for k, v in vals_p.items()})
            for c, ia, ib, r in couplings:
                ja, jb = c.get("joint_a"), c.get("joint_b")
                if ja in given and jb in given and ia < len(vals_p[ja]) and ib < len(vals_p[jb]) \
                        and abs(vals_p[jb][ib] - r * vals_p[ja][ia]) > 1e-6:
                    errors.append(f"{where}: '{jb}' = {vals_p[jb][ib]} violates coupling '{c.get('id')}' (expected {r * vals_p[ja][ia]:.6g})")
            for jid, vec in derived.items():
                if jid in given or jid not in jmap:
                    continue
                for k, x in enumerate(vec):
                    lo, hi = jrange(jmap[jid], k)
                    if (lo is not None and x < lo - 1e-9) or (hi is not None and x > hi + 1e-9):
                        errors.append(f"{where}: '{jid}' reaches {x:.4g} through a coupling, outside its range [{lo}, {hi}]")
        ref = p.get("aero_setting_ref")
        if ref is not None and ref not in settings:
            errors.append(f"{where}: aero_setting_ref '{ref}' is not an aerodynamics.components[].settings name")
        pose_ok[pid] = good

    # ── actuator stroke in the reference pose and every pose ───────────────
    acts = _constraints(doc, "actuator")
    if acts:
        targets = [(None, "reference pose")] + [(p.get("id"), f"pose '{p.get('id')}'") for p in poses if pose_ok.get(p.get("id"), False)]
        for pid, label in targets:
            try:
                lengths = element_lengths(doc, pid)
            except Exception:                                  # a malformed pose was already reported
                continue
            for c in acts:
                L = lengths.get(c.get("id"))
                rl, st = c.get("retracted_length"), c.get("stroke") or {}
                if L is None or not isinstance(rl, (int, float)):
                    continue
                lo, hi = rl + st.get("min", 0.0), rl + st.get("max", 0.0)
                if L < lo - 1e-4 or L > hi + 1e-4:
                    errors.append(f"{label}: actuator '{c.get('id')}' needs length {L:.3f} m, outside its reach {lo:.3f}–{hi:.3f} m")

    # ── contacts: reference-pose gap and interpenetration in poses ─────────────────
    if contacts:
        for i, c, ok in contacts:
            where = f"constraints[{i}] '{c.get('id')}'"
            if not ok:
                continue
            try:
                d0 = contact_distance(doc, c, None)
            except Exception:
                d0 = None
            if d0 is None:
                continue
            gap = (c.get("law") or {}).get("gap", 0.0) or 0.0
            if abs(d0 - gap) > COINCIDE_TOL:
                errors.append(f"{where}: the shapes are {d0 * 1000:.1f} mm apart at the reference pose but law.gap is {gap * 1000:.1f} mm")
            for p in poses:
                if not pose_ok.get(p.get("id"), False):
                    continue
                try:
                    dp = contact_distance(doc, c, p.get("id"))
                except Exception:
                    continue
                if dp is not None and dp < min(gap, 0.0) - COINCIDE_TOL:
                    warnings.append(f"pose '{p.get('id')}': contact '{c.get('id')}' interpenetrates by {(min(gap, 0.0) - dp) * 1000:.1f} mm")
    return errors, warnings


def main(argv):
    show = "--poses" in argv
    files = [a for a in argv if a != "--poses"]
    if not files:
        print(__doc__)
        return 2
    bad = False
    for path in files:
        with open(path, "r", encoding="utf-8") as f:
            doc = json.load(f)
        if show:
            print(path)
            for pid in [None] + [p.get("id") for p in doc.get("poses") or [] if isinstance(p, dict)]:
                r = pose_cg(doc, pid)
                ls = ", ".join(f"{k} {v:.3f}" for k, v in element_lengths(doc, pid).items())
                ii = pose_inertia(doc, pid)
                print(f"  {pid or 'reference':<16} CG {', '.join(f'{v:.3f}' for v in r['cg'])}  dCG {', '.join(f'{v:+.4f}' for v in r['delta'])}"
                      + (f"  lengths: {ls}" if ls else ""))
                if ii:
                    print(f"  {'':<16} inertia Ixx {ii['Ixx']:.1f} Iyy {ii['Iyy']:.1f} Izz {ii['Izz']:.1f} Ixy {ii['Ixy']:.2f} Ixz {ii['Ixz']:.2f} Iyz {ii['Iyz']:.2f}")
            continue
        errs, warns = check(doc)
        print(f"{path}: {len(errs)} error(s), {len(warns)} warning(s)")
        for w in warns:
            print(f"  WARN   {w}")
        for e in errs:
            print(f"  ERROR  {e}")
        bad = bad or bool(errs)
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
