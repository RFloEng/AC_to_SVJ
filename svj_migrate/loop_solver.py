# Vendored from RFloEng/SVJ-standard-vehicle-json, tools/loop_solver.py (commit 4cb783c, Apache-2.0).
# Only the import lines were changed.
#!/usr/bin/env python3
"""
SVJ loop-closure solver (spec §26) — poses of mechanisms with closed kinematic loops.

body_transforms() in constraints_check.py walks a tree: it takes a joint value for every joint and
ignores the loops. A suspension is a closed loop (chassis - arm - upright - tie rod - rack - chassis), so its
pose cannot be read off joint values; it has to be solved. This module does that numerically:

  * the unknowns are the rigid motions (rotation vector + translation) of the bodies of one mechanism,
    the chassis and ground being fixed;
  * every `joint` constraint contributes residuals for the motions it forbids (e.g. a revolute joint:
    the two frame origins coincide and the z axes stay parallel); its free coordinates stay free;
  * prescribed inputs add residuals: joint coordinates (`joint_values`: the steering rack position,
    a door angle) and wheel travel (`travel`: the height a station's wheel centre rises, bump positive; the file's own frame is read, ISO 8855 z up or legacy SAE J670 z down);
  * Gauss-Newton iterations with Levenberg-Marquardt damping and a finite-difference Jacobian, so
    a mechanism with spare freedoms (an unmodelled bushing, an `x_derived` joint without a stated
    type) still has a minimum-motion solution, and the residual tells whether the loops closed.

Results: wheel-centre position, camber, toe and the wheel's spin axis per station and travel, and the
derived bump-steer and camber gain, and the kingpin axis of a steered station (caster, KPI, scrub radius, caster trail). Pure Python, no dependencies.

Usage:
    python tools/loop_solver.py file.svj.json                      # sweep every station, -50..+50 mm
    python tools/loop_solver.py file.svj.json --station FL --range 0.08 --steps 9
    python tools/loop_solver.py file.svj.json --station FL --steer 0.02     # rack at +20 mm
    python tools/loop_solver.py file.svj.json --check-alignment             # declared alignment vs geometry

Approximations: bushing compliance is not applied (joints are rigid), except that a bushing direction with a rate below
    SOFT_RATE is left free (a twist beam's torsion); wheel spin is not
modelled (the wheel body is not a loop member); tyre deflection is not part of travel.
"""

import json
import math
import sys

from . import constraints_check as cc
from .frame import frame_of, ISO


def _up(doc):
    """+1 when z points up (ISO 8855), -1 when it points down (legacy SAE J670): the one sign this solver depends on."""
    return 1.0 if frame_of(doc) == ISO else -1.0

ROOTS = cc.ROOTS
_TOL = 1e-10
SOFT_RATE = 1e4         # a bushing rate below this (N/m, N m/rad) leaves its direction free: compliance, not a constraint
_FREE_POS = {            # joint type -> (position components left free in frame A, rotation components left free)
    "revolute": ((), (2,)),
    "translational": ((2,), ()),
    "cylindrical": ((2,), (2,)),
    "spherical": ((), (0, 1, 2)),
    "universal": ((), (0, 1)),          # the twist about the axis between the crosses is left free as well
    "planar": ((0, 1), (2,)),
    "fixed": ((), ()),
}


# ── small linear algebra ──────────────────────────────────────────────────────
def _rotvec(R):
    """Rotation vector of a rotation matrix (angle < pi)."""
    tr = R[0][0] + R[1][1] + R[2][2]
    c = max(-1.0, min(1.0, (tr - 1.0) / 2.0))
    ang = math.acos(c)
    v = (R[2][1] - R[1][2], R[0][2] - R[2][0], R[1][0] - R[0][1])
    s = math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)
    if s < 1e-14:
        return (0.0, 0.0, 0.0)
    k = ang / s
    return (v[0] * k, v[1] * k, v[2] * k)


def _transform(params):
    w = params[:3]
    ang = math.sqrt(w[0] ** 2 + w[1] ** 2 + w[2] ** 2)
    return (cc.rot_axis(w, ang) if ang else cc.I3), tuple(params[3:6])


def _solve_linear(A, b):
    """Solve A x = b by Gaussian elimination with partial pivoting."""
    n = len(b)
    M = [row[:] + [b[i]] for i, row in enumerate(A)]
    for i in range(n):
        p = max(range(i, n), key=lambda r: abs(M[r][i]))
        if abs(M[p][i]) < 1e-18:
            M[p][i] = 1e-18
        M[i], M[p] = M[p], M[i]
        for r in range(i + 1, n):
            f = M[r][i] / M[i][i]
            if f:
                for c in range(i, n + 1):
                    M[r][c] -= f * M[i][c]
    x = [0.0] * n
    for i in range(n - 1, -1, -1):
        x[i] = (M[i][n] - sum(M[i][j] * x[j] for j in range(i + 1, n))) / M[i][i]
    return x


# ── mechanism model ───────────────────────────────────────────────────────────
def _station_wheel_bodies(doc):
    return {st.get("wheel_body") for st in doc.get("wheel_stations") or [] if isinstance(st, dict)}


def _components(doc):
    """Groups of free bodies connected by joints (the chassis and ground cut the graph). Wheel bodies are left out."""
    bodies = cc._bodies(doc)
    wheels = _station_wheel_bodies(doc)
    adj = {}
    for c in cc._constraints(doc, "joint"):
        a, b = c.get("body_a"), c.get("body_b")
        for x, y in ((a, b), (b, a)):
            if x in bodies and x not in wheels and y in bodies and y not in wheels:
                adj.setdefault(x, set()).add(y)
    for c in cc._constraints(doc, "joint"):        # bodies hanging only on a root
        for x in (c.get("body_a"), c.get("body_b")):
            if x in bodies and x not in wheels:
                adj.setdefault(x, set())
    seen, comps = set(), []
    for start in sorted(adj):
        if start in seen:
            continue
        stack, comp = [start], []
        seen.add(start)
        while stack:
            x = stack.pop()
            comp.append(x)
            for y in adj[x]:
                if y not in seen:
                    seen.add(y)
                    stack.append(y)
        comps.append(sorted(comp))
    return comps


def _joint_residuals(doc, c, X):
    """Residuals of the motions joint `c` forbids, in frame A of the joint."""
    fa, fb = cc._frame(doc, c, "a"), cc._frame(doc, c, "b")
    if fa is None or fb is None:
        return []
    jt = c.get("joint_type")
    free_p, free_r = _FREE_POS.get(jt, ((), ()))
    Xa, Xb = X.get(c.get("body_a"), cc.IDENT), X.get(c.get("body_b"), cc.IDENT)
    pa, pb = cc.apply(Xa, fa[0]), cc.apply(Xb, fb[0])
    Ra = cc._mm(Xa[0], cc.quat_to_mat(fa[1]))
    Rb = cc._mm(Xb[0], cc.quat_to_mat(fb[1]))
    d = cc._mv(cc._tr(Ra), cc._sub(pb, pa))                      # B's origin in A's frame
    w = _rotvec(cc._mm(cc._tr(Ra), Rb))                          # B's orientation in A's frame
    # reference orientation offset between the two frames (zero when the frames coincide, as the checker demands)
    w0 = _rotvec(cc._mm(cc._tr(cc.quat_to_mat(fa[1])), cc.quat_to_mat(fb[1])))
    bush = c.get("bushing") if isinstance(c.get("bushing"), dict) else {}
    soft_p = {i for i, n in enumerate("xyz") if isinstance(bush.get("rate_" + n), (int, float)) and bush["rate_" + n] < SOFT_RATE}
    soft_r = {i for i, n in enumerate("xyz") if isinstance(bush.get("rate_r" + n), (int, float)) and bush["rate_r" + n] < SOFT_RATE}
    out = [d[i] for i in range(3) if i not in free_p and i not in soft_p]
    out += [w[i] - w0[i] for i in range(3) if i not in free_r and i not in soft_r]
    return out


def _coordinate(doc, c, X):
    """Measured coordinates of a joint (revolute angle, slide distance, ...) in the current state."""
    fa, fb = cc._frame(doc, c, "a"), cc._frame(doc, c, "b")
    jt = c.get("joint_type")
    Xa, Xb = X.get(c.get("body_a"), cc.IDENT), X.get(c.get("body_b"), cc.IDENT)
    pa, pb = cc.apply(Xa, fa[0]), cc.apply(Xb, fb[0])
    Ra = cc._mm(Xa[0], cc.quat_to_mat(fa[1]))
    Rb = cc._mm(Xb[0], cc.quat_to_mat(fb[1]))
    d = cc._mv(cc._tr(Ra), cc._sub(pb, pa))
    w = _rotvec(cc._mm(cc._tr(Ra), Rb))
    if jt == "revolute":
        return [w[2]]
    if jt == "translational":
        return [d[2]]
    if jt == "cylindrical":
        return [w[2], d[2]]
    return None


def _wheel_centre(doc, st, X):
    ub, mk = st.get("upright_body"), st.get("wheel_center_marker") or "wheel_center"
    m = cc._marker(doc, ub, mk)
    if not m:
        return None
    return cc.apply(X.get(ub, cc.IDENT), tuple(m["position"]))


def _station_map(doc):
    return {st.get("id"): st for st in doc.get("wheel_stations") or [] if isinstance(st, dict)}


class SolveError(ValueError):
    pass


def solve(doc, travel=None, joint_values=None, start=None, max_iter=60, tol=1e-9):
    """Solve the pose in which each station's wheel centre has risen by travel[station] (m) and the
    joints named in `joint_values` ({joint_id: value or [values]}) sit at those coordinates.

    Returns {'transforms': {body: (R, t)}, 'residual': max abs residual, 'converged': bool,
             'iterations': n, 'wheel_centres': {station: [x, y, z]}}.
    `start` is a previous result's transforms (continuation along a sweep).
    """
    travel = dict(travel or {})
    jv = {k: (list(v) if isinstance(v, (list, tuple)) else [v]) for k, v in (joint_values or {}).items()}
    jv = cc.apply_couplings(doc, jv)
    stations = _station_map(doc)
    for sid in travel:
        if sid not in stations:
            raise SolveError(f"unknown wheel station '{sid}'")
    joints = {c.get("id"): c for c in cc._constraints(doc, "joint")}
    for jid in jv:
        if jid not in joints:
            raise SolveError(f"unknown joint '{jid}'")
    ref_centre = {sid: _wheel_centre(doc, st, {}) for sid, st in stations.items()}
    up = _up(doc)
    X_all = {r: cc.IDENT for r in ROOTS}
    max_res, iters, ok = 0.0, 0, True
    wheels = _station_wheel_bodies(doc)
    for comp in _components(doc):
        cset = set(comp)
        cjoints = [c for c in joints.values() if c.get("body_a") in cset or c.get("body_b") in cset]
        cjoints = [c for c in cjoints
                   if all(b in cset or b in ROOTS or b in wheels for b in (c.get("body_a"), c.get("body_b")))
                   and not (c.get("body_a") in wheels or c.get("body_b") in wheels)]
        # every station of the mechanism is held at its stated travel, zero when not named, so a sweep of one
        # wheel does not let the others (or the rack) drift to the minimum-motion solution
        csts = [sid for sid, st in stations.items() if st.get("upright_body") in cset]
        hold = {c["id"]: [0.0] for c in cjoints if c.get("id") in {st.get("steering_joint") for st in stations.values()}
                and c.get("joint_type") in ("revolute", "translational") and c["id"] not in jv}
        want_all = dict(hold)
        want_all.update(jv)
        pres = [c for c in cjoints if c.get("id") in want_all and _coordinate_safe(doc, c)]
        n = 6 * len(comp)
        index = {b: i for i, b in enumerate(comp)}
        x = [0.0] * n
        if start:
            for b, i in index.items():
                if b in start:
                    R, t = start[b]
                    w = _rotvec(R)
                    x[6 * i:6 * i + 3] = list(w)
                    x[6 * i + 3:6 * i + 6] = list(t)

        def state(xv):
            X = dict(X_all)
            for b, i in index.items():
                X[b] = _transform(xv[6 * i:6 * i + 6])
            return X

        def resid(xv):
            X = state(xv)
            r = []
            for c in cjoints:
                r += _joint_residuals(doc, c, X)
            for c in pres:
                want = want_all[c["id"]]
                got = _coordinate(doc, c, X) or []
                if got and c.get("joint_type") in ("revolute", "cylindrical") and want_all[c["id"]]:
                    # the measured angle lies in (-pi, pi]; a steering wheel turns several times, so compare modulo 2*pi
                    got = [got[0] + 2 * math.pi * round((want_all[c["id"]][0] - got[0]) / (2 * math.pi))] + list(got[1:])
                # the angular residual is compared in the frame-A rotation vector; the slide in metres
                r += [(got[k] - want[k]) if k < len(want) else 0.0 for k in range(len(got))]
            for sid in csts:
                p = _wheel_centre(doc, stations[sid], X)
                if p is not None and ref_centre[sid] is not None:
                    r.append(p[2] - ref_centre[sid][2] - up * travel.get(sid, 0.0))     # bump raises the wheel: +z in ISO, -z in SAE
            return r

        lam, r0 = 1e-6, resid(x)
        it = 0
        for it in range(1, max_iter + 1):
            m = len(r0)
            if m == 0 or max(abs(v) for v in r0) < tol:
                break
            J = [[0.0] * n for _ in range(m)]
            for j in range(n):
                h = 1e-7
                xp = x[:]
                xp[j] += h
                rp = resid(xp)
                for i in range(m):
                    J[i][j] = (rp[i] - r0[i]) / h
            JtJ = [[sum(J[k][i] * J[k][j] for k in range(m)) for j in range(n)] for i in range(n)]
            Jtr = [sum(J[k][i] * r0[k] for k in range(m)) for i in range(n)]
            for _try in range(12):
                A = [row[:] for row in JtJ]
                for i in range(n):
                    A[i][i] += lam
                dx = _solve_linear(A, [-v for v in Jtr])
                xn = [x[i] + dx[i] for i in range(n)]
                rn = resid(xn)
                if sum(v * v for v in rn) < sum(v * v for v in r0):
                    x, r0, lam = xn, rn, max(lam / 5.0, 1e-12)
                    break
                lam *= 10.0
            else:
                break
        res = max([abs(v) for v in r0] or [0.0])
        max_res = max(max_res, res)
        iters = max(iters, it)
        ok = ok and res < 1e-6
        X_all.update(state(x))
    # bodies outside every mechanism: follow the plain tree walk
    base = cc.body_transforms(doc, jv)
    for b, t in base.items():
        X_all.setdefault(b, t)
    centres = {}
    for sid, st in stations.items():
        p = _wheel_centre(doc, st, X_all)
        if p is not None:
            centres[sid] = [round(v, 9) for v in p]
    return {"transforms": X_all, "residual": max_res, "converged": ok, "iterations": iters, "wheel_centres": centres}


def _coordinate_safe(doc, c):
    return c.get("joint_type") in ("revolute", "translational", "cylindrical") and \
        cc._frame(doc, c, "a") is not None and cc._frame(doc, c, "b") is not None


# ── wheel alignment ───────────────────────────────────────────────────────────
def _wheel_hinge(doc, st):
    for c in cc._constraints(doc, "joint"):
        if {c.get("body_a"), c.get("body_b")} == {st.get("upright_body"), st.get("wheel_body")}:
            return c
    return None


def wheel_axis(doc, sid, X):
    """Unit spin axis of the station's wheel in the pose X, pointing outboard, or None."""
    st = _station_map(doc).get(sid)
    if not st:
        return None
    h = _wheel_hinge(doc, st)
    if not h:
        return None
    side = "a" if h.get("body_a") == st.get("upright_body") else "b"
    fr = cc._frame(doc, h, side)
    if fr is None:
        return None
    R = cc._mm(X.get(st["upright_body"], cc.IDENT)[0], cc.quat_to_mat(fr[1]))
    a = (R[0][2], R[1][2], R[2][2])
    centre = _wheel_centre(doc, st, X) or (0.0, 0.0, 0.0)
    if a[1] * (centre[1] if centre[1] else 1.0) < 0:
        a = (-a[0], -a[1], -a[2])
    return a


def alignment(doc, sid, X):
    """{'camber_deg', 'toe_deg'} of a station in pose X (in the file's frame; the angles do not depend on it).
    Camber is positive with the top of the wheel leaning outwards, toe positive for toe-in."""
    a = wheel_axis(doc, sid, X)
    if a is None:
        return None
    centre = _wheel_centre(doc, _station_map(doc)[sid], X)
    s = 1.0 if (centre and centre[1] >= 0) else -1.0
    return {"camber_deg": math.degrees(math.asin(max(-1.0, min(1.0, -_up(doc) * a[2])))),
            "toe_deg": math.degrees(math.atan2(a[0], s * a[1]))}


def sweep(doc, station, travels, steer=None, steering_joint=None):
    """Solve a station through a list of wheel travels (m, bump positive). Returns one dict per travel:
    travel, converged, residual, wheel_centre, camber_deg, toe_deg and the lengths of the elements."""
    st = _station_map(doc).get(station)
    if st is None:
        raise SolveError(f"unknown wheel station '{station}'")
    jv = {}
    sj = steering_joint or st.get("steering_joint")
    if steer is not None:
        if not sj:
            raise SolveError(f"station '{station}' has no steering_joint")
        jv[sj] = steer
    out, prev = [], None
    for t in travels:
        r = solve(doc, travel={station: t}, joint_values=jv, start=prev)
        prev = r["transforms"]
        al = alignment(doc, station, r["transforms"]) or {}
        out.append({"travel": t, "converged": r["converged"], "residual": r["residual"],
                    "wheel_centre": r["wheel_centres"].get(station), **al,
                    "element_lengths": _lengths(doc, r["transforms"])})
    return out


def _lengths(doc, X):
    out = {}
    for c in cc._constraints(doc):
        if c.get("type") not in ("spring", "actuator", "damper") or c.get("kind") == "torsional":
            continue
        fa, fb = cc._frame(doc, c, "a"), cc._frame(doc, c, "b")
        if fa is None or fb is None:
            continue
        out[c["id"]] = math.dist(cc.apply(X.get(c.get("body_a"), cc.IDENT), fa[0]),
                                 cc.apply(X.get(c.get("body_b"), cc.IDENT), fb[0]))
    return out


def steering_axis(doc, station, travel=0.0, delta=0.002):
    """Kingpin geometry of a steered station: the screw axis of the upright for a small rack movement.

    Returns {'caster_deg', 'kpi_deg', 'scrub_radius', 'caster_trail'} (either frame; caster positive = axis leaning back at the top,
    kpi positive = leaning inwards at the top; scrub radius positive = ground point inboard of the axis) or None."""
    st = _station_map(doc).get(station)
    sj = st.get("steering_joint") if st else None
    if not sj:
        return None
    lo = solve(doc, travel={station: travel}, joint_values={sj: -delta})
    hi = solve(doc, travel={station: travel}, joint_values={sj: delta})
    if not (lo["converged"] and hi["converged"]):
        return None
    ub = st["upright_body"]
    rel = cc.compose(hi["transforms"][ub], _inv(lo["transforms"][ub]))
    w = _rotvec(rel[0])
    n = math.sqrt(sum(v * v for v in w))
    if n < 1e-9:
        return None
    u = tuple(v / n for v in w)
    up = _up(doc)
    if u[2] * up < 0:                              # orient the axis upwards
        u = tuple(-v for v in u)
    mid = solve(doc, travel={station: travel}, joint_values={sj: 0.0})
    centre = _wheel_centre(doc, st, mid["transforms"])
    side = 1.0 if centre[1] >= 0 else -1.0
    # a point on the axis: P = t / (I - R) restricted to the plane normal to u; use the point nearest the wheel centre
    R, t = rel
    A = [[(1.0 if i == j else 0.0) - R[i][j] + u[i] * u[j] for j in range(3)] for i in range(3)]
    q = _solve_linear(A, list(t))
    d = cc._sub(centre, tuple(q))
    k = sum(d[i] * u[i] for i in range(3))
    foot = tuple(q[i] + k * u[i] for i in range(3))                    # point of the axis level with the wheel centre
    z_ground = centre[2] - up * _tyre_radius(st)
    sc = (z_ground - foot[2]) / u[2] if abs(u[2]) > 1e-9 else 0.0
    ground = tuple(foot[i] + sc * u[i] for i in range(3))              # where the axis meets the ground
    return {"caster_deg": math.degrees(math.atan2(-u[0], up * u[2])),
            "kpi_deg": math.degrees(math.atan2(-u[1] * side, up * u[2])),
            "scrub_radius": side * (centre[1] - ground[1]),
            "caster_trail": ground[0] - centre[0]}


def roll_center(doc, station, delta=0.005):
    """Front-view roll centre of one wheel station from the instant centre of its upright.

    The upright's motion between wheel travel -delta and +delta is a rotation about x and a translation in the
    y-z plane; its instant centre IC solves (I - R) IC = t. The line from the tyre contact point through IC
    meets the centreline (y = 0) at the roll centre. Returns {'height': m above the ground (positive up),
    'instant_centre': [y, z], 'contact': [y, z]} or None (parallel motion, or the mechanism does not close)."""
    st = _station_map(doc).get(station)
    if not st:
        return None
    lo, hi = solve(doc, travel={station: -delta}), solve(doc, travel={station: delta})
    if not (lo["converged"] and hi["converged"]):
        return None
    ub = st["upright_body"]
    mid = solve(doc)
    rel = cc.compose(hi["transforms"][ub], _inv(lo["transforms"][ub]))
    R, t = rel
    th = math.atan2(R[2][1], R[1][1])                          # rotation about x (the y-z plane angle)
    if abs(th) < 1e-9:
        return None
    c, sn = math.cos(th), math.sin(th)
    det = (1 - c) ** 2 + sn ** 2
    ty, tz = t[1], t[2]
    icy = ((1 - c) * ty - sn * tz) / det * 1.0
    icz = (sn * ty + (1 - c) * tz) / det * 1.0
    centre = _wheel_centre(doc, st, mid["transforms"])
    up = _up(doc)
    contact = (centre[1], centre[2] - up * _tyre_radius(st))
    dy, dz = icy - contact[0], icz - contact[1]
    if abs(dy) < 1e-9:
        return None
    k = -contact[0] / dy
    z_rc = contact[1] + k * dz
    return {"height": up * (z_rc - contact[1]), "instant_centre": [icy, icz], "contact": list(contact)}


def _inv(t):
    Rt = cc._tr(t[0])
    return Rt, tuple(-v for v in cc._mv(Rt, t[1]))


def _tyre_radius(st):
    w = st.get("wheel") or {}
    for k in ("loaded_radius", "rolling_radius", "radius"):
        if isinstance(w.get(k), (int, float)):
            return float(w[k])
    t = st.get("tire") or {}
    for k in ("loaded_radius", "radius"):
        if isinstance(t.get(k), (int, float)):
            return float(t[k])
    return 0.3


def gains(doc, station, delta=0.01):
    """Camber gain (deg per m of bump) and bump steer (deg of toe per m of bump) by central difference around the reference."""
    lo, hi = sweep(doc, station, [-delta, delta])
    if not (lo["converged"] and hi["converged"]) or "toe_deg" not in lo:
        return None
    return {"camber_gain_deg_per_m": (hi["camber_deg"] - lo["camber_deg"]) / (2 * delta),
            "bump_steer_deg_per_m": (hi["toe_deg"] - lo["toe_deg"]) / (2 * delta)}


def check_alignment(doc, tol_angle_deg=0.5, tol_length=0.01):
    """Warnings where the alignment a station declares (`wheel_stations[].alignment`, rad / m, relative to the
    centreline) differs from what the geometry gives at the reference pose. Stations that do not close, or that
    declare nothing, are skipped. Declared values flagged `_est` are still compared."""
    warnings = []
    if (doc.get("_metadata") or {}).get("alignment_convention", "relative_to_centerline") != "relative_to_centerline":
        return warnings
    for st in doc.get("wheel_stations") or []:
        al, sid = st.get("alignment"), st.get("id")
        if not isinstance(al, dict) or not sid:
            continue
        r = solve(doc)
        if not r["converged"]:
            warnings.append(f"alignment {sid}: the mechanism does not close in the reference pose (residual {r['residual']:.1e}); not compared")
            continue
        got = alignment(doc, sid, r["transforms"]) or {}
        for key, name in (("camber", "camber_deg"), ("toe", "toe_deg")):
            if isinstance(al.get(key), (int, float)) and name in got and abs(math.degrees(al[key]) - got[name]) > tol_angle_deg:
                warnings.append(f"alignment {sid}: declared {key} {math.degrees(al[key]):+.2f} deg, geometry gives {got[name]:+.2f} deg")
        ax = steering_axis(doc, sid) if any(k in al for k in ("caster", "kpi", "scrub_radius", "caster_trail")) else None
        if ax:
            for key, name in (("caster", "caster_deg"), ("kpi", "kpi_deg")):
                if isinstance(al.get(key), (int, float)) and abs(math.degrees(al[key]) - ax[name]) > tol_angle_deg:
                    warnings.append(f"alignment {sid}: declared {key} {math.degrees(al[key]):+.2f} deg, geometry gives {ax[name]:+.2f} deg")
            for key in ("scrub_radius", "caster_trail"):
                if isinstance(al.get(key), (int, float)) and abs(al[key] - ax[key]) > tol_length:
                    warnings.append(f"alignment {sid}: declared {key} {al[key] * 1000:+.0f} mm, geometry gives {ax[key] * 1000:+.0f} mm")
    return warnings


# ── command line ──────────────────────────────────────────────────────────────
def main(argv):
    import argparse
    ap = argparse.ArgumentParser(description="SVJ loop-closure solver: wheel travel sweeps")
    ap.add_argument("file")
    ap.add_argument("--station")
    ap.add_argument("--range", type=float, default=0.05, help="travel limit in m (sweeps -range..+range)")
    ap.add_argument("--steps", type=int, default=5)
    ap.add_argument("--steer", type=float, help="steering joint coordinate to hold during the sweep")
    ap.add_argument("--check-alignment", action="store_true", help="compare the declared alignment with the geometry")
    a = ap.parse_args(argv)
    with open(a.file, encoding="utf-8") as f:
        doc = json.load(f)
    if a.check_alignment:
        ws = check_alignment(doc)
        for w in ws:
            print("  WARN ", w)
        print("alignment matches the geometry" if not ws else f"{len(ws)} alignment difference(s)")
        return 0
    stations = [a.station] if a.station else [s.get("id") for s in doc.get("wheel_stations") or []]
    n = max(2, a.steps)
    travels = [-a.range + 2 * a.range * i / (n - 1) for i in range(n)]
    bad = 0
    for sid in stations:
        print(f"{sid}:")
        try:
            rows = sweep(doc, sid, travels, steer=a.steer)
        except SolveError as e:
            print(f"  {e}")
            bad += 1
            continue
        for r in rows:
            flag = "" if r["converged"] else "  NOT CLOSED"
            bad += 0 if r["converged"] else 1
            cam, toe = r.get("camber_deg"), r.get("toe_deg")
            ang = f"camber {cam:+7.3f}  toe {toe:+7.3f}" if cam is not None else "no wheel hinge"
            print(f"  travel {r['travel'] * 1000:+7.1f} mm  {ang}  residual {r['residual']:.1e}{flag}")
        ax = steering_axis(doc, sid)
        if ax:
            print(f"  caster {ax['caster_deg']:+.2f} deg  kpi {ax['kpi_deg']:+.2f} deg  scrub radius {ax['scrub_radius'] * 1000:+.1f} mm  caster trail {ax['caster_trail'] * 1000:+.1f} mm")
        g = gains(doc, sid)
        if g:
            print(f"  camber gain {g['camber_gain_deg_per_m']:+.2f} deg/m   bump steer {g['bump_steer_deg_per_m']:+.2f} deg/m")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
