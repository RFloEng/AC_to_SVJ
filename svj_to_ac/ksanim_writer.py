"""Write Assetto Corsa ``animations/*.ksanim`` for the movable parts of an SVJ (doors, bonnet, boot, wipers ...).

The SVJ describes a movable part as a body hinged (or slid) to the chassis by a joint with a range, and poses
that set the joint values (spec 24).  AC wants, per animation, the local transform of each moving KN5 node at
every frame.  This module turns the one into the other:

* the movable parts are the ``revolute`` / ``translational`` joints from the chassis to a body of category
  ``attachment`` that is not a dashboard needle (those are driven by ``analog_instruments.ini``);
* the pivot and axis come from the joint's marker on the chassis (the axis is its +Z), converted to the KN5's
  space the way the mesh was (``kn5 = (-Y, -(Z - cg_z), X - cg_x)`` from the loader's SAE axes, a proper rotation);
* a frame turns the node's world matrix about that axis through that pivot, by ``value * t`` for ``t`` from 0 to 1,
  and gives the local transform below the node's parent (so any pivot works, not only one at the node's origin);
* an animation called ``*wiper*`` goes there and back (rest, full, rest), as AC's own do; the others go from rest to the
  pose and stay;
* the file is named after the animation the part had when it came from AC (``x_assettocorsa.moving_parts``), else
  ``car_<pose id>``.  Which names the game itself looks for is not known here: a door animation under another
  name will not play from the game's own triggers.

Format (little endian), as ``ksanim.py`` reads it: ``int32 version (2)``, ``int32 nodes``, per node
``int32 name length, name, int32 frames, frames x 10 float32`` (quaternion x y z w, position, scale).
"""
from __future__ import annotations

import math
import struct
from pathlib import Path

import numpy as np

ANIM_FPS = 30.0
_STEP_DEG = 3.0                    # a frame at least every 3 degrees (or 2 cm)
_MAX_FRAMES = 120
_DIR = np.array([[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]])    # SAE direction -> KN5 direction


def _quat_rotate(q, v) -> np.ndarray:
    x, y, z, w = (float(c) for c in q)
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    return R @ np.asarray(v, dtype="f8")


def _rot_to_quat(R: np.ndarray) -> np.ndarray:
    """(x, y, z, w) of a rotation matrix (column-vector convention)."""
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = [(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s]
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        q = [0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s]
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        q = [(R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s]
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        q = [(R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s, (R[1, 0] - R[0, 1]) / s]
    q = np.array(q, dtype="f8")
    return q / np.linalg.norm(q)


def _axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    a = axis / np.linalg.norm(axis)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * (K @ K)


# --------------------------------------------------------------------------- #
# What moves                                                                   #
# --------------------------------------------------------------------------- #


def movable_parts(doc) -> list[dict]:
    """The movable parts of the SVJ: ``joint``, ``body``, ``node`` (the mesh node), ``name`` (the AC name),
    ``kind`` (``revolute`` / ``translational``), ``pivot`` and ``axis`` (loader / SAE axes), ``value`` (the
    largest pose value, rad or m) and ``animation``."""
    raw = doc.raw or {}
    chassis = raw.get("chassis") or {}
    bodies = {b.get("id"): b for b in chassis.get("mass_bodies") or [] if isinstance(b, dict)}
    cmarkers = chassis.get("markers") or {}
    ac_parts = {p.get("node"): p for p in ((raw.get("x_assettocorsa") or {}).get("moving_parts") or {}).get("parts") or []}
    joints = {}
    for c in raw.get("constraints") or []:
        if not isinstance(c, dict) or c.get("type") != "joint" or c.get("joint_type") not in ("revolute", "translational"):
            continue
        b = bodies.get(c.get("body_b"))
        if (c.get("body_a") != "chassis" or not b or b.get("category") != "attachment" or b.get("x_instrument")):
            continue
        fr = cmarkers.get(c.get("marker_a")) or {}
        if "position" not in fr:
            continue
        axis = _quat_rotate(fr.get("orientation") or [0, 0, 0, 1], [0, 0, 1])
        joints[c["id"]] = {"joint": c["id"], "body": b["id"], "kind": c["joint_type"], "pivot": np.array(fr["position"], dtype="f8"),
                           "axis": axis, "range": c.get("range") or {}}
    out: dict = {}
    for pose in raw.get("poses") or []:
        for jid, val in (pose.get("values") or {}).items():
            j = joints.get(jid)
            if not j or not val:
                continue
            body = bodies[j["body"]]
            ac = ac_parts.get(body.get("name"))
            anim = (ac or {}).get("animation") or f"car_{pose.get('id')}"
            cur = out.get(jid)
            if cur is None or abs(val) > abs(cur["value"]):
                out[jid] = {**j, "value": float(val), "animation": anim,
                            "node": (body.get("visual") or {}).get("node") or f"SVJ::body::{body['id']}",
                            "name": body.get("name") if ac else None}
    return list(out.values())


def node_names(doc) -> dict[str, str]:
    """``SVJ::body::<id>`` -> the node name the part had in AC, for the parts that came from AC: its animation
    addresses the node by that name."""
    return {p["node"]: p["name"] for p in movable_parts(doc) if p.get("name")}


# --------------------------------------------------------------------------- #
# The tracks                                                                   #
# --------------------------------------------------------------------------- #


def _to_kn5(point_sae, cg) -> np.ndarray:
    x, y, z = (float(v) for v in point_sae)
    return np.array([-y, -(z - cg[2]), x - cg[0]])


def _frames(part: dict, fr: dict, cg, loop: bool = False) -> np.ndarray:
    """``(n, 10)`` frames of one node: quaternion, position, scale."""
    P = _to_kn5(part["pivot"], cg)
    a = _DIR @ part["axis"]
    a = a / np.linalg.norm(a)
    v = part["value"]
    n = int(min(_MAX_FRAMES, max(2, math.ceil((abs(v) / math.radians(_STEP_DEG)) if part["kind"] == "revolute"
                                                else abs(v) / 0.02) + 1)))
    if loop:                                                    # there and back: rest, full, rest
        n = max(n, 3) * 2 - 1
    W, pw = fr["world"], fr["parent"]
    inv_pw = np.linalg.inv(pw)
    rows, prev = [], None
    for k in range(n):
        t = k / (n - 1)
        if loop:
            t = 1.0 - abs(2.0 * t - 1.0)
        M = np.eye(4)
        if part["kind"] == "revolute":
            R = _axis_angle(a, v * t)
            M[:3, :3] = R
            M[:3, 3] = P - R @ P
        else:
            M[:3, 3] = a * (v * t)
        loc = inv_pw @ M @ W
        q = _rot_to_quat(loc[:3, :3] / np.cbrt(abs(np.linalg.det(loc[:3, :3])) or 1.0))
        if prev is not None and float(q @ prev) < 0:
            q = -q
        prev = q
        sc = [float(np.linalg.norm(loc[:3, i])) for i in range(3)]
        rows.append([*q, *loc[:3, 3], *sc])
    return np.array(rows, dtype="f4")


def write(doc, frames: dict, out_dir: Path, cg) -> tuple[list[Path], list[str]]:
    """``animations/<name>.ksanim`` for each animation of the movable parts whose nodes were written (``frames``
    is ``kn5_writer.Report.frames``).  Returns ``(files, notes)``."""
    out_dir = Path(out_dir)
    by_anim: dict = {}
    notes: list[str] = []
    for p in movable_parts(doc):
        fr = frames.get(p["node"])
        if fr is None:
            notes.append(f"{p['body']}: no mesh node {p['node']}, no animation written")
            continue
        by_anim.setdefault(p["animation"], []).append((p, fr))
    files: list[Path] = []
    for anim, items in by_anim.items():
        blob = struct.pack("<ii", 2, len(items))
        for p, fr in items:
            nm = fr["name"].encode("utf-8")
            fs = _frames(p, fr, cg, loop="wiper" in anim.lower())
            blob += struct.pack("<i", len(nm)) + nm + struct.pack("<i", len(fs)) + fs.astype("<f4").tobytes()
        out_dir.mkdir(parents=True, exist_ok=True)
        path = out_dir / f"{anim}.ksanim"
        path.write_bytes(blob)
        files.append(path)
    if files:
        notes.append(f"{len(files)} animation(s) written ({', '.join(f.stem for f in files)}); the game plays them by name, "
                     "and the names are those the part had in AC or car_<pose>")
    return files, notes


# --------------------------------------------------------------------------- #
# Suspension travel: SUSP_xx nodes and animations/car_SUSP_xx.ksanim           #
# --------------------------------------------------------------------------- #
#
# Kunos' cars carry ``animations/car_SUSP_LF.ksanim``: 100 frames of the nodes that move with the wheel (hub, arms),
# the first with the wheel at its lowest, the last at its highest, the middle one the static pose (about +-0.04 m).
# Here the bodies of a corner's mechanism that are bound to a mesh node (the upright through the station's
# ``visual``, arms and links through their own ``visual.node``) are moved by the loop-closure solver at 100 wheel
# travels and each node's local transform is written.  The upright node is named ``SUSP_<corner>``.

SUSP_TRAVEL_M = 0.04
SUSP_FRAMES = 100
_CORNER = {"fl": "LF", "fr": "RF", "rl": "LR", "rr": "RR"}


def _pose_kn5(R, t, cg) -> np.ndarray:
    """The rigid motion ``p' = R p + t`` of the loader's SAE axes as a 4x4 in the KN5's space."""
    c = np.array([0.0, cg[2], -cg[0]])
    Rk = _DIR @ np.array(R, dtype="f8") @ _DIR.T
    M = np.eye(4)
    M[:3, :3] = Rk
    M[:3, 3] = c - Rk @ c + _DIR @ np.array(t, dtype="f8")
    return M


def suspension_parts(doc) -> list[dict]:
    """Mesh nodes that follow a wheel station: ``station``, ``corner`` (LF/RF/LR/RR), ``body``, ``node`` (``SVJ::...``)
    and ``name`` (``SUSP_<corner>`` for the upright, else None)."""
    raw = doc.raw or {}
    bodies = {b.get("id"): b for b in (raw.get("chassis") or {}).get("mass_bodies") or [] if isinstance(b, dict)}
    out = []
    for st in raw.get("wheel_stations") or []:
        sid = str(st.get("id") or "")
        corner = _CORNER.get(sid.lower())
        up = st.get("upright_body")
        if not corner or up not in bodies:
            continue
        node = (st.get("visual") or {}).get("node") or (bodies[up].get("visual") or {}).get("node")
        if node:
            out.append({"station": sid, "corner": corner, "body": up, "node": node, "name": f"SUSP_{corner}"})
        for bid, b in bodies.items():
            n2 = (b.get("visual") or {}).get("node")
            if (bid != up and n2 and b.get("category") in ("suspension", "structural")
                    and str(bid).lower().endswith("_" + sid.lower())):
                out.append({"station": sid, "corner": corner, "body": bid, "node": n2, "name": None})
    return out


def suspension_node_names(doc) -> dict[str, str]:
    return {p["node"]: p["name"] for p in suspension_parts(doc) if p["name"]}


def _susp_poses(raw, sid: str, ts) -> dict:
    """``{frame index: transforms}`` for travels ``ts`` (sorted), solved outward from the static pose."""
    from svj_migrate import loop_solver as ls

    poses: dict = {}
    for idxs in ([i for i, t in enumerate(ts) if t >= 0], [i for i, t in enumerate(ts) if t < 0][::-1]):
        prev = None
        for i in idxs:
            r = ls.solve(raw, travel={sid: float(ts[i])}, start=prev)
            if not r["converged"]:
                raise ls.SolveError(f"travel {ts[i]:+.3f} m does not close (residual {r['residual']:.1e})")
            prev = r["transforms"]
            poses[i] = prev
    return poses


def write_suspension(doc, frames: dict, out_dir: Path, cg, travel: float = SUSP_TRAVEL_M, n: int = SUSP_FRAMES
                     ) -> tuple[list[Path], list[str]]:
    """``car_SUSP_<corner>.ksanim`` per corner whose mechanism the solver closes (``frames`` is ``Report.frames``)."""
    notes: list[str] = []
    files: list[Path] = []
    by_station: dict = {}
    for p in suspension_parts(doc):
        if p["node"] in frames:
            by_station.setdefault((p["station"], p["corner"]), []).append(p)
    ts = np.linspace(-travel, travel, n)
    for (sid, corner), items in by_station.items():
        try:
            poses = _susp_poses(doc.raw, sid, ts)
        except Exception as exc:                # noqa: BLE001 (SolveError, a missing joint, an odd file)
            notes.append(f"{sid}: suspension animation skipped: {exc}")
            continue
        tracks = []
        for p in items:
            fr = frames[p["node"]]
            inv_pw = np.linalg.inv(fr["parent"])
            rows, prev = [], None
            for i in range(n):
                tr = poses[i].get(p["body"])
                if tr is None:
                    break
                loc = inv_pw @ _pose_kn5(tr[0], tr[1], cg) @ fr["world"]
                q = _rot_to_quat(loc[:3, :3] / np.cbrt(abs(np.linalg.det(loc[:3, :3])) or 1.0))
                if prev is not None and float(q @ prev) < 0:
                    q = -q
                prev = q
                rows.append([*q, *loc[:3, 3], *[float(np.linalg.norm(loc[:3, k])) for k in range(3)]])
            if len(rows) == n:
                tracks.append((fr["name"], np.array(rows, dtype="<f4")))
        if not tracks:
            continue
        blob = struct.pack("<ii", 2, len(tracks))
        for name, arr in tracks:
            nm = name.encode("utf-8")
            blob += struct.pack("<i", len(nm)) + nm + struct.pack("<i", len(arr)) + arr.tobytes()
        Path(out_dir).mkdir(parents=True, exist_ok=True)
        path = Path(out_dir) / f"car_SUSP_{corner}.ksanim"
        path.write_bytes(blob)
        files.append(path)
    if files:
        notes.append(f"suspension travel written for {len(files)} corner(s) ({n} frames, droop to bump, +-{travel * 1000:.0f} mm "
                     "around the static pose, as Kunos' cars); travel range and node set are assumed, not read from AC")
    return files, notes
