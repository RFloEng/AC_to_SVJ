"""
Reader for Assetto Corsa ``.ksanim`` files (``<car>/animations/*.ksanim``).

Layout (little endian)::

    int32 version (2)   int32 node_count
    per node:  int32 name_len, name, int32 frame_count,
               frame_count x 10 float32  =  quaternion x y z w, position x y z, scale x y z

A track holds the node's *local* transform (relative to its parent) for each frame, in
the KN5's own axes (left-handed, Y up).  For doors, flaps and wipers frame 0 is the
node's rest pose in the KN5 and the node's origin is the pivot, so a hinge is a pure
rotation of that node about an axis through its origin.  Nothing here raises on a bad
file: unreadable files give an empty result.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np

ANIM_FPS = 30.0            # the files carry no frame rate; 30 fps is the usual AC rate
_FRAME_FLOATS = 10
_DRIVER_PREFIX = "DRIVER:"  # the driver rig animations: bones of a model the car file lacks


@dataclass
class Track:
    quat: np.ndarray        # (n, 4) x, y, z, w
    pos: np.ndarray         # (n, 3)
    scale: np.ndarray       # (n, 3)

    @property
    def frames(self) -> int:
        return len(self.quat)


def read_ksanim(path: Path) -> dict[str, Track]:
    """``{node name: Track}`` for one file; ``{}`` when it is not a readable ksanim."""
    try:
        d = Path(path).read_bytes()
        pos = 0

        def i32() -> int:
            nonlocal pos
            v = int.from_bytes(d[pos:pos + 4], "little", signed=True)
            pos += 4
            return v

        _version, n = i32(), i32()
        if not 0 <= n <= 5000:
            return {}
        out: dict[str, Track] = {}
        for _ in range(n):
            ln = i32()
            if not 0 < ln < 256:
                return {}
            name = d[pos:pos + ln].decode("latin1")
            pos += ln
            frames = i32()
            nbytes = frames * _FRAME_FLOATS * 4
            if not 0 < frames < 100000 or pos + nbytes > len(d):
                return {}
            a = np.frombuffer(d, dtype="<f4", count=frames * _FRAME_FLOATS,
                              offset=pos).reshape(frames, _FRAME_FLOATS).astype("f8")
            pos += nbytes
            out[name] = Track(quat=a[:, 0:4].copy(), pos=a[:, 4:7].copy(),
                              scale=a[:, 7:10].copy())
        return out if pos == len(d) else {}
    except Exception:                                          # noqa: BLE001
        return {}


def track_moves(t: Track, tol: float = 1e-5) -> bool:
    """True when any component of the track changes over its frames."""
    return bool(np.ptp(t.quat, axis=0).max() > tol or np.ptp(t.pos, axis=0).max() > tol
                or np.ptp(t.scale, axis=0).max() > tol)


def load_car_animations(car_dir: Path) -> dict[str, dict[str, Track]]:
    """``{animation name: {node name: Track}}`` for a car folder's ``animations/*.ksanim``.

    Driver-rig bones and nodes that never move are left out; an animation with nothing
    left is dropped.
    """
    out: dict[str, dict[str, Track]] = {}
    adir = Path(car_dir) / "animations"
    try:
        files = sorted(adir.glob("*.ksanim"))
    except OSError:
        return out
    for f in files:
        nodes = {n: t for n, t in read_ksanim(f).items()
                 if not n.startswith(_DRIVER_PREFIX) and track_moves(t)}
        if nodes:
            out[f.stem] = nodes
    return out


# -- quaternion helpers (x, y, z, w) ------------------------------------------

def q_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    x1, y1, z1, w1 = np.moveaxis(a, -1, 0)
    x2, y2, z2, w2 = np.moveaxis(b, -1, 0)
    return np.stack([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
                     w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2], axis=-1)


def q_conj(q: np.ndarray) -> np.ndarray:
    return q * np.array([-1.0, -1.0, -1.0, 1.0])


def q_axis_angle(q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Unit axes (n, 3) and angles (n,) in [0, pi] of quaternions (n, 4)."""
    q = q / np.linalg.norm(q, axis=-1, keepdims=True)
    q = np.where(q[..., 3:4] < 0, -q, q)
    s = np.linalg.norm(q[..., :3], axis=-1)
    angle = 2.0 * np.arctan2(s, q[..., 3])
    axis = np.where(s[..., None] > 1e-9, q[..., :3] / np.maximum(s, 1e-12)[..., None],
                    np.array([1.0, 0.0, 0.0]))
    return axis, angle


def joint_from_track(t: Track, tol_angle: float = math.radians(0.05),
                     tol_pos: float = 1e-4) -> Optional[dict]:
    """Describe a track as a joint between the node and its parent, or None if still.

    ``type``: ``revolute`` (fixed pivot, rotation about one axis), ``prismatic`` (no
    rotation, motion along one line) or ``general``.  Axes and the pivot are in the
    parent node's frame (KN5 axes); ``axis_local`` is the axis in the node's own frame.
    The range runs from frame 0 (the rest pose) to the frame farthest from it.
    """
    if t.frames < 2 or not track_moves(t):
        return None
    rel = q_mul(t.quat, q_conj(t.quat[0:1]))                 # rotation since frame 0
    axes, angles = q_axis_angle(rel)
    dpos = t.pos - t.pos[0]
    travel = np.linalg.norm(dpos, axis=1)
    pivot_fixed = bool(travel.max() <= tol_pos)
    scale_fixed = bool(np.ptp(t.scale, axis=0).max() <= 1e-5)
    k = int(np.argmax(angles))
    rotates = bool(angles.max() > tol_angle)
    info: dict = {"frames": t.frames, "pivot": [round(float(v), 5) for v in t.pos[0]]}
    if rotates and pivot_fixed and scale_fixed:
        ax = axes[k]
        # every frame must turn about that one axis, else the part is not a simple hinge
        turned = angles > math.radians(1.0)
        if turned.any() and float(np.abs(axes[turned] @ ax).min()) < math.cos(math.radians(2.0)):
            info["type"] = "general"
            return info
        ax_local = q_axis_angle(q_mul(q_conj(t.quat[0:1]), t.quat[k:k + 1]))[0][0]
        info.update(type="revolute", angle_rad=round(float(angles[k]), 5),
                    axis=[round(float(v), 5) for v in ax],
                    axis_local=[round(float(v), 5) for v in ax_local])
    elif (not rotates) and scale_fixed and travel.max() > tol_pos:
        d = dpos[int(np.argmax(travel))]
        d = d / np.linalg.norm(d)
        off_line = np.linalg.norm(dpos - np.outer(dpos @ d, d), axis=1).max()
        if off_line <= tol_pos * 10:
            info.update(type="prismatic", travel_m=round(float(travel.max()), 5),
                        axis=[round(float(v), 5) for v in d])
        else:
            info["type"] = "general"
    else:
        info["type"] = "general"
    return info
