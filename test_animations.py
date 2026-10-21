"""Tests for .ksanim reading, glTF animation export and the moving-parts description
(synthetic files only, no AC content)."""
from __future__ import annotations

import math
import struct
import sys
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))

import kn5_reader as K
import ksanim as A
import test_kn5_export as T


def _ksanim(path: Path, nodes: dict) -> Path:
    """nodes: {name: (n_frames, 10 floats per frame list)}"""
    out = struct.pack("<ii", 2, len(nodes))
    for name, frames in nodes.items():
        b = name.encode()
        out += struct.pack("<i", len(b)) + b + struct.pack("<i", len(frames))
        for f in frames:
            out += struct.pack("<10f", *f)
    path.write_bytes(out)
    return path


def _rot_y(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _qx(deg):
    h = math.radians(deg) / 2
    return np.array([math.sin(h), 0.0, 0.0, math.cos(h)])


def _door(n=20, open_deg=70.0, pivot=(0.4, 0.8, 0.2)):
    """A node whose rest rotation is 30 deg about Y, opening about its own X axis."""
    R0 = _rot_y(30)
    q0 = K._rot_to_quat(R0)
    frames = []
    for i in range(n):
        q = A.q_mul(q0, _qx(open_deg * i / (n - 1)))
        frames.append([*q, *pivot, 1, 1, 1])
    # KN5 matrices are row-major with the rotation transposed, translation in row 3
    m = np.eye(4)
    m[:3, :3] = R0.T
    m[3, :3] = pivot
    return frames, [float(v) for v in m.flatten()]


def test_ksanim_roundtrip_and_bad_files():
    frames, _ = _door()
    with tempfile.TemporaryDirectory() as t:
        p = _ksanim(Path(t) / "d.ksanim", {"DOOR": frames})
        tr = A.read_ksanim(p)["DOOR"]
        assert tr.frames == 20 and np.allclose(tr.pos[0], (0.4, 0.8, 0.2), atol=1e-6)
        (Path(t) / "bad.ksanim").write_bytes(b"\x02\x00\x00\x00\x05\x00\x00\x00junk")
        assert A.read_ksanim(Path(t) / "bad.ksanim") == {}
        assert A.read_ksanim(Path(t) / "missing.ksanim") == {}
        (Path(t) / "trunc.ksanim").write_bytes(p.read_bytes()[:-7])
        assert A.read_ksanim(Path(t) / "trunc.ksanim") == {}


def test_car_animations_skip_driver_rig_and_static_nodes():
    frames, _ = _door()
    still = [[0, 0, 0, 1, 0, 0, 0, 1, 1, 1]] * 5
    with tempfile.TemporaryDirectory() as t:
        car = Path(t)
        (car / "animations").mkdir()
        _ksanim(car / "animations" / "door.ksanim",
                {"DOOR": frames, "DRIVER:RIG_Arm": frames, "STILL": still})
        _ksanim(car / "animations" / "nothing.ksanim", {"STILL": still})
        got = A.load_car_animations(car)
        assert set(got) == {"door"} and set(got["door"]) == {"DOOR"}
        assert A.load_car_animations(car / "nope") == {}


def test_joint_from_track_hinge_slider_and_general():
    frames, _ = _door(open_deg=70.0)
    tr = A.Track(*[np.array(frames)[:, a:b] for a, b in ((0, 4), (4, 7), (7, 10))])
    j = A.joint_from_track(tr)
    assert j["type"] == "revolute" and abs(math.degrees(j["angle_rad"]) - 70.0) < 0.01
    assert np.allclose(j["axis_local"], (1, 0, 0), atol=1e-4)
    # parent-frame axis = the node's X axis = first column of Ry(30)
    assert np.allclose(j["axis"], _rot_y(30)[:, 0], atol=1e-4)
    assert j["pivot"] == [0.4, 0.8, 0.2]
    # slider
    n = 10
    quat = np.tile([0, 0, 0, 1.0], (n, 1))
    pos = np.array([[0, 0.1 * i, 0] for i in range(n)])
    sl = A.joint_from_track(A.Track(quat, pos, np.ones((n, 3))))
    assert sl["type"] == "prismatic" and abs(sl["travel_m"] - 0.9) < 1e-4
    # two different axes in turn: not a simple hinge
    q = np.array([A.q_mul(_qx(10 * i), np.array([0, math.sin(math.radians(5 * i)), 0,
                                                    math.cos(math.radians(5 * i))]))
                  for i in range(n)])
    assert A.joint_from_track(A.Track(q, np.zeros((n, 3)), np.ones((n, 3))))["type"] == "general"
    assert A.joint_from_track(A.Track(quat[:1], pos[:1], np.ones((1, 3)))) is None


def _car_with_door(t: Path):
    frames, m = _door()
    # parent shifted 1 m along X: the model-frame pivot must include it
    shift = list(T.IDENTITY)
    shift[12] = 1.0
    root = T.N("ROOT", children=[T.N("PARENT", matrix=shift, children=[
        T.N("DOOR", matrix=m, children=[T.N("g_door", "mesh")])])])
    car = Path(t) / "car"
    T._write(car / "car.kn5", root)
    (car / "animations").mkdir()
    _ksanim(car / "animations" / "car_door.ksanim", {"DOOR": frames})
    return car


def test_glb_gets_animations_and_matching_rest_pose():
    import pygltflib
    with tempfile.TemporaryDirectory() as t:
        car = _car_with_door(Path(t))
        rep: dict = {}
        out = K.kn5_all_lods_to_glbs(car, Path(t) / "o", include_skins=False, reports=rep)
        g = pygltflib.GLTF2().load_binary(str(out["A"]))
        assert [a.name for a in g.animations] == ["car_door"]
        assert rep["A"]["animations"] == {"car_door": ["DOOR"]}
        node = next(n for n in g.nodes if n.name == "DOOR")
        assert node.matrix is None and node.rotation and node.translation   # TRS now
        blob = g.binary_blob()

        def acc(i):
            a = g.accessors[i]
            bv = g.bufferViews[a.bufferView]
            k = {"SCALAR": 1, "VEC3": 3, "VEC4": 4}[a.type]
            return np.frombuffer(blob, "<f4", a.count * k, bv.byteOffset).reshape(a.count, k)
        an = g.animations[0]
        ch = next(c for c in an.channels if c.target.path == "rotation")
        smp = an.samplers[ch.sampler]
        times, q = acc(smp.input), acc(smp.output).astype("f8")
        assert len(times) == 20 and abs(times[-1, 0] - 19 / 30) < 1e-5
        assert abs(abs(q[0] @ np.array(node.rotation)) - 1) < 1e-5        # frame 0 = rest
        rel = A.q_mul(q[-1:], A.q_conj(q[0:1]))
        axis, ang = A.q_axis_angle(rel)
        assert abs(math.degrees(ang[0]) - 70.0) < 0.05
        # AC (x, y, z) -> glTF (x, -y, -z): the glTF axis is the AC axis with Y, Z negated
        ac_axis = _rot_y(30)[:, 0]
        assert np.allclose(axis[0], ac_axis * np.array([1, -1, -1]), atol=1e-4)
        # no translation channel: the pivot is fixed
        assert {c.target.path for c in an.channels} == {"rotation"}
        # opting out leaves the GLB without animations and the node with its matrix
        out2 = K.kn5_all_lods_to_glbs(car, Path(t) / "o2", include_skins=False,
                                      include_animations=False)
        g2 = pygltflib.GLTF2().load_binary(str(out2["A"]))
        assert not g2.animations
        assert next(n for n in g2.nodes if n.name == "DOOR").matrix is not None


def test_moving_parts_description():
    with tempfile.TemporaryDirectory() as t:
        car = _car_with_door(Path(t))
        parts = K.describe_moving_parts(car / "car.kn5", car)
        assert len(parts) == 1
        p = parts[0]
        assert p["node"] == "DOOR" and p["parent"] == "PARENT" and p["joint"] == "revolute"
        assert p["animation"] == "car_door" and p["fps"] == 30.0
        assert abs(math.degrees(p["angle_rad"]) - 70.0) < 0.01
        assert p["pivot"] == [0.4, 0.8, 0.2]
        assert np.allclose(p["pivot_model"], [1.4, 0.8, 0.2], atol=1e-4)     # parent +1 m in X
        assert np.allclose(p["axis_model"], _rot_y(30)[:, 0], atol=1e-4)
        assert K.describe_moving_parts(car / "car.kn5", Path(t) / "no_car") == []


def test_movable_parts_become_svj_bodies_joints_and_poses():
    import converter as C
    with tempfile.TemporaryDirectory() as t:
        car = _car_with_door(Path(t))
        pl = {"translation": (0.0, 0.5, -1.0), "wheel_centers": {}, "pitch_deg": 0.0}
        parts = K.describe_moving_parts(car / "car.kn5", car, placement=pl)
        m = parts[0]
        # KN5 model (X left, Y up, Z forward) -> SVJ (X forward, Y right, Z down)
        assert np.allclose(m["pivot_svj"], [0.2 - 1.0, -1.4, -(0.8 + 0.5)], atol=1e-4)
        ax = _rot_y(30)[:, 0]
        assert np.allclose(m["axis_svj"], [ax[2], -ax[0], -ax[1]], atol=1e-4)
        assert "pivot_svj" not in K.describe_moving_parts(car / "car.kn5", car)[0]

        svj = {"chassis": {}}
        nb, nj, ren = C.movable_parts_to_svj(svj, parts, "car_mesh")
        assert (nb, nj) == (1, 1) and ren == {"DOOR": "SVJ::body::door"}
        body = svj["chassis"]["mass_bodies"][0]
        assert body["id"] == "door" and body["mass"] == 0 and body["x_derived"] is True
        assert body["visual"] == {"mesh_ref": "car_mesh", "node": "SVJ::body::door"}
        mk = svj["chassis"]["markers"]["door_hinge"]
        assert mk == body["markers"]["hinge"] and mk["position"] == m["pivot_svj"]
        # the marker's +Z axis is the hinge axis
        x, y, z, w = mk["orientation"]
        zax = [2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y)]
        assert np.allclose(zax, m["axis_svj"], atol=1e-5)
        j = svj["constraints"][0]
        assert (j["id"], j["joint_type"], j["body_a"], j["body_b"]) == (
            "hinge_door", "revolute", "chassis", "door")
        assert abs(j["range"]["max"] - math.radians(70.0)) < 1e-4 and j["range"]["min"] == 0.0
        assert svj["poses"][0] == {"id": "closed"}
        assert svj["poses"][1] == {"id": "car_door", "values": {"hinge_door": j["range"]["max"]}}
        # suspension travel and already bound nodes are not movable parts
        for nm, kw in (("HUB_LF", {}), ("Anim_Susp0", {}), ("DOOR", {"bound": {"DOOR"}})):
            sv = {"chassis": {}}
            assert C.movable_parts_to_svj(sv, [dict(m, node=nm)], "m", **kw)[0] == 0
        # nothing usable: the SVJ is left as it was
        bare = {"chassis": {}}
        assert C.movable_parts_to_svj(bare, [], "m") == (0, 0, {}) and bare == {"chassis": {}}


def test_part_ids_put_the_side_last_and_stay_unique():
    import converter as C
    taken: set = set()
    assert C._part_id("DOOR_LF", taken) == "door_fl"
    assert C._part_id("Door-LF", taken) == "door_fl_2"
    assert C._part_id("WIPER_R", taken) == "wiper_r"
    assert C._part_id("3d_hood", taken) == "part_3d_hood"
    assert C._part_id("__", taken) == "part"


ALL_TESTS = [v for k, v in sorted(globals().items()) if k.startswith("test_")]


def run() -> bool:
    ok = True
    for fn in ALL_TESTS:
        try:
            fn()
            print(f"  ✓  {fn.__name__}")
        except Exception as e:                    # noqa: BLE001
            ok = False
            print(f"  ✗  {fn.__name__}: {type(e).__name__}: {e}")
    return ok


if __name__ == "__main__":
    sys.exit(0 if run() else 1)
