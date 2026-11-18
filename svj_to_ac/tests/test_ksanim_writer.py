"""SVJ -> AC: the movable parts of an SVJ become animations/*.ksanim that the forward converter's reader reads back."""
from __future__ import annotations

import math
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "svj_to_ac"))

import ksanim as KA                                   # noqa: E402
import kn5_writer as kw                               # noqa: E402
import ksanim_writer as W                             # noqa: E402
import mesh_scene as ms                               # noqa: E402

CG = (-1.0, 0.0, -0.4)                                # SAE: 1 m behind the front axle, 0.4 m up


def _doc(extra_poses=None, ac_animation=None):
    parts = [{"node": "DOOR_L", "animation": ac_animation}] if ac_animation else []
    return SimpleNamespace(raw={
        "chassis": {
            "mass_bodies": [{"id": "door_fl", "name": "DOOR_L", "category": "attachment",
                             "visual": {"node": "SVJ::body::door_fl"}},
                            {"id": "geo_arrow_rpm", "category": "attachment", "x_instrument": {"ac_object": "X"}}],
            "markers": {"door_fl_hinge": {"position": [0.5, 0.8, -0.5], "orientation": [0, 0, 0, 1]},     # axis +Z (down)
                        "needle": {"position": [0, 0, 0], "orientation": [0, 0, 0, 1]}}},
        "constraints": [{"id": "hinge_door_fl", "type": "joint", "joint_type": "revolute", "body_a": "chassis",
                         "marker_a": "door_fl_hinge", "body_b": "door_fl", "marker_b": "hinge", "range": {"min": 0, "max": 1.2}},
                        {"id": "hinge_geo_arrow_rpm", "type": "joint", "joint_type": "revolute", "body_a": "chassis",
                         "marker_a": "needle", "body_b": "geo_arrow_rpm"}],
        "poses": [{"id": "closed"}, {"id": "door_fl_open", "values": {"hinge_door_fl": math.pi / 2}},
                  {"id": "gauge_rpm", "values": {"hinge_geo_arrow_rpm": 3.0}}] + (extra_poses or []),
        "x_assettocorsa": {"moving_parts": {"parts": parts}}})


def _scene():
    m = np.eye(4)
    m[:3, 3] = (0.8, 0.5, -0.5)                       # the hinge point in the glTF asset frame (X right, Y up, -Z forward)
    quad = ms.SceneMesh(np.array([[0, 0, 0], [0, 0, -1], [0, 1, 0]], np.float32), None, None,
                        np.array([0, 1, 2], np.uint32), ms.SceneMaterial("m"), "q")
    return ms.SceneNode("root", children=[ms.SceneNode("SVJ::body::door_fl", matrix=m, meshes=[quad])])


def test_only_real_movable_parts_count() -> None:
    parts = W.movable_parts(_doc())
    assert [p["joint"] for p in parts] == ["hinge_door_fl"]                  # the needle is left to analog_instruments.ini
    assert parts[0]["animation"] == "car_door_fl_open" and parts[0]["value"] == pytest.approx(math.pi / 2)
    assert W.node_names(_doc()) == {}                                         # not from AC: the id names the node
    assert W.node_names(_doc(ac_animation="car_door_L")) == {"SVJ::body::door_fl": "DOOR_L"}


def test_animation_turns_the_node_about_the_pivot(tmp_path) -> None:
    doc = _doc(ac_animation="car_door_L")
    data, rep = kw.build_kn5(_scene(), "car", 0.4, 1.0, names=W.node_names(doc), record={"SVJ::body::door_fl"})
    files, notes = W.write(doc, rep.frames, tmp_path / "animations", CG)
    assert [f.name for f in files] == ["car_door_L.ksanim"]
    tr = KA.load_car_animations(tmp_path)["car_door_L"]["DOOR_L"]
    assert tr.frames >= 20
    # hinge point (SAE 0.5, 0.8, -0.5) -> kn5 (-0.8, -(-0.5 + 0.4), 0.5 + 1.0) = (-0.8, 0.1, 1.5); axis SAE +Z -> kn5 (0, -1, 0)
    pivot = np.array([-0.8, 0.1, 1.5])
    assert tr.pos[0] == pytest.approx(pivot, abs=1e-5) and tr.pos[-1] == pytest.approx(pivot, abs=1e-5)
    R0, R1 = (_mat(q) for q in (tr.quat[0], tr.quat[-1]))
    turn = R1 @ R0.T                                   # the world turn of the last frame (parent is the model root)
    axis = np.array([0.0, -1.0, 0.0])
    assert turn @ axis == pytest.approx(axis, abs=1e-5)                       # the axis stays put
    ang = math.acos(max(-1.0, min(1.0, (np.trace(turn) - 1) / 2)))
    assert ang == pytest.approx(math.pi / 2, abs=1e-4)
    assert turn @ np.array([1.0, 0.0, 0.0]) == pytest.approx([0.0, 0.0, 1.0], abs=1e-5)       # right-handed about the axis


def _mat(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def test_a_wiper_goes_there_and_back(tmp_path) -> None:
    doc = _doc()
    doc.raw["poses"].append({"id": "wiper", "values": {"hinge_door_fl": 0.5}})
    parts = W.movable_parts(doc)
    parts[0]["animation"] = "car_wiper"
    _, rep = kw.build_kn5(_scene(), "car", 0.4, 1.0, record={"SVJ::body::door_fl"})
    fs = W._frames(parts[0], rep.frames["SVJ::body::door_fl"], CG, loop=True)
    assert np.allclose(fs[0, :4], fs[-1, :4], atol=1e-5) and not np.allclose(fs[0, :4], fs[len(fs) // 2, :4], atol=1e-3)


def test_a_part_without_a_mesh_node_is_reported(tmp_path) -> None:
    files, notes = W.write(_doc(), {}, tmp_path, CG)
    assert files == [] and any("no mesh node" in n for n in notes)


# ---- suspension travel ---------------------------------------------------- #


def _susp_doc():
    return SimpleNamespace(raw={
        "chassis": {"mass_bodies": [
            {"id": "upright_fl", "category": "suspension", "visual": {"node": "SVJ::suspension::upright_fl"}},
            {"id": "arm_fl", "category": "suspension", "visual": {"node": "SVJ::suspension::arm_fl"}},
            {"id": "upright_zz", "category": "suspension"}]},
        "wheel_stations": [{"id": "FL", "upright_body": "upright_fl", "wheel_body": "wheel_fl"}]})


def test_suspension_parts_and_names() -> None:
    parts = W.suspension_parts(_susp_doc())
    assert {p["node"] for p in parts} == {"SVJ::suspension::upright_fl", "SVJ::suspension::arm_fl"}
    assert W.suspension_node_names(_susp_doc()) == {"SVJ::suspension::upright_fl": "SUSP_LF"}


def test_suspension_animation_moves_up_with_the_wheel(tmp_path, monkeypatch) -> None:
    def poses(raw, sid, ts):                          # the wheel rises: SAE z is down, so z decreases
        return {i: {b: (np.eye(3), (0.0, 0.0, -float(t))) for b in ("upright_fl", "arm_fl")} for i, t in enumerate(ts)}
    monkeypatch.setattr(W, "_susp_poses", poses)
    eye = np.eye(4)
    frames = {n: {"name": nm, "world": eye, "parent": eye}
              for n, nm in (("SVJ::suspension::upright_fl", "SUSP_LF"), ("SVJ::suspension::arm_fl", "ARM_LF"))}
    files, notes = W.write_suspension(_susp_doc(), frames, tmp_path, CG)
    assert [f.name for f in files] == ["car_SUSP_LF.ksanim"] and notes
    tracks = KA.read_ksanim(files[0])
    y = np.asarray(tracks["SUSP_LF"].pos)[:, 1]
    assert len(y) == W.SUSP_FRAMES and y[0] == pytest.approx(-W.SUSP_TRAVEL_M, abs=1e-5)
    assert y[-1] == pytest.approx(W.SUSP_TRAVEL_M, abs=1e-5) and set(tracks) == {"SUSP_LF", "ARM_LF"}


def test_a_corner_that_does_not_close_is_skipped(tmp_path, monkeypatch) -> None:
    def boom(raw, sid, ts):
        raise ValueError("no closure")
    monkeypatch.setattr(W, "_susp_poses", boom)
    eye = np.eye(4)
    frames = {"SVJ::suspension::upright_fl": {"name": "SUSP_LF", "world": eye, "parent": eye}}
    files, notes = W.write_suspension(_susp_doc(), frames, tmp_path, CG)
    assert files == [] and "skipped" in notes[0]
