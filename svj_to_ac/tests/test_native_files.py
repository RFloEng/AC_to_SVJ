"""lights.ini, cameras.ini, damage.ini, ai.ini: carried in the SVJ by AC -> SVJ and written back by SVJ -> AC."""
from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "svj_to_ac"))

import ac_native_files as N                                     # noqa: E402
import ac_parsers as P                                          # noqa: E402
from ac_writers import native_files_ini as W                    # noqa: E402

CAM = "[CAMERA_0]\nPOSITION=0,1.425,-0.345\nFORWARD=0,-0.19858,0.97985\nUP=0,0.98008,0.19854\nFOV=60\n"


def _data(tmp_path):
    d = tmp_path / "data"
    d.mkdir()
    (d / "cameras.ini").write_text(CAM)
    (d / "lights.ini").write_text("[HEADER]\nVERSION=3\n[LIGHT_0]\nNAME=LOWBEAM_BULB\nCOLOR=200,200,255\n")
    (d / "ai.ini").write_text("[GEARS]\nUP=7000\n; a comment\nDOWN=5000\n")
    return d


def _doc(entries, cg=(-1.0, 0.0, -0.4)):
    return SimpleNamespace(assetto=SimpleNamespace(raw={"native_files": entries}),
                           chassis=SimpleNamespace(center_of_gravity=list(cg)))


def test_only_the_files_that_exist_are_carried(tmp_path) -> None:
    got = N.collect(_data(tmp_path), translation=None)
    assert set(got) == {"cameras.ini", "lights.ini", "ai.ini"}
    assert got["ai.ini"]["sections"]["GEARS"] == {"UP": "7000", "DOWN": "5000"}


def test_cameras_are_stored_in_iso_axes_and_come_back(tmp_path) -> None:
    T = (0.0, 0.4257, 0.343)                                   # the mesh placement's translation
    got = N.collect(_data(tmp_path), translation=T)
    cam = got["cameras.ini"]
    assert cam["frame"] == "ISO_8855"
    pos = cam["sections"]["CAMERA_0"]["POSITION"]
    assert pos == pytest.approx([-0.345 + 0.343, 0.0, 1.425 + 0.4257], abs=1e-4)           # SAE (z, -x, -y - T1) as the mesh, then ISO
    # a camera 1.425 up, 0.345 behind the model origin: forward is X, up is Z
    assert cam["sections"]["CAMERA_0"]["FORWARD"] == pytest.approx([0.97985, 0.0, -0.19858])
    # out: the CG sits 1 m behind the front axle and 0.4 m up (SAE: x -1, z -0.4): kn5 = (y, z + cg_z, x - cg_x)
    d = tmp_path / "out"
    files = W.write(_doc(got), d)
    assert {f.name for f in files} == {"cameras.ini", "lights.ini", "ai.ini"}
    back = P.parse_ini((d / "cameras.ini").read_text())["CAMERA_0"]
    p = [float(x) for x in back["POSITION"].split(",")]
    iso = pos
    assert p == pytest.approx([iso[1], iso[2] - 0.4, iso[0] + 1.0], abs=1e-4)
    assert [float(x) for x in back["FORWARD"].split(",")] == pytest.approx([0.0, -0.19858, 0.97985], abs=1e-4)
    assert back["FOV"] == "60"


def test_without_a_placement_cameras_stay_in_the_kn5_space(tmp_path) -> None:
    got = N.collect(_data(tmp_path), translation=None)
    assert got["cameras.ini"]["frame"] == "kn5_model"
    assert got["cameras.ini"]["sections"]["CAMERA_0"]["POSITION"] == "0,1.425,-0.345"
    d = tmp_path / "out"
    W.write(_doc(got), d)
    assert P.parse_ini((d / "cameras.ini").read_text())["CAMERA_0"]["POSITION"] == "0,1.425,-0.345"


def test_nothing_without_the_block(tmp_path) -> None:
    assert W.write(_doc({}), tmp_path / "x") == []
