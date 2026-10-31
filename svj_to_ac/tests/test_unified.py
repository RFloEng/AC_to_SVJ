"""A unified suspension (SVJ 0.100+) read back into corner blocks, and the AC files written
from it, checked against the AC car it was converted from (the repository's test_car)."""
from __future__ import annotations

import json
import re
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
ROOT = REPO.parent
sys.path.insert(0, str(REPO))
sys.path.append(str(ROOT))

import svj_loader  # noqa: E402
from ac_writers import suspensions_ini  # noqa: E402
from unified_view import is_unified, normalize  # noqa: E402

TEST_CAR = ROOT / "test_car"


def _parse(text: str) -> dict:
    sec, out = "", {}
    for line in text.splitlines():
        line = line.split(";")[0].strip()
        m = re.match(r"\[(.+?)\]", line)
        if m:
            sec = m.group(1).upper()
            out.setdefault(sec, {})
        elif "=" in line:
            k, v = line.split("=", 1)
            out.setdefault(sec, {})[k.strip().upper()] = v.strip()
    return out


@pytest.fixture(scope="module")
def roundtrip(tmp_path_factory):
    if not TEST_CAR.is_dir():
        pytest.skip("no test_car")
    from converter import _clean, build_svj, read_car_directory
    ini, cm, ctrl, dd = read_car_directory(TEST_CAR)
    svj, _, _ = build_svj(ini, cm, data_dir=dd, ctrl_files=ctrl, include_plots=False)
    svj = _clean(svj)
    assert is_unified(svj)                              # the converter writes SVJ 0.100
    path = tmp_path_factory.mktemp("rt") / "car.svj.json"
    path.write_text(json.dumps(svj), encoding="utf-8")
    doc = svj_loader.load(path)
    out = suspensions_ini.write(doc, tmp_path_factory.mktemp("ac"))
    return doc, _parse(out.read_text(encoding="utf-8")), _parse(
        (TEST_CAR / "data" / "suspensions.ini").read_text(encoding="utf-8", errors="ignore"))


def _vec(v: str) -> list:
    return [float(x) for x in v.replace("|", ",").split(",")]


def test_corner_blocks_are_derived(roundtrip) -> None:
    doc, _, _ = roundtrip
    assert set(doc.suspension.corners) == {"FL", "FR", "RL", "RR"}
    top = doc.suspension["FL"].topology
    assert top["system_type"] == "double_wishbone"
    assert {l["name"] for l in top["links"]} >= {"upper_wishbone", "lower_wishbone"}


def test_front_pickups_match_the_original_car(roundtrip) -> None:
    _, new, old = roundtrip
    for key in ("WBCAR_TOP_FRONT", "WBCAR_TOP_REAR", "WBCAR_BOTTOM_FRONT",
                "WBCAR_BOTTOM_REAR", "WBTYRE_TOP", "WBTYRE_BOTTOM"):
        a, b = _vec(old["FRONT"][key]), _vec(new["FRONT"][key])
        assert all(abs(x - y) < 2e-3 for x, y in zip(a, b)), (key, a, b)


def test_scalars_use_ac_keys(roundtrip) -> None:
    _, new, old = roundtrip
    f = new["FRONT"]
    assert "CAMBER" not in f and "BUMP" not in f and "WHEEL_MASS" not in f   # not AC keys
    assert float(f["SPRING_RATE"]) == pytest.approx(float(old["FRONT"]["SPRING_RATE"]), rel=1e-3)
    # the synthetic test_car says BUMP; real AC files say DAMP_BUMP, the key the game reads
    src = old["FRONT"].get("DAMP_BUMP") or old["FRONT"]["BUMP"]
    assert float(f["DAMP_BUMP"]) == pytest.approx(float(src), rel=1e-2)
    # BASEY is the CG height minus the wheel radius (the synthetic test_car uses it as a ride
    # height, so it is not compared): the CG of this car is 0.46 m up, the wheel radius 0.305
    assert float(f["BASEY"]) == pytest.approx(0.305 - 0.46, abs=0.01)


def test_tyres_can_come_from_a_car_of_an_installed_ac(tmp_path) -> None:
    """tyre_choice takes the chosen car's tyre sections and keeps the converted car's size."""
    import shutil

    from ac_writers import tyres_ini
    if not TEST_CAR.is_dir():
        pytest.skip("no test_car")
    cars = tmp_path / "cars"
    shutil.copytree(TEST_CAR / "data", cars / "donor" / "data")
    doc = svj_loader.load(REPO / "examples" / "mx5_nd_club.svj.json")
    out = tyres_ini.write(doc, tmp_path / "out", {
        "cars_dir": str(cars), "front": {"car": "donor", "section": "FRONT"}})
    text = out.read_text(encoding="utf-8")
    parsed = _parse(text)
    assert "FRONT" in parsed and "REAR" in parsed                  # the rear is fitted as usual
    donor = _parse((TEST_CAR / "data" / "tyres.ini").read_text(encoding="utf-8", errors="ignore"))
    assert parsed["FRONT"]["DY0"] == donor["FRONT"]["DY0"]          # the donor's grip
    set_ = next(v for k, v in doc.tires.sets.items() if "front" in k)
    assert float(parsed["FRONT"]["WIDTH"]) == pytest.approx(set_["dimensions"]["section_width"], abs=1e-4)
    assert "chosen with tyre_match" in text
