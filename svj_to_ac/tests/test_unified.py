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


def _hinged_arm_doc() -> dict:
    """A minimal unified suspension: a MacPherson corner whose lower arm is one revolute joint
    (as an SVJ from another simulator may give it), with a spring, a damper and both stops."""
    frame = lambda pos, q=(0.0, 0.0, 0.0, 1.0): {"position": list(pos), "orientation": list(q)}
    bodies = [{"id": b, "mass": 0.0, "position": [0, -0.7, -0.3]}
              for b in ("upright_fl", "wheel_fl", "lower_arm_fl", "strut_fl")]
    bodies[0]["markers"] = {"wheel_center": frame([0.0, -0.78, -0.31])}
    joint = lambda jid, jt, a, fa, b, fb=None: {"id": jid, "type": "joint", "joint_type": jt,
                                                "body_a": a, "frame_a": fa, "body_b": b,
                                                "frame_b": fb or fa}
    spring_a, spring_b = [0.0, -0.6, -0.8], [0.0, -0.78, -0.31]         # about 0.5 m long, vertical-ish
    cons = [
        joint("hinge_wheel_fl", "revolute", "upright_fl", frame([0, -0.78, -0.31]), "wheel_fl"),
        joint("hinge_lower_arm_fl", "revolute", "chassis",
              frame([0.01, -0.55, -0.26], (0.0, 0.7071068, 0.0, 0.7071068)), "lower_arm_fl"),
        joint("ball_lower_arm_fl_outboard", "spherical", "lower_arm_fl",
              frame([0.01, -0.71, -0.20]), "upright_fl"),
        joint("ball_strut_fl_inboard", "spherical", "chassis", frame([-0.04, -0.55, -0.86]), "strut_fl"),
        joint("slide_strut_fl_outboard", "translational", "strut_fl",
              frame([0.01, -0.62, -0.19]), "upright_fl"),
        {"id": "spring_fl", "type": "spring", "body_a": "chassis", "frame_a": frame(spring_a),
         "body_b": "upright_fl", "frame_b": frame(spring_b), "reference": "element",
         "rate": 22400.0, "free_length": 0.60},
        {"id": "damper_fl", "type": "damper", "body_a": "chassis", "frame_a": frame(spring_a),
         "body_b": "upright_fl", "frame_b": frame(spring_b), "reference": "element",
         "bump_curve": [[0.0, 0.0], [0.5, 1600.0]], "rebound_curve": [[0.0, 0.0], [0.5, 1800.0]]},
        {"id": "bump_stop_fl", "type": "spring", "body_a": "chassis", "frame_a": frame(spring_a),
         "body_b": "upright_fl", "frame_b": frame(spring_b), "free_length": 0.45,
         "rate_curve": [[-1.0, 0.0], [0.0, 0.0], [0.3, 69000.0]]},
    ]
    return {"wheel_stations": [{"id": "FL", "upright_body": "upright_fl", "wheel_body": "wheel_fl",
                                "wheel_center_marker": "wheel_center", "topology_hint": "macpherson",
                                "alignment": {"camber": -0.02}}],
            "chassis": {"mass_bodies": bodies}, "constraints": cons}


def test_an_arm_given_as_one_hinge_and_the_stops_of_another_source() -> None:
    data, notes = normalize(_hinged_arm_doc())
    c = data["suspension"]["FL"]
    arm = next(l for l in c["topology"]["links"] if l["name"] == "lower_arm")
    assert len(arm["inboard_points"]) == 2 and arm["_est"]                 # two points of the hinge axis
    xs = [p[0] for p in arm["inboard_points"]]
    assert xs[0] > xs[1] and abs(xs[0] - xs[1] - 0.30) < 1e-6              # front first, ARM_HALF each side
    assert {l["name"] for l in c["topology"]["links"]} >= {"strut", "lower_arm"}
    stops = c["damper"]["bumpstops"]
    mr = c["spring"]["rate"] / 22400.0 ** 1                                # rate is at the wheel: k * MR^2
    assert 0.4 ** 2 <= mr <= 1.0
    # the element is ~0.51 m long, the bump stop's free length 0.45: it engages after ~0.06 m
    assert 0.04 < stops["up"] < 0.12 and stops["rate"] > 40000
    assert any(note for note in notes)                                     # the motion ratio is a guess


def test_tyres_are_fitted_to_a_pacejka_block_of_another_source() -> None:
    import sys
    sys.path.append(str(ROOT))
    from pacejka_inverse import fit_ac_params
    import tyre_match as T
    lat = {"pCy1": 1.35, "pDy1": 1.0, "pDy2": -0.2, "pEy1": -0.4, "pKy1": -16.0, "pKy2": 1.6}
    lon = {"pCx1": 1.6, "pDx1": 1.05, "pDx2": -0.15, "pEx1": 0.2, "pKx1": 20.0, "pKx2": 0.0}
    tset = {"pacejka": {"model": "MF62", "lateral": lat, "longitudinal": lon},
            "reference": {"load": 4000.0},
            "dimensions": {"section_width": 0.205, "overall_diameter": 0.63},
            "rim": {"diameter": 0.4318}}
    p = fit_ac_params(tset)
    assert {"DY0", "DX0", "LS_EXPY", "FRICTION_LIMIT_ANGLE", "WIDTH", "RADIUS", "RIM_RADIUS"} <= set(p)
    assert 0.8 < p["DY0"] < 1.3 and 0.8 < p["DX0"] < 1.4 and p["RADIUS"] == pytest.approx(0.315)
    target = T.svj_set_features(tset)
    from tire_lab import ACTyreParams
    from tire_lab import ac_fx, ac_fy
    # peak grip of the fit at the reference load is the target's within 10 %
    mu_fit = T.ac_features(ACTyreParams(FZ0=p["FZ0"], DY0=p["DY0"], DY1=p["DY1"], LS_EXPY=p["LS_EXPY"],
                                        DX0=p["DX0"], DX1=p["DX1"], LS_EXPX=p["LS_EXPX"],
                                        FLEX=p["FLEX"]))["mu_y"]
    assert abs(mu_fit - target["mu_y"]) / target["mu_y"] < 0.12


def _mx5_json() -> dict:
    return json.loads((REPO / "examples" / "mx5_nd_club.svj.json").read_text(encoding="utf-8"))


def _load_dict(d: dict, tmp_path: Path):
    p = tmp_path / "car.svj.json"
    p.write_text(json.dumps(d), encoding="utf-8")
    return svj_loader.load(p)


def test_a_file_without_tyres_brakes_or_powertrain_still_converts(tmp_path) -> None:
    import pack_car
    d = _mx5_json()
    for k in ("tires", "brakes", "powertrain"):
        d.pop(k)
    doc = _load_dict(d, tmp_path)
    assert any("no tyres" in n for n in doc.notes) and any("no brakes" in n for n in doc.notes)
    res = pack_car.assemble(doc, tmp_path / "out")
    tyres = _parse((res.data_dir / "tyres.ini").read_text(encoding="utf-8"))
    assert "FRONT" in tyres and "REAR" in tyres                      # a generic tyre of the wheel's size
    assert "conversion_log.txt" in {p.name for p in res.car_dir.iterdir()}


def test_more_than_two_axles_is_refused_with_a_reason(tmp_path) -> None:
    d = {"vehicle_info": {}, "chassis": {}, "wheel_stations": [
        {"id": f"A{i}{s}", "axle": f"A{i}", "side": s} for i in (1, 2, 3) for s in "LR"]}
    with pytest.raises(svj_loader.SvjLoadError, match="two axles"):
        _load_dict(d, tmp_path)


def test_sparse_corners_get_a_placeholder_geometry(tmp_path) -> None:
    from ac_writers import suspensions_ini
    d = _mx5_json()
    for k in ("FL", "FR", "RL", "RR"):                               # BeamNG gives a hub centre only
        d["suspension"][k]["topology"] = {"hardpoints": {"hub_centre": [0.0, 0.465, 0.0]}}
    doc = _load_dict(d, tmp_path)
    text = suspensions_ini.write(doc, tmp_path / "data").read_text(encoding="utf-8")
    assert "PLACEHOLDER" in text and "WBCAR_TOP_FRONT" in text and _parse(text)["FRONT"]["TYPE"] == "1"


def test_other_suspension_types_map_onto_ac_types(tmp_path) -> None:
    from ac_writers import suspensions_ini
    d = _mx5_json()
    for k in ("RL", "RR"):
        d["suspension"][k]["topology"]["system_type"] = "de_dion"
    doc = _load_dict(d, tmp_path)
    text = suspensions_ini.write(doc, tmp_path / "data").read_text(encoding="utf-8")
    assert _parse(text)["REAR"]["TYPE"] == "3"                       # AC's solid axle
