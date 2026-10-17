"""
Tests for the drivetrain.ini -> SVJ conversion (AWD structure, centre coupling,
clutch, reverse ratio, shift profiles, differential lock control).

Synthetic data only - no Assetto Corsa content.  Run directly or via smoke_test.py.
"""
from __future__ import annotations

import shutil
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from ac_parsers import parse_drivetrain, parse_ini
from converter import (DIFF_TYPE_MAP, _build_driveline, _load_diff_controllers,
                       _lock_control, drive_layout, drive_type, infer_engine_layout)

AWD_INI = """[HEADER]
VERSION=3
[TRACTION]
TYPE=AWD
[GEARS]
COUNT=3
GEAR_R=-2.000
GEAR_1=3.2
GEAR_2=2.1
GEAR_3=1.5
FINAL=4.0
[DIFFERENTIAL]
POWER=0.25
COAST=0.25
PRELOAD=5
[AWD]
FRONT_SHARE=47
FRONT_DIFF_POWER=0.80
FRONT_DIFF_COAST=0.40
FRONT_DIFF_PRELOAD=35
CENTRE_DIFF_POWER=0.70
CENTRE_DIFF_COAST=0.60
CENTRE_DIFF_PRELOAD=60
REAR_DIFF_POWER=0.60
REAR_DIFF_COAST=0.40
REAR_DIFF_PRELOAD=300
[CLUTCH]
MAX_TORQUE=700
[AUTOCLUTCH]
UPSHIFT_PROFILE=NONE
DOWNSHIFT_PROFILE=MY_DOWNSHIFT
[MY_DOWNSHIFT]
POINT_0=50
POINT_1=280
POINT_2=600
[AUTO_SHIFTER]
UP=7200
DOWN=3500
"""

AWD2_INI = """[TRACTION]
TYPE=AWD2
[GEARS]
COUNT=2
GEAR_1=3.0
GEAR_2=2.0
FINAL=3.5
[DIFFERENTIAL]
POWER=0.5
COAST=0.5
PRELOAD=10
[AWD2]
FRONT_DIFF_POWER=0.05
FRONT_DIFF_COAST=0.05
FRONT_DIFF_PRELOAD=3
CENTRE_RAMP_TORQUE=40.9
CENTRE_MAX_TORQUE=816.6
REAR_DIFF_POWER=0.23
REAR_DIFF_COAST=0.36
REAR_DIFF_PRELOAD=11
"""

RWD_WITH_LEFTOVER_AWD = """[TRACTION]
TYPE=RWD
[GEARS]
COUNT=2
GEAR_1=3.0
GEAR_2=2.0
FINAL=3.5
[DIFFERENTIAL]
POWER=0.3
COAST=0.2
PRELOAD=25
[AWD]
FRONT_SHARE=40
FRONT_DIFF_POWER=0.9
"""

CTRL_INI = """[CONTROLLER_0]
INPUT=GEAR
COMBINATOR=MULT
LUT=gear_mult.lut
FILTER=0.9
UP_LIMIT=1000
DOWN_LIMIT=0.0

[CONTROLLER_1]
INPUT=BRAKE
COMBINATOR=ADD
LUT=brake_add.lut
FILTER=0.99
UP_LIMIT=1
DOWN_LIMIT=0.0
"""


def _dt(text: str) -> dict:
    return parse_drivetrain(parse_ini(text))


def test_number_with_a_double_decimal_point_typo():
    from ac_parsers import _num
    assert _num("2.58.0") == 2.58 and _num("0.59.0") == 0.59
    assert _num("2.530      (2.52)") == 2.53           # annotation still fine
    assert _num("1,2,3") is None and _num("1.2.3.4") is None


def test_awd_section_parsed_and_normalised():
    dt = _dt(AWD_INI)
    awd = dt["awd"]
    assert awd["model"] == "awd" and awd["front_share"] == 0.47      # 47 % -> fraction
    assert awd["front"] == {"power": 0.8, "coast": 0.4, "preload": 35.0}
    assert awd["rear"] == {"power": 0.6, "coast": 0.4, "preload": 300.0}
    assert awd["centre"] == {"power": 0.7, "coast": 0.6, "preload": 60.0}
    # a share written as a fraction stays a fraction
    assert _dt(AWD_INI.replace("FRONT_SHARE=47", "FRONT_SHARE=0.4"))["awd"]["front_share"] == 0.4
    # 1 means 1 % (rear-biased GT-Rs, Lamborghinis), not 100 %
    assert _dt(AWD_INI.replace("FRONT_SHARE=47", "FRONT_SHARE=1"))["awd"]["front_share"] == 0.01


def test_clutch_reverse_ratio_and_extras():
    dt = _dt(AWD_INI)
    assert dt["clutch"] == {"max_torque": 700.0}
    assert dt["gearbox"]["reverse_ratio"] == 2.0                     # AC writes it negative
    assert dt["auto_shifter"] == {"up": 7200.0, "down": 3500.0}


def test_shift_profiles_resolved_by_name():
    dt = _dt(AWD_INI)
    sp = dt["shift_profiler"]
    assert "upshift" not in sp                                       # UPSHIFT_PROFILE=NONE
    assert sp["downshift"] == {"name": "MY_DOWNSHIFT",
                               "points": {"point_0": 50.0, "point_1": 280.0, "point_2": 600.0}}
    # the older fixed section name still works when AUTOCLUTCH names no profile
    legacy = "[TRACTION]\nTYPE=RWD\n[GEARS]\nCOUNT=1\nGEAR_1=3\n[DOWNSHIFT_PROFILER]\nPOINT_0=10\n"
    assert _dt(legacy)["shift_profiler"]["downshift"]["points"] == {"point_0": 10.0}


def test_awd2_is_a_coupling_not_a_split():
    dt = _dt(AWD2_INI)
    assert dt["awd"]["model"] == "awd2" and "front_share" not in dt["awd"]
    assert dt["awd"]["centre"] == {"ramp_torque": 40.9, "max_torque": 816.6}
    diffs, tc = _build_driveline(dt, drive_layout(dt["layout_raw"]), "lsd_clutch", "front")
    assert tc["type"] == "on_demand" and "torque_split" not in tc
    assert tc["coupling"] == {"ramp_torque": 40.9, "max_torque": 816.6}
    assert "center_diff" not in tc                                    # no lock values given
    assert [d["location"] for d in diffs] == ["front", "rear"]


def test_leftover_awd_section_on_a_rwd_car_is_ignored():
    dt = _dt(RWD_WITH_LEFTOVER_AWD)
    assert dt["awd"] is None
    diffs, tc = _build_driveline(dt, "FR", "lsd_clutch", "rear")
    assert tc is None and len(diffs) == 1 and diffs[0]["location"] == "rear"
    assert diffs[0]["preload"] == 25.0 and diffs[0]["lock_power"] == 0.3


def test_awd_driveline_shape():
    dt = _dt(AWD_INI)
    diffs, tc = _build_driveline(dt, "AWD", "lsd_clutch", "front")
    by = {d["location"]: d for d in diffs}
    assert set(by) == {"front", "rear"}
    assert by["front"] == {"id": "diff_front", "location": "front", "type": "lsd_clutch",
                           "final_drive": 4.0, "preload": 35.0, "lock_power": 0.8, "lock_coast": 0.4}
    assert by["rear"]["preload"] == 300.0 and by["rear"]["lock_power"] == 0.6
    assert tc["type"] == "full_time" and tc["torque_split"] == [0.47, 0.53]
    assert tc["center_diff"] == {"type": "lsd_clutch", "location": "center", "preload": 60.0,
                                 "lock_power": 0.7, "lock_coast": 0.6}


def test_awd_car_without_awd_data_keeps_the_single_diff():
    text = AWD_INI.split("[AWD]")[0] + "[CLUTCH]\nMAX_TORQUE=500\n"
    dt = _dt(text)
    diffs, tc = _build_driveline(dt, "AWD", "lsd_clutch", "front")
    assert tc is None and len(diffs) == 1 and diffs[0]["preload"] == 5.0


def test_layout_and_type_names():
    assert drive_layout("AWD2") == "AWD" and drive_type("AWD") == "AWD"
    assert drive_type("FR") == "RWD" and drive_type("FF") == "FWD"
    assert DIFF_TYPE_MAP["TORSEN"] == "lsd_torsen" and DIFF_TYPE_MAP["VISCOUS"] == "lsd_viscous"


def test_engine_layout_inferred_from_weight_split():
    assert infer_engine_layout("FR", 0.53, "Some Coupe") == ("FR", None)
    assert infer_engine_layout("FR", 0.46, "Some Coupe") == ("FR", None)
    assert infer_engine_layout("FR", 0.42, "Mid Racer") == ("MR", "weight")
    assert infer_engine_layout("FR", 0.38, "Porsche 911 Carrera") == ("RR", "name")
    # other layouts and missing data are left alone
    assert infer_engine_layout("FF", 0.60, "x") == ("FF", None)
    assert infer_engine_layout("AWD", 0.40, "x") == ("AWD", None)
    assert infer_engine_layout("FR", 0.0, "x") == ("FR", None)


def test_unusable_cm_weight_does_not_abort_the_conversion():
    import copy
    from converter import build_svj, read_car_directory
    src = Path(__file__).parent / "test_car"
    if not src.is_dir():
        return
    ini, cm, ctrl, dd = read_car_directory(src)
    for bad in ("_kg*", "-", ""):
        cm2 = copy.deepcopy(cm) or {}
        cm2.setdefault("specs", {})["weight"] = bad
        build_svj(ini, cm2, data_dir=dd, ctrl_files=ctrl)       # must not raise


def test_lock_control_from_controller_files():
    with tempfile.TemporaryDirectory() as t:
        data = Path(t)
        (data / "gear_mult.lut").write_text("0|1\n1|12.5\n2|9.9\n", encoding="utf-8")
        (data / "brake_add.lut").write_text("0|0\n1|0.5\n", encoding="utf-8")
        ctl = _load_diff_controllers({"ctr_awd_center_lock.ini": CTRL_INI}, data)
    lc = _lock_control(ctl)
    assert lc["target"] == "lock" and lc["_source"] == ["ctr_awd_center_lock.ini"]
    s0, s1 = lc["stages"]
    assert s0 == {"input": "gear", "combine": "multiply", "curve": [[0.0, 1.0], [1.0, 12.5], [2.0, 9.9]],
                  "x_filter": 0.9, "min": 0.0, "max": 1000.0}
    assert s1["combine"] == "add" and s1["max"] == 1.0 and s1["curve"][-1] == [1.0, 0.5]
    # attached to the centre diff of an AWD car (lock_control, SVJ v0.99.3)
    _, tc = _build_driveline(_dt(AWD_INI), "AWD", "lsd_clutch", "front", ctl)
    assert tc["center_diff"]["lock_control"]["stages"][0]["input"] == "gear"


def test_converter_end_to_end_awd(schema_path: Path | None = None):
    import json
    from converter import build_svj, read_car_directory, _clean
    src = Path(__file__).parent / "test_car"
    if not src.is_dir():
        return
    with tempfile.TemporaryDirectory() as t:
        car = Path(t) / "awdcar"
        shutil.copytree(src, car)
        (car / "data" / "drivetrain.ini").write_text(AWD_INI, encoding="utf-8")
        (car / "data" / "ctr_awd_center_lock.ini").write_text(CTRL_INI, encoding="utf-8")
        (car / "data" / "gear_mult.lut").write_text("0|1\n1|12.5\n", encoding="utf-8")
        (car / "data" / "brake_add.lut").write_text("0|0\n1|0.5\n", encoding="utf-8")
        ini, cm, ctrl, dd = read_car_directory(car)
        svj, log, _ = build_svj(ini, cm, data_dir=dd, ctrl_files=ctrl)
        svj = _clean(svj)
    pt = svj["powertrain"]
    assert pt["layout"] == "AWD" and svj["vehicle_info"]["drive_type"] == "AWD"
    assert pt["clutch"] == {"max_torque": 700.0}
    assert pt["gearbox"]["reverse_ratios"] == [2.0]
    assert pt["transfer_case"]["torque_split"] == [0.47, 0.53]
    assert [d["location"] for d in pt["differentials"]] == ["front", "rear"]
    x = svj["x_assettocorsa"]
    assert x["drivetrain"]["awd"]["front_share"] == 0.47
    assert "ctr_awd_center_lock.ini" in x["drivetrain"]["controllers"]
    assert x["driver_aids"]["shift_profiler"]["downshift"]["name"] == "MY_DOWNSHIFT"
    assert any("Driveline: full_time transfer case, 47% front" in l for l in log), log
    # validate the powertrain block against the SVJ schema when the standard is around
    sp = (Path(schema_path) if schema_path else
          Path(__file__).resolve().parents[2] / "SVJ-standard-vehicle-json_repo" / "repo"
          / "schema" / "svj.schema.json")
    if sp.is_file():
        try:
            import jsonschema
        except ImportError:
            return
        schema = json.loads(sp.read_text(encoding="utf-8"))
        sub = {**schema, "required": [], "properties": {"powertrain": schema["properties"]["powertrain"]}}
        errs = list(jsonschema.Draft7Validator(sub).iter_errors({"powertrain": pt}))
        assert not errs, [e.message for e in errs][:3]


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
