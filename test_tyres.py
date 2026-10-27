"""Tests for the tyre set written to the SVJ: the size the car's author marked in tyres.ini,
and what is left out (the CSP-only values).  Synthetic data only."""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from ac_parsers import marked_size_for, parse_marked_tyre_sizes, parse_tyre_extras

RAW = """;F 235/40ZR18 91 W, 8.0JX18 ET46 5X108
;R 295/35ZR18 99 W, 10.5JX18 ET81.2 5X108
[FRONT]
NAME=Avon CR322 185/70R13
[REAR_1]
NAME=Street 90s
[FRONT_2]
NAME=Slicks 9999/99R99
"""


def test_marked_sizes_are_read_from_names_and_comment_lines():
    m = parse_marked_tyre_sizes(RAW)
    assert marked_size_for(m, "FRONT") == (185, 70, 13)         # the section's own NAME
    assert marked_size_for(m, "FRONT_1") == (185, 70, 13)       # else the plain axle section
    assert marked_size_for(m, "REAR_1") == (295, 35, 18)        # else the ;R comment line
    assert marked_size_for(m, "FRONT_2") == (185, 70, 13)       # an implausible size is ignored
    assert marked_size_for(parse_marked_tyre_sizes("[FRONT]%sNAME=Street" % chr(10)), "FRONT") is None


def test_csp_values_are_not_written():
    ex = parse_tyre_extras({"FALLOFF_LEVEL": "0.86", "FALLOFF_SPEED": "2", "DY_REF": "1.6",
                            "RELAXATION_LENGTH": "0.07", "SPEED_SENSITIVITY": "0.003"})
    assert "csp" not in ex and ex["relaxation_length_m"] == 0.07


def test_marked_size_goes_into_the_set_and_a_contradicting_one_does_not():
    import shutil
    import tempfile
    from converter import build_svj, read_car_directory
    src = Path(__file__).parent / "test_car"
    if not src.is_dir():
        return
    sets = {}
    for tag, mark in (("good", "205/55R15"), ("bad", "205/20R15")):
        with tempfile.TemporaryDirectory() as t:
            car = Path(t) / "c"
            shutil.copytree(src, car)
            ini = (car / "data" / "tyres.ini").read_text(errors="replace")
            (car / "data" / "tyres.ini").write_text(";F %s 91 W%s" % (mark, chr(10)) + ini)
            i, cm, ctrl, dd = read_car_directory(car)
            svj, _, _ = build_svj(i, cm, data_dir=dd, ctrl_files=ctrl, include_plots=False)
            sets[tag] = next(v for k, v in svj["tires"]["sets"].items() if "front" in k)
    # test_car's tyre is 205 wide, 15" rim, 56 % in the physics
    assert sets["good"]["size_code"] == "205/55R15" and "x_size_code_source" in sets["good"]
    assert sets["bad"]["size_code"] != "205/20R15" and "x_size_code_note" in sets["bad"]


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
