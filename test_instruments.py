"""Dashboard needles: analog_instruments.ini, the pivot / axis / range of a needle in the KN5,
and the SVJ body that carries them (synthetic files only, no AC content)."""
from __future__ import annotations

import math
import sys
import tempfile
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent))

import ac_parsers as P
import converter as C
import kn5_reader as K
import test_kn5_export as T

INI = """[RPM_INDICATOR]
ZERO=0
STEP=0.023
OBJECT_NAME=GEO_ARROW_RPM
OBJECT_NAME_MAX=
MIN_VALUE=0
LUT=rpm.lut

[WATER_TEMP]
OBJECT_NAME=GEO_ARROW_WATER
ZERO=10
STEP=2
MIN_VALUE=60

[PLACE_HOLDER_0]
OBJECT_NAME=NOTHING
"""


def _data(t: Path) -> Path:
    d = Path(t) / "car" / "data"
    d.mkdir(parents=True)
    (d / "analog_instruments.ini").write_text(INI)
    (d / "rpm.lut").write_text("0|0\n1000|30\n2000|60\n8000|240\n")
    return d


def _rot_z(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _needle_matrix(pos, deg=0.0):
    m = np.eye(4)
    m[:3, :3] = _rot_z(deg).T                     # KN5 matrices: row-major, rotation transposed
    m[3, :3] = pos
    return [float(v) for v in m.flatten()]


def _kn5(t: Path) -> Path:
    shift = list(T.IDENTITY)
    shift[12] = 0.1                               # a parent offset the pivot must include
    root = T.N("ROOT", children=[T.N("DASH", matrix=shift, children=[
        T.N("GEO_ARROW_RPM", matrix=_needle_matrix((0.4, 0.7, 0.9)), children=[T.N("rpm_mesh", "mesh")]),
        T.N("GEO_ARROW_WATER", matrix=_needle_matrix((0.5, 0.7, 0.9), 51.3), children=[T.N("w_mesh", "mesh")])])])
    p = Path(t) / "car.kn5"
    T._write(p, root)
    return p


def test_ini_is_read_and_placeholders_are_dropped() -> None:
    with tempfile.TemporaryDirectory() as t:
        d = _data(Path(t))
        ins = P.parse_analog_instruments((d / "analog_instruments.ini").read_text(), d)
    assert [i["object_name"] for i in ins] == ["GEO_ARROW_RPM", "GEO_ARROW_WATER", "NOTHING"]
    rpm, water, hold = ins
    assert rpm["signal"] == "rpm" and rpm["lut"][-1] == [8000.0, 240.0] and rpm["step_deg"] == 0.023
    assert water["signal"] == "water_temp" and water["zero_deg"] == 10 and water["min_value"] == 60
    assert hold["signal"] is None
    assert P.gauge_signal("OIL_TEMP") == "oil_temp" and P.gauge_signal("TURBO_INDICATOR_1") == "boost"


def test_angle_follows_lut_or_step() -> None:
    lut = {"lut": [[0, 0], [1000, 30], [2000, 60]], "zero_deg": 0.0}
    assert K.gauge_angle_deg(lut, 1500) == pytest.approx(45.0)
    assert K.gauge_angle_deg(lut, 3000) == pytest.approx(90.0)                # the last slope goes on
    lin = {"zero_deg": 10.0, "step_deg": 2.0, "min_value": 60.0}
    assert K.gauge_angle_deg(lin, 60) == 10.0 and K.gauge_angle_deg(lin, 90) == 70.0


def test_needle_pivot_axis_and_range() -> None:
    with tempfile.TemporaryDirectory() as t:
        d = _data(Path(t))
        ins = P.parse_analog_instruments((d / "analog_instruments.ini").read_text(), d)
        parts = K.describe_instruments(_kn5(Path(t)), ins, max_values={"rpm": 8000.0})
    by = {p["node"]: p for p in parts}
    assert set(by) == {"GEO_ARROW_RPM", "GEO_ARROW_WATER"}               # a gauge not in the model is skipped
    rpm, water = by["GEO_ARROW_RPM"], by["GEO_ARROW_WATER"]
    assert rpm["pivot_model"] == pytest.approx([0.5, 0.7, 0.9])            # node origin + the parent's offset
    assert rpm["axis_model"] == pytest.approx([0, 0, 1])                   # the dial normal, pointing forward
    assert rpm["range_rad"] == pytest.approx([0.0, math.radians(240)]) and not rpm["max_value_guessed"]
    assert water["axis_model"] == pytest.approx([0, 0, 1])                 # a rotated needle keeps Z
    assert water["max_value"] == 120.0 and water["max_value_guessed"]       # no top reading: a default
    assert water["range_rad"][0] == pytest.approx(math.radians(10))


def test_a_backward_normal_is_flipped_to_point_into_the_dash() -> None:
    m = np.eye(4)
    m[:3, :3] = np.diag([1.0, -1.0, -1.0])                                 # local Z points to the driver
    m[3, :3] = (0, 0.5, 1.0)
    root = T.N("ROOT", children=[T.N("GEO_ARROW_RPM", matrix=[float(v) for v in m.flatten()],
                                     children=[T.N("g", "mesh")])])
    with tempfile.TemporaryDirectory() as t:
        p = Path(t) / "car.kn5"
        T._write(p, root)
        ins = [{"section": "RPM_INDICATOR", "signal": "rpm", "object_name": "GEO_ARROW_RPM",
                "zero_deg": 0.0, "step_deg": 1.0, "min_value": 0.0, "lut": None}]
        part = K.describe_instruments(p, ins, max_values={"rpm": 100})[0]
    assert part["axis_model"] == pytest.approx([0, 0, 1])
    assert part["axis"] == pytest.approx([0, 0, 1])                        # local axis flipped with it


def test_svj_body_carries_the_mapping() -> None:
    with tempfile.TemporaryDirectory() as t:
        d = _data(Path(t))
        ins = P.parse_analog_instruments((d / "analog_instruments.ini").read_text(), d)
        parts = K.describe_instruments(_kn5(Path(t)), ins, max_values={"rpm": 8000.0})
    for p in parts:                                   # as the converter does, with an SVJ-frame placement
        p["pivot_svj"], p["axis_svj"] = [0.1, 0.2, 0.3], [1.0, 0.0, 0.0]
    svj = {"chassis": {}}
    nb, nj, ren = C.movable_parts_to_svj(svj, parts, "car_mesh")
    assert (nb, nj) == (2, 2)
    bodies = {b["id"]: b for b in svj["chassis"]["mass_bodies"]}
    xi = bodies["geo_arrow_rpm"]["x_instrument"]
    assert xi["signal"] == "rpm" and xi["ac_object"] == "GEO_ARROW_RPM" and xi["max_value"] == 8000.0
    assert xi["lut_value_to_deg"][1] == [1000.0, 30.0] and "max_value_est" not in xi
    assert bodies["geo_arrow_water"]["x_instrument"]["max_value_est"] is True
    rng = {c["body_b"]: c["range"] for c in svj["constraints"]}
    assert rng["geo_arrow_rpm"] == {"min": 0.0, "max": pytest.approx(math.radians(240), abs=1e-4)}
    assert rng["geo_arrow_water"]["min"] == pytest.approx(math.radians(10), abs=1e-4)
