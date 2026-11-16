"""SVJ -> AC: the dashboard needles go back to data/analog_instruments.ini and keep their KN5 names."""
from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "svj_to_ac"))

from ac_writers import analog_instruments_ini as W      # noqa: E402
import ac_parsers                                        # noqa: E402


def _doc(extra=None):
    bodies = [
        {"id": "geo_arrow_rpm", "x_instrument": {
            "signal": "rpm", "ac_section": "RPM_INDICATOR", "ac_object": "GEO_ARROW_RPM", "zero_deg": 0.0,
            "step_deg_per_unit": 0.023, "min_value": 0.0, "max_value": 8000.0, "lut_file": "analog_rpm_curve.lut",
            "lut_value_to_deg": [[0, 0], [1000, 30], [8000, 240]]}},
        {"id": "geo_arrow_water", "x_instrument": {
            "signal": "water_temp", "ac_section": "WATER_TEMP", "ac_object": "GEO_ARROW_WATER", "zero_deg": 0.0,
            "step_deg_per_unit": 1.7, "min_value": 60.0, "max_value": 120.0}},
        {"id": "door_fl"},
    ]
    return SimpleNamespace(raw={"chassis": {"mass_bodies": bodies + (extra or [])}})


def test_ini_and_luts_are_written_and_read_back(tmp_path) -> None:
    files = W.write(_doc(), tmp_path)
    assert {f.name for f in files} == {"analog_instruments.ini", "analog_rpm_curve.lut"}
    back = ac_parsers.parse_analog_instruments((tmp_path / "analog_instruments.ini").read_text(), tmp_path)
    by = {i["object_name"]: i for i in back}
    assert by["GEO_ARROW_RPM"]["step_deg"] == 0.023 and by["GEO_ARROW_RPM"]["lut"][-1] == [8000.0, 240.0]
    assert by["GEO_ARROW_WATER"]["min_value"] == 60.0 and by["GEO_ARROW_WATER"]["lut"] is None


def test_nothing_without_needles(tmp_path) -> None:
    doc = SimpleNamespace(raw={"chassis": {"mass_bodies": [{"id": "door_fl"}]}})
    assert W.write(doc, tmp_path) == [] and not (tmp_path / "analog_instruments.ini").exists()


def test_kn5_names_come_from_the_bodies() -> None:
    assert W.ac_object_names(_doc()) == {"SVJ::body::geo_arrow_rpm": "GEO_ARROW_RPM",
                                         "SVJ::body::geo_arrow_water": "GEO_ARROW_WATER"}
