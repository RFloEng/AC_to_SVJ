"""Round-trip tests for the suspensions writer.

Four families of tests:

* Native pass-through round-trip (TYPE=1 DWB and TYPE=2 multi-link
  loaded from ``AC_to_SVJ/test_car/data/suspensions.ini``): write back
  and assert parsed-INI equality.
* Canonical SVJ -> AC path on ``mx5_nd_club.svj.json`` (TYPE=1 DWB):
  every pickup matches its expected AC value to < 1 mm, computed
  independently here via :mod:`coords` so the writer can't cheat.
* All six AC TYPEs scaffold-emit (minimal synthetic SvjDoc per type):
  writer produces the right ``TYPE=`` line and emits the right keyset.
* Panhard-vs-Watts disambiguation: synthetic SVJ has both populated;
  writer emits PANHARD_*, comments WATTS_* on a trailing line, and
  appends a warning to ``conversion_log.txt``.
"""

from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from typing import Any

import pytest

import svj_loader
from coords import pickup_lateral_to_ac, svj_to_ac_point
from svj_loader import (
    SvjAssettoExt,
    SvjBrakes,
    SvjChassis,
    SvjCorner,
    SvjDoc,
    SvjMetadata,
    SvjPowertrain,
    SvjSuspension,
    SvjTires,
    SvjVehicleInfo,
)

from ac_writers import suspensions_ini
from ac_writers._axle_context import SYSTEM_TYPE_TO_AC, AxleContext, resolve_axle

# Conftest puts AC_to_SVJ on sys.path.
from ac_parsers import parse_ini, parse_suspension_axle  # noqa: E402

REPO = Path(__file__).resolve().parents[1]
AC_DATA = REPO.parent / "test_car" / "data"


# --------------------------------------------------------------------------- #
# Helpers                                                                     #
# --------------------------------------------------------------------------- #


def _normalize(ini: dict[str, dict[str, str]]) -> dict[str, dict[str, Any]]:
    out: dict[str, dict[str, Any]] = {}
    for sec, kv in ini.items():
        out[sec.upper()] = {}
        for k, v in kv.items():
            s = v.strip()
            parts = [p.strip() for p in s.replace("|", ",").split(",") if p.strip()]
            if len(parts) >= 2 and all(_is_num(p) for p in parts):
                out[sec.upper()][k.upper()] = tuple(round(float(p), 6) for p in parts)
            elif _is_num(s):
                out[sec.upper()][k.upper()] = float(s)
            else:
                out[sec.upper()][k.upper()] = s
    return out


def _is_num(s: str) -> bool:
    try:
        float(s)
        return True
    except ValueError:
        return False


def _doc_with_native(native: dict[str, dict[str, dict[str, str]]]) -> SvjDoc:
    ext = SvjAssettoExt(raw={"_ac_native": native})
    return SvjDoc(
        vehicle_info=SvjVehicleInfo("X", "Y", 2020, "v", "RWD"),
        chassis=SvjChassis(1058.0, 2.31, 1.495, 1.505, [0.0, 0.0, -0.46]),
        suspension=SvjSuspension(corners={
            "FL": SvjCorner("FL", [0, -0.748, -0.3]),
            "FR": SvjCorner("FR", [0, 0.748, -0.3]),
            "RL": SvjCorner("RL", [-2.31, -0.751, -0.3]),
            "RR": SvjCorner("RR", [-2.31, 0.751, -0.3]),
        }),
        tires=SvjTires(sets={"front": {}}),
        powertrain=SvjPowertrain(layout="FR"),
        brakes=SvjBrakes(0.6, "fixed"),
        metadata=SvjMetadata(version="0.95"),
        assetto=ext,
    )


def _synthetic_corner(system_type: str, **topology_extras: Any) -> SvjCorner:
    """Build a corner with a single link group covering one system_type's
    canonical pickups. Coordinates are nonsense but plausibly geometric;
    we only care that they round-trip cleanly through the writer."""
    base_top: dict[str, Any] = {
        "system_type": system_type,
        "upright": {
            "id": "upright",
            "hardpoints": {
                "wheel_center": [0.0, -0.75, -0.3],
                "upper_ball_joint": [-0.03, -0.59, -0.42],
                "lower_ball_joint": [0.00, -0.65, -0.21],
                "steering_tie_rod_end": [0.12, -0.65, -0.25],
            },
        },
        "links": [],
    }
    base_top.update(topology_extras)
    return SvjCorner("FL", [0.0, -0.75, -0.3], topology=base_top)


def _ac_pickup_expected(svj_point, ctx: AxleContext) -> tuple[float, float, float]:
    """Reference transform for the writer's pickup output, computed independently:
    AC measures a pickup from the wheel centre, X inboard, Y up, Z forward."""
    sx, sy, sz = svj_point
    ax = pickup_lateral_to_ac(sy, ctx.half_track)
    if ctx.wheel_center is not None:
        wx, _, wz = ctx.wheel_center
        return (round(ax, 6), round(-(sz - wz), 6), round(sx - wx, 6))
    _, ay, az = svj_to_ac_point((sx, sy, sz))
    return (round(ax, 6), round(ay, 6), round(az, 6))


def _read_pickup(ini_sec: dict[str, str], key: str) -> tuple[float, float, float]:
    s = ini_sec[key]
    parts = [p.strip() for p in s.replace("|", ",").split(",") if p.strip()]
    return (float(parts[0]), float(parts[1]), float(parts[2]))


# --------------------------------------------------------------------------- #
# Native pass-through round-trip                                              #
# --------------------------------------------------------------------------- #


class TestNativePassthrough:
    def test_dwb_multilink_roundtrip(self, tmp_path: Path) -> None:
        orig_parsed = parse_ini((AC_DATA / "suspensions.ini").read_text(encoding="utf-8"))
        doc = _doc_with_native({"suspensions": orig_parsed})
        out = suspensions_ini.write(doc, tmp_path)
        new_parsed = parse_ini(out.read_text(encoding="utf-8"))
        assert _normalize(orig_parsed) == _normalize(new_parsed)

    def test_per_axle_semantic_via_parse_suspension_axle(self, tmp_path: Path) -> None:
        orig = parse_ini((AC_DATA / "suspensions.ini").read_text(encoding="utf-8"))
        doc = _doc_with_native({"suspensions": orig})
        out = suspensions_ini.write(doc, tmp_path)
        new = parse_ini(out.read_text(encoding="utf-8"))
        for axle in ("FRONT", "REAR"):
            assert parse_suspension_axle(orig[axle]) == parse_suspension_axle(new[axle])

    def test_lf_and_no_bom(self, tmp_path: Path) -> None:
        orig = parse_ini((AC_DATA / "suspensions.ini").read_text(encoding="utf-8"))
        doc = _doc_with_native({"suspensions": orig})
        out = suspensions_ini.write(doc, tmp_path)
        raw = out.read_bytes()
        assert b"\r" not in raw
        assert raw.endswith(b"\n")
        assert not raw.startswith(b"\xef\xbb\xbf")


# --------------------------------------------------------------------------- #
# Canonical SVJ -> AC pickup accuracy                                         #
# --------------------------------------------------------------------------- #


class TestCanonicalDWBFromMx5:
    """Each pickup written from the mx5 fixture must match the independent
    coords-based reference to < 1 mm."""

    @pytest.fixture(scope="class")
    def written_ini(self, tmp_path_factory) -> dict[str, dict[str, str]]:
        doc = svj_loader.load(REPO / "examples" / "mx5_nd_club.svj.json")
        out_dir = tmp_path_factory.mktemp("susp")
        out = suspensions_ini.write(doc, out_dir)
        return parse_ini(out.read_text(encoding="utf-8"))

    def test_front_is_dwb(self, written_ini) -> None:
        assert int(float(written_ini["FRONT"]["TYPE"])) == SYSTEM_TYPE_TO_AC["double_wishbone"]

    def test_track_scalar_matches(self, written_ini) -> None:
        assert float(written_ini["FRONT"]["TRACK"]) == pytest.approx(1.496, abs=1e-6)
        assert float(written_ini["REAR"]["TRACK"]) == pytest.approx(1.503, abs=1e-6)

    def test_pickups_within_1mm(self, written_ini) -> None:
        doc = svj_loader.load(REPO / "examples" / "mx5_nd_club.svj.json")
        ctx_front = resolve_axle(doc, "FRONT")
        fl = doc.suspension["FL"]
        top = fl.topology
        upper = next(l for l in top["links"] if l["name"] == "upper_wishbone")
        lower = next(l for l in top["links"] if l["name"] == "lower_wishbone")
        tie = next(l for l in top["links"] if l["name"] == "steering_tie_rod")

        checks = {
            "WBCAR_TOP_FRONT": upper["inboard_points"][0],
            "WBCAR_TOP_REAR": upper["inboard_points"][1],
            "WBCAR_BOTTOM_FRONT": lower["inboard_points"][0],
            "WBCAR_BOTTOM_REAR": lower["inboard_points"][1],
            "WBCAR_STEER": tie["inboard_points"][0],
            "WBTYRE_TOP": top["upright"]["hardpoints"]["upper_ball_joint"],
            "WBTYRE_BOTTOM": top["upright"]["hardpoints"]["lower_ball_joint"],
            "WBTYRE_STEER": top["upright"]["hardpoints"]["steering_tie_rod_end"],
        }

        for key, svj_pt in checks.items():
            assert key in written_ini["FRONT"], f"missing pickup {key}"
            got = _read_pickup(written_ini["FRONT"], key)
            expected = _ac_pickup_expected(svj_pt, ctx_front)
            for g, e in zip(got, expected):
                assert abs(g - e) < 1e-3, f"{key}: {got} vs {expected}"


# --------------------------------------------------------------------------- #
# All six AC TYPEs                                                            #
# --------------------------------------------------------------------------- #


_MIN_HP = {
    "wheel_center": [0.0, -0.75, -0.3],
    "upper_ball_joint": [-0.03, -0.59, -0.42],
    "lower_ball_joint": [0.00, -0.65, -0.21],
    "steering_tie_rod_end": [0.12, -0.65, -0.25],
}


def _link(name: str, inboard, outboard) -> dict[str, Any]:
    return {"name": name, "type": "arm", "inboard_points": inboard, "outboard_point": outboard}


def _doc_with_topology(system_type: str, links: list[dict[str, Any]]) -> SvjDoc:
    corner = SvjCorner(
        "FL",
        [0.0, -0.75, -0.3],
        topology={
            "system_type": system_type,
            "upright": {"id": "FL", "hardpoints": dict(_MIN_HP)},
            "links": links,
        },
    )
    return SvjDoc(
        vehicle_info=SvjVehicleInfo("Acme", "Synth", 2020, "test", "RWD"),
        chassis=SvjChassis(1000.0, 2.3, 1.5, 1.5, [0, 0, -0.4]),
        suspension=SvjSuspension(corners={
            "FL": corner,
            "FR": SvjCorner("FR", [0, 0.75, -0.3]),
            # Same topology for rear to keep resolver happy.
            "RL": SvjCorner(
                "RL",
                [-2.3, -0.75, -0.3],
                topology={
                    "system_type": system_type,
                    "upright": {"id": "RL", "hardpoints": dict(_MIN_HP)},
                    "links": deepcopy(links),
                },
            ),
            "RR": SvjCorner("RR", [-2.3, 0.75, -0.3]),
        }),
        tires=SvjTires(sets={"tire_front_1": {"dimensions": {"overall_diameter": 0.62}},
                              "tire_rear_1": {"dimensions": {"overall_diameter": 0.62}}}),
        powertrain=SvjPowertrain(layout="FR"),
        brakes=SvjBrakes(0.6, "fixed"),
        metadata=SvjMetadata(version="0.95"),
    )


_FIXTURES = {
    "macpherson": [
        _link("lower_wishbone", [[0.45, -0.12, 0.17], [0.45, -0.12, -0.15]], _MIN_HP["lower_ball_joint"]),
        _link("strut", [[0.38, -0.72, -0.01]], [0.23, -0.165, -0.015]),
        _link("steering_tie_rod", [[0.45, -0.205, 0.17]], _MIN_HP["steering_tie_rod_end"]),
    ],
    "double_wishbone": [
        _link("upper_wishbone", [[0.38, -0.40, 0.12], [0.38, -0.40, -0.12]], _MIN_HP["upper_ball_joint"]),
        _link("lower_wishbone", [[0.45, -0.12, 0.17], [0.45, -0.12, -0.15]], _MIN_HP["lower_ball_joint"]),
        _link("steering_tie_rod", [[0.45, -0.205, 0.17]], _MIN_HP["steering_tie_rod_end"]),
    ],
    "multi_link": [
        _link("upper_wishbone", [[0.37, -0.38, 0.06], [0.37, -0.38, -0.06]], _MIN_HP["upper_ball_joint"]),
        _link("lower_wishbone", [[0.45, -0.11, 0.08], [0.45, -0.11, -0.10]], _MIN_HP["lower_ball_joint"]),
        _link("toe_link", [[0.45, -0.285, 0.17]], _MIN_HP["steering_tie_rod_end"]),
    ],
    "solid_axle": [
        _link("link_lower", [[0.45, -0.30, 0.00]], [0.10, -0.55, 0.00]),
        _link("link_upper", [[0.45, -0.20, 0.10]], [0.10, -0.45, 0.10]),
        _link("panhard_rod", [[0.20, -0.65, 0.00]], [0.20, 0.65, 0.00]),
    ],
    "trailing_arm": [
        _link("trailing_arm", [[0.50, -0.30, 0.50]], [0.10, -0.65, 0.00]),
    ],
    "semi_trailing_arm": [
        _link("semi_trailing_arm", [[0.50, -0.30, 0.45]], [0.10, -0.65, 0.00]),
    ],
}

# AC TYPE -> required key names that MUST appear for that type.
_REQUIRED_KEYS = {
    0: {"WBCAR_BOTTOM_FRONT", "WBCAR_BOTTOM_REAR", "STRUT_CAR", "STRUT_TYRE"},
    1: {"WBCAR_TOP_FRONT", "WBCAR_TOP_REAR", "WBCAR_BOTTOM_FRONT", "WBCAR_BOTTOM_REAR",
        "WBTYRE_TOP", "WBTYRE_BOTTOM"},
    2: {"WBCAR_TOP_FRONT", "WBCAR_BOTTOM_FRONT", "WBCAR_TIE", "WBTYRE_TOP", "WBTYRE_BOTTOM"},
    3: {"LINK_0_CAR", "LINK_0_AXLE", "PANHARD_CAR", "PANHARD_AXLE"},
    4: {"TRAIL_CAR", "TRAIL_TYRE"},
    5: {"SEMI_TRAIL_CAR", "SEMI_TRAIL_TYRE"},
}


@pytest.mark.parametrize("system_type", list(_FIXTURES))
def test_all_six_types_emit_required_keys(system_type: str, tmp_path: Path) -> None:
    doc = _doc_with_topology(system_type, _FIXTURES[system_type])
    out = suspensions_ini.write(doc, tmp_path)
    parsed = parse_ini(out.read_text(encoding="utf-8"))
    ac_type = SYSTEM_TYPE_TO_AC[system_type]
    assert int(float(parsed["FRONT"]["TYPE"])) == ac_type
    required = _REQUIRED_KEYS[ac_type]
    missing = required - set(parsed["FRONT"])
    assert not missing, f"{system_type}: missing pickup keys {missing}"


# --------------------------------------------------------------------------- #
# Panhard / Watts disambiguation                                              #
# --------------------------------------------------------------------------- #


class TestPanhardWattsDisambiguation:
    def _doc_with_both(self) -> SvjDoc:
        links = list(_FIXTURES["solid_axle"]) + [
            _link("watts_link", [[0.30, -0.30, 0.05]], [0.30, 0.30, 0.05]),
        ]
        return _doc_with_topology("solid_axle", links)

    def test_panhard_emitted_watts_commented(self, tmp_path: Path) -> None:
        doc = self._doc_with_both()
        out = suspensions_ini.write(doc, tmp_path)
        text = out.read_text(encoding="utf-8")
        parsed = parse_ini(text)
        # PANHARD survives parse_ini
        assert "PANHARD_CAR" in parsed["FRONT"]
        # WATTS must NOT be parsed as a real key (because we emit it as a `;` comment)
        assert "WATTS_CAR" not in parsed["FRONT"]
        # ...but the literal line should be in the rendered text.
        assert "; WATTS_CAR=" in text

    def test_warning_logged_to_conversion_log(self, tmp_path: Path) -> None:
        doc = self._doc_with_both()
        # write to a `data/` subdir so the log lands in the parent.
        data_dir = tmp_path / "content_cars" / "car" / "data"
        data_dir.mkdir(parents=True)
        suspensions_ini.write(doc, data_dir)
        log = data_dir.parent / "conversion_log.txt"
        assert log.exists()
        body = log.read_text(encoding="utf-8")
        assert "panhard" in body.lower() and "watts" in body.lower()
