"""
Tests for the suspensions.ini -> SVJ corner topology (solid axles, struts, corners
without geometry, spring/damper mounts).  Synthetic data only.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from ac_parsers import parse_ini, parse_suspension_axle
from converter import build_corner

WB = """WBCAR_TOP_FRONT=0.30,0.11,0.08
WBCAR_TOP_REAR=0.30,0.11,-0.06
WBCAR_BOTTOM_FRONT=0.31,-0.08,0.19
WBCAR_BOTTOM_REAR=0.31,-0.07,-0.09
WBTYRE_TOP=0.15,0.11,0.0
WBTYRE_BOTTOM=0.10,-0.09,0.01
WBCAR_STEER=0.31,-0.02,0.09
WBTYRE_STEER=0.10,-0.03,0.12
"""
STRUT = "STRUT_CAR=0.20,0.30,-0.05\nSTRUT_TYRE=0.10,-0.10,0.03\n"


def _corner(typ: str, body: str, cid: str = "RL"):
    sec = parse_ini(f"[S]\nTYPE={typ}\nBASEY=-0.14\nTRACK=1.5\nSPRING_RATE=20000\n"
                    f"DAMP_BUMP=1500\nDAMP_REBOUND=3000\n{body}")["S"]
    sa = parse_suspension_axle(sec)
    return build_corner(cid, sa, 2.6, 1.5, 1.5, 0.5, 0.15, 800, 0.6, "x", 0.3, 0.4, {})


def _names(c):
    return {L["name"]: L for L in c["topology"]["links"]}


def test_solid_axle_gets_rods_and_an_axle_body():
    c = _corner("AXLE", WB)
    t = c["topology"]
    assert t["system_type"] == "solid_axle"
    names = _names(c)
    assert set(names) == {"upper_link", "lower_link"}          # no wishbones, no toe link
    assert all(L["type"] == "rod" and len(L["inboard_points"]) == 1 for L in names.values())
    assert "upper_link_end" in t["upright"]["hardpoints"]
    assert t["axle_body"]["id"] == "axle_rear"
    # both sides share the same axle body
    assert _corner("AXLE", WB, "RR")["topology"]["axle_body"]["id"] == "axle_rear"


def test_macpherson_has_a_strut_link_and_no_upper_arm():
    c = _corner("STRUT", WB + STRUT, "FL")
    names = _names(c)
    assert "upper_control_arm" not in names and "upper_wishbone" not in names
    assert names["strut"]["type"] == "strut"
    assert names["strut"]["outboard_ref"] == "hardpoints.strut_top"
    assert "strut_top" in c["topology"]["upright"]["hardpoints"]
    assert {"lower_control_arm", "steering_tie_rod"} <= set(names)


def test_macpherson_damper_uses_the_strut_axis():
    c = _corner("STRUT", WB + STRUT, "FL")
    mr = c["spring"]["motion_ratio"]
    assert 0.5 <= mr < 1.0 and c["damper"]["motion_ratio"] == mr
    assert c["spring"]["axis"] == "strut"
    assert c["spring"]["inboard_mount"] == c["damper"]["inboard_mount"]
    # damper curves are on the damper: velocity x MR, force / MR
    ref = _corner("DWB", WB, "FL")["damper"]["bump_curve"]
    got = c["damper"]["bump_curve"]
    assert abs(got[-1][0] - ref[-1][0] * mr) < 1e-3 and abs(got[-1][1] - ref[-1][1] / mr) < 0.1


def test_strut_keys_on_a_wishbone_axle_are_not_a_locating_link():
    c = _corner("DWB", WB + STRUT)
    assert "damper_link" not in _names(c) and "strut" not in _names(c)
    assert c["damper"]["x_ac_strut"]["car"]
    assert {"upper_wishbone", "lower_wishbone", "toe_link"} <= set(_names(c))


def test_placeholder_keeps_the_vertical_axis_with_spec_mount_names():
    c = _corner("DWB", WB)
    for part in ("spring", "damper"):
        p = c[part]
        assert p["axis"] == "vertical" and p["motion_ratio"] == 1.0
        assert p["inboard_mount"] == p["inboard_point"]
        assert p["outboard_mount"] == p["outboard_point"]


def test_no_hardpoints_means_no_links_not_a_virtual_one():
    for typ in ("COSMIC", "AXLE"):
        c = _corner(typ, "")
        assert c["topology"]["links"] == []
        assert "No usable hardpoints" in c["topology"]["_note"]
    assert _corner("AXLE", "")["topology"]["axle_body"]["id"] == "axle_rear"


def test_missing_tie_rod_is_reported():
    body = WB.replace("WBTYRE_STEER=0.10,-0.03,0.12\n", "")
    c = _corner("DWB", body, "FL")
    assert "steering_tie_rod" not in _names(c)
    assert "No steering tie rod" in c["topology"]["_note"]


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
