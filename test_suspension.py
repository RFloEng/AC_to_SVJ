"""
Tests for the suspensions.ini -> SVJ corner topology (solid axles, struts, corners
without geometry, spring/damper mounts).  Synthetic data only.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import tempfile

from ac_parsers import (_resolve_lut, parse_bumpstop_tables, parse_damper_tables, parse_ini,
                        parse_suspension_axle)
from converter import _attach_arb, _attach_csp_tables, _interp, build_corner

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
    assert names["strut"]["outboard_ref"] == "hardpoints.strut_lower"
    hp = c["topology"]["upright"]["hardpoints"]
    assert "strut_lower" in hp and hp["strut_top"] == names["strut"]["inboard_points"][0]
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
        assert "inboard_point" not in p and "outboard_point" not in p   # misspellings (0.99.4)


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


def _csp_dir(t) -> Path:
    d = Path(t) / "data"
    (d / "dampers_asym").mkdir(parents=True)
    (d / "front_bs_0.lut").write_text("0|0\n0.1|200\n0.5|600\n", encoding="utf-8")
    (d / "front_rs_0.lut").write_text("0|0\n0.1|-300\n0.5|-900\n", encoding="utf-8")  # sign ignored
    (d / "dampers_asym" / "front_bs_1.lut").write_text("0|0\n0.1|250\n", encoding="utf-8")
    (d / "dampers_asym" / "front_rs_1.lut").write_text("0|0\n0.1|350\n", encoding="utf-8")
    (d / "front_bs_fast_1.lut").write_text("0.5|700\n2|1500\n", encoding="utf-8")
    (d / "rubber.lut").write_text("0|0\n0.01|500\n0.02|2000\n", encoding="utf-8")
    return d


DAMPERS = """[HEADER]
ENABLE=1
[FRONT]
BUMP_SLOW_0=front_bs_0.lut
BUMP_SLOW_1=\\dampers_asym\\front_bs_1.lut
BUMP_FAST_1=front_bs_fast_1.lut
REBOUND_SLOW_0=front_rs_0.lut
REBOUND_SLOW_1=\\dampers_asym\\front_rs_1.lut
"""


def test_damper_tables_are_read_with_sub_folders_and_slow_fast_merge():
    with tempfile.TemporaryDirectory() as t:
        d = _csp_dir(t)
        dt = parse_damper_tables(parse_ini(DAMPERS), d)
        tabs = dt["axles"]["FRONT"]["tables"]
        assert sorted(tabs["bump_slow"]) == [0, 1] and tabs["bump_fast"][1][-1] == [2.0, 1500.0]
        assert _resolve_lut(d, "..\\x.lut") is None            # never leaves data/
        sa_f, sa_r = {}, {}
        parsed = {"dampers.ini": parse_ini(DAMPERS),
                  "setup.ini": parse_ini("[DAMPER_BUMP_LF]\nDEFAULT=1\n[DAMPER_REBOUND_LF]\nDEFAULT=1\n")}
        _attach_csp_tables(sa_f, sa_r, parsed, d, [])
        tab = sa_f["damper_tables"]
        assert tab["bump_index"] == 1 and tab["bump"][-1] == [2.0, 1500.0]   # slow, then fast
        assert tab["bump"][0] == [0.0, 0.0] and "damper_tables" not in sa_r
        assert tab["settings"]["bump_slow"] == [0, 1]
        # without a setup default the lowest setting is used, and the sign is dropped
        sa2: dict = {}
        _attach_csp_tables(sa2, {}, {"dampers.ini": parse_ini(DAMPERS)}, d, [])
        assert sa2["damper_tables"]["rebound"][-1] == [0.5, 900.0]


def test_corner_uses_the_damper_and_bump_stop_tables():
    with tempfile.TemporaryDirectory() as t:
        d = _csp_dir(t)
        sec = parse_ini("[S]\nTYPE=DWB\nBASEY=-0.14\nTRACK=1.5\nSPRING_RATE=20000\n"
                        "DAMP_BUMP=1500\nDAMP_REBOUND=3000\nPACKER_RANGE=0.12\n"
                        "BUMP_STOP_LUT=rubber.lut\n" + WB)["S"]
        sa = parse_suspension_axle(sec)
        _attach_csp_tables(sa, {}, {"dampers.ini": parse_ini(DAMPERS)}, d, [])
        c = build_corner("FL", sa, 2.6, 1.5, 1.5, 0.5, 0.15, 800, 0.6, "x", 0.3, 0.4, {})
        dm = c["damper"]
        assert dm["reference"] == "wheel" and dm["motion_ratio"] == 1.0 and dm["_est"] is False
        assert dm["bump_curve"][1] == [0.1, 200.0] and dm["x_settings"]["bump_slow"] == [0, 1]
        bs = c["bump_stop"]
        assert bs["gap"] == 0.12 and bs["type"] == "rubber" and bs["rate_curve"][-1] == [0.02, 2000.0]


def test_cosmic_coilover_tables_are_component_level():
    with tempfile.TemporaryDirectory() as t:
        d = _csp_dir(t)
        ini = parse_ini(DAMPERS.replace("[FRONT]", "[FRONT_C0]"))
        sa: dict = {}
        _attach_csp_tables(sa, {}, {"dampers.ini": ini}, d, [])
        assert sa["damper_tables"]["coilover"] == 0
        base = parse_suspension_axle(parse_ini("[S]\nTYPE=COSMIC\nBASEY=-0.14\nTRACK=1.5\n")["S"])
        base["damper_tables"] = sa["damper_tables"]
        c = build_corner("FL", base, 2.6, 1.5, 1.5, 0.5, 0.15, 800, 0.6, "x", 0.3, 0.4, {})
        assert c["damper"]["reference"] == "damper"


def test_bumpstops_ini_tables():
    with tempfile.TemporaryDirectory() as t:
        d = _csp_dir(t)
        ini = parse_ini("[HEADER]\nENABLE=1\n[FRONT_C0]\nDEFAULT=0\nSTACK=2\n"
                        "[FRONT_C0_0]\nLUT=rubber.lut\n[HEAVE_FRONT]\nDEFAULT=0\n")
        bt = parse_bumpstop_tables(ini, d)
        assert list(bt) == ["FRONT"] and bt["FRONT"]["stack"] == 2
        assert bt["FRONT"]["luts"][0][-1] == [0.02, 2000.0]


def test_inline_lut_and_interpolation():
    assert _resolve_lut(None, "(|-0.2=1|0=0.85|0.2=1|)") == [[-0.2, 1.0], [0.0, 0.85], [0.2, 1.0]]
    assert _interp([[-0.1, 1.0], [0.1, 0.8]], 0.0) == 0.9
    assert _interp([[0.0, 1.0], [0.1, 0.8]], -5) == 1.0 and _interp([[0.0, 1.0], [0.1, 0.8]], 5) == 0.8


def test_motion_ratio_table_becomes_geometry_and_makes_damper_tables_damper_level():
    with tempfile.TemporaryDirectory() as t:
        d = _csp_dir(t)
        (d / "mr.lut").write_text("-0.05|0.95\n0|0.9\n0.05|0.85\n", encoding="utf-8")
        sec = parse_ini("[S]\nTYPE=DWB\nBASEY=-0.14\nTRACK=1.5\nSPRING_RATE=20000\n"
                        "MOTION_RATIO=mr.lut\n" + WB)["S"]
        sa = parse_suspension_axle(sec)
        _attach_csp_tables(sa, {}, {"dampers.ini": parse_ini(DAMPERS)}, d, [])
        assert sa["motion_ratio_table"][1] == [0.0, 0.9]
        c = build_corner("FL", sa, 2.6, 1.5, 1.5, 0.5, 0.15, 800, 0.6, "x", 0.3, 0.4, {})
        assert c["spring"]["motion_ratio"] == 0.9 and c["spring"]["rate"] == 20000.0   # still the wheel rate
        assert len(c["spring"]["x_motion_ratio_curve"]) == 3
        assert c["damper"]["reference"] == "damper" and c["damper"]["motion_ratio"] == 0.9
        # synthesised (wheel-level) dampers keep ratio 1 and only carry the curve as data
        sa2 = parse_suspension_axle(sec)
        sa2["motion_ratio_table"] = sa["motion_ratio_table"]
        c2 = build_corner("FL", sa2, 2.6, 1.5, 1.5, 0.5, 0.15, 800, 0.6, "x", 0.3, 0.4, {})
        assert c2["damper"]["motion_ratio"] == 1.0 and "x_motion_ratio_curve" in c2["damper"]


def test_anti_roll_bar_comes_from_the_arb_section():
    parsed = {"suspensions.ini": parse_ini("[ARB]\nEXTEND=1\nFRONT=16660\nFRONT_MOTION_RATIO=0.65\nREAR=8798\n")}
    sa_f, sa_r = {}, {}
    _attach_arb(sa_f, sa_r, parsed, [])
    assert sa_f["arb_rate"] == 16660.0 and sa_f["arb_motion_ratio"] == 0.65
    assert sa_r["arb_rate"] == 8798.0 and "arb_motion_ratio" not in sa_r
    sa = parse_suspension_axle(parse_ini("[S]\nTYPE=DWB\nTRACK=1.5\n" + WB)["S"])
    sa.update(sa_f)
    c = build_corner("FL", sa, 2.6, 1.5, 1.5, 0.5, 0.15, 800, 0.6, "x", 0.3, 0.4, {})
    assert c["arb"]["bar_rate"] == 16660.0 and c["arb"]["motion_ratio"] == 0.65 and "_est" not in c["arb"]
    # no [ARB] section: the old placeholder, flagged
    c0 = build_corner("FL", parse_suspension_axle(parse_ini("[S]\nTYPE=DWB\nTRACK=1.5\n" + WB)["S"]),
                      2.6, 1.5, 1.5, 0.5, 0.15, 800, 0.6, "x", 0.3, 0.4, {})
    assert c0["arb"]["_est"] is True


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
