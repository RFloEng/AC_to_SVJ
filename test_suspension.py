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
from converter import (_attach_arb, _attach_cosmic, _attach_csp_tables, _csp_physics_flags,
                       _interp, build_corner)
from cosmic import reduce_cosmic

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


COSMIC_DWB = """[FRONT]
TYPE=COSMIC
BASEY=-0.14
TRACK=1.6
DJ0=UCA_FRONT
DJ0_BODY_A=CHASSIS
DJ0_BODY_B=HUB_L
DJ0_POS_A=0.35, 0.12, 0.14
DJ0_POS_B=0.15, 0.12, -0.02
DJ1=UCA_REAR
DJ1_POS_A=0.35, 0.09, -0.08
DJ1_POS_B=0.15, 0.12, -0.02
J0=LCA_FRONT
J0_BODY_A=CAMBER_BODY
J0_BODY_B=LCA_BODY
J0_POS=0.43, -0.15, 0.26
J1=LCA_REAR
J1_BODY_A=CAMBER_BODY
J1_BODY_B=LCA_BODY
J1_POS=0.43, -0.15, -0.06
J2=LCA_BJ
J2_BODY_A=LCA_BODY
J2_POS=0.09, -0.14, 0.0
STEER_JOINT_0=STEER
STEER_JOINT_0_POS_A=0.37, 0.04, 0.10
STEER_JOINT_0_POS_B=0.09, 0.07, 0.13
BODY_0_NAME=CAMBER_BODY
BODY_1_NAME=LCA_BODY
SLIDER_0=CAMBER_SLIDER
SLIDER_0_BODY_A=CHASSIS
SLIDER_0_BODY_B=CAMBER_BODY
SLIDER_0_POS_A=0.53, -0.15, 0.10
SLIDER_0_POS_B=0.43, -0.15, 0.10
DJ2=CAMBER_ADJUSTER
DJ2_BODY_A=CHASSIS
DJ2_BODY_B=CAMBER_BODY
DJ2_POS_A=0.53, -0.15, 0.10
DJ2_POS_B=0.43, -0.15, 0.10
[FRONT_COILOVER_0]
BODY_A=CHASSIS
BODY_B=LCA_BODY
POS_A=0.33, 0.17, 0.0
POS_B=0.17, -0.15, 0.0
RATE=40000
[FRONT_SPRING_0]
BODY_A=CHASSIS
BODY_B=HUB_L
POS_A=0, 1, 0
POS_B=0, 0, 0
RATE=8000
"""

COSMIC_STRUT = """[FRONT]
TYPE=COSMIC
BASEY=-0.19
TRACK=1.5
BODY_0_NAME=LCA_IN
BODY_1_NAME=LCA_OUT
BODY_2_NAME=STRUT_BODY
J0=LCA_FRONT
J0_BODY_A=CHASSIS
J0_BODY_B=LCA_IN
J0_POS=0.43, -0.13, 0.30
J1=LCA_REAR
J1_BODY_A=CHASSIS
J1_BODY_B=LCA_IN
J1_POS=0.43, -0.13, -0.02
J2=LCA_BJ
J2_BODY_A=LCA_OUT
J2_POS=0.11, -0.13, 0.03
J3=STRUT_TOP
J3_BODY_A=CHASSIS
J3_BODY_B=STRUT_BODY
J3_POS=0.31, 0.43, -0.02
DJ0=ADJ
DJ0_BODY_A=LCA_IN
DJ0_BODY_B=LCA_OUT
DJ0_POS_A=0.11, -0.13, -0.07
DJ0_POS_B=0.11, -0.13, 0.03
SLIDER_0=ADJ_SLIDER
SLIDER_0_BODY_A=LCA_IN
SLIDER_0_BODY_B=LCA_OUT
SLIDER_0_POS_A=0.11, -0.13, -0.07
SLIDER_0_POS_B=0.11, -0.13, 0.03
SLIDER_1=STRUT_SLIDER
SLIDER_1_BODY_A=STRUT_BODY
SLIDER_1_POS_A=0.31, 0.43, -0.02
SLIDER_1_POS_B=0.17, -0.01, -0.07
STEER_JOINT_0=TOE
STEER_JOINT_0_POS_A=0.45, -0.11, -0.09
STEER_JOINT_0_POS_B=0.14, -0.09, -0.11
[FRONT_COILOVER_0]
BODY_A=STRUT_BODY
POS_A=0.31, 0.43, -0.02
POS_B=0.17, -0.01, -0.07
RATE=29000
"""


def test_cosmic_double_wishbone_is_reduced_to_arms_and_a_tie_rod():
    r = reduce_cosmic(parse_ini(COSMIC_DWB), "FRONT")
    assert r["system_type"] == "double_wishbone" and not r["notes"]
    by = {l["name"]: l for l in r["links"]}
    assert set(by) == {"upper_wishbone", "lower_wishbone", "steering_tie_rod"}
    assert len(by["upper_wishbone"]["inboard"]) == 2 and by["upper_wishbone"]["end"] == [0.15, 0.12, -0.02]
    assert by["lower_wishbone"]["end"] == [0.09, -0.14, 0.0]        # through the adjuster-merged body
    assert len(by["lower_wishbone"]["inboard"]) == 2
    (c,) = r["coilovers"]
    assert 0.3 < c["motion_ratio"] < 1.0 and abs(c["wheel_rate"] - 40000 * c["motion_ratio"] ** 2) < 5
    assert r["extra_wheel_rate"] == 8000.0


def test_cosmic_strut_is_a_macpherson_with_the_real_damper():
    r = reduce_cosmic(parse_ini(COSMIC_STRUT), "FRONT")
    assert r["system_type"] == "macpherson", r
    by = {l["name"]: l for l in r["links"]}
    assert by["strut"]["type"] == "strut" and by["strut"]["end"] == [0.17, -0.01, -0.07]
    assert by["strut"]["inboard"] == [[0.31, 0.43, -0.02]]
    (c,) = r["coilovers"]
    assert c["rate"] == 29000.0 and 0.9 < c["motion_ratio"] <= 1.0     # near-vertical strut


def test_cosmic_corner_gets_links_mounts_and_a_wheel_rate():
    ini = parse_ini(COSMIC_DWB)
    sa = parse_suspension_axle(ini["FRONT"])
    _attach_cosmic(sa, {}, {"suspensions.ini": ini}, [])
    c = build_corner("FL", sa, 2.6, 1.5, 1.5, 0.5, 0.15, 800, 0.6, "x", 0.3, 0.4, {})
    t = c["topology"]
    assert t["system_type"] == "double_wishbone" and len(t["links"]) == 3
    assert "upper_ball_joint" in t["upright"]["hardpoints"] and "_note" not in t
    mr = c["spring"]["motion_ratio"]
    assert c["spring"]["axis"] == "strut" and c["spring"]["inboard_mount"] != c["spring"]["outboard_mount"]
    assert abs(c["spring"]["rate"] - (40000 * mr * mr + 8000)) < 5          # component -> wheel rate


def test_unrecognised_cosmic_axle_says_so_instead_of_guessing():
    ini = parse_ini("[REAR]\nTYPE=COSMIC\nBODY_0_NAME=DEDION\nDJ0=TOP\nDJ0_BODY_A=DEDION\n"
                    "DJ0_BODY_B=CHASSIS\nDJ0_POS_A=0.7, 0.07, -0.04\nDJ0_POS_B=0.7, 0.07, 0.5\n")
    r = reduce_cosmic(ini, "REAR")
    assert r["system_type"] == "custom" and r["links"] == [] and r["notes"]


def test_csp_physics_flags_are_kept_as_written():
    parsed = {
        "suspensions.ini": parse_ini("[_EXTENSION]\nTORQUE_MODE_EX=2\nFIX_PROGRESSIVE_RATE=1\n[FRONT]\nTYPE=DWB\n"),
        "engine.ini": parse_ini("[TURBO_0]\nMAX_BOOST=1\nEXT_SPIN_DELAY=0.1\nEXT_GAS_CURVE=(|0=0|1=1|)\n"),
        "tyres.ini": parse_ini("[FRONT]\nWIDTH=0.2\n"),
    }
    f = _csp_physics_flags(parsed)
    assert f["suspensions.ini"] == {"_EXTENSION": {"TORQUE_MODE_EX": "2", "FIX_PROGRESSIVE_RATE": "1"}}
    assert f["engine.ini"] == {"TURBO_0": {"EXT_SPIN_DELAY": "0.1", "EXT_GAS_CURVE": "(|0=0|1=1|)"}}
    assert "tyres.ini" not in f and _csp_physics_flags({}) == {}


def test_repeated_bump_stop_points_stay_a_rising_curve():
    sec = parse_ini("[S]\nTYPE=DWB\nBASEY=-0.14\nTRACK=1.5\nSPRING_RATE=20000\nDAMP_BUMP=1500\nDAMP_REBOUND=3000\n" + WB)["S"]
    sa = parse_suspension_axle(sec)
    sa["bumpstop_table"] = {"stack": 1.0, "curve": [(0.0, 0.0), (0.01, 10.0), (0.02, 40.0),
                                                     (0.02, 40.0), (0.02, 40.0)]}
    sa["packer_range"] = 0.03
    c = build_corner("RL", sa, 2.6, 1.5, 1.5, 0.5, 0.15, 800, 0.6, "x", 0.3, 0.4, {})
    assert c["bump_stop"]["rate_curve"] == [[0.0, 0.0], [0.01, 10.0], [0.02, 40.0]]


def test_zero_front_track_is_written_as_a_small_positive_value():
    from converter import build_svj, read_car_directory
    src = Path(__file__).parent / "test_car"
    if not src.is_dir():
        return
    ini, cm, ctrl, dd = read_car_directory(src)
    ini = dict(ini)
    ini["suspensions.ini"] = ini["suspensions.ini"].replace("TRACK=", "TRACK=0 ;", 1)
    svj, log, _ = build_svj(ini, cm, data_dir=dd, ctrl_files=ctrl, unified_suspension=False)
    assert 0 < svj["chassis"]["track_front"] <= 0.01
    assert any("one front wheel" in l for l in log)


def test_unified_suspension_output():
    """build_svj writes bodies, joints and wheel_stations (spec 25) and no corner section;
    unified_suspension=False keeps the deprecated section."""
    from converter import build_svj, read_car_directory, _clean
    src = Path(__file__).parent / "test_car"
    if not src.is_dir():
        return
    ini, cm, ctrl, dd = read_car_directory(src)
    old, _, _ = build_svj(ini, cm, data_dir=dd, ctrl_files=ctrl, unified_suspension=False)
    new, log, _ = build_svj(ini, cm, data_dir=dd, ctrl_files=ctrl)
    assert "suspension" in old and old["_metadata"]["version"] == "0.100"
    assert "suspension" not in new, "corner section still present"
    ws = {w["id"]: w for w in new["wheel_stations"]}
    assert set(ws) == {"FL", "FR", "RL", "RR"}
    bodies = {b["id"]: b for b in new["chassis"]["mass_bodies"]}
    for st in ws.values():
        assert st["wheel_body"] in bodies and st["upright_body"] in bodies
        assert st["tire"]["set_ref"] and st["brake"]
    ids = {c["id"] for c in new["constraints"]}
    assert all(w["steering_joint"] in ids for w in ws.values() if "steering_joint" in w)
    kinds = {c["type"] for c in new["constraints"]}
    assert "joint" in kinds and ("spring" in kinds or "damper" in kinds)
    assert any("Suspension written as bodies" in l for l in log)
    # same tyre/brake data as the corner section carried
    co, cn = _clean(old), _clean(new)
    assert all(cn[k] == co[k] for k in ("tires", "powertrain", "brakes", "aerodynamics"))

NL = chr(10)


def test_ac_damper_is_two_slopes_and_vanilla_bump_stops_are_kept():
    from converter import ac_damper_curve
    assert ac_damper_curve(2244, 1320, 0.03) == [[0.0, 0], [0.03, 67.32], [0.5, 687.72]]
    assert ac_damper_curve(2000, None, None) == [[0.0, 0], [0.5, 1000.0]]       # straight line
    body = NL.join(["DAMP_BUMP=2244", "DAMP_REBOUND=6500", "DAMP_FAST_BUMP=1320",
                    "DAMP_FAST_BUMPTHRESHOLD=0.03", "BUMP_STOP_RATE=35000", "BUMPSTOP_UP=0.06",
                    "BUMPSTOP_DN=0.105", "PACKER_RANGE=0.28"]) + NL
    c = _corner("DWB", body, "FL")
    assert c["bump_stop"] == {"gap": 0.06, "type": "rubber", "rate": 35000.0}
    assert c["rebound_stop"]["gap"] == 0.105
    assert c["static_setup"]["x_packer_range"] == 0.28
    assert c["damper"]["bump_curve"] == [[0.0, 0], [0.03, 67.32], [0.5, 687.72]]


def test_static_camber_is_read_in_degrees():
    import math
    sec = parse_ini("[S]\nTYPE=DWB\nSTATIC_CAMBER=-2.0\n")["S"]
    assert abs(parse_suspension_axle(sec)["camber"] - math.radians(-2.0)) < 1e-9
    sec = parse_ini("[S]\nTYPE=DWB\nCAMBER=-0.015\n")["S"]          # as written
    assert parse_suspension_axle(sec)["camber"] == -0.015


# ── derived geometry (susp_geometry) ─────────────────────────────────────────────────
import math

from susp_geometry import anti_geometry


def test_anti_dive_and_anti_squat():
    # IC 1 m behind the front wheel and 0.3 m up; wheelbase 2.6, CG 0.5, 60 % front brakes
    a = anti_geometry([-1.0, -0.30], 0.0, 2.6, 0.5, True, 0.6)
    assert abs(a["anti_dive_percent"] - 0.3 * 5.2 * 100 * 0.6) < 1e-9
    # IC 1 m ahead of the rear wheel and 0.3 m up
    a = anti_geometry([-1.6, -0.30], -2.6, 2.6, 0.5, False, 0.6)
    assert abs(a["anti_squat_percent"] - 0.3 * 5.2 * 100) < 1e-9
    # IC almost above the wheel: ill-conditioned, left out
    assert anti_geometry([-2.55, -0.30], -2.6, 2.6, 0.5, False, 0.6) is None
    # IC ahead of a front wheel: pro-dive (negative)
    assert anti_geometry([1.0, -0.30], 0.0, 2.6, 0.5, True, 0.6)["anti_dive_percent"] < 0


def test_corner_carries_the_side_view_geometry():
    c = _corner("DWB", WB, "FL")
    al = c["alignment"]
    assert "caster_trail" not in al and "scrub_radius" not in al    # the loop solver's job
    g = al.get("x_geometry", {})
    assert "roll_centre_height" not in g and "kingpin" not in g
    # nothing to derive from on a corner without links
    assert "x_geometry" not in _corner("DWB", "", "FL")["alignment"]
    assert c["wheel"]["loaded_radius"] == 0.3            # what the solver reads



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
