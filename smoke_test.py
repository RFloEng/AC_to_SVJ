"""Smoke test for AC→SVJ converter beta 0.9 — verifies engine/geometry/
electronics parsing, WBTYRE wheel-space transform, TYPE classifier,
suspensions.ini [BASIC] WHEELBASE lookup, SVJ 0.95 aero shape, and
DWB V-shape geometry."""
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from converter import build_svj, read_car_directory, _clean

def main():
    car = Path(__file__).parent / "test_car"
    ini_files, cm_meta, ctrl_files, data_dir = read_car_directory(car)
    print(f"ini  files loaded: {sorted(ini_files.keys())}")
    print(f"ctrl files loaded: {sorted(ctrl_files.keys())}")
    print(f"data_dir         : {data_dir}")
    print(f"cm metadata      : {'yes' if cm_meta else 'no'}")

    svj, log, bench = build_svj(ini_files, cm_meta,
                                data_dir=data_dir, ctrl_files=ctrl_files)
    svj = _clean(svj)

    print("\n──── LOG ────")
    for line in log:
        print(line)

    print("\n──── STRUCTURAL CHECKS ────")
    e = svj["powertrain"]["engine"]
    dr = svj["powertrain"]["gearbox"]
    x_ac = svj.get("x_assettocorsa", {})
    corner_fl = svj["suspension"]["FL"]
    corner_fr = svj["suspension"]["FR"]
    corner_rl = svj["suspension"]["RL"]
    corner_rr = svj["suspension"]["RR"]
    wb = svj["chassis"]["wheelbase"]
    tf = svj["chassis"]["track_front"]
    tr = svj["chassis"]["track_rear"]

    def hp(c, key):
        return c["topology"]["upright"]["hardpoints"].get(key)

    checks = {
        "metadata ok": svj["_metadata"]["specification"] == "SVJ"
                       and svj["_metadata"]["version"] == "0.95"
                       and "ac_to_svj_converter" in svj["_metadata"]["author"],
        "data_origin present": (
            svj["_metadata"].get("data_origin", {}).get("type") == "simulation"
        ),
        "power.lut parsed (≥10 pts)": len(e["torque_curve"]) >= 10,
        "power.lut source flag set": e["_source"]["power_lut"] is True,
        "coast curve parsed":        e.get("coast_curve") is not None,
        "coast_lut source flag":     e["_source"]["coast_lut"] is True,
        "limiter_hz populated":      e.get("limiter_hz") == 40.0,
        "altitude_sensitivity":      e.get("altitude_sensitivity") == 0.10,
        "fuel_consumption_k":        e.get("fuel_consumption_k") == 0.00005,
        "damage engine captured":    "engine" in x_ac.get("damage", {}),
        "shift_up_time parsed":      dr.get("shift_up_time") == 80.0,
        "gearbox inertia":           dr.get("inertia") == 0.025,
        "diff type LSD":             svj["powertrain"]["differentials"][0]["type"] == "lsd_clutch",
        "driver_aids autoblip":      "autoblip" in x_ac.get("driver_aids", {}),
        "driver_aids autoclutch":    "autoclutch" in x_ac.get("driver_aids", {}),
        "driver_aids profiler":      "shift_profiler" in x_ac.get("driver_aids", {}),
        "drivetrain damage":         "drivetrain" in x_ac.get("damage", {}),
        "hardpoints upper ball":     "upper_ball_joint" in corner_fl["topology"]["upright"]["hardpoints"],
        "hardpoints lower ball":     "lower_ball_joint" in corner_fl["topology"]["upright"]["hardpoints"],
        "links upper wishbone":      any(l["name"] == "upper_wishbone" for l in corner_fl["topology"]["links"]),
        "links lower wishbone":      any("wishbone" in l["name"] or "control_arm" in l["name"]
                                         for l in corner_fl["topology"]["links"]),
        "links steering_tie_rod":    any(l["name"] in ("tie_rod", "steering_tie_rod")
                                         for l in corner_fl["topology"]["links"]),
        "bumpstops carried":         "bumpstops" in corner_fl["damper"],
        "progressive spring":        "progressive_factor" in corner_fl["spring"],
        "rod length":                "rod_length" in corner_fl["spring"],
        "brakes adjustable":         svj["brakes"]["bias_type"] == "adjustable",
        "handbrake parsed":          svj["brakes"].get("handbrake") is not None,
        # v0.99 / SVJ 0.95: wings live in components[] (plus a synthetic body)
        "aero components (≥3)":
            len(svj["aerodynamics"].get("components", []) or []) >= 3,
        "aero body component":
            any(c.get("name") == "body"
                for c in svj["aerodynamics"].get("components", []) or []),
        "aero wing components":
            sum(1 for c in (svj["aerodynamics"].get("components") or [])
                if c.get("type") == "wing") == 2,
        "aero global_coefficients":
            "Cd" in (svj["aerodynamics"].get("global_coefficients") or {}),
        "aero active_systems.drs":
            (svj["aerodynamics"].get("active_systems") or {}).get("drs") is not None,
        "electronics ABS":           "ctrl_abs.ini" in x_ac.get("electronics", {}),
        "electronics TC":            "ctrl_tc.ini" in x_ac.get("electronics", {}),
        "setup ranges":              "setup_ranges" in x_ac and len(x_ac["setup_ranges"]) >= 5,
        "car.ini INFO":              x_ac.get("info", {}).get("author") == "AC_to_SVJ Reference",
        "car.ini controls":          x_ac.get("controls", {}).get("steer_lock") == 540.0,
        "fuel tank position":        "fuel_tank_position" in x_ac,
        "pit_stop parsed":           x_ac.get("pit_stop", {}).get("tyre_change_time") == 20.0,
        "versions block":            "versions" in x_ac,
        # ── v0.96.1: corner positioning + vertical spring/damper ─────────────
        "FL wheel_center Y negative": hp(corner_fl, "wheel_center")[1] < 0,
        "FR wheel_center Y positive": hp(corner_fr, "wheel_center")[1] > 0,
        "RL X at -wheelbase":        abs(hp(corner_rl, "wheel_center")[0] + wb) < 1e-6,
        "RR X at -wheelbase":        abs(hp(corner_rr, "wheel_center")[0] + wb) < 1e-6,
        "FL/FR distinct":            hp(corner_fl, "wheel_center") != hp(corner_fr, "wheel_center"),
        "FL/RL distinct":            hp(corner_fl, "wheel_center") != hp(corner_rl, "wheel_center"),
        "RL/RR distinct":            hp(corner_rl, "wheel_center") != hp(corner_rr, "wheel_center"),
        "spring axis vertical":      corner_fl["spring"].get("axis") == "vertical",
        "spring MR = 1.0":           corner_fl["spring"]["motion_ratio"] == 1.0,
        "damper axis vertical":      corner_fl["damper"].get("axis") == "vertical",
        "damper MR = 1.0":           corner_fl["damper"]["motion_ratio"] == 1.0,
        "spring inboard above out":  corner_fl["spring"]["inboard_point"][2]
                                     <  corner_fl["spring"]["outboard_point"][2],
        "corner.position set":       corner_fl.get("position") is not None,
        # ── v0.97: WBTYRE wheel-space transform ───────────────────────────────
        # Upright ball joints are wheel-space offsets, inboard-positive in AC.
        # After transform they must sit INSIDE the wheel, i.e. |Y_BJ| < |Y_wc|
        # on the same side of the car as the wheel.
        "FR upper_BJ inside FR wheel":
            hp(corner_fr, "upper_ball_joint")[1] > 0
            and abs(hp(corner_fr, "upper_ball_joint")[1])
                < abs(hp(corner_fr, "wheel_center")[1]),
        "FL upper_BJ inside FL wheel":
            hp(corner_fl, "upper_ball_joint")[1] < 0
            and abs(hp(corner_fl, "upper_ball_joint")[1])
                < abs(hp(corner_fl, "wheel_center")[1]),
        "FR lower_BJ inside FR wheel":
            hp(corner_fr, "lower_ball_joint")[1] > 0
            and abs(hp(corner_fr, "lower_ball_joint")[1])
                < abs(hp(corner_fr, "wheel_center")[1]),
        "FL lower_BJ inside FL wheel":
            hp(corner_fl, "lower_ball_joint")[1] < 0
            and abs(hp(corner_fl, "lower_ball_joint")[1])
                < abs(hp(corner_fl, "wheel_center")[1]),
        "RR upper_BJ inside RR wheel":
            hp(corner_rr, "upper_ball_joint")[1] > 0
            and abs(hp(corner_rr, "upper_ball_joint")[1])
                < abs(hp(corner_rr, "wheel_center")[1]),
        "RL upper_BJ inside RL wheel":
            hp(corner_rl, "upper_ball_joint")[1] < 0
            and abs(hp(corner_rl, "upper_ball_joint")[1])
                < abs(hp(corner_rl, "wheel_center")[1]),
        "FL/FR upper_BJ mirrored":
            abs(hp(corner_fl, "upper_ball_joint")[1]
                + hp(corner_fr, "upper_ball_joint")[1]) < 1e-6,
        "Front upper_BJ outside centre (|Y| > 0.5m)":
            abs(hp(corner_fr, "upper_ball_joint")[1]) > 0.5,
        "WBCAR inboard point is chassis-side (|Y| < wheel |Y|)":
            abs(corner_fr["topology"]["links"][0]["inboard_points"][0][1])
              < abs(hp(corner_fr, "wheel_center")[1]),
        "Rear upper_BJ near X=-wheelbase":
            abs(hp(corner_rr, "upper_ball_joint")[0] + wb) < 0.1,
        "Rear upper_BJ Y on same side as rear wheel":
            (hp(corner_rr, "upper_ball_joint")[1]
             * hp(corner_rr, "wheel_center")[1]) > 0,
        "DWB V-shape: front upper inboard wider than lower (|Y_upper| > |Y_lower|)":
            abs(corner_fr["topology"]["links"][0]["inboard_points"][0][1])
              > abs(corner_fr["topology"]["links"][1]["inboard_points"][0][1]),
    }
    for name, ok in checks.items():
        print(f"  {'✓' if ok else '✗'}  {name}")
    if not all(checks.values()):
        print("\n✗ one or more structural checks failed")
        sys.exit(1)

    print("\n──── PACEJKA CHECKS ────")
    ts_names = list(svj["tires"]["sets"].keys())
    print(f"tire sets: {ts_names}")
    for ts_name in ts_names:
        ts = svj["tires"]["sets"][ts_name]
        has = "pacejka_mf52" in ts.get("models", {})
        print(f"  {'✓' if has else '✗'}  '{ts_name}' has models.pacejka_mf52")
        if has:
            mf = ts["models"]["pacejka_mf52"]
            print(f"      lat  R² = {mf['pure_slip_lateral']['_metrics']['r2']:.4f}")
            print(f"      long R² = {mf['pure_slip_longitudinal']['_metrics']['r2']:.4f}")

    print("\n──── BENCH RESULTS ────")
    for axle, br in bench.items():
        print(f"  {axle}: lateral R² = {br.fit_lateral['r2']:.4f}, "
              f"longitudinal R² = {br.fit_longitudinal['r2']:.4f}")

    # Dump the SVJ
    out = Path(__file__).parent / "test_out_svj.json"
    out.write_text(json.dumps(svj, indent=2))
    print(f"\nSVJ written to {out}  ({len(json.dumps(svj))} bytes)")

    print("\n✅ All v0.97 structural checks passed.")

    # ─── v0.98: Xsara real-car validation ───────────────────────────────────
    xsara_dir = Path(__file__).parent / "real_car" / "acra_citroen_xsara_kit_car"
    if xsara_dir.is_dir():
        print("\n──── v0.98 REAL-CAR CHECKS (Citroën Xsara Kit Car) ────")
        ini2, cm2, ctrl2, dd2 = read_car_directory(xsara_dir)
        svj2, log2, _ = build_svj(ini2, cm2, data_dir=dd2, ctrl_files=ctrl2)
        svj2 = _clean(svj2)
        x_ac2 = svj2.get("x_assettocorsa", {})
        rc = {
            "Xsara wheelbase == 2.563":
                abs(svj2["chassis"]["wheelbase"] - 2.563) < 1e-6,
            "Xsara RR X near -2.563":
                abs(svj2["suspension"]["RR"]["topology"]["upright"]
                        ["hardpoints"]["wheel_center"][0] + 2.563) < 1e-6,
            "Xsara front susp == macpherson (STRUT)":
                svj2["suspension"]["FL"]["topology"]["system_type"] == "macpherson",
            "Xsara rear susp == double_wishbone (DWB)":
                svj2["suspension"]["RL"]["topology"]["system_type"] == "double_wishbone",
            "Xsara upper_BJ inside all 4 wheels": all(
                abs(svj2["suspension"][cn]["topology"]["upright"]
                        ["hardpoints"]["upper_ball_joint"][1])
                < abs(svj2["suspension"][cn]["topology"]["upright"]
                          ["hardpoints"]["wheel_center"][1])
                for cn in ("FL", "FR", "RL", "RR")
            ),
        }
        for name, ok in rc.items():
            print(f"  {'✓' if ok else '✗'}  {name}")
        if not all(rc.values()):
            print("\n✗ v0.98 real-car checks failed")
            sys.exit(1)

    # Loader behaviour: open_car on a missing car returns an empty ini
    # map and a clear source-note, so callers in convert_single_dir /
    # convert_batch can surface that to the user without crashing.
    print("\n──── Loader behaviour ────")
    from acd_reader import open_car, UnpackNeededError
    fake = Path(__file__).parent / "_no_such_car"
    inis, _, note = open_car(fake)
    print(f"  ✓  open_car on missing car returns empty + note ({note!r})")

    print("\n✅ All structural checks passed.")

if __name__ == "__main__":
    main()
