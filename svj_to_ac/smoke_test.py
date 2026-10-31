"""Smoke test for SVJ_to_AC beta 0.1.0 -- exercises loader, every writer,
end-to-end conversion, reproducible zip, FBX skeleton, Pacejka fast
path, and the no-inline-units guard rail.

Mirrors the structure of ``AC_to_SVJ/smoke_test.py``: a single script,
no pytest, numbered checks, one pass/fail line per check, exit code
reflects total failures.

Usage::

    python smoke_test.py        # all checks
    python smoke_test.py -v     # plus stack traces on failure
"""

from __future__ import annotations

import random
import re
import sys
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parent
AC_TO_SVJ = REPO.parent          # AC_to_SVJ is the repository this folder lives in
MX5 = REPO / "examples" / "mx5_nd_club.svj.json"

sys.path.insert(0, str(REPO))
if AC_TO_SVJ.is_dir():
    sys.path.append(str(AC_TO_SVJ))


VERBOSE = "-v" in sys.argv or "--verbose" in sys.argv


# --------------------------------------------------------------------------- #
# Tiny check harness                                                          #
# --------------------------------------------------------------------------- #


class Result:
    def __init__(self) -> None:
        self.passed = 0
        self.failed = 0

    def check(self, name: str, condition: bool, detail: str = "") -> None:
        if condition:
            self.passed += 1
            mark = "PASS"
        else:
            self.failed += 1
            mark = "FAIL"
        suffix = f"  -- {detail}" if detail else ""
        print(f"  [{mark}] {name}{suffix}")

    def run(self, name: str, fn) -> None:
        try:
            fn()
        except Exception as exc:  # noqa: BLE001
            self.failed += 1
            print(f"  [FAIL] {name}  -- {type(exc).__name__}: {exc}")
            if VERBOSE:
                import traceback
                traceback.print_exc()


# --------------------------------------------------------------------------- #
# Checks                                                                      #
# --------------------------------------------------------------------------- #


def section(title: str) -> None:
    print(f"\n--- {title} ---")


def main() -> int:
    res = Result()
    print("SVJ_to_AC v0.1.0 -- smoke test")
    print(f"repo: {REPO}")
    print(f"AC_to_SVJ sibling: {AC_TO_SVJ.is_dir()}")

    # ---------- 1. Loader ----------
    section("1. Loader")
    import svj_loader

    def _load_mx5():
        return svj_loader.load(MX5)

    try:
        doc = _load_mx5()
        res.check("loads mx5_nd_club.svj.json", doc.vehicle_info.make == "Mazda")
        res.check("metadata version 0.95", doc.metadata.version == "0.95")
        res.check("validate() returns no errors", svj_loader.validate(doc) == [],
                  detail=f"{len(svj_loader.validate(doc))} errors")
        res.check("required sections present",
                  all(getattr(doc, s) is not None
                      for s in ("vehicle_info", "chassis", "suspension",
                                "tires", "powertrain", "brakes")))
    except Exception as exc:
        res.failed += 1
        print(f"  [FAIL] loader crashed: {exc}")
        return 1  # nothing else will work

    # ---------- 2. Coords + Units ----------
    section("2. Coords + Units")
    import coords
    import units

    rng = random.Random(1)
    pts = [(rng.uniform(-100, 100), rng.uniform(-100, 100), rng.uniform(-100, 100))
           for _ in range(100)]
    out = coords.ac_to_svj_cloud(coords.svj_to_ac_cloud(pts))
    max_err = max(abs(a - b) for p, o in zip(pts, out) for a, b in zip(p, o))
    res.check("100-point round-trip < 1e-9", max_err < 1e-9, f"max_err={max_err:.2e}")

    res.check("rad <-> deg roundtrip", abs(units.deg_to_rad(units.rad_to_deg(1.234)) - 1.234) < 1e-12)
    res.check("rad/s <-> rpm roundtrip", abs(units.rpm_to_radps(units.radps_to_rpm(6800)) - 6800) < 1e-9)
    res.check("Pa <-> psi roundtrip", abs(units.psi_to_pa(units.pa_to_psi(200000)) - 200000) < 1e-6)
    res.check("K <-> C roundtrip", abs(units.c_to_k(units.k_to_c(323.15)) - 323.15) < 1e-9)

    # ---------- 3. Pickup-point convention ----------
    section("3. Pickup-point convention")
    ax = coords.pickup_lateral_to_ac(0.75, 0.78)
    res.check("pickup_lateral_to_ac(0.75, 0.78) == 0.03", abs(ax - 0.03) < 1e-9)
    res.check("pickup_z_to_ac(0.32) == -0.32", coords.pickup_z_to_ac(0.32) == -0.32)

    # ---------- 4. INI writers via AC_to_SVJ parsers ----------
    section("4. INI writers (round-trip via AC_to_SVJ parsers)")
    if not AC_TO_SVJ.is_dir():
        print("  [SKIP] AC_to_SVJ sibling missing -- skipping round-trip checks")
    else:
        from ac_parsers import parse_aero, parse_brakes, parse_drivetrain, parse_ini
        from ac_writers import (
            aero_ini, brakes_ini, car_ini, drivetrain_ini,
            electronics_ini, engine_ini, setup_ini, suspensions_ini,
            tyres_ini, ui_car_json,
        )
        AC_DATA = AC_TO_SVJ / "test_car" / "data"

        def _native_doc(blocks):
            from svj_loader import (
                SvjAssettoExt, SvjBrakes, SvjChassis, SvjCorner, SvjDoc,
                SvjMetadata, SvjPowertrain, SvjSuspension, SvjTires,
                SvjVehicleInfo,
            )
            return SvjDoc(
                vehicle_info=SvjVehicleInfo("X", "Y", 2020, "v", "RWD"),
                chassis=SvjChassis(1000.0, 2.3, 1.5, 1.5, [0.0, 0.0, -0.4]),
                suspension=SvjSuspension(corners={"FL": SvjCorner("FL", [0, 0, 0])}),
                tires=SvjTires(sets={"front": {}}),
                powertrain=SvjPowertrain(layout="FR"),
                brakes=SvjBrakes(0.6, "fixed"),
                metadata=SvjMetadata(version="0.95"),
                assetto=SvjAssettoExt(raw={"_ac_native": blocks}),
            )

        with tempfile.TemporaryDirectory() as td:
            tmp = Path(td)
            for name, writer, parse_fn in (
                ("car",      car_ini,         None),
                ("brakes",   brakes_ini,      parse_brakes),
                ("aero",     aero_ini,        None),
                ("setup",    setup_ini,       None),
            ):
                orig = parse_ini((AC_DATA / f"{name}.ini").read_text(encoding="utf-8"))
                d = _native_doc({name: orig})
                out_path = writer.write(d, tmp)
                got = parse_ini(out_path.read_text(encoding="utf-8"))
                res.check(f"{name}.ini parse_ini dict-equal", _normalize(orig) == _normalize(got))
                if parse_fn is not None:
                    res.check(f"{name}.ini parse_{name} semantic-equal",
                              parse_fn(orig) == parse_fn(got))

            # aero semantic via parse_aero(.., None, None)
            orig = parse_ini((AC_DATA / "aero.ini").read_text(encoding="utf-8"))
            d = _native_doc({"aero": orig})
            out_path = aero_ini.write(d, tmp)
            got = parse_ini(out_path.read_text(encoding="utf-8"))
            res.check("aero.ini parse_aero semantic-equal",
                      parse_aero(orig, None, None) == parse_aero(got, None, None))

            # suspensions via parse_suspension_axle per axle
            from ac_parsers import parse_suspension_axle
            orig = parse_ini((AC_DATA / "suspensions.ini").read_text(encoding="utf-8"))
            d = _native_doc({"suspensions": orig})
            out_path = suspensions_ini.write(d, tmp)
            got = parse_ini(out_path.read_text(encoding="utf-8"))
            res.check("suspensions FRONT parse-equal",
                      parse_suspension_axle(orig["FRONT"]) == parse_suspension_axle(got["FRONT"]))
            res.check("suspensions REAR parse-equal",
                      parse_suspension_axle(orig["REAR"]) == parse_suspension_axle(got["REAR"]))

            # drivetrain semantic
            orig = parse_ini((AC_DATA / "drivetrain.ini").read_text(encoding="utf-8"))
            d = _native_doc({"drivetrain": orig})
            out_path = drivetrain_ini.write(d, tmp)
            got = parse_ini(out_path.read_text(encoding="utf-8"))
            res.check("drivetrain.ini parse_drivetrain semantic-equal",
                      parse_drivetrain(orig) == parse_drivetrain(got))

            # engine + companion power.lut
            from ac_parsers import parse_engine, parse_lut
            orig = parse_ini((AC_DATA / "engine.ini").read_text(encoding="utf-8"))
            power_lut = parse_lut((AC_DATA / "power.lut").read_text(encoding="utf-8"))
            d = _native_doc({"engine": orig})
            d.assetto.raw["engine_luts"] = {"power.lut": power_lut}
            out_path = engine_ini.write(d, tmp)
            got_ini = parse_ini(out_path.read_text(encoding="utf-8"))
            got_lut = parse_lut((tmp / "power.lut").read_text(encoding="utf-8"))
            res.check("engine.ini parse_ini dict-equal", _normalize(orig) == _normalize(got_ini))
            res.check("power.lut parse_lut dict-equal", got_lut == power_lut)

            # tyres fast path -- log must NOT contain pacejka warning
            d2 = svj_loader.load(MX5)
            log = tmp / "conversion_log.txt"
            if log.exists():
                log.unlink()
            tyres_ini.write(d2, tmp)
            warning_emitted = (
                log.exists() and "WARNING [pacejka]" in log.read_text(encoding="utf-8")
            )
            res.check("tyres.ini fast path (no Pacejka warning)", not warning_emitted)

            # electronics + ctrl
            d3 = _native_doc({})
            d3.assetto.driver_aids = {"abs": {"present": True, "active": True}}
            electronics_ini.write(d3, tmp)
            res.check("electronics.ini renders without crashing",
                      (tmp / "electronics.ini").is_file())

            d4 = _native_doc({})
            d4.assetto.controls = {"abs": {"version": 1, "controllers": [
                {"input": "SPEED", "combinator": "ADD", "lut": [[0, 0], [80, 1]]}
            ]}}
            from ac_writers import ctrl_files
            ctrl_files.write(d4, tmp)
            res.check("ctrl_abs.ini renders without crashing",
                      (tmp / "ctrl_abs.ini").is_file())

            # ui_car.json
            ui_car_json.write(d2, tmp)
            import json as _json
            j = _json.loads((tmp / "ui_car.json").read_text(encoding="utf-8"))
            res.check("ui_car.json has canonical fields",
                      all(k in j for k in ("name", "brand", "year", "class", "specs")))

    # ---------- 5. End-to-end conversion ----------
    section("5. End-to-end conversion")
    import pack_car
    with tempfile.TemporaryDirectory() as td:
        out_dir = Path(td)
        result = pack_car.assemble(doc, out_dir)
        required = {
            "data/car.ini", "data/suspensions.ini", "data/tyres.ini",
            "data/engine.ini", "data/power.lut", "data/drivetrain.ini",
            "data/aero.ini", "data/brakes.ini", "data/electronics.ini",
            "data/setup.ini", "ui/ui_car.json", "ui/badge.png",
            "ui/preview.jpg", "sfx/GUIDs.txt",
        }
        emitted = {p.relative_to(result.car_dir).as_posix() for p in result.files}
        missing = required - emitted
        res.check("all 14 required files emitted", not missing,
                  detail=f"missing {missing}" if missing else f"{len(emitted)} files")

        zip_path = pack_car.convert(doc, out_dir / "zipdir")
        res.check("convert() produces a zip", zip_path.is_file() and zip_path.stat().st_size > 1000,
                  detail=f"{zip_path.stat().st_size} bytes")

    # ---------- 6. Reproducible zip ----------
    section("6. Reproducible zip")
    with tempfile.TemporaryDirectory() as td:
        z1 = pack_car.convert(doc, Path(td) / "r1")
        z2 = pack_car.convert(doc, Path(td) / "r2")
        sha1 = pack_car.sha256_of(z1)
        sha2 = pack_car.sha256_of(z2)
        res.check("two runs produce byte-identical zip", sha1 == sha2,
                  detail=f"sha1={sha1[:10]}.. sha2={sha2[:10]}..")

    # ---------- 7. FBX skeleton ----------
    section("7. FBX skeleton")
    import fbx_skeleton
    with tempfile.TemporaryDirectory() as td:
        tmp = Path(td)
        fbx_path = fbx_skeleton.write(doc, tmp / "car.fbx")
        text = fbx_path.read_text(encoding="ascii")
        res.check("FBXVersion 7400 (FBX 2014)", "FBXVersion: 7400" in text)
        res.check("Z-up axis (UpAxis=1, FrontAxis=2)",
                  re.search(r'"UpAxis".*?,\s*1\b', text)
                  and re.search(r'"FrontAxis".*?,\s*2\b', text))
        res.check("UnitScaleFactor=1.0",
                  re.search(r'"UnitScaleFactor".*?,\s*1\b', text) is not None)
        for n in ("WHEEL_LF", "WHEEL_RF", "WHEEL_LR", "WHEEL_RR",
                  "SUSP_LF", "SUSP_RF", "SUSP_LR", "SUSP_RR",
                  "COCKPIT_HR", "STEER_HR", "BODY"):
            res.check(f"FBX dummy {n} present", f'"Model::{n}"' in text)
        for w in ("LF", "RF", "LR", "RR"):
            res.check(f"FBX wheel mesh WHEEL_{w}_MESH present",
                      f'"Geometry::WHEEL_{w}_MESH"' in text)
        # Companion files
        fbx_skeleton.write_lods_ini(tmp / "lods.ini")
        fbx_skeleton.write_ksanim_placeholder(tmp / "persistence.ksanim")
        res.check("lods.ini has [LOD_0]",
                  "[LOD_0]" in (tmp / "lods.ini").read_text(encoding="utf-8"))
        res.check("persistence.ksanim placeholder created",
                  (tmp / "persistence.ksanim").is_file())

    # ---------- 8. Pacejka inverse fast path ----------
    section("8. Pacejka inverse fast path")
    import pacejka_inverse
    front_key = next(k for k in doc.tires.sets if "front" in k)
    params = pacejka_inverse.fit_ac_params(doc.tires.sets[front_key])
    res.check("fast path returns FZ0", "FZ0" in params and params["FZ0"] > 0)
    res.check("fast path returns DY0 within [0.5, 3.0]", 0.5 <= params["DY0"] <= 3.0)

    # ---------- 9. Single-source-of-truth guard rails ----------
    section("9. Guard rails")
    # No banned constants outside units.py / tests/test_units.py / test_no_inline_units.py.
    units_path = REPO / "units.py"
    excluded = {units_path, REPO / "tests" / "test_units.py", REPO / "smoke_test.py",
                REPO / "tests" / "test_no_inline_units.py"}
    excluded_dirs = {"examples", "test_car", "assets", "__pycache__", ".pytest_cache"}
    BANNED = ("57.29", "57.2958", "6894.7", "273.15", "0.0254", "0.453592",
              "0.745699", "0.000145", "0.000145038", "9.80665")
    offenders: list[tuple[Path, int]] = []
    for p in REPO.rglob("*.py"):
        if any(part in excluded_dirs for part in p.parts) or p in excluded:
            continue
        for n, line in enumerate(p.read_text(encoding="utf-8").splitlines(), 1):
            code = line.split("#", 1)[0]
            if any(b in code for b in BANNED):
                offenders.append((p, n))
                break
    res.check("no inline unit constants outside units.py",
              not offenders, detail=f"{len(offenders)} offender(s)" if offenders else "")

    # No null-byte corruption (Synology sync glitch sanity).
    null_offenders: list[Path] = []
    for p in REPO.rglob("*.py"):
        if any(part in excluded_dirs for part in p.parts):
            continue
        if b"\x00" in p.read_bytes():
            null_offenders.append(p)
    res.check("no null-byte corruption in any .py file", not null_offenders,
              detail=", ".join(str(p.name) for p in null_offenders))

    # ---------- Summary ----------
    print()
    print(f"==== {res.passed} passed, {res.failed} failed ====")
    return 0 if res.failed == 0 else 1


def _normalize(parsed: dict[str, dict[str, str]]) -> dict[str, dict[str, object]]:
    out: dict[str, dict[str, object]] = {}
    for sec, kv in parsed.items():
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


if __name__ == "__main__":
    raise SystemExit(main())
