"""Stage-6 round-trip tests: engine, drivetrain, electronics, setup,
ctrl_*, tyres.

Same strategy as ``test_writers_basic.py``: native pass-through via
``_ac_native`` -> writer -> reparse -> deep-equal; plus a canonical
SVJ-fallback smoke test per writer.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import pytest

import svj_loader
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

from ac_writers import (
    ctrl_files,
    drivetrain_ini,
    electronics_ini,
    engine_ini,
    setup_ini,
    tyres_ini,
)
import pacejka_inverse

from ac_parsers import (  # noqa: E402
    parse_drivetrain,
    parse_engine,
    parse_ini,
    parse_lut,
)

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
            try:
                out[sec.upper()][k.upper()] = float(s)
            except ValueError:
                out[sec.upper()][k.upper()] = s
    return out


def _native_doc(native: dict[str, dict[str, dict[str, str]]]) -> SvjDoc:
    ext = SvjAssettoExt(raw={"_ac_native": native})
    return SvjDoc(
        vehicle_info=SvjVehicleInfo("X", "Y", 2020, "v", "RWD"),
        chassis=SvjChassis(1058.0, 2.31, 1.495, 1.505, [0.0, 0.0, -0.46]),
        suspension=SvjSuspension(corners={"FL": SvjCorner("FL", [0, 0, 0])}),
        tires=SvjTires(sets={"front": {}}),
        powertrain=SvjPowertrain(layout="FR"),
        brakes=SvjBrakes(0.6, "fixed"),
        metadata=SvjMetadata(version="0.95"),
        assetto=ext,
    )


# --------------------------------------------------------------------------- #
# engine.ini + power.lut                                                      #
# --------------------------------------------------------------------------- #


class TestEngineIni:
    def test_native_passthrough_roundtrip(self, tmp_path: Path) -> None:
        orig = parse_ini((AC_DATA / "engine.ini").read_text(encoding="utf-8"))
        # Stash the AC LUTs so writer emits the companion files.
        power_lut = parse_lut((AC_DATA / "power.lut").read_text(encoding="utf-8"))
        coast_lut = parse_lut((AC_DATA / "coast_curve.lut").read_text(encoding="utf-8"))
        doc = _native_doc({"engine": orig})
        doc.assetto.raw["engine_luts"] = {
            "power.lut": power_lut,
            "coast_curve.lut": coast_lut,
        }
        out = engine_ini.write(doc, tmp_path)
        new = parse_ini(out.read_text(encoding="utf-8"))
        assert _normalize(orig) == _normalize(new)
        assert (tmp_path / "power.lut").is_file()
        assert (tmp_path / "coast_curve.lut").is_file()

    def test_power_lut_roundtrips_through_parse_lut(self, tmp_path: Path) -> None:
        orig = parse_ini((AC_DATA / "engine.ini").read_text(encoding="utf-8"))
        power_lut = parse_lut((AC_DATA / "power.lut").read_text(encoding="utf-8"))
        doc = _native_doc({"engine": orig})
        doc.assetto.raw["engine_luts"] = {"power.lut": power_lut}
        engine_ini.write(doc, tmp_path)
        new_lut = parse_lut((tmp_path / "power.lut").read_text(encoding="utf-8"))
        assert new_lut == power_lut

    def test_svj_fallback_renders(self, tmp_path: Path) -> None:
        doc = svj_loader.load(REPO / "examples" / "mx5_nd_club.svj.json")
        out = engine_ini.write(doc, tmp_path)
        parsed = parse_ini(out.read_text(encoding="utf-8"))
        assert "ENGINE_DATA" in parsed
        assert float(parsed["ENGINE_DATA"]["LIMITER"]) > 5000
        # power.lut companion exists and parses.
        plut = parse_lut((tmp_path / "power.lut").read_text(encoding="utf-8"))
        assert plut and all(len(row) == 2 for row in plut)


# --------------------------------------------------------------------------- #
# drivetrain.ini                                                              #
# --------------------------------------------------------------------------- #


class TestDrivetrainIni:
    def test_native_passthrough_roundtrip(self, tmp_path: Path) -> None:
        orig = parse_ini((AC_DATA / "drivetrain.ini").read_text(encoding="utf-8"))
        doc = _native_doc({"drivetrain": orig})
        out = drivetrain_ini.write(doc, tmp_path)
        new = parse_ini(out.read_text(encoding="utf-8"))
        assert _normalize(orig) == _normalize(new)

    def test_parse_drivetrain_semantic(self, tmp_path: Path) -> None:
        orig = parse_ini((AC_DATA / "drivetrain.ini").read_text(encoding="utf-8"))
        doc = _native_doc({"drivetrain": orig})
        out = drivetrain_ini.write(doc, tmp_path)
        new = parse_ini(out.read_text(encoding="utf-8"))
        assert parse_drivetrain(orig) == parse_drivetrain(new)

    def test_svj_fallback_renders(self, tmp_path: Path) -> None:
        doc = svj_loader.load(REPO / "examples" / "mx5_nd_club.svj.json")
        out = drivetrain_ini.write(doc, tmp_path)
        parsed = parse_ini(out.read_text(encoding="utf-8"))
        assert "GEARS" in parsed
        assert int(parsed["GEARS"]["COUNT"]) >= 4
        assert parsed["TRACTION"]["TYPE"] in {"FWD", "RWD", "AWD"}


# --------------------------------------------------------------------------- #
# electronics.ini                                                             #
# --------------------------------------------------------------------------- #


class TestElectronicsIni:
    def test_emits_header_only_when_no_aids(self, tmp_path: Path) -> None:
        doc = _native_doc({})  # no _ac_native, no driver_aids
        out = electronics_ini.write(doc, tmp_path)
        parsed = parse_ini(out.read_text(encoding="utf-8"))
        assert parsed["HEADER"]["VERSION"] == "1"

    def test_emits_aid_sections_when_driver_aids_present(self, tmp_path: Path) -> None:
        doc = _native_doc({})
        doc.assetto = SvjAssettoExt(
            driver_aids={
                "abs": {"present": True, "active": True, "presets": 3},
                "tc": {"present": True, "slip_threshold": 0.12},
            }
        )
        out = electronics_ini.write(doc, tmp_path)
        parsed = parse_ini(out.read_text(encoding="utf-8"))
        assert "ABS" in parsed
        assert "TRACTION_CONTROL" in parsed


# --------------------------------------------------------------------------- #
# setup.ini                                                                   #
# --------------------------------------------------------------------------- #


class TestSetupIni:
    def test_native_passthrough_roundtrip(self, tmp_path: Path) -> None:
        orig = parse_ini((AC_DATA / "setup.ini").read_text(encoding="utf-8"))
        doc = _native_doc({"setup": orig})
        out = setup_ini.write(doc, tmp_path)
        new = parse_ini(out.read_text(encoding="utf-8"))
        assert _normalize(orig) == _normalize(new)

    def test_svj_fallback_emits_ranges(self, tmp_path: Path) -> None:
        doc = svj_loader.load(REPO / "examples" / "mx5_nd_club.svj.json")
        out = setup_ini.write(doc, tmp_path)
        parsed = parse_ini(out.read_text(encoding="utf-8"))
        assert "HEADER" in parsed
        # mx5 fixture has setup_ranges populated.
        assert any(k.startswith("PRESSURE_") or k.startswith("BRAKE_") for k in parsed)


# --------------------------------------------------------------------------- #
# ctrl_*.ini + LUT pairs                                                      #
# --------------------------------------------------------------------------- #


class TestCtrlFiles:
    def test_native_passthrough_abs_tc(self, tmp_path: Path) -> None:
        abs_orig = parse_ini((AC_DATA / "ctrl_abs.ini").read_text(encoding="utf-8"))
        tc_orig = parse_ini((AC_DATA / "ctrl_tc.ini").read_text(encoding="utf-8"))
        doc = _native_doc({"ctrl_abs": abs_orig, "ctrl_tc": tc_orig})
        # Bridge: ctrl_files reads from x_assettocorsa.controls; for the
        # native pass-through path we still need a placeholder entry per name
        # so the writer knows to emit a file for it.
        doc.assetto.controls = {"abs": {"controllers": []}, "tc": {"controllers": []}}
        written = ctrl_files.write(doc, tmp_path)
        assert (tmp_path / "ctrl_abs.ini") in written
        assert (tmp_path / "ctrl_tc.ini") in written
        new_abs = parse_ini((tmp_path / "ctrl_abs.ini").read_text(encoding="utf-8"))
        new_tc = parse_ini((tmp_path / "ctrl_tc.ini").read_text(encoding="utf-8"))
        assert _normalize(abs_orig) == _normalize(new_abs)
        assert _normalize(tc_orig) == _normalize(new_tc)

    def test_inline_lut_format(self, tmp_path: Path) -> None:
        doc = _native_doc({})
        doc.assetto.controls = {
            "abs": {
                "version": 1,
                "controllers": [
                    {
                        "input": "SPEED",
                        "combinator": "ADD",
                        "lut": [[0, 0.0], [5, 0.4], [20, 0.85], [40, 0.95], [80, 1.0]],
                        "filter": 0.95,
                        "up_limit": 1,
                        "down_limit": 0,
                    }
                ],
            }
        }
        written = ctrl_files.write(doc, tmp_path)
        text = (tmp_path / "ctrl_abs.ini").read_text(encoding="utf-8")
        assert "LUT=(|" in text
        assert "0=0" in text and "80=1" in text
        # No external .lut file because the curve is short.
        assert not any(p.suffix == ".lut" for p in written)

    def test_external_lut_paired(self, tmp_path: Path) -> None:
        # 12 points -> above INLINE_LUT_THRESHOLD -> external file.
        pairs = [[float(i), float(i) * 0.1] for i in range(12)]
        doc = _native_doc({})
        doc.assetto.controls = {
            "turbo0": {
                "version": 1,
                "controllers": [
                    {"input": "RPM", "combinator": "ADD", "lut": pairs}
                ],
            }
        }
        written = ctrl_files.write(doc, tmp_path)
        ini_path = tmp_path / "ctrl_turbo0.ini"
        assert ini_path in written
        # Companion LUT file must exist and be referenced.
        text = ini_path.read_text(encoding="utf-8")
        # Filename appears as the LUT value.
        assert ".lut" in text
        external = [p for p in written if p.suffix == ".lut"]
        assert external, "external LUT file not emitted"
        # Parses back cleanly.
        parsed_lut = parse_lut(external[0].read_text(encoding="utf-8"))
        assert len(parsed_lut) == 12


# --------------------------------------------------------------------------- #
# tyres.ini + pacejka_inverse                                                 #
# --------------------------------------------------------------------------- #


class TestTyresIni:
    def test_native_passthrough_roundtrip(self, tmp_path: Path) -> None:
        orig = parse_ini((AC_DATA / "tyres.ini").read_text(encoding="utf-8"))
        doc = _native_doc({"tyres": orig})
        out = tyres_ini.write(doc, tmp_path)
        new = parse_ini(out.read_text(encoding="utf-8"))
        assert _normalize(orig) == _normalize(new)

    def test_source_params_fast_path(self, tmp_path: Path) -> None:
        """When tires.sets[X].models.pacejka_mf52._source_params is present,
        the AC parameters come back verbatim."""
        doc = svj_loader.load(REPO / "examples" / "mx5_nd_club.svj.json")
        # mx5 fixture carries _source_params; that's the lossless path.
        out = tyres_ini.write(doc, tmp_path)
        parsed = parse_ini(out.read_text(encoding="utf-8"))
        assert "FRONT" in parsed or "FRONT_0" in parsed
        front = parsed.get("FRONT") or parsed.get("FRONT_0")
        # Check a few characteristic AC keys came through.
        assert "DY0" in front
        assert "FZ0" in front
        assert float(front["FZ0"]) > 0
        # The conversion log should NOT have a tyre warning (fast path).
        log = tmp_path.parent / "conversion_log.txt"
        if log.exists():
            assert "WARNING [pacejka]" not in log.read_text(encoding="utf-8")

    def test_slow_path_logs_warning(self, tmp_path: Path) -> None:
        """If we feed a compound with only MF coefficients (no
        ``_source_params``), the fitter runs and logs a warning."""
        # Build a minimal SvjDoc with one front compound carrying MF only.
        front = {
            "dimensions": {"section_width": 0.225, "overall_diameter": 0.62},
            "rim": {"diameter": 0.4572},
            "pressure": 200000.0,
            "models": {
                "pacejka_mf52": {
                    "version": "MF 5.2",
                    "reference_load_N": 3000.0,
                    "pure_slip_lateral": {
                        "pCy1": 1.4, "pDy1": 1.3, "pDy2": -0.1,
                        "pEy1": -0.3, "pEy2": -0.3, "pKy1": 22.0, "pKy2": 2.0,
                    },
                    "pure_slip_longitudinal": {
                        "pCx1": 1.5, "pDx1": 1.4, "pDx2": -0.1,
                        "pEx1": -1.0, "pEx2": 0.2, "pEx3": -1.5,
                        "pEx4": 0.0, "pKx1": 25.0, "pKx2": 0.0, "pKx3": 1.0,
                    },
                }
            },
        }
        doc = _native_doc({})
        doc.tires = SvjTires(sets={"tire_front_1_synth": front})
        out_dir = tmp_path / "data"
        out_dir.mkdir()
        tyres_ini.write(doc, out_dir)
        log = out_dir.parent / "conversion_log.txt"
        assert log.exists()
        assert "WARNING [pacejka]" in log.read_text(encoding="utf-8")

    def test_compound_section_has_required_keys(self, tmp_path: Path) -> None:
        doc = svj_loader.load(REPO / "examples" / "mx5_nd_club.svj.json")
        out = tyres_ini.write(doc, tmp_path)
        parsed = parse_ini(out.read_text(encoding="utf-8"))
        front = parsed.get("FRONT") or parsed.get("FRONT_0")
        required = {"WIDTH", "RADIUS", "FZ0", "DY0", "DX0"}
        assert required <= set(front)


# --------------------------------------------------------------------------- #
# Pacejka inverse direct tests                                                #
# --------------------------------------------------------------------------- #


class TestPacejkaInverseDirect:
    def test_source_params_passthrough(self) -> None:
        doc = svj_loader.load(REPO / "examples" / "mx5_nd_club.svj.json")
        front_key = next(k for k in doc.tires.sets if "front" in k)
        params = pacejka_inverse.fit_ac_params(doc.tires.sets[front_key])
        # Values should match the _source_params block.
        src = doc.tires.sets[front_key]["models"]["pacejka_mf52"]["_source_params"]
        assert params["DY0"] == src["DY0"]
        assert params["DX0"] == src["DX0"]
        assert params["FZ0"] == src["FZ0"]

    def test_missing_inputs_raises(self) -> None:
        with pytest.raises(pacejka_inverse.PacejkaInverseError):
            pacejka_inverse.fit_ac_params({})

    def test_slow_path_returns_plausible_params(self) -> None:
        """The slow path must at least return a finite, in-range DY0/DX0."""
        compound = {
            "dimensions": {"section_width": 0.225, "overall_diameter": 0.62},
            "rim": {"diameter": 0.4572},
            "models": {
                "pacejka_mf52": {
                    "reference_load_N": 3000.0,
                    "pure_slip_lateral": {
                        "pCy1": 1.4, "pDy1": 1.3, "pDy2": -0.1,
                        "pEy1": -0.3, "pEy2": -0.3, "pKy1": 22.0, "pKy2": 2.0,
                    },
                    "pure_slip_longitudinal": {
                        "pCx1": 1.5, "pDx1": 1.4, "pDx2": -0.1,
                        "pEx1": -1.0, "pEx2": 0.2, "pEx3": -1.5,
                        "pEx4": 0.0, "pKx1": 25.0, "pKx2": 0.0, "pKx3": 1.0,
                    },
                }
            },
        }
        params = pacejka_inverse.fit_ac_params(compound)
        assert 0.5 <= params["DY0"] <= 3.0
        assert 0.5 <= params["DX0"] <= 3.0
        assert 0.0 < params["FLEX"] < 1.0
