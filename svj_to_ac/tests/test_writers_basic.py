"""Round-trip tests for the Stage-4 writers (car, aero, brakes, lut).

Strategy
--------

For each writer we round-trip through ``AC_to_SVJ/ac_parsers.parse_ini``:

1. Read the ground-truth AC INI in ``AC_to_SVJ/test_car/data/``.
2. Parse it to a ``section -> {key: str}`` dict via ``parse_ini``.
3. Stash that parsed dict as the SVJ ``x_assettocorsa._ac_native["<name>"]``
   block on a synthetic :class:`SvjDoc`.
4. Run our writer.
5. Re-parse the writer output with ``parse_ini``.
6. Assert the two dicts are numerically equal (string comparison after
   normalising floats so that ``0.140`` and ``0.14`` are considered
   equal).

Plus dedicated tests for ``lut_writer``: round-trip values, header
comment behaviour, LF endings, trailing newline.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import pytest

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

import lut_writer
from ac_writers import aero_ini, brakes_ini, car_ini

# AC_to_SVJ sibling clone (added to sys.path by tests/conftest.py).
from ac_parsers import parse_aero, parse_brakes, parse_ini, parse_lut  # noqa: E402

REPO = Path(__file__).resolve().parents[1]
AC_DATA = REPO.parent / "test_car" / "data"


# --------------------------------------------------------------------------- #
# Helpers                                                                     #
# --------------------------------------------------------------------------- #


def _normalize_ini(parsed: dict[str, dict[str, str]]) -> dict[str, dict[str, Any]]:
    """Convert numeric-looking strings to floats so ``0.14`` == ``0.140``."""
    out: dict[str, dict[str, Any]] = {}
    for sec, kv in parsed.items():
        out[sec.upper()] = {}
        for k, v in kv.items():
            out[sec.upper()][k.upper()] = _normalize_value(v)
    return out


def _normalize_value(v: str) -> Any:
    s = v.strip()
    # Vec3 detection
    parts = [p.strip() for p in s.replace("|", ",").split(",") if p.strip()]
    if len(parts) >= 2 and all(_is_number(p) for p in parts):
        return tuple(float(p) for p in parts)
    if _is_number(s):
        return float(s)
    return s


def _is_number(s: str) -> bool:
    try:
        float(s)
        return True
    except ValueError:
        return False


def _doc_with_native(native_inis: dict[str, dict[str, dict[str, str]]]) -> SvjDoc:
    """Build a minimally-valid SvjDoc carrying ``_ac_native`` pass-through."""
    ext = SvjAssettoExt(
        raw={"_ac_native": native_inis},
    )
    return SvjDoc(
        vehicle_info=SvjVehicleInfo("X", "Y", 2020, "v", "RWD"),
        chassis=SvjChassis(1000.0, 2.3, 1.5, 1.5, [0.0, 0.0, -0.4]),
        suspension=SvjSuspension(corners={"FL": SvjCorner("FL", [0, 0, 0])}),
        tires=SvjTires(sets={"front": {}}),
        powertrain=SvjPowertrain(layout="FR"),
        brakes=SvjBrakes(0.6, "fixed"),
        metadata=SvjMetadata(version="0.95"),
        assetto=ext,
    )


# --------------------------------------------------------------------------- #
# lut_writer                                                                  #
# --------------------------------------------------------------------------- #


class TestLutWriter:
    def test_basic_roundtrip(self) -> None:
        pairs = [(1000.0, 110.0), (2000.0, 200.0), (5000.5, 0.5)]
        text = lut_writer.format_lut(pairs)
        parsed = parse_lut(text)
        assert parsed == [list(p) for p in pairs]

    def test_trailing_newline(self) -> None:
        text = lut_writer.format_lut([(0, 0)])
        assert text.endswith("\n")

    def test_lf_only_in_file(self, tmp_path: Path) -> None:
        out = tmp_path / "x.lut"
        lut_writer.write_lut(out, [(1, 2), (3, 4)])
        raw = out.read_bytes()
        assert b"\r" not in raw

    def test_header_comment(self) -> None:
        text = lut_writer.format_lut(
            [(1, 1)], header_comment="rpm | torque_Nm"
        )
        assert text.splitlines()[0].startswith("; rpm")

    def test_rejects_wrong_pair_length(self) -> None:
        with pytest.raises(ValueError):
            lut_writer.format_lut([(1, 2, 3)])  # type: ignore[list-item]


# --------------------------------------------------------------------------- #
# car.ini                                                                     #
# --------------------------------------------------------------------------- #


class TestCarIni:
    def test_native_passthrough_roundtrip(self, tmp_path: Path) -> None:
        orig_text = (AC_DATA / "car.ini").read_text(encoding="utf-8")
        orig_parsed = parse_ini(orig_text)

        doc = _doc_with_native({"car": orig_parsed})
        out = car_ini.write(doc, tmp_path)
        new_parsed = parse_ini(out.read_text(encoding="utf-8"))

        assert _normalize_ini(orig_parsed) == _normalize_ini(new_parsed)
        assert out.name == "car.ini"

    def test_lf_and_utf8(self, tmp_path: Path) -> None:
        orig_parsed = parse_ini((AC_DATA / "car.ini").read_text(encoding="utf-8"))
        doc = _doc_with_native({"car": orig_parsed})
        out = car_ini.write(doc, tmp_path)
        raw = out.read_bytes()
        assert b"\r" not in raw  # no CRLF anywhere
        assert raw.endswith(b"\n")  # trailing newline
        assert not raw.startswith(b"\xef\xbb\xbf")  # no BOM

    def test_svj_fallback_basic_keys(self, tmp_path: Path) -> None:
        """Without ``_ac_native``, writer still emits a parseable file
        carrying the canonical SVJ fields."""
        mx5 = REPO / "examples" / "mx5_nd_club.svj.json"
        import svj_loader
        doc = svj_loader.load(mx5)
        out = car_ini.write(doc, tmp_path)
        parsed = parse_ini(out.read_text(encoding="utf-8"))
        assert "BASIC" in parsed
        assert float(parsed["BASIC"]["TOTALMASS"]) == pytest.approx(doc.chassis.mass_total)
        assert float(parsed["BASIC"]["WHEELBASE"]) == pytest.approx(doc.chassis.wheelbase)
        assert parsed["BASIC"]["BRAND"] == doc.vehicle_info.make


# --------------------------------------------------------------------------- #
# brakes.ini                                                                  #
# --------------------------------------------------------------------------- #


class TestBrakesIni:
    def test_native_passthrough_roundtrip(self, tmp_path: Path) -> None:
        orig_parsed = parse_ini((AC_DATA / "brakes.ini").read_text(encoding="utf-8"))
        doc = _doc_with_native({"brakes": orig_parsed})
        out = brakes_ini.write(doc, tmp_path)
        new_parsed = parse_ini(out.read_text(encoding="utf-8"))
        assert _normalize_ini(orig_parsed) == _normalize_ini(new_parsed)

    def test_parse_brakes_semantic_equivalence(self, tmp_path: Path) -> None:
        """Run ``ac_parsers.parse_brakes`` over both INIs; they must yield
        equal semantic dicts (this is the strictest end-to-end check)."""
        orig = parse_brakes(parse_ini((AC_DATA / "brakes.ini").read_text(encoding="utf-8")))
        doc = _doc_with_native({
            "brakes": parse_ini((AC_DATA / "brakes.ini").read_text(encoding="utf-8"))
        })
        out = brakes_ini.write(doc, tmp_path)
        new = parse_brakes(parse_ini(out.read_text(encoding="utf-8")))
        assert orig == new

    def test_svj_fallback_writes_required_sections(self, tmp_path: Path) -> None:
        """Without native, writer still emits FRONT/REAR/BRAKES."""
        mx5 = REPO / "examples" / "mx5_nd_club.svj.json"
        import svj_loader
        doc = svj_loader.load(mx5)
        out = brakes_ini.write(doc, tmp_path)
        parsed = parse_ini(out.read_text(encoding="utf-8"))
        assert {"FRONT", "REAR", "BRAKES"} <= set(parsed)
        assert float(parsed["BRAKES"]["BIAS"]) == pytest.approx(doc.brakes.bias)


# --------------------------------------------------------------------------- #
# aero.ini                                                                    #
# --------------------------------------------------------------------------- #


class TestAeroIni:
    def test_native_passthrough_roundtrip(self, tmp_path: Path) -> None:
        orig_parsed = parse_ini((AC_DATA / "aero.ini").read_text(encoding="utf-8"))
        doc = _doc_with_native({"aero": orig_parsed})
        out = aero_ini.write(doc, tmp_path)
        new_parsed = parse_ini(out.read_text(encoding="utf-8"))
        assert _normalize_ini(orig_parsed) == _normalize_ini(new_parsed)

    def test_parse_aero_semantic_equivalence(self, tmp_path: Path) -> None:
        orig = parse_aero(
            parse_ini((AC_DATA / "aero.ini").read_text(encoding="utf-8")),
            None,
            None,
        )
        doc = _doc_with_native({
            "aero": parse_ini((AC_DATA / "aero.ini").read_text(encoding="utf-8"))
        })
        out = aero_ini.write(doc, tmp_path)
        new = parse_aero(parse_ini(out.read_text(encoding="utf-8")), None, None)
        assert orig == new

    def test_svj_fallback_writes_body_and_wings(self, tmp_path: Path) -> None:
        mx5 = REPO / "examples" / "mx5_nd_club.svj.json"
        import svj_loader
        doc = svj_loader.load(mx5)
        out = aero_ini.write(doc, tmp_path)
        parsed = parse_ini(out.read_text(encoding="utf-8"))
        assert "BODY" in parsed
        # Frontal area should round-trip.
        assert float(parsed["BODY"]["FRONTAL_AREA"]) == pytest.approx(
            doc.aerodynamics.reference["frontal_area"]
        )


# --------------------------------------------------------------------------- #
# Cross-cutting INI formatting rules                                          #
# --------------------------------------------------------------------------- #


@pytest.mark.parametrize(
    "writer,name",
    [(car_ini, "car"), (aero_ini, "aero"), (brakes_ini, "brakes")],
)
def test_ini_format_rules(writer, name, tmp_path: Path) -> None:
    """All three writers must produce LF endings, UTF-8 no BOM, trailing newline."""
    orig_parsed = parse_ini((AC_DATA / f"{name}.ini").read_text(encoding="utf-8"))
    doc = _doc_with_native({name: orig_parsed})
    out = writer.write(doc, tmp_path)
    raw = out.read_bytes()
    assert b"\r" not in raw, f"{name}.ini contains CRLF"
    assert raw.endswith(b"\n"), f"{name}.ini missing trailing newline"
    assert not raw.startswith(b"\xef\xbb\xbf"), f"{name}.ini has BOM"
    text = raw.decode("utf-8")
    # No spaces around `=`
    for line in text.splitlines():
        if "=" in line and not line.startswith(";") and not line.startswith("["):
            assert " = " not in line, f"{name}.ini: spaces around = in: {line!r}"
