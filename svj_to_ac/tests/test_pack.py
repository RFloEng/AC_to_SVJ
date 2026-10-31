"""Stage-7 acceptance tests: full conversion -> folder + reproducible zip.

Covers:

* End-to-end assemble of mx5_nd_club.svj.json into a Content
  Manager-shaped tree (data/, ui/, sfx/).
* Minimum required files exist (per ``02_PROJECT_SPEC.md §11``).
* ``ui_car.json`` validates against Kunos canonical fields (name,
  brand, year, class, specs.{}).
* Zip is reproducible: two consecutive runs produce a byte-identical
  archive (SHA-256 equality, per pitfalls doc §"Reproducible zips").
"""

from __future__ import annotations

import json
import zipfile
from pathlib import Path

import pytest

import svj_loader
import pack_car
from ac_writers import ui_car_json

REPO = Path(__file__).resolve().parents[1]
MX5 = REPO / "examples" / "mx5_nd_club.svj.json"


# Minimum required files Content Manager / AC expect (per spec §4 + §11).
REQUIRED_RELATIVE = (
    "data/car.ini",
    "data/suspensions.ini",
    "data/tyres.ini",
    "data/engine.ini",
    "data/power.lut",
    "data/drivetrain.ini",
    "data/aero.ini",
    "data/brakes.ini",
    "data/electronics.ini",
    "data/setup.ini",
    "ui/ui_car.json",
    "ui/badge.png",
    "ui/preview.jpg",
    "sfx/GUIDs.txt",
)


@pytest.fixture(scope="module")
def doc() -> svj_loader.SvjDoc:
    return svj_loader.load(MX5)


# --------------------------------------------------------------------------- #
# ui_car.json                                                                 #
# --------------------------------------------------------------------------- #


class TestUiCarJson:
    def test_writes_file(self, doc, tmp_path: Path) -> None:
        out = ui_car_json.write(doc, tmp_path)
        assert out.is_file()
        assert out.name == "ui_car.json"

    def test_is_utf8_no_bom_lf(self, doc, tmp_path: Path) -> None:
        out = ui_car_json.write(doc, tmp_path)
        raw = out.read_bytes()
        assert not raw.startswith(b"\xef\xbb\xbf"), "ui_car.json has BOM"
        assert b"\r" not in raw, "ui_car.json contains CRLF"
        assert raw.endswith(b"\n")

    def test_canonical_fields_present(self, doc, tmp_path: Path) -> None:
        out = ui_car_json.write(doc, tmp_path)
        data = json.loads(out.read_text(encoding="utf-8"))
        # Required Kunos top-level fields.
        for k in ("name", "brand", "year", "class", "specs"):
            assert k in data, f"missing canonical key {k!r}"
        assert data["brand"] == doc.vehicle_info.make
        assert data["year"] == doc.vehicle_info.year
        # specs uses display strings with units.
        specs = data["specs"]
        assert isinstance(specs, dict)
        assert "weight" in specs and specs["weight"].endswith("kg")
        assert "bhp" in specs and specs["bhp"].endswith("bhp")

    def test_native_passthrough_wins(self, doc, tmp_path: Path) -> None:
        """Whatever lives under x_assettocorsa._ac_native['ui_car'] is
        emitted verbatim."""
        from svj_loader import SvjAssettoExt

        custom = {"name": "PASS-THROUGH", "brand": "Stub", "specs": {"weight": "1kg"}}
        doc.assetto = SvjAssettoExt(raw={"_ac_native": {"ui_car": custom}})
        out = ui_car_json.write(doc, tmp_path)
        data = json.loads(out.read_text(encoding="utf-8"))
        assert data["name"] == "PASS-THROUGH"
        # restore so other tests see canonical fields
        doc.assetto = svj_loader.load(MX5).assetto


# --------------------------------------------------------------------------- #
# pack_car.assemble                                                           #
# --------------------------------------------------------------------------- #


class TestAssemble:
    def test_writes_full_tree(self, doc, tmp_path: Path) -> None:
        result = pack_car.assemble(doc, tmp_path)
        assert result.car_dir.is_dir()
        assert result.data_dir.is_dir()
        assert result.ui_dir.is_dir()
        assert result.sfx_dir.is_dir()
        # Returned file list is sorted and contains every emitted file.
        for f in result.files:
            assert f.is_file(), f"missing file {f}"
        # Sort invariant.
        assert list(result.files) == sorted(result.files)

    def test_minimum_required_files_exist(self, doc, tmp_path: Path) -> None:
        result = pack_car.assemble(doc, tmp_path)
        rel_paths = {p.relative_to(result.car_dir).as_posix() for p in result.files}
        missing = [r for r in REQUIRED_RELATIVE if r not in rel_paths]
        assert not missing, f"missing required files: {missing}"

    def test_sfx_guids_references_model(self, doc, tmp_path: Path) -> None:
        result = pack_car.assemble(doc, tmp_path)
        text = (result.sfx_dir / "GUIDs.txt").read_text(encoding="utf-8")
        assert "engine_ext" in text and "engine_int" in text
        assert pack_car.model_id(doc) in text

    def test_ui_car_json_parses(self, doc, tmp_path: Path) -> None:
        result = pack_car.assemble(doc, tmp_path)
        data = json.loads((result.ui_dir / "ui_car.json").read_text(encoding="utf-8"))
        assert data["brand"] == doc.vehicle_info.make

    def test_badge_and_preview_have_content(self, doc, tmp_path: Path) -> None:
        result = pack_car.assemble(doc, tmp_path)
        assert (result.ui_dir / "badge.png").stat().st_size > 100
        assert (result.ui_dir / "preview.jpg").stat().st_size > 1000


# --------------------------------------------------------------------------- #
# Reproducible zip                                                            #
# --------------------------------------------------------------------------- #


class TestReproducibleZip:
    def test_two_runs_produce_identical_zip(self, doc, tmp_path: Path) -> None:
        run1 = tmp_path / "run1"
        run2 = tmp_path / "run2"
        zip1 = pack_car.convert(doc, run1)
        zip2 = pack_car.convert(doc, run2)
        # Sanity checks
        assert zip1.is_file() and zip2.is_file()
        assert zip1.stat().st_size > 0
        # SHA-256 equality is the strict reproducibility criterion.
        assert pack_car.sha256_of(zip1) == pack_car.sha256_of(zip2), (
            "zip is not reproducible: SHA-256 mismatch between runs"
        )

    def test_zip_entries_have_fixed_mtime(self, doc, tmp_path: Path) -> None:
        z = pack_car.convert(doc, tmp_path)
        with zipfile.ZipFile(z) as zf:
            for info in zf.infolist():
                assert info.date_time == (1980, 1, 1, 0, 0, 0), (
                    f"{info.filename} has mtime {info.date_time}"
                )

    def test_zip_entries_sorted(self, doc, tmp_path: Path) -> None:
        z = pack_car.convert(doc, tmp_path)
        with zipfile.ZipFile(z) as zf:
            names = [info.filename for info in zf.infolist()]
        assert names == sorted(names)

    def test_zip_contains_required_arcnames(self, doc, tmp_path: Path) -> None:
        z = pack_car.convert(doc, tmp_path)
        model = pack_car.model_id(doc)
        with zipfile.ZipFile(z) as zf:
            names = set(zf.namelist())
        for rel in REQUIRED_RELATIVE:
            arc = f"{model}/{rel}"
            assert arc in names, f"missing in zip: {arc}"


# --------------------------------------------------------------------------- #
# model_id slugifier                                                          #
# --------------------------------------------------------------------------- #


class TestModelId:
    def test_slug_lowercase_underscored(self, doc) -> None:
        slug = pack_car.model_id(doc)
        assert slug == slug.lower()
        assert " " not in slug
        # no double underscores
        assert "__" not in slug

    def test_brand_and_model_combined(self, doc) -> None:
        slug = pack_car.model_id(doc)
        assert "mazda" in slug
