"""Stage-9 acceptance tests for ``svj_to_ac`` (the Gradio UI module).

We don't launch a server in tests; instead we:

* Import the module (it must not raise).
* Exercise the UI-agnostic convert callbacks (``convert_one``,
  ``convert_batch``, ``make_fbx_only``) end-to-end against the canonical
  mx5 fixture.
* Build the Gradio ``Blocks`` object and assert it has 4 tabs whose
  labels start with "1 - ", "2 - ", "3 - ", "4 - ".

Tests requiring gradio are skipped automatically if gradio isn't
installed.
"""

from __future__ import annotations

from pathlib import Path

import pytest

import svj_to_ac

REPO = Path(__file__).resolve().parents[1]
MX5 = REPO / "examples" / "mx5_nd_club.svj.json"

try:
    import gradio as _gr  # noqa: F401
    HAS_GRADIO = True
except Exception:
    HAS_GRADIO = False


# --------------------------------------------------------------------------- #
# Module surface                                                              #
# --------------------------------------------------------------------------- #


class TestModuleSurface:
    def test_public_symbols(self) -> None:
        for name in ("build_ui", "main", "convert_one", "convert_batch",
                     "make_fbx_only", "ConversionResult", "UI_VERSION"):
            assert hasattr(svj_to_ac, name), f"missing {name}"

    def test_ui_version_string(self) -> None:
        assert isinstance(svj_to_ac.UI_VERSION, str)
        assert svj_to_ac.UI_VERSION


# --------------------------------------------------------------------------- #
# convert_one                                                                 #
# --------------------------------------------------------------------------- #


class TestConvertOne:
    def test_mx5_succeeds(self, tmp_path: Path) -> None:
        result = svj_to_ac.convert_one(MX5, tmp_path)
        assert result.success
        assert result.zip_path is not None
        assert result.zip_path.is_file()
        assert result.zip_path.stat().st_size > 1000
        assert "OK:" in result.log
        assert result.summary["brand"] == "Mazda"

    def test_invalid_path_fails_gracefully(self, tmp_path: Path) -> None:
        result = svj_to_ac.convert_one(tmp_path / "nope.svj.json", tmp_path / "out")
        assert not result.success
        assert result.zip_path is None
        assert "FAIL" in result.log

    def test_corrupted_json_fails_gracefully(self, tmp_path: Path) -> None:
        bad = tmp_path / "bad.svj.json"
        bad.write_text("{not json")
        result = svj_to_ac.convert_one(bad, tmp_path / "out")
        assert not result.success
        assert "error" in result.summary


# --------------------------------------------------------------------------- #
# convert_batch                                                               #
# --------------------------------------------------------------------------- #


class TestConvertBatch:
    def test_one_car_folder(self, tmp_path: Path) -> None:
        src = tmp_path / "cars"
        src.mkdir()
        (src / "mx5.svj.json").write_bytes(MX5.read_bytes())
        result = svj_to_ac.convert_batch(src, tmp_path / "out")
        assert result.success
        assert result.zip_path is not None
        # Name follows the AC_to_SVJ naming convention.
        assert result.zip_path.name.startswith("svj_ac_batch_")
        assert result.zip_path.name.endswith(f"_conv{svj_to_ac.UI_VERSION}.zip")
        assert result.summary["cars"] == 1
        assert result.summary["succeeded"] == 1

    def test_two_cars_folder(self, tmp_path: Path) -> None:
        src = tmp_path / "cars"
        src.mkdir()
        (src / "a.svj.json").write_bytes(MX5.read_bytes())
        (src / "b.svj.json").write_bytes(MX5.read_bytes())
        result = svj_to_ac.convert_batch(src, tmp_path / "out")
        assert result.success
        assert result.summary["cars"] == 2
        # batch_summary.csv lands inside the staging dir.
        staging = tmp_path / "out" / "_batch_staging"
        csv_path = staging / "batch_summary.csv"
        assert csv_path.is_file()
        lines = csv_path.read_text(encoding="utf-8").splitlines()
        # 1 header + 2 rows.
        assert len(lines) >= 3

    def test_empty_folder_fails(self, tmp_path: Path) -> None:
        empty = tmp_path / "empty"
        empty.mkdir()
        result = svj_to_ac.convert_batch(empty, tmp_path / "out")
        assert not result.success
        assert "no .svj.json" in result.log

    def test_missing_folder_fails(self, tmp_path: Path) -> None:
        result = svj_to_ac.convert_batch(tmp_path / "absent", tmp_path / "out")
        assert not result.success


# --------------------------------------------------------------------------- #
# make_fbx_only                                                               #
# --------------------------------------------------------------------------- #


class TestMakeFbxOnly:
    def test_from_svj(self, tmp_path: Path) -> None:
        result = svj_to_ac.make_fbx_only(MX5, tmp_path)
        assert result.success
        assert result.zip_path is not None
        assert result.zip_path.suffix == ".fbx"
        # Companion files
        assert (tmp_path / "lods.ini").is_file()
        assert (tmp_path / "persistence.ksanim").is_file()
        # FBX content sanity
        text = result.zip_path.read_text(encoding="ascii")
        assert "FBXVersion: 7400" in text

    def test_from_manual_dims(self, tmp_path: Path) -> None:
        result = svj_to_ac.make_fbx_only(
            None, tmp_path,
            wheelbase=2.7, track_front=1.55, track_rear=1.6,
            overall_length=4.3, overall_width=1.85, overall_height=1.4,
            tyre_width=0.245,
        )
        assert result.success
        text = result.zip_path.read_text(encoding="ascii")
        assert "WHEEL_LF" in text and "BODY" in text


# --------------------------------------------------------------------------- #
# UI structure (Gradio Blocks)                                                #
# --------------------------------------------------------------------------- #


@pytest.mark.skipif(not HAS_GRADIO, reason="gradio not installed")
class TestUiStructure:
    def test_build_ui_returns_blocks(self) -> None:
        ui = svj_to_ac.build_ui()
        import gradio as gr
        assert isinstance(ui, gr.Blocks)

    def test_four_tabs_present(self) -> None:
        ui = svj_to_ac.build_ui()
        # Walk the block tree looking for Tab components.
        labels: list[str] = []
        for block in ui.blocks.values():
            cls = type(block).__name__
            if cls in ("Tab", "TabItem"):
                label = getattr(block, "label", None)
                if label:
                    labels.append(label)
        assert len(labels) == 4, f"expected 4 tabs, got {labels}"
        # Each tab is numbered 1..4 in its label per build plan convention.
        for i, label in enumerate(sorted(labels), start=1):
            assert label.startswith(f"{i} -"), f"tab #{i} label {label!r} doesn't start with '{i} -'"

    def test_app_title(self) -> None:
        ui = svj_to_ac.build_ui()
        assert "SVJ" in ui.title and "AC" in ui.title
