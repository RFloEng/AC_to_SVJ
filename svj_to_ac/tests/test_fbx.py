"""Stage-8 acceptance tests for ``fbx_skeleton``.

KsEditor isn't available here, so we don't open the FBX in the editor.
Instead we sanity-check the file structure programmatically (per the
build plan's fallback path):

* Header version is ``FBXVersion: 7400`` (FBX 2014).
* GlobalSettings carries the Z-up axis triple and ``UnitScaleFactor=1``.
* All required dummy names appear: ``WHEEL_LF/RF/LR/RR``,
  ``SUSP_LF/RF/LR/RR``, ``COCKPIT_HR``, ``STEER_HR``, ``Car``.
* BODY mesh exists.
* Four wheel cylinder meshes (``WHEEL_*_MESH``) exist.
* Companion files: ``lods.ini`` with LOD_0, ``persistence.ksanim``
  placeholder (empty).
* Output is ASCII-only (no Unicode bleed-through).
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

import fbx_skeleton
import svj_loader

REPO = Path(__file__).resolve().parents[1]
MX5 = REPO / "examples" / "mx5_nd_club.svj.json"


@pytest.fixture(scope="module")
def doc() -> svj_loader.SvjDoc:
    return svj_loader.load(MX5)


@pytest.fixture
def fbx_path(doc, tmp_path: Path) -> Path:
    return fbx_skeleton.write(doc, tmp_path / "car.fbx")


# --------------------------------------------------------------------------- #
# Header & global settings                                                    #
# --------------------------------------------------------------------------- #


class TestHeader:
    def test_fbx_2014_version(self, fbx_path: Path) -> None:
        text = fbx_path.read_text(encoding="ascii")
        assert "FBXVersion: 7400" in text

    def test_creator_present(self, fbx_path: Path) -> None:
        text = fbx_path.read_text(encoding="ascii")
        assert "Creator:" in text and "SVJ_to_AC" in text

    def test_ascii_only(self, fbx_path: Path) -> None:
        raw = fbx_path.read_bytes()
        for b in raw:
            assert b < 128, f"non-ASCII byte 0x{b:02x} in FBX output"


class TestGlobalSettings:
    @pytest.fixture
    def settings_text(self, fbx_path: Path) -> str:
        text = fbx_path.read_text(encoding="ascii")
        m = re.search(r"GlobalSettings:\s*\{(.*?)\}\s*\n", text, re.DOTALL)
        assert m, "GlobalSettings block not found"
        return m.group(1)

    def test_z_up_axis(self, settings_text: str) -> None:
        # UpAxis: 1, UpAxisSign: 1
        assert re.search(r'"UpAxis".*?,\s*1\b', settings_text)
        assert re.search(r'"UpAxisSign".*?,\s*1\b', settings_text)

    def test_front_axis(self, settings_text: str) -> None:
        assert re.search(r'"FrontAxis".*?,\s*2\b', settings_text)
        assert re.search(r'"FrontAxisSign".*?,\s*1\b', settings_text)

    def test_coord_axis(self, settings_text: str) -> None:
        assert re.search(r'"CoordAxis".*?,\s*0\b', settings_text)
        assert re.search(r'"CoordAxisSign".*?,\s*1\b', settings_text)

    def test_unit_scale_factor_1(self, settings_text: str) -> None:
        assert re.search(r'"UnitScaleFactor".*?,\s*1\b', settings_text)
        assert re.search(r'"OriginalUnitScaleFactor".*?,\s*1\b', settings_text)


# --------------------------------------------------------------------------- #
# Required dummies                                                            #
# --------------------------------------------------------------------------- #


class TestDummies:
    @pytest.fixture
    def text(self, fbx_path: Path) -> str:
        return fbx_path.read_text(encoding="ascii")

    @pytest.mark.parametrize("name", [
        "Car",
        "WHEEL_LF", "WHEEL_RF", "WHEEL_LR", "WHEEL_RR",
        "SUSP_LF", "SUSP_RF", "SUSP_LR", "SUSP_RR",
        "COCKPIT_HR", "STEER_HR",
    ])
    def test_dummy_present(self, text: str, name: str) -> None:
        assert f'"Model::{name}"' in text, f"missing Model::{name}"

    def test_wheel_susp_share_position(self, doc, text: str) -> None:
        """Each SUSP_* must sit at the same translation as its matching WHEEL_*."""
        # Extract Lcl Translation for each named Model::<name> entry.
        positions: dict[str, tuple[float, float, float]] = {}
        for match in re.finditer(
            r'Model:\s*\d+,\s*"Model::(\S+?)",\s*"Null"\s*\{[^}]*?'
            r'"Lcl Translation",\s*"Lcl Translation",\s*"",\s*"A",\s*'
            r'([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)',
            text, re.DOTALL,
        ):
            name = match.group(1)
            positions[name] = (float(match.group(2)), float(match.group(3)), float(match.group(4)))
        for pair in (("WHEEL_LF", "SUSP_LF"), ("WHEEL_RF", "SUSP_RF"),
                     ("WHEEL_LR", "SUSP_LR"), ("WHEEL_RR", "SUSP_RR")):
            w, s = pair
            assert w in positions and s in positions
            assert positions[w] == positions[s], f"{s} not co-located with {w}"


# --------------------------------------------------------------------------- #
# Meshes                                                                      #
# --------------------------------------------------------------------------- #


class TestMeshes:
    @pytest.fixture
    def text(self, fbx_path: Path) -> str:
        return fbx_path.read_text(encoding="ascii")

    def test_body_mesh(self, text: str) -> None:
        assert '"Model::BODY"' in text
        assert '"Geometry::BODY"' in text

    @pytest.mark.parametrize("wheel", ["LF", "RF", "LR", "RR"])
    def test_four_wheel_cylinder_meshes(self, text: str, wheel: str) -> None:
        name = f"WHEEL_{wheel}_MESH"
        assert f'"Model::{name}"' in text, f"missing wheel mesh {name}"
        assert f'"Geometry::{name}"' in text

    def test_default_material(self, text: str) -> None:
        assert 'Material::ksPerPixel_default' in text
        assert '"phong"' in text


# --------------------------------------------------------------------------- #
# Connections                                                                 #
# --------------------------------------------------------------------------- #


class TestConnections:
    def test_connections_block_present(self, fbx_path: Path) -> None:
        text = fbx_path.read_text(encoding="ascii")
        assert "Connections:" in text
        # Car is connected to scene root (0).
        m = re.search(r'C:\s*"OO",\s*(\d+),\s*0', text)
        assert m, "no top-level OO connection to root"


# --------------------------------------------------------------------------- #
# Companion files                                                             #
# --------------------------------------------------------------------------- #


class TestCompanions:
    def test_lods_ini(self, tmp_path: Path) -> None:
        out = fbx_skeleton.write_lods_ini(tmp_path / "lods.ini")
        text = out.read_text(encoding="utf-8")
        assert "[LOD_0]" in text
        assert "FILE=" in text

    def test_ksanim_placeholder(self, tmp_path: Path) -> None:
        out = fbx_skeleton.write_ksanim_placeholder(tmp_path / "persistence.ksanim")
        assert out.is_file()
        assert out.stat().st_size == 0


# --------------------------------------------------------------------------- #
# Dimension overrides                                                         #
# --------------------------------------------------------------------------- #


class TestDimensionOverrides:
    def test_overall_length_drives_body_x(self, doc, tmp_path: Path) -> None:
        fbx_path = fbx_skeleton.write(
            doc, tmp_path / "car.fbx",
            overall_length=5.0,
            overall_width=2.0,
            overall_height=1.5,
        )
        text = fbx_path.read_text(encoding="ascii")
        # Body half-extents should appear as ±length_cm/2 = ±250
        assert "250" in text  # half-length cm
        assert "100" in text  # half-width cm
        assert "75" in text   # half-height cm
