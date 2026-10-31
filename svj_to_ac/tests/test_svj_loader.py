"""Tests for ``svj_loader``.

Covers:

* Happy path on ``examples/mx5_nd_club.svj.json``.
* Failure mode when a required section (``chassis``) is removed.
* Optional sections (``_metadata``, ``aerodynamics``, ``x_assettocorsa``)
  may be absent without ``load`` raising.
* :func:`validate` returns a clean list on the canonical fixture and
  flags a corrupted one.
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

import svj_loader
from svj_loader import (
    CORNERS,
    MissingSectionError,
    SvjDoc,
    SvjLoadError,
    load,
    validate,
)

REPO = Path(__file__).resolve().parents[1]
EXAMPLE = REPO / "examples" / "mx5_nd_club.svj.json"


# --------------------------------------------------------------------------- #
# Fixtures                                                                    #
# --------------------------------------------------------------------------- #


@pytest.fixture(scope="module")
def example_doc() -> SvjDoc:
    return load(EXAMPLE)


@pytest.fixture
def stripped_no_chassis(tmp_path: Path) -> Path:
    data = json.loads(EXAMPLE.read_text(encoding="utf-8"))
    del data["chassis"]
    out = tmp_path / "no_chassis.svj.json"
    out.write_text(json.dumps(data), encoding="utf-8")
    return out


@pytest.fixture
def stripped_optional(tmp_path: Path) -> Path:
    """Same example but with every optional section removed."""
    data = json.loads(EXAMPLE.read_text(encoding="utf-8"))
    for key in ("_metadata", "aerodynamics", "x_assettocorsa"):
        data.pop(key, None)
    out = tmp_path / "no_optionals.svj.json"
    out.write_text(json.dumps(data), encoding="utf-8")
    return out


# --------------------------------------------------------------------------- #
# Happy path                                                                  #
# --------------------------------------------------------------------------- #


def test_example_loads(example_doc: SvjDoc) -> None:
    assert example_doc.source_path == EXAMPLE
    assert example_doc.vehicle_info.make == "Mazda"
    assert example_doc.vehicle_info.model == "Mazda MX5 ND"
    assert example_doc.vehicle_info.year == 2015


def test_chassis_scalars(example_doc: SvjDoc) -> None:
    c = example_doc.chassis
    assert c.mass_total == pytest.approx(1058.0)
    assert c.wheelbase == pytest.approx(2.31)
    assert c.track_front == pytest.approx(1.496)
    assert c.track_rear == pytest.approx(1.503)
    assert len(c.center_of_gravity) == 3
    assert set(c.mass_unsprung_per_corner) == set(CORNERS)


def test_all_four_corners_present(example_doc: SvjDoc) -> None:
    s = example_doc.suspension
    for corner in CORNERS:
        assert corner in s.corners
        assert len(s[corner].position) == 3
        assert s[corner].topology.get("system_type") == "double_wishbone"


def test_powertrain_layout(example_doc: SvjDoc) -> None:
    pt = example_doc.powertrain
    assert pt.layout == "FR"
    assert "torque_curve" in pt.engine or "torque" in pt.engine
    assert pt.differentials  # at least one


def test_brakes_bias(example_doc: SvjDoc) -> None:
    assert 0.0 <= example_doc.brakes.bias <= 1.0


def test_metadata_loaded(example_doc: SvjDoc) -> None:
    md = example_doc.metadata
    assert md is not None
    assert md.specification == "SVJ"
    assert md.version == "0.95"
    assert md.coordinate_system == "SAE_J670"
    assert md.units == "SI"


def test_assetto_extension_loaded(example_doc: SvjDoc) -> None:
    ext = example_doc.assetto
    assert ext is not None
    assert ext.setup_ranges  # non-empty
    assert ext.driver_aids   # non-empty


def test_aerodynamics_loaded(example_doc: SvjDoc) -> None:
    aero = example_doc.aerodynamics
    assert aero is not None
    assert aero.reference.get("frontal_area")
    assert aero.global_coefficients.get("Cd")


def test_tires_aliases(example_doc: SvjDoc) -> None:
    """The British-spelling alias ``.tyres`` mirrors ``.tires``."""
    assert example_doc.tires is example_doc.tyres
    assert example_doc.tires.sets  # non-empty


def test_raw_dict_preserves_full_input(example_doc: SvjDoc) -> None:
    raw_keys = set(example_doc.raw)
    assert {"vehicle_info", "chassis", "suspension", "tires", "powertrain", "brakes"} <= raw_keys


# --------------------------------------------------------------------------- #
# Failure mode                                                                #
# --------------------------------------------------------------------------- #


def test_missing_chassis_raises(stripped_no_chassis: Path) -> None:
    with pytest.raises(MissingSectionError) as info:
        load(stripped_no_chassis)
    assert "chassis" in info.value.missing


def test_invalid_json_raises(tmp_path: Path) -> None:
    bad = tmp_path / "bad.svj.json"
    bad.write_text("{not json", encoding="utf-8")
    with pytest.raises(SvjLoadError):
        load(bad)


def test_nonexistent_file_raises(tmp_path: Path) -> None:
    with pytest.raises(SvjLoadError):
        load(tmp_path / "nope.svj.json")


def test_root_not_object_raises(tmp_path: Path) -> None:
    bad = tmp_path / "array.svj.json"
    bad.write_text("[]", encoding="utf-8")
    with pytest.raises(SvjLoadError):
        load(bad)


# --------------------------------------------------------------------------- #
# Optional sections                                                           #
# --------------------------------------------------------------------------- #


def test_optional_sections_may_be_absent(stripped_optional: Path) -> None:
    doc = load(stripped_optional)
    assert doc.metadata is None
    assert doc.aerodynamics is None
    assert doc.assetto is None
    # Required sections still intact.
    assert doc.vehicle_info.make == "Mazda"
    assert doc.chassis.mass_total > 0


# --------------------------------------------------------------------------- #
# Validation                                                                  #
# --------------------------------------------------------------------------- #


def test_validate_clean_example(example_doc: SvjDoc) -> None:
    errors = validate(example_doc)
    # The canonical fixture should pass cleanly.
    assert errors == []


def test_validate_flags_bad_mass(example_doc: SvjDoc) -> None:
    # Mutate a copy so we don't taint the cached fixture.
    from copy import deepcopy

    bad = deepcopy(example_doc)
    bad.chassis.mass_total = -1.0
    errs = validate(bad)
    assert any(e.path == "chassis.mass_total" for e in errs)


def test_validate_flags_wrong_coords(example_doc: SvjDoc) -> None:
    from copy import deepcopy

    bad = deepcopy(example_doc)
    assert bad.metadata is not None
    bad.metadata.coordinate_system = "iso8855"
    errs = validate(bad)
    assert any(e.path == "_metadata.coordinate_system" and e.severity == "error" for e in errs)


def test_validate_flags_missing_corner(example_doc: SvjDoc) -> None:
    from copy import deepcopy

    bad = deepcopy(example_doc)
    del bad.suspension.corners["RR"]
    errs = validate(bad)
    assert any("RR" in e.message for e in errs)
