"""SVJ 0.112 variants[]: a variant of the car converts as its own AC car."""
from __future__ import annotations

import json
import zipfile
from pathlib import Path

import pytest

import pack_car
import svj_loader
import svj_to_ac

EXAMPLE = Path(__file__).resolve().parents[1] / "examples" / "mx5_nd_club.svj.json"


def _file(tmp_path: Path, variants) -> Path:
    d = json.loads(EXAMPLE.read_text(encoding="utf-8"))
    d["variants"] = variants
    p = tmp_path / "car.svj.json"
    p.write_text(json.dumps(d), encoding="utf-8")
    return p


def _mass(p: Path, variant=None) -> float:
    return svj_loader.load(p, variant=variant).chassis.mass_total


def test_variants_are_listed_and_the_base_is_the_default(tmp_path) -> None:
    p = _file(tmp_path, [{"id": "heavy", "set": {"chassis.mass_total": 1500}}])
    assert svj_loader.variant_ids(p) == ["heavy"]
    assert svj_loader.variant_ids(EXAMPLE) == []
    base = svj_loader.load(p)
    assert base.chassis.mass_total != 1500
    assert any("1 variant" in n and "heavy" in n for n in base.notes)


def test_a_variant_is_applied_before_anything_else(tmp_path) -> None:
    p = _file(tmp_path, [{"id": "heavy", "name": "Heavy", "set": {"chassis.mass_total": 1500}}])
    doc = svj_loader.load(p, variant="heavy")
    assert doc.chassis.mass_total == 1500 and doc.metadata.raw["variant"] == "heavy"
    assert pack_car.model_id(doc).endswith("_heavy")
    assert pack_car.model_id(svj_loader.load(p)) + "_heavy" == pack_car.model_id(doc)


def test_unknown_or_broken_variant_is_a_load_error(tmp_path) -> None:
    p = _file(tmp_path, [{"id": "bad", "set": {"chassis.no_such_key": 1}}])
    with pytest.raises(svj_loader.SvjLoadError, match="not found"):
        svj_loader.load(p, variant="nope")
    with pytest.raises(svj_loader.SvjLoadError, match="cannot be applied"):
        svj_loader.load(p, variant="bad")


def test_all_variants_write_one_car_each(tmp_path) -> None:
    p = _file(tmp_path, [{"id": "a", "set": {"chassis.mass_total": 1400}},
                         {"id": "b", "set": {"chassis.mass_total": 1600}}])
    res = svj_to_ac.convert_one(p, tmp_path / "out", include_mesh=False, variant="all")
    assert res.success and res.summary["variants"] == ["base", "a", "b"]
    names = sorted(Path(z).name for z in res.summary["zips"])
    assert len(names) == 3 and names[1].endswith("_a.zip") and names[2].endswith("_b.zip")
    assert not names[0].endswith(("_a.zip", "_b.zip"))                        # the base car
    with zipfile.ZipFile(res.summary["zips"][1]) as z:                     # variant a
        assert any(n.endswith("data/car.ini") for n in z.namelist())
