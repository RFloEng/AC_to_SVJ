"""An ISO 8855 file (SVJ 0.109+) and the same car in SAE J670 axes load to the same data."""
import json
from pathlib import Path

import pytest

import svj_loader

REPO = Path(__file__).resolve().parents[2]


def _build(frame, tmp_path):
    import sys
    sys.path.insert(0, str(REPO))
    from converter import _clean, build_svj, read_car_directory
    src = REPO / "test_car"
    if not src.is_dir():
        pytest.skip("test_car missing")
    ini, cm, ctrl, dd = read_car_directory(src)
    svj, _, _ = build_svj(ini, cm, data_dir=dd, ctrl_files=ctrl, include_plots=False, frame=frame)
    p = tmp_path / f"{frame}.svj.json"
    p.write_text(json.dumps(_clean(svj)), encoding="utf-8")
    return p


def test_iso_and_sae_files_load_to_the_same_corners(tmp_path):
    iso = svj_loader.load(_build("ISO_8855", tmp_path))
    sae = svj_loader.load(_build("SAE_J670", tmp_path))
    assert any("ISO 8855" in n for n in iso.notes)
    for c in ("FL", "FR", "RL", "RR"):
        a, b = iso.suspension[c], sae.suspension[c]
        assert a.position == pytest.approx(b.position, abs=1e-9), c
        assert a.raw.get("spring") == b.raw.get("spring") and a.raw.get("damper") == b.raw.get("damper"), c
        assert a.topology == b.topology, c
    assert iso.chassis.center_of_gravity == pytest.approx(sae.chassis.center_of_gravity, abs=1e-9)
    assert iso.chassis.center_of_gravity[2] < 0          # the writers keep working in SAE axes (Z down)


def test_sae_signed_tyre_sets_are_read_with_iso_signs(tmp_path):
    p = _build("ISO_8855", tmp_path)
    d = json.loads(p.read_text(encoding="utf-8"))
    from svj_migrate.tiremodel import to_iso_set
    for s in d["tires"]["sets"].values():
        s["pacejka"] = to_iso_set(s["pacejka"])           # now SAE-signed and declared so
    q = tmp_path / "sae_tyres.svj.json"
    q.write_text(json.dumps(d), encoding="utf-8")
    ref = svj_loader.load(p)
    got = svj_loader.load(q)
    assert any("rewritten from SAE" in n for n in got.notes)
    assert got.tyres.sets == ref.tyres.sets
