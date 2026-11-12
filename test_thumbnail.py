from pathlib import Path

import batch_convert as B
import car_archives as A


def test_thumbnail_is_the_first_skins_preview(tmp_path):
    for skin in ("b_skin", "a_skin"):
        (tmp_path / "skins" / skin).mkdir(parents=True)
        (tmp_path / "skins" / skin / "Preview.JPG").write_bytes(skin.encode())
    t = B.find_car_thumbnail(tmp_path)
    assert t is not None and t.parent.name == "a_skin"


def test_thumbnail_falls_back_to_a_skin_that_has_one(tmp_path):
    (tmp_path / "skins" / "a_skin").mkdir(parents=True)
    (tmp_path / "skins" / "b_skin").mkdir(parents=True)
    (tmp_path / "skins" / "b_skin" / "preview.jpg").write_bytes(b"x")
    assert B.find_car_thumbnail(tmp_path).parent.name == "b_skin"


def test_no_thumbnail_when_there_are_no_skins(tmp_path):
    assert B.find_car_thumbnail(tmp_path) is None


def test_archive_extraction_keeps_previews_even_without_skins():
    assert A._wanted("skins/red/preview.jpg", include_skins=False)
    assert not A._wanted("skins/red/body.dds", include_skins=False)


def test_tyre_plots_are_jpeg():
    import tire_lab
    r = tire_lab.run_bench(tire_lab.ACTyreParams())
    for data in (r.lateral_png, r.longitudinal_png, r.mu_vs_fz_png):
        assert data[:3] == bytes([0xFF, 0xD8, 0xFF])
