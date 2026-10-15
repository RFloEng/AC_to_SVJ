"""Tests for reading cars from .zip files (synthetic archives, no AC content)."""
from __future__ import annotations

import sys
import tempfile
import zipfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import car_archives
from car_archives import (SEP, ArchiveToolError, extract_car, find_7zip, list_archive_cars,
                          scan_archive_folder, scan_zip_folder, split_label)

INI = "[HEADER]\nVERSION=1\n"


def _zip(path: Path, files: dict) -> Path:
    with zipfile.ZipFile(path, "w") as z:
        for n, b in files.items():
            z.writestr(n, b)
    return path


def test_finds_cars_at_any_depth_and_skips_accessories():
    with tempfile.TemporaryDirectory() as t:
        z = _zip(Path(t) / "pack.zip", {
            "content/cars/alpha/data/car.ini": INI,
            "content/cars/alpha/alpha.kn5": b"k",
            "content/cars/alpha/extension/data/x.ini": INI,      # accessory, not a car
            "beta/data.acd": b"enc",                              # encrypted only
            "readme.txt": "hi",
        })
        cars = {c.name: c for c in list_archive_cars(z)}
        assert set(cars) == {"alpha", "beta"}
        assert cars["alpha"].has_data and not cars["alpha"].has_acd
        assert cars["beta"].has_acd and not cars["beta"].has_data
        assert cars["alpha"].label == f"pack.zip{SEP}alpha"
        assert split_label(cars["alpha"].label) == ("pack.zip", "alpha")
        assert split_label("plain_folder") is None


def test_car_at_zip_root_is_named_after_the_zip():
    with tempfile.TemporaryDirectory() as t:
        z = _zip(Path(t) / "my_mod_v2.zip", {"data/car.ini": INI, "ui/ui_car.json": "{}"})
        (car,) = list_archive_cars(z)
        assert car.prefix == "" and car.name == "my_mod_v2"


def test_extract_keeps_only_what_the_converter_reads():
    with tempfile.TemporaryDirectory() as t:
        z = _zip(Path(t) / "p.zip", {
            "car/data/car.ini": INI, "car/data/power.lut": "0|0\n",
            "car/car.kn5": b"k", "car/car_LOD_B.kn5": b"k",
            "car/unencrypted/car_decrypted.kn5": b"k",
            "car/ui/ui_car.json": "{}",
            "car/skins/red/livery.dds": b"d", "car/skins/red/notes.txt": "x",
            "car/sfx/engine.bank": b"huge", "car/extension/acc.kn5": b"k",
            "other/data/car.ini": INI,                           # a different car in the zip
        })
        (car,) = [c for c in list_archive_cars(z) if c.name == "car"]
        out = Path(t) / "x"
        d = extract_car(car, out)
        got = sorted(p.relative_to(d).as_posix() for p in d.rglob("*") if p.is_file())
        assert got == ["car.kn5", "car_LOD_B.kn5", "data/car.ini", "data/power.lut",
                       "skins/red/livery.dds", "ui/ui_car.json",
                       "unencrypted/car_decrypted.kn5"], got
        d2 = extract_car(car, Path(t) / "y", include_skins=False)
        assert not (d2 / "skins").exists()


def test_extract_never_writes_outside_the_destination():
    with tempfile.TemporaryDirectory() as t:
        z = _zip(Path(t) / "evil.zip", {
            "car/data/car.ini": INI,
            "car/data/../../../escaped.ini": "x",
            "car/data/.." + chr(92) + ".." + chr(92) + "escaped2.ini": "x",
        })
        (car,) = list_archive_cars(z)
        extract_car(car, Path(t) / "out")
        assert not (Path(t) / "escaped.ini").exists()
        assert not (Path(t) / "escaped2.ini").exists()
        assert not any(p.name.startswith("escaped") for p in Path(t).rglob("*"))


def test_bad_zip_and_folder_scan():
    with tempfile.TemporaryDirectory() as t:
        (Path(t) / "broken.zip").write_bytes(b"not a zip")
        _zip(Path(t) / "good.zip", {"c/data/car.ini": INI})
        labels = [c.label for c in scan_zip_folder(Path(t))]
        assert labels == [f"good.zip{SEP}c"]


def test_extracted_car_runs_through_the_normal_reader():
    from converter import read_car_directory
    src = Path(__file__).parent / "test_car"
    if not (src / "data").is_dir():
        return
    with tempfile.TemporaryDirectory() as t:
        zp = Path(t) / "dl.zip"
        with zipfile.ZipFile(zp, "w") as z:
            for f in src.rglob("*"):
                if f.is_file():
                    z.write(f, "mycar/" + f.relative_to(src).as_posix())
        (car,) = list_archive_cars(zp)
        d = extract_car(car, Path(t) / "x")
        ini, cm, ctrl, dd = read_car_directory(d)
        assert "car.ini" in ini and dd == d / "data"


def _make_7z(path: Path, files: dict) -> bool:
    """Build a .7z with the 7-Zip command line; False when 7-Zip is not installed."""
    tool = find_7zip()
    if tool is None:
        return False
    import subprocess
    with tempfile.TemporaryDirectory() as t:
        for n, b in files.items():
            f = Path(t) / n
            f.parent.mkdir(parents=True, exist_ok=True)
            f.write_bytes(b if isinstance(b, bytes) else b.encode())
        subprocess.run([tool, "a", "-bd", "-y", str(path), "*"], cwd=t, check=True,
                       capture_output=True)
    return True


def test_7z_archive_lists_and_extracts_like_a_zip():
    with tempfile.TemporaryDirectory() as t:
        files = {"content/cars/gamma/data/car.ini": INI, "content/cars/gamma/gamma.kn5": b"k",
                 "content/cars/gamma/ui/ui_car.json": "{}",
                 "content/cars/gamma/sfx/e.bank": b"big", "content/cars/gamma/skins/a/x.dds": b"d"}
        if not _make_7z(Path(t) / "dl.7z", files):
            return                                   # no 7-Zip here: nothing to check
        (car,) = list_archive_cars(Path(t) / "dl.7z")
        assert car.name == "gamma" and car.has_data
        d = extract_car(car, Path(t) / "x")
        got = sorted(p.relative_to(d).as_posix() for p in d.rglob("*") if p.is_file())
        assert got == ["data/car.ini", "gamma.kn5", "skins/a/x.dds", "ui/ui_car.json"], got
        assert [c.label for c in scan_archive_folder(Path(t))] == [f"dl.7z{SEP}gamma"]
    with tempfile.TemporaryDirectory() as t:
        if _make_7z(Path(t) / "root.7z", {"data/car.ini": INI, "car.kn5": b"k"}):
            (car,) = list_archive_cars(Path(t) / "root.7z")
            assert car.name == "root" and car.prefix == ""
            d = extract_car(car, Path(t) / "x")
            assert (d / "data" / "car.ini").is_file() and (d / "car.kn5").is_file()


def test_rar_without_7zip_gives_a_clear_message():
    real = car_archives.find_7zip
    car_archives.find_7zip = lambda: None
    try:
        with tempfile.TemporaryDirectory() as t:
            rar = Path(t) / "mod.rar"
            rar.write_bytes(b"Rar!")
            try:
                list_archive_cars(rar)
                raise AssertionError("expected ArchiveToolError")
            except ArchiveToolError as e:
                assert "7-Zip" in str(e) and "mod.rar" in str(e)
            problems: list = []
            assert scan_archive_folder(Path(t), problems.append) == [] and len(problems) == 1
    finally:
        car_archives.find_7zip = real


ALL_TESTS = [v for k, v in sorted(globals().items()) if k.startswith("test_")]


def run() -> bool:
    ok = True
    for fn in ALL_TESTS:
        try:
            fn()
            print(f"  ✓  {fn.__name__}")
        except Exception as e:                    # noqa: BLE001
            ok = False
            print(f"  ✗  {fn.__name__}: {type(e).__name__}: {e}")
    return ok


if __name__ == "__main__":
    sys.exit(0 if run() else 1)
