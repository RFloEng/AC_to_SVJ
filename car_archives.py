"""
Read Assetto Corsa cars straight from downloaded .zip files, without installing them.

A mod archive is usually ``<car_folder>/data/...`` (sometimes inside ``content/cars/``
or with no folder at all).  ``list_archive_cars`` finds every car inside a zip;
``extract_car`` unpacks just what the converter reads (data, KN5 models, ui, skins) into
a scratch folder laid out like an installed car, so the normal pipeline runs on it.

Only the standard library is used.  Entries are extracted by name through a
whitelist and never outside the destination (no zip-slip), and sizes are capped.
"""
from __future__ import annotations

import re
import shutil
import zipfile
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from typing import Optional

#: separator used in the names the GUI shows: ``<zip file name>::<car folder>``
SEP = "::"

# A car lives where ``data/<ini>`` or ``data.acd`` sits.
_DATA_INI = re.compile(r"^(?P<prefix>(?:.*/)?)data/[^/]+\.(?:ini|lut)$", re.IGNORECASE)
_DATA_ACD = re.compile(r"^(?P<prefix>(?:.*/)?)data\.acd$", re.IGNORECASE)

_SKIN_EXT = {".dds", ".png", ".jpg", ".jpeg", ".ini", ".json"}
_MAX_FILE = 2 * 1024 ** 3          # 2 GB per member
_MAX_TOTAL = 6 * 1024 ** 3         # 6 GB per car


@dataclass
class ArchiveCar:
    zip_path: Path
    prefix: str            # path of the car folder inside the zip ("" = zip root, ends with "/")
    name: str              # car folder name (zip stem when the car sits at the zip root)
    has_data: bool         # unpacked data/ present
    has_acd: bool          # data.acd present (needs unpacking when has_data is False)

    @property
    def label(self) -> str:
        return f"{self.zip_path.name}{SEP}{self.name}"


def _norm(name: str) -> str:
    return name.replace("\\", "/")


def list_archive_cars(zip_path: Path) -> list[ArchiveCar]:
    """Every car folder found in the zip (empty for a damaged or unrelated zip)."""
    zip_path = Path(zip_path)
    try:
        with zipfile.ZipFile(zip_path) as z:
            names = [_norm(n) for n in z.namelist()]
    except (zipfile.BadZipFile, OSError, NotImplementedError):
        return []
    roots: dict[str, dict] = {}
    for n in names:
        if n.endswith("/"):
            continue
        m = _DATA_INI.match(n)
        kind = "data"
        if not m:
            m = _DATA_ACD.match(n)
            kind = "acd"
        if not m:
            continue
        prefix = m.group("prefix")
        parts = [p for p in prefix.split("/") if p]
        # data/ folders of accessories or backups are not cars
        if any(p.lower() in ("extension", "skins", "unencrypted", "sfx", "ui") for p in parts):
            continue
        roots.setdefault(prefix, {"data": False, "acd": False})[kind == "data" and "data" or "acd"] = True
    cars = []
    for prefix, f in sorted(roots.items()):
        parts = [p for p in prefix.split("/") if p]
        name = parts[-1] if parts else zip_path.stem
        cars.append(ArchiveCar(zip_path, prefix, name, f["data"], f["acd"]))
    return cars


def scan_zip_folder(folder: Path) -> list[ArchiveCar]:
    """All cars in the ``*.zip`` files directly inside ``folder``."""
    out: list[ArchiveCar] = []
    for z in sorted(Path(folder).glob("*.zip")):
        out.extend(list_archive_cars(z))
    return out


def _wanted(rel: str, include_skins: bool) -> bool:
    """Which members (path relative to the car folder) the converter can use."""
    low = rel.lower()
    parts = low.split("/")
    if parts[0] == "data":
        return True
    if parts[0] == "data.acd" and len(parts) == 1:
        return True
    if parts[0] == "ui":
        return True
    if parts[0] in ("extension", "sfx", "animations", "analog_instruments", "setups"):
        return False
    if low.endswith(".kn5"):
        return True                      # car.kn5, LODs, and an unencrypted/ copy
    if parts[0] == "skins" and include_skins:
        return PurePosixPath(low).suffix in _SKIN_EXT
    return False


def extract_car(car: ArchiveCar, dest_root: Path, include_skins: bool = True) -> Path:
    """Unpack ``car`` under ``dest_root/<name>`` and return that folder.

    Raises ``ValueError`` for an archive that exceeds the size caps.
    """
    dest = Path(dest_root) / car.name
    dest.mkdir(parents=True, exist_ok=True)
    dest_resolved = dest.resolve()
    total = 0
    with zipfile.ZipFile(car.zip_path) as z:
        for info in z.infolist():
            name = _norm(info.filename)
            if info.is_dir() or not name.startswith(car.prefix):
                continue
            rel = name[len(car.prefix):]
            if not rel or not _wanted(rel, include_skins):
                continue
            if info.file_size > _MAX_FILE:
                raise ValueError(f"{name}: {info.file_size} bytes exceeds the per-file limit")
            total += info.file_size
            if total > _MAX_TOTAL:
                raise ValueError(f"{car.zip_path.name}: car data exceeds the size limit")
            target = (dest / rel).resolve()
            try:
                target.relative_to(dest_resolved)
            except ValueError:
                continue                  # '..' or absolute path: never write outside dest
            target.parent.mkdir(parents=True, exist_ok=True)
            with z.open(info) as src, open(target, "wb") as out:
                shutil.copyfileobj(src, out)
    return dest


def split_label(label: str) -> Optional[tuple[str, str]]:
    """``"a.zip::car"`` -> ``("a.zip", "car")``; None for a plain folder name."""
    if SEP in label:
        z, c = label.split(SEP, 1)
        return z, c
    return None
