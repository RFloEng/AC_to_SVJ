"""
Read Assetto Corsa cars straight from downloaded archives, without installing them.

Supported: ``.zip`` (standard library) and ``.rar`` / ``.7z`` (through the 7-Zip command
line, which has to be installed; nothing is bundled).

A mod archive is usually ``<car_folder>/data/...`` (sometimes inside ``content/cars/``
or with no folder at all).  ``list_archive_cars`` finds every car inside an archive;
``extract_car`` unpacks just what the converter reads (data, KN5 models, ui, skins) into
a scratch folder laid out like an installed car, so the normal pipeline runs on it.

Entries are selected by name through a whitelist and never written outside the
destination (no zip-slip), and sizes are capped.
"""
from __future__ import annotations

import os
import re
import shutil
import subprocess
import tempfile
import zipfile
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from typing import Callable, Optional

#: separator used in the names the GUI shows: ``<archive file name>::<car folder>``
SEP = "::"

ARCHIVE_SUFFIXES = (".zip", ".rar", ".7z")
_SEVENZIP_SUFFIXES = (".rar", ".7z")

# A car lives where ``data/<ini>`` or ``data.acd`` sits.
_DATA_INI = re.compile(r"^(?P<prefix>(?:.*/)?)data/[^/]+\.(?:ini|lut)$", re.IGNORECASE)
_DATA_ACD = re.compile(r"^(?P<prefix>(?:.*/)?)data\.acd$", re.IGNORECASE)

_SKIN_EXT = {".dds", ".png", ".jpg", ".jpeg", ".ini", ".json"}
_MAX_FILE = 2 * 1024 ** 3          # 2 GB per member
_MAX_TOTAL = 6 * 1024 ** 3         # 6 GB per car
_TOOL_TIMEOUT = 1800               # seconds for one 7-Zip call


@dataclass
class ArchiveCar:
    archive_path: Path
    prefix: str            # path of the car folder inside the archive ("" = root, ends with "/")
    name: str              # car folder name (archive stem when the car sits at the root)
    has_data: bool         # unpacked data/ present
    has_acd: bool          # data.acd present (needs unpacking when has_data is False)

    @property
    def zip_path(self) -> Path:            # name used before .rar / .7z were supported
        return self.archive_path

    @property
    def label(self) -> str:
        return f"{self.archive_path.name}{SEP}{self.name}"


class ArchiveToolError(RuntimeError):
    """A .rar / .7z archive could not be read (no 7-Zip, damaged, password protected)."""


def _norm(name: str) -> str:
    return name.replace("\\", "/")


# ── 7-Zip backend ─────────────────────────────────────────────────────────────

def find_7zip() -> Optional[str]:
    """Path of the 7-Zip command line (7z / 7zz / 7za), or None."""
    for exe in ("7z", "7zz", "7za"):
        p = shutil.which(exe)
        if p:
            return p
    for base in (os.environ.get("ProgramFiles"), os.environ.get("ProgramFiles(x86)"),
                 r"C:\Program Files", r"C:\Program Files (x86)"):
        if base:
            cand = Path(base) / "7-Zip" / "7z.exe"
            if cand.is_file():
                return str(cand)
    return None


def _run7z(args: list, tool: str) -> str:
    try:
        r = subprocess.run([tool, *args], capture_output=True, text=True,
                           encoding="utf-8", errors="replace", timeout=_TOOL_TIMEOUT)
    except (OSError, subprocess.TimeoutExpired) as e:
        raise ArchiveToolError(f"7-Zip failed: {e}") from e
    if r.returncode not in (0, 1):                       # 1 = warning only
        tail = (r.stderr or r.stdout or "").strip().splitlines()[-1:] or ["unknown error"]
        raise ArchiveToolError(f"7-Zip: {tail[0]}")
    return r.stdout


def _list_7z(path: Path, tool: str) -> list[tuple[str, int]]:
    """``[(name, size)]`` of the files in a .rar / .7z archive (directories left out)."""
    out = _run7z(["l", "-slt", "-ba", "-sccUTF-8", str(path)], tool)
    entries: list[tuple[str, int]] = []
    for block in re.split(r"\r?\n\r?\n", out):
        f: dict = {}
        for line in block.splitlines():
            k, sep, v = line.partition(" = ")
            if sep:
                f[k.strip()] = v.strip()
        if "Path" not in f:
            continue
        if f.get("Encrypted") == "+":
            raise ArchiveToolError(f"{path.name} is password protected")
        if f.get("Folder") == "+" or f.get("Attributes", "").startswith("D"):
            continue
        try:
            size = int(f.get("Size") or 0)
        except ValueError:
            size = 0
        entries.append((_norm(f["Path"]), size))
    return entries


def _entries(path: Path) -> list[tuple[str, int]]:
    """``[(name, size)]`` for a zip, rar or 7z archive."""
    if path.suffix.lower() == ".zip":
        with zipfile.ZipFile(path) as z:
            return [(_norm(i.filename), i.file_size) for i in z.infolist() if not i.is_dir()]
    tool = find_7zip()
    if tool is None:
        raise ArchiveToolError(
            f"{path.name}: 7-Zip is needed to read {path.suffix} files "
            "(install it from 7-zip.org and make sure 7z.exe is in its default folder or on PATH)")
    return _list_7z(path, tool)


# ── discovery ─────────────────────────────────────────────────────────────────

def list_archive_cars(archive_path: Path) -> list[ArchiveCar]:
    """Every car folder found in the archive.

    Empty for an unrelated archive; raises :class:`ArchiveToolError` when a .rar / .7z
    cannot be read (so the caller can tell the user why), while a damaged zip is
    treated as unrelated.
    """
    archive_path = Path(archive_path)
    try:
        entries = _entries(archive_path)
    except (zipfile.BadZipFile, OSError, NotImplementedError):
        return []
    roots: dict[str, dict] = {}
    for n, _size in entries:
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
        roots.setdefault(prefix, {"data": False, "acd": False})[kind] = True
    cars = []
    for prefix, f in sorted(roots.items()):
        parts = [p for p in prefix.split("/") if p]
        name = parts[-1] if parts else archive_path.stem
        cars.append(ArchiveCar(archive_path, prefix, name, f["data"], f["acd"]))
    return cars


def scan_archive_folder(folder: Path,
                        on_problem: Optional[Callable[[str], None]] = None) -> list[ArchiveCar]:
    """All cars in the ``.zip`` / ``.rar`` / ``.7z`` files directly inside ``folder``.

    ``on_problem`` receives one message per archive that could not be read.
    """
    out: list[ArchiveCar] = []
    files = sorted(p for p in Path(folder).iterdir()
                   if p.is_file() and p.suffix.lower() in ARCHIVE_SUFFIXES)
    for a in files:
        try:
            out.extend(list_archive_cars(a))
        except ArchiveToolError as e:
            if on_problem:
                on_problem(str(e))
    return out


scan_zip_folder = scan_archive_folder          # name used before .rar / .7z support


# ── extraction ────────────────────────────────────────────────────────────────

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


def _safe_rel(rel: str) -> bool:
    """A relative path with no '..' / absolute / drive parts."""
    if not rel or rel.startswith("/") or re.match(r"^[A-Za-z]:", rel):
        return False
    return ".." not in rel.split("/")


def extract_car(car: ArchiveCar, dest_root: Path, include_skins: bool = True) -> Path:
    """Unpack ``car`` under ``dest_root/<name>`` and return that folder.

    Raises ``ValueError`` for an archive that exceeds the size caps and
    :class:`ArchiveToolError` for a .rar / .7z that 7-Zip cannot extract.
    """
    dest = Path(dest_root) / car.name
    dest.mkdir(parents=True, exist_ok=True)
    wanted: list[str] = []
    total = 0
    for name, size in _entries(car.archive_path):
        if not name.startswith(car.prefix):
            continue
        rel = name[len(car.prefix):]
        if not rel or not _wanted(rel, include_skins):
            continue
        if size > _MAX_FILE:
            raise ValueError(f"{name}: {size} bytes exceeds the per-file limit")
        total += size
        if total > _MAX_TOTAL:
            raise ValueError(f"{car.archive_path.name}: car data exceeds the size limit")
        wanted.append(name)

    if car.archive_path.suffix.lower() == ".zip":
        _extract_zip(car, dest, wanted)
    else:
        _extract_7z(car, dest, wanted)
    return dest


def _extract_zip(car: ArchiveCar, dest: Path, wanted: list[str]) -> None:
    dest_resolved = dest.resolve()
    keep = set(wanted)
    with zipfile.ZipFile(car.archive_path) as z:
        for info in z.infolist():
            name = _norm(info.filename)
            if info.is_dir() or name not in keep:
                continue
            target = (dest / name[len(car.prefix):]).resolve()
            try:
                target.relative_to(dest_resolved)
            except ValueError:
                continue                  # '..' or absolute path: never write outside dest
            target.parent.mkdir(parents=True, exist_ok=True)
            with z.open(info) as src, open(target, "wb") as out:
                shutil.copyfileobj(src, out)


def _extract_7z(car: ArchiveCar, dest: Path, wanted: list[str]) -> None:
    tool = find_7zip()
    if tool is None or not wanted:
        return
    safe = [n for n in wanted if _safe_rel(n)]
    with tempfile.TemporaryDirectory(prefix="ac_svj_7z_") as stage:
        listfile = Path(stage) / "names.txt"
        listfile.write_text("\n".join(safe), encoding="utf-8")
        out_dir = Path(stage) / "out"
        _run7z(["x", "-y", "-bd", "-aoa", "-spd", "-scsUTF-8", f"-o{out_dir}",
                str(car.archive_path), f"@{listfile}"], tool)
        base = out_dir
        for part in [p for p in car.prefix.split("/") if p]:
            base = base / part
        if not base.is_dir():
            return
        out_resolved = out_dir.resolve()
        for f in base.rglob("*"):
            if not f.is_file():
                continue
            try:
                f.resolve().relative_to(out_resolved)
            except ValueError:
                continue
            target = dest / f.relative_to(base)
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.move(str(f), str(target))


def split_label(label: str) -> Optional[tuple[str, str]]:
    """``"a.zip::car"`` -> ``("a.zip", "car")``; None for a plain folder name."""
    if SEP in label:
        z, c = label.split(SEP, 1)
        return z, c
    return None
