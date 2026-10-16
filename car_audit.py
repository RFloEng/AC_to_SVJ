"""
Audit a (huge) cars folder: which cars cannot be converted yet, and why.

For every car folder, and every car inside a .zip / .rar / .7z next to them:

* **data**: ``unpacked`` (a ``data/`` folder with ini files), ``needs_unpack`` (only an
  encrypted ``data.acd``) or ``no_data``;
* **kn5** (optional, reads the model files, slow on a big folder): ``clean``,
  ``encrypted_copy_found`` (an unencrypted copy was found, see docs/ENCRYPTED_CARS.md),
  ``encrypted_no_copy`` or ``no_kn5``.

Detection only; nothing is decrypted.  ``write_reports`` writes a CSV with every car
and plain lists (one name per line) of the cars that need attention, to repair later.
"""
from __future__ import annotations

import csv
from datetime import datetime
from pathlib import Path
from typing import Callable, Optional

from car_archives import ArchiveToolError, scan_archive_folder


def _data_state(car: Path) -> str:
    d = car / "data"
    try:
        if d.is_dir() and any(d.glob("*.ini")):
            return "unpacked"
        if (car / "data.acd").is_file():
            return "needs_unpack"
    except OSError:
        pass
    return "no_data"


def _kn5_state(car: Path) -> str:
    try:
        from kn5_reader import resolve_car_kn5
    except ImportError:
        return "not_checked"
    try:
        c = resolve_car_kn5(car)
    except Exception:                                          # noqa: BLE001
        return "not_checked"
    skipped = bool(getattr(c, "skipped_encrypted", None))
    if c.path is None:
        return "encrypted_no_copy" if skipped else "no_kn5"
    return "encrypted_copy_found" if skipped else "clean"


def _looks_like_car(d: Path) -> bool:
    try:
        return ((d / "data").is_dir() or (d / "data.acd").is_file()
                or any(d.glob("*.kn5")))
    except OSError:
        return False


def audit_folder(root: Path, check_kn5: bool = False,
                 progress: Optional[Callable[[int, int, str], None]] = None) -> list[dict]:
    """One row per car: ``{"name", "source", "data", "kn5", "note"}``."""
    root = Path(root)
    subdirs = sorted(d for d in root.iterdir() if d.is_dir())
    cars = [d for d in subdirs if _looks_like_car(d)]
    problems: list[str] = []
    archived = scan_archive_folder(root, problems.append)
    total = len(cars) + len(archived)
    rows: list[dict] = []
    for i, d in enumerate(cars):
        if progress:
            progress(i + 1, total, d.name)
        rows.append({"name": d.name, "source": "folder", "data": _data_state(d),
                     "kn5": _kn5_state(d) if check_kn5 else "not_checked", "note": ""})
    for j, a in enumerate(archived):
        if progress:
            progress(len(cars) + j + 1, total, a.label)
        data = "unpacked" if a.has_data else ("needs_unpack" if a.has_acd else "no_data")
        rows.append({"name": a.label, "source": "archive", "data": data,
                     "kn5": "not_checked", "note": ""})
    for msg in problems:
        rows.append({"name": msg.split(":")[0], "source": "archive", "data": "unreadable",
                     "kn5": "not_checked", "note": msg})
    return rows


def write_reports(rows: list[dict], out_dir: Path, stamp: Optional[str] = None) -> dict:
    """Write ``cars_audit_<stamp>.csv`` and the plain lists; returns the paths + counts."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = stamp or datetime.now().strftime("%Y%m%d_%H%M%S")
    paths: dict = {}
    p = out_dir / f"cars_audit_{stamp}.csv"
    with open(p, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=["name", "source", "data", "kn5", "note"])
        w.writeheader()
        w.writerows(rows)
    paths["csv"] = p
    for key, label, pick in (
            ("need_unpack", "cars_need_unpack", lambda r: r["data"] == "needs_unpack"),
            ("kn5_encrypted", "cars_kn5_encrypted", lambda r: r["kn5"] == "encrypted_no_copy"),
            ("unreadable", "cars_unreadable_archives", lambda r: r["data"] == "unreadable")):
        names = [r["name"] for r in rows if pick(r)]
        if names:
            q = out_dir / f"{label}_{stamp}.txt"
            q.write_text("\n".join(names) + "\n", encoding="utf-8")
            paths[key] = q
    paths["counts"] = {
        "cars": sum(1 for r in rows if r["data"] != "unreadable"),
        "ready": sum(1 for r in rows if r["data"] == "unpacked"
                     and r["kn5"] in ("not_checked", "clean", "encrypted_copy_found")),
        "need_unpack": sum(1 for r in rows if r["data"] == "needs_unpack"),
        "kn5_encrypted": sum(1 for r in rows if r["kn5"] == "encrypted_no_copy"),
        "unreadable": sum(1 for r in rows if r["data"] == "unreadable"),
    }
    return paths
