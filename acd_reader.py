"""AC car-data loader.

Loads ini files for an AC car. Supports cars where the ``data/`` folder
has been unpacked (mod cars and stock Kunos cars that the user has
extracted via Content Manager).

When a car ships with only the encrypted ``data.acd`` archive and no
unpacked ``data/`` directory, ``open_car`` returns an empty ini map
together with a clear ``source_note`` explaining the workaround: open
the car in Content Manager → Tools → Unpack Data, then re-run the
converter.

Public API
──────────
  * open_car(car_path) → (ini_files, data_dir, source_note)
  * UnpackNeededError — raised by callers that want to handle the
    encrypted-only case explicitly.

Source notes
────────────
  * "unpacked data/ (N ini)"  — found and read N ini files from data/
  * "needs unpack: data.acd"  — only the encrypted archive is present
  * "no data/ folder and no data.acd" — neither is present
"""
from __future__ import annotations

from pathlib import Path
from typing import Optional


class UnpackNeededError(RuntimeError):
    """Raised when a car only ships with an encrypted ``data.acd`` archive
    and no unpacked ``data/`` directory. The caller should surface this
    to the user with the recommended Content Manager workaround."""

    def __init__(self, car_path: Path):
        self.car_path = car_path
        super().__init__(
            f"Car {car_path.name!r} ships with an encrypted data.acd and no "
            f"unpacked data/ folder. To convert this car, open it in Content "
            f"Manager and use \"Tools → Unpack Data\" (or extract the data/ "
            f"folder by any other means), then re-run the converter."
        )


def open_car(car_path: Path,
             required_names: Optional[list[str]] = None
             ) -> tuple[dict[str, str], Path, str]:
    """Open an AC car folder.

    Returns ``(ini_files, data_dir, source_note)`` where:
      * ``ini_files`` maps filename → UTF-8 text for every *.ini we could
        load. Empty dict if the car needs unpacking or has no data at all.
      * ``data_dir`` is the directory holding .lut and .ini files for
        downstream parsers. When the car has no readable data, this falls
        back to the car_path itself (so callers don't crash on missing
        attribute access).
      * ``source_note`` is a short human-readable string describing where
        the data came from (or why we couldn't read it).

    This function never raises — when only ``data.acd`` is present it
    returns an empty ``ini_files`` and a clear note explaining the
    Content Manager unpack workaround. Callers in the batch flow should
    check ``ini_files`` is non-empty before proceeding, and log the
    ``source_note`` on skip so the user knows which cars need attention.
    """
    data_dir = car_path / "data"
    if data_dir.is_dir() and any(data_dir.glob("*.ini")):
        ini_files = {}
        for p in data_dir.glob("*.ini"):
            ini_files[p.name] = p.read_text(encoding="utf-8", errors="replace")
        return ini_files, data_dir, f"unpacked data/ ({len(ini_files)} ini)"

    acd = car_path / "data.acd"
    if acd.is_file():
        return ({}, data_dir if data_dir.is_dir() else car_path,
                f"needs unpack: data.acd present (use Content Manager → "
                f"Tools → Unpack Data on '{car_path.name}')")

    return ({}, data_dir if data_dir.is_dir() else car_path,
            "no data/ folder and no data.acd")
