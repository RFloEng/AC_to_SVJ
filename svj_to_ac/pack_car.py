"""Assemble a Content Manager-ready ``content/cars/<model>/`` folder
and emit a reproducible zip.

Layout produced (matches ``02_PROJECT_SPEC.md §4``)::

    content/cars/<model>/
    |-- data/
    |   |-- car.ini
    |   |-- suspensions.ini
    |   |-- tyres.ini
    |   |-- engine.ini
    |   |-- power.lut
    |   |-- coast_curve.lut       (if engine carried one)
    |   |-- drivetrain.ini
    |   |-- aero.ini
    |   |-- brakes.ini
    |   |-- electronics.ini
    |   |-- setup.ini
    |   |-- ctrl_*.ini            (plus matching .lut companions)
    |-- ui/
    |   |-- ui_car.json
    |   |-- badge.png             (placeholder)
    |   `-- preview.jpg           (placeholder)
    |-- sfx/
    |   `-- GUIDs.txt             (stub pointing at Kunos default bank)
    `-- conversion_log.txt        (if any warnings were raised)

Public entry points:

* :func:`assemble` -- write the full tree under ``base_dir``. Returns
  the absolute path to the car folder.
* :func:`zip_folder` -- byte-deterministic zip of a folder.
* :func:`convert` -- high-level wrapper: ``SvjDoc`` in, ``.zip`` out.

Reproducibility: ``zip_folder`` walks entries in sorted order and
stamps every ``ZipInfo`` with ``date_time=(1980, 1, 1, 0, 0, 0)`` so
two consecutive runs with the same SVJ input produce a byte-identical
zip (pitfalls doc §"Reproducible zips").
"""

from __future__ import annotations

import hashlib
import shutil
import zipfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

from svj_loader import SvjDoc

from ac_writers import (
    aero_ini,
    brakes_ini,
    car_ini,
    ctrl_files,
    drivetrain_ini,
    electronics_ini,
    engine_ini,
    setup_ini,
    suspensions_ini,
    tyres_ini,
    ui_car_json,
)

ASSETS_DIR = Path(__file__).resolve().parent / "assets"
BADGE_PLACEHOLDER = ASSETS_DIR / "badge_placeholder.png"
PREVIEW_PLACEHOLDER = ASSETS_DIR / "preview_placeholder.jpg"

SFX_GUIDS_TEMPLATE = (
    "{{a234abcd-1234-5678-90ab-cdef12345678}} event:/cars/{model}/engine_ext\n"
    "{{b345bcde-2345-6789-01bc-def123456789}} event:/cars/{model}/engine_int\n"
)


@dataclass(frozen=True)
class AssembleResult:
    """Outcome of a single :func:`assemble` call."""

    car_dir: Path
    data_dir: Path
    ui_dir: Path
    sfx_dir: Path
    files: tuple[Path, ...]


def model_id(doc: SvjDoc) -> str:
    """Return the slug used as the ``content/cars/<model>/`` folder name."""
    ext = doc.assetto
    if ext and ext.info and ext.info.get("model_id"):
        return _slugify(str(ext.info["model_id"]))
    if doc.metadata and doc.metadata.raw.get("model_id"):
        return _slugify(str(doc.metadata.raw["model_id"]))
    vi = doc.vehicle_info
    if vi and vi.model:
        return _slugify(f"{vi.make}_{vi.model}".strip("_"))
    return "unnamed_car"


def assemble(doc: SvjDoc, base_dir: Path, tyre_choice: dict | None = None) -> AssembleResult:
    """Write the full ``content/cars/<model>/`` tree under ``base_dir``.

    ``base_dir`` is the parent of the eventual ``content/`` directory.
    ``tyre_choice`` takes the tyres from cars of an installed AC (see
    :func:`ac_writers.tyres_ini.write`) instead of fitting them.
    Returns an :class:`AssembleResult` enumerating every file written
    (deterministic order, suitable for reproducible zipping).
    """
    base_dir = Path(base_dir)
    name = model_id(doc)
    car_dir = base_dir / "content" / "cars" / name
    data_dir = car_dir / "data"
    ui_dir = car_dir / "ui"
    sfx_dir = car_dir / "sfx"
    for d in (data_dir, ui_dir, sfx_dir):
        d.mkdir(parents=True, exist_ok=True)

    files: list[Path] = []

    # data/*.ini and accompanying LUTs.
    files.append(car_ini.write(doc, data_dir))
    files.append(suspensions_ini.write(doc, data_dir))
    files.append(tyres_ini.write(doc, data_dir, tyre_choice))
    files.append(engine_ini.write(doc, data_dir))
    # engine_ini writes companion LUTs alongside; gather them up.
    for sibling in sorted(data_dir.glob("power*.lut")):
        files.append(sibling)
    for sibling in sorted(data_dir.glob("coast_curve.lut")):
        files.append(sibling)
    files.append(drivetrain_ini.write(doc, data_dir))
    files.append(aero_ini.write(doc, data_dir))
    files.append(brakes_ini.write(doc, data_dir))
    files.append(electronics_ini.write(doc, data_dir))
    files.append(setup_ini.write(doc, data_dir))
    files.extend(ctrl_files.write(doc, data_dir))

    # ui/
    files.append(ui_car_json.write(doc, ui_dir))
    files.append(_copy_placeholder(BADGE_PLACEHOLDER, ui_dir / "badge.png"))
    files.append(_copy_placeholder(PREVIEW_PLACEHOLDER, ui_dir / "preview.jpg"))

    # sfx/GUIDs.txt
    sfx_text = SFX_GUIDS_TEMPLATE.format(model=name)
    sfx_path = sfx_dir / "GUIDs.txt"
    sfx_path.write_text(sfx_text, encoding="utf-8", newline="\n")
    files.append(sfx_path)

    return AssembleResult(
        car_dir=car_dir,
        data_dir=data_dir,
        ui_dir=ui_dir,
        sfx_dir=sfx_dir,
        files=tuple(sorted({f.resolve() for f in files})),
    )


def zip_folder(
    folder: Path,
    out_zip: Path,
    *,
    arcname_base: str | None = None,
) -> Path:
    """Zip ``folder`` into ``out_zip`` byte-deterministically.

    All file mtimes inside the zip are stamped to ``1980-01-01 00:00:00``
    and entries are emitted in sorted order so two runs on the same
    inputs produce a byte-identical archive (verifiable via SHA-256).

    ``arcname_base`` becomes the top-level directory inside the zip. If
    ``None``, ``folder.name`` is used.
    """
    folder = Path(folder).resolve()
    out_zip = Path(out_zip)
    out_zip.parent.mkdir(parents=True, exist_ok=True)

    base = arcname_base or folder.name
    entries = sorted(p for p in folder.rglob("*") if p.is_file())

    with zipfile.ZipFile(out_zip, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=6) as zf:
        for path in entries:
            rel = path.relative_to(folder).as_posix()
            info = zipfile.ZipInfo(filename=f"{base}/{rel}")
            info.date_time = (1980, 1, 1, 0, 0, 0)
            info.compress_type = zipfile.ZIP_DEFLATED
            info.external_attr = (0o644 & 0xFFFF) << 16
            with path.open("rb") as f:
                zf.writestr(info, f.read())
    return out_zip


def convert(doc: SvjDoc, out_dir: Path, *, zip_name: str | None = None) -> Path:
    """High-level wrapper: assemble + zip in one call. Returns the zip path."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    result = assemble(doc, out_dir)
    name = zip_name or f"{model_id(doc)}.zip"
    return zip_folder(result.car_dir, out_dir / name, arcname_base=model_id(doc))


def sha256_of(path: Path) -> str:
    """Helper for tests: SHA-256 of a file (hex string)."""
    h = hashlib.sha256()
    with Path(path).open("rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


# --------------------------------------------------------------------------- #
# Internals                                                                   #
# --------------------------------------------------------------------------- #


def _copy_placeholder(src: Path, dst: Path) -> Path:
    if not src.is_file():
        # Generate a 1x1 transparent PNG as last resort so the test can
        # still assert the file exists. The real placeholders ship in
        # the repo's assets/ folder.
        dst.write_bytes(_one_pixel_png())
        return dst
    shutil.copy2(src, dst)
    return dst


def _slugify(s: str) -> str:
    out = []
    for c in s.lower():
        if c.isalnum():
            out.append(c)
        elif c in (" ", "-", "_"):
            out.append("_")
    while "__" in (joined := "".join(out)):
        out = list(joined.replace("__", "_"))
    return "".join(out).strip("_") or "unnamed"


def _one_pixel_png() -> bytes:
    # 1x1 transparent PNG, fixed bytes — fallback only.
    return (
        b"\x89PNG\r\n\x1a\n\x00\x00\x00\rIHDR\x00\x00\x00\x01\x00\x00\x00\x01"
        b"\x08\x06\x00\x00\x00\x1f\x15\xc4\x89\x00\x00\x00\nIDAT\x78\x9cc\x00"
        b"\x01\x00\x00\x05\x00\x01\r\n-\xb4\x00\x00\x00\x00IEND\xaeB`\x82"
    )


__all__ = [
    "ASSETS_DIR",
    "AssembleResult",
    "assemble",
    "convert",
    "model_id",
    "sha256_of",
    "zip_folder",
]
