"""Writer for the game files an AC -> SVJ conversion carried in ``x_assettocorsa.native_files``:
``lights.ini``, ``cameras.ini``, ``damage.ini``, ``ai.ini`` (see ``ac_native_files.py`` of the converter).

Camera vectors stored in the SVJ's ISO 8855 axes go back to the KN5 space of the new ``car.kn5`` (the origin there
is the centre of gravity: ``kn5 = (y, z - h_cg, x + d_f)`` from ISO ``(x, y, z)`` with the CG at the SVJ frame's
``(cg_x, cg_z)``); vectors kept in ``kn5_model`` space are written as they were.
"""
from __future__ import annotations

from pathlib import Path

from svj_loader import SvjDoc

from . import format_section, join_sections, write_text

FILES = ("lights.ini", "cameras.ini", "damage.ini", "ai.ini")


def _num(v) -> str:
    return f"{float(v):.6g}"


def iso_to_kn5(v, cg, point: bool) -> list:
    """ISO 8855 point / direction of the file -> the KN5 space of the car this package writes (``cg`` is the SAE CG)."""
    x, y, z = (float(c) for c in v)
    if point:
        return [y, z + cg[2], x - cg[0]]
    return [y, z, x]


def native_files(doc: SvjDoc) -> dict:
    raw = doc.assetto.raw if doc.assetto and doc.assetto.raw else {}
    nf = raw.get("native_files")
    return nf if isinstance(nf, dict) else {}


def write(doc: SvjDoc, out_dir: Path) -> list[Path]:
    """The files present in the SVJ, nothing when it carries none."""
    cg = doc.chassis.center_of_gravity
    out: list[Path] = []
    for name, entry in native_files(doc).items():
        if name not in FILES or not isinstance(entry, dict) or not entry.get("sections"):
            continue
        convert = entry.get("frame") == "ISO_8855" and cg and len(cg) == 3
        sections = []
        for sec, kv in entry["sections"].items():
            items = {}
            for k, v in kv.items():
                if convert and name == "cameras.ini" and k in ("POSITION", "FORWARD", "UP") and isinstance(v, list) and len(v) == 3:
                    v = ", ".join(_num(c) for c in iso_to_kn5(v, cg, point=(k == "POSITION")))
                elif isinstance(v, list):
                    v = ", ".join(_num(c) for c in v)
                items[k] = v
            sections.append(format_section(sec, items))
        out.append(write_text(Path(out_dir) / name, join_sections(sections)))
    return out
