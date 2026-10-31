"""Writer for ``data/electronics.ini``.

The test_car AC fixture has no ``electronics.ini`` -- on stock AC the
electronics live inside the per-controller ``ctrl_*.ini`` files. We
still emit a minimal ``electronics.ini`` when the SVJ carries a
``x_assettocorsa.driver_aids`` block describing TC/ESP/ABS presets,
mostly for Content Manager's UI.

Sourcing order:

1. ``x_assettocorsa._ac_native["electronics"]`` (pass-through).
2. Canonical SVJ ``doc.assetto.driver_aids``: ``abs``, ``tc``, ``esp``
   sub-blocks become ``[ABS]``, ``[TRACTION_CONTROL]``,
   ``[STABILITY_CONTROL]`` sections.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

from svj_loader import SvjDoc

from . import (
    format_section,
    get_ac_native_ini,
    join_sections,
    render_sections_from_native,
    write_text,
)

FILENAME = "electronics.ini"


def write(doc: SvjDoc, out_dir: Path) -> Path:
    """Write ``out_dir / 'electronics.ini'``. Returns the file path."""
    return write_text(Path(out_dir) / FILENAME, _render(doc))


def _render(doc: SvjDoc) -> str:
    native = get_ac_native_ini(doc, "electronics")
    if native is not None:
        return render_sections_from_native(native)
    return _render_from_svj(doc)


def _render_from_svj(doc: SvjDoc) -> str:
    sections: list[str] = [format_section("HEADER", {"VERSION": 1})]
    aids = (doc.assetto.driver_aids if doc.assetto else None) or {}

    for svj_key, ac_section in (
        ("abs", "ABS"),
        ("tc", "TRACTION_CONTROL"),
        ("esp", "STABILITY_CONTROL"),
    ):
        block = aids.get(svj_key)
        if not block:
            continue
        sections.append(
            format_section(
                ac_section,
                {
                    "PRESENT": block.get("present", True),
                    "ACTIVE": block.get("active"),
                    "PRESETS": block.get("presets"),
                    "SLIP_THRESHOLD": block.get("slip_threshold"),
                    "DEFAULT_LEVEL": block.get("default_level"),
                },
            )
        )

    return join_sections(sections)


__all__ = ["FILENAME", "write"]
