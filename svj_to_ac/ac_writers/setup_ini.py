"""Writer for ``data/setup.ini``.

AC's setup.ini defines adjustability *ranges* for the in-game garage,
not the current car values. Each section names an adjustable (e.g.
``PRESSURE_FL``, ``BRAKE_BIAS``, ``ARB_FRONT``) and supplies
``MIN``, ``MAX``, ``STEP``, ``DEFAULT``.

Sourcing order:

1. ``x_assettocorsa._ac_native["setup"]`` (pass-through).
2. Canonical SVJ ``doc.assetto.setup_ranges``: a dict keyed by
   adjustable name with ``{min, max, step, default}`` keys.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping

from svj_loader import SvjDoc

from . import (
    format_section,
    get_ac_native_ini,
    join_sections,
    render_sections_from_native,
    write_text,
)

FILENAME = "setup.ini"


def write(doc: SvjDoc, out_dir: Path) -> Path:
    """Write ``out_dir / 'setup.ini'``. Returns the file path."""
    return write_text(Path(out_dir) / FILENAME, _render(doc))


def _render(doc: SvjDoc) -> str:
    native = get_ac_native_ini(doc, "setup")
    if native is not None:
        return render_sections_from_native(native)
    return _render_from_svj(doc)


def _render_from_svj(doc: SvjDoc) -> str:
    sections: list[str] = [format_section("HEADER", {"VERSION": 1})]
    ranges = (doc.assetto.setup_ranges if doc.assetto else None) or {}

    for name, body in ranges.items():
        if not isinstance(body, Mapping):
            continue
        sections.append(
            format_section(
                name.upper(),
                {
                    "MIN": body.get("min"),
                    "MAX": body.get("max"),
                    "STEP": body.get("step"),
                    "DEFAULT": body.get("default"),
                },
            )
        )

    return join_sections(sections)


__all__ = ["FILENAME", "write"]
