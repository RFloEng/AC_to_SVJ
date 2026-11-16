"""Writer for ``data/analog_instruments.ini``: the dashboard needles.

A needle is a body with a ``x_instrument`` block (written by the AC -> SVJ converter): the
signal it shows, the AC section and node name, and the mapping ZERO / STEP / MIN_VALUE or
the value -> degrees table.  Needles of an SVJ that has no such block are not written: AC
drives them from this file only, and no other simulator's gauge definition maps onto it
without guessing the dial.
"""

from __future__ import annotations

import re
from pathlib import Path

from lut_writer import write_lut
from svj_loader import SvjDoc

from . import format_section, join_sections, write_text

FILENAME = "analog_instruments.ini"


def instruments(doc: SvjDoc) -> list[dict]:
    """The ``x_instrument`` blocks of the SVJ's bodies, in order, each with its body ``id``."""
    out = []
    for b in (doc.raw.get("chassis") or {}).get("mass_bodies") or []:
        xi = b.get("x_instrument")
        if isinstance(xi, dict) and xi.get("ac_object"):
            out.append({**xi, "body": b.get("id")})
    return out


def ac_object_names(doc: SvjDoc) -> dict[str, str]:
    """``SVJ::body::<id>`` -> the AC node name a needle must have in the KN5."""
    return {f"SVJ::body::{i['body']}": i["ac_object"] for i in instruments(doc)}


def write(doc: SvjDoc, out_dir: Path) -> list[Path]:
    """Write ``analog_instruments.ini`` (and its LUTs); nothing when the SVJ has no needle."""
    items = instruments(doc)
    if not items:
        return []
    out_dir = Path(out_dir)
    files: list[Path] = []
    sections = []
    used: set = set()
    for i in items:
        sec = re.sub(r"[^A-Z0-9_]", "_", str(i.get("ac_section") or i["signal"]).upper())
        n, base = 2, sec
        while sec in used:
            sec, n = f"{base}_{n}", n + 1
        used.add(sec)
        kv = {"ZERO": i.get("zero_deg", 0.0), "STEP": i.get("step_deg_per_unit", 1.0),
              "OBJECT_NAME": i["ac_object"], "OBJECT_NAME_MAX": i.get("ac_object_max"),
              "MIN_VALUE": i.get("min_value", 0.0)}
        lut = i.get("lut_value_to_deg")
        if lut:
            fname = re.sub(r"[^A-Za-z0-9_.-]", "_", str(i.get("lut_file") or f"analog_{i['signal']}_curve.lut"))
            files.append(write_lut(out_dir / fname, lut))
            kv["LUT"] = fname
        sections.append(format_section(sec, kv))
    files.insert(0, write_text(out_dir / FILENAME, join_sections(sections)))
    return files
