"""Writers for the ctrl_*.ini family plus their companion ``.lut``s.

AC's electronics live in per-controller files in ``data/``: ``ctrl_abs.ini``,
``ctrl_tc.ini``, ``ctrl_turbo0.ini``, etc. Each file has a ``[HEADER]``
plus one or more ``[CONTROLLER_N]`` sections; the curve can either be
inline (``LUT=(|x=y|x=y|)``) or a reference to a separate ``.lut`` file
(``LUT=ctrl_turbo0.lut``). When the LUT is external, the ``.lut`` MUST
exist on disk -- otherwise AC silently ignores the controller (pitfalls
doc §"ctrl LUT references"). This module always emits the pair
atomically.

Sourcing
--------

Each SVJ controller comes from ``x_assettocorsa.controls[<name>]``,
which carries a dict shaped like::

    {
        "version": 1,
        "controllers": [
            {
                "id": "ctrl_0",
                "input": "RPM",
                "combinator": "ADD",
                "filter": 0.95,
                "up_limit": 1,
                "down_limit": 0,
                "lut": [[500, 0.0], [4000, 1.0], [7000, 0.5]],
                # OR
                "lut_raw": "(|500=0.00|4000=1.00|7000=0.50|)",
                # OR
                "lut_ref": "ctrl_turbo0.lut",
            },
            ...
        ],
    }

When ``lut`` (parsed pairs) is present and the curve has more than
``INLINE_LUT_THRESHOLD`` points we emit it as an external ``.lut`` file
named ``ctrl_<name>.lut``. Below the threshold we use the inline
``(|x=y|x=y|)`` syntax to match Kunos's stock files.

``x_assettocorsa._ac_native["ctrl_<name>"]`` (per file) bypasses the
canonical path with byte-faithful pass-through.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

import lut_writer
from svj_loader import SvjDoc

from . import (
    format_section,
    format_value,
    get_ac_native_ini,
    join_sections,
    render_sections_from_native,
    write_text,
)

# Above this many points we externalise the curve to a .lut file.
INLINE_LUT_THRESHOLD = 8


def write(doc: SvjDoc, out_dir: Path) -> list[Path]:
    """Write every ``ctrl_*.ini`` and its companion ``.lut``.

    Returns the list of files emitted (INIs + LUTs). The order is
    deterministic for reproducible zip packaging downstream.
    """
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    written: list[Path] = []

    controls = _controls_source(doc)
    for name, block in sorted(controls.items()):
        ini_path, lut_paths = _write_one(doc, name, block, out_dir)
        written.append(ini_path)
        written.extend(lut_paths)
    return written


def _controls_source(doc: SvjDoc) -> dict[str, Any]:
    """Resolve the source dict of ``{name: controller_block}`` pairs.

    The SVJ extension stores controls keyed by canonical name (``abs``,
    ``tc``, ``turbo0``, ...) under ``x_assettocorsa.controls``. We
    surface them with their ``ctrl_<name>`` filename prefix added.
    """
    if doc.assetto is None:
        return {}
    ctrls = doc.assetto.controls or {}
    if not isinstance(ctrls, Mapping):
        return {}
    out: dict[str, Any] = {}
    for name, body in ctrls.items():
        if not isinstance(body, Mapping):
            continue
        out[str(name).lower()] = dict(body)
    return out


def _write_one(
    doc: SvjDoc, name: str, block: Mapping[str, Any], out_dir: Path
) -> tuple[Path, list[Path]]:
    """Emit a single ctrl_<name>.ini and any external LUTs it references."""
    ini_filename = f"ctrl_{name}.ini"
    # Check for a per-file native pass-through.
    native = get_ac_native_ini(doc, f"ctrl_{name}")
    if native is not None:
        return write_text(out_dir / ini_filename, render_sections_from_native(native)), []

    text, lut_files = _render_one(name, block)
    ini_path = write_text(out_dir / ini_filename, text)
    lut_paths: list[Path] = []
    for lut_name, pairs in lut_files.items():
        lut_paths.append(lut_writer.write_lut(out_dir / lut_name, pairs))
    return ini_path, lut_paths


def _render_one(
    name: str, block: Mapping[str, Any]
) -> tuple[str, dict[str, list[tuple[float, float]]]]:
    sections: list[str] = [
        format_section("HEADER", {"VERSION": block.get("version", 1)})
    ]
    lut_files: dict[str, list[tuple[float, float]]] = {}

    controllers = block.get("controllers") or []
    if not isinstance(controllers, Iterable):
        controllers = []

    for idx, c in enumerate(controllers):
        if not isinstance(c, Mapping):
            continue
        sec_name = f"CONTROLLER_{idx}"

        lut_value, ext_lut = _serialize_lut(c, base_name=f"ctrl_{name}_{idx}")
        if ext_lut is not None:
            lut_files[ext_lut[0]] = ext_lut[1]

        sec_block = {
            "INPUT": c.get("input"),
            "COMBINATOR": c.get("combinator"),
            "LUT": lut_value,
            "FILTER": c.get("filter"),
            "UP_LIMIT": c.get("up_limit"),
            "DOWN_LIMIT": c.get("down_limit"),
        }
        sections.append(format_section(sec_name, sec_block))

    return join_sections(sections), lut_files


def _serialize_lut(
    c: Mapping[str, Any], *, base_name: str
) -> tuple[str | None, tuple[str, list[tuple[float, float]]] | None]:
    """Decide whether to emit the LUT inline or external.

    Returns ``(value_for_LUT_key_or_None, optional_external_lut_pair)``.
    The external lut is ``(filename, pairs)`` if one should be written.
    """
    # 1. Explicit external reference wins -- caller is asking for a separate file.
    ref = c.get("lut_ref")
    if isinstance(ref, str) and ref.strip():
        pairs = _normalize_pairs(c.get("lut"))
        # External reference: we still emit the LUT file if pairs were
        # provided (otherwise we assume the caller copied it manually).
        if pairs:
            return ref.strip(), (ref.strip(), pairs)
        return ref.strip(), None

    # 2. Raw inline string (already in AC's `(|x=y|...)` syntax).
    raw = c.get("lut_raw")
    if isinstance(raw, str) and raw.strip().startswith("("):
        return raw.strip(), None

    # 3. Parsed pairs -> inline or external based on length.
    pairs = _normalize_pairs(c.get("lut"))
    if not pairs:
        return None, None
    if len(pairs) <= INLINE_LUT_THRESHOLD:
        return _format_inline_lut(pairs), None
    filename = f"{base_name}.lut"
    return filename, (filename, pairs)


def _normalize_pairs(pairs: Any) -> list[tuple[float, float]]:
    if not isinstance(pairs, Iterable) or isinstance(pairs, (str, bytes)):
        return []
    out: list[tuple[float, float]] = []
    for p in pairs:
        if isinstance(p, Sequence) and len(p) >= 2:
            try:
                out.append((float(p[0]), float(p[1])))
            except (TypeError, ValueError):
                pass
    return out


def _format_inline_lut(pairs: list[tuple[float, float]]) -> str:
    """Render the AC inline LUT format: ``(|x=y|x=y|...|)``."""
    body = "|".join(f"{format_value(x)}={format_value(y)}" for x, y in pairs)
    return f"(|{body}|)"


__all__ = ["INLINE_LUT_THRESHOLD", "write"]
