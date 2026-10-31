"""Writer for ``data/tyres.ini``.

Emits one ``[FRONT]`` / ``[REAR]`` pair (one compound) or
``[FRONT_N]`` / ``[REAR_N]`` blocks per compound when multiple are
present. Optional ``[THERMAL_FRONT_N]`` / ``[THERMAL_REAR_N]`` thermal
sections are emitted when the SVJ carries thermal data.

Sourcing
--------

1. ``x_assettocorsa._ac_native["tyres"]`` (pass-through).
2. Canonical SVJ ``doc.tires.sets`` -- one entry per compound. The AC
   parameters per entry come from :func:`pacejka_inverse.fit_ac_params`
   which prefers ``models.pacejka_mf52._source_params`` (lossless) and
   falls back to a scipy least-squares fit (logs WARNING).

We group entries by axle prefix: keys matching ``front`` go into the
front pair, ``rear`` into the rear pair. Compounds are numbered in
insertion order; a single front/rear pair gets the plain
``[FRONT]`` / ``[REAR]`` form to match Kunos's road-car convention.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping

from svj_loader import SvjDoc
from pacejka_inverse import CANONICAL_AC_KEYS, fit_ac_params

from . import (
    format_section,
    get_ac_native_ini,
    join_sections,
    render_sections_from_native,
    write_text,
)

FILENAME = "tyres.ini"


def write(doc: SvjDoc, out_dir: Path, tyre_choice: Mapping[str, Any] | None = None) -> Path:
    """Write ``out_dir / 'tyres.ini'``. Returns the file path.

    ``tyre_choice`` takes the tyres from cars of an installed AC instead of fitting them:
    ``{"cars_dir": ".../content/cars", "front": {"car": ..., "section": ...}, "rear": {...},
    "adapt_size": True}`` (see :func:`tyre_match.rank`).  An axle left out is fitted as usual.
    """
    out_dir = Path(out_dir)
    log_path = out_dir.parent / "conversion_log.txt"
    if tyre_choice:
        text = _render_from_choice(doc, tyre_choice, log_path=log_path)
        if text is not None:
            return write_text(out_dir / FILENAME, text)
    return write_text(out_dir / FILENAME, _render(doc, log_path=log_path))


def _first_set(doc: SvjDoc, axle: str) -> Mapping[str, Any] | None:
    for key, body in (doc.tires.sets or {}).items():
        if axle in key.lower():
            return body
    return None


def _render_from_choice(doc: SvjDoc, choice: Mapping[str, Any], *, log_path: Path | None) -> str | None:
    """tyres.ini with the sections of the chosen cars' tyres (their own `[THERMAL_*]` too).

    The grip, stiffness and thermal data are the chosen tyre's; with ``adapt_size`` (the
    default) the width, radius and rim radius are the SVJ tyre's, so the wheel keeps the size
    of the car being converted.
    """
    import re

    from tyre_match import tyres_ini_sections                       # AC_to_SVJ, one folder up

    cars_dir = choice.get("cars_dir")
    if not cars_dir:
        return None
    out: list[str] = ["[HEADER]", "VERSION=10", "", "[COMPOUND_DEFAULT]", "INDEX=0", ""]
    used = False
    for axle in ("front", "rear"):
        pick = choice.get(axle)
        if not pick:
            fitted = _first_set(doc, axle)
            if fitted is not None:                                 # fit this axle as usual
                out.append(_compound_section(axle.upper(), fitted, log_path=log_path, side=axle))
                out.append("")
            continue
        text = tyres_ini_sections(Path(cars_dir), pick["car"], pick["section"])
        src_suffix = pick["section"].split("_", 1)[1] if "_" in pick["section"] else ""
        text = re.sub(r"^\[(THERMAL_)?(FRONT|REAR)(_\d+)?\]", lambda m: f"[{m.group(1) or ''}{axle.upper()}]",
                      text, flags=re.M)
        tset = _first_set(doc, axle)
        if tset is not None and choice.get("adapt_size", True):
            dims, rim = tset.get("dimensions") or {}, tset.get("rim") or {}
            repl = {"WIDTH": dims.get("section_width"),
                    "RADIUS": (dims.get("overall_diameter") or 0) / 2 or None,
                    "RIM_RADIUS": (rim.get("diameter") or 0) / 2 or None}
            for k, v in repl.items():
                if v:
                    text = re.sub(rf"^{k}=[^;\r\n]*", f"{k}={v:.4f}", text, count=1, flags=re.M)
        out.append(f"; {axle}: tyre of {pick['car']} [{pick['section']}] chosen with tyre_match")
        out.append(text.rstrip())
        out.append("")
        used = True
    return "\n".join(out).rstrip() + "\n" if used else None


def _render(doc: SvjDoc, *, log_path: Path | None = None) -> str:
    native = get_ac_native_ini(doc, "tyres")
    if native is not None:
        return render_sections_from_native(native)
    return _render_from_svj(doc, log_path=log_path)


def _render_from_svj(doc: SvjDoc, *, log_path: Path | None) -> str:
    sets = doc.tires.sets or {}
    front_keys = [k for k in sets if "front" in k.lower()]
    rear_keys = [k for k in sets if "rear" in k.lower()]

    sections: list[str] = []
    n_front = len(front_keys)
    n_rear = len(rear_keys)
    multi = max(n_front, n_rear) > 1

    for idx, key in enumerate(front_keys):
        suffix = f"_{idx}" if multi else ""
        sec = _compound_section(
            f"FRONT{suffix}", sets[key], log_path=log_path, side="front"
        )
        sections.append(sec)
    for idx, key in enumerate(rear_keys):
        suffix = f"_{idx}" if multi else ""
        sec = _compound_section(
            f"REAR{suffix}", sets[key], log_path=log_path, side="rear"
        )
        sections.append(sec)

    # Thermal sections, if SVJ carries them.
    for idx, key in enumerate(front_keys):
        suffix = f"_{idx}" if multi else ""
        sec = _thermal_section(f"THERMAL_FRONT{suffix}", sets[key])
        if sec:
            sections.append(sec)
    for idx, key in enumerate(rear_keys):
        suffix = f"_{idx}" if multi else ""
        sec = _thermal_section(f"THERMAL_REAR{suffix}", sets[key])
        if sec:
            sections.append(sec)

    return join_sections(sections)


def _compound_section(
    name: str,
    compound: Mapping[str, Any],
    *,
    log_path: Path | None,
    side: str,
) -> str:
    """Render one ``[FRONT_N]``/``[REAR_N]`` section."""
    params = fit_ac_params(compound, log_path=log_path, side=side)
    # Preserve AC's canonical key order for clean diffs.
    block: dict[str, Any] = {}
    for key in CANONICAL_AC_KEYS:
        if key in params and params[key] is not None:
            block[key] = params[key]
    # Pass through anything else (CAMBER_GAIN, PRESSURE_REF, etc.).
    for key, val in params.items():
        if key not in block and val is not None:
            block[key] = val
    return format_section(name, block)


def _thermal_section(name: str, compound: Mapping[str, Any]) -> str | None:
    thermal = compound.get("thermal")
    if not isinstance(thermal, Mapping):
        return None
    block = {
        "PERFORMANCE_CURVE": thermal.get("performance_curve"),
        "CORE_TRANSFER": thermal.get("core_transfer"),
        "SURFACE_TRANSFER": thermal.get("surface_transfer"),
        "PATCH_TRANSFER": thermal.get("patch_transfer"),
        "FRICTION_K": thermal.get("friction_k"),
        "ROLLING_K": thermal.get("rolling_k"),
        "IDEAL_TEMP": thermal.get("ideal_temp_c"),
    }
    if not any(v is not None for v in block.values()):
        return None
    return format_section(name, block)


__all__ = ["FILENAME", "write"]
