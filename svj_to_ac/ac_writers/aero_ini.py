"""Writer for ``data/aero.ini``.

Emits ``[BODY]``, per-wing ``[WING_N]`` blocks, and optional ``[DRS]``.
Field set matches ``ac_parsers.parse_aero``.

Sourcing order, per field:

1. ``x_assettocorsa._ac_native["aero"]`` (pass-through).
2. Canonical SVJ ``doc.aerodynamics``: ``reference.frontal_area``,
   ``global_coefficients.Cd`` / ``Cl_front`` / ``Cl_rear``,
   ``components[]`` -> one ``WING_N`` per entry, optional
   ``x_assettocorsa.raw["aero"].drs``.

Coordinate notes: AC wing ``POSITION`` is AC-axis. If the SVJ source
provides ``position_ac`` we use it verbatim; otherwise ``position`` is
assumed to be in SVJ axes (X-forward, Y-right, Z-down) and is
transformed via :func:`coords.svj_to_ac_point`.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping

from coords import svj_to_ac_point
from svj_loader import SvjDoc

from . import (
    format_section,
    format_vec3,
    get_ac_native_ini,
    join_sections,
    render_sections_from_native,
    write_text,
)

FILENAME = "aero.ini"


def write(doc: SvjDoc, out_dir: Path) -> Path:
    return write_text(Path(out_dir) / FILENAME, _render(doc))


def _render(doc: SvjDoc) -> str:
    native = get_ac_native_ini(doc, "aero")
    if native is not None:
        return render_sections_from_native(native)
    return _render_from_svj(doc)


def _render_from_svj(doc: SvjDoc) -> str:
    aero = doc.aerodynamics
    if aero is None:
        return join_sections([format_section("BODY", {"FRONTAL_AREA": 1.8, "CD": 0.3})])

    ref = aero.reference or {}
    glob = aero.global_coefficients or aero.coefficients or {}
    raw_ext = doc.assetto.raw.get("aero", {}) if doc.assetto else {}

    body = {
        "FRONTAL_AREA": ref.get("frontal_area"),
        "CD": glob.get("Cd"),
        "FRONT": glob.get("Cl_front"),
        "REAR": glob.get("Cl_rear"),
        "WIND_MULT": raw_ext.get("wind_mult"),
        "DAMAGE_K": raw_ext.get("damage_k"),
    }
    sections: list[str] = [format_section("BODY", body)]

    for idx, comp in enumerate(_wing_components(aero.components or [])):
        sections.append(format_section(f"WING_{idx}", _wing_block(comp)))

    drs = raw_ext.get("drs")
    if drs:
        sections.append(format_section("DRS", {
            "ENABLED": drs.get("enabled"),
            "WING_INDEX": drs.get("wing_index"),
            "ACTIVE_SPEED_THRESHOLD": drs.get("zone_speed_threshold"),
        }))

    return join_sections(sections)


def _wing_components(components: list[Mapping[str, Any]]) -> list[Mapping[str, Any]]:
    """Filter out the body component; only proper wings go to WING_N."""
    out: list[Mapping[str, Any]] = []
    for c in components:
        if str(c.get("name", "")).lower() == "body":
            continue
        out.append(c)
    return out


def _wing_block(comp: Mapping[str, Any]) -> dict[str, Any]:
    coeffs = comp.get("coefficients") or {}
    return {
        "NAME": comp.get("name"),
        "POSITION": format_vec3(_wing_position_ac(comp)),
        "CHORD": comp.get("chord"),
        "SPAN": comp.get("span"),
        "ANGLE": comp.get("angle"),
        "CL_GAIN": coeffs.get("Cl") if isinstance(coeffs, Mapping) else None,
        "CD_GAIN": coeffs.get("Cd") if isinstance(coeffs, Mapping) else None,
        "LUT_AOA_CL": comp.get("aoa_cl_curve_ref"),
        "LUT_AOA_CD": comp.get("aoa_cd_curve_ref"),
    }


def _wing_position_ac(comp: Mapping[str, Any]) -> list[float] | None:
    """Resolve a wing position in AC coordinates.

    ``position_ac`` is already AC-axis (used by SVJs converted from AC).
    ``position`` is SVJ-axis (used by SVJs from non-AC sources) and gets
    transformed via :func:`coords.svj_to_ac_point`.
    """
    if comp.get("position_ac") is not None:
        return list(comp["position_ac"])
    pos = comp.get("position")
    if pos is None or len(pos) != 3:
        return None
    return list(svj_to_ac_point(pos))


__all__ = ["FILENAME", "write"]
