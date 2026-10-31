"""Writer for ``data/brakes.ini``.

Emits ``[FRONT]``, ``[REAR]``, ``[BRAKES]``, optional ``[HANDBRAKE]``.
Field set matches ``ac_parsers.parse_brakes``.

Sourcing order, per field:

1. ``x_assettocorsa._ac_native["brakes"]`` (pass-through).
2. ``x_assettocorsa.raw["brakes"]`` scalar overrides per axle.
3. SVJ canonical per-corner ``brake.max_torque_nm`` /
   ``brake.disc.outer_diameter`` (averaged across the two corners).
4. Sensible road-car defaults.
"""

from __future__ import annotations

from pathlib import Path

from svj_loader import SvjDoc

from . import (
    format_section,
    get_ac_native_ini,
    join_sections,
    render_sections_from_native,
    write_text,
)

FILENAME = "brakes.ini"

_DEFAULT_FRONT = {"DISC_INERTIA": 0.2, "DISC_RADIUS": 0.140, "MAX_TORQUE": 3000}
_DEFAULT_REAR = {"DISC_INERTIA": 0.15, "DISC_RADIUS": 0.130, "MAX_TORQUE": 2200}


def write(doc: SvjDoc, out_dir: Path) -> Path:
    """Write ``out_dir / 'brakes.ini'``. Returns the file path."""
    return write_text(Path(out_dir) / FILENAME, _render(doc))


def _render(doc: SvjDoc) -> str:
    native = get_ac_native_ini(doc, "brakes")
    if native is not None:
        return render_sections_from_native(native)
    return _render_from_svj(doc)


def _render_from_svj(doc: SvjDoc) -> str:
    sections: list[str] = []
    brakes = doc.brakes

    ext_raw = doc.assetto.raw.get("brakes", {}) if doc.assetto else {}
    front = {
        **_DEFAULT_FRONT,
        **_per_axle_from_corners(doc, ("FL", "FR")),
        **{k.upper(): v for k, v in (ext_raw.get("front", {}) or {}).items()},
    }
    rear = {
        **_DEFAULT_REAR,
        **_per_axle_from_corners(doc, ("RL", "RR")),
        **{k.upper(): v for k, v in (ext_raw.get("rear", {}) or {}).items()},
    }
    sections.append(format_section("FRONT", front))
    sections.append(format_section("REAR", rear))

    is_adjustable = (brakes.bias_type or "").lower() == "adjustable"
    brakes_block = {
        "BIAS": brakes.bias,
        "COCKPIT_ADJUSTABLE": is_adjustable,
        "BIAS_STEP": ext_raw.get("bias_step"),
        "BIAS_MIN": ext_raw.get("bias_min"),
        "BIAS_MAX": ext_raw.get("bias_max"),
    }
    sections.append(format_section("BRAKES", brakes_block))

    hb = ext_raw.get("handbrake")
    if hb:
        sections.append(
            format_section(
                "HANDBRAKE",
                {
                    "MAX_TORQUE": hb.get("max_torque"),
                    "RATIO": hb.get("ratio"),
                    "CURVE": hb.get("curve"),
                },
            )
        )

    return join_sections(sections)


def _per_axle_from_corners(doc: SvjDoc, corner_keys: tuple[str, ...]) -> dict[str, float]:
    """Average per-corner brake fields onto an AC axle block.

    Reads ``corner.raw["brake"]["max_torque_nm"]`` and
    ``corner.raw["brake"]["disc"]["outer_diameter"]`` -- both populated
    in canonical SVJ v0.95 even for cars that didn't originate from AC.
    """
    torques: list[float] = []
    diams: list[float] = []
    for ck in corner_keys:
        corner = doc.suspension.corners.get(ck)
        if corner is None:
            continue
        brake = corner.raw.get("brake") if isinstance(corner.raw, dict) else None
        if not isinstance(brake, dict):
            continue
        if brake.get("max_torque_nm") is not None:
            torques.append(float(brake["max_torque_nm"]))
        disc = brake.get("disc")
        if isinstance(disc, dict) and disc.get("outer_diameter") is not None:
            diams.append(float(disc["outer_diameter"]) / 2.0)
    out: dict[str, float] = {}
    if torques:
        out["MAX_TORQUE"] = sum(torques) / len(torques)
    if diams:
        out["DISC_RADIUS"] = sum(diams) / len(diams)
    return out


__all__ = ["FILENAME", "write"]
