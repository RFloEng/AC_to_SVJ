"""Writer for ``data/drivetrain.ini``.

Emits the canonical Kunos sections: ``HEADER``, ``TRACTION``,
``GEARBOX``, ``GEARS``, ``DIFFERENTIAL``, optional ``AUTOCLUTCH``,
``AUTOBLIP``, ``DOWNSHIFT_PROFILER``, ``DAMAGE``.

Sourcing order:

1. ``x_assettocorsa._ac_native["drivetrain"]`` (pass-through).
2. Canonical SVJ ``doc.powertrain``: ``layout`` (drive type),
   ``gearbox`` (ratios, change times, inertia), ``differentials[0]``
   (LSD power/coast/preload). ``x_assettocorsa.driver_aids`` carries
   autoclutch / autoblip / downshift_profiler when present.
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

FILENAME = "drivetrain.ini"

# SVJ powertrain.layout / vehicle_info.drive_type -> AC TRACTION.TYPE.
_LAYOUT_MAP = {
    "FF": "FWD", "FWD": "FWD",
    "FR": "RWD", "RWD": "RWD", "MR": "RWD", "RR": "RWD",
    "AWD": "AWD", "4WD": "AWD", "F4": "AWD", "M4": "AWD",
}


def write(doc: SvjDoc, out_dir: Path) -> Path:
    """Write ``out_dir / 'drivetrain.ini'``. Returns the file path."""
    return write_text(Path(out_dir) / FILENAME, _render(doc))


def _render(doc: SvjDoc) -> str:
    native = get_ac_native_ini(doc, "drivetrain")
    if native is not None:
        return render_sections_from_native(native)
    return _render_from_svj(doc)


def _render_from_svj(doc: SvjDoc) -> str:
    sections: list[str] = [format_section("HEADER", {"VERSION": 3})]
    pt = doc.powertrain

    layout_raw = (pt.layout or doc.vehicle_info.drive_type or "RWD").upper()
    ac_layout = _LAYOUT_MAP.get(layout_raw, "RWD")
    traction: dict[str, Any] = {"TYPE": ac_layout}
    if ac_layout == "AWD":
        rear_bias = pt.raw.get("rear_bias")
        if rear_bias is None:
            rear_bias = 0.6
        traction["REAR_BIAS"] = rear_bias
    elif ac_layout == "RWD":
        traction["REAR_BIAS"] = 1.0
    sections.append(format_section("TRACTION", traction))

    # [GEARBOX]
    gbx = pt.gearbox or {}
    gbx_block = {
        "CHANGE_UP_TIME": _to_ms(gbx.get("change_up_s")),
        "CHANGE_DN_TIME": _to_ms(gbx.get("change_down_s")),
        "AUTO_CUTOFF_TIME": _to_ms(gbx.get("auto_cutoff_s")),
        "SUPPORTS_SHIFTER": gbx.get("supports_shifter"),
        "VALID_SHIFT_RPM_WINDOW": gbx.get("valid_shift_window"),
        "CONTROLS_WINDOW_GAIN": gbx.get("controls_gain"),
        "INERTIA": gbx.get("inertia"),
    }
    sections.append(format_section("GEARBOX", gbx_block))

    # [GEARS]
    ratios = gbx.get("ratios") or []
    gears_block: dict[str, Any] = {"COUNT": len(ratios)}
    for i, r in enumerate(ratios, start=1):
        gears_block[f"GEAR_{i}"] = r
    gears_block["FINAL"] = gbx.get("final_drive", 3.5)
    sections.append(format_section("GEARS", gears_block))

    # [DIFFERENTIAL]
    diff_list = pt.differentials or []
    diff = diff_list[0] if diff_list else {}
    sections.append(
        format_section(
            "DIFFERENTIAL",
            {
                "TYPE": (diff.get("type") or "LSD").upper(),
                "POWER": diff.get("power", 0.5),
                "COAST": diff.get("coast", 0.3),
                "PRELOAD": diff.get("preload", 50.0),
            },
        )
    )

    # Optional driver-aid sections under x_assettocorsa.driver_aids.
    aids = (doc.assetto.driver_aids if doc.assetto else None) or {}

    autoclutch = aids.get("autoclutch")
    if autoclutch:
        sections.append(format_section("AUTOCLUTCH", _aid_block(autoclutch)))

    autoblip = aids.get("autoblip")
    if autoblip:
        sections.append(format_section("AUTOBLIP", _aid_block(autoblip)))

    dsp = aids.get("shift_profiler") or aids.get("downshift_profiler")
    if dsp:
        sections.append(format_section("DOWNSHIFT_PROFILER", _aid_block(dsp)))

    # Damage (from x_assettocorsa.damage.drivetrain)
    damage = (
        (doc.assetto.damage.get("drivetrain") if doc.assetto else None) or {}
    )
    if damage:
        sections.append(
            format_section(
                "DAMAGE",
                {
                    "CLUTCH_TORQUE": damage.get("clutch_torque"),
                    "RPM_THRESHOLD": damage.get("rpm_threshold"),
                    "GEAR_DAMAGE_RATE": damage.get("gear_damage"),
                },
            )
        )

    return join_sections(sections)


def _to_ms(seconds: Any) -> Any:
    """Convert a possibly-second value to milliseconds (AC's unit) but
    pass through values already in ms (heuristic: > 5)."""
    if seconds is None:
        return None
    v = float(seconds)
    if v > 5.0:
        return v
    return int(round(v * 1000))


def _aid_block(d: dict[str, Any]) -> dict[str, Any]:
    """Pass the lower-case-keyed driver-aid block through with upper-cased keys."""
    return {k.upper(): v for k, v in d.items() if v is not None}


__all__ = ["FILENAME", "write"]
