"""Writer for ``data/car.ini``.

Emits the standard Kunos sections: ``INFO``, ``BASIC``, ``CONTROLS``,
``WEIGHT``, ``FUEL``, ``FUELTANK``, ``RULES``, ``PIT_STOP``. Field set
mirrors the keys the AC_to_SVJ parser reads (``parse_car_extras`` and
the ``BASIC`` block consumed elsewhere by ``converter.py``).

Sourcing order, per field:

1. ``x_assettocorsa._ac_native["car"]`` (pass-through) -- exact
   reproduction of the source AC INI. Used by round-trip tests.
2. Canonical SVJ fields (chassis mass, wheelbase, vehicle_info, etc.)
   combined with whatever ``x_assettocorsa`` carries.

Coordinate notes: ``GRAPHICS_OFFSET`` and ``FUELTANK.POSITION`` are AC
vectors. The SVJ extension stores ``fuel_tank_position`` in AC axes
already (the forward converter preserves it verbatim), so we pass it
through without invoking :mod:`coords`.
"""

from __future__ import annotations

from pathlib import Path

from svj_loader import SvjDoc

from . import (
    format_section,
    format_value,
    format_vec3,
    get_ac_native_ini,
    join_sections,
    render_sections_from_native,
    write_text,
)

FILENAME = "car.ini"


def write(doc: SvjDoc, out_dir: Path) -> Path:
    """Write ``out_dir / 'car.ini'``. Returns the file path."""
    return write_text(Path(out_dir) / FILENAME, _render(doc))


def _render(doc: SvjDoc) -> str:
    native = get_ac_native_ini(doc, "car")
    if native is not None:
        return render_sections_from_native(native)
    return _render_from_svj(doc)


def _render_from_svj(doc: SvjDoc) -> str:
    sections: list[str] = []

    # [INFO]
    md = doc.metadata
    info = {
        "AUTHOR": (md.author if md else None) or "SVJ_to_AC",
        "VERSION": "1.0",
    }
    sections.append(format_section("INFO", info))

    # [BASIC]
    vi = doc.vehicle_info
    ch = doc.chassis
    ext = doc.assetto
    cm = ext.cm_specs if ext else {}
    info_block = ext.info if ext else {}
    screen_name = info_block.get("screen_name") if info_block else None

    cg_height = -float(ch.center_of_gravity[2]) if ch.center_of_gravity else 0.0

    basic = {
        "GRAPHICS_OFFSET": "0, 0, 0",
        "GRAPHICS_PITCH_ROTATION": 0,
        "NAME": vi.model,
        "SCREEN_NAME": screen_name or vi.model,
        "BRAND": vi.make,
        "TAGS": _format_tags(cm.get("tags") if cm else None),
        "CLASS": cm.get("class") if cm else None,
        "TOTALMASS": ch.mass_total,
        "WHEELBASE": ch.wheelbase,
        "CG_HEIGHT": round(cg_height, 4),
    }
    sections.append(format_section("BASIC", basic))

    # [CONTROLS]
    ctrls = (ext.controls if ext else {}) or {}
    if ctrls:
        controls = {
            "FFMULT": ctrls.get("ffb_mult"),
            "STEER_LOCK": ctrls.get("steer_lock"),
            "STEER_RATIO": ctrls.get("steer_ratio"),
            "STEER_ASSIST": ctrls.get("steer_assist"),
        }
        if any(v is not None for v in controls.values()):
            sections.append(format_section("CONTROLS", controls))

    # [WEIGHT]
    rules = (ext.raw.get("rules") if ext else None) or {}
    minimum = rules.get("min_weight") if rules else None
    sections.append(format_section("WEIGHT", {"MINIMUM": int(round(minimum or ch.mass_total))}))

    # [FUEL]
    fuel = {
        "FUEL": int(ch.fuel_initial) if ch.fuel_initial is not None else None,
        "MAX_FUEL": int(ch.fuel_capacity) if ch.fuel_capacity is not None else None,
        "CONSUMPTION": 0.1,
    }
    if any(v is not None for v in fuel.values()):
        sections.append(format_section("FUEL", fuel))

    # [FUELTANK]
    ft = ext.fuel_tank_position if ext else None
    if ft:
        sections.append(format_section("FUELTANK", {"POSITION": format_vec3(ft)}))

    # [RULES]
    if rules:
        sections.append(format_section("RULES", {k.upper(): v for k, v in rules.items()}))

    # [PIT_STOP]
    pit = (ext.pit_stop if ext else None) or {}
    if pit:
        sections.append(
            format_section(
                "PIT_STOP",
                {
                    "TYRE_CHANGE_TIME_SEC": pit.get("tyre_change_time"),
                    "FUEL_LITER_SEC": pit.get("fuel_rate"),
                    "ENGINE_REPAIR_RATE": pit.get("engine_fix_rate"),
                    "BODY_REPAIR_RATE": pit.get("body_fix_rate"),
                    "SUSPENSION_REPAIR_TIME": pit.get("suspension_fix"),
                },
            )
        )

    return join_sections(sections)


def _format_tags(tags):
    if not tags:
        return None
    if isinstance(tags, str):
        return tags
    return ",".join(str(t).lstrip("#") for t in tags)


__all__ = ["FILENAME", "write"]
