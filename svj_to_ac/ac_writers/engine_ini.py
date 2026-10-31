"""Writer for ``data/engine.ini`` plus companion ``power.lut`` and
``coast_curve.lut``.

Sourcing order:

1. ``x_assettocorsa._ac_native["engine"]`` (pass-through) -- exact INI
   reproduction. The companion LUTs are emitted from
   ``x_assettocorsa.raw["engine_luts"]`` (a dict ``{name: pairs}``) when
   present.
2. Canonical SVJ ``doc.powertrain.engine``: ``idle_rpm``, ``max_rpm``,
   ``inertia``, ``limiter_hz``, ``altitude_sensitivity``,
   ``torque_curve`` (list of ``[rad/s, Nm]`` pairs).

Units: SVJ engine speeds are rad/s; AC uses RPM in INI and LUT. The
conversion goes through :func:`units.radps_to_rpm` -- never inline.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Sequence

import lut_writer
from svj_loader import SvjDoc
from units import radps_to_rpm

from . import (
    format_section,
    get_ac_native_ini,
    join_sections,
    render_sections_from_native,
    write_text,
)

FILENAME = "engine.ini"
POWER_LUT_NAME = "power.lut"
COAST_LUT_NAME = "coast_curve.lut"


def write(doc: SvjDoc, out_dir: Path) -> Path:
    """Write engine.ini and its companion LUTs. Returns the INI path."""
    out_dir = Path(out_dir)
    text, luts = _render_with_luts(doc)
    ini_path = write_text(out_dir / FILENAME, text)
    for name, pairs in luts.items():
        lut_writer.write_lut(out_dir / name, pairs)
    return ini_path


def _render(doc: SvjDoc) -> str:
    text, _ = _render_with_luts(doc)
    return text


def _render_with_luts(
    doc: SvjDoc,
) -> tuple[str, dict[str, list[tuple[float, float]]]]:
    native = get_ac_native_ini(doc, "engine")
    if native is not None:
        return render_sections_from_native(native), _native_luts(doc)
    return _render_from_svj(doc)


def _native_luts(doc: SvjDoc) -> dict[str, list[tuple[float, float]]]:
    if doc.assetto is None:
        return {}
    cached = doc.assetto.raw.get("engine_luts") if doc.assetto.raw else None
    out: dict[str, list[tuple[float, float]]] = {}
    if isinstance(cached, dict):
        for name, pairs in cached.items():
            if isinstance(pairs, list):
                out[name] = [(float(p[0]), float(p[1])) for p in pairs if len(p) >= 2]
    return out


def _render_from_svj(
    doc: SvjDoc,
) -> tuple[str, dict[str, list[tuple[float, float]]]]:
    engine = doc.powertrain.engine or {}
    sections: list[str] = []
    luts: dict[str, list[tuple[float, float]]] = {}

    # [HEADER]
    header = {"VERSION": 2, "POWER_CURVE": POWER_LUT_NAME}
    coast_curve = _torque_curve_in_rpm(engine.get("coast_curve"))
    if coast_curve:
        header["COAST_CURVE"] = COAST_LUT_NAME
        luts[COAST_LUT_NAME] = coast_curve
    sections.append(format_section("HEADER", header))

    # power.lut
    torque_curve_rpm = _torque_curve_in_rpm(engine.get("torque_curve"))
    if torque_curve_rpm:
        luts[POWER_LUT_NAME] = torque_curve_rpm
    else:
        # Emit a minimal flat curve so AC doesn't choke; flag the issue.
        luts[POWER_LUT_NAME] = [
            (float(engine.get("idle_rpm_rpm") or 800), 100.0),
            (float(engine.get("max_rpm_rpm") or 7000), 100.0),
        ]

    # [ENGINE_DATA]
    idle_rpm = _to_rpm(engine.get("idle_rpm"), default=800.0)
    max_rpm = _to_rpm(engine.get("max_rpm"), default=7000.0)
    engine_data = {
        "ALTITUDE_SENSITIVITY": engine.get("altitude_sensitivity"),
        "INERTIA": engine.get("inertia"),
        "MINIMUM": int(round(idle_rpm)),
        "LIMITER": int(round(max_rpm)),
        "LIMITER_HZ": engine.get("limiter_hz"),
    }
    sections.append(format_section("ENGINE_DATA", engine_data))

    # [COAST_REF] -- only if SVJ didn't provide a coast_curve.
    if not coast_curve:
        sections.append(
            format_section(
                "COAST_REF",
                {"RPM": int(round(max_rpm)), "TORQUE": 60, "NON_LINEARITY": 0},
            )
        )

    # [DAMAGE] from x_assettocorsa.damage.engine
    damage = (
        (doc.assetto.damage.get("engine") if doc.assetto else None) or {}
    )
    if damage:
        sections.append(
            format_section(
                "DAMAGE",
                {
                    "TURBO_BOOST_THRESHOLD": damage.get("turbo_boost_threshold"),
                    "TURBO_DAMAGE_K": damage.get("turbo_damage_k"),
                    "RPM_THRESHOLD": damage.get("rpm_threshold"),
                    "RPM_DAMAGE_K": damage.get("rpm_damage_k"),
                },
            )
        )

    # [THROTTLE_RESPONSE]
    tr = engine.get("throttle_response")
    if tr is not None:
        sections.append(format_section("THROTTLE_RESPONSE", {"CURVE": tr}))

    # [FUEL_CONSUMPTION]
    fc = engine.get("fuel_consumption_k")
    if fc is not None:
        sections.append(format_section("FUEL_CONSUMPTION", {"K": fc}))

    return join_sections(sections), luts


def _to_rpm(value: Any, default: float) -> float:
    """Accept either an SVJ rad/s scalar or an already-RPM value.

    The mx5 SVJ fixture stores engine ``idle_rpm`` and ``max_rpm`` as
    RPM (per the AC source) -- a value above ``200`` is taken as already
    being RPM; otherwise we treat it as rad/s and convert.
    """
    if value is None:
        return float(default)
    v = float(value)
    if v > 200.0:
        return v
    return float(radps_to_rpm(v))


def _torque_curve_in_rpm(
    curve: Sequence[Sequence[float]] | None,
) -> list[tuple[float, float]]:
    if not curve:
        return []
    out: list[tuple[float, float]] = []
    for pair in curve:
        if len(pair) < 2:
            continue
        x = float(pair[0])
        y = float(pair[1])
        x_rpm = _to_rpm(x, default=x)
        out.append((round(x_rpm, 3), round(y, 3)))
    return out


__all__ = ["COAST_LUT_NAME", "FILENAME", "POWER_LUT_NAME", "write"]
