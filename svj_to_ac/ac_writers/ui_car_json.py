"""Writer for ``ui/ui_car.json``.

This is the file Content Manager reads to populate the car preview
(name, brand, year, class, performance numbers, torque/power curves).
We emit Kunos's canonical key set; everything that doesn't have a Kunos
home goes under ``specs`` so Content Manager will still surface it.

Sourcing
--------

1. ``x_assettocorsa._ac_native["ui_car"]`` (pass-through; the raw block
   is a dict the caller has already shaped).
2. Canonical SVJ fields: ``vehicle_info`` (make/model/year/variant),
   ``_metadata.description``, ``x_assettocorsa.cm_specs`` (power_bhp,
   torque_nm, top_speed_kmh, accel_0_100_s, power_to_weight, tags),
   ``chassis.mass_total`` (weight). Torque/power curves are derived
   from ``powertrain.engine.torque_curve`` (rad/s -> RPM via
   :func:`units.radps_to_rpm`).

Format
------

UTF-8 without BOM, LF line endings, two-space indentation, trailing
newline. Kunos's own ui_car.json files vary in formatting; we use
``json.dumps(..., indent=2)`` for human-friendly diffs.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Mapping, Sequence

from svj_loader import SvjDoc
from units import radps_to_rpm, rpm_to_radps

FILENAME = "ui_car.json"


# Kunos canonical top-level keys (Content Manager checks these).
CANONICAL_KEYS = (
    "name",
    "brand",
    "year",
    "class",
    "tags",
    "description",
    "specs",
    "torqueCurve",
    "powerCurve",
)


def write(doc: SvjDoc, out_dir: Path) -> Path:
    """Write ``out_dir / 'ui_car.json'``. Returns the file path."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    data = _build(doc)
    text = json.dumps(data, indent=2, ensure_ascii=False) + "\n"
    path = out_dir / FILENAME
    path.write_text(text, encoding="utf-8", newline="\n")
    return path


def _build(doc: SvjDoc) -> dict[str, Any]:
    native = _get_native(doc)
    if native is not None:
        return dict(native)

    vi = doc.vehicle_info
    md = doc.metadata
    ext = doc.assetto
    cm = (ext.cm_specs if ext else None) or {}

    out: dict[str, Any] = {
        "name": _screen_name(doc),
        "brand": vi.make,
        "year": vi.year if vi.year else None,
        "class": str(cm.get("class") or _infer_class(vi.variant)),
        "tags": _tags(cm.get("tags")),
        "description": _description(md),
        "specs": _specs(doc, cm),
        "torqueCurve": _torque_curve(doc),
        "powerCurve": _power_curve(doc),
    }
    # Strip Nones from top level (Kunos tolerates omitted keys, prefers no nulls).
    return {k: v for k, v in out.items() if v is not None}


def _get_native(doc: SvjDoc) -> Mapping[str, Any] | None:
    if doc.assetto is None:
        return None
    native = doc.assetto.raw.get("_ac_native") if doc.assetto.raw else None
    if not isinstance(native, Mapping):
        return None
    block = native.get("ui_car")
    return block if isinstance(block, Mapping) else None


def _screen_name(doc: SvjDoc) -> str:
    ext = doc.assetto
    if ext and ext.info and ext.info.get("screen_name"):
        return str(ext.info["screen_name"])
    return doc.vehicle_info.model or "Unnamed"


def _infer_class(variant: str | None) -> str:
    if not variant:
        return "street"
    return str(variant).lower()


def _tags(raw: Any) -> list[str] | None:
    if not raw:
        return None
    if isinstance(raw, str):
        items = [raw]
    elif isinstance(raw, Sequence):
        items = [str(t) for t in raw]
    else:
        return None
    cleaned = []
    for t in items:
        t = t.strip()
        if not t:
            continue
        if not t.startswith("#"):
            t = "#" + t
        cleaned.append(t)
    return cleaned or None


def _description(md) -> str | None:
    if md is None:
        return None
    return md.description or None


def _specs(doc: SvjDoc, cm: Mapping[str, Any]) -> dict[str, str] | None:
    """Kunos's spec block uses display strings with units, not numbers."""
    block: dict[str, str] = {}

    if "power_bhp" in cm:
        block["bhp"] = f"{int(round(float(cm['power_bhp'])))}bhp"
    if "torque_nm" in cm:
        block["torque"] = f"{int(round(float(cm['torque_nm'])))}Nm"
    if "top_speed_kmh" in cm:
        block["topspeed"] = f"{int(round(float(cm['top_speed_kmh'])))}km/h"
    if "accel_0_100_s" in cm:
        block["acceleration"] = f"{float(cm['accel_0_100_s']):.1f}s 0-100"
    if "power_to_weight" in cm:
        block["pwratio"] = str(cm["power_to_weight"])

    if doc.chassis.mass_total > 0:
        block["weight"] = f"{int(round(doc.chassis.mass_total))}kg"

    return block or None


def _torque_curve(doc: SvjDoc) -> list[list[float]] | None:
    curve = (doc.powertrain.engine or {}).get("torque_curve")
    return _to_rpm_pairs(curve)


def _power_curve(doc: SvjDoc) -> list[list[float]] | None:
    """Derive a power curve from the torque curve: P [kW] = T [Nm] * w [rad/s] / 1000.

    Returns the curve in (RPM, kW) for Content Manager's UI.
    """
    eng = doc.powertrain.engine or {}
    # If SVJ already carries a power curve, prefer it.
    pc = eng.get("power_curve")
    if pc:
        return _to_rpm_pairs(pc)
    torque = eng.get("torque_curve")
    if not torque:
        return None
    from units import w_to_kw  # local import keeps units.py the only conversion path

    out: list[list[float]] = []
    for pair in torque:
        if len(pair) < 2:
            continue
        x = float(pair[0])
        nm = float(pair[1])
        # x may be either rad/s or RPM (heuristic: >200 == RPM)
        if x > 200.0:
            rpm = x
            radps = rpm_to_radps(rpm)
        else:
            radps = x
            rpm = radps_to_rpm(radps)
        kw = w_to_kw(nm * radps)
        out.append([round(rpm, 2), round(kw, 3)])
    return out or None


def _to_rpm_pairs(curve: Any) -> list[list[float]] | None:
    if not curve:
        return None
    out: list[list[float]] = []
    for pair in curve:
        if len(pair) < 2:
            continue
        x = float(pair[0])
        y = float(pair[1])
        if x <= 200.0:
            x = radps_to_rpm(x)
        out.append([round(x, 2), round(y, 3)])
    return out or None


__all__ = ["CANONICAL_KEYS", "FILENAME", "write"]
