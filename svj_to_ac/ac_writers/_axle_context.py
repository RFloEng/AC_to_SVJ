"""Per-axle context for the suspensions writer.

AC pickup coordinates need two pieces of information that aren't in the
pickup record itself:

* ``half_track`` — to convert SVJ centreline-relative Y into AC
  inboard-from-wheel X (see :func:`coords.pickup_lateral_to_ac`).
* ``rolling_radius`` — to fold the wheel radius into AC pickup Z
  (see :func:`coords.pickup_z_to_ac`).

Both must be resolved per axle *before* any pickup is converted (pitfalls
doc §"Coordinate sign mistakes"). The :class:`AxleContext` dataclass is
a tiny carrier so each pickup transform takes a single argument.
"""

from __future__ import annotations

from dataclasses import dataclass

from coords import pickup_lateral_to_ac, pickup_z_to_ac
from svj_loader import SvjChassis, SvjDoc

# AC TYPE integer per SVJ ``system_type`` (spec §6).
SYSTEM_TYPE_TO_AC: dict[str, int] = {
    "macpherson": 0,
    "double_wishbone": 1,
    "multi_link": 2,
    "solid_axle": 3,
    "trailing_arm": 4,
    "semi_trailing_arm": 5,
}

# Inverse map for round-trip checks.
AC_TO_SYSTEM_TYPE = {v: k for k, v in SYSTEM_TYPE_TO_AC.items()}


@dataclass(frozen=True)
class AxleContext:
    """Resolved geometry for one AC axle (FRONT or REAR).

    Attributes
    ----------
    axle
        ``"FRONT"`` or ``"REAR"``.
    half_track
        Half-track of the axle, in metres (positive).
    rolling_radius
        Tyre rolling radius for this axle, in metres (positive).
    system_type
        SVJ ``system_type`` string (``"macpherson"``,
        ``"double_wishbone"``, ...).
    ac_type
        Corresponding AC ``TYPE`` integer (0..5).
    """

    axle: str  # "FRONT" or "REAR"
    half_track: float
    rolling_radius: float
    system_type: str
    # Wheel centre of the left corner in the SVJ vehicle frame (x, y, z).  AC pickups are
    # measured from it on all three axes, so a file whose origin is not on the front axle
    # (other simulators') comes out right.
    wheel_center: tuple[float, float, float] | None = None

    @property
    def ac_type(self) -> int:
        return SYSTEM_TYPE_TO_AC.get(self.system_type, 0)

    # Convenience wrappers so writers stay one-call-from `coords`.
    def lateral_to_ac(self, svj_lateral: float) -> float:
        return pickup_lateral_to_ac(svj_lateral, self.half_track)

    def z_offset(self) -> float:
        return pickup_z_to_ac(self.rolling_radius)


def resolve_axle(
    doc: SvjDoc,
    axle: str,
    *,
    rolling_radius_override: float | None = None,
) -> AxleContext:
    """Build an :class:`AxleContext` from an :class:`SvjDoc`.

    Half-track comes from ``doc.chassis.track_front`` / ``track_rear``
    divided by two. Rolling radius is read from the first tyre set
    matching ``axle`` (sets keyed ``"tire_front_*"`` for the front axle,
    ``"tire_rear_*"`` for the rear); the override is for tests.
    Suspension type is read from the first corner of this axle
    (FL/FR for front, RL/RR for rear).
    """
    if axle not in ("FRONT", "REAR"):
        raise ValueError(f"axle must be FRONT or REAR, got {axle!r}")

    ch: SvjChassis = doc.chassis
    half_track = (ch.track_front if axle == "FRONT" else ch.track_rear) / 2.0

    corner_keys = ("FL", "FR") if axle == "FRONT" else ("RL", "RR")
    wheel_center = None
    for k in corner_keys:
        c = doc.suspension.corners.get(k)
        pos = (c.position if c is not None else None) or []
        if len(pos) == 3 and any(abs(v) > 1e-9 for v in pos):
            wheel_center = (float(pos[0]), float(pos[1]), float(pos[2]))
            half_track = abs(wheel_center[1]) or half_track
            break

    rolling_radius = rolling_radius_override or _resolve_rolling_radius(doc, axle)
    if wheel_center is not None and rolling_radius_override is None:
        rolling_radius = _wheel_radius(doc, corner_keys) or rolling_radius

    system_type = "double_wishbone"
    for k in corner_keys:
        if k in doc.suspension.corners:
            t = doc.suspension.corners[k].topology
            if isinstance(t, dict):
                st = t.get("system_type")
                if isinstance(st, str):
                    system_type = st
                    break

    return AxleContext(
        axle=axle,
        half_track=half_track,
        rolling_radius=float(rolling_radius),
        system_type=system_type,
        wheel_center=wheel_center,
    )


def _wheel_radius(doc: SvjDoc, corner_keys) -> float | None:
    """``wheel.loaded_radius`` of a corner (the radius the wheel centre height is built
    from), else None."""
    for k in corner_keys:
        c = doc.suspension.corners.get(k)
        w = (c.raw.get("wheel") if c is not None and isinstance(c.raw, dict) else None) or {}
        v = w.get("loaded_radius")
        if isinstance(v, (int, float)) and v > 0:
            return float(v)
    return None


def _resolve_rolling_radius(doc: SvjDoc, axle: str) -> float:
    """Pick the rolling radius for an axle from the SVJ ``tires.sets``.

    Falls back to half the overall diameter, then 0.3 m if nothing is
    available (a quiet last resort — caller may pass an override).
    """
    label = "front" if axle == "FRONT" else "rear"
    for key, body in doc.tires.sets.items():
        if label in key.lower():
            dims = (body or {}).get("dimensions", {})
            for field in ("rolling_radius", "loaded_radius", "static_radius"):
                v = dims.get(field) if isinstance(dims, dict) else None
                if v is not None:
                    return float(v)
            od = dims.get("overall_diameter") if isinstance(dims, dict) else None
            if od is not None:
                return float(od) / 2.0
    return 0.3


__all__ = [
    "AC_TO_SYSTEM_TYPE",
    "AxleContext",
    "SYSTEM_TYPE_TO_AC",
    "resolve_axle",
]
