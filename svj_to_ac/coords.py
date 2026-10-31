"""SVJ (SAE J670, Z-down) <-> AC (Y-up) coordinate transform.

This is the **single source of truth** for the axis flip between SVJ
(``X-forward, Y-right, Z-down``) and Assetto Corsa
(``X-right, Y-up, Z-forward``). Every writer that touches a 3D point
must call into this module; inline ``y/z`` swaps are a bug.

Two flavours of transform are exposed:

* :func:`svj_to_ac_point` / :func:`svj_to_ac_vector` -- free 3D transforms
  (no translation component). Used for chassis dimensions, COG, mesh
  positions, anything the AC parser treats as plain coordinates.

* :func:`pickup_lateral_to_ac` / :func:`pickup_z_to_ac` -- pickup-point
  conventions. AC stores suspension hardpoints with X measured
  *inboard-positive from the wheel centre* (not the car centreline) and
  Z offset by ``-rolling_radius``. See ``ai_handoff/05_PITFALLS.md`` and
  the AC_to_SVJ README ``Suspension`` section for context.

Forward (AC -> SVJ, per AC_to_SVJ): ``svj = (ac.z, ac.x, -ac.y)``.
Inverse (this project, SVJ -> AC): ``ac = (svj.y, -svj.z, svj.x)``.
"""

from __future__ import annotations

from typing import Iterable, Sequence

Point3 = tuple[float, float, float]


# --------------------------------------------------------------------------- #
# Free 3D transforms                                                          #
# --------------------------------------------------------------------------- #


def svj_to_ac_point(p: Sequence[float]) -> Point3:
    """Map an SVJ point ``(x, y, z)`` to AC ``(x, y, z)``.

    SVJ axes:  X-forward, Y-right, Z-down.
    AC axes:   X-right, Y-up, Z-forward.

    Transform: ``ac = (svj.y, -svj.z, svj.x)``.
    """
    sx, sy, sz = _unpack3(p)
    return (sy, -sz, sx)


def svj_to_ac_vector(v: Sequence[float]) -> Point3:
    """Same as :func:`svj_to_ac_point`. Provided as a name hint for
    callers that pass a free vector (no translation) rather than a
    point; the transform is identical because the SVJ-AC coordinate
    spaces share an origin convention (both anchored at car-relative 0)."""
    return svj_to_ac_point(v)


def ac_to_svj_point(p: Sequence[float]) -> Point3:
    """Inverse of :func:`svj_to_ac_point`.

    Used in round-trip tests and by any tooling that needs to round-trip
    a point back to SVJ for diffing.

    Transform: ``svj = (ac.z, ac.x, -ac.y)``.
    """
    ax, ay, az = _unpack3(p)
    return (az, ax, -ay)


def ac_to_svj_vector(v: Sequence[float]) -> Point3:
    """See :func:`svj_to_ac_vector` -- same transform as the point form."""
    return ac_to_svj_point(v)


# --------------------------------------------------------------------------- #
# Pickup-point conventions                                                    #
# --------------------------------------------------------------------------- #


def pickup_lateral_to_ac(svj_lateral: float, half_track: float) -> float:
    """Convert an SVJ lateral (Y) coordinate to AC's inboard-positive X.

    AC's suspension-pickup X is **inboard-positive from the wheel centre**,
    *not* the car centreline. SVJ keeps the centreline-relative value. So
    for any pickup whose SVJ-Y is ``svj_lateral`` (signed; negative on the
    driver side in J670), the AC equivalent on a wheel at ``half_track``
    from the centreline is::

        ac_x = half_track - abs(svj_lateral)

    Worked example from ``05_PITFALLS.md``: SVJ Y = 0.75, half-track 0.78
    -> AC X = 0.03 (3 cm inboard from the wheel centre).

    Parameters
    ----------
    svj_lateral
        SVJ Y coordinate of the pickup, signed.
    half_track
        Half-track of the relevant axle, in metres (positive).
    """
    return half_track - abs(svj_lateral)


def pickup_z_to_ac(rolling_radius: float) -> float:
    """Vertical offset baked into AC pickup Z.

    AC stores ``chassis_z_offset = -rolling_radius`` against every pickup
    so that a pickup at SVJ-Z = 0 sits at AC-Z = ``-rolling_radius``.
    Callers pass the *positive* tyre rolling radius and receive the
    correctly-signed AC offset.
    """
    return -rolling_radius


# --------------------------------------------------------------------------- #
# Bulk helpers                                                                #
# --------------------------------------------------------------------------- #


def svj_to_ac_cloud(points: Iterable[Sequence[float]]) -> list[Point3]:
    """Apply :func:`svj_to_ac_point` to an iterable of points."""
    return [svj_to_ac_point(p) for p in points]


def ac_to_svj_cloud(points: Iterable[Sequence[float]]) -> list[Point3]:
    """Apply :func:`ac_to_svj_point` to an iterable of points."""
    return [ac_to_svj_point(p) for p in points]


# --------------------------------------------------------------------------- #
# Helpers                                                                     #
# --------------------------------------------------------------------------- #


def _unpack3(p: Sequence[float]) -> tuple[float, float, float]:
    if len(p) != 3:
        raise ValueError(f"expected a 3-component point, got length {len(p)}")
    return float(p[0]), float(p[1]), float(p[2])


__all__ = [
    "Point3",
    "ac_to_svj_cloud",
    "ac_to_svj_point",
    "ac_to_svj_vector",
    "pickup_lateral_to_ac",
    "pickup_z_to_ac",
    "svj_to_ac_cloud",
    "svj_to_ac_point",
    "svj_to_ac_vector",
]
