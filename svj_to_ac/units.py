"""SI (SVJ) <-> AC unit conversion.

This is the **single source of truth** for every unit conversion in the
project. Writers must import from here; inline ``* 57.29``, ``* 60 /
(2 * pi)``, ``/ 6894.76`` etc. anywhere else in the repo is a bug
(``tests/test_no_inline_units.py`` enforces this with a grep test).

Conversion table (matches ``ai_handoff/02_PROJECT_SPEC.md`` section 7):

============================  =========  ===========  ================================
Quantity                      SVJ unit   AC unit      Function
============================  =========  ===========  ================================
Length                        m          m            :func:`m_to_m`        (identity)
Mass                          kg         kg           :func:`kg_to_kg`      (identity)
Inertia                       kg.m^2     kg.m^2       :func:`inertia_to_ac` (identity)
Force                         N          N            :func:`n_to_n`        (identity)
Torque                        N.m        N.m          :func:`nm_to_nm`      (identity)
Angle, camber/toe             rad        deg          :func:`rad_to_deg`
Angle, steering rack          rad        rad          :func:`rad_to_rad`    (identity)
Angular velocity (engine)     rad/s      RPM          :func:`radps_to_rpm`
Power                         W          kW           :func:`w_to_kw`
Pressure (tyres)              Pa         psi          :func:`pa_to_psi`
Temperature                   K          degC         :func:`k_to_c`
============================  =========  ===========  ================================

Every conversion ships with its inverse (``deg_to_rad``, ``rpm_to_radps``,
etc.) so that round-trip tests can verify symmetry and so that reverse
plumbing (AC -> SVJ check passes) shares the same helpers.

All functions are pure and accept either a scalar or a numpy array; the
math operators used (``*``, ``-``, ``/``) broadcast transparently.
"""

from __future__ import annotations

import math
from typing import TypeVar

# Generic so the static-type signature also matches numpy ndarrays.
Numeric = TypeVar("Numeric", float, int)

# --------------------------------------------------------------------------- #
# Conversion factors (kept here, used only here)                              #
# --------------------------------------------------------------------------- #

_RAD_TO_DEG = 180.0 / math.pi
_DEG_TO_RAD = math.pi / 180.0

_RADPS_TO_RPM = 60.0 / (2.0 * math.pi)
_RPM_TO_RADPS = (2.0 * math.pi) / 60.0

# Pascal to psi: 1 psi = 6894.757293168 Pa exactly (NIST).
_PA_TO_PSI = 1.0 / 6894.757293168
_PSI_TO_PA = 6894.757293168

# Watt to kilowatt: 1 kW = 1000 W.
_W_TO_KW = 1.0 / 1000.0
_KW_TO_W = 1000.0

# Kelvin to Celsius: T_C = T_K - 273.15.
_K_TO_C_OFFSET = 273.15


# --------------------------------------------------------------------------- #
# Identity passes (kept as named helpers so writers stay one-call-from-units) #
# --------------------------------------------------------------------------- #


def m_to_m(v):
    """Metres -> metres (identity). Wrapper exists so writers never bypass units.py."""
    return v


def kg_to_kg(v):
    """Kilograms -> kilograms (identity)."""
    return v


def inertia_to_ac(v):
    """kg.m^2 -> kg.m^2 (identity)."""
    return v


def n_to_n(v):
    """Newtons -> Newtons (identity)."""
    return v


def nm_to_nm(v):
    """Newton-metres -> Newton-metres (identity)."""
    return v


def rad_to_rad(v):
    """Radians -> radians (identity). Used for the steering rack angle."""
    return v


# --------------------------------------------------------------------------- #
# Real conversions                                                            #
# --------------------------------------------------------------------------- #


def rad_to_deg(v):
    """Radians -> degrees. Used for camber and toe in AC suspensions.ini."""
    return v * _RAD_TO_DEG


def deg_to_rad(v):
    """Degrees -> radians."""
    return v * _DEG_TO_RAD


def radps_to_rpm(v):
    """rad/s -> RPM. Used for engine speed in AC engine.ini and power.lut."""
    return v * _RADPS_TO_RPM


def rpm_to_radps(v):
    """RPM -> rad/s."""
    return v * _RPM_TO_RADPS


def pa_to_psi(v):
    """Pascal -> psi. Used for tyre pressures in AC setup.ini."""
    return v * _PA_TO_PSI


def psi_to_pa(v):
    """psi -> Pascal."""
    return v * _PSI_TO_PA


def w_to_kw(v):
    """Watt -> kilowatt. Used for the ``power`` field in ui/ui_car.json."""
    return v * _W_TO_KW


def kw_to_w(v):
    """Kilowatt -> Watt."""
    return v * _KW_TO_W


def k_to_c(v):
    """Kelvin -> Celsius. Used for tyre and brake temperatures."""
    return v - _K_TO_C_OFFSET


def c_to_k(v):
    """Celsius -> Kelvin."""
    return v + _K_TO_C_OFFSET


__all__ = [
    "c_to_k",
    "deg_to_rad",
    "inertia_to_ac",
    "k_to_c",
    "kg_to_kg",
    "kw_to_w",
    "m_to_m",
    "n_to_n",
    "nm_to_nm",
    "pa_to_psi",
    "psi_to_pa",
    "rad_to_deg",
    "rad_to_rad",
    "radps_to_rpm",
    "rpm_to_radps",
    "w_to_kw",
]
