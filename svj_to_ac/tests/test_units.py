"""Tests for ``units``: every conversion plus its inverse, degenerate values."""

from __future__ import annotations

import math

import pytest

from units import (
    c_to_k,
    deg_to_rad,
    inertia_to_ac,
    k_to_c,
    kg_to_kg,
    kw_to_w,
    m_to_m,
    n_to_n,
    nm_to_nm,
    pa_to_psi,
    psi_to_pa,
    rad_to_deg,
    rad_to_rad,
    radps_to_rpm,
    rpm_to_radps,
    w_to_kw,
)


# --------------------------------------------------------------------------- #
# Identity helpers                                                            #
# --------------------------------------------------------------------------- #


@pytest.mark.parametrize(
    "fn,value",
    [
        (m_to_m, 1.5),
        (kg_to_kg, 1058.0),
        (inertia_to_ac, 0.13),
        (n_to_n, 9810.0),
        (nm_to_nm, 200.0),
        (rad_to_rad, 0.42),
    ],
)
def test_identity_helpers_preserve_value(fn, value) -> None:
    assert fn(value) == value
    assert fn(0.0) == 0.0
    assert fn(-value) == -value


# --------------------------------------------------------------------------- #
# Angle: rad <-> deg                                                          #
# --------------------------------------------------------------------------- #


def test_rad_to_deg_known_values() -> None:
    assert rad_to_deg(0.0) == 0.0
    assert rad_to_deg(math.pi) == pytest.approx(180.0)
    assert rad_to_deg(math.pi / 2) == pytest.approx(90.0)
    # Camber 1 degree -> rad 0.01745329...
    one_deg_in_rad = 1.0 * math.pi / 180.0
    assert rad_to_deg(one_deg_in_rad) == pytest.approx(1.0)


def test_rad_deg_roundtrip() -> None:
    for v in (-7.0, -1.0, 0.0, 0.42, math.pi, 6.28):
        assert deg_to_rad(rad_to_deg(v)) == pytest.approx(v)


# --------------------------------------------------------------------------- #
# Angular velocity: rad/s <-> RPM                                             #
# --------------------------------------------------------------------------- #


def test_radps_to_rpm_known_values() -> None:
    assert radps_to_rpm(0.0) == 0.0
    # 2*pi rad/s = 60 rpm
    assert radps_to_rpm(2 * math.pi) == pytest.approx(60.0)
    # Idle ~780 rpm
    assert rpm_to_radps(780.0) == pytest.approx(81.6814, rel=1e-4)


def test_radps_rpm_roundtrip() -> None:
    for v in (0.0, 100.0, 6800.0, -2000.0):
        assert radps_to_rpm(rpm_to_radps(v)) == pytest.approx(v)


# --------------------------------------------------------------------------- #
# Pressure: Pa <-> psi                                                        #
# --------------------------------------------------------------------------- #


def test_pa_to_psi_known_values() -> None:
    # 1 atm ~ 101325 Pa ~ 14.6959 psi
    assert pa_to_psi(101325.0) == pytest.approx(14.6959, rel=1e-4)
    # Tyre at 30 psi ~ 206843 Pa
    assert psi_to_pa(30.0) == pytest.approx(206842.7, rel=1e-4)


def test_pa_psi_roundtrip() -> None:
    for v in (0.0, 100000.0, 250000.0):
        assert psi_to_pa(pa_to_psi(v)) == pytest.approx(v)


def test_pa_to_psi_negative_passthrough() -> None:
    """Negative pressures aren't physical but the maths must still be linear."""
    assert pa_to_psi(-100000.0) == pytest.approx(-14.5037, rel=1e-4)


# --------------------------------------------------------------------------- #
# Power: W <-> kW                                                             #
# --------------------------------------------------------------------------- #


def test_w_to_kw_known_values() -> None:
    assert w_to_kw(0.0) == 0.0
    assert w_to_kw(1000.0) == pytest.approx(1.0)
    assert w_to_kw(119_300.0) == pytest.approx(119.3)
    assert kw_to_w(160.0) == pytest.approx(160_000.0)


def test_w_kw_roundtrip() -> None:
    for v in (0.0, 1000.0, 200_000.0, -50_000.0):
        assert kw_to_w(w_to_kw(v)) == pytest.approx(v)


# --------------------------------------------------------------------------- #
# Temperature: K <-> C                                                        #
# --------------------------------------------------------------------------- #


def test_k_to_c_known_values() -> None:
    assert k_to_c(273.15) == pytest.approx(0.0)
    assert k_to_c(0.0) == pytest.approx(-273.15)
    assert c_to_k(25.0) == pytest.approx(298.15)


def test_k_c_roundtrip() -> None:
    for v in (0.0, 273.15, 1000.0, -500.0):
        assert c_to_k(k_to_c(v)) == pytest.approx(v)


# --------------------------------------------------------------------------- #
# Very-large / very-small values                                              #
# --------------------------------------------------------------------------- #


def test_radps_to_rpm_huge() -> None:
    assert radps_to_rpm(1e9) > 0
    # Round-trip preserved within float epsilon
    assert rpm_to_radps(radps_to_rpm(1e9)) == pytest.approx(1e9, rel=1e-12)


def test_pa_to_psi_tiny() -> None:
    assert pa_to_psi(1e-12) > 0
    assert psi_to_pa(pa_to_psi(1e-12)) == pytest.approx(1e-12, rel=1e-9)
