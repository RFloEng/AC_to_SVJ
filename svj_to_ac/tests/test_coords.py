"""Tests for ``coords``: free transforms and pickup-point conventions."""

from __future__ import annotations

import math
import random

import pytest

from coords import (
    ac_to_svj_cloud,
    ac_to_svj_point,
    ac_to_svj_vector,
    pickup_lateral_to_ac,
    pickup_z_to_ac,
    svj_to_ac_cloud,
    svj_to_ac_point,
    svj_to_ac_vector,
)


# --------------------------------------------------------------------------- #
# svj_to_ac_point / svj_to_ac_vector                                          #
# --------------------------------------------------------------------------- #


def test_svj_to_ac_point_basic_axes() -> None:
    """Hand-verified: AC X = SVJ Y, AC Y = -SVJ Z, AC Z = SVJ X."""
    # SVJ unit basis -> expected AC images
    assert svj_to_ac_point((1.0, 0.0, 0.0)) == (0.0, 0.0, 1.0)  # forward
    assert svj_to_ac_point((0.0, 1.0, 0.0)) == (1.0, 0.0, 0.0)  # right
    assert svj_to_ac_point((0.0, 0.0, 1.0)) == (0.0, -1.0, 0.0)  # down -> -up


def test_svj_to_ac_vector_matches_point() -> None:
    p = (0.123, -4.5, 6.78)
    assert svj_to_ac_vector(p) == svj_to_ac_point(p)


def test_ac_to_svj_point_basic_axes() -> None:
    """Inverse: SVJ X = AC Z, SVJ Y = AC X, SVJ Z = -AC Y."""
    assert ac_to_svj_point((1.0, 0.0, 0.0)) == (0.0, 1.0, 0.0)
    assert ac_to_svj_point((0.0, 1.0, 0.0)) == (0.0, 0.0, -1.0)
    assert ac_to_svj_point((0.0, 0.0, 1.0)) == (1.0, 0.0, 0.0)


def test_ac_to_svj_vector_matches_point() -> None:
    p = (1.0, 2.0, 3.0)
    assert ac_to_svj_vector(p) == ac_to_svj_point(p)


# --------------------------------------------------------------------------- #
# 100-point round-trip (acceptance test from build plan)                      #
# --------------------------------------------------------------------------- #


def test_random_cloud_roundtrip_1e_minus_9() -> None:
    """Acceptance test: 100-point random cloud, svj -> ac -> svj, max error < 1e-9.

    From ``03_BUILD_PLAN.md`` Stage 3 acceptance.
    """
    rng = random.Random(20260517)  # deterministic seed
    pts = [
        (rng.uniform(-100, 100), rng.uniform(-100, 100), rng.uniform(-100, 100))
        for _ in range(100)
    ]
    out = ac_to_svj_cloud(svj_to_ac_cloud(pts))
    max_err = 0.0
    for original, recovered in zip(pts, out):
        for a, b in zip(original, recovered):
            max_err = max(max_err, abs(a - b))
    assert max_err < 1e-9, f"round-trip max error {max_err} >= 1e-9"


def test_reverse_roundtrip_ac_first() -> None:
    """Same test starting from AC space."""
    rng = random.Random(20260518)
    pts = [
        (rng.uniform(-50, 50), rng.uniform(-50, 50), rng.uniform(-50, 50))
        for _ in range(100)
    ]
    out = svj_to_ac_cloud(ac_to_svj_cloud(pts))
    max_err = max(abs(a - b) for orig, rec in zip(pts, out) for a, b in zip(orig, rec))
    assert max_err < 1e-9


# --------------------------------------------------------------------------- #
# Degenerate inputs                                                           #
# --------------------------------------------------------------------------- #


def test_origin_maps_to_origin() -> None:
    assert svj_to_ac_point((0.0, 0.0, 0.0)) == (0.0, 0.0, 0.0)
    assert ac_to_svj_point((0.0, 0.0, 0.0)) == (0.0, 0.0, 0.0)


def test_very_large_values_preserve_precision() -> None:
    p = (1e9, -1e9, 1e9)
    out = ac_to_svj_point(svj_to_ac_point(p))
    for a, b in zip(p, out):
        assert math.isclose(a, b, rel_tol=0, abs_tol=1e-3)


def test_wrong_length_raises() -> None:
    with pytest.raises(ValueError):
        svj_to_ac_point((1.0, 2.0))  # type: ignore[arg-type]
    with pytest.raises(ValueError):
        ac_to_svj_point((1.0, 2.0, 3.0, 4.0))  # type: ignore[arg-type]


# --------------------------------------------------------------------------- #
# Pickup-point conventions                                                    #
# --------------------------------------------------------------------------- #


def test_pickup_pitfalls_worked_example() -> None:
    """From ``05_PITFALLS.md`` Coordinate sign mistakes:

    Known FL upper wishbone outer point at SVJ (1.4, 0.75, 0.35),
    half-track 0.78 -> AC must equal (0.03, -0.35, 1.4).
    """
    svj_x, svj_y, svj_z = 1.4, 0.75, 0.35
    half_track = 0.78

    ac_x = pickup_lateral_to_ac(svj_y, half_track)
    # Y and Z use the standard svj_to_ac transform (just for those components).
    _, ac_y_free, ac_z_free = svj_to_ac_point((svj_x, svj_y, svj_z))
    # ac_y_free = -svj_z = -0.35; ac_z_free = svj_x = 1.4.
    assert ac_x == pytest.approx(0.03)
    assert ac_y_free == pytest.approx(-0.35)
    assert ac_z_free == pytest.approx(1.4)


def test_pickup_lateral_is_signed_abs() -> None:
    """Left-side pickups (negative SVJ Y) yield the same inboard distance
    as their right-side mirrors."""
    assert pickup_lateral_to_ac(0.5, 0.78) == pytest.approx(0.28)
    assert pickup_lateral_to_ac(-0.5, 0.78) == pytest.approx(0.28)


def test_pickup_lateral_zero_at_wheel_centre() -> None:
    """A pickup whose SVJ-Y is exactly half-track lands at AC X = 0."""
    assert pickup_lateral_to_ac(0.78, 0.78) == 0.0


def test_pickup_z_negates_radius() -> None:
    assert pickup_z_to_ac(0.32) == pytest.approx(-0.32)
    assert pickup_z_to_ac(0.0) == 0.0


# --------------------------------------------------------------------------- #
# Cloud helpers                                                               #
# --------------------------------------------------------------------------- #


def test_cloud_helpers_match_pointwise() -> None:
    pts = [(1.0, 2.0, 3.0), (-1.5, 0.0, 2.7), (1e-3, 1e-3, 1e-3)]
    assert svj_to_ac_cloud(pts) == [svj_to_ac_point(p) for p in pts]
    assert ac_to_svj_cloud(pts) == [ac_to_svj_point(p) for p in pts]
