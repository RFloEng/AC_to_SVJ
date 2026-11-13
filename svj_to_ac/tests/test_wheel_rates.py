"""Spring, damper and bar definitions of other simulators brought to AC's wheel-level values."""
from __future__ import annotations

import math

import pytest

import wheel_rates as wr


def test_wheel_referenced_rate_is_used_as_it_is() -> None:
    k, notes = wr.spring_wheel_rate({"id": "s", "rate": 30000.0, "reference": "wheel"}, mr=None)
    assert k == 30000.0 and not notes


def test_element_rate_is_brought_to_the_wheel_by_mr_squared() -> None:
    k, _ = wr.spring_wheel_rate({"id": "s", "rate": 22400.0, "reference": "element"}, mr=0.9594)
    assert k == pytest.approx(22400.0 * 0.9594 ** 2)


def test_element_rate_without_a_ratio_is_flagged() -> None:
    k, notes = wr.spring_wheel_rate({"id": "s", "rate": 22400.0}, mr=None)
    assert k == 22400.0 and any("no motion ratio" in n for n in notes)


def test_force_length_curve_gives_the_local_slope() -> None:
    # a compression spring: force falls as the length grows, 100 kN/m around 0.30 m
    curve = [[0.25, 5000.0], [0.30, 0.0], [0.35, -5000.0], [0.40, -10000.0]]
    k, notes = wr.spring_wheel_rate({"id": "s", "rate_curve": curve}, mr=0.5, length=0.30)
    assert k == pytest.approx(100000.0 * 0.25)
    assert any("non-linear" in n for n in notes)


def test_torsion_bar_motion_ratio_is_the_wheel_speed_per_radian() -> None:
    # bar along Y, wheel centre 0.4 m behind it: the wheel moves 0.4 m per radian
    mr = wr.torsional_motion_ratio([0.0, 0.0, 0.0], [0.0, 1.0, 0.0], [-0.4, 0.6, 0.0], [0.0, 0.0, 1.0])
    assert mr == pytest.approx(0.4)
    # a bar pointing at the wheel centre has no lever
    assert wr.torsional_motion_ratio([0, 0, 0], [0, 1, 0], [0.0, 0.6, 0.0], [0, 0, 1]) is None


def test_helpers_stops_and_packers_are_not_the_main_spring() -> None:
    for ident, typ in (("helper_spring_fl", "coil"), ("bump_stop_fl", "coil"), ("stop_spring_fl", "rubber"),
                       ("x", "rubber")):
        assert wr.is_auxiliary({"id": ident, "spring_type": typ})
    assert not wr.is_auxiliary({"id": "spring_fl", "spring_type": "coilover"})


def test_damper_scalar_coefficient_is_brought_to_the_wheel() -> None:
    out, notes = wr.damper_wheel_curves({"id": "d", "damping": 20000.0}, mr=0.5)
    assert out["bump_curve"][-1] == [1.0, pytest.approx(5000.0)]
    assert out["rebound_curve"] == out["bump_curve"] and notes


def test_damper_curves_follow_v_over_mr_and_f_times_mr() -> None:
    out, _ = wr.damper_wheel_curves({"bump_curve": [[0.0, 0.0], [0.5, 1000.0]]}, mr=0.5)
    assert out["bump_curve"] == [[0.0, 0.0], [1.0, 500.0]]
    out, _ = wr.damper_wheel_curves({"reference": "wheel", "bump_curve": [[0.0, 0.0], [0.5, 1000.0]]}, mr=0.5)
    assert out["bump_curve"] == [[0.0, 0.0], [0.5, 1000.0]]


def test_stop_curve_with_a_slack_range_gives_both_gaps() -> None:
    curve = [[0.09, 125000.0], [0.10, 12500.0], [0.15, 0.0], [0.30, 0.0], [0.31, -1000.0]]
    out = wr.stop_from_curve(curve, 0.2, 0.5)
    assert out["up"] == pytest.approx((0.2 - 0.15) / 0.5)
    assert out["down"] == pytest.approx((0.30 - 0.2) / 0.5)
    assert out["rate"] > 0


@pytest.mark.parametrize("bar,expected", [
    ({"wheel_rate_equiv": 36814.9, "bar_rate": 20000.0}, 36814.9),               # the standard's field wins
    ({"bar_rate": 15500.0, "_note": "Stiffness as written in the [ARB] section"}, 15500.0),   # AC's own value
    ({"bar_rate": 1000.0, "lever_arm": 0.25}, 16000.0),                          # N*m/rad over lever squared
])
def test_arb_wheel_rate(bar, expected) -> None:
    k, _ = wr.arb_wheel_rate(bar)
    assert k == pytest.approx(expected)


def test_arb_of_an_ac_file_is_not_converted() -> None:
    k, _ = wr.arb_wheel_rate({"bar_rate": 15500.0}, ac_origin=True)
    assert k == 15500.0


def test_arb_torsional_rate_without_a_lever_is_flagged() -> None:
    k, notes = wr.arb_wheel_rate({"id": "arb", "bar_rate": 1000.0})
    assert k == pytest.approx(1000.0 / wr.DEFAULT_ARB_LEVER ** 2) and notes


def test_legacy_spring_blocks() -> None:
    assert wr.legacy_spring_wheel_rate({"rate": 45000.0})[0] == 45000.0
    k, _ = wr.legacy_spring_wheel_rate({"type": "torsion_bar", "torsional_rate": 1200.0, "arm_length": 0.4})
    assert k == pytest.approx(1200.0 / 0.16)
    k, _ = wr.legacy_spring_wheel_rate({"rate": 1.0, "rate_curve": [[0.0, 0.0], [0.05, 2500.0], [0.1, 5000.0]]})
    assert k == pytest.approx(50000.0)
    assert wr.legacy_spring_wheel_rate({"type": "air", "effective_area": 0.01})[0] is None


def test_arb_rate_too_stiff_for_a_torsional_bar_is_taken_as_a_wheel_rate() -> None:
    k, notes = wr.arb_wheel_rate({"id": "arb", "bar_rate": 90000.0})
    assert k == 90000.0 and any("too stiff" in n for n in notes)
