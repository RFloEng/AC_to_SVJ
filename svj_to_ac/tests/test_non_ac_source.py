"""End-to-end test for SVJ files that did NOT come from AC_to_SVJ.

A non-AC SVJ has no ``x_assettocorsa`` extension and therefore no
``_ac_native`` pass-through blocks. The converter must still produce a
drivable AC car by reading the canonical SVJ per-corner blocks
(``spring``, ``damper``, ``arb``, ``alignment``, ``brake``, ``wheel``,
``static_setup``) and the canonical aero ``components[].position``
(SVJ axes).

This test builds a fully-synthetic SvjDoc with no ``assetto`` block
and verifies every AC scalar the canonical path is supposed to recover.
"""

from __future__ import annotations

from copy import deepcopy
from pathlib import Path

import math

import pytest

from coords import svj_to_ac_point
from svj_loader import (
    SvjAerodynamics,
    SvjBrakes,
    SvjChassis,
    SvjCorner,
    SvjDoc,
    SvjMetadata,
    SvjPowertrain,
    SvjSuspension,
    SvjTires,
    SvjVehicleInfo,
)

from ac_parsers import parse_ini  # noqa: E402
from ac_writers import aero_ini, brakes_ini, suspensions_ini


def _corner(name: str, pos, *, spring_rate: float, damper_bump: float,
            damper_rebound: float, arb_rate: float, camber: float,
            toe: float, brake_torque: float, disc_diam: float,
            wheel_mass: float, rim_diam: float) -> SvjCorner:
    """Synthesise a full SVJ corner with every canonical block populated."""
    return SvjCorner(
        name=name,
        position=pos,
        topology={
            "system_type": "double_wishbone",
            "upright": {"id": name, "hardpoints": {
                "wheel_center": pos,
                "upper_ball_joint": [pos[0] - 0.03, pos[1], pos[2] - 0.12],
                "lower_ball_joint": [pos[0], pos[1], pos[2] + 0.10],
                "steering_tie_rod_end": [pos[0] + 0.12, pos[1], pos[2] + 0.05],
            }},
            "links": [
                {"name": "upper_wishbone",
                 "inboard_points": [[pos[0] + 0.13, pos[1] + 0.38, pos[2] - 0.12],
                                    [pos[0] - 0.17, pos[1] + 0.38, pos[2] - 0.12]],
                 "outboard_point": [pos[0] - 0.03, pos[1], pos[2] - 0.12]},
                {"name": "lower_wishbone",
                 "inboard_points": [[pos[0] + 0.45, pos[1] + 0.62, pos[2] + 0.17],
                                    [pos[0] - 0.40, pos[1] + 0.62, pos[2] + 0.17]],
                 "outboard_point": [pos[0], pos[1], pos[2] + 0.10]},
                {"name": "steering_tie_rod",
                 "inboard_points": [[pos[0] + 0.12, pos[1] + 0.50, pos[2] + 0.05]],
                 "outboard_point": [pos[0] + 0.12, pos[1], pos[2] + 0.05]},
            ],
        },
        raw={
            "position": pos,
            "topology": {},  # already exposed via topology= above
            "wheel": {"rim_diameter": rim_diam, "rim_width": 0.205,
                      "mass": wheel_mass, "set_ref": "tire_front_1"},
            "spring": {"type": "coilover", "rate": spring_rate,
                       "motion_ratio": 1.0, "rod_length": 0.18},
            "damper": {
                "type": "monotube", "motion_ratio": 1.0,
                "bump_curve": [[0.0, 0.0], [0.1, damper_bump * 0.1],
                               [0.5, damper_bump * 0.5]],
                "rebound_curve": [[0.0, 0.0], [0.1, damper_rebound * 0.1],
                                  [0.5, damper_rebound * 0.5]],
                "bumpstops": {"up": 0.045, "down": 0.060},
            },
            "arb": {"bar_id": f"arb_{name}", "bar_rate": arb_rate},
            "alignment": {"camber": camber, "toe": toe},
            "brake": {"max_torque_nm": brake_torque,
                      "disc": {"type": "vented", "outer_diameter": disc_diam}},
            "tire": {"set_ref": "tire_front_1"},
            "static_setup": {"ride_height": -0.12},
        },
    )


def _non_ac_doc() -> SvjDoc:
    """A complete SVJ that was *not* converted from AC -- no ``assetto`` block."""
    wb = 2.55
    htf = 0.75
    htr = 0.76
    return SvjDoc(
        vehicle_info=SvjVehicleInfo("Acme", "ProtoCar", 2026, "concept", "RWD"),
        chassis=SvjChassis(
            mass_total=1180.0,
            wheelbase=wb,
            track_front=2 * htf,
            track_rear=2 * htr,
            center_of_gravity=[0.0, 0.0, -0.45],
            mass_unsprung_per_corner={"FL": 14.0, "FR": 14.0, "RL": 13.0, "RR": 13.0},
        ),
        suspension=SvjSuspension(corners={
            "FL": _corner("FL", [0.0, -htf, -0.32],
                          spring_rate=65000.0, damper_bump=3300.0,
                          damper_rebound=4100.0, arb_rate=18000.0,
                          camber=-0.018, toe=-0.001,
                          brake_torque=3600.0, disc_diam=0.56,
                          wheel_mass=14.0, rim_diam=0.4572),
            "FR": _corner("FR", [0.0, htf, -0.32],
                          spring_rate=65000.0, damper_bump=3300.0,
                          damper_rebound=4100.0, arb_rate=18000.0,
                          camber=-0.018, toe=-0.001,
                          brake_torque=3600.0, disc_diam=0.56,
                          wheel_mass=14.0, rim_diam=0.4572),
            "RL": _corner("RL", [-wb, -htr, -0.32],
                          spring_rate=48000.0, damper_bump=2500.0,
                          damper_rebound=3300.0, arb_rate=11000.0,
                          camber=-0.020, toe=0.0008,
                          brake_torque=2400.0, disc_diam=0.50,
                          wheel_mass=13.0, rim_diam=0.4572),
            "RR": _corner("RR", [-wb, htr, -0.32],
                          spring_rate=48000.0, damper_bump=2500.0,
                          damper_rebound=3300.0, arb_rate=11000.0,
                          camber=-0.020, toe=0.0008,
                          brake_torque=2400.0, disc_diam=0.50,
                          wheel_mass=13.0, rim_diam=0.4572),
        }),
        tires=SvjTires(sets={
            "tire_front_1": {"dimensions": {"section_width": 0.225,
                                              "overall_diameter": 0.64}},
            "tire_rear_1": {"dimensions": {"section_width": 0.245,
                                            "overall_diameter": 0.64}},
        }),
        powertrain=SvjPowertrain(layout="FR"),
        brakes=SvjBrakes(0.58, "fixed"),
        metadata=SvjMetadata(version="0.95"),
        aerodynamics=SvjAerodynamics(
            reference={"frontal_area": 1.85},
            global_coefficients={"Cd": 0.31, "Cl_front": -0.02, "Cl_rear": -0.04},
            components=[
                # body skipped by writer; included for realism
                {"name": "body", "type": "fixed",
                 "coefficients": {"Cd": -2.5, "Cl": -1.8}},
                # SVJ-axis position: x_forward=2.0, y_right=0.0, z_down=-0.23
                {"name": "front_splitter", "type": "wing",
                 "position": [2.0, 0.0, -0.23],
                 "chord": 0.2, "span": 1.4, "angle": 0.0,
                 "coefficients": {"Cl": -0.03, "Cd": 0.01}},
                {"name": "rear_lip", "type": "wing",
                 "position": [-2.0, 0.0, -0.33],
                 "chord": 0.15, "span": 1.3, "angle": 4.0,
                 "coefficients": {"Cl": -0.04, "Cd": 0.01}},
            ],
        ),
        # Critically: NO assetto block.
    )


# --------------------------------------------------------------------------- #
# Suspensions canonical recovery                                              #
# --------------------------------------------------------------------------- #


class TestSuspensionsCanonical:
    @pytest.fixture
    def parsed(self, tmp_path: Path):
        doc = _non_ac_doc()
        out = suspensions_ini.write(doc, tmp_path)
        return parse_ini(out.read_text(encoding="utf-8"))

    def test_front_spring_rate(self, parsed) -> None:
        assert float(parsed["FRONT"]["SPRING_RATE"]) == pytest.approx(65000.0)

    def test_rear_spring_rate(self, parsed) -> None:
        assert float(parsed["REAR"]["SPRING_RATE"]) == pytest.approx(48000.0)

    def test_front_damping(self, parsed) -> None:
        assert float(parsed["FRONT"]["DAMP_BUMP"]) == pytest.approx(3300.0, rel=1e-3)
        assert float(parsed["FRONT"]["DAMP_REBOUND"]) == pytest.approx(4100.0, rel=1e-3)

    def test_rear_damping(self, parsed) -> None:
        assert float(parsed["REAR"]["DAMP_BUMP"]) == pytest.approx(2500.0, rel=1e-3)
        assert float(parsed["REAR"]["DAMP_REBOUND"]) == pytest.approx(3300.0, rel=1e-3)

    def test_arb(self, parsed) -> None:
        assert float(parsed["ARB"]["FRONT"]) == pytest.approx(18000.0)
        assert float(parsed["ARB"]["REAR"]) == pytest.approx(11000.0)

    def test_camber_toe(self, parsed) -> None:
        # STATIC_CAMBER is in degrees
        assert float(parsed["FRONT"]["STATIC_CAMBER"]) == pytest.approx(math.degrees(-0.018), abs=1e-3)
        assert float(parsed["FRONT"]["TOE_OUT"]) == pytest.approx(-0.001, abs=1e-6)
        assert float(parsed["REAR"]["STATIC_CAMBER"]) == pytest.approx(math.degrees(-0.020), abs=1e-3)
        assert float(parsed["REAR"]["TOE_OUT"]) == pytest.approx(0.0008, abs=1e-6)

    def test_bumpstops(self, parsed) -> None:
        assert float(parsed["FRONT"]["BUMPSTOP_UP"]) == pytest.approx(0.045)
        assert float(parsed["FRONT"]["BUMPSTOP_DN"]) == pytest.approx(0.060)

    def test_wheel_mass_and_rim(self, parsed) -> None:
        # HUB_MASS is per axle: both wheels
        assert float(parsed["FRONT"]["HUB_MASS"]) == pytest.approx(28.0)
        assert float(parsed["REAR"]["HUB_MASS"]) == pytest.approx(26.0)

    def test_rod_length(self, parsed) -> None:
        assert float(parsed["FRONT"]["ROD_LENGTH"]) == pytest.approx(0.18)

    def test_basey_from_the_cg_height(self, parsed) -> None:
        # AC: wheel radius + BASEY = CG height; radius 0.32 (overall 0.64), CG at 0.45 m
        assert float(parsed["FRONT"]["BASEY"]) == pytest.approx(0.32 - 0.45)

    def test_type_is_dwb(self, parsed) -> None:
        # Topology system_type == double_wishbone -> AC TYPE = 1
        assert int(float(parsed["FRONT"]["TYPE"])) == 1


# --------------------------------------------------------------------------- #
# Brakes canonical recovery                                                   #
# --------------------------------------------------------------------------- #


class TestBrakesCanonical:
    @pytest.fixture
    def parsed(self, tmp_path: Path):
        doc = _non_ac_doc()
        out = brakes_ini.write(doc, tmp_path)
        return parse_ini(out.read_text(encoding="utf-8"))

    def test_front_max_torque(self, parsed) -> None:
        assert float(parsed["FRONT"]["MAX_TORQUE"]) == pytest.approx(3600.0)

    def test_rear_max_torque(self, parsed) -> None:
        assert float(parsed["REAR"]["MAX_TORQUE"]) == pytest.approx(2400.0)

    def test_disc_radius(self, parsed) -> None:
        # outer_diameter 0.56 m -> radius 0.28
        assert float(parsed["FRONT"]["DISC_RADIUS"]) == pytest.approx(0.28)
        assert float(parsed["REAR"]["DISC_RADIUS"]) == pytest.approx(0.25)

    def test_bias_preserved(self, parsed) -> None:
        assert float(parsed["BRAKES"]["BIAS"]) == pytest.approx(0.58)


# --------------------------------------------------------------------------- #
# Aero coordinate transform                                                   #
# --------------------------------------------------------------------------- #


class TestAeroCoordsTransform:
    def test_wing_position_transformed_from_svj(self, tmp_path: Path) -> None:
        doc = _non_ac_doc()
        out = aero_ini.write(doc, tmp_path)
        parsed = parse_ini(out.read_text(encoding="utf-8"))

        # SVJ front splitter at (2.0, 0.0, -0.23)  ->  AC (0.0, 0.23, 2.0)
        wing0 = parsed["WING_0"]["POSITION"]
        parts = [float(p.strip()) for p in wing0.replace("|", ",").split(",") if p.strip()]
        expected = svj_to_ac_point((2.0, 0.0, -0.23))
        for got, exp in zip(parts, expected):
            assert abs(got - exp) < 1e-6, f"{got} != {exp}"

    def test_rear_wing_position_transformed(self, tmp_path: Path) -> None:
        doc = _non_ac_doc()
        out = aero_ini.write(doc, tmp_path)
        parsed = parse_ini(out.read_text(encoding="utf-8"))
        wing1 = parsed["WING_1"]["POSITION"]
        parts = [float(p.strip()) for p in wing1.replace("|", ",").split(",") if p.strip()]
        expected = svj_to_ac_point((-2.0, 0.0, -0.33))
        for got, exp in zip(parts, expected):
            assert abs(got - exp) < 1e-6


# --------------------------------------------------------------------------- #
# Mixed: native overrides win over canonical                                  #
# --------------------------------------------------------------------------- #


class TestNativeOverridesWin:
    """When an SVJ has both canonical SVJ blocks AND an x_assettocorsa
    AC-native override, the override must take precedence (it's the
    user explicitly saying 'use my AC value')."""

    def test_x_assettocorsa_brakes_overrides_corner(self, tmp_path: Path) -> None:
        from svj_loader import SvjAssettoExt

        doc = _non_ac_doc()
        doc.assetto = SvjAssettoExt(raw={"brakes": {"front": {"MAX_TORQUE": 9999}}})
        out = brakes_ini.write(doc, tmp_path)
        parsed = parse_ini(out.read_text(encoding="utf-8"))
        assert float(parsed["FRONT"]["MAX_TORQUE"]) == pytest.approx(9999)
