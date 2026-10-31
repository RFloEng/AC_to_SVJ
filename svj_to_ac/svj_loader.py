"""SVJ v0.95 JSON loader and schema validation.

Loads a Standard Vehicle JSON (SAE J670 axes, SI units) into a typed
``SvjDoc`` tree and validates the contents against the v0.95 schema. The
public API is:

* :func:`load`     -- read a file, raise on missing required sections.
* :func:`validate` -- soft check, returns ``list[SvjError]``.
* :class:`SvjDoc`  -- top-level dataclass mirroring the JSON tree.

The schema mirrored here matches ``AC_to_SVJ/examples/mx5_nd_club.svj.json``
rather than the speculative key list in ``ai_handoff/02_PROJECT_SPEC.md``:
real SVJ files use ``vehicle_info`` + ``_metadata`` (not ``metadata``),
``tires`` (not ``tyres``), ``aerodynamics`` (not ``aero``), and carry
electronics, setup ranges, and controllers under the ``x_assettocorsa``
extension block. The dataclass tree exposes both shapes so downstream
writers can read either name.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from unified_view import normalize as normalize_unified

REQUIRED_SECTIONS: tuple[str, ...] = (
    "vehicle_info",
    "chassis",
    "suspension",
    "tires",
    "powertrain",
    "brakes",
)
OPTIONAL_SECTIONS: tuple[str, ...] = ("_metadata", "aerodynamics", "x_assettocorsa")
SUPPORTED_VERSION = "0.95"
EXPECTED_COORDS = "SAE_J670"
EXPECTED_UNITS = "SI"
CORNERS: tuple[str, ...] = ("FL", "FR", "RL", "RR")


class SvjLoadError(Exception):
    """Raised by :func:`load` when the file cannot be turned into an SvjDoc."""


class MissingSectionError(SvjLoadError):
    """One or more required top-level sections are missing from the JSON."""

    def __init__(self, missing: list[str]) -> None:
        self.missing = list(missing)
        super().__init__(
            "SVJ JSON is missing required section(s): " + ", ".join(self.missing)
        )


@dataclass(frozen=True)
class SvjError:
    """Soft validation finding produced by :func:`validate`."""

    path: str
    message: str
    severity: str = "warning"


@dataclass
class SvjMetadata:
    specification: str | None = None
    version: str | None = None
    coordinate_system: str | None = None
    units: str | None = None
    alignment_convention: str | None = None
    source_format: str | None = None
    author: str | None = None
    created: str | None = None
    description: str | None = None
    raw: dict[str, Any] = field(default_factory=dict)


@dataclass
class SvjVehicleInfo:
    make: str
    model: str
    year: int
    variant: str
    drive_type: str
    raw: dict[str, Any] = field(default_factory=dict)


@dataclass
class SvjChassis:
    mass_total: float
    wheelbase: float
    track_front: float
    track_rear: float
    center_of_gravity: list[float]
    mass_unsprung_per_corner: dict[str, float] = field(default_factory=dict)
    fuel_capacity: float | None = None
    fuel_initial: float | None = None
    raw: dict[str, Any] = field(default_factory=dict)


@dataclass
class SvjCorner:
    name: str
    position: list[float]
    topology: dict[str, Any] = field(default_factory=dict)
    raw: dict[str, Any] = field(default_factory=dict)


@dataclass
class SvjSuspension:
    corners: dict[str, SvjCorner]

    def __getitem__(self, corner: str) -> SvjCorner:
        return self.corners[corner]


@dataclass
class SvjTires:
    sets: dict[str, dict[str, Any]] = field(default_factory=dict)
    raw: dict[str, Any] = field(default_factory=dict)


@dataclass
class SvjPowertrain:
    layout: str
    engine: dict[str, Any] = field(default_factory=dict)
    gearbox: dict[str, Any] = field(default_factory=dict)
    differentials: list[dict[str, Any]] = field(default_factory=list)
    raw: dict[str, Any] = field(default_factory=dict)


@dataclass
class SvjBrakes:
    bias: float
    bias_type: str
    raw: dict[str, Any] = field(default_factory=dict)


@dataclass
class SvjAerodynamics:
    reference: dict[str, Any] = field(default_factory=dict)
    global_coefficients: dict[str, Any] = field(default_factory=dict)
    coefficients: dict[str, Any] = field(default_factory=dict)
    components: list[dict[str, Any]] = field(default_factory=list)
    raw: dict[str, Any] = field(default_factory=dict)


@dataclass
class SvjAssettoExt:
    cm_specs: dict[str, Any] = field(default_factory=dict)
    damage: dict[str, Any] = field(default_factory=dict)
    driver_aids: dict[str, Any] = field(default_factory=dict)
    setup_ranges: dict[str, Any] = field(default_factory=dict)
    info: dict[str, Any] = field(default_factory=dict)
    controls: dict[str, Any] = field(default_factory=dict)
    pit_stop: dict[str, Any] = field(default_factory=dict)
    fuel_tank_position: list[float] | None = None
    versions: dict[str, Any] = field(default_factory=dict)
    raw: dict[str, Any] = field(default_factory=dict)


@dataclass
class SvjDoc:
    vehicle_info: SvjVehicleInfo
    chassis: SvjChassis
    suspension: SvjSuspension
    tires: SvjTires
    powertrain: SvjPowertrain
    brakes: SvjBrakes
    metadata: SvjMetadata | None = None
    aerodynamics: SvjAerodynamics | None = None
    assetto: SvjAssettoExt | None = None
    raw: dict[str, Any] = field(default_factory=dict)
    source_path: Path | None = None
    notes: list[str] = field(default_factory=list)   # what the loader had to guess

    @property
    def tyres(self) -> SvjTires:
        return self.tires

    @property
    def aero(self) -> SvjAerodynamics | None:
        return self.aerodynamics


def load(path: str | Path) -> SvjDoc:
    """Read ``path``, parse JSON, build :class:`SvjDoc`.

    Raises
    ------
    SvjLoadError
        On JSON parse failure or unreadable file.
    MissingSectionError
        If any required top-level section is absent.
    """
    p = Path(path)
    try:
        text = p.read_text(encoding="utf-8")
    except OSError as exc:
        raise SvjLoadError(f"cannot read {p}: {exc}") from exc

    try:
        data: dict[str, Any] = json.loads(text)
    except json.JSONDecodeError as exc:
        raise SvjLoadError(f"invalid JSON in {p}: {exc}") from exc

    if not isinstance(data, dict):
        raise SvjLoadError(f"SVJ root must be an object, got {type(data).__name__}")

    data, notes = normalize_unified(data)          # SVJ 0.100+: derive the corner blocks
    missing = [s for s in REQUIRED_SECTIONS if s not in data]
    if missing:
        raise MissingSectionError(missing)

    doc = _build_doc(data, source_path=p)
    doc.notes.extend(notes)
    return doc


def validate(doc: SvjDoc) -> list[SvjError]:
    """Soft validation. Returns a list of findings (empty = clean)."""
    errors: list[SvjError] = []

    if doc.metadata is not None:
        if doc.metadata.version and doc.metadata.version != SUPPORTED_VERSION:
            errors.append(
                SvjError(
                    "_metadata.version",
                    f"unexpected SVJ version {doc.metadata.version!r} (loader targets {SUPPORTED_VERSION!r})",
                    "warning",
                )
            )
        if doc.metadata.coordinate_system and doc.metadata.coordinate_system != EXPECTED_COORDS:
            errors.append(
                SvjError(
                    "_metadata.coordinate_system",
                    f"expected {EXPECTED_COORDS!r}, got {doc.metadata.coordinate_system!r}",
                    "error",
                )
            )
        if doc.metadata.units and doc.metadata.units != EXPECTED_UNITS:
            errors.append(
                SvjError(
                    "_metadata.units",
                    f"expected {EXPECTED_UNITS!r}, got {doc.metadata.units!r}",
                    "error",
                )
            )

    if doc.chassis.mass_total <= 0:
        errors.append(SvjError("chassis.mass_total", "must be > 0", "error"))
    if doc.chassis.wheelbase <= 0:
        errors.append(SvjError("chassis.wheelbase", "must be > 0", "error"))
    if doc.chassis.track_front <= 0:
        errors.append(SvjError("chassis.track_front", "must be > 0", "error"))
    if doc.chassis.track_rear <= 0:
        errors.append(SvjError("chassis.track_rear", "must be > 0", "error"))

    missing_corners = [c for c in CORNERS if c not in doc.suspension.corners]
    if missing_corners:
        errors.append(
            SvjError(
                "suspension",
                f"missing corner(s): {', '.join(missing_corners)}",
                "error",
            )
        )

    if not doc.tires.sets:
        errors.append(SvjError("tires.sets", "no tire sets defined", "error"))

    if not (0.0 <= doc.brakes.bias <= 1.0):
        errors.append(SvjError("brakes.bias", "expected fraction in [0, 1]", "warning"))

    return errors


def _build_doc(data: dict[str, Any], source_path: Path | None) -> SvjDoc:
    return SvjDoc(
        vehicle_info=_build_vehicle_info(data["vehicle_info"]),
        chassis=_build_chassis(data["chassis"]),
        suspension=_build_suspension(data["suspension"]),
        tires=_build_tires(data["tires"]),
        powertrain=_build_powertrain(data["powertrain"]),
        brakes=_build_brakes(data["brakes"]),
        metadata=_build_metadata(data.get("_metadata")),
        aerodynamics=_build_aerodynamics(data.get("aerodynamics")),
        assetto=_build_assetto(data.get("x_assettocorsa")),
        raw=data,
        source_path=source_path,
    )


def _build_metadata(d: dict[str, Any] | None) -> SvjMetadata | None:
    if d is None:
        return None
    return SvjMetadata(
        specification=d.get("specification"),
        version=d.get("version"),
        coordinate_system=d.get("coordinate_system"),
        units=d.get("units"),
        alignment_convention=d.get("alignment_convention"),
        source_format=d.get("source_format"),
        author=d.get("author"),
        created=d.get("created"),
        description=d.get("description"),
        raw=dict(d),
    )


def _build_vehicle_info(d: dict[str, Any]) -> SvjVehicleInfo:
    return SvjVehicleInfo(
        make=str(d.get("make", "")),
        model=str(d.get("model", "")),
        year=int(d.get("year", 0)),
        variant=str(d.get("variant", "")),
        drive_type=str(d.get("drive_type", "")),
        raw=dict(d),
    )


def _build_chassis(d: dict[str, Any]) -> SvjChassis:
    return SvjChassis(
        mass_total=float(d.get("mass_total", 0.0)),
        wheelbase=float(d.get("wheelbase", 0.0)),
        track_front=float(d.get("track_front", 0.0)),
        track_rear=float(d.get("track_rear", 0.0)),
        center_of_gravity=list(d.get("center_of_gravity", [0.0, 0.0, 0.0])),
        mass_unsprung_per_corner=dict(d.get("mass_unsprung_per_corner", {})),
        fuel_capacity=_to_optional_float(d.get("fuel_capacity")),
        fuel_initial=_to_optional_float(d.get("fuel_initial")),
        raw=dict(d),
    )


def _build_suspension(d: dict[str, Any]) -> SvjSuspension:
    corners: dict[str, SvjCorner] = {}
    for name in CORNERS:
        if name in d:
            c = d[name]
            corners[name] = SvjCorner(
                name=name,
                position=list(c.get("position", [0.0, 0.0, 0.0])),
                topology=dict(c.get("topology", {})),
                raw=dict(c),
            )
    return SvjSuspension(corners=corners)


def _build_tires(d: dict[str, Any]) -> SvjTires:
    return SvjTires(sets=dict(d.get("sets", {})), raw=dict(d))


def _build_powertrain(d: dict[str, Any]) -> SvjPowertrain:
    return SvjPowertrain(
        layout=str(d.get("layout", "")),
        engine=dict(d.get("engine", {})),
        gearbox=dict(d.get("gearbox", {})),
        differentials=list(d.get("differentials", [])),
        raw=dict(d),
    )


def _build_brakes(d: dict[str, Any]) -> SvjBrakes:
    return SvjBrakes(
        bias=float(d.get("bias", 0.0)),
        bias_type=str(d.get("bias_type", "")),
        raw=dict(d),
    )


def _build_aerodynamics(d: dict[str, Any] | None) -> SvjAerodynamics | None:
    if d is None:
        return None
    return SvjAerodynamics(
        reference=dict(d.get("reference", {})),
        global_coefficients=dict(d.get("global_coefficients", {})),
        coefficients=dict(d.get("coefficients", {})),
        components=list(d.get("components", [])),
        raw=dict(d),
    )


def _build_assetto(d: dict[str, Any] | None) -> SvjAssettoExt | None:
    if d is None:
        return None
    return SvjAssettoExt(
        cm_specs=dict(d.get("cm_specs", {})),
        damage=dict(d.get("damage", {})),
        driver_aids=dict(d.get("driver_aids", {})),
        setup_ranges=dict(d.get("setup_ranges", {})),
        info=dict(d.get("info", {})),
        controls=dict(d.get("controls", {})),
        pit_stop=dict(d.get("pit_stop", {})),
        fuel_tank_position=(
            list(d["fuel_tank_position"]) if "fuel_tank_position" in d else None
        ),
        versions=dict(d.get("versions", {})),
        raw=dict(d),
    )


def _to_optional_float(v: Any) -> float | None:
    if v is None:
        return None
    return float(v)


__all__ = [
    "CORNERS",
    "EXPECTED_COORDS",
    "EXPECTED_UNITS",
    "MissingSectionError",
    "OPTIONAL_SECTIONS",
    "REQUIRED_SECTIONS",
    "SUPPORTED_VERSION",
    "SvjAerodynamics",
    "SvjAssettoExt",
    "SvjBrakes",
    "SvjChassis",
    "SvjCorner",
    "SvjDoc",
    "SvjError",
    "SvjLoadError",
    "SvjMetadata",
    "SvjPowertrain",
    "SvjSuspension",
    "SvjTires",
    "SvjVehicleInfo",
    "load",
    "validate",
]
