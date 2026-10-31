"""ASCII FBX 2014 skeleton generator.

Produces a minimal-but-valid ASCII FBX file that KsEditor can compile
into a working ``.kn5``. The generated file contains:

* One root null ``Car``.
* Four wheel dummies ``WHEEL_LF/RF/LR/RR`` positioned from SVJ
  wheelbase + half-track + rolling radius (converted SVJ -> AC via
  :func:`coords.svj_to_ac_point`, then SI metres -> centimetres via
  ``* 100``).
* Four suspension dummies ``SUSP_LF/RF/LR/RR`` as siblings of the
  wheel dummies, same translation.
* Cockpit dummies ``COCKPIT_HR`` and ``STEER_HR``.
* A cube mesh ``BODY`` sized to the overall vehicle bounding box.
* One cylinder mesh per wheel (``WHEEL_LF_MESH``, etc.) sized to tyre
  width + rolling radius.
* A single default ``ksPerPixel``-style grey material attached to every
  mesh.

Pitfalls accommodated (``ai_handoff/05_PITFALLS.md §"KsEditor FBX requirements"``):

* Header version ``FBXVersion: 7400`` (FBX 2014).
* ``GlobalSettings``: ``UpAxis=1, UpAxisSign=1, FrontAxis=2,
  FrontAxisSign=1, CoordAxis=0, CoordAxisSign=1`` (Z-up, Y-forward,
  X-right).
* ``UnitScaleFactor=1.0`` paired with ``OriginalUnitScaleFactor=1.0``;
  coordinates are emitted in centimetres so 1 unit == 1 cm.
* Dummy names exactly ``WHEEL_LF/RF/LR/RR`` and ``SUSP_LF/RF/LR/RR``
  (uppercase, underscore separator).

Companion files (Stage 7 packager wires them up):

* :func:`write_lods_ini` -- ``lods.ini`` with a single ``LOD_0`` entry.
* :func:`write_ksanim_placeholder` -- empty ``persistence.ksanim``.

No FBX binding library is introduced (per build plan); the format is
hand-rolled from a string template.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Iterable

from coords import svj_to_ac_point
from svj_loader import SvjDoc

# Convert SVJ-metres to AC FBX-centimetres (Z-up world). The factor 100
# is a unit conversion (m -> cm); kept here as the only centimetre
# constant in the project (FBX is the only consumer of cm).
_M_TO_CM = 100.0

WHEEL_NAMES = ("WHEEL_LF", "WHEEL_RF", "WHEEL_LR", "WHEEL_RR")
SUSP_NAMES = ("SUSP_LF", "SUSP_RF", "SUSP_LR", "SUSP_RR")
COCKPIT_NAMES = ("COCKPIT_HR", "STEER_HR")

# A coarse but distinct integer-id sequence -- FBX requires unique
# 64-bit ids per object. We use offsets from a base so collisions
# between dummies, meshes, and materials are impossible.
_ID_BASE = {
    "scene": 100,
    "material": 1_000,
    "dummy": 10_000,
    "mesh_model": 20_000,
    "mesh_geom": 30_000,
}


def write(
    doc: SvjDoc,
    out_path: Path,
    *,
    overall_length: float | None = None,
    overall_width: float | None = None,
    overall_height: float | None = None,
    tyre_width: float | None = None,
) -> Path:
    """Write an ASCII FBX 2014 skeleton for ``doc`` to ``out_path``.

    Optional override args let the FBX Lab tab (Stage 9) supply
    dimensions when the SVJ doesn't carry them.
    """
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    text = render(
        doc,
        overall_length=overall_length,
        overall_width=overall_width,
        overall_height=overall_height,
        tyre_width=tyre_width,
    )
    out_path.write_text(text, encoding="ascii", newline="\n")
    return out_path


def render(
    doc: SvjDoc,
    *,
    overall_length: float | None = None,
    overall_width: float | None = None,
    overall_height: float | None = None,
    tyre_width: float | None = None,
) -> str:
    """Compose the full FBX text."""
    dims = _resolve_dimensions(
        doc,
        overall_length=overall_length,
        overall_width=overall_width,
        overall_height=overall_height,
        tyre_width=tyre_width,
    )
    positions = _wheel_positions_cm(doc)

    return (
        _header()
        + _global_settings()
        + _documents()
        + _references()
        + _definitions()
        + _objects(doc, dims, positions)
        + _connections(positions)
        + _takes()
    )


# --------------------------------------------------------------------------- #
# Companion files                                                             #
# --------------------------------------------------------------------------- #


def write_lods_ini(out_path: Path, fbx_name: str = "car.kn5") -> Path:
    """Emit a minimal ``lods.ini`` with a single ``LOD_0`` entry.

    AC reads this file to know which KN5 file to use for each detail
    level. Single-LOD cars just declare LOD_0 = always active.
    """
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    text = (
        "[LOD_0]\n"
        f"FILE={fbx_name}\n"
        "IN=0\n"
        "OUT=10000\n"
    )
    out_path.write_text(text, encoding="utf-8", newline="\n")
    return out_path


def write_ksanim_placeholder(out_path: Path) -> Path:
    """Emit an empty ``persistence.ksanim`` placeholder.

    Real ksanim files are a binary keyframe format; KsEditor regenerates
    them on save, so the placeholder just needs to exist and be empty.
    """
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_bytes(b"")
    return out_path


# --------------------------------------------------------------------------- #
# Dimensions / positions                                                      #
# --------------------------------------------------------------------------- #


def _resolve_dimensions(
    doc: SvjDoc,
    *,
    overall_length: float | None,
    overall_width: float | None,
    overall_height: float | None,
    tyre_width: float | None,
) -> dict[str, float]:
    """Pick body-box dimensions in centimetres."""
    ch = doc.chassis
    # SVJ doesn't store overall-vehicle dimensions canonically, so
    # estimate from wheelbase + half-track when the caller doesn't pass
    # them. These defaults are intentionally generous so the resulting
    # cube reads as a car-shaped block in KsEditor.
    length_m = float(overall_length) if overall_length else (ch.wheelbase + 1.0)
    width_m = float(overall_width) if overall_width else (max(ch.track_front, ch.track_rear) + 0.1)
    height_m = float(overall_height) if overall_height else 1.30
    tyre_w_m = float(tyre_width) if tyre_width else _resolve_tyre_width(doc)
    # Rolling radius (front) for the wheel cylinder.
    rr_m = _resolve_rolling_radius(doc)
    return {
        "length_cm": length_m * _M_TO_CM,
        "width_cm": width_m * _M_TO_CM,
        "height_cm": height_m * _M_TO_CM,
        "tyre_width_cm": tyre_w_m * _M_TO_CM,
        "wheel_radius_cm": rr_m * _M_TO_CM,
    }


def _resolve_tyre_width(doc: SvjDoc) -> float:
    for body in (doc.tires.sets or {}).values():
        dims = (body or {}).get("dimensions", {})
        if isinstance(dims, dict) and dims.get("section_width"):
            return float(dims["section_width"])
    return 0.225  # 225 mm default


def _resolve_rolling_radius(doc: SvjDoc) -> float:
    for body in (doc.tires.sets or {}).values():
        dims = (body or {}).get("dimensions", {})
        if isinstance(dims, dict):
            for f in ("rolling_radius", "loaded_radius", "static_radius"):
                if f in dims:
                    return float(dims[f])
            if "overall_diameter" in dims:
                return float(dims["overall_diameter"]) / 2.0
    return 0.32  # 320 mm default


def _wheel_positions_cm(doc: SvjDoc) -> dict[str, tuple[float, float, float]]:
    """Wheel-centre world positions in AC FBX coordinates (Z-up, cm).

    SVJ corner positions are SAE J670 ``(x_forward, y_right, z_down)``
    in metres. We invoke :func:`coords.svj_to_ac_point` (-> AC Y-up),
    then swap AC ``(x_right, y_up, z_forward)`` to FBX ``(x, y_up=z,
    z_forward=y)`` -- FBX inside KsEditor uses Z-up where AC's Y-up
    aligns with FBX's Z.

    The mapping AC->FBX-Zup is: ``(ac.x, ac.z, ac.y)``. Combined with
    the SVJ->AC step, the net transform per coordinate is::

        fbx.x = svj.y                  (right)
        fbx.y = svj.x                  (forward)
        fbx.z = -svj.z                 (up; SVJ z is down)
    """
    positions: dict[str, tuple[float, float, float]] = {}
    for svj_key, name in (
        ("FL", "WHEEL_LF"),
        ("FR", "WHEEL_RF"),
        ("RL", "WHEEL_LR"),
        ("RR", "WHEEL_RR"),
    ):
        corner = doc.suspension.corners.get(svj_key)
        if corner is None:
            # Synthesise from wheelbase + track if SVJ doesn't carry the corner.
            wb = doc.chassis.wheelbase
            ht = (
                doc.chassis.track_front / 2.0
                if "F" in svj_key
                else doc.chassis.track_rear / 2.0
            )
            sign_x = wb if "F" in svj_key else -wb
            sign_y = ht if "R" in svj_key[-1:] else -ht
            svj_pt = (sign_x, sign_y, 0.0)
        else:
            svj_pt = tuple(corner.position[:3]) if len(corner.position) >= 3 else (0.0, 0.0, 0.0)

        ax, ay, az = svj_to_ac_point(svj_pt)
        # AC (X-right, Y-up, Z-forward) -> FBX Z-up world: (x, z, y)
        fbx = (ax * _M_TO_CM, az * _M_TO_CM, ay * _M_TO_CM)
        positions[name] = fbx
    return positions


def _cockpit_positions_cm(doc: SvjDoc) -> dict[str, tuple[float, float, float]]:
    """Driver / steering-wheel dummy positions (FBX Z-up, cm)."""
    wb = doc.chassis.wheelbase
    # Default: 0.4 m left of centre, 0.7 m up, 0.3 m back from front axle.
    cockpit_x_m = -0.4
    cockpit_y_m = wb / 2.0 - 0.3  # back-from-front-axle measured along forward
    cockpit_z_m = 0.7
    cockpit_cm = (cockpit_x_m * _M_TO_CM, cockpit_y_m * _M_TO_CM, cockpit_z_m * _M_TO_CM)
    # STEER dummy: 0.1 m forward of COCKPIT_HR, 0.05 m down.
    steer_cm = (cockpit_cm[0], cockpit_cm[1] + 0.1 * _M_TO_CM, cockpit_cm[2] - 0.05 * _M_TO_CM)
    return {"COCKPIT_HR": cockpit_cm, "STEER_HR": steer_cm}


# --------------------------------------------------------------------------- #
# FBX text emitters                                                           #
# --------------------------------------------------------------------------- #


def _header() -> str:
    return (
        "; FBX 7.4.0 project file\n"
        "; ----------------------------------------------------\n"
        "\n"
        "FBXHeaderExtension:  {\n"
        "    FBXHeaderVersion: 1003\n"
        "    FBXVersion: 7400\n"
        "    CreationTimeStamp:  {\n"
        "        Version: 1000\n"
        "        Year: 2026\n"
        "        Month: 1\n"
        "        Day: 1\n"
        "        Hour: 0\n"
        "        Minute: 0\n"
        "        Second: 0\n"
        "        Millisecond: 0\n"
        "    }\n"
        '    Creator: "SVJ_to_AC v0.1 - ASCII FBX 2014"\n'
        "    SceneInfo: \"SceneInfo::GlobalInfo\", \"UserData\" {\n"
        "        Type: \"UserData\"\n"
        "        Version: 100\n"
        "        MetaData:  {\n"
        "            Version: 100\n"
        '            Title: ""\n'
        '            Subject: ""\n'
        '            Author: "SVJ_to_AC"\n'
        '            Keywords: ""\n'
        '            Revision: ""\n'
        '            Comment: ""\n'
        "        }\n"
        "    }\n"
        "}\n"
        "\n"
    )


def _global_settings() -> str:
    return (
        "GlobalSettings:  {\n"
        "    Version: 1000\n"
        "    Properties70:  {\n"
        '        P: "UpAxis", "int", "Integer", "", 1\n'
        '        P: "UpAxisSign", "int", "Integer", "", 1\n'
        '        P: "FrontAxis", "int", "Integer", "", 2\n'
        '        P: "FrontAxisSign", "int", "Integer", "", 1\n'
        '        P: "CoordAxis", "int", "Integer", "", 0\n'
        '        P: "CoordAxisSign", "int", "Integer", "", 1\n'
        '        P: "OriginalUpAxis", "int", "Integer", "", -1\n'
        '        P: "OriginalUpAxisSign", "int", "Integer", "", 1\n'
        '        P: "UnitScaleFactor", "double", "Number", "", 1\n'
        '        P: "OriginalUnitScaleFactor", "double", "Number", "", 1\n'
        '        P: "AmbientColor", "ColorRGB", "Color", "", 0, 0, 0\n'
        '        P: "DefaultCamera", "KString", "", "", "Producer Perspective"\n'
        '        P: "TimeMode", "enum", "", "", 11\n'
        "    }\n"
        "}\n"
        "\n"
    )


def _documents() -> str:
    return (
        "Documents:  {\n"
        "    Count: 1\n"
        '    Document: 1, "", "Scene" {\n'
        "        Properties70:  {\n"
        '            P: "SourceObject", "object", "", ""\n'
        '            P: "ActiveAnimStackName", "KString", "", "", ""\n'
        "        }\n"
        "        RootNode: 0\n"
        "    }\n"
        "}\n"
        "\n"
    )


def _references() -> str:
    return "References:  {\n}\n\n"


def _definitions() -> str:
    return (
        "Definitions:  {\n"
        "    Version: 100\n"
        "    Count: 4\n"
        '    ObjectType: "GlobalSettings" { Count: 1 }\n'
        '    ObjectType: "Model" { Count: 11 }\n'
        '    ObjectType: "Geometry" { Count: 5 }\n'
        '    ObjectType: "Material" { Count: 1 }\n'
        "}\n"
        "\n"
    )


def _objects(
    doc: SvjDoc,
    dims: dict[str, float],
    positions: dict[str, tuple[float, float, float]],
) -> str:
    out: list[str] = ["Objects:  {\n"]

    # Root Car null.
    out.append(_null_model(_ID_BASE["dummy"], "Car", (0.0, 0.0, 0.0)))
    # Wheel + susp dummies (Susp shares wheel position).
    for i, (w_name, s_name) in enumerate(zip(WHEEL_NAMES, SUSP_NAMES)):
        pos = positions[w_name]
        out.append(_null_model(_ID_BASE["dummy"] + 100 + i * 2, w_name, pos))
        out.append(_null_model(_ID_BASE["dummy"] + 101 + i * 2, s_name, pos))
    # Cockpit dummies.
    cockpit = _cockpit_positions_cm(doc)
    for i, name in enumerate(COCKPIT_NAMES):
        out.append(_null_model(_ID_BASE["dummy"] + 200 + i, name, cockpit[name]))

    # Default Kunos-grey material.
    out.append(_material(_ID_BASE["material"], "ksPerPixel_default"))

    # BODY mesh.
    out.append(
        _cube_geometry(
            _ID_BASE["mesh_geom"], "BODY",
            length_cm=dims["length_cm"],
            width_cm=dims["width_cm"],
            height_cm=dims["height_cm"],
        )
    )
    out.append(_mesh_model(_ID_BASE["mesh_model"], "BODY", (0.0, 0.0, dims["height_cm"] / 2.0)))

    # Wheel cylinder meshes.
    for i, name in enumerate(WHEEL_NAMES):
        geom_id = _ID_BASE["mesh_geom"] + 100 + i
        model_id = _ID_BASE["mesh_model"] + 100 + i
        out.append(
            _cylinder_geometry(
                geom_id, f"{name}_MESH",
                radius_cm=dims["wheel_radius_cm"],
                width_cm=dims["tyre_width_cm"],
            )
        )
        out.append(_mesh_model(model_id, f"{name}_MESH", positions[name]))

    out.append("}\n\n")
    return "".join(out)


def _connections(positions: dict[str, tuple[float, float, float]]) -> str:
    out: list[str] = [
        "Connections:  {\n",
        f'    C: "OO", {_ID_BASE["dummy"]}, 0\n',  # Car -> root
    ]
    # Wheel + susp dummies under Car.
    for i, (w_name, s_name) in enumerate(zip(WHEEL_NAMES, SUSP_NAMES)):
        wid = _ID_BASE["dummy"] + 100 + i * 2
        sid = _ID_BASE["dummy"] + 101 + i * 2
        out.append(f'    C: "OO", {wid}, {_ID_BASE["dummy"]}\n')
        out.append(f'    C: "OO", {sid}, {_ID_BASE["dummy"]}\n')
    # Cockpit dummies under Car.
    for i in range(len(COCKPIT_NAMES)):
        cid = _ID_BASE["dummy"] + 200 + i
        out.append(f'    C: "OO", {cid}, {_ID_BASE["dummy"]}\n')

    # BODY mesh + material.
    out.append(f'    C: "OO", {_ID_BASE["mesh_model"]}, {_ID_BASE["dummy"]}\n')
    out.append(f'    C: "OO", {_ID_BASE["mesh_geom"]}, {_ID_BASE["mesh_model"]}\n')
    out.append(f'    C: "OO", {_ID_BASE["material"]}, {_ID_BASE["mesh_model"]}\n')

    # Wheel meshes attached to their dummies.
    for i in range(len(WHEEL_NAMES)):
        geom_id = _ID_BASE["mesh_geom"] + 100 + i
        model_id = _ID_BASE["mesh_model"] + 100 + i
        dummy_id = _ID_BASE["dummy"] + 100 + i * 2
        out.append(f'    C: "OO", {model_id}, {dummy_id}\n')
        out.append(f'    C: "OO", {geom_id}, {model_id}\n')
        out.append(f'    C: "OO", {_ID_BASE["material"]}, {model_id}\n')

    out.append("}\n\n")
    return "".join(out)


def _takes() -> str:
    return (
        "Takes:  {\n"
        '    Current: ""\n'
        "}\n"
    )


# --------------------------------------------------------------------------- #
# Object emitters                                                             #
# --------------------------------------------------------------------------- #


def _null_model(uid: int, name: str, translation: tuple[float, float, float]) -> str:
    tx, ty, tz = translation
    return (
        f'    Model: {uid}, "Model::{name}", "Null" {{\n'
        "        Version: 232\n"
        "        Properties70:  {\n"
        f'            P: "Lcl Translation", "Lcl Translation", "", "A", {tx}, {ty}, {tz}\n'
        '            P: "DefaultAttributeIndex", "int", "Integer", "", 0\n'
        "        }\n"
        '        Shading: T\n'
        '        Culling: "CullingOff"\n'
        "    }\n"
    )


def _mesh_model(uid: int, name: str, translation: tuple[float, float, float]) -> str:
    tx, ty, tz = translation
    return (
        f'    Model: {uid}, "Model::{name}", "Mesh" {{\n'
        "        Version: 232\n"
        "        Properties70:  {\n"
        f'            P: "Lcl Translation", "Lcl Translation", "", "A", {tx}, {ty}, {tz}\n'
        '            P: "DefaultAttributeIndex", "int", "Integer", "", 0\n'
        "        }\n"
        '        Shading: Y\n'
        '        Culling: "CullingOff"\n'
        "    }\n"
    )


def _material(uid: int, name: str) -> str:
    return (
        f'    Material: {uid}, "Material::{name}", "" {{\n'
        "        Version: 102\n"
        '        ShadingModel: "phong"\n'
        "        MultiLayer: 0\n"
        "        Properties70:  {\n"
        '            P: "ShadingModel", "KString", "", "", "phong"\n'
        '            P: "DiffuseColor", "Color", "", "A", 0.6, 0.6, 0.6\n'
        '            P: "SpecularColor", "Color", "", "A", 0.3, 0.3, 0.3\n'
        '            P: "Shininess", "Number", "", "A", 20\n'
        "        }\n"
        "    }\n"
    )


def _cube_geometry(uid: int, name: str, *, length_cm: float, width_cm: float, height_cm: float) -> str:
    """Axis-aligned box centred at origin (caller translates the parent model)."""
    hx = width_cm / 2.0
    hy = length_cm / 2.0
    hz = height_cm / 2.0
    verts = [
        (-hx, -hy, -hz), ( hx, -hy, -hz), ( hx,  hy, -hz), (-hx,  hy, -hz),
        (-hx, -hy,  hz), ( hx, -hy,  hz), ( hx,  hy,  hz), (-hx,  hy,  hz),
    ]
    # Triangle indices in FBX polygon-vertex-index format: last index of
    # each face is bitwise-NOT (negative-one-based) to mark end of polygon.
    faces = [
        (0, 1, 2, 3),  # bottom
        (4, 7, 6, 5),  # top
        (0, 4, 5, 1),  # front
        (1, 5, 6, 2),  # right
        (2, 6, 7, 3),  # back
        (3, 7, 4, 0),  # left
    ]
    return _geometry(uid, name, verts, faces)


def _cylinder_geometry(uid: int, name: str, *, radius_cm: float, width_cm: float, segments: int = 12) -> str:
    """Cylinder along the X axis (wheels spin around X in AC FBX)."""
    verts: list[tuple[float, float, float]] = []
    hx = width_cm / 2.0
    for i in range(segments):
        ang = 2.0 * math.pi * i / segments
        y = radius_cm * math.cos(ang)
        z = radius_cm * math.sin(ang)
        verts.append((-hx, y, z))
    for i in range(segments):
        ang = 2.0 * math.pi * i / segments
        y = radius_cm * math.cos(ang)
        z = radius_cm * math.sin(ang)
        verts.append((hx, y, z))
    faces: list[tuple[int, ...]] = []
    for i in range(segments):
        a = i
        b = (i + 1) % segments
        c = b + segments
        d = a + segments
        faces.append((a, b, c, d))
    # End caps as triangle fans (split into quads where possible).
    for i in range(1, segments - 1):
        faces.append((0, i + 1, i))
        faces.append((segments, segments + i, segments + i + 1))
    return _geometry(uid, name, verts, faces)


def _geometry(uid: int, name: str, verts: list[tuple[float, float, float]], faces: list[tuple[int, ...]]) -> str:
    vert_flat: list[float] = []
    for v in verts:
        vert_flat.extend(v)
    poly_flat: list[int] = []
    for face in faces:
        for k, idx in enumerate(face):
            if k == len(face) - 1:
                poly_flat.append(-(idx + 1))
            else:
                poly_flat.append(idx)
    vert_text = ",".join(_fmt_num(x) for x in vert_flat)
    poly_text = ",".join(str(i) for i in poly_flat)
    return (
        f'    Geometry: {uid}, "Geometry::{name}", "Mesh" {{\n'
        f"        Vertices: *{len(vert_flat)} {{\n"
        f"            a: {vert_text}\n"
        "        }\n"
        f"        PolygonVertexIndex: *{len(poly_flat)} {{\n"
        f"            a: {poly_text}\n"
        "        }\n"
        "        GeometryVersion: 124\n"
        '        LayerElementNormal: 0 {\n'
        "            Version: 102\n"
        '            Name: ""\n'
        '            MappingInformationType: "ByPolygonVertex"\n'
        '            ReferenceInformationType: "Direct"\n'
        "            Normals: *3 { a: 0,0,1 }\n"
        "        }\n"
        '        Layer: 0 {\n'
        "            Version: 100\n"
        "            LayerElement:  {\n"
        '                Type: "LayerElementNormal"\n'
        "                TypedIndex: 0\n"
        "            }\n"
        "        }\n"
        "    }\n"
    )


def _fmt_num(v: float) -> str:
    if v == int(v):
        return str(int(v))
    return f"{v:.6f}"


__all__ = [
    "COCKPIT_NAMES",
    "SUSP_NAMES",
    "WHEEL_NAMES",
    "render",
    "write",
    "write_ksanim_placeholder",
    "write_lods_ini",
]
