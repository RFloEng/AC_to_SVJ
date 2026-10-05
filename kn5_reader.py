"""
KN5 reader and glTF exporter for Assetto Corsa 3D models.

Binary format reference: RaduMC/kn5-converter (C# reverse-engineering)

File layout
-----------
  Magic        : 6 bytes   "sc6969"
  Version      : int32     (1, 2, 4, 5, 6 …)
  [if ver > 5] : int32     unknown (673425)
  Textures     : count + {type:i32, nameLen:i32, name, dataLen:i32, data}
  Materials    : count + {nameLen, name, shaderLen, shader, i16,
                          [if ver>4: i32], propCount + {nameLen, name,
                          f32, skip36}, texCount + {sampleNameLen, name,
                          slot:i32, texNameLen, texName}}
  Nodes (tree) : readNodes() — recursive, always starting at root

Node record
-----------
  type         : int32      1=transform, 2=static mesh, 3=skinned mesh
  nameLen      : int32
  name         : bytes(nameLen)
  childCount   : int32
  active       : byte

  Type 1 (transform):
    matrix 4x4 : 16 x float32 (row-major, translation in row 3)

  Type 2 (static mesh):
    3 bytes  (isRenderable, isCastShadow, isVisible flags)
    vertCount: int32
    per vertex: pos(3f) + normal(3f) + uv(2f) + tangent(3f) = 11f = 44 bytes
    indexCount: int32
    indices  : indexCount x uint16
    materialID: int32
    29 bytes (layer, LOD-in, LOD-out, renderable flags)

  Type 3 (skinned mesh):
    3 bytes  (same flags)
    boneCount: int32
    per bone : nameLen, name, 4x4 matrix (64 bytes)
    vertCount: int32
    per vertex: pos(3f) + normal(3f) + uv(2f) + tangent(3f) +
                bone_weights(4f) + bone_indices(4f) = 19f = 76 bytes
    indexCount: int32
    indices  : indexCount x uint16
    materialID: int32
    12 bytes

Coordinate system
-----------------
AC / KN5  : left-handed, X-right, Y-up, Z-forward.
Three.js  : right-handed, X-right, Y-up, Z-backward  (glTF standard).

Transform applied: negate Y and Z  ->  three = (ac.x, -ac.y, -ac.z).
Negating two axes preserves handedness, so triangle winding is kept as-is.
"""

from __future__ import annotations

import io
import math
import re
import struct
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np

# --- Low-level binary helpers -------------------------------------------------

def _read_str(f: io.RawIOBase, length: int) -> str:
    return f.read(length).decode("utf-8", errors="replace")

def _read_i32(f: io.RawIOBase) -> int:
    return struct.unpack("<i", f.read(4))[0]

def _read_u16(f: io.RawIOBase) -> int:
    return struct.unpack("<H", f.read(2))[0]

def _read_f32(f: io.RawIOBase) -> float:
    return struct.unpack("<f", f.read(4))[0]

def _read_mat4(f: io.RawIOBase) -> list[float]:
    """Read 16 floats -- row-major 4x4 matrix as flat list."""
    return list(struct.unpack("<16f", f.read(64)))

def _len_str(f: io.RawIOBase) -> str:
    length = _read_i32(f)
    return _read_str(f, length)


# --- Data classes -------------------------------------------------------------

@dataclass
class Kn5Texture:
    name: str
    data: bytes          # raw DDS / PNG bytes


@dataclass
class Kn5Material:
    name: str
    shader: str
    blend_mode:  int   = 0      # 0=opaque, 1=alpha-blend, 256=alpha-test/cutout
    ks_ambient:  float = 0.6
    ks_diffuse:  float = 0.6
    ks_specular: float = 0.9
    ks_specular_exp: float = 20.0   # Blinn exponent; 20 when the property is absent
    diffuse_mult: float = 1.0
    tx_diffuse:       Optional[str] = None
    tx_normal:        Optional[str] = None
    tx_detail:        Optional[str] = None   # detail/overlay tiling texture
    tx_normal_detail: Optional[str] = None   # detail normal map
    use_detail:       bool  = False          # useDetail property (shader gate)
    detail_uv_mult:   float = 1.0            # detailUVMultiplier (typically 5–500)
    normal_uv_mult:   float = 1.0            # normalUVMultiplier
    fresnel_max:      float = 0.0            # fresnelMaxLevel (cap on reflectance)
    sun_specular:     float = 0.0            # sunSpecular: >0 marks car paint (clear lacquer lobe)
    sun_specular_exp: float = 1500.0         # sunSpecularEXP


@dataclass
class Kn5Node:
    node_type:   int          # 1=transform, 2=mesh, 3=skinned
    name:        str
    active:      bool
    children:    list["Kn5Node"] = field(default_factory=list)
    # Type-1 fields
    matrix:      Optional[list[float]] = None   # 16 floats, row-major
    # Type-2/3 fields
    positions:   Optional[np.ndarray] = None    # (N,3) float32
    normals:     Optional[np.ndarray] = None    # (N,3) float32
    uvs:         Optional[np.ndarray] = None    # (N,2) float32
    tangents:    Optional[np.ndarray] = None    # (N,4) float32  XYZ + W handedness
    indices:     Optional[np.ndarray] = None    # (M,)  uint16
    material_id: int = -1
    index_count: int = 0                        # set even when geometry=False


@dataclass
class Kn5Model:
    name:      str
    version:   int
    textures:  list[Kn5Texture]   = field(default_factory=list)
    materials: list[Kn5Material]  = field(default_factory=list)
    root:      Optional[Kn5Node]  = None


# --- KN5 parser ---------------------------------------------------------------

def _read_textures(f: io.RawIOBase) -> list[Kn5Texture]:
    count = _read_i32(f)
    textures = []
    for _ in range(count):
        tex_type = _read_i32(f)             # 0=external ref (name only), 1=embedded
        name = _len_str(f)
        if tex_type == 0:
            # External reference: name only -- NO size/data fields in the file.
            textures.append(Kn5Texture(name=name, data=b""))
        else:
            # Embedded (type 1): size (i32) + raw bytes
            size = _read_i32(f)
            data = f.read(size)
            textures.append(Kn5Texture(name=name, data=data))
    return textures


def _read_materials(f: io.RawIOBase, version: int) -> list[Kn5Material]:
    count = _read_i32(f)
    materials = []
    for _ in range(count):
        name   = _len_str(f)
        shader = _len_str(f)
        blend_mode = struct.unpack("<h", f.read(2))[0]   # int16: blend mode
        if version > 4:
            f.read(4)                       # int32: depth mode / alpha flag
        mat = Kn5Material(name=name, shader=shader, blend_mode=blend_mode)

        prop_count = _read_i32(f)
        for _ in range(prop_count):
            prop_name  = _len_str(f)
            prop_value = _read_f32(f)
            f.read(36)                      # float array (unused by us)
            if prop_name == "ksAmbient":               mat.ks_ambient      = prop_value
            elif prop_name == "ksDiffuse":             mat.ks_diffuse      = prop_value
            elif prop_name == "ksSpecular":            mat.ks_specular     = prop_value
            elif prop_name == "ksSpecularEXP":         mat.ks_specular_exp = prop_value
            elif prop_name == "diffuseMult":           mat.diffuse_mult    = prop_value
            elif prop_name == "useDetail":             mat.use_detail      = prop_value > 0.0
            elif prop_name == "detailUVMultiplier":    mat.detail_uv_mult  = prop_value
            elif prop_name == "normalUVMultiplier":    mat.normal_uv_mult  = prop_value
            elif prop_name == "fresnelMaxLevel":      mat.fresnel_max     = prop_value
            elif prop_name == "sunSpecular":          mat.sun_specular    = prop_value
            elif prop_name == "sunSpecularEXP":       mat.sun_specular_exp = prop_value

        tex_count = _read_i32(f)
        for _ in range(tex_count):
            sample_name = _len_str(f)
            _slot       = _read_i32(f)
            tex_name    = _len_str(f)
            if sample_name == "txDiffuse":         mat.tx_diffuse       = tex_name
            elif sample_name == "txNormal":        mat.tx_normal        = tex_name
            elif sample_name == "txDetail":        mat.tx_detail        = tex_name
            elif sample_name == "txNormalDetail":  mat.tx_normal_detail = tex_name

        materials.append(mat)
    return materials


def _fold_texture_case(model: "Kn5Model") -> int:
    """
    Collapse texture-table entries that differ only in case and point every
    material slot at the surviving spelling.  Returns how many were folded.

    AC is a Windows game and matches texture names case-insensitively, so a car
    can store ``INT_DEcals.dds`` and ask a material for ``INT_Decals.dds`` and
    still render.  Written out literally that fails on case-sensitive systems
    (Linux, macOS) and two entries would collide on a case-insensitive disk.
    The surviving entry is the one with the largest blob (never the 1x1 stub).
    """
    keep: dict[str, int] = {}
    for i, t in enumerate(model.textures):
        k = t.name.lower()
        if k not in keep or len(t.data) > len(model.textures[keep[k]].data):
            keep[k] = i
    folded = len(model.textures) - len(keep)
    canon = {k: model.textures[i].name for k, i in keep.items()}
    model.textures = [model.textures[i] for i in sorted(keep.values())]
    for m in model.materials:
        for attr in ("tx_diffuse", "tx_normal", "tx_detail", "tx_normal_detail"):
            v = getattr(m, attr)
            if v:
                setattr(m, attr, canon.get(v.lower(), v))
    return folded


def _read_node(f: io.RawIOBase, geometry: bool) -> Kn5Node:
    """
    Read one node record (header + body). Children are NOT read here;
    caller fills node.children after recursion.
    `geometry=False` skips reading vertex/index buffers (node-name scan only).
    """
    node_type   = _read_i32(f)
    name        = _len_str(f)
    child_count = _read_i32(f)
    active      = bool(f.read(1)[0])

    node = Kn5Node(node_type=node_type, name=name, active=active)

    if node_type == 1:
        node.matrix = _read_mat4(f)

    elif node_type == 2:                    # static mesh
        f.read(3)                           # flags
        vert_count = _read_i32(f)
        # AC vertex layout: pos(12) + norm(12) + uv(8) + tangent(12) = 44 bytes (11 floats)
        if geometry:
            raw = np.frombuffer(f.read(vert_count * 44), dtype="<f4").reshape(vert_count, 11)
            node.positions = raw[:, 0:3].copy()
            node.normals   = raw[:, 3:6].copy()
            node.uvs       = raw[:, 6:8].copy()
            # AC uses DirectX UV convention (V=0 at top), same as glTF -- no V-flip needed.
            # Tangent XYZ at floats 8-10; W not stored in AC, set +1.0 (handedness).
            tan_xyz = raw[:, 8:11].copy()
            tan_w   = np.ones((vert_count, 1), dtype="<f4")
            node.tangents = np.concatenate([tan_xyz, tan_w], axis=1)
        else:
            f.seek(vert_count * 44, 1)
        idx_count   = _read_i32(f)
        node.index_count = idx_count
        if geometry:
            node.indices = np.frombuffer(f.read(idx_count * 2), dtype="<u2").copy()
        else:
            f.seek(idx_count * 2, 1)
        node.material_id = _read_i32(f)
        f.read(29)                          # layer / LOD / flags

    elif node_type == 3:                    # skinned mesh
        f.read(3)                           # flags
        bone_count = _read_i32(f)
        for _ in range(bone_count):
            _len_str(f)                     # bone name
            f.read(64)                      # 4x4 bone matrix
        vert_count = _read_i32(f)
        if geometry:
            # AC skinned vertex layout: pos(12) + norm(12) + uv(8) + tangent(12)
            # + bone_weights(4xf32=16) + bone_indices(4xf32=16) = 76 bytes = 19 floats
            raw = np.frombuffer(f.read(vert_count * 76), dtype="<f4").reshape(vert_count, 19)
            node.positions = raw[:, 0:3].copy()
            node.normals   = raw[:, 3:6].copy()
            node.uvs       = raw[:, 6:8].copy()
            # No V-flip -- AC DirectX UVs match glTF convention.
            tan_xyz = raw[:, 8:11].copy()
            tan_w   = np.ones((vert_count, 1), dtype="<f4")
            node.tangents = np.concatenate([tan_xyz, tan_w], axis=1)
        else:
            f.seek(vert_count * 76, 1)
        idx_count   = _read_i32(f)
        node.index_count = idx_count
        if geometry:
            node.indices = np.frombuffer(f.read(idx_count * 2), dtype="<u2").copy()
        else:
            f.seek(idx_count * 2, 1)
        node.material_id = _read_i32(f)
        f.read(12)

    else:
        # Unknown node type -- can't safely continue (don't know size to skip).
        raise ValueError(f"Unknown KN5 node type {node_type} (node '{name}'). "
                         "File may be a newer KN5 version or CSP-protected.")

    # Recurse children
    for _ in range(child_count):
        node.children.append(_read_node(f, geometry))

    return node


def parse_kn5(path: Path, geometry: bool = True) -> Kn5Model:
    """
    Parse a KN5 file.

    Parameters
    ----------
    path     : Path to the .kn5 file.
    geometry : If False, vertex/index buffers are skipped -- much faster,
               useful when you only need the node-name tree.
    """
    with open(path, "rb") as f:
        magic = f.read(6)
        if magic != b"sc6969":
            raise ValueError(f"Not a valid KN5 file (magic={magic!r}): {path}")

        version = _read_i32(f)
        if version not in (1, 2, 4, 5, 6):
            raise ValueError(
                f"KN5 version {version} is not supported (known: 1,2,4,5,6): {path}"
            )
        if version > 5:
            f.read(4)                       # skip unknown int32 present in v6

        textures  = _read_textures(f)
        materials = _read_materials(f, version)
        root      = _read_node(f, geometry)

    model = Kn5Model(
        name=path.stem,
        version=version,
        textures=textures,
        materials=materials,
        root=root,
    )
    _fold_texture_case(model)
    return model


# --- Node-name scanner (lightweight) ------------------------------------------

def _collect_names(node: Kn5Node, out: list[str]) -> None:
    out.append(node.name)
    for child in node.children:
        _collect_names(child, out)


def scan_kn5_nodes(path: Path) -> list[str]:
    """
    Return all node names from the KN5 tree without loading geometry.
    Very fast -- only reads the structural skeleton of the file.
    """
    model = parse_kn5(path, geometry=False)
    names: list[str] = []
    if model.root:
        _collect_names(model.root, names)
    return names


# --- AC node -> SVJ body-id mapping -------------------------------------------

_AC_TO_SVJ_BODY: list[tuple[list[str], str]] = [
    (["BODY", "CHASSIS", "CAR_BODY", "BODY_HR", "COCKPIT_HR",
      "EXTERIOR", "SHELL", "BODYSHELL"],                          "chassis"),
    (["WHEEL_LF", "SUSP_LF", "WHEEL_FL", "SUSP_FL",
      "UPRIGHT_LF", "HUB_LF"],                                   "upright_fl"),
    (["WHEEL_RF", "SUSP_RF", "WHEEL_FR", "SUSP_FR",
      "UPRIGHT_RF", "HUB_RF"],                                   "upright_fr"),
    (["WHEEL_LR", "SUSP_LR", "WHEEL_RL", "SUSP_RL",
      "UPRIGHT_LR", "HUB_LR"],                                   "upright_rl"),
    (["WHEEL_RR", "SUSP_RR", "UPRIGHT_RR", "HUB_RR"],            "upright_rr"),
]


def map_ac_nodes_to_svj(node_names: list[str]) -> dict[str, str]:
    """
    Map AC node names to SVJ body ids.
    Returns {svj_body_id: ac_node_name} for every SVJ body that was matched.
    """
    upper = {n.upper(): n for n in node_names}
    result: dict[str, str] = {}
    for patterns, svj_id in _AC_TO_SVJ_BODY:
        for pat in patterns:
            if pat in upper:
                result[svj_id] = upper[pat]
                break
    return result


# --- Coordinate transform: AC -> Three.js / glTF ------------------------------
#
#   AC frame    : X-right, Y-up, Z-forward  (left-handed)
#   Three.js    : X-right, Y-up, Z-backward (right-handed, glTF standard)
#   Transform   : three = (ac.x, ac.y, -ac.z)   <- negate Z only
#
# Triangle winding is reversed because handedness flips (left->right-handed).

# ---------------------------------------------------------------------------
# Runtime variants: meshes AC swaps in and out, which a static GLB cannot
# ---------------------------------------------------------------------------
# Exported next to the originals they read as a broken model rather than extra
# detail: damage shells sit a hair off the clean panels and z-fight with them,
# and blurred-wheel discs are opaque and cover the spokes.  They are dropped
# (subtree and all) unless keep_variants=True.
#
# The rules are exact on purpose.  A node is a blur/damage variant when one of
# its name *tokens* is exactly "blur" or "damage" (tokens split on _ - . and
# whitespace, case-insensitive), so RIM_BLUR_LF, "rim blur lf" and
# EXT_Rim_Blur match but UNDAMAGED_PANEL does not.  The earlier substring list
# also caught dent/bent/crash/deform; those are not reliable markers (e.g.
# "bent" matches "BENTLEY_BADGE"), so they are no longer special-cased.
_VARIANT_TOKENS: frozenset[str] = frozenset({"blur", "damage"})
_NAME_SPLIT = re.compile(r"[\s_.\-]+")


def _is_variant_name(name: str) -> bool:
    """True for *_BLUR / *_DAMAGE style runtime-variant meshes."""
    return any(t in _VARIANT_TOKENS for t in _NAME_SPLIT.split(name.lower()))


def lowres_twins(names) -> set[str]:
    """
    Names of the low-res halves of in-file LOD pairs.

    AC ships two versions of the cockpit and steering wheel *inside* the top
    LOD (COCKPIT_HR / COCKPIT_LR, STEER_HR / STEER_LR) and swaps them by
    camera.  Exported together they overlap.  A name is a low-res twin only
    when it ends in ``_LR`` **and** its ``_HR`` partner is also present:
    ``_LR`` is far more often "Left Rear" (WHEEL_LR, SUSP_LR, DISC_LR, ...),
    and none of those have an ``_HR`` twin, so the suspension is never touched.
    """
    have = {n.lower() for n in names}
    return {n for n in names
            if n.lower().endswith("_lr") and n[:-3].lower() + "_hr" in have}


def _walk_names(node: "Kn5Node") -> list[str]:
    out: list[str] = []
    _collect_names(node, out)
    return out


def _ac_to_three(arr: np.ndarray) -> np.ndarray:
    """
    Vectorised AC -> Three.js/glTF space for an (N, 3) float32 array.
    Negates Y and Z: three = (ac.x, -ac.y, -ac.z).
    """
    out = arr.copy()
    out[:, 1] = -arr[:, 1]
    out[:, 2] = -arr[:, 2]
    return out

_ac_to_sae_batch = _ac_to_three  # backward-compat alias


def _mat4_ac_to_three(m: list[float]) -> list[float]:
    """
    Convert a KN5 row-major 4x4 node matrix to a glTF column-major 4x4
    matrix in Three.js/glTF space.
    """
    M_col = np.array(m, dtype="f4").reshape(4, 4).T  # KN5 row-major -> col-vector
    M_three = M_col.copy()
    # P = diag(1, -1, -1, 1)  →  M_three = P * M_col * P
    M_three[1, :]  = -M_col[1, :]
    M_three[2, :]  = -M_col[2, :]
    M_three[:, 1]  = -M_three[:, 1]
    M_three[:, 2]  = -M_three[:, 2]
    # [1,1] [1,2] [2,1] [2,2] are double-negated → back to original (no restore needed)
    return M_three.flatten(order="F").tolist()        # column-major for glTF

_mat4_ac_to_sae = _mat4_ac_to_three  # backward-compat alias


# --- Front-axle Z finder (for mesh origin alignment) -------------------------

def _find_front_axle_z(root: "Kn5Node") -> Optional[float]:
    """
    Walk the KN5 node tree and return the front-axle Z coordinate in AC space.

    Searches for HUB_LF / HUB_RF (or WHEEL_LF / WHEEL_RF as fallback) and
    averages their world-space Z values.  The KN5 matrix is row-major with
    translation in row 3; world accumulation: world = local @ parent_world.

    Returns None if no front-axle node is found.
    """
    FRONT_AXLE_NAMES = {"HUB_LF", "HUB_RF", "WHEEL_LF", "WHEEL_RF"}
    identity = np.eye(4, dtype="f4")
    found_z: list[float] = []

    def _walk(node: "Kn5Node", parent_world: np.ndarray) -> None:
        if node.node_type == 1 and node.matrix:
            local = np.array(node.matrix, dtype="f4").reshape(4, 4)
            # Row-major convention: p_world = p_local @ local @ parent_world
            world = local @ parent_world
        else:
            world = parent_world

        if node.name.upper() in FRONT_AXLE_NAMES:
            # Translation of world matrix: row 3, columns 0-2 -> [tx, ty, tz]
            z_ac = float(world[3, 2])
            found_z.append(z_ac)

        for child in node.children:
            _walk(child, world)

    _walk(root, identity)

    if not found_z:
        return None
    return float(np.mean(found_z))


# --- glTF export --------------------------------------------------------------

def _load_skin_tex(skin_dir: Path, tex_name: str) -> Optional[bytes]:
    """
    Return raw bytes of a texture override found in a skin folder, or None.
    Matches by stem (case-insensitive), accepts .dds / .png / .jpg / .jpeg.
    """
    stem = Path(tex_name).stem.lower()
    try:
        for f in skin_dir.iterdir():
            if (f.stem.lower() == stem and
                    f.suffix.lower() in (".dds", ".png", ".jpg", ".jpeg")):
                return f.read_bytes()
    except OSError:
        pass
    return None


def _is_uvmult_shader(shader: str) -> bool:
    """
    True for the Kunos ksPerPixel*_UVMult shader family.

    In these shaders the ``diffuseMult`` (and ``normalMult``) material property
    is NOT a colour-brightness multiplier as in every other shader — it is a
    UV *tiling* factor applied to the main textures.  e.g. a carbon-weave
    diffuse with diffuseMult=15 tiles 15× across the mesh's UVs.  Generic
    converters that treat diffuseMult as brightness (or ignore it on textured
    materials) render the pattern 15× too large; Content Manager's showroom
    runs the real shader and tiles correctly.  A value of 0 means "no multiply"
    (i.e. 1×) for unique-mapped textures like instrument panels or AO bakes.
    """
    return "uvmult" in shader.lower()


def _ci_child(parent: Path, name: str) -> Optional[Path]:
    """
    ``parent / name`` matched case-insensitively (AC is a Windows game, so a car
    folder may say ``Skins`` or ``Car.KN5`` and still load there; on Linux and
    macOS a literal path would miss).  Returns the entry as it is actually
    spelled on disk (an exact-case match wins if several differ only in case) --
    on a case-insensitive filesystem ``exists()`` would echo *our* spelling back.
    """
    try:
        want = name.lower()
        found: Optional[Path] = None
        for child in parent.iterdir():
            if child.name == name:
                return child
            if found is None and child.name.lower() == want:
                found = child
        return found
    except OSError:
        return None


def _find_skins(car_path: Path) -> list[tuple[str, Path]]:
    """
    Return [(skin_name, skin_folder), ...] sorted alphabetically from
    <car_path>/skins/.  Returns an empty list if no skins directory exists.
    """
    skins_dir = _ci_child(Path(car_path), "skins")
    if skins_dir is None or not skins_dir.is_dir():
        return []
    return sorted(
        [(d.name, d) for d in skins_dir.iterdir() if d.is_dir()],
        key=lambda x: x[0].lower(),
    )


# --- Livery paint -------------------------------------------------------------
#
# A livery in Assetto Corsa lives *outside* the model.  The KN5 carries whichever
# skin was loaded at export time (often a grey template) and the game swaps in
# same-named files from skins/<name>/ at load.  On many Kunos road cars the
# body diffuse is a shared grey panel/AO sheet, and the paint is a *flat colour*
# in the material's txDetail map (useDetail > 0), which AC's shader multiplies
# into the diffuse as ``diffuse * detail * 2``.  So the "paint" of a skin is one
# texel of its detail map.  (Approach first described in semiloker's
# assetto-corsa-gltf; reimplemented here.)

# Names a car is painted *beside*: a rim is a painted material on most Kunos
# cars and often has more triangles than the bodywork, so size alone picks the
# wheels.  Name hints first, triangle count inside a band.
_PAINT_HINT = re.compile(r"car[_ ]?paint|(?:^|[^a-z])body(?:[^a-z]|$)|chassis", re.I)
_NOT_BODY = re.compile(
    r"rim|wheel|tyre|tire|brake|calip|disc|glass|window|light|lamp|badge|logo|"
    r"plate|mirror|seat|interior|cockpit|dash|carpet|leather|belt|driver|steer|"
    r"engine|exhaust|grill|plastic|chrome|rubber|shadow", re.I)


def paint_rank(name: str) -> tuple[int, int]:
    """Sort key band for a painted material: lower = more likely the bodywork."""
    low = name.lower()
    inside = 1 if low.startswith("int") else 0     # interior copy of the paint
    if _NOT_BODY.search(low):
        return (2, inside)
    if _PAINT_HINT.search(low):
        return (0, inside)
    return (1, inside)


def flat_colour(blob: Optional[bytes]) -> Optional[tuple[list[int], list[float]]]:
    """
    ``(srgb_bytes, linear_factor)`` if the texture is ONE colour, else None.

    srgb_bytes is the texture's own 8-bit colour (what a swatch should show);
    linear_factor is the glTF baseColorFactor equivalent of AC's
    ``diffuse * detail * 2``: doubled in gamma space, clamped, *then*
    linearised (the order matters).  Extrema are taken over the full image so a
    grainy/patterned map is never mistaken for a flat tint.
    """
    if not blob or len(blob) < 128:
        return None
    try:
        from PIL import Image
        Image.MAX_IMAGE_PIXELS = None
        im = Image.open(io.BytesIO(blob))
        im.load()
        im = im.convert("RGB")
    except Exception:                                    # noqa: BLE001
        return None
    ext = im.getextrema()
    if max(hi - lo for lo, hi in ext) > 6:               # varies: a pattern
        return None
    srgb: list[int] = []
    lin: list[float] = []
    for lo, hi in ext:
        mid = (lo + hi) * 0.5 / 255.0
        srgb.append(int(round(mid * 255.0)))
        c = min(2.0 * mid, 1.0)
        lin.append(round(c / 12.92 if c <= 0.04045
                         else ((c + 0.055) / 1.055) ** 2.4, 5))
    return srgb, lin


def detail_tint(blob: Optional[bytes]) -> Optional[list[float]]:
    """Linear baseColorFactor RGB for a flat txDetail map, else None."""
    fc = flat_colour(blob)
    return fc[1] if fc else None


def paint_slots(materials: list["Kn5Material"],
                tris: dict[int, int]) -> list[tuple[int, str, str]]:
    """
    ``(index, name, txDetail)`` for every material that has an *active* paint
    slot, bodywork first.  useDetail is AC's own switch: without it the shader
    never samples txDetail, so it is not a colour of anything.
    """
    out = [(i, m.name, m.tx_detail) for i, m in enumerate(materials)
           if m.tx_detail and m.use_detail]
    out.sort(key=lambda t: paint_rank(t[1]) + (-tris.get(t[0], 0),))
    return out


def _skin_dirs(kn5_path: Path) -> list[tuple[str, Path]]:
    return _find_skins(Path(kn5_path).parent)


def list_skins(kn5_path: Path) -> dict:
    """
    Every livery a car ships and the colour each one paints it.

    Returns ``{"car", "skins": [{"name", "colours": [{"material", "texture",
    "rgb", "tris", "source"}]}]}``.  Within a skin, colours are ordered
    bodywork-first, so ``colours[0]`` is "the car's colour".  A final entry
    named ``none`` lists what the KN5's own embedded textures paint.  A skin
    whose paint is a pattern rather than a flat colour has an empty list.
    Geometry is not loaded (triangle counts come from index counts only).
    """
    kn5_path = Path(kn5_path)
    out: dict = {"car": kn5_path.stem, "skins": []}
    try:
        model = parse_kn5(kn5_path, geometry=False)
    except ValueError as e:
        out["error"] = str(e)
        return out

    tris: dict[int, int] = {}

    def _walk(n: Kn5Node) -> None:
        if n.node_type in (2, 3) and n.material_id >= 0:
            tris[n.material_id] = tris.get(n.material_id, 0) + n.index_count // 3
        for c in n.children:
            _walk(c)
    if model.root:
        _walk(model.root)

    slots = paint_slots(model.materials, tris)
    own = {t.name: t.data for t in model.textures}
    cache: dict = {}

    def colour(key, blob):
        if key not in cache:
            cache[key] = flat_colour(blob)
        return cache[key]

    entries: list[tuple[str, Optional[Path]]] = list(_skin_dirs(kn5_path))
    entries.append(("none", None))
    for sname, sdir in entries:
        entry = {"name": sname, "colours": []}
        for mi, mname, dds in slots:
            raw = _load_skin_tex(sdir, dds) if sdir is not None else None
            if raw is not None:
                c, src = colour((sname, dds), raw), "skin"
            else:
                c, src = colour(("kn5", dds), own.get(dds)), "kn5"
            if c is None:
                continue
            entry["colours"].append({"material": mname, "texture": dds,
                                     "rgb": c[0], "tris": tris.get(mi, 0),
                                     "source": src})
        out["skins"].append(entry)
    return out


def format_skin_report(report: dict) -> str:
    """list_skins() as tab-separated lines: skin, material, RRGGBB, tris, source."""
    lines = [f"# car\t{report['car']}"]
    if report.get("error"):
        lines.append(f"# error\t{report['error']}")
        return "\n".join(lines)
    for sk in report["skins"]:
        if not sk["colours"]:
            lines.append(f"skin\t{sk['name']}\t-\t-\t0\t-")
            continue
        for c in sk["colours"]:
            r, g, b = c["rgb"]
            lines.append(f"skin\t{sk['name']}\t{c['material']}\t"
                         f"{r:02X}{g:02X}{b:02X}\t{c['tris']}\t{c['source']}")
    return "\n".join(lines)


def blinn_to_roughness(exponent: float, intensity: float = 1.0) -> float:
    """
    AC's Blinn-Phong specular -> glTF roughness.

    ``roughness = sqrt(2 / (exp + 2))`` is the standard Blinn-exponent to
    GGX-alpha mapping.  The intensity (ksSpecular) scales the peak of the lobe,
    which is what the exponent controls too, so it is folded in as
    ``exp * intensity`` (floored at 0.02): intensity 1 changes nothing, intensity
    0 makes the surface matte.  Clamped to [0.04, 1].
    """
    eff = max(exponent, 1.0) * max(intensity, 0.02)
    return min(max((2.0 / (eff + 2.0)) ** 0.5, 0.04), 1.0)


def kn5_to_glb(
    path: Path,
    output_path: Optional[Path] = None,
    embed_textures: bool = True,
    skins: Optional[list[tuple[str, Path]]] = None,
    keep_variants: bool = False,
    default_skin: Optional[str] = "first",
) -> bytes:
    """
    Convert a KN5 file to a self-contained GLB (binary glTF).

    Parameters
    ----------
    path           : Source .kn5 file.
    output_path    : If given, the GLB bytes are also written here.
    embed_textures : Embed texture images in the GLB (default True).
    keep_variants  : Keep runtime-variant meshes (*_BLUR, *_DAMAGE and the
                     low-res ``_LR`` half of in-file ``_HR``/``_LR`` pairs).
                     Default False drops them, subtree and all.
    skins          : Optional list of (skin_name, skin_folder) tuples from
                     _find_skins().  When provided, each skin becomes a named
                     variant via the KHR_materials_variants extension.
    default_skin   : Skin the base materials are resolved against: "first"
                     (default; what AC itself loads), a skin name, or "none"
                     to keep the KN5's embedded (template) textures.

    Returns
    -------
    bytes  : Raw GLB content.
    """
    try:
        import pygltflib
    except ImportError:
        raise ImportError(
            "pygltflib is required for KN5->glTF export. "
            "Run: pip install pygltflib"
        )

    model = parse_kn5(path, geometry=True)

    gltf = pygltflib.GLTF2()
    gltf.asset = pygltflib.Asset(generator="ac_to_svj kn5_reader", version="2.0")

    bin_data = bytearray()

    def _add_buffer_view(data: bytes, target: int = 0) -> int:
        offset = len(bin_data)
        bin_data.extend(data)
        # Pad to 4-byte alignment
        if len(bin_data) % 4:
            bin_data.extend(b"\x00" * (4 - len(bin_data) % 4))
        bv = pygltflib.BufferView(buffer=0, byteOffset=offset, byteLength=len(data),
                                   target=target)
        gltf.bufferViews.append(bv)
        return len(gltf.bufferViews) - 1

    def _add_accessor(bv_idx: int, comp_type: int, acc_type: str,
                      count: int, min_vals=None, max_vals=None) -> int:
        acc = pygltflib.Accessor(
            bufferView=bv_idx, componentType=comp_type, type=acc_type,
            count=count, byteOffset=0,
            min=min_vals, max=max_vals,
        )
        gltf.accessors.append(acc)
        return len(gltf.accessors) - 1

    def _embed_raw(raw: bytes, name: str) -> int:
        """Convert raw image bytes (DDS/PNG/JPEG) and add to gltf. Returns glTF texture index or -1."""
        img_data = raw
        mime = "image/png"
        if raw[:4] == b"DDS ":
            try:
                from PIL import Image as PILImage
                # AC textures can be very large (e.g. 8192×12288 = 100 M px).
                # PIL's decompression-bomb guard is intended for untrusted web
                # images; local KN5 textures are trusted, so disable the limit.
                PILImage.MAX_IMAGE_PIXELS = None
                buf = io.BytesIO(raw)
                pil_img = PILImage.open(buf)
                if "A" in pil_img.mode:
                    a = np.asarray(pil_img)[:, :, 3].ravel()
                    extreme = float((a <= 10).sum() + (a >= 245).sum())
                    tex_bimodal[name] = extreme / len(a) > 0.85
                out_buf = io.BytesIO()
                pil_img.save(out_buf, format="PNG")
                img_data = out_buf.getvalue()
            except Exception:
                return -1
        elif raw[:3] == b"\xff\xd8\xff":
            mime = "image/jpeg"
        bv_idx = _add_buffer_view(img_data)
        gltf.images.append(pygltflib.Image(bufferView=bv_idx, mimeType=mime, name=name))
        gltf.samplers.append(pygltflib.Sampler(
            magFilter=pygltflib.LINEAR,
            minFilter=pygltflib.LINEAR_MIPMAP_LINEAR,
            wrapS=pygltflib.REPEAT,
            wrapT=pygltflib.REPEAT,
        ))
        gltf.textures.append(pygltflib.Texture(
            source=len(gltf.images) - 1,
            sampler=len(gltf.samplers) - 1,
            name=name,
        ))
        return len(gltf.textures) - 1

    # -- Build texture images --------------------------------------------------
    tex_idx:      dict[str, int]  = {}   # texture name -> glTF texture index
    tex_bimodal:  dict[str, bool] = {}   # texture name -> True if alpha is bimodal
    #   A bimodal alpha means pixels are almost entirely near-0 or near-255 —
    #   i.e. it is a cutout mask rather than a continuous transparency map.
    #   Used below to promote blend_mode==1 materials to MASK when appropriate.
    if embed_textures:
        for kn5_tex in model.textures:
            if not kn5_tex.data:
                continue
            tidx = _embed_raw(kn5_tex.data, kn5_tex.name)
            if tidx >= 0:
                tex_idx[kn5_tex.name] = tidx

    ext_used: set[str] = set()

    # -- Livery / paint resolution ---------------------------------------------
    # The game replaces KN5 textures by name with files from skins/<name>/.  The
    # base materials are resolved against AC's own default skin (the first one)
    # so the GLB opens in a real livery rather than the grey export template;
    # every skin is then a KHR_materials_variants entry.
    kn5_blobs = {t.name: t.data for t in model.textures}
    skin_diffuse: dict[tuple, int] = {}              # (skin, tex) -> gltf tex idx
    skin_tint: dict[tuple, Optional[list]] = {}      # (skin, detail tex) -> tint

    base_skin: Optional[tuple[str, Path]] = None
    if embed_textures and skins and default_skin != "none":
        if default_skin in (None, "first"):
            base_skin = skins[0]
        else:
            base_skin = next((s for s in skins
                              if s[0].lower() == default_skin.lower()), None)
            if base_skin is None:
                raise ValueError(
                    f"No skin {default_skin!r}; have: "
                    + ", ".join(n for n, _ in skins))

    def _resolve_paint(kn5_mat: Kn5Material, skin):
        """-> (diffuse gltf tex idx | None, alpha key, flat-detail tint | None)"""
        tx = kn5_mat.tx_diffuse
        d_idx = tex_idx.get(tx) if tx else None
        d_key = tx
        sname, sdir = skin if skin else (None, None)
        if sdir is not None and tx:
            k = (sname, tx)
            if k not in skin_diffuse:
                raw = _load_skin_tex(sdir, tx)
                skin_diffuse[k] = (_embed_raw(raw, f"{sname}/{tx}")
                                   if raw is not None else -1)
            if skin_diffuse[k] >= 0:
                d_idx, d_key = skin_diffuse[k], f"{sname}/{tx}"
        tint = None
        td = kn5_mat.tx_detail
        if td and kn5_mat.use_detail:
            k = (sname, td)
            if k not in skin_tint:
                raw = _load_skin_tex(sdir, td) if sdir is not None else None
                skin_tint[k] = detail_tint(raw if raw is not None
                                           else kn5_blobs.get(td))
            tint = skin_tint[k]
        return d_idx, d_key, tint

    def _make_gmat(kn5_mat: Kn5Material, name: str, d_idx, d_key, tint):
        has_tex = d_idx is not None
        if has_tex:
            # A flat txDetail is the paint colour: AC multiplies it into the
            # (grey) diffuse, which is exactly what baseColorFactor does.
            base_factor = (list(tint) + [1.0]) if tint else [1.0, 1.0, 1.0, 1.0]
        else:
            # diffuseMult is a brightness factor for normal shaders, but a UV
            # tiling factor for _UVMult shaders - never apply it as brightness there.
            bright_mult = 1.0 if _is_uvmult_shader(kn5_mat.shader) else kn5_mat.diffuse_mult
            v = max(0.05, kn5_mat.ks_diffuse * bright_mult)
            base_factor = [v, v, v, 1.0]
        pbr = pygltflib.PbrMetallicRoughness(
            baseColorFactor=base_factor,
            metallicFactor=0.0,
            roughnessFactor=round(blinn_to_roughness(
                kn5_mat.ks_specular_exp, kn5_mat.ks_specular), 4),
        )
        if has_tex:
            pbr.baseColorTexture = pygltflib.TextureInfo(index=d_idx)
        gmat = pygltflib.Material(name=name, pbrMetallicRoughness=pbr)
        if embed_textures and kn5_mat.tx_normal and kn5_mat.tx_normal in tex_idx:
            gmat.normalTexture = pygltflib.NormalMaterialTexture(
                index=tex_idx[kn5_mat.tx_normal]
            )
        # Detail texture - exported as occlusionTexture referencing TEXCOORD_1.
        # Three.js aoMap requires UV1 (vUv2 in the shader); texCoord=0 is silently
        # ignored.  The mesh-building step pre-bakes scaled UVs into TEXCOORD_1
        # (uv x detailUVMultiplier), so no KHR_texture_transform extension is needed.
        # Skipped when the detail map was a flat colour: that is the paint tint
        # above, and as an AO map it would only darken the surface uniformly.
        if (embed_textures
                and tint is None
                and kn5_mat.tx_detail
                and kn5_mat.tx_detail in tex_idx
                and (kn5_mat.use_detail or kn5_mat.detail_uv_mult > 1.0)):
            gmat.occlusionTexture = pygltflib.OcclusionTextureInfo(
                index=tex_idx[kn5_mat.tx_detail],
                texCoord=1,       # Three.js aoMap samples from UV1 (TEXCOORD_1)
                strength=1.0,
            )
        # fresnelMaxLevel is AC's cap on how much a surface may reflect (black
        # trim and glass set ~0.2, paint ~0.6).  glTF's dielectric Fresnel has no
        # such knob; KHR_materials_specular's specularFactor is exactly it.
        if kn5_mat.fresnel_max > 0.0:
            gmat.extensions = {"KHR_materials_specular": {
                "specularFactor": round(min(kn5_mat.fresnel_max, 1.0), 4)}}
            ext_used.add("KHR_materials_specular")
        # Car paint: AC adds a second, tighter lobe (sunSpecular / sunSpecularEXP)
        # for the clear lacquer over the base coat.  Only paint sets it, so its
        # presence is the author's own marker for "this panel is painted".
        if kn5_mat.sun_specular > 0.0:
            sx = max(kn5_mat.sun_specular_exp, 1.0)
            # sunSpecular is an intensity in AC units, not a coverage; /20 puts
            # typical values (12..40) either side of the range the extension wants.
            gmat.extensions = gmat.extensions or {}
            gmat.extensions["KHR_materials_clearcoat"] = {
                "clearcoatFactor": round(min(kn5_mat.sun_specular / 20.0, 1.0), 4),
                "clearcoatRoughnessFactor": round(
                    min(max((2.0 / (sx + 2.0)) ** 0.5, 0.02), 1.0), 4)}
            ext_used.add("KHR_materials_clearcoat")
        # Map KN5 blend mode to glTF alphaMode:
        #   0   -> OPAQUE  (ignore alpha channel)
        #   256 -> MASK    (explicit alpha-test flag: belts, grilles, plates)
        #   1   -> BLEND or MASK depending on texture alpha distribution.
        #          Some car mods use blend=1 for both true semi-transparent
        #          surfaces (glass, lens) and hard-edge cutouts (grille, decals).
        #          We resolve the ambiguity by inspecting the diffuse texture:
        #          if alpha is bimodal (>=85 % of pixels near 0 or near 255)
        #          the surface is a cutout -> MASK; otherwise -> BLEND.
        if kn5_mat.blend_mode == 256:
            gmat.alphaMode = "MASK"
            gmat.alphaCutoff = 0.5
        elif kn5_mat.blend_mode == 1:
            if d_key and tex_bimodal.get(d_key, False):
                gmat.alphaMode = "MASK"
                gmat.alphaCutoff = 0.5
            else:
                gmat.alphaMode = "BLEND"
        else:
            gmat.alphaMode = "OPAQUE"
        return gmat

    # -- Build materials -------------------------------------------------------
    mat_gltf_idx: list[int] = []
    base_paint: list[tuple] = []
    for kn5_mat in model.materials:
        res = _resolve_paint(kn5_mat, base_skin)
        base_paint.append(res)
        gltf.materials.append(_make_gmat(kn5_mat, kn5_mat.name, *res))
        mat_gltf_idx.append(len(gltf.materials) - 1)

    # Runtime variants (blur / damage / in-file low-res twins) are dropped
    # outright rather than hidden behind a transparent material.
    dropped_variants = 0
    lowres: set[str] = (set() if keep_variants or model.root is None
                        else lowres_twins(_walk_names(model.root)))

    # -- Build skin variants (KHR_materials_variants) --------------------------
    # variant_names[0] = "Default" (base materials: the default skin when one is
    # applied, else the KN5's embedded textures), [1..] = skin names.
    # mat_variants[kn5_mat_id][variant_idx] = glTF material index.
    variant_names: list[str] = []
    mat_variants: list[list[int]] = [[idx] for idx in mat_gltf_idx]

    if embed_textures and skins:
        variant_names = ["Default"] + [sn for sn, _ in skins]

        for skin in skins:
            for kn5_mid, kn5_mat in enumerate(model.materials):
                res = _resolve_paint(kn5_mat, skin)
                base = base_paint[kn5_mid]
                if (res[0], res[2]) == (base[0], base[2]):
                    mat_variants[kn5_mid].append(mat_gltf_idx[kn5_mid])  # same look
                else:
                    gltf.materials.append(
                        _make_gmat(kn5_mat, f"{kn5_mat.name}_{skin[0]}", *res))
                    mat_variants[kn5_mid].append(len(gltf.materials) - 1)

        ext_used.add("KHR_materials_variants")
        gltf.extensions = {
            "KHR_materials_variants": {
                "variants": [{"name": n} for n in variant_names]
            }
        }

    # -- Build scene nodes (DFS, mirrors KN5 tree) -----------------------------
    scene = pygltflib.Scene(name=model.name, nodes=[])
    gltf.scenes.append(scene)
    gltf.scene = 0

    def _process_node(kn5_node: Kn5Node,
                      parent_gltf_idx: Optional[int]) -> Optional[int]:
        nonlocal dropped_variants
        if not keep_variants and (_is_variant_name(kn5_node.name)
                                  or kn5_node.name in lowres):
            dropped_variants += 1       # subtree goes with it
            return None
        gnode = pygltflib.Node(name=kn5_node.name)

        if kn5_node.node_type == 1 and kn5_node.matrix:
            gnode.matrix = _mat4_ac_to_three(kn5_node.matrix)

        elif kn5_node.node_type in (2, 3) and kn5_node.positions is not None:
            # AC (left-handed) -> Three.js/glTF (right-handed): negate Z.
            pos = _ac_to_three(kn5_node.positions)
            nrm = _ac_to_three(kn5_node.normals)
            uvs = kn5_node.uvs

            # Tangents: apply same Y+Z negate as positions/normals.
            # W sign is kept as-is (no winding reversal, so no handedness compensation needed).
            tan_raw = kn5_node.tangents          # (N,4)
            tan_xyz = _ac_to_three(tan_raw[:, :3])
            tan_w   = tan_raw[:, 3:4]            # unchanged
            tan = np.concatenate([tan_xyz, tan_w], axis=1).astype("<f4")

            # Winding unchanged: double axis negation (Y+Z) preserves handedness.
            idx = kn5_node.indices

            # Accessors
            pos_bytes = pos.astype("<f4").tobytes()
            bv_pos = _add_buffer_view(pos_bytes, target=pygltflib.ARRAY_BUFFER)
            mn = pos.min(axis=0).tolist()
            mx = pos.max(axis=0).tolist()
            acc_pos = _add_accessor(bv_pos, pygltflib.FLOAT, "VEC3",
                                    len(pos), mn, mx)

            nrm_bytes = nrm.astype("<f4").tobytes()
            bv_nrm = _add_buffer_view(nrm_bytes, target=pygltflib.ARRAY_BUFFER)
            acc_nrm = _add_accessor(bv_nrm, pygltflib.FLOAT, "VEC3", len(nrm))

            tan_bytes = tan.tobytes()
            bv_tan = _add_buffer_view(tan_bytes, target=pygltflib.ARRAY_BUFFER)
            acc_tan = _add_accessor(bv_tan, pygltflib.FLOAT, "VEC4", len(tan))

            # TEXCOORD_0: base UVs. For _UVMult shaders, Kunos applies diffuseMult
            # as a UV *tiling* factor on the main textures (e.g. carbon-weave at
            # 15×, mesh grille at 40×).  Bake that scale in so the pattern tiles
            # at the correct frequency instead of appearing N× too large.
            mid = kn5_node.material_id
            uv0 = uvs
            if 0 <= mid < len(model.materials):
                raw_mat = model.materials[mid]
                if _is_uvmult_shader(raw_mat.shader) and raw_mat.diffuse_mult > 1.0:
                    uv0 = uvs * float(raw_mat.diffuse_mult)

            uv_bytes = uv0.astype("<f4").tobytes()
            bv_uv = _add_buffer_view(uv_bytes, target=pygltflib.ARRAY_BUFFER)
            acc_uv = _add_accessor(bv_uv, pygltflib.FLOAT, "VEC2", len(uv0))

            # TEXCOORD_1: pre-scaled UVs for detail texture (Three.js aoMap uses UV1).
            # detailUVMultiplier is read from the material and baked here so no
            # KHR_texture_transform extension is needed.  Always computed from the
            # ORIGINAL uvs, independent of any _UVMult diffuse scaling above.
            acc_uv1 = None
            if embed_textures and 0 <= mid < len(model.materials):
                raw_mat = model.materials[mid]
                if (raw_mat.tx_detail
                        and raw_mat.tx_detail in tex_idx
                        and (raw_mat.use_detail or raw_mat.detail_uv_mult > 1.0)):
                    uv_det = (uvs * float(raw_mat.detail_uv_mult)).astype("<f4")
                    bv_uv1 = _add_buffer_view(uv_det.tobytes(),
                                              target=pygltflib.ARRAY_BUFFER)
                    acc_uv1 = _add_accessor(bv_uv1, pygltflib.FLOAT, "VEC2",
                                            len(uv_det))

            # KN5 indices are uint16, but glTF reserves 65535 (primitive restart)
            # for UNSIGNED_SHORT, so a mesh that uses index 65535 (i.e. has more
            # than 65535 vertices) must be written as UNSIGNED_INT.
            if len(pos) > 65535 or (len(idx) and int(idx.max()) >= 65535):
                idx_bytes = idx.astype("<u4").tobytes()
                idx_type = pygltflib.UNSIGNED_INT
            else:
                idx_bytes = idx.astype("<u2").tobytes()
                idx_type = pygltflib.UNSIGNED_SHORT
            bv_idx = _add_buffer_view(idx_bytes, target=pygltflib.ELEMENT_ARRAY_BUFFER)
            acc_idx = _add_accessor(bv_idx, idx_type, "SCALAR", len(idx))

            mat_idx = (mat_gltf_idx[kn5_node.material_id]
                       if 0 <= kn5_node.material_id < len(mat_gltf_idx) else None)

            primitive = pygltflib.Primitive(
                attributes=pygltflib.Attributes(
                    POSITION=acc_pos,
                    NORMAL=acc_nrm,
                    TANGENT=acc_tan,
                    TEXCOORD_0=acc_uv,
                    TEXCOORD_1=acc_uv1,   # None for meshes without detail texture
                ),
                indices=acc_idx,
                material=mat_idx,
            )
            # Attach skin variant mappings when skins were supplied and this
            # mesh's material actually differs across variants.
            kn5_mid = kn5_node.material_id
            if (variant_names and mat_idx is not None
                    and 0 <= kn5_mid < len(mat_variants)):
                v_list = mat_variants[kn5_mid]
                if not all(v == v_list[0] for v in v_list):
                    mat_to_vis: dict[int, list[int]] = {}
                    for vi, mi in enumerate(v_list):
                        mat_to_vis.setdefault(mi, []).append(vi)
                    primitive.extensions = {
                        "KHR_materials_variants": {
                            "mappings": [
                                {"material": mi, "variants": vis}
                                for mi, vis in mat_to_vis.items()
                            ]
                        }
                    }
            mesh = pygltflib.Mesh(name=kn5_node.name, primitives=[primitive])
            gltf.meshes.append(mesh)
            gnode.mesh = len(gltf.meshes) - 1

        gltf.nodes.append(gnode)
        this_idx = len(gltf.nodes) - 1

        if parent_gltf_idx is None:
            scene.nodes.append(this_idx)
        else:
            if gltf.nodes[parent_gltf_idx].children is None:
                gltf.nodes[parent_gltf_idx].children = []
            gltf.nodes[parent_gltf_idx].children.append(this_idx)

        for child in kn5_node.children:
            _process_node(child, this_idx)

        return this_idx

    # -- Orientation + front-axle alignment ------------------------------------
    if model.root:
        # 180° around Z. Quaternion [x, y, z, w] = [0, 0, 1, 0].
        rot_node = pygltflib.Node(
            name="_ac_z_rotation",
            rotation=[0.0, 0.0, 1.0, 0.0],
        )
        gltf.nodes.append(rot_node)
        rot_idx = len(gltf.nodes) - 1
        scene.nodes.append(rot_idx)

        front_axle_z = _find_front_axle_z(model.root)
        if front_axle_z is not None and abs(front_axle_z) > 0.01:
            wrapper = pygltflib.Node(
                name="_ac_front_axle_align",
                translation=[0.0, 0.0, float(front_axle_z)],
            )
            gltf.nodes.append(wrapper)
            wrapper_idx = len(gltf.nodes) - 1
            if gltf.nodes[rot_idx].children is None:
                gltf.nodes[rot_idx].children = []
            gltf.nodes[rot_idx].children.append(wrapper_idx)
            _process_node(model.root, wrapper_idx)
        else:
            _process_node(model.root, rot_idx)

    if ext_used:
        gltf.extensionsUsed = sorted(ext_used)

    # -- Finalise buffer -------------------------------------------------------
    gltf.buffers.append(pygltflib.Buffer(byteLength=len(bin_data)))
    gltf.set_binary_blob(bytes(bin_data))
    glb_bytes = b"".join(gltf.save_to_bytes())
    if output_path:
        output_path.write_bytes(glb_bytes)
    return glb_bytes


# --- Convenience: find the main KN5 / all LODs in an AC car folder -----------

_EFFECT_PREFIXES = ("3d", "smoke", "particle", "collider", "blur_", "tyre_")


def _kn5_files(car_path: Path) -> list[Path]:
    """All .kn5 files in a car folder (extension matched case-insensitively)."""
    try:
        return [p for p in Path(car_path).iterdir()
                if p.is_file() and p.suffix.lower() == ".kn5"]
    except OSError:
        return []


def find_car_kn5(car_path: Path) -> Optional[Path]:
    """
    Return the primary (LOD A) KN5 file for an AC car folder.
    Falls back to largest non-effect KN5 if the stem-named file is absent.
    """
    car_path = Path(car_path)
    named = _ci_child(car_path, f"{car_path.name}.kn5")
    if named is not None and named.is_file():
        return named

    all_kn5 = _kn5_files(car_path)
    mesh_kn5 = [p for p in all_kn5
                if not p.stem.lower().startswith(_EFFECT_PREFIXES)]
    candidates = mesh_kn5 if mesh_kn5 else all_kn5
    if not candidates:
        return None
    return max(candidates, key=lambda p: p.stat().st_size)


def find_car_kn5_lods(car_path: Path) -> list[tuple[str, Path]]:
    """
    Return all LOD KN5 files for an AC car folder, in order.

    Returns a list of (lod_label, path) pairs, e.g.:
        [("A", PosixPath(".../car.kn5")),
         ("B", PosixPath(".../car_LOD_B.kn5")),
         ("C", PosixPath(".../car_LOD_C.kn5"))]

    LOD A is the main file (stem == folder name).
    LOD B-D are named  <stem>_LOD_B.kn5 / _LOD_C.kn5 / _LOD_D.kn5.
    File names are matched case-insensitively.
    Returns an empty list when no KN5 is found at all.
    """
    car_path = Path(car_path)
    stem = car_path.name          # e.g. "bo_caterham_165_lhd"
    result: list[tuple[str, Path]] = []

    lod_a = _ci_child(car_path, f"{stem}.kn5")
    if lod_a is not None and lod_a.is_file():
        result.append(("A", lod_a))
    else:
        # Fallback: largest non-effect KN5 that doesn't look like a LOD file
        all_kn5 = [p for p in _kn5_files(car_path)
                   if not p.stem.lower().startswith(_EFFECT_PREFIXES)
                   and "_lod_" not in p.stem.lower()]
        if all_kn5:
            result.append(("A", max(all_kn5, key=lambda p: p.stat().st_size)))

    for label in ("B", "C", "D"):
        lod_path = _ci_child(car_path, f"{stem}_LOD_{label}.kn5")
        if lod_path is not None and lod_path.is_file():
            result.append((label, lod_path))

    return result


def kn5_all_lods_to_glbs(
    car_path: Path,
    output_dir: Path,
    embed_textures: bool = True,
    include_skins: bool = True,
    keep_variants: bool = False,
    default_skin: Optional[str] = "first",
) -> dict[str, Path]:
    """
    Export one GLB per LOD found in an AC car folder.

    Output filenames mirror the KN5 names:
        LOD A  ->  <output_dir>/<stem>.glb
        LOD B  ->  <output_dir>/<stem>_LOD_B.glb
        LOD C  ->  <output_dir>/<stem>_LOD_C.glb

    Parameters
    ----------
    car_path      : AC car folder.
    output_dir    : Directory where GLBs are written (created if needed).
    embed_textures: Passed through to kn5_to_glb() for each LOD.
    include_skins : When True (default), embed all skin liveries as
                    KHR_materials_variants inside the GLB.
    keep_variants : Keep *_BLUR / *_DAMAGE / in-file low-res meshes.
    default_skin  : Skin the base materials use ("first", a name, or "none").

    Returns
    -------
    dict  : {lod_label: output_path}  for every LOD that was exported.
    """
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    lods = find_car_kn5_lods(car_path)
    skins = _find_skins(car_path) if include_skins else []
    results: dict[str, Path] = {}
    for label, kn5_path in lods:
        out_path = output_dir / f"{kn5_path.stem}.glb"
        kn5_to_glb(kn5_path, output_path=out_path,
                   embed_textures=embed_textures,
                   skins=skins if skins else None,
                   keep_variants=keep_variants,
                   default_skin=default_skin)
        results[label] = out_path
    return results


# --- CLI entry point ----------------------------------------------------------

def _main(argv: Optional[list[str]] = None) -> int:
    import argparse
    ap = argparse.ArgumentParser(
        prog="kn5_reader.py",
        description="Convert an Assetto Corsa .kn5 to GLB, or list a car's skins.")
    ap.add_argument("kn5", type=Path, help="car .kn5 file")
    ap.add_argument("output", type=Path, nargs="?", help="output .glb (default: beside the kn5)")
    ap.add_argument("--keep-variants", action="store_true",
                    help="keep *_BLUR, *_DAMAGE and in-file low-res (_LR) meshes")
    ap.add_argument("--list-skins", action="store_true",
                    help="write nothing; print each skin and the colour it paints "
                         "(tab-separated: skin, material, RRGGBB, tris, source)")
    ap.add_argument("--skin", default="first", metavar="NAME",
                    help="skin for the base materials: first (default), a skin "
                         "name, or none (KN5 embedded textures). All skins are "
                         "still embedded as KHR_materials_variants.")
    ap.add_argument("--no-skins", action="store_true",
                    help="do not embed skins as variants")
    ap.add_argument("--scan-nodes", action="store_true",
                    help="also print the node names and the SVJ body binding map")
    a = ap.parse_args(argv)

    if a.list_skins:
        print(format_skin_report(list_skins(a.kn5)))
        return 0

    if a.scan_nodes:
        names = scan_kn5_nodes(a.kn5)
        print(f"{len(names)} nodes found:")
        for n in names:
            print(f"  {n}")
        print("SVJ visual binding map:")
        for svj_id, ac_name in map_ac_nodes_to_svj(names).items():
            print(f"  {svj_id:20s} <- {ac_name}")

    dst = a.output or a.kn5.with_suffix(".glb")
    skins = [] if a.no_skins else _skin_dirs(a.kn5)
    print(f"Converting {a.kn5.name} ...")
    try:
        glb = kn5_to_glb(a.kn5, output_path=dst, skins=skins or None,
                         keep_variants=a.keep_variants, default_skin=a.skin)
    except ValueError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    print(f"  Written {len(glb):,} bytes -> {dst}")
    return 0


if __name__ == "__main__":
    sys.exit(_main())
