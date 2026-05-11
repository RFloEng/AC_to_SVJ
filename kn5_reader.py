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

Transform applied: negate Z  ->  three = (ac.x, ac.y, -ac.z).
This matches  sae2three(ac_to_svj(p)) = (p.x, p.y, -p.z)  so the GLB
overlays correctly on the physics skeleton in the SVJ viewer without any
extra rotation.

Triangle winding is reversed (swap index[1] <-> index[2]) because the
handedness changes from left-handed (AC) to right-handed (Three.js).
"""

from __future__ import annotations

import io
import math
import struct
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
    ks_specular_exp: float = 1.0
    diffuse_mult: float = 1.0
    tx_diffuse:  Optional[str] = None
    tx_normal:   Optional[str] = None


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
            if prop_name == "ksAmbient":        mat.ks_ambient      = prop_value
            elif prop_name == "ksDiffuse":      mat.ks_diffuse      = prop_value
            elif prop_name == "ksSpecular":     mat.ks_specular     = prop_value
            elif prop_name == "ksSpecularEXP":  mat.ks_specular_exp = prop_value
            elif prop_name == "diffuseMult":    mat.diffuse_mult    = prop_value

        tex_count = _read_i32(f)
        for _ in range(tex_count):
            sample_name = _len_str(f)
            _slot       = _read_i32(f)
            tex_name    = _len_str(f)
            if sample_name == "txDiffuse": mat.tx_diffuse = tex_name
            elif sample_name == "txNormal": mat.tx_normal = tex_name

        materials.append(mat)
    return materials


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

    return Kn5Model(
        name=path.stem,
        version=version,
        textures=textures,
        materials=materials,
        root=root,
    )


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
# Ephemeral-mesh detection
# ---------------------------------------------------------------------------
# AC controls these meshes at runtime (hidden by default, shown only during
# motion or damage events).  We render them fully transparent so they don't
# make the car look crashed / spinning when displayed as a static model.
_EPHEMERAL_SUBSTRINGS: frozenset[str] = frozenset({
    "blur",      # motion-blur rim discs  (RIM_BLUR_*, rim blur lf, …)
    "damage",    # damage-state body panels
    "dent",      # dented panels
    "bent",      # bent panels
    "crash",     # crash-state geometry
    "deform",    # deformable/crumple-zone meshes
})

def _is_ephemeral(name: str) -> bool:
    """Return True for meshes AC shows/hides dynamically at runtime."""
    nl = name.lower()
    return any(s in nl for s in _EPHEMERAL_SUBSTRINGS)


def _ac_to_three(arr: np.ndarray) -> np.ndarray:
    """
    Vectorised AC -> Three.js/glTF space for an (N, 3) float32 array.
    Just negates Z: three = (ac.x, ac.y, -ac.z).
    """
    out = arr.copy()
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
    M_three[2, :]  = -M_col[2, :]
    M_three[:, 2]  = -M_three[:, 2]
    M_three[2, 2]  =  M_col[2, 2]
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

def kn5_to_glb(
    path: Path,
    output_path: Optional[Path] = None,
    embed_textures: bool = True,
) -> bytes:
    """
    Convert a KN5 file to a self-contained GLB (binary glTF).

    Parameters
    ----------
    path           : Source .kn5 file.
    output_path    : If given, the GLB bytes are also written here.
    embed_textures : Embed texture images in the GLB (default True).

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
            img_data = kn5_tex.data
            mime = "image/png"
            if img_data[:4] == b"DDS ":
                try:
                    from PIL import Image as PILImage
                    buf = io.BytesIO(img_data)
                    pil_img = PILImage.open(buf)
                    # Record alpha bimodality before format conversion.
                    if "A" in pil_img.mode:
                        a = np.asarray(pil_img)[:, :, 3].ravel()
                        extreme = float((a <= 10).sum() + (a >= 245).sum())
                        tex_bimodal[kn5_tex.name] = extreme / len(a) > 0.85
                    out_buf = io.BytesIO()
                    pil_img.save(out_buf, format="PNG")
                    img_data = out_buf.getvalue()
                    mime = "image/png"
                except Exception:
                    continue
            elif img_data[:3] == b"\xff\xd8\xff":
                mime = "image/jpeg"

            bv_idx = _add_buffer_view(img_data)
            img = pygltflib.Image(bufferView=bv_idx, mimeType=mime,
                                   name=kn5_tex.name)
            gltf.images.append(img)
            sampler = pygltflib.Sampler(
                magFilter=pygltflib.LINEAR,
                minFilter=pygltflib.LINEAR_MIPMAP_LINEAR,
                wrapS=pygltflib.REPEAT,
                wrapT=pygltflib.REPEAT,
            )
            gltf.samplers.append(sampler)
            gtex = pygltflib.Texture(
                source=len(gltf.images) - 1,
                sampler=len(gltf.samplers) - 1,
                name=kn5_tex.name,
            )
            gltf.textures.append(gtex)
            tex_idx[kn5_tex.name] = len(gltf.textures) - 1

    # -- Build materials -------------------------------------------------------
    mat_gltf_idx: list[int] = []
    for kn5_mat in model.materials:
        has_tex = embed_textures and kn5_mat.tx_diffuse and kn5_mat.tx_diffuse in tex_idx
        if has_tex:
            base_factor = [1.0, 1.0, 1.0, 1.0]
        else:
            v = max(0.05, kn5_mat.ks_diffuse * kn5_mat.diffuse_mult)
            base_factor = [v, v, v, 1.0]
        pbr = pygltflib.PbrMetallicRoughness(
            baseColorFactor=base_factor,
            metallicFactor=0.0,
            roughnessFactor=max(0.0, 1.0 - kn5_mat.ks_specular),
        )
        if has_tex:
            pbr.baseColorTexture = pygltflib.TextureInfo(
                index=tex_idx[kn5_mat.tx_diffuse]
            )
        gmat = pygltflib.Material(name=kn5_mat.name, pbrMetallicRoughness=pbr)
        if embed_textures and kn5_mat.tx_normal and kn5_mat.tx_normal in tex_idx:
            gmat.normalTexture = pygltflib.NormalMaterialTexture(
                index=tex_idx[kn5_mat.tx_normal]
            )
        # Map KN5 blend mode to glTF alphaMode:
        #   0   -> OPAQUE  (ignore alpha channel)
        #   256 -> MASK    (explicit alpha-test flag: belts, grilles, plates)
        #   1   -> BLEND or MASK depending on texture alpha distribution.
        #          Some car mods use blend=1 for both true semi-transparent
        #          surfaces (glass, lens) and hard-edge cutouts (grille, decals).
        #          We resolve the ambiguity by inspecting the diffuse texture:
        #          if alpha is bimodal (≥85 % of pixels near 0 or near 255)
        #          the surface is a cutout → MASK; otherwise → BLEND.
        if kn5_mat.blend_mode == 256:
            gmat.alphaMode = "MASK"
            gmat.alphaCutoff = 0.5
        elif kn5_mat.blend_mode == 1:
            tx = kn5_mat.tx_diffuse
            if tx and tex_bimodal.get(tx, False):
                gmat.alphaMode = "MASK"
                gmat.alphaCutoff = 0.5
            else:
                gmat.alphaMode = "BLEND"
        else:
            gmat.alphaMode = "OPAQUE"
        gltf.materials.append(gmat)
        mat_gltf_idx.append(len(gltf.materials) - 1)

    # Shared fully-transparent material for blur/damage meshes.
    # All ephemeral nodes share one entry to keep the material table small.
    _hidden_mat = pygltflib.Material(
        name="_hidden_ephemeral",
        pbrMetallicRoughness=pygltflib.PbrMetallicRoughness(
            baseColorFactor=[0.0, 0.0, 0.0, 0.0],
            metallicFactor=0.0,
            roughnessFactor=1.0,
        ),
        alphaMode="BLEND",
    )
    gltf.materials.append(_hidden_mat)
    _hidden_mat_idx: int = len(gltf.materials) - 1

    # -- Build scene nodes (DFS, mirrors KN5 tree) -----------------------------
    scene = pygltflib.Scene(name=model.name, nodes=[])
    gltf.scenes.append(scene)
    gltf.scene = 0

    def _process_node(kn5_node: Kn5Node, parent_gltf_idx: Optional[int]) -> int:
        gnode = pygltflib.Node(name=kn5_node.name)

        if kn5_node.node_type == 1 and kn5_node.matrix:
            gnode.matrix = _mat4_ac_to_three(kn5_node.matrix)

        elif kn5_node.node_type in (2, 3) and kn5_node.positions is not None:
            # AC (left-handed) -> Three.js/glTF (right-handed): negate Z.
            pos = _ac_to_three(kn5_node.positions)
            nrm = _ac_to_three(kn5_node.normals)
            uvs = kn5_node.uvs

            # Tangents: negate Z on XYZ component.
            # Negate W to compensate for the winding reversal below,
            # keeping B = cross(N,T)*W consistent with the UV encoding.
            tan_raw = kn5_node.tangents          # (N,4)
            tan_xyz = _ac_to_three(tan_raw[:, :3])
            tan_w   = -tan_raw[:, 3:4]           # negate handedness for winding flip
            tan = np.concatenate([tan_xyz, tan_w], axis=1).astype("<f4")

            # Reverse triangle winding: handedness flips (left->right-handed).
            idx = kn5_node.indices.reshape(-1, 3)[:, [0, 2, 1]].flatten()

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

            uv_bytes = uvs.astype("<f4").tobytes()
            bv_uv = _add_buffer_view(uv_bytes, target=pygltflib.ARRAY_BUFFER)
            acc_uv = _add_accessor(bv_uv, pygltflib.FLOAT, "VEC2", len(uvs))

            idx16 = idx.astype("<u2").tobytes()
            bv_idx = _add_buffer_view(idx16, target=pygltflib.ELEMENT_ARRAY_BUFFER)
            acc_idx = _add_accessor(bv_idx, pygltflib.UNSIGNED_SHORT, "SCALAR",
                                    len(idx))

            mat_idx = (mat_gltf_idx[kn5_node.material_id]
                       if 0 <= kn5_node.material_id < len(mat_gltf_idx) else None)
            # Blur-rim / damage meshes: override to fully transparent so the
            # static GLB doesn't look like a crashed or spinning car.
            if _is_ephemeral(kn5_node.name):
                mat_idx = _hidden_mat_idx

            primitive = pygltflib.Primitive(
                attributes=pygltflib.Attributes(
                    POSITION=acc_pos,
                    NORMAL=acc_nrm,
                    TANGENT=acc_tan,
                    TEXCOORD_0=acc_uv,
                ),
                indices=acc_idx,
                material=mat_idx,
            )
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

    # -- Front-axle alignment wrapper ------------------------------------------
    # AC cars store their mesh with the front axle at some arbitrary Z in
    # model space.  The SVJ viewer expects the front axle at Three.js Z = 0.
    # After the AC->Three.js transform (negate Z), a front-axle node that sat
    # at AC Z = +F ends up at Three.js Z = -F.  Adding a wrapper node with
    # translation [0, 0, +F] shifts everything so the front axle lands at 0.
    if model.root:
        front_axle_z = _find_front_axle_z(model.root)
        if front_axle_z is not None and abs(front_axle_z) > 0.01:
            wrapper = pygltflib.Node(
                name="_ac_front_axle_align",
                translation=[0.0, 0.0, float(front_axle_z)],
            )
            gltf.nodes.append(wrapper)
            wrapper_idx = len(gltf.nodes) - 1
            scene.nodes.append(wrapper_idx)
            _process_node(model.root, wrapper_idx)
        else:
            _process_node(model.root, None)

    # -- Finalise buffer -------------------------------------------------------
    gltf.buffers.append(pygltflib.Buffer(byteLength=len(bin_data)))
    gltf.set_binary_blob(bytes(bin_data))
    glb_bytes = b"".join(gltf.save_to_bytes())
    if output_path:
        output_path.write_bytes(glb_bytes)
    return glb_bytes


# --- Convenience: find the main KN5 / all LODs in an AC car folder -----------

_EFFECT_PREFIXES = ("3d", "smoke", "particle", "collider", "blur_", "tyre_")


def find_car_kn5(car_path: Path) -> Optional[Path]:
    """
    Return the primary (LOD A) KN5 file for an AC car folder.
    Falls back to largest non-effect KN5 if the stem-named file is absent.
    """
    car_path = Path(car_path)
    named = car_path / f"{car_path.name}.kn5"
    if named.exists():
        return named

    all_kn5 = list(car_path.glob("*.kn5"))
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
    Returns an empty list when no KN5 is found at all.
    """
    car_path = Path(car_path)
    stem = car_path.name          # e.g. "bo_caterham_165_lhd"
    result: list[tuple[str, Path]] = []

    lod_a = car_path / f"{stem}.kn5"
    if lod_a.exists():
        result.append(("A", lod_a))
    else:
        # Fallback: largest non-effect KN5 that doesn't look like a LOD file
        all_kn5 = [p for p in car_path.glob("*.kn5")
                   if not p.stem.lower().startswith(_EFFECT_PREFIXES)
                   and "_lod_" not in p.stem.lower()]
        if all_kn5:
            result.append(("A", max(all_kn5, key=lambda p: p.stat().st_size)))

    for label in ("B", "C", "D"):
        lod_path = car_path / f"{stem}_LOD_{label}.kn5"
        if lod_path.exists():
            result.append((label, lod_path))

    return result


def kn5_all_lods_to_glbs(
    car_path: Path,
    output_dir: Path,
    embed_textures: bool = True,
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

    Returns
    -------
    dict  : {lod_label: output_path}  for every LOD that was exported.
    """
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    lods = find_car_kn5_lods(car_path)
    results: dict[str, Path] = {}
    for label, kn5_path in lods:
        out_path = output_dir / f"{kn5_path.stem}.glb"
        kn5_to_glb(kn5_path, output_path=out_path, embed_textures=embed_textures)
        results[label] = out_path
    return results


# --- CLI entry point ----------------------------------------------------------

if __name__ == "__main__":
    import sys as _sys
    if len(_sys.argv) < 2:
        print("Usage: python kn5_reader.py <car.kn5> [output.glb]")
        _sys.exit(1)
    src = Path(_sys.argv[1])
    dst = Path(_sys.argv[2]) if len(_sys.argv) > 2 else src.with_suffix(".glb")
    print("Scanning nodes ...")
    names = scan_kn5_nodes(src)
    print(f"  {len(names)} nodes found:")
    for n in names:
        print(f"    {n}")
    mapping = map_ac_nodes_to_svj(names)
    print("\nSVJ visual binding map:")
    for svj_id, ac_name in mapping.items():
        print(f"  {svj_id:20s} <- {ac_name}")
    print("\nConverting to GLB ...")
    glb = kn5_to_glb(src, output_path=dst)
    print(f"  Written {len(glb):,} bytes -> {dst}")
