"""Read a car mesh from a glTF/GLB or OpenUSD/USDZ asset into one neutral scene tree.

The SVJ standard's mesh assets (``assets.meshes[]``, spec 22) are glTF (``.glb``/``.gltf``) or OpenUSD
(``.usd``/``.usda``/``.usdc``/``.usdz``); both carry one node per bound part, named
``SVJ::<category>::<id>`` (USD spells ``::`` as ``__``).  This module reads either into the same tree
so the KN5 writer does not care which it was:

    SceneNode(name, matrix, children, meshes)          matrix: 4x4, column vectors, local to the parent
    SceneMesh(positions, normals, uvs, indices, material)
    SceneMaterial(name, base_color, base_texture, normal_texture, alpha_mode, ...)

Everything stays in the asset's own frame (glTF: Y up, -Z forward, right-handed; USD: its up axis and
metres-per-unit are applied to the root so the result is Y up in metres).  Textures are returned as encoded
image bytes (PNG / JPEG / DDS) with a file name; nothing is decoded unless a colour factor has to be baked.
"""
from __future__ import annotations

import io
import math
import struct
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np


@dataclass
class SceneTexture:
    name: str
    data: bytes


@dataclass
class SceneMaterial:
    name: str
    base_color: tuple = (1.0, 1.0, 1.0, 1.0)
    base_texture: Optional[SceneTexture] = None
    normal_texture: Optional[SceneTexture] = None
    alpha_mode: str = "OPAQUE"            # OPAQUE | MASK | BLEND
    alpha_cutoff: float = 0.5
    metallic: float = 0.0
    roughness: float = 0.6
    emissive: tuple = (0.0, 0.0, 0.0)
    double_sided: bool = False


@dataclass
class SceneMesh:
    positions: np.ndarray                  # (N, 3) float32
    normals: Optional[np.ndarray]          # (N, 3) or None
    uvs: Optional[np.ndarray]              # (N, 2) or None
    indices: np.ndarray                    # (M,) uint32, triangles
    material: SceneMaterial
    name: str = ""


@dataclass
class SceneNode:
    name: str
    matrix: np.ndarray = field(default_factory=lambda: np.eye(4))
    children: list = field(default_factory=list)
    meshes: list = field(default_factory=list)


class SceneError(Exception):
    pass


def _norm_name(name: str) -> str:
    """USD prim names cannot hold ':': ``SVJ__wheel__wheel_fl`` is ``SVJ::wheel::wheel_fl``."""
    return name.replace("__", "::") if name.startswith("SVJ__") else name


# --------------------------------------------------------------------------- #
# glTF / GLB                                                                  #
# --------------------------------------------------------------------------- #

_COMP = {5120: np.int8, 5121: np.uint8, 5122: np.int16, 5123: np.uint16, 5125: np.uint32, 5126: np.float32}
_NCOMP = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4, "MAT2": 4, "MAT3": 9, "MAT4": 16}


def _accessor(g, blob: bytes, idx: int) -> np.ndarray:
    a = g.accessors[idx]
    dt = np.dtype(_COMP[a.componentType])
    n = _NCOMP[a.type]
    out = np.zeros((a.count, n), dtype=dt)
    if a.bufferView is not None:
        bv = g.bufferViews[a.bufferView]
        start = (bv.byteOffset or 0) + (a.byteOffset or 0)
        stride = bv.byteStride or dt.itemsize * n
        if stride == dt.itemsize * n:
            out = np.frombuffer(blob, dtype=dt, count=a.count * n, offset=start).reshape(a.count, n).copy()
        else:
            for i in range(a.count):
                out[i] = np.frombuffer(blob, dtype=dt, count=n, offset=start + i * stride)
    if a.normalized and dt.kind in "iu":
        out = out.astype(np.float32) / float(np.iinfo(dt).max)
    return out


def _image_bytes(g, blob: bytes, src: int, base: Optional[Path]) -> tuple[Optional[bytes], str]:
    im = g.images[src]
    name = im.name or f"image_{src}"
    data = None
    if im.bufferView is not None:
        bv = g.bufferViews[im.bufferView]
        off = bv.byteOffset or 0
        data = bytes(blob[off:off + bv.byteLength])
    elif im.uri:
        if im.uri.startswith("data:"):
            import base64
            data = base64.b64decode(im.uri.split(",", 1)[1])
        elif base is not None and (base / im.uri).is_file():
            data = (base / im.uri).read_bytes()
            name = Path(im.uri).stem
    ext = ".png"
    if data:
        if data[:3] == b"\xff\xd8\xff":
            ext = ".jpg"
        elif data[:4] == b"DDS ":
            ext = ".dds"
    return data, name + ext


def _quat_matrix(q) -> np.ndarray:
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def load_gltf(path: Path) -> SceneNode:
    import pygltflib
    g = pygltflib.GLTF2().load(str(path))
    if g is None:
        raise SceneError(f"cannot read {path}")
    blob = g.binary_blob() or b""
    if not blob and g.buffers and g.buffers[0].uri:
        uri = g.buffers[0].uri
        if uri.startswith("data:"):
            import base64
            blob = base64.b64decode(uri.split(",", 1)[1])
        else:
            blob = (path.parent / uri).read_bytes()
    base = path.parent

    tex_cache: dict[int, Optional[SceneTexture]] = {}

    def texture(info) -> Optional[SceneTexture]:
        if info is None or info.index is None:
            return None
        if info.index not in tex_cache:
            src = g.textures[info.index].source
            data, name = _image_bytes(g, blob, src, base) if src is not None else (None, "")
            tex_cache[info.index] = SceneTexture(name, data) if data else None
        return tex_cache[info.index]

    mat_cache: dict[int, SceneMaterial] = {}

    def material(i: Optional[int]) -> SceneMaterial:
        if i is None:
            return SceneMaterial("default")
        if i not in mat_cache:
            m = g.materials[i]
            pbr = m.pbrMetallicRoughness
            mm = SceneMaterial(m.name or f"material_{i}")
            if pbr is not None:
                if pbr.baseColorFactor:
                    mm.base_color = tuple(float(v) for v in pbr.baseColorFactor)
                mm.base_texture = texture(pbr.baseColorTexture)
                mm.metallic = 1.0 if pbr.metallicFactor is None else float(pbr.metallicFactor)
                mm.roughness = 1.0 if pbr.roughnessFactor is None else float(pbr.roughnessFactor)
            mm.normal_texture = texture(m.normalTexture)
            mm.alpha_mode = m.alphaMode or "OPAQUE"
            mm.alpha_cutoff = 0.5 if m.alphaCutoff is None else float(m.alphaCutoff)
            mm.emissive = tuple(float(v) for v in (m.emissiveFactor or (0, 0, 0)))
            mm.double_sided = bool(m.doubleSided)
            mat_cache[i] = mm
        return mat_cache[i]

    def build(i: int) -> SceneNode:
        n = g.nodes[i]
        if n.matrix:
            M = np.array(n.matrix, dtype=np.float64).reshape(4, 4, order="F")
        else:
            M = np.eye(4)
            R = _quat_matrix(n.rotation) if n.rotation else np.eye(3)
            S = np.array(n.scale) if n.scale else np.ones(3)
            M[:3, :3] = R * S
            if n.translation:
                M[:3, 3] = n.translation
        node = SceneNode(n.name or f"node_{i}", M)
        if n.mesh is not None:
            for k, prim in enumerate(g.meshes[n.mesh].primitives):
                if prim.mode not in (None, 4):
                    continue
                pos = _accessor(g, blob, prim.attributes.POSITION).astype(np.float32)
                nrm = _accessor(g, blob, prim.attributes.NORMAL).astype(np.float32) if prim.attributes.NORMAL is not None else None
                uv = _accessor(g, blob, prim.attributes.TEXCOORD_0).astype(np.float32) if prim.attributes.TEXCOORD_0 is not None else None
                idx = (_accessor(g, blob, prim.indices).reshape(-1).astype(np.uint32) if prim.indices is not None
                       else np.arange(len(pos), dtype=np.uint32))
                node.meshes.append(SceneMesh(pos, nrm, uv, idx, material(prim.material), f"{node.name}_{k}"))
        node.children = [build(c) for c in (n.children or [])]
        return node

    sc = g.scenes[g.scene or 0] if g.scenes else None
    roots = list(sc.nodes) if sc is not None else [i for i in range(len(g.nodes))]
    root = SceneNode("root")
    root.children = [build(r) for r in roots]
    return root


# --------------------------------------------------------------------------- #
# OpenUSD / USDZ                                                              #
# --------------------------------------------------------------------------- #


def load_usd(path: Path) -> SceneNode:
    try:
        from pxr import Ar, Gf, Sdf, Usd, UsdGeom, UsdShade
    except ImportError as exc:                              # pragma: no cover
        raise SceneError("reading USD needs the 'usd-core' package (pip install usd-core)") from exc
    stage = Usd.Stage.Open(str(path))
    if stage is None:
        raise SceneError(f"cannot open {path}")
    tc = Usd.TimeCode.Default()
    mpu = UsdGeom.GetStageMetersPerUnit(stage) or 1.0
    up = UsdGeom.GetStageUpAxis(stage)

    def read_asset(ap) -> Optional[bytes]:
        raw = ap.resolvedPath or ap.path
        if not raw:
            return None
        try:
            a = Ar.GetResolver().OpenAsset(Ar.ResolvedPath(raw))
            if a:
                return bytes(a.GetBuffer())
        except Exception:
            pass
        p = Path(raw)
        return p.read_bytes() if p.is_file() else None

    mat_cache: dict[str, SceneMaterial] = {}

    def input_source(inp):
        """The shader output feeding ``inp`` (a texture reader) or None."""
        src = inp.GetConnectedSource()
        return src

    def tex_of(shader) -> Optional[SceneTexture]:
        if shader is None or not shader.GetPrim().IsValid():
            return None
        f = shader.GetInput("file")
        if not f:
            return None
        ap = f.Get(tc)
        if ap is None:
            return None
        data = read_asset(ap)
        if not data:
            return None
        name = Path((ap.path or "tex").split("[")[-1].rstrip("]")).name or "tex.png"
        return SceneTexture(name, data)

    def material(mat) -> SceneMaterial:
        key = str(mat.GetPath())
        if key in mat_cache:
            return mat_cache[key]
        out = SceneMaterial(mat.GetPrim().GetName())
        surf = mat.ComputeSurfaceSource()[0] if mat else None
        if surf:
            def val(name, default):
                inp = surf.GetInput(name)
                return default if not inp or inp.Get(tc) is None else inp.Get(tc)
            def src_shader(name):
                inp = surf.GetInput(name)
                if not inp:
                    return None, None
                cs = inp.GetConnectedSource()
                if not cs:
                    return None, None
                return UsdShade.Shader(cs[0].GetPrim()), cs[1]
            col = val("diffuseColor", (1.0, 1.0, 1.0))
            out.base_color = (float(col[0]), float(col[1]), float(col[2]), 1.0)
            sh, _ = src_shader("diffuseColor")
            out.base_texture = tex_of(sh)
            if sh is not None:
                sc = sh.GetInput("scale")
                if sc and sc.Get(tc) is not None:
                    s = sc.Get(tc)
                    out.base_color = (float(s[0]), float(s[1]), float(s[2]), float(s[3]) if len(s) > 3 else 1.0)
            nsh, _ = src_shader("normal")
            out.normal_texture = tex_of(nsh)
            out.roughness = float(val("roughness", 0.6))
            out.metallic = float(val("metallic", 0.0))
            opac = surf.GetInput("opacity")
            if opac:
                cs = opac.GetConnectedSource()
                const = opac.Get(tc)
                if cs:
                    thr = surf.GetInput("opacityThreshold")
                    if thr and thr.Get(tc):
                        out.alpha_mode, out.alpha_cutoff = "MASK", float(thr.Get(tc))
                    else:
                        out.alpha_mode = "BLEND"
                elif const is not None and float(const) < 0.999:
                    out.alpha_mode = "BLEND"
                    out.base_color = out.base_color[:3] + (float(const),)
            em = val("emissiveColor", (0, 0, 0))
            out.emissive = (float(em[0]), float(em[1]), float(em[2]))
        mat_cache[key] = out
        return out

    def mesh_of(prim) -> Optional[SceneMesh]:
        m = UsdGeom.Mesh(prim)
        pts = m.GetPointsAttr().Get(tc)
        if not pts:
            return None
        counts = np.array(m.GetFaceVertexCountsAttr().Get(tc), dtype=np.int64)
        fvi = np.array(m.GetFaceVertexIndicesAttr().Get(tc), dtype=np.int64)
        P = np.array(pts, dtype=np.float32)
        # fan triangulation of every face, as corner indices into the face-vertex list
        starts = np.concatenate([[0], np.cumsum(counts)[:-1]])
        tri_c = []
        for s, c in zip(starts, counts):
            for k in range(1, c - 1):
                tri_c.append((s, s + k, s + k + 1))
        if not tri_c:
            return None
        tri_c = np.array(tri_c, dtype=np.int64)                 # (T, 3) corner numbers
        corners = tri_c.reshape(-1)                              # (3T,)
        vert = fvi[corners]
        positions = P[vert]
        nrm = None
        na = m.GetNormalsAttr().Get(tc)
        if na is not None and len(na):
            N = np.array(na, dtype=np.float32)
            interp = m.GetNormalsInterpolation()
            nrm = N[corners] if interp == "faceVarying" else N[vert] if len(N) == len(P) else None
        uv = None
        pv = UsdGeom.PrimvarsAPI(prim).GetPrimvar("st") or UsdGeom.PrimvarsAPI(prim).GetPrimvar("UVMap")
        if pv and pv.IsDefined():
            vals = pv.Get(tc)
            if vals is not None:
                U = np.array(vals, dtype=np.float32)
                idxs = pv.GetIndices(tc)
                if idxs is not None and len(idxs):
                    U = U[np.array(idxs, dtype=np.int64)]
                interp = pv.GetInterpolation()
                uv = U[corners] if interp == "faceVarying" else U[vert] if len(U) == len(P) else None
                if uv is not None:
                    uv = uv.copy()
                    uv[:, 1] = 1.0 - uv[:, 1]                    # USD's V is up, glTF / AC's down
        bound = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
        mm = material(bound) if bound else SceneMaterial("default")
        return SceneMesh(positions, nrm, uv, np.arange(len(positions), dtype=np.uint32), mm, prim.GetName())

    def matrix_of(prim) -> np.ndarray:
        xf = UsdGeom.Xformable(prim)
        if not xf:
            return np.eye(4)
        r = xf.GetLocalTransformation(tc)
        m = r[0] if isinstance(r, tuple) else r
        return np.array([[m[r][c] for c in range(4)] for r in range(4)], dtype=np.float64).T   # row -> column vectors

    def build(prim) -> Optional[SceneNode]:
        if not prim.IsActive() or prim.GetTypeName() in ("Material", "Shader", "Scope") and prim.GetName() == "Materials":
            return None
        if prim.GetTypeName() == "Material":
            return None
        node = SceneNode(_norm_name(prim.GetName()), matrix_of(prim))
        if prim.IsA(UsdGeom.Mesh):
            mesh = mesh_of(prim)
            if mesh is not None:
                node.meshes.append(mesh)
        for c in prim.GetChildren():
            ch = build(c)
            if ch is not None:
                node.children.append(ch)
        return node

    root = SceneNode("root")
    dp = stage.GetDefaultPrim()
    tops = [dp] if dp and dp.IsValid() else [p for p in stage.GetPseudoRoot().GetChildren()]
    for t in tops:
        ch = build(t)
        if ch is not None:
            root.children.append(ch)
    # to metres and Y up
    fix = np.eye(4)
    if up == "Z":
        fix[:3, :3] = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], dtype=np.float64)
    fix[:3, :3] *= mpu
    root.matrix = fix
    return root


def load_scene(path) -> SceneNode:
    """A glTF / GLB or OpenUSD / USDZ file as a :class:`SceneNode` tree."""
    p = Path(path)
    ext = p.suffix.lower()
    if ext in (".glb", ".gltf"):
        return load_gltf(p)
    if ext in (".usd", ".usda", ".usdc", ".usdz"):
        return load_usd(p)
    raise SceneError(f"unsupported mesh format {ext!r} (glTF/GLB or USD/USDZ)")


def walk(node: SceneNode, parent_world: Optional[np.ndarray] = None):
    """Yield ``(node, world_matrix)`` depth first."""
    w = node.matrix if parent_world is None else parent_world @ node.matrix
    yield node, w
    for c in node.children:
        yield from walk(c, w)


def count(node: SceneNode) -> dict:
    n = m = t = 0
    for nd, _ in walk(node):
        n += 1
        for ms in nd.meshes:
            m += 1
            t += len(ms.indices) // 3
    return {"nodes": n, "meshes": m, "triangles": t}
