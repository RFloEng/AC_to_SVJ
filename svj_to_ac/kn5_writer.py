"""Write an Assetto Corsa ``car.kn5`` from a glTF/GLB or OpenUSD/USDZ car mesh (``mesh_scene``).

It is the inverse of ``kn5_reader.kn5_to_glb`` of the forward converter: the same axis flip, the same
physics-origin shift and the same part names, so a car that went AC -> SVJ -> AC comes back where it was.

Frames.  The asset is in the SVJ glTF frame (origin on the ground under the front axle, Y up, -Z forward).
The KN5 model space has its origin at the centre of gravity (``GRAPHICS_OFFSET`` is written as 0, 0, 0):

    kn5 = (-X, Y - h_cg, -Z + d_f)        h_cg: height of the CG, d_f: distance of the CG behind the front axle

a 180 degree turn about Y plus a shift, which keeps the triangle winding.  Node matrices follow as
``L M L`` with ``L = diag(-1, 1, -1)``, vertices as ``L v``, and each part sits at the model's top level with its
full world matrix, so the wheels are direct children of the root whose origin is the wheel centre, as the game
expects.

Parts.  A node named ``SVJ::<category>::<id>`` is one part: ``SVJ::wheel::wheel_fl`` is ``WHEEL_LF``,
``SVJ::brake::disc_fl`` is ``DISC_LF``, ``SVJ::steering::wheel`` is ``STEER_HR``; the others keep their id.
Everything outside such a node goes in as it is.  Without wheel parts the car is written without rotating wheels
(the report says so).

Materials.  glTF / UsdPreviewSurface -> AC's Blinn-Phong shaders: ``ksPerPixel`` / ``ksPerPixelNM`` (normal map),
``ksPerPixelAT`` (alpha cut-out), ``ksPerPixelAlpha`` (blend); specular power from the roughness (the inverse of
what the forward converter reads), intensity from the metallic factor; a base-colour factor is multiplied into
the texture, or becomes a 1x1 texture when there is none.  Textures are embedded as PNG / JPEG / DDS as they come.
Nothing here can be tested in the game from a script: the file is read back with ``kn5_reader.parse_kn5``.
"""
from __future__ import annotations

import hashlib
import io
import re
import struct
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from mesh_scene import SceneMaterial, SceneMesh, SceneNode, SceneTexture

KN5_VERSION = 5
MAX_VERTS = 65535
_L = np.diag([-1.0, 1.0, -1.0])

_WHEEL = {"fl": "LF", "fr": "RF", "rl": "LR", "rr": "RR"}
_SVJ_NAME = re.compile(r"^SVJ::([^:]+)::(.+)$")


# --------------------------------------------------------------------------- #
# Names                                                                       #
# --------------------------------------------------------------------------- #


def ac_name(svj_name: str) -> tuple[str, str]:
    """``(AC node name, role)`` of an ``SVJ::category::id`` node; role is "wheel", "disc", "steer" or "part"."""
    m = _SVJ_NAME.match(svj_name)
    if not m:
        return svj_name, "part"
    cat, ident = m.group(1), m.group(2)
    low = ident.lower()
    if cat == "wheel" and low.startswith("wheel_") and low[6:] in _WHEEL:
        return f"WHEEL_{_WHEEL[low[6:]]}", "wheel"
    if cat == "brake" and low.startswith("disc_") and low[5:] in _WHEEL:
        return f"DISC_{_WHEEL[low[5:]]}", "disc"
    if cat == "steering" and low in ("wheel", "steering_wheel"):
        return "STEER_HR", "steer"
    return ident, "part"


# --------------------------------------------------------------------------- #
# Geometry                                                                    #
# --------------------------------------------------------------------------- #


def weld(pos, nrm, uv, idx):
    """Merge identical vertices (the USD reader unwelds every corner)."""
    cols = [pos]
    cols.append(nrm if nrm is not None else np.zeros((len(pos), 3), np.float32))
    cols.append(uv if uv is not None else np.zeros((len(pos), 2), np.float32))
    key = np.ascontiguousarray(np.round(np.concatenate(cols, axis=1).astype(np.float32), 6))
    _, first, inv = np.unique(key.view([("", key.dtype)] * key.shape[1]).reshape(-1),
                              return_index=True, return_inverse=True)
    return (pos[first], None if nrm is None else nrm[first], None if uv is None else uv[first],
            inv.astype(np.uint32)[idx])


def tangents(pos, nrm, uv, idx) -> np.ndarray:
    """Per-vertex tangents from the UV layout (accumulated per triangle, orthogonalised to the normal)."""
    t = np.zeros_like(pos, dtype=np.float64)
    if uv is None or len(idx) < 3:
        t[:, 0] = 1.0
        return t.astype(np.float32)
    tri = idx.reshape(-1, 3)
    p0, p1, p2 = (pos[tri[:, k]].astype(np.float64) for k in range(3))
    u0, u1, u2 = (uv[tri[:, k]].astype(np.float64) for k in range(3))
    e1, e2 = p1 - p0, p2 - p0
    d1, d2 = u1 - u0, u2 - u0
    det = d1[:, 0] * d2[:, 1] - d2[:, 0] * d1[:, 1]
    r = np.where(np.abs(det) > 1e-12, 1.0 / np.where(np.abs(det) > 1e-12, det, 1.0), 0.0)
    tt = (e1 * d2[:, 1:2] - e2 * d1[:, 1:2]) * r[:, None]
    for k in range(3):
        np.add.at(t, tri[:, k], tt)
    if nrm is not None:
        n = nrm.astype(np.float64)
        t = t - n * np.sum(n * t, axis=1, keepdims=True)
    ln = np.linalg.norm(t, axis=1, keepdims=True)
    bad = ln[:, 0] < 1e-12
    t = t / np.where(ln < 1e-12, 1.0, ln)
    t[bad] = (1.0, 0.0, 0.0)
    return t.astype(np.float32)


def vertex_normals(pos, idx) -> np.ndarray:
    n = np.zeros(pos.shape, dtype=np.float64)
    tri = idx.reshape(-1, 3)
    fn = np.cross(pos[tri[:, 1]].astype(np.float64) - pos[tri[:, 0]], pos[tri[:, 2]].astype(np.float64) - pos[tri[:, 0]])
    for k in range(3):
        np.add.at(n, tri[:, k], fn)
    ln = np.linalg.norm(n, axis=1, keepdims=True)
    n = n / np.where(ln < 1e-12, 1.0, ln)
    return n.astype(np.float32)


def split_chunks(idx: np.ndarray, nverts: int):
    """Triangle index chunks that each touch at most MAX_VERTS vertices: yields (local_indices, vertex_ids)."""
    if nverts <= MAX_VERTS:
        yield idx, None
        return
    tri = idx.reshape(-1, 3)
    start = 0
    while start < len(tri):
        used: dict = {}
        sel = []
        i = start
        while i < len(tri):
            new = [v for v in tri[i] if v not in used]
            if len(used) + len(set(new)) > MAX_VERTS:
                break
            for v in tri[i]:
                if v not in used:
                    used[v] = len(used)
            sel.append(i)
            i += 1
        chunk = np.array([[used[v] for v in tri[j]] for j in sel], dtype=np.uint32).reshape(-1)
        yield chunk, np.fromiter(used.keys(), dtype=np.int64, count=len(used))
        start = i


# --------------------------------------------------------------------------- #
# Materials                                                                   #
# --------------------------------------------------------------------------- #


def _tint_png(data: bytes, color) -> bytes:
    """The texture multiplied by a base-colour factor, as PNG (an unreadable image is returned unchanged)."""
    try:
        from PIL import Image
        im = Image.open(io.BytesIO(data)).convert("RGBA")
        a = np.asarray(im, dtype=np.float32) / 255.0
        a = a * np.array(color, dtype=np.float32)
        out = Image.fromarray((np.clip(a, 0, 1) * 255 + 0.5).astype(np.uint8), "RGBA")
        buf = io.BytesIO()
        out.save(buf, format="PNG")
        return buf.getvalue()
    except Exception:
        return data


def _solid_png(color) -> bytes:
    from PIL import Image
    c = tuple(int(round(min(max(v, 0.0), 1.0) * 255)) for v in color)
    buf = io.BytesIO()
    Image.new("RGBA", (4, 4), c).save(buf, format="PNG")
    return buf.getvalue()


def _srgb_to_linear_free(c):
    return c


@dataclass
class Kn5Material:
    name: str
    shader: str
    blend: int                         # int16 of the file: alphaBlendMode | alphaTested << 8
    props: dict
    textures: list                     # [(sampler, slot, texture name)]
    transparent: bool = False


@dataclass
class Report:
    meshes: int = 0
    triangles: int = 0
    vertices: int = 0
    materials: int = 0
    textures: int = 0
    wheels: list = field(default_factory=list)
    parts: list = field(default_factory=list)
    notes: list = field(default_factory=list)
    split: int = 0


class _Builder:
    def __init__(self, model_name: str):
        self.model = model_name
        self.textures: dict[str, tuple[str, bytes]] = {}      # sha -> (name, data)
        self.tex_names: set = set()
        self.materials: list[Kn5Material] = []
        self.mat_index: dict = {}
        self.report = Report()

    def texture(self, tex: Optional[SceneTexture], data: Optional[bytes] = None, stem: str = "tex") -> Optional[str]:
        blob = data if data is not None else (tex.data if tex else None)
        if not blob:
            return None
        sha = hashlib.sha1(blob).hexdigest()
        if sha in self.textures:
            return self.textures[sha][0]
        ext = ".png"
        if blob[:3] == b"\xff\xd8\xff":
            ext = ".jpg"
        elif blob[:4] == b"DDS ":
            ext = ".dds"
        base = re.sub(r"[^A-Za-z0-9_.-]", "_", (tex.name.rsplit(".", 1)[0] if tex else stem)) or stem
        name, n = base + ext, 1
        while name.lower() in self.tex_names:
            n += 1
            name = f"{base}_{n}{ext}"
        self.tex_names.add(name.lower())
        self.textures[sha] = (name, blob)
        return name

    def material(self, m: SceneMaterial) -> int:
        key = (m.name, m.alpha_mode, tuple(round(v, 4) for v in m.base_color), m.roughness, m.metallic,
               id(m.base_texture.data) if m.base_texture else 0, id(m.normal_texture.data) if m.normal_texture else 0)
        if key in self.mat_index:
            return self.mat_index[key]
        r = min(max(m.roughness, 0.04), 1.0)
        spec_exp = float(np.clip(2.0 / (r * r) - 2.0, 1.0, 1000.0))
        spec = float(np.clip(0.12 + 0.55 * m.metallic + 0.25 * (1.0 - r), 0.05, 1.0))
        diffuse_tex = None
        white = all(v > 0.995 for v in m.base_color)
        if m.base_texture is not None:
            data = m.base_texture.data if white or m.base_texture.data[:4] == b"DDS " else _tint_png(m.base_texture.data, m.base_color)
            diffuse_tex = self.texture(m.base_texture if white else None, data=data,
                                       stem=f"{m.name}_diffuse") if not white else self.texture(m.base_texture)
        else:
            diffuse_tex = self.texture(None, data=_solid_png(m.base_color), stem=f"{m.name}_colour")
        normal_tex = self.texture(m.normal_texture) if m.normal_texture is not None else None
        shader = "ksPerPixelNM" if normal_tex else "ksPerPixel"
        blend, transparent = 0, False
        if m.alpha_mode == "BLEND":
            shader, blend, transparent = "ksPerPixelAlpha", 1, True
            normal_tex = None                      # no normal-mapped blend shader is assumed
        elif m.alpha_mode == "MASK":
            shader, blend = "ksPerPixelAT", 256
        props = {"ksAmbient": 0.4, "ksDiffuse": 0.6, "ksSpecular": spec, "ksSpecularEXP": spec_exp, "ksEmissive": 0.0}
        if any(v > 0.01 for v in m.emissive):
            props["ksEmissive"] = float(max(m.emissive))
        texs = []
        if diffuse_tex:
            texs.append(("txDiffuse", 0, diffuse_tex))
        if normal_tex:
            texs.append(("txNormal", 1, normal_tex))
        name = m.name or f"material_{len(self.materials)}"
        existing = {x.name for x in self.materials}
        base, n = name, 1
        while name in existing:
            n += 1
            name = f"{base}_{n}"
        self.materials.append(Kn5Material(name, shader, blend, props, texs, transparent))
        self.mat_index[key] = len(self.materials) - 1
        return self.mat_index[key]


# --------------------------------------------------------------------------- #
# Serialisation                                                               #
# --------------------------------------------------------------------------- #


def _s(text: str) -> bytes:
    b = text.encode("utf-8")
    return struct.pack("<i", len(b)) + b


def _mat_bytes(M: np.ndarray) -> bytes:
    """4x4 column-vector matrix -> KN5's row-major, translation in the last row."""
    return struct.pack("<16f", *np.asarray(M, dtype=np.float32).T.reshape(-1))


def _mesh_record(name: str, mesh: SceneMesh, mat_id: int, transparent: bool, Lpos: np.ndarray, rep: Report) -> list[bytes]:
    pos, nrm, uv, idx = mesh.positions, mesh.normals, mesh.uvs, mesh.indices.astype(np.int64)
    if len(idx) < 3:
        return []
    if uv is None:
        uv = np.zeros((len(pos), 2), np.float32)
    if len(idx) // 3 * 3 != len(idx):
        idx = idx[: len(idx) // 3 * 3]
    pos, nrm, uv, idx = weld(pos, nrm, uv, idx)
    idx = idx.astype(np.int64)
    if nrm is None:
        nrm = vertex_normals(pos, idx)
    tan = tangents(pos, nrm, uv, idx)
    pos, nrm, tan = (a @ Lpos.T for a in (pos, nrm, tan))
    out = []
    for k, (cidx, vids) in enumerate(split_chunks(idx, len(pos))):
        if vids is None:
            p, n_, u, t = pos, nrm, uv, tan
        else:
            p, n_, u, t = pos[vids], nrm[vids], uv[vids], tan[vids]
            rep.split += 1
        verts = np.concatenate([p, n_, u, t], axis=1).astype("<f4")
        lo, hi = p.min(axis=0), p.max(axis=0)
        c = (lo + hi) / 2
        radius = float(np.linalg.norm(hi - c))
        rec = bytearray()
        rec += struct.pack("<i", 2) + _s(name if k == 0 else f"{name}_{k}") + struct.pack("<i", 0) + b"\x01"
        rec += bytes([1, 1, 1 if transparent else 0])
        rec += struct.pack("<i", len(verts)) + verts.tobytes()
        rec += struct.pack("<i", len(cidx)) + cidx.astype("<u2").tobytes()
        rec += struct.pack("<i", mat_id)
        rec += struct.pack("<i", 0) + struct.pack("<ff", 0.0, 0.0) + struct.pack("<4f", *c, radius) + b"\x01"
        assert len(rec) > 0
        out.append(bytes(rec))
        rep.meshes += 1
        rep.triangles += len(cidx) // 3
        rep.vertices += len(verts)
    return out


def _node_bytes(name: str, matrix: np.ndarray, children: list[bytes]) -> bytes:
    return (struct.pack("<i", 1) + _s(name) + struct.pack("<i", len(children)) + b"\x01"
            + _mat_bytes(matrix) + b"".join(children))


def build_kn5(scene: SceneNode, model_name: str, cg_height: float, cg_behind_front_axle: float,
              ) -> tuple[bytes, Report]:
    """The KN5 bytes of ``scene`` placed with ``kn5 = (-X, Y - h_cg, -Z + d_f)`` (see the module docstring)."""
    b = _Builder(model_name)
    rep = b.report
    A = np.eye(4)
    A[:3, :3] = _L
    A[:3, 3] = (0.0, -float(cg_height), float(cg_behind_front_axle))       # world: kn5 = A . svj
    L4 = np.eye(4)
    L4[:3, :3] = _L
    used: dict = {}

    def unique(name: str) -> str:
        n = used.get(name, 0)
        used[name] = n + 1
        return name if n == 0 else f"{name}_{n}"

    def sub(node: SceneNode) -> bytes:
        """A subtree below a part: local matrices conjugated by L, meshes as children."""
        kids = []
        for m in node.meshes:
            mid = b.material(m.material)
            kids += _mesh_record(unique(m.name or node.name), m, mid, b.materials[mid].transparent, _L, rep)
        kids += [sub(c) for c in node.children]
        return _node_bytes(unique(node.name), L4 @ node.matrix @ L4, kids)

    parts: list[bytes] = []

    def top(node: SceneNode, world: np.ndarray, role_seen: bool) -> None:
        name, role = ac_name(node.name)
        is_part = bool(_SVJ_NAME.match(node.name))
        if is_part:
            kids = []
            for m in node.meshes:
                mid = b.material(m.material)
                kids += _mesh_record(unique(m.name or name), m, mid, b.materials[mid].transparent, _L, rep)
            kids += [sub(c) for c in node.children]
            M = A @ world @ L4
            parts.append(_node_bytes(unique(name), M, kids))
            rep.parts.append(name)
            if role == "wheel":
                rep.wheels.append(name)
            return
        if node.meshes:
            kids = []
            for m in node.meshes:
                mid = b.material(m.material)
                kids += _mesh_record(unique(m.name or name), m, mid, b.materials[mid].transparent, _L, rep)
            parts.append(_node_bytes(unique(name), A @ world @ L4, kids))
        for c in node.children:
            top(c, world @ c.matrix, role_seen)

    top(scene, scene.matrix, False)
    if not rep.wheels:
        rep.notes.append("no SVJ::wheel::wheel_xx parts in the mesh: the car has no rotating wheels in the game")
    rep.materials, rep.textures = len(b.materials), len(b.textures)

    out = bytearray(b"sc6969" + struct.pack("<i", KN5_VERSION))
    out += struct.pack("<i", len(b.textures))
    for name, data in b.textures.values():
        out += struct.pack("<i", 1) + _s(name) + struct.pack("<i", len(data)) + data
    out += struct.pack("<i", len(b.materials))
    for m in b.materials:
        out += _s(m.name) + _s(m.shader) + struct.pack("<h", m.blend) + struct.pack("<i", 0)
        out += struct.pack("<i", len(m.props))
        for k, v in m.props.items():
            out += _s(k) + struct.pack("<f", float(v)) + b"\x00" * 36
        out += struct.pack("<i", len(m.textures))
        for sampler, slot, tname in m.textures:
            out += _s(sampler) + struct.pack("<i", slot) + _s(tname)
    out += _node_bytes(model_name, np.eye(4), parts)
    return bytes(out), rep
