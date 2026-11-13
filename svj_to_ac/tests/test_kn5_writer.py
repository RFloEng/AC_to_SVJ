"""KN5 writer: a mesh scene goes out as a car.kn5 that the forward converter's reader reads back."""
from __future__ import annotations

import io
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import kn5_reader as kr                      # noqa: E402
import kn5_writer as kw                      # noqa: E402
import mesh_scene as ms                      # noqa: E402


def _png(color=(200, 30, 30, 255)) -> bytes:
    from PIL import Image
    buf = io.BytesIO()
    Image.new("RGBA", (4, 4), color).save(buf, format="PNG")
    return buf.getvalue()


def _quad(x0=0.0, y0=0.0, z0=0.0, size=0.5, mat=None) -> ms.SceneMesh:
    p = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]], np.float32) * size + [x0, y0, z0]
    uv = np.array([[0, 0], [1, 0], [1, 1], [0, 1]], np.float32)
    return ms.SceneMesh(p, None, uv, np.array([0, 1, 2, 0, 2, 3], np.uint32),
                        mat or ms.SceneMaterial("paint", base_texture=ms.SceneTexture("paint.png", _png())), "quad")


def _scene() -> ms.SceneNode:
    root = ms.SceneNode("root")
    body = ms.SceneNode("SVJ::body::chassis", meshes=[_quad(-1, 0.2, -2, 2)])
    wheel = ms.SceneNode("SVJ::wheel::wheel_fl", matrix=_t(-0.8, 0.3, 0.0), meshes=[_quad(-0.1, -0.1, -0.1, 0.2)])
    other = ms.SceneNode("mirror", matrix=_t(0, 1, -1), meshes=[_quad()])
    root.children = [body, wheel, other]
    return root


def _t(x, y, z) -> np.ndarray:
    m = np.eye(4)
    m[:3, 3] = (x, y, z)
    return m


def _read(tmp_path, data: bytes):
    p = tmp_path / "car.kn5"
    p.write_bytes(data)
    return kr.parse_kn5(p)


def test_parts_are_named_for_ac_and_the_wheel_sits_at_its_centre(tmp_path) -> None:
    data, rep = kw.build_kn5(_scene(), "car", cg_height=0.4, cg_behind_front_axle=1.5)
    m = _read(tmp_path, data)
    top = {c.name: c for c in m.root.children}
    assert set(top) >= {"chassis", "WHEEL_LF", "mirror"}
    assert rep.wheels == ["WHEEL_LF"] and rep.meshes == 3 and rep.triangles == 6
    # svj (X right = -0.8, height 0.3, forward of the front axle 0) -> kn5 (0.8, 0.3 - 0.4, 1.5)
    assert np.array(top["WHEEL_LF"].matrix).reshape(4, 4)[3, :3] == pytest.approx([0.8, -0.1, 1.5], abs=1e-5)


def test_vertices_follow_the_forward_converters_axes(tmp_path) -> None:
    sc = ms.SceneNode("root", meshes=[_quad(1.0, 2.0, 3.0, 1.0)])
    data, _ = kw.build_kn5(sc, "car", cg_height=0.0, cg_behind_front_axle=0.0)
    m = _read(tmp_path, data)
    node = m.root.children[0].children[0]
    # kn5 = (-X, Y, -Z), the node matrix holds the (zero) shift: the quad's corners (1,2,3) .. (2,3,3)
    got = {tuple(np.round(v, 5)) for v in node.positions}
    assert got == {(-1.0, 2.0, -3.0), (-2.0, 2.0, -3.0), (-2.0, 3.0, -3.0), (-1.0, 3.0, -3.0)}
    n = node.normals[0]
    assert abs(n[2]) == pytest.approx(1.0, abs=1e-5)                      # a quad in the XY plane


def test_winding_is_kept(tmp_path) -> None:
    sc = ms.SceneNode("root", meshes=[_quad(size=1.0)])
    data, _ = kw.build_kn5(sc, "car", 0.0, 0.0)
    node = _read(tmp_path, data).root.children[0].children[0]
    a, b, c = (node.positions[i] for i in node.indices[:3])
    src = np.cross(np.array([1, 0, 0]), np.array([1, 1, 0]))              # +Z for the source triangle
    got = np.cross(b - a, c - a)
    # the map (-X, Y, -Z) is a rotation: the face normal is rotated with it, not mirrored
    assert got == pytest.approx(np.array([-src[0], src[1], -src[2]]), abs=1e-5)


def test_materials_pick_ac_shaders(tmp_path) -> None:
    mats = {
        "plain": ms.SceneMaterial("plain", base_texture=ms.SceneTexture("a.png", _png())),
        "nm": ms.SceneMaterial("nm", base_texture=ms.SceneTexture("b.png", _png((1, 2, 3, 255))),
                               normal_texture=ms.SceneTexture("n.png", _png((128, 128, 255, 255)))),
        "cut": ms.SceneMaterial("cut", alpha_mode="MASK", base_texture=ms.SceneTexture("c.png", _png((9, 9, 9, 255)))),
        "glass": ms.SceneMaterial("glass", alpha_mode="BLEND", base_color=(0.2, 0.3, 0.4, 0.3)),
        "colour": ms.SceneMaterial("colour", base_color=(0.9, 0.1, 0.1, 1.0), metallic=1.0, roughness=0.2),
    }
    root = ms.SceneNode("root", meshes=[_quad(mat=m) for m in mats.values()])
    data, rep = kw.build_kn5(root, "car", 0, 0)
    m = _read(tmp_path, data)
    shader = {x.name: x.shader for x in m.materials}
    assert shader == {"plain": "ksPerPixel", "nm": "ksPerPixelNM", "cut": "ksPerPixelAT",
                      "glass": "ksPerPixelAlpha", "colour": "ksPerPixel"}
    by = {x.name: x for x in m.materials}
    assert by["nm"].tx_normal and by["plain"].tx_diffuse
    assert by["colour"].tx_diffuse                       # a 1x1 colour texture stands in for the factor
    assert by["colour"].ks_specular > by["plain"].ks_specular      # metal shines more
    assert by["colour"].ks_specular_exp > by["plain"].ks_specular_exp   # and is smoother
    assert by["cut"].blend_mode == 256 and by["glass"].blend_mode == 1
    assert rep.textures == len(m.textures)


def test_a_mesh_over_65535_vertices_is_split(tmp_path) -> None:
    n = 70000
    rng = np.random.default_rng(1)
    pos = rng.random((n, 3), dtype=np.float32)
    idx = np.arange(n, dtype=np.uint32)[: n // 3 * 3]
    big = ms.SceneMesh(pos, None, rng.random((n, 2), dtype=np.float32), idx, ms.SceneMaterial("m"), "big")
    data, rep = kw.build_kn5(ms.SceneNode("root", meshes=[big]), "car", 0, 0)
    m = _read(tmp_path, data)
    meshes = [c for c in m.root.children[0].children]
    assert len(meshes) >= 2 and all(len(c.positions) <= 65535 for c in meshes)
    assert sum(len(c.indices) for c in meshes) == len(idx)


def test_no_wheel_nodes_is_reported() -> None:
    _, rep = kw.build_kn5(ms.SceneNode("root", meshes=[_quad()]), "car", 0, 0)
    assert rep.wheels == [] and any("no rotating wheels" in n for n in rep.notes)


def test_ac_names() -> None:
    assert kw.ac_name("SVJ::wheel::wheel_rr") == ("WHEEL_RR", "wheel")
    assert kw.ac_name("SVJ::brake::disc_fl") == ("DISC_LF", "disc")
    assert kw.ac_name("SVJ::steering::wheel") == ("STEER_HR", "steer")
    assert kw.ac_name("SVJ::suspension::upright_fl") == ("upright_fl", "part")
    assert kw.ac_name("GEO_BODY") == ("GEO_BODY", "part")


def test_gltf_is_read_back_with_its_transform_and_material(tmp_path) -> None:
    pygltflib = pytest.importorskip("pygltflib")
    pos = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]], np.float32)
    idx = np.array([0, 1, 2], np.uint16)
    blob = pos.tobytes() + idx.tobytes() + b"\x00\x00"
    g = pygltflib.GLTF2(
        scene=0, scenes=[pygltflib.Scene(nodes=[0])],
        nodes=[pygltflib.Node(name="SVJ::wheel::wheel_fr", mesh=0, translation=[0.8, 0.3, 0.0])],
        meshes=[pygltflib.Mesh(primitives=[pygltflib.Primitive(attributes=pygltflib.Attributes(POSITION=0), indices=1, material=0)])],
        materials=[pygltflib.Material(name="red", alphaMode="BLEND",
                                      pbrMetallicRoughness=pygltflib.PbrMetallicRoughness(baseColorFactor=[1, 0, 0, 0.5]))],
        accessors=[pygltflib.Accessor(bufferView=0, componentType=5126, count=3, type="VEC3", max=[1, 1, 0], min=[0, 0, 0]),
                   pygltflib.Accessor(bufferView=1, componentType=5123, count=3, type="SCALAR")],
        bufferViews=[pygltflib.BufferView(buffer=0, byteLength=36), pygltflib.BufferView(buffer=0, byteOffset=36, byteLength=6)],
        buffers=[pygltflib.Buffer(byteLength=len(blob))])
    g.set_binary_blob(blob)
    path = tmp_path / "t.glb"
    g.save_binary(str(path))
    sc = ms.load_scene(path)
    assert ms.count(sc) == {"nodes": 2, "meshes": 1, "triangles": 1}
    node = sc.children[0]
    assert node.name == "SVJ::wheel::wheel_fr" and node.matrix[:3, 3] == pytest.approx([0.8, 0.3, 0.0])
    mesh = node.meshes[0]
    assert mesh.material.alpha_mode == "BLEND" and mesh.material.base_color[3] == pytest.approx(0.5)
    data, rep = kw.build_kn5(sc, "car", 0.4, 1.5)
    assert rep.wheels == ["WHEEL_RF"]


def test_unsupported_format_is_an_error(tmp_path) -> None:
    p = tmp_path / "car.obj"
    p.write_text("v 0 0 0")
    with pytest.raises(ms.SceneError):
        ms.load_scene(p)
