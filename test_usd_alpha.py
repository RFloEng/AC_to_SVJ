import base64
import json
import os
import struct
import zlib

from pxr import Usd, UsdShade

from svj_migrate import gltf2usd


_STAGES = []


def _png():
    raw = b"\x00\xff\x00\x00\x80"
    def chunk(t, d):
        c = struct.pack(">I", len(d)) + t + d
        return c + struct.pack(">I", zlib.crc32(t + d) & 0xFFFFFFFF)
    return (bytes([0x89]) + b"PNG" + bytes([13, 10, 26, 10]) + chunk(b"IHDR", struct.pack(">IIBBBBB", 1, 1, 8, 6, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw)) + chunk(b"IEND", b""))


def _gltf(alpha_mode):
    buf = struct.pack("<9f", 0, 0, 0, 1, 0, 0, 0, 1, 0)
    uri = lambda b: "data:application/octet-stream;base64," + base64.b64encode(b).decode()
    mat = {"pbrMetallicRoughness": {"baseColorTexture": {"index": 0}}, "alphaMode": alpha_mode}
    if alpha_mode == "MASK":
        mat["alphaCutoff"] = 0.4
    return {"asset": {"version": "2.0"}, "scene": 0, "scenes": [{"nodes": [0]}], "nodes": [{"mesh": 0}],
            "meshes": [{"primitives": [{"attributes": {"POSITION": 0}, "material": 0}]}],
            "materials": [mat], "textures": [{"source": 0}],
            "images": [{"uri": "data:image/png;base64," + base64.b64encode(_png()).decode()}],
            "buffers": [{"byteLength": len(buf), "uri": uri(buf)}],
            "bufferViews": [{"buffer": 0, "byteLength": len(buf)}],
            "accessors": [{"bufferView": 0, "componentType": 5126, "count": 3, "type": "VEC3",
                           "min": [0, 0, 0], "max": [1, 1, 0]}]}


def _surface(tmp_path, mode):
    src = tmp_path / f"{mode}.gltf"
    src.write_text(json.dumps(_gltf(mode)))
    dst = str(tmp_path / f"{mode}.usda")
    gltf2usd.convert(str(src), dst, "Car")
    stage = Usd.Stage.Open(dst)
    prim = next(p for p in stage.Traverse() if p.GetName() == "Surface")
    _STAGES.append(stage)                      # a Shader is only valid while its stage lives
    return UsdShade.Shader(prim)


def test_blend_takes_opacity_from_the_texture_alpha(tmp_path):
    sh = _surface(tmp_path, "BLEND")
    assert sh.GetInput("opacity").HasConnectedSource()
    assert sh.GetInput("opacityThreshold") is None or not sh.GetInput("opacityThreshold").Get()


def test_mask_sets_the_cutout_threshold(tmp_path):
    sh = _surface(tmp_path, "MASK")
    assert sh.GetInput("opacity").HasConnectedSource()
    assert abs(sh.GetInput("opacityThreshold").Get() - 0.4) < 1e-6


def test_opaque_ignores_the_alpha_channel(tmp_path):
    sh = _surface(tmp_path, "OPAQUE")
    assert not sh.GetInput("opacity").HasConnectedSource()
    assert sh.GetInput("opacity").Get() == 1.0


def test_clearcoat_specular_cap_and_detail_map_reach_the_usd(tmp_path):
    g = _gltf("OPAQUE")
    g["materials"][0]["extensions"] = {"KHR_materials_clearcoat": {"clearcoatFactor": 0.8, "clearcoatRoughnessFactor": 0.1},
                                       "KHR_materials_specular": {"specularFactor": 0.5}}
    g["materials"][0]["occlusionTexture"] = {"index": 0, "texCoord": 1}
    g["meshes"][0]["primitives"][0]["attributes"]["TEXCOORD_1"] = 1
    g["buffers"][0]["uri"] = "data:application/octet-stream;base64," + base64.b64encode(
        struct.pack("<9f", 0, 0, 0, 1, 0, 0, 0, 1, 0) + struct.pack("<6f", 0, 0, 1, 0, 0, 1)).decode()
    g["buffers"][0]["byteLength"] = 60
    g["bufferViews"] = [{"buffer": 0, "byteLength": 36}, {"buffer": 0, "byteOffset": 36, "byteLength": 24}]
    g["accessors"].append({"bufferView": 1, "componentType": 5126, "count": 3, "type": "VEC2"})
    src = tmp_path / "x.gltf"
    src.write_text(json.dumps(g))
    dst = str(tmp_path / "x.usda")
    gltf2usd.convert(str(src), dst, "Car")
    stage = Usd.Stage.Open(dst)
    sh = UsdShade.Shader(next(p for p in stage.Traverse() if p.GetName() == "Surface"))
    assert abs(sh.GetInput("clearcoat").Get() - 0.8) < 1e-6
    assert 1.2 < sh.GetInput("ior").Get() < 1.4            # factor 0.5 -> F0 0.02 -> ior ~1.38
    assert sh.GetInput("occlusion").HasConnectedSource()
    mesh = next(p for p in stage.Traverse() if p.GetTypeName() == "Mesh")
    assert mesh.GetAttribute("primvars:st1").IsValid()
