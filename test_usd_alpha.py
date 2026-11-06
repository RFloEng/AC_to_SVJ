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


def test_clearcoat_and_specular_cap_reach_the_usd_but_the_detail_map_does_not(tmp_path):
    g = _gltf("OPAQUE")
    g["materials"][0]["extensions"] = {"KHR_materials_clearcoat": {"clearcoatFactor": 0.8, "clearcoatRoughnessFactor": 0.1},
                                       "KHR_materials_specular": {"specularFactor": 0.5}}
    g["materials"][0]["occlusionTexture"] = {"index": 0, "texCoord": 1}
    src = tmp_path / "x.gltf"
    src.write_text(json.dumps(g))
    dst = str(tmp_path / "x.usda")
    gltf2usd.convert(str(src), dst, "Car")
    stage = Usd.Stage.Open(dst)
    sh = UsdShade.Shader(next(p for p in stage.Traverse() if p.GetName() == "Surface"))
    assert abs(sh.GetInput("clearcoat").Get() - 0.8) < 1e-6
    assert 1.2 < sh.GetInput("ior").Get() < 1.4            # factor 0.5 -> F0 0.02 -> ior ~1.38
    assert not sh.GetInput("occlusion")                    # a tiled detail map is not ambient occlusion


def test_base_colour_factor_tints_the_texture(tmp_path):
    g = _gltf("OPAQUE")
    g["materials"][0]["pbrMetallicRoughness"]["baseColorFactor"] = [1.0, 0.67, 0.1, 1.0]
    src = tmp_path / "t.gltf"
    src.write_text(json.dumps(g))
    dst = str(tmp_path / "t.usda")
    gltf2usd.convert(str(src), dst, "Car")
    stage = Usd.Stage.Open(dst)
    tx = UsdShade.Shader(next(p for p in stage.Traverse() if p.GetName() == "tex_diffuseColor"))
    sc = tx.GetInput("scale").Get()
    assert abs(sc[1] - 0.67) < 1e-6 and abs(sc[2] - 0.1) < 1e-6


def test_opaque_materials_get_images_without_alpha(tmp_path):
    from PIL import Image
    import io
    # 1x1 PNG with alpha 0: the RGB must survive in the USD's image
    buf = io.BytesIO()
    Image.new("RGBA", (1, 1), (120, 120, 120, 0)).save(buf, "PNG")
    g = _gltf("OPAQUE")
    g["images"][0]["uri"] = "data:image/png;base64," + base64.b64encode(buf.getvalue()).decode()
    src = tmp_path / "o.gltf"
    src.write_text(json.dumps(g))
    gltf2usd.convert(str(src), str(tmp_path / "o.usda"), "Car")
    files = list((tmp_path / "o_tex").glob("*.png"))
    assert files and all(Image.open(f).mode == "RGB" for f in files)
    # a blended material keeps its alpha
    g = _gltf("BLEND")
    g["images"][0]["uri"] = "data:image/png;base64," + base64.b64encode(buf.getvalue()).decode()
    src = tmp_path / "b.gltf"
    src.write_text(json.dumps(g))
    gltf2usd.convert(str(src), str(tmp_path / "b.usda"), "Car")
    assert all(Image.open(f).mode == "RGBA" for f in (tmp_path / "b_tex").glob("*.png"))
