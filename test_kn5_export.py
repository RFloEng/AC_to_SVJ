"""
Tests for the KN5 -> GLB exporter (kn5_reader.py).

No Assetto Corsa content is used.  `make_kn5` below is a minimal *encoder* for
the container layout kn5_reader.parse_kn5 decodes, so the tests build their own
tiny synthetic car in a temp dir and the suite needs no AC install.

Run directly (``python test_kn5_export.py``) or via smoke_test.py.
"""
from __future__ import annotations

import io
import struct
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import numpy as np

import kn5_reader as K


# --- synthetic KN5 encoder ----------------------------------------------------

def _s(txt: str) -> bytes:
    b = txt.encode("utf-8")
    return struct.pack("<i", len(b)) + b


IDENTITY = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]


class N:
    """Node spec: kind 'dummy' (type 1) or 'mesh' (type 2)."""

    def __init__(self, name, kind="dummy", children=(), material=0,
                 verts=None, tris=None, matrix=None):
        self.name, self.kind, self.children = name, kind, list(children)
        self.material, self.matrix = material, matrix or IDENTITY
        # default: one triangle
        self.verts = verts if verts is not None else [
            ((0, 0, 0), (0, 1, 0), (0, 0), (1, 0, 0)),
            ((1, 0, 0), (0, 1, 0), (1, 0), (1, 0, 0)),
            ((0, 0, 1), (0, 1, 0), (0, 1), (1, 0, 0)),
        ]
        self.tris = tris if tris is not None else [(0, 1, 2)]


def _node(n: N) -> bytes:
    if n.kind == "dummy":
        out = struct.pack("<i", 1) + _s(n.name) + struct.pack("<i", len(n.children))
        out += bytes([1]) + struct.pack("<16f", *n.matrix)
    else:
        out = struct.pack("<i", 2) + _s(n.name) + struct.pack("<i", len(n.children))
        out += bytes([1]) + bytes([1, 1, 0])
        out += struct.pack("<i", len(n.verts))
        for p, nr, uv, tg in n.verts:
            out += struct.pack("<11f", *p, *nr, *uv, *tg)
        idx = [i for t in n.tris for i in t]
        out += struct.pack("<i", len(idx)) + struct.pack("<%dH" % len(idx), *idx)
        out += struct.pack("<i", n.material) + b"\x00" * 29
    for c in n.children:
        out += _node(c)
    return out


def make_kn5(root: N, materials=(), textures=(), version=5) -> bytes:
    """materials: [(name, shader, {prop: float}, {slot: texname})]
    textures: [(name, bytes)] embedded."""
    out = b"sc6969" + struct.pack("<i", version)
    if version > 5:
        out += b"\x00" * 4
    out += struct.pack("<i", len(textures))
    for name, data in textures:
        out += struct.pack("<i", 1) + _s(name) + struct.pack("<i", len(data)) + data
    out += struct.pack("<i", len(materials))
    for name, shader, props, slots in materials:
        out += _s(name) + _s(shader) + struct.pack("<h", 0)
        if version > 4:
            out += b"\x00" * 4
        out += struct.pack("<i", len(props))
        for k, v in props.items():
            out += _s(k) + struct.pack("<f", v) + b"\x00" * 36
        out += struct.pack("<i", len(slots))
        for slot, tex in slots.items():
            out += _s(slot) + struct.pack("<i", 0) + _s(tex)
    return out + _node(root)


def png_bytes(size=(8, 8), rgb=(200, 30, 20)) -> bytes:
    from PIL import Image
    b = io.BytesIO()
    Image.new("RGB", size, rgb).save(b, "PNG")
    return b.getvalue()


def _export(root: N, tmp: Path, **kw):
    """Write a synthetic kn5, export it, return (pygltflib.GLTF2, glb_bytes)."""
    import pygltflib
    p = tmp / "synth.kn5"
    p.write_bytes(make_kn5(root, materials=kw.pop("materials", [
        ("MAT", "ksPerPixel", {}, {})]), textures=kw.pop("textures", ())))
    glb = K.kn5_to_glb(p, **kw)
    return pygltflib.GLTF2.load_from_bytes(glb), glb


def _names(g) -> set[str]:
    return {n.name for n in g.nodes}


# --- item 1: variants ---------------------------------------------------------

def test_variant_names():
    assert K._is_variant_name("RIM_BLUR_LF")
    assert K._is_variant_name("rim blur lf")
    assert K._is_variant_name("EXT_Rim_Blur")
    assert K._is_variant_name("BODY_DAMAGE")
    assert not K._is_variant_name("UNDAMAGED_PANEL")
    assert not K._is_variant_name("BENTLEY_BADGE")      # old substring "bent"
    assert not K._is_variant_name("BLURRY_LAMP")


def test_lowres_twins():
    names = ["COCKPIT_HR", "COCKPIT_LR", "STEER_HR", "STEER_LR",
             "WHEEL_LR", "SUSP_LR", "DISC_LR", "LONE_LR"]
    assert K.lowres_twins(names) == {"COCKPIT_LR", "STEER_LR"}
    # case-insensitive partner test
    assert K.lowres_twins(["Dash_hr", "Dash_LR"]) == {"Dash_LR"}


def _variant_tree() -> N:
    return N("ROOT", children=[
        N("BODY", "mesh"),
        N("WHEEL_LR", "mesh"),                 # Left Rear: must survive
        N("COCKPIT_HR", "mesh"),
        N("COCKPIT_LR", "mesh"),               # low-res twin: dropped
        N("RIM_BLUR_LF", "mesh", children=[N("BLUR_CHILD", "mesh")]),
        N("BODY_DAMAGE", "mesh"),
        N("UNDAMAGED_PANEL", "mesh"),
    ])


def test_variants_dropped_by_default():
    with tempfile.TemporaryDirectory() as t:
        g, _ = _export(_variant_tree(), Path(t))
    names = _names(g)
    for keep in ("BODY", "WHEEL_LR", "COCKPIT_HR", "UNDAMAGED_PANEL"):
        assert keep in names, keep
    for gone in ("COCKPIT_LR", "RIM_BLUR_LF", "BLUR_CHILD", "BODY_DAMAGE"):
        assert gone not in names, gone
    # no placeholder transparent material is left behind
    assert not any(m.name == "_hidden_ephemeral" for m in g.materials)


def test_keep_variants():
    with tempfile.TemporaryDirectory() as t:
        g, _ = _export(_variant_tree(), Path(t), keep_variants=True)
    names = _names(g)
    for n in ("COCKPIT_LR", "RIM_BLUR_LF", "BLUR_CHILD", "BODY_DAMAGE"):
        assert n in names, n


# --- item 2: livery paint -----------------------------------------------------

def flat_png(rgb, size=(128, 128)) -> bytes:
    """One flat colour. Big enough that flat_colour() does not treat it as a stub."""
    from PIL import Image
    b = io.BytesIO()
    Image.new("RGB", size, rgb).save(b, "PNG")
    blob = b.getvalue()
    assert len(blob) >= 128, len(blob)
    return blob


def noisy_png(size=(64, 64)) -> bytes:
    """A varying (pattern-like) texture."""
    from PIL import Image
    rng = np.random.RandomState(7)
    arr = rng.randint(0, 256, size=(size[1], size[0], 3), dtype=np.uint8)
    b = io.BytesIO()
    Image.fromarray(arr, "RGB").save(b, "PNG")
    return b.getvalue()


def _lin(v8: int) -> float:
    """Independent re-statement of "double in gamma space, clamp, linearise"."""
    c = min(2.0 * v8 / 255.0, 1.0)
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


def _make_car(root: Path, skin_files=None) -> Path:
    """A tiny car folder: car.kn5 + skins/<name>/<file>.  The body is painted by
    a flat txDetail over a shared template diffuse; the rim has MORE triangles
    than the body (to prove ordering is not by size alone); TRIM has a varying
    detail map."""
    body = N("BODY", "mesh", material=0)
    rim = N("RIM_LF", "mesh", material=1, tris=[(0, 1, 2), (0, 2, 1)])
    trim = N("TRIM", "mesh", material=2)
    tree = N("ROOT", children=[body, rim, trim])
    mats = [
        ("EXT_CARPAINT", "ksPerPixelMultiMap", {"useDetail": 1.0},
         {"txDiffuse": "template.png", "txDetail": "metal_detail.png"}),
        ("RIM", "ksPerPixelMultiMap", {"useDetail": 1.0},
         {"txDiffuse": "template.png", "txDetail": "rim_detail.png"}),
        ("TRIM", "ksPerPixelMultiMap",
         {"useDetail": 1.0, "detailUVMultiplier": 10.0},
         {"txDiffuse": "template.png", "txDetail": "grain.png"}),
    ]
    tex = [("template.png", noisy_png()),
           ("metal_detail.png", flat_png((128, 128, 128))),
           ("rim_detail.png", flat_png((120, 120, 120))),
           ("grain.png", noisy_png())]
    kn5_path = root / "car.kn5"
    kn5_path.write_bytes(make_kn5(tree, materials=mats, textures=tex))
    for skin, files in (skin_files or {}).items():
        d = root / "skins" / skin
        d.mkdir(parents=True)
        for fname, data in files.items():
            (d / fname).write_bytes(data)
    return kn5_path


_SKINS = {
    "blue": {"metal_detail.png": flat_png((30, 33, 36)),
             "rim_detail.png": flat_png((200, 200, 200))},
    "red":  {"metal_detail.png": flat_png((126, 1, 0)),
             "rim_detail.png": flat_png((200, 200, 200))},
}


def _mat(g, name):
    return next(m for m in g.materials if m.name == name)


def test_flat_colour_and_tint():
    fc = K.flat_colour(flat_png((126, 1, 0)))
    assert fc is not None and fc[0] == [126, 1, 0]
    assert abs(fc[1][0] - _lin(126)) < 1e-4 and abs(fc[1][2]) < 1e-9
    assert K.flat_colour(noisy_png()) is None          # a pattern is not a colour
    assert K.flat_colour(b"tiny") is None              # stub
    # (148,148,148) doubles past white: the SWATCH keeps the real colour
    fc = K.flat_colour(flat_png((148, 148, 148)))
    assert fc[0] == [148, 148, 148] and fc[1] == [1.0, 1.0, 1.0]


def test_paint_rank_prefers_bodywork():
    assert K.paint_rank("EXT_Carpaint") < K.paint_rank("RIM")
    assert K.paint_rank("Body")[0] == 0
    # an interior copy of the paint sorts after the exterior one in its band
    assert K.paint_rank("EXT_Carpaint") < K.paint_rank("INT_OCC_Carpaint")
    assert K.paint_rank("whatever")[0] == 1
    assert K.paint_rank("RIM_front")[0] == 2


def test_list_skins():
    with tempfile.TemporaryDirectory() as t:
        kn5_path = _make_car(Path(t), _SKINS)
        rep = K.list_skins(kn5_path)
    assert [s["name"] for s in rep["skins"]] == ["blue", "red", "none"]
    red = rep["skins"][1]["colours"]
    # bodywork first although the rim has more triangles
    assert red[0]["material"] == "EXT_CARPAINT" and red[0]["rgb"] == [126, 1, 0]
    assert red[0]["source"] == "skin" and red[0]["tris"] == 1
    assert red[1]["material"] == "RIM" and red[1]["tris"] == 2
    # the varying TRIM detail is a pattern, not a colour: not listed
    assert all(c["material"] != "TRIM" for c in red)
    none = rep["skins"][2]["colours"]
    assert none[0]["source"] == "kn5" and none[0]["rgb"] == [128, 128, 128]
    text = K.format_skin_report(rep)
    assert "skin	red	EXT_CARPAINT	7E0100	1	skin" in text


def test_livery_tint_and_variants():
    with tempfile.TemporaryDirectory() as t:
        root = Path(t)
        kn5_path = _make_car(root, _SKINS)
        import pygltflib
        glb = K.kn5_to_glb(kn5_path, skins=K._find_skins(root))
        g = pygltflib.GLTF2.load_from_bytes(glb)
        glb_none = K.kn5_to_glb(kn5_path, skins=K._find_skins(root),
                                default_skin="none")
        g_none = pygltflib.GLTF2.load_from_bytes(glb_none)
    # base materials follow AC's default = first skin = "blue"
    base = _mat(g, "EXT_CARPAINT").pbrMetallicRoughness.baseColorFactor
    assert all(abs(a - b) < 1e-4 for a, b in zip(base, [_lin(30), _lin(33), _lin(36), 1.0]))
    # a flat detail is the paint, so it must NOT also be exported as AO
    assert _mat(g, "EXT_CARPAINT").occlusionTexture is None
    # a varying detail keeps the AO path and is not tinted
    trim = _mat(g, "TRIM")
    assert trim.occlusionTexture is not None
    assert list(trim.pbrMetallicRoughness.baseColorFactor) == [1.0, 1.0, 1.0, 1.0]
    # variants: Default + each skin; red differs from base, blue does not
    names = [v["name"] for v in g.extensions["KHR_materials_variants"]["variants"]]
    assert names == ["Default", "blue", "red"]
    red = _mat(g, "EXT_CARPAINT_red").pbrMetallicRoughness.baseColorFactor
    assert abs(red[0] - _lin(126)) < 1e-4 and red[1] < 0.01
    assert not any(m.name == "EXT_CARPAINT_blue" for m in g.materials)
    assert "KHR_materials_variants" in g.extensionsUsed
    # default_skin="none": base uses the KN5's embedded grey detail -> factor 1
    b0 = _mat(g_none, "EXT_CARPAINT").pbrMetallicRoughness.baseColorFactor
    assert list(b0) == [1.0, 1.0, 1.0, 1.0]
    # unknown default skin is an error, not a silent fallback
    try:
        with tempfile.TemporaryDirectory() as t:
            kp = _make_car(Path(t), _SKINS)
            K.kn5_to_glb(kp, skins=K._find_skins(Path(t)), default_skin="nope")
        raise AssertionError("expected ValueError")
    except ValueError as e:
        assert "nope" in str(e)


# --- item 3: materials --------------------------------------------------------

def test_blinn_to_roughness():
    assert abs(K.blinn_to_roughness(50, 1.0) - (2 / 52) ** 0.5) < 1e-9
    assert K.blinn_to_roughness(50, 0.0) > K.blinn_to_roughness(50, 1.0)  # matte
    assert K.blinn_to_roughness(1e9, 1.0) == 0.04                          # clamp
    assert 0.04 <= K.blinn_to_roughness(1, 0.02) <= 1.0


def test_material_mapping():
    mats = [
        ("PAINT", "ksPerPixel", {"ksSpecular": 1.0, "ksSpecularEXP": 50.0,
                                 "fresnelMaxLevel": 0.45, "sunSpecular": 12.0,
                                 "sunSpecularEXP": 1500.0}, {}),
        ("MATTE", "ksPerPixel", {"ksSpecular": 0.0, "ksSpecularEXP": 50.0}, {}),
        ("PLAIN", "ksPerPixel", {}, {}),
    ]
    root = N("ROOT", children=[N("A", "mesh", material=0),
                               N("B", "mesh", material=1),
                               N("C", "mesh", material=2)])
    with tempfile.TemporaryDirectory() as t:
        g, _ = _export(root, Path(t), materials=mats)
    paint, matte, plain = (_mat(g, n) for n in ("PAINT", "MATTE", "PLAIN"))
    assert abs(paint.pbrMetallicRoughness.roughnessFactor - (2 / 52) ** 0.5) < 1e-3
    assert abs(matte.pbrMetallicRoughness.roughnessFactor - (2 / 3) ** 0.5) < 1e-3
    assert paint.extensions["KHR_materials_specular"]["specularFactor"] == 0.45
    cc = paint.extensions["KHR_materials_clearcoat"]
    assert cc["clearcoatFactor"] == 0.6
    assert abs(cc["clearcoatRoughnessFactor"] - (2 / 1502) ** 0.5) < 1e-3
    # only paint gets a clearcoat; materials without the properties get no extension
    assert not matte.extensions and not plain.extensions
    assert {"KHR_materials_specular", "KHR_materials_clearcoat"} <= set(g.extensionsUsed)


# --- item 4: robustness -------------------------------------------------------

def test_texture_case_folding():
    big, small = noisy_png((64, 64)), png_bytes((8, 8))
    mats = [("M", "ksPerPixel", {}, {"txDiffuse": "FOO.png"})]
    with tempfile.TemporaryDirectory() as t:
        p = Path(t) / "x.kn5"
        p.write_bytes(make_kn5(N("ROOT", children=[N("A", "mesh")]), materials=mats,
                               textures=[("Foo.png", big), ("foo.PNG", small)]))
        model = K.parse_kn5(p)
        glb = K.kn5_to_glb(p)
    assert [tx.name for tx in model.textures] == ["Foo.png"]      # larger blob survives
    assert model.materials[0].tx_diffuse == "Foo.png"             # slot re-pointed
    import pygltflib
    g = pygltflib.GLTF2.load_from_bytes(glb)
    assert len(g.images) == 1 and _mat(g, "M").pbrMetallicRoughness.baseColorTexture


def test_skin_lookup_case_insensitive():
    skins = {"red": {"METAL_DETAIL.PNG": flat_png((126, 1, 0))}}
    with tempfile.TemporaryDirectory() as t:
        root = Path(t)
        kn5_path = _make_car(root, skins)
        rep = K.list_skins(kn5_path)
        assert K._ci_child(root, "SKINS") == root / "skins"
        assert K._ci_child(root, "nope") is None
    red = rep["skins"][0]["colours"]
    assert red and red[0]["source"] == "skin" and red[0]["rgb"] == [126, 1, 0]


def test_car_folder_lookup_case_insensitive():
    with tempfile.TemporaryDirectory() as t:
        car = Path(t) / "MyCar"
        (car / "SKINS" / "red").mkdir(parents=True)
        (car / "mycar.KN5").write_bytes(make_kn5(N("ROOT"), materials=[]))
        (car / "mycar_lod_b.KN5").write_bytes(make_kn5(N("ROOT"), materials=[]))
        assert K.find_car_kn5(car).name == "mycar.KN5"
        assert [lbl for lbl, _ in K.find_car_kn5_lods(car)] == ["A", "B"]
        assert [n for n, _ in K._find_skins(car)] == ["red"]


def _comp_types(g):
    return [g.accessors[pr.indices].componentType
            for m in g.meshes for pr in m.primitives]


def test_large_mesh_indices():
    import pygltflib
    n = 65540
    verts = [((float(i % 100), 0.0, float(i // 100)), (0, 1, 0), (0.0, 0.0), (1, 0, 0))
             for i in range(n)]
    big = N("BIG", "mesh", verts=verts, tris=[(0, 1, 65535)])
    small = N("SMALL", "mesh")
    with tempfile.TemporaryDirectory() as t:
        g, _ = _export(N("ROOT", children=[big, small]), Path(t))
    types = dict(zip([m.name for m in g.meshes], _comp_types(g)))
    assert types["BIG"] == pygltflib.UNSIGNED_INT       # 65535 is the restart value
    assert types["SMALL"] == pygltflib.UNSIGNED_SHORT
    blob = g.binary_blob()
    acc = next(a for m, a in zip(g.meshes, (g.accessors[pr.indices] for mm in g.meshes
                                            for pr in mm.primitives)) if m.name == "BIG")
    bv = g.bufferViews[acc.bufferView]
    got = np.frombuffer(blob[bv.byteOffset:bv.byteOffset + 12], dtype="<u4")
    assert list(got) == [0, 1, 65535]


ALL_TESTS = [v for k, v in sorted(globals().items()) if k.startswith("test_")]


def run() -> bool:
    """Run every test; print a ✓/✗ line each. Returns True when all pass."""
    ok = True
    for fn in ALL_TESTS:
        try:
            fn()
            print(f"  ✓  {fn.__name__}")
        except Exception as e:                    # noqa: BLE001
            ok = False
            print(f"  ✗  {fn.__name__}: {type(e).__name__}: {e}")
    return ok


if __name__ == "__main__":
    sys.exit(0 if run() else 1)
