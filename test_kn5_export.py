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


# --- item 5: export report ----------------------------------------------------

def test_export_report():
    # WHEEL_LF puts the "front axle" at AC z=2, which the exporter folds into a
    # translation node; the BODY triangle sits at AC (0,0,0),(1,0,0),(0,0,1).
    axle = N("WHEEL_LF", matrix=[1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 2, 1])
    root = N("ROOT", children=[axle, N("BODY", "mesh"),
                               N("RIM_BLUR_LF", "mesh")])   # dropped: not counted
    rep = {}
    with tempfile.TemporaryDirectory() as t:
        _export(root, Path(t), report=rep)
    assert rep["meshes"] == 1 and rep["triangles"] == 1
    assert rep["variants_dropped"] == 1 and rep["materials"] == 1 and rep["images"] == 0
    assert rep["transforms"] == 2                      # ROOT + WHEEL_LF
    # AC -> glTF: x'=-x, y'=y, z'=2-z  (axis fix + front-axle alignment)
    assert rep["bbox_min"] == [-1.0, 0.0, 1.0], rep["bbox_min"]
    assert rep["bbox_max"] == [0.0, 0.0, 2.0], rep["bbox_max"]
    assert rep["size"] == [1.0, 0.0, 1.0]
    text = K.format_export_report(rep)
    assert "triangles : 1" in text and "1 variants dropped" in text


def test_export_report_verbose_prints():
    import contextlib
    buf = io.StringIO()
    with tempfile.TemporaryDirectory() as t, contextlib.redirect_stdout(buf):
        _export(N("ROOT", children=[N("BODY", "mesh")]), Path(t), verbose=True)
    out = buf.getvalue()
    assert "nodes" in out and "bbox" in out and "materials" in out


# --- SVJ v0.99.2 visual bindings ----------------------------------------------

def test_part_mapping():
    names = ["BODY", "SUSP_LF", "WHEEL_LF", "DISC_LF", "WHEEL_RR", "HUB_RF",
             "WHEEL_LR", "STEER_HR", "STEER_LR"]
    parts = {(b["part"], b["station"]): b for b in K.map_ac_nodes_to_svj_parts(names)}
    assert parts[("chassis", None)]["node"] == "SVJ::body::chassis"
    assert parts[("upright", "FL")]["node"] == "SVJ::suspension::upright_fl"
    assert parts[("upright", "FR")]["ac_name"] == "HUB_RF"      # fallback prefix
    assert parts[("wheel", "FL")]["node"] == "SVJ::wheel::wheel_fl"
    assert parts[("wheel", "RR")]["node"] == "SVJ::wheel::wheel_rr"
    assert parts[("wheel", "RL")]["ac_name"] == "WHEEL_LR"      # LR = Left Rear
    assert parts[("disc", "FL")]["node"] == "SVJ::brake::disc_fl"
    assert ("disc", "FR") not in parts                          # nothing invented


def test_node_rename():
    root = N("ROOT", children=[N("WHEEL_LF", "mesh"), N("DISC_LF", "mesh")])
    with tempfile.TemporaryDirectory() as t:
        g, _ = _export(root, Path(t), node_names={
            "WHEEL_LF": "SVJ::wheel::wheel_fl",
            "DISC_LF": "SVJ::wheel::wheel_fl"})        # duplicate target: skipped
    by = {n.name: n for n in g.nodes}
    assert by["SVJ::wheel::wheel_fl"].extras == {"ac_name": "WHEEL_LF"}
    assert "DISC_LF" in by                                # not renamed onto a duplicate


def test_converter_emits_v0992_bindings():
    """Synthetic car folder -> real build_svj: bindings exist, are named by the
    SVJ::<category>::<id> convention, and the exported GLB contains those nodes."""
    import shutil
    import pygltflib
    from converter import build_svj, read_car_directory, _clean
    src = Path(__file__).parent / "test_car"
    if not src.is_dir():
        return
    with tempfile.TemporaryDirectory() as t:
        car = Path(t) / "synthcar"
        shutil.copytree(src, car)
        tree = N("ROOT", children=[
            N("BODY", "mesh"), N("SUSP_LF", "mesh"), N("WHEEL_LF", "mesh"),
            N("DISC_LF", "mesh"), N("WHEEL_RR", "mesh")])
        (car / "synthcar.kn5").write_bytes(make_kn5(tree, materials=[
            ("MAT", "ksPerPixel", {}, {})]))
        ini, cm, ctrl, dd = read_car_directory(car)
        out = Path(t) / "out"
        svj, log, _ = build_svj(ini, cm, data_dir=dd, ctrl_files=ctrl,
                                glb_output_dir=out)
        svj = _clean(svj)
        glb = out / "meshes" / "synthcar.glb"
        assert glb.is_file(), log
        names = {n.name for n in pygltflib.GLTF2().load(str(glb)).nodes}
    assert svj["_metadata"]["version"] == "0.99.2"
    assert svj["vehicle_info"]["drive_type"] in ("FWD", "RWD", "AWD", "4WD")
    sus = svj["suspension"]
    assert svj["chassis"]["visual"]["node"] == "SVJ::body::chassis"
    assert sus["FL"]["visual"]["node"] == "SVJ::suspension::upright_fl"
    assert sus["FL"]["wheel"]["visual"]["node"] == "SVJ::wheel::wheel_fl"
    assert sus["RR"]["wheel"]["visual"]["node"] == "SVJ::wheel::wheel_rr"
    assert "visual" not in sus["FR"] and "visual" not in sus["FR"]["wheel"]
    bound = [sus["FL"]["visual"]["node"], sus["FL"]["wheel"]["visual"]["node"],
             sus["RR"]["wheel"]["visual"]["node"], svj["chassis"]["visual"]["node"]]
    disc = (sus["FL"].get("brake") or {}).get("disc") or {}
    if "visual" in disc:
        bound.append(disc["visual"]["node"])
    for node in bound:
        assert node in names, (node, sorted(names))     # bindings resolve in the GLB


# --- encrypted KN5s and model resolution --------------------------------------

def _car_nodes(extra=()):
    names = ["BODY", "WHEEL_LF", "WHEEL_RF", "WHEEL_LR", "WHEEL_RR",
             "SUSP_LF", "SUSP_RF", "SUSP_LR", "SUSP_RR", *extra]
    return N("ROOT", children=[N(n, "mesh") for n in names])


def _write(path: Path, tree: N, encrypted=False):
    path.parent.mkdir(parents=True, exist_ok=True)
    blob = make_kn5(tree, materials=[("M", "ksPerPixel", {}, {})])
    path.write_bytes(blob + (K.KN5_ENC_MARKER if encrypted else b""))
    return path


def test_encryption_marker_detected():
    with tempfile.TemporaryDirectory() as t:
        enc = _write(Path(t) / "a.kn5", _car_nodes(), encrypted=True)
        clean = _write(Path(t) / "b.kn5", _car_nodes())
        assert K.is_encrypted_kn5(enc) and not K.is_encrypted_kn5(clean)


def test_resolve_plain_prefers_folder_name():
    with tempfile.TemporaryDirectory() as t:
        car = Path(t) / "mycar"
        _write(car / "other.kn5", _car_nodes(["EXTRA"] * 0))
        want = _write(car / "mycar.kn5", _car_nodes())
        c = K.resolve_car_kn5(car)
        assert c.path == want and not c.skipped_encrypted
        assert K.find_car_kn5(car) == want


def test_resolve_uses_unencrypted_subfolder_copy():
    with tempfile.TemporaryDirectory() as t:
        car = Path(t) / "rfc_car"
        enc = _write(car / "model.kn5", _car_nodes(), encrypted=True)
        clean = _write(car / "unencrypted" / "model_decrypted.kn5", _car_nodes())
        c = K.resolve_car_kn5(car)
        assert c.path == clean, c.message
        assert c.skipped_encrypted == [enc] and "unencrypted copy" in c.reason
        assert K.find_car_kn5(car) == clean
        assert [lbl for lbl, _ in K.find_car_kn5_lods(car)] == ["A"]


def test_resolve_refuses_when_only_encrypted():
    with tempfile.TemporaryDirectory() as t:
        car = Path(t) / "car"
        _write(car / "car.kn5", _car_nodes(), encrypted=True)
        c = K.resolve_car_kn5(car)
        assert c.refused and "UNENCRYPTED" in c.message
        assert K.find_car_kn5(car) is None and K.find_car_kn5_lods(car) == []
        try:
            K.kn5_all_lods_to_glbs(car, Path(t) / "out")
            raise AssertionError("expected EncryptedKn5Error")
        except K.EncryptedKn5Error as e:
            assert "unencrypted" in str(e).lower()


def test_resolve_ignores_accessories_and_unrelated_models():
    with tempfile.TemporaryDirectory() as t:
        car = Path(t) / "car"
        _write(car / "car.kn5", _car_nodes(), encrypted=True)
        # accessory folder: never considered, even though it is a clean KN5
        _write(car / "extension" / "spoiler.kn5", _car_nodes())
        # a clean KN5 of something else: rejected for low node overlap
        other = _write(car / "unencrypted" / "other.kn5",
                       N("ROOT", children=[N(f"X{i}", "mesh") for i in range(9)]))
        c = K.resolve_car_kn5(car)
        assert c.refused
        assert [p for p, _ in c.rejected] == [other]
        assert "node overlap" in c.rejected[0][1]


def test_resolve_override():
    with tempfile.TemporaryDirectory() as t:
        car = Path(t) / "car"
        _write(car / "car.kn5", _car_nodes(), encrypted=True)
        clean = _write(car / "elsewhere" / "x.kn5", N("ROOT", children=[N("Z", "mesh")]))
        assert K.resolve_car_kn5(car, override=clean).path == clean
        assert K.resolve_car_kn5(car, override=car / "car.kn5").refused
        try:
            K.resolve_car_kn5(car, override=car / "missing.kn5")
            raise AssertionError("expected ValueError")
        except ValueError:
            pass


def test_export_refuses_encrypted_file():
    with tempfile.TemporaryDirectory() as t:
        enc = _write(Path(t) / "a.kn5", _car_nodes(), encrypted=True)
        try:
            K.kn5_to_glb(enc)
            raise AssertionError("expected EncryptedKn5Error")
        except K.EncryptedKn5Error:
            pass
        assert K.list_skins(enc).get("error")


def test_skins_and_lods_follow_the_chosen_copy():
    with tempfile.TemporaryDirectory() as t:
        car = Path(t) / "car"
        _write(car / "car.kn5", _car_nodes(), encrypted=True)
        clean = _write(car / "unencrypted" / "car_unencrypted.kn5", _car_nodes())
        _write(car / "unencrypted" / "car_unencrypted_LOD_B.kn5", _car_nodes())
        _write(car / "unencrypted" / "car_LOD_C.kn5", _car_nodes(), encrypted=True)
        (car / "skins" / "red").mkdir(parents=True)
        assert [n for n, _ in K._skin_dirs(clean)] == ["red"]       # skins in car root
        assert [lbl for lbl, _ in K.find_car_kn5_lods(car)] == ["A", "B"]


def test_converter_uses_unencrypted_copy():
    import shutil
    import pygltflib
    from converter import build_svj, read_car_directory, _clean
    src = Path(__file__).parent / "test_car"
    if not src.is_dir():
        return
    with tempfile.TemporaryDirectory() as t:
        car = Path(t) / "synthcar"
        shutil.copytree(src, car)
        _write(car / "model.kn5", _car_nodes(), encrypted=True)
        _write(car / "unencrypted" / "model_decrypted.kn5", _car_nodes())
        ini, cm, ctrl, dd = read_car_directory(car)
        out = Path(t) / "out"
        svj, log, _ = build_svj(ini, cm, data_dir=dd, ctrl_files=ctrl,
                                glb_output_dir=out)
        glbs = list((out / "meshes").glob("*.glb"))
        assert [g.name for g in glbs] == ["model_decrypted.glb"], (glbs, log)
        assert any("skipped (encrypted)" in l for l in log), log
        assert svj["assets"]["meshes"][0]["uri"] == "meshes/model_decrypted.glb"
        # a car with ONLY an encrypted KN5 is refused and explained, never exported
        (car / "unencrypted" / "model_decrypted.kn5").unlink()
        svj2, log2, _ = build_svj(ini, cm, data_dir=dd, ctrl_files=ctrl,
                                  glb_output_dir=Path(t) / "out2")
        assert "assets" not in svj2 and any("UNENCRYPTED" in l for l in log2), log2


# --- crash / damage textures --------------------------------------------------

def test_damage_textures_left_out_by_default():
    mats = [("PAINT", "ksPerPixelMultiMap_damage_dirt", {},
             {"txDiffuse": "skin.png", "txDamage": "skin.png",       # same file: kept
              "txDamageMask": "damage_mask.png"})]                    # only damage: dropped
    tex = [("skin.png", noisy_png()), ("damage_mask.png", noisy_png()),
           ("body_crash_dirt.png", noisy_png()),                      # name token, unused
           ("undamaged_panel.png", noisy_png())]                      # not a token match
    root = N("ROOT", children=[N("A", "mesh")])
    with tempfile.TemporaryDirectory() as t:
        g, _ = _export(root, Path(t), materials=mats, textures=tex)
        rep = {}
        _export(root, Path(t), materials=mats, textures=tex, report=rep)
        g_all, _ = _export(root, Path(t), materials=mats, textures=tex,
                           include_damage_textures=True)
    names = {im.name for im in g.images}
    assert "skin.png" in names                                  # still used as diffuse
    assert "damage_mask.png" not in names and "body_crash_dirt.png" not in names
    assert "undamaged_panel.png" in names
    assert rep["damage_textures_skipped"] == 2 and rep["damage_textures_skipped_bytes"] > 0
    assert {"damage_mask.png", "body_crash_dirt.png"} <= {im.name for im in g_all.images}
    assert "damage textures left out" in K.format_export_report(rep)


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
