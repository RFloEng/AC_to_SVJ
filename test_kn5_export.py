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
