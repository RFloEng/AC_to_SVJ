# Vendored from RFloEng/SVJ-standard-vehicle-json, tools/mesh_check.py (commit af68666, Apache-2.0).
# Unchanged except for this header.
"""
SVJ mesh-side binding checker (spec §22.9, §22.10).

`integrity_check.py` checks the SVJ side of the visual bindings. This module opens the glTF / GLB and OpenUSD files the
bindings point at and checks the other side: do the bound nodes exist, are they unambiguous, and does the geometry sit
where the SVJ part says it is. For glTF it reads only the JSON of the asset (node tree, transforms and the min / max of
the POSITION accessors), never the binary buffers, and has no dependencies. OpenUSD assets (spec §22.10) are read with
the optional `usd-core` library (`pip install usd-core`); without it they are skipped with a note. A USD node is a prim
with the attribute `svj:node`, or named `SVJ__category__id`; the stage must author metersPerUnit = 1.

Mesh problems are **appearance** problems: they are reported as warnings, never as errors, and never make the physics
description invalid. (`integrity_check.py --strict` still fails on warnings.)

Checks:
  M1  the asset file exists and is readable glTF 2.0 / GLB or OpenUSD (other formats are skipped with a note)
  M2  every bound node exists (by name, or by the `extras.svj_node` of a node when the name was changed)
  M3  a bound node name is unique in the asset (otherwise the binding is ambiguous)
  M4  `extras.svj_node`, when present, agrees with the node name
  M5  nodes named `SVJ::...` that no part is bound to (advisory)
  M6  units: the size of the geometry matches the size of the SVJ parts (a factor of 10, 100 or 1000 means mm / cm / m)
  M7  axes: the geometry fits the declared `coordinate_system`; when another convention fits better, it says which
  M8  placement: the SVJ position of a part lies inside (or near) the bounding box of its node; a common shift of all
      parts is reported once, as a shift of the asset against the SVJ origin

Usage (through integrity_check.py):
    python tools/integrity_check.py vehicle.svj.json --meshes [--strict]
"""

import json
import math
import os
import re
import struct

NODE_PREFIX = "SVJ::"
CATEGORY_SKIP_ORPHAN = ("helper", "lod")
_GLB_MAGIC = b"glTF"
_CHUNK_JSON = 0x4E4F534A


class MeshError(ValueError):
    pass


# ── reading ───────────────────────────────────────────────────────────────────
def read_gltf(path):
    """The JSON part of a .glb or .gltf file as a dict."""
    try:
        with open(path, "rb") as f:
            data = f.read()
    except FileNotFoundError:
        raise MeshError("file not found")
    except OSError as e:
        raise MeshError(f"cannot read: {e}")
    if data[:4] == _GLB_MAGIC:
        if len(data) < 20:
            raise MeshError("truncated GLB")
        offset = 12
        while offset + 8 <= len(data):
            clen, ctype = struct.unpack("<II", data[offset:offset + 8])
            if ctype == _CHUNK_JSON:
                try:
                    return json.loads(data[offset + 8:offset + 8 + clen].decode("utf-8"))
                except (UnicodeDecodeError, json.JSONDecodeError) as e:
                    raise MeshError(f"GLB JSON chunk is not valid JSON: {e}")
            offset += 8 + clen
        raise MeshError("GLB has no JSON chunk")
    try:
        return json.loads(data.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError):
        raise MeshError("not a GLB and not a glTF JSON file")


# ── small matrix helpers (4x4 row-major lists) ────────────────────────────────
def _ident():
    return [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]


def _mm(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def _apply(m, p):
    return [m[i][0] * p[0] + m[i][1] * p[1] + m[i][2] * p[2] + m[i][3] for i in range(3)]


def _trs(t, q, s):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    r = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
         [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
         [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    m = _ident()
    for i in range(3):
        for j in range(3):
            m[i][j] = r[i][j] * s[j]
        m[i][3] = t[i]
    return m


def _local(node):
    if isinstance(node.get("matrix"), list) and len(node["matrix"]) == 16:
        m = node["matrix"]                                   # glTF stores matrices column-major
        return [[float(m[j * 4 + i]) for j in range(4)] for i in range(4)]
    return _trs(node.get("translation") or [0, 0, 0], node.get("rotation") or [0, 0, 0, 1], node.get("scale") or [1, 1, 1])


def _euler_xyz(rx, ry, rz):
    """Rotation matrix of a three.js Euler angle with order XYZ: Rx * Ry * Rz."""
    cx, sx, cy, sy, cz, sz = math.cos(rx), math.sin(rx), math.cos(ry), math.sin(ry), math.cos(rz), math.sin(rz)
    rxm = [[1, 0, 0], [0, cx, -sx], [0, sx, cx]]
    rym = [[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]]
    rzm = [[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]]
    mul = lambda a, b: [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    return mul(mul(rxm, rym), rzm)


def asset_transform(entry):
    """The `x_transform` of an `assets.meshes[]` entry (position, rotation as XYZ Euler, scale) as a 4x4 matrix."""
    xf = (entry or {}).get("x_transform")
    if not isinstance(xf, dict):
        return _ident()
    r = _euler_xyz(*(xf.get("rotation") or [0, 0, 0])[:3])
    s = xf.get("scale") or [1, 1, 1]
    p = xf.get("position") or [0, 0, 0]
    m = _ident()
    for i in range(3):
        for j in range(3):
            m[i][j] = r[i][j] * s[j]
        m[i][3] = p[i]
    return m


# ── node tree ─────────────────────────────────────────────────────────────────
def node_boxes(gltf):
    """World-space bounding box [min, max] of every node's subtree, from the accessor bounds, or None for a node
    without geometry. Result: {node index: box or None}."""
    nodes = gltf.get("nodes") or []
    meshes, accessors = gltf.get("meshes") or [], gltf.get("accessors") or []
    children = {i: [c for c in (n.get("children") or []) if isinstance(c, int) and 0 <= c < len(nodes)] for i, n in enumerate(nodes)}
    has_parent = {c for cs in children.values() for c in cs}
    world, own = {}, {}

    def walk(i, parent, depth=0):
        if i in world or depth > 64:
            return
        world[i] = _mm(parent, _local(nodes[i]))
        for c in children[i]:
            walk(c, world[i], depth + 1)

    for i in range(len(nodes)):
        if i not in has_parent:
            walk(i, _ident())
    for i in range(len(nodes)):
        walk(i, _ident())                                 # nodes in a cycle or missed

    def corners(box, m):
        lo, hi = box
        pts = [_apply(m, [x, y, z]) for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]
        return [min(p[k] for p in pts) for k in range(3)], [max(p[k] for p in pts) for k in range(3)]

    for i, n in enumerate(nodes):
        box = None
        mi = n.get("mesh")
        if isinstance(mi, int) and 0 <= mi < len(meshes):
            for prim in meshes[mi].get("primitives") or []:
                ai = (prim.get("attributes") or {}).get("POSITION")
                if isinstance(ai, int) and 0 <= ai < len(accessors):
                    a = accessors[ai]
                    if isinstance(a.get("min"), list) and isinstance(a.get("max"), list) and len(a["min"]) == 3 and len(a["max"]) == 3 and i in world:
                        b = corners((a["min"], a["max"]), world[i])
                        box = b if box is None else ([min(box[0][k], b[0][k]) for k in range(3)], [max(box[1][k], b[1][k]) for k in range(3)])
        own[i] = box

    memo = {}

    def subtree(i, depth=0):
        if i in memo:
            return memo[i]
        box = own.get(i)
        if depth < 64:
            for c in children[i]:
                b = subtree(c, depth + 1)
                if b is not None:
                    box = b if box is None else ([min(box[0][k], b[0][k]) for k in range(3)], [max(box[1][k], b[1][k]) for k in range(3)])
        memo[i] = box
        return box

    return {i: subtree(i) for i in range(len(nodes))}


# ── coordinate conventions ────────────────────────────────────────────────────
def _axis(text):
    m = re.fullmatch(r"\s*([+-]?)([XYZxyz])\s*", str(text))
    if not m:
        return None
    v = [0.0, 0.0, 0.0]
    v["XYZ".index(m.group(2).upper())] = -1.0 if m.group(1) == "-" else 1.0
    return v


def _cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def convention(up, forward, handedness="right"):
    """Rows of the matrix that maps asset axes to SAE J670 (x forward, y right, z down), or None."""
    u, f = _axis(up), _axis(forward)
    if u is None or f is None or sum(a * b for a, b in zip(u, f)) != 0:
        return None
    r = _cross(f, u) if str(handedness).lower() != "left" else _cross(u, f)
    return [f, r, [-c for c in u]]


CANDIDATES = (("SAE J670 axes", None), ("Y-up, -Z forward (Blender export)", ("Y", "-Z")), ("Y-up, +Z forward (glTF front)", ("Y", "Z")),
              ("Z-up, +X forward", ("Z", "X")))


def _to_sae(box, rows):
    pts = [[sum(rows[i][k] * p[k] for k in range(3)) for i in range(3)] for p in
           ([x, y, z] for x in (box[0][0], box[1][0]) for y in (box[0][1], box[1][1]) for z in (box[0][2], box[1][2]))]
    return [min(p[k] for p in pts) for k in range(3)], [max(p[k] for p in pts) for k in range(3)]


def _gap(p, box):
    """Distance from point p to the box (0 inside)."""
    return math.sqrt(sum(max(box[0][k] - p[k], 0.0, p[k] - box[1][k]) ** 2 for k in range(3)))


def _diag(box):
    return math.sqrt(sum((box[1][k] - box[0][k]) ** 2 for k in range(3)))


def _median(v):
    s = sorted(v)
    n = len(s)
    return s[n // 2] if n % 2 else (s[n // 2 - 1] + s[n // 2]) / 2.0


# ── the checks ────────────────────────────────────────────────────────────────
class _Index:
    """What the checks need from an asset, whatever its format: names maps a binding name to the keys of the nodes (or
    prims) that carry it, extras the fallback map (glTF `extras.svj_node`), boxes the world box of each key in metres,
    present the (name, fallback name) of every SVJ node found, for the orphan check."""

    def __init__(self):
        self.names, self.extras, self.boxes, self.present = {}, {}, {}, []
        self.info = {}          # key -> {"physics": [...], "mass": [...], "collision": bool, "animated": bool}: data other than shape and look


class MeshSkip(MeshError):
    """The asset cannot be checked here (a missing optional library): a note, not a warning."""


GLTF_EXT = (".glb", ".gltf")
USD_EXT = (".usd", ".usda", ".usdc", ".usdz")
_USD_NAME = re.compile(r"^SVJ__([a-z]+)__(.+)$")


def _index_gltf(path, ref, uri, warnings):
    gltf = read_gltf(path)
    ix = _Index()
    nodes = gltf.get("nodes") or []
    for i, n in enumerate(nodes):
        name = n.get("name") if isinstance(n.get("name"), str) else None
        sv = (n.get("extras") or {}).get("svj_node") if isinstance(n.get("extras"), dict) else None
        sv = sv if isinstance(sv, str) else None
        if name:
            ix.names.setdefault(name, []).append(i)
        if sv:
            ix.extras.setdefault(sv, []).append(i)
            if name and name.startswith(NODE_PREFIX) and name != sv:
                warnings.append(f"[mesh '{ref}'] node '{name}' has extras.svj_node '{sv}': name and extras disagree")
        if name and name.startswith(NODE_PREFIX) or sv:
            ix.present.append((name if name and name.startswith(NODE_PREFIX) else sv, sv))
    ix.boxes = node_boxes(gltf)
    return ix


def _usd_non_appearance(prim, Usd, UsdGeom):
    """What a prim and its descendants carry besides shape and look (spec 22.10, rule 8): physics schemas and attributes,
    masses, collision shapes, and animated transforms. All of it may only be a copy of data that the SVJ JSON has first."""
    info = {"physics": [], "mass": [], "collision": False, "animated": False}
    for p in Usd.PrimRange(prim):
        schemas = list(p.GetAppliedSchemas()) + ([p.GetTypeName()] if p.GetTypeName().startswith("Physics") else [])
        attrs = [a for a in p.GetAuthoredAttributes() if a.GetName().startswith("physics:")]
        if schemas and any(s.startswith("Physics") for s in schemas) or attrs:
            info["physics"] += [s for s in schemas if s.startswith("Physics")] or ["physics:*"]
        m = p.GetAttribute("physics:mass")
        if m and m.HasAuthoredValue() and isinstance(m.Get(), (int, float)):
            info["mass"].append(float(m.Get()))
        if "PhysicsCollisionAPI" in schemas or "PhysicsMeshCollisionAPI" in schemas:
            info["collision"] = True
        xf = UsdGeom.Xformable(p)
        if xf and xf.TransformMightBeTimeVarying():
            info["animated"] = True
    return info


def _index_usd(path, ref, uri, warnings):
    """Index of an OpenUSD asset (spec 22.10): prims found by `svj:node` or by the prim name SVJ__category__id."""
    try:
        from pxr import Usd, UsdGeom
    except ImportError:
        raise MeshSkip("an OpenUSD asset; reading it needs the usd-core library (pip install usd-core), so its nodes are not checked")
    if not os.path.exists(path):
        raise MeshError("file not found")
    try:
        stage = Usd.Stage.Open(path)
    except Exception as e:                                           # pxr raises its own error types
        raise MeshError(f"not readable as OpenUSD: {e}")
    if stage is None:
        raise MeshError("not readable as OpenUSD")
    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    if not stage.HasAuthoredMetadata("metersPerUnit"):
        warnings.append(f"[mesh '{ref}'] the stage {uri} does not author metersPerUnit (OpenUSD readers then assume 0.01 m); spec 22.10 requires 1")
        mpu = 1.0
    elif abs(mpu - 1.0) > 1e-9:
        warnings.append(f"[mesh '{ref}'] the stage {uri} has metersPerUnit = {mpu:g}; spec 22.10 requires 1 (the geometry is checked in metres)")
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    ix = _Index()
    for prim in stage.Traverse():
        attr = prim.GetAttribute("svj:node")
        logical = attr.Get() if attr and attr.HasAuthoredValue() else None
        logical = logical if isinstance(logical, str) else None
        m = _USD_NAME.match(prim.GetName())
        by_name = f"SVJ::{m.group(1)}::{m.group(2)}" if m else None
        if not (logical or by_name):
            continue
        key = str(prim.GetPath())
        if logical and by_name and logical != by_name:
            warnings.append(f"[mesh '{ref}'] prim {key} is named for '{by_name}' but its svj:node is '{logical}': name and attribute disagree")
        for lg in {x for x in (logical, by_name) if x}:
            ix.names.setdefault(lg, []).append(key)
        ix.present.append((logical or by_name, None))
        ix.info[key] = _usd_non_appearance(prim, Usd, UsdGeom)
        rng = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        ix.boxes[key] = None if rng.IsEmpty() else ([rng.GetMin()[k] * mpu for k in range(3)], [rng.GetMax()[k] * mpu for k in range(3)])
    return ix


def _check_preview(ref, prev, base_dir, nodes, warnings):
    """The glTF preview of a USD asset exists and carries every bound node (spec 22.10)."""
    if os.path.splitext(prev)[1].lower() not in GLTF_EXT:
        warnings.append(f"[mesh '{ref}'] preview '{prev}' is not a .glb or .gltf file")
        return
    ppath = os.path.normpath(os.path.join(base_dir, prev))
    try:
        pix = _index_gltf(ppath, ref, prev, warnings)
    except MeshError as e:
        warnings.append(f"[mesh '{ref}'] preview {prev}: {e}" + (" (not generated yet? tools/usd2gltf.py writes it)" if not os.path.exists(ppath) else ""))
        return
    for node in sorted(nodes):
        if node not in pix.names and node not in pix.extras:
            warnings.append(f"[mesh '{ref}'] preview {prev} does not contain the bound node '{node}' (regenerate it from the USD asset)")


def _check_copy_of_json(data, binding, loc, node, uri, info, warnings, notes):
    """Spec 22.10, rule 8: physics data and animation in an asset may only be copies of what the JSON has first."""
    ids = binding.get("ids") or set()
    chassis = data.get("chassis") or {}
    part = chassis if "chassis" in ids else next((b for b in (chassis.get("mass_bodies") or []) if isinstance(b, dict) and b.get("id") in ids), None)
    jmass = (part or {}).get("mass_total" if "chassis" in ids else "mass")
    if info["physics"] and part is None:
        warnings.append(f"[{loc}] '{node}' in {uri} carries physics data ({', '.join(sorted(set(info['physics'])))}) but the JSON has no body for this part: the JSON must have it first")
    for m in info["mass"]:
        if not isinstance(jmass, (int, float)):
            warnings.append(f"[{loc}] '{node}' in {uri} carries a mass ({m:g} kg) that the JSON does not have for this part")
        elif abs(m - jmass) > 1e-6 * max(1.0, abs(jmass)):
            warnings.append(f"[{loc}] the mass in {uri} for '{node}' ({m:g} kg) differs from the JSON ({jmass:g} kg); the JSON wins")
    if info["collision"] and part is not None and not part.get("collision"):
        warnings.append(f"[{loc}] '{node}' in {uri} carries a collision shape that the JSON has no `collision` proxy for (spec 28)")
    if info["animated"]:
        moved = {c.get("body_b") for c in (data.get("constraints") or []) if isinstance(c, dict) and c.get("type") == "joint"}
        if ids & moved:
            notes.append(f"[{loc}] '{node}' is animated in {uri} and the JSON moves this part by a joint; the JSON wins, the asset's animation is ignored")
        else:
            warnings.append(f"[{loc}] '{node}' is animated in {uri} but the JSON has no joint that moves this part: an animation must be in the JSON first (joint, poses[])")


def check_meshes(data, bindings, base_dir):
    """(errors, warnings, notes) for the asset side of `bindings` (the list of integrity_check.collect_bindings).
    errors is always empty: mesh problems are appearance warnings (see module docstring)."""
    warnings, notes = [], []
    meshes = {m["id"]: m for m in (data.get("assets") or {}).get("meshes", []) if isinstance(m, dict) and "id" in m}
    if not meshes:
        return [], warnings, ["--meshes: the file declares no assets.meshes; nothing to check"]
    declared = (data.get("_metadata") or {}).get("coordinate_system")
    declared_rows = convention(declared.get("up"), declared.get("forward"), declared.get("handedness", "right")) if isinstance(declared, dict) else None

    by_mesh = {}
    for b in bindings:
        v = b["visual"]
        ref = v.get("mesh_ref") or (next(iter(meshes)) if len(meshes) == 1 else None)
        if ref in meshes and isinstance(v.get("node"), str):
            by_mesh.setdefault(ref, []).append(b)

    for ref, entry in meshes.items():
        uri = entry.get("uri", "")
        ext = os.path.splitext(uri)[1].lower()
        fmt = entry.get("format") or ("usd" if ext in USD_EXT else "gltf" if ext in GLTF_EXT else None)
        if fmt not in ("gltf", "usd"):
            notes.append(f"[mesh '{ref}'] '{uri}' is not a glTF or OpenUSD asset; its nodes are not checked")
            continue
        path = os.path.normpath(os.path.join(base_dir, uri))
        prev = entry.get("preview")
        if prev and fmt == "usd":
            _check_preview(ref, prev, base_dir, {b["visual"]["node"] for b in by_mesh.get(ref, [])}, warnings)
        elif prev:
            notes.append(f"[mesh '{ref}'] `preview` only applies to OpenUSD assets; ignored for {uri}")
        try:
            ix = (_index_usd if fmt == "usd" else _index_gltf)(path, ref, uri, warnings)
        except MeshSkip as e:
            notes.append(f"[mesh '{ref}'] {uri}: {e}")
            continue
        except MeshError as e:
            warnings.append(f"[mesh '{ref}'] {uri}: {e}" + (" (a placeholder path?)" if not os.path.exists(path) else ""))
            continue
        bound, items = set(), []
        for b in by_mesh.get(ref, []):
            node, loc = b["visual"]["node"], b["location"]
            bound.add(node)
            idx = ix.names.get(node)
            if not idx and ix.extras.get(node):
                idx = ix.extras[node]
                notes.append(f"[{loc}] node '{node}' found through extras.svj_node (the node was renamed in the asset)")
            if not idx:
                warnings.append(f"[{loc}] node '{node}' does not exist in {uri}")
                continue
            if len(idx) > 1:
                warnings.append(f"[{loc}] node '{node}' occurs {len(idx)} times in {uri}: the binding is ambiguous")
            box = ix.boxes.get(idx[0])
            if b.get("pos") is not None and box is not None and b["visual"].get("placement", "rigid") == "rigid":
                items.append((loc, node, b["pos"], box))
            if ix.info.get(idx[0]):
                _check_copy_of_json(data, b, loc, node, uri, ix.info[idx[0]], warnings, notes)
        for name, via in ix.present:
            m = re.match(r"^SVJ::([a-z0-9_]+)::", name or "")
            if m and m.group(1) not in CATEGORY_SKIP_ORPHAN and name not in bound and via not in bound:
                warnings.append(f"[mesh '{ref}'] node '{name}' in {uri} is not bound by any part of this file")
        _check_geometry(ref, uri, entry, items, declared_rows, warnings, notes)
    return [], warnings, notes


def _check_geometry(ref, uri, entry, items, declared_rows, warnings, notes):
    if len(items) < 1:
        return
    xf = asset_transform(entry)
    scaled = [(loc, node, pos, ([_apply(xf, [x, y, z]) for x in (b[0][0], b[1][0]) for y in (b[0][1], b[1][1]) for z in (b[0][2], b[1][2])])) for loc, node, pos, b in items]
    boxes = [(loc, node, pos, ([min(p[k] for p in pts) for k in range(3)], [max(p[k] for p in pts) for k in range(3)])) for loc, node, pos, pts in scaled]

    # M6: units. The spread of the box centres against the spread of the SVJ positions, independent of the axes
    if len(boxes) >= 3:
        c = [[(b[0][k] + b[1][k]) / 2 for k in range(3)] for _, _, _, b in boxes]
        span = lambda pts: math.sqrt(sum((max(p[k] for p in pts) - min(p[k] for p in pts)) ** 2 for k in range(3)))
        sv, sm = span([p for _, _, p, _ in boxes]), span(c)
        if sv > 0.5 and sm > 0:
            ratio = sm / sv
            if not 0.5 <= ratio <= 2.0:
                hint = {1000.0: "millimetres", 100.0: "centimetres", 0.001: "a size 1000 times too small", 0.01: "a size 100 times too small"}
                near = min(hint, key=lambda k: abs(math.log10(ratio / k)))
                why = f" (looks like {hint[near]})" if abs(math.log10(ratio / near)) < 0.15 else ""
                warnings.append(f"[mesh '{ref}'] the geometry in {uri} is {ratio:.3g} times the size of the SVJ parts{why}; set x_transform.scale or re-export in metres")
                return

    # M7 + M8: axes and placement. Score each convention by how many parts have their SVJ position inside, or near, their box
    def score(rows, shift=(0.0, 0.0, 0.0)):
        ok, gaps = 0, []
        for loc, node, pos, box in boxes:
            sb = _to_sae(box, rows) if rows else box
            sb = ([sb[0][k] + shift[k] for k in range(3)], [sb[1][k] + shift[k] for k in range(3)])
            g = _gap(pos, sb)
            gaps.append(g)
            ok += g <= 0.10 + 0.15 * _diag(sb)
        return ok / len(boxes), gaps

    cands = [("the declared coordinate_system", declared_rows)] if declared_rows else [("SAE J670 axes", None)]
    cands += [(n, convention(*a)) for n, a in CANDIDATES if a and convention(*a) != declared_rows]
    scores = [(score(rows)[0], i, name, rows) for i, (name, rows) in enumerate(cands)]
    first = scores[0]

    def outliers(rows):
        sae = [(loc, node, pos, _to_sae(box, rows) if rows else box) for loc, node, pos, box in boxes]
        for (loc, node, pos, b), g in list(zip(sae, score(rows)[1])):
            if g > 0.10 + 0.15 * _diag(b):
                c = [(b[0][k] + b[1][k]) / 2 for k in range(3)]
                warnings.append(f"[{loc}] node '{node}': the SVJ position ({pos[0]:+.2f}, {pos[1]:+.2f}, {pos[2]:+.2f}) m lies {g:.2f} m outside the node's "
                                f"box (centre {c[0]:+.2f}, {c[1]:+.2f}, {c[2]:+.2f}) in {uri}")

    if first[0] >= 0.7:                       # the convention fits most parts: name the few that do not
        outliers(first[3])
        return
    best = max(scores, key=lambda s: (s[0], -s[1]))
    if best[0] >= 0.7 and best[1] != 0:
        warnings.append(f"[mesh '{ref}'] the geometry in {uri} fits {best[2]} better than {first[2]}: check _metadata.coordinate_system")
        return
    rows = best[3] if best[0] > first[0] else first[3]
    sae = [(loc, node, pos, _to_sae(box, rows) if rows else box) for loc, node, pos, box in boxes]
    shift = tuple(_median([pos[k] - (b[0][k] + b[1][k]) / 2 for _, _, pos, b in sae]) for k in range(3))
    if len(boxes) >= 3 and max(abs(s) for s in shift) > 0.05 and score(rows, shift)[0] >= 0.7:
        # the asset sits at -shift against the SVJ origin (SAE axes); x_transform.position is in the asset's own axes
        fix = [sum(rows[i][k] * shift[i] for i in range(3)) for k in range(3)] if rows else list(shift)
        warnings.append(f"[mesh '{ref}'] the geometry in {uri} appears shifted by ({-shift[0]:+.3f}, {-shift[1]:+.3f}, {-shift[2]:+.3f}) m "
                        f"(SAE x, y, z) against the SVJ origin; set x_transform.position to about ({fix[0]:+.3f}, {fix[1]:+.3f}, {fix[2]:+.3f}) "
                        f"(asset axes) or re-export with the origin at the front axle")
        return
    outliers(rows)
