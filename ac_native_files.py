"""AC data files with no SVJ home, carried in ``x_assettocorsa.native_files`` so an AC -> SVJ -> AC car keeps them.

``lights.ini`` (light nodes, colours, the light animation file), ``cameras.ini`` (the in-car cameras), ``damage.ini``
(which nodes shake or deform, and from what speed) and ``ai.ini`` (the AI driver's gears, pedals, steering and
look-ahead) describe how the *game* treats the car; the standard has no field for any of them (it has no camera,
light, damage or AI-driver section).  They are copied as parsed sections, keys and values as strings.

Two kinds of value depend on the model's space and are converted: the ``POSITION`` / ``FORWARD`` / ``UP`` of
``cameras.ini`` are points and directions in the KN5's space (X left, Y up, Z forward, origin as the mesh has it), so they
are stored in the SVJ's ISO 8855 axes with the origin of the file (the same placement as the mesh, ``frame`` says which)
and put back in the KN5 space of the new ``car.kn5`` on the way out.  Node names (``NAME=``) are kept: the nodes that
come from AC keep theirs.  A file the game also needs but this does not carry (``lights.ksanim``, the camera and
mirror sub-files) is named in the car's conversion log.
"""
from __future__ import annotations

from pathlib import Path
from typing import Optional

from ac_parsers import parse_ini

FILES = ("lights.ini", "cameras.ini", "damage.ini", "ai.ini")
_VEC_KEYS = {"cameras.ini": {"POSITION", "FORWARD", "UP"}}
_ISO = "ISO_8855"


def _vec(v: str) -> Optional[list]:
    try:
        out = [float(x) for x in str(v).replace("|", ",").split(",")]
    except ValueError:
        return None
    return out if len(out) == 3 else None


def kn5_to_iso(v, translation, point: bool):
    """KN5 model-space point / direction -> ISO 8855 of the file.  ``translation`` is the mesh placement's (``T``)."""
    x, y, z = v
    sx, sy, sz = z, -x, -y                              # SAE J670 (X fwd, Y right, Z down), as the mesh
    if point and translation is not None:
        sx, sz = sx + translation[2], sz - translation[1]
    return [round(sx, 5), round(-sy, 5), round(-sz, 5)]


def collect(data_dir, translation=None, pitch_deg: float = 0.0) -> dict:
    """``{file: {"sections": {SECTION: {KEY: value}}, "frame"?: ...}}`` of the files that exist in ``data_dir``.

    Cameras are converted when the placement is known (a translation, and no pitch); otherwise they are kept in the
    KN5's own space and ``frame`` says ``kn5_model``."""
    out: dict = {}
    for name in FILES:
        f = Path(data_dir) / name
        try:
            text = f.read_text(encoding="utf-8", errors="replace")
        except OSError:
            continue
        secs = parse_ini(text)
        if not secs:
            continue
        entry: dict = {"sections": {s: dict(kv) for s, kv in secs.items()}}
        keys = _VEC_KEYS.get(name)
        if keys:
            ok = translation is not None and not pitch_deg
            for kv in entry["sections"].values():
                for k in keys & set(kv):
                    v = _vec(kv[k])
                    if v is not None and ok:
                        kv[k] = kn5_to_iso(v, translation, point=(k == "POSITION"))
            entry["frame"] = _ISO if ok else "kn5_model"
        out[name] = entry
    return out
