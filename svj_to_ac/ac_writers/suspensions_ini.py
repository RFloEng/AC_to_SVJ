"""Writer for ``data/suspensions.ini``.

Six AC ``TYPE`` values (0..5), each with its own pickup-key set, plus
per-axle ``half_track`` / ``rolling_radius`` plumbing for the pickup-
coordinate convention (see :mod:`coords` and
``ai_handoff/05_PITFALLS.md``).

Sourcing order, per field:

1. ``x_assettocorsa._ac_native["suspensions"]`` (pass-through) -- exact
   reproduction of the source AC INI sections.
2. ``x_assettocorsa.raw["suspensions"][axle]`` scalar overrides.
3. Canonical SVJ per-corner blocks (``spring``, ``damper``, ``arb``,
   ``alignment``, ``wheel``, ``static_setup``) -- so non-AC-sourced
   SVJs still produce a drivable car.
4. Pickup points come from ``doc.suspension.corners[*].topology``
   walked through :class:`AxleContext` + :mod:`coords`. ``doc.chassis``
   supplies track widths; ``doc.tires`` supplies rolling radius.

Panhard / Watts disambiguation: for TYPE=3 (``solid_axle``) AC permits
either a ``PANHARD_CAR/AXLE`` pair *or* a ``WATTS_CAR/AXLE`` pair,
never both. If SVJ supplies both, Panhard wins and Watts geometry is
appended as a ``;`` comment block.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Mapping, Sequence

from coords import svj_to_ac_point
from svj_loader import SvjDoc

from . import (
    format_section,
    format_value,
    format_vec3,
    get_ac_native_ini,
    join_sections,
    render_sections_from_native,
    write_text,
)
from ._axle_context import AxleContext, resolve_axle

FILENAME = "suspensions.ini"

SCALAR_KEYS: tuple[str, ...] = (
    "BASEY",
    "TRACK",
    "ROD_LENGTH",
    "HUB_MASS",
    "STATIC_CAMBER",
    "TOE_OUT",
    "CASTER",
    "KINGPIN",
    "TYRE_STATIC_OFFSET",
    "SPRING_RATE",
    "PROGRESSIVE_SPRING_RATE",
    "BUMP_STOP_RATE",
    "BUMPSTOP_UP",
    "BUMPSTOP_DN",
    "PACKER_RANGE",
    "DAMP_BUMP",
    "DAMP_FAST_BUMP",
    "DAMP_FAST_BUMPTHRESHOLD",
    "DAMP_REBOUND",
    "DAMP_FAST_REBOUND",
    "DAMP_FAST_REBOUNDTHRESHOLD",
)


def write(doc: SvjDoc, out_dir: Path) -> Path:
    return write_text(Path(out_dir) / FILENAME, _render(doc, out_dir=Path(out_dir)))


def _render(doc: SvjDoc, *, out_dir: Path | None = None) -> str:
    native = get_ac_native_ini(doc, "suspensions")
    if native is not None:
        return render_sections_from_native(native)
    return _render_from_svj(doc, out_dir=out_dir)


def _render_from_svj(doc: SvjDoc, *, out_dir: Path | None) -> str:
    sections: list[str] = []
    sections.append(format_section("HEADER", {"VERSION": 2}))
    basic = _basic_block(doc)
    if basic:
        sections.append(format_section("BASIC", basic))

    for axle in ("FRONT", "REAR"):
        ctx = resolve_axle(doc, axle)
        block = _axle_block(doc, axle, ctx, out_dir=out_dir)
        sections.append(_render_axle_section(axle, block))

    arb_front = _scalar_for_axle(doc, "FRONT", "arb_rate") or _canonical_scalar_default(
        doc, "FRONT", "ARB", resolve_axle(doc, "FRONT")) or 0
    arb_rear = _scalar_for_axle(doc, "REAR", "arb_rate") or _canonical_scalar_default(
        doc, "REAR", "ARB", resolve_axle(doc, "REAR")) or 0
    if arb_front or arb_rear:
        sections.append(format_section("ARB", {"FRONT": arb_front, "REAR": arb_rear}))

    return join_sections(sections)


def _basic_block(doc: SvjDoc) -> dict[str, Any]:
    """[BASIC]: the wheelbase and the share of the weight on the front axle."""
    wb = doc.chassis.wheelbase
    cg = doc.chassis.center_of_gravity
    out: dict[str, Any] = {}
    if wb:
        out["WHEELBASE"] = round(float(wb), 4)
        fl = doc.suspension.corners.get("FL") or doc.suspension.corners.get("FR")
        front_x = fl.position[0] if fl is not None and len(fl.position) == 3 else 0.0
        if cg and len(cg) == 3 and 0 < float(wb):
            out["CG_LOCATION"] = round(1.0 - (float(front_x) - float(cg[0])) / float(wb), 4)
    return out


def _render_axle_section(axle: str, block: dict[str, Any]) -> str:
    head = f"[{axle}]"
    lines: list[str] = [head]
    lines.append(f"TYPE={format_value(block['TYPE'])}")
    for key in SCALAR_KEYS:
        if key in block and block[key] is not None:
            lines.append(f"{key}={format_value(block[key])}")
    for k, v in block.get("_pickups", {}).items():
        lines.append(f"{k}={v}")
    for comment_line in block.get("_trailing_comments", []):
        lines.append(f"; {comment_line}")
    return "\n".join(lines)


def _axle_block(doc: SvjDoc, axle: str, ctx: AxleContext, *, out_dir: Path | None) -> dict[str, Any]:
    block: dict[str, Any] = {"TYPE": ctx.ac_type}
    ac_raw = (doc.assetto.raw.get("suspensions", {}) if doc.assetto else {}) or {}
    axle_ext = (ac_raw.get(axle) or {}) if isinstance(ac_raw, dict) else {}
    for key in SCALAR_KEYS:
        v = axle_ext.get(key) if isinstance(axle_ext, dict) else None
        if v is None:
            v = _canonical_scalar_default(doc, axle, key, ctx)
        if v is not None:
            block[key] = v
    pickups, trailing = _emit_pickups(doc, axle, ctx, out_dir=out_dir)
    block["_pickups"] = pickups
    block["_trailing_comments"] = trailing
    return block


# --------------------------------------------------------------------------- #
# Canonical SVJ -> AC scalar resolver                                         #
# --------------------------------------------------------------------------- #


def _mean(vals):
    vals = [v for v in vals if v is not None]
    return sum(vals) / len(vals) if vals else None


def _corner_blocks(doc: SvjDoc, axle: str, name: str) -> list:
    keys = ("FL", "FR") if axle == "FRONT" else ("RL", "RR")
    out = []
    for k in keys:
        c = doc.suspension.corners.get(k)
        b = c.raw.get(name) if c is not None and isinstance(c.raw, dict) else None
        if isinstance(b, dict):
            out.append(b)
    return out


def _canonical_scalar_default(doc: SvjDoc, axle: str, key: str, ctx: AxleContext) -> Any:
    """SVJ value for an AC scalar of ``suspensions.ini`` (None: leave the key out).

    Reads the corner blocks (v0.95 ``suspension`` or the view of a unified file), so a
    file from another simulator still gives a drivable car.  Camber goes out in degrees
    (``STATIC_CAMBER``), a damper curve as AC's two slopes and a threshold.
    """
    springs, dampers = _corner_blocks(doc, axle, "spring"), _corner_blocks(doc, axle, "damper")
    wheels, aligns = _corner_blocks(doc, axle, "wheel"), _corner_blocks(doc, axle, "alignment")
    stops = [d.get("bumpstops") for d in dampers if isinstance(d.get("bumpstops"), dict)]

    if key == "TRACK":
        return doc.chassis.track_front if axle == "FRONT" else doc.chassis.track_rear
    if key == "BASEY":
        # AC: wheel radius + BASEY = height of the CG
        cg = doc.chassis.center_of_gravity
        return round(ctx.rolling_radius + float(cg[2]), 4) if cg and len(cg) == 3 else None
    if key == "HUB_MASS":
        ms = [w.get("mass") for w in wheels if w.get("mass") is not None]
        if not ms:
            keys = ("FL", "FR") if axle == "FRONT" else ("RL", "RR")
            ms = [doc.chassis.mass_unsprung_per_corner[k] for k in keys
                  if k in doc.chassis.mass_unsprung_per_corner]
        m = _mean([float(v) for v in ms])
        return None if m is None else round(2 * m, 3)      # per axle, both wheels
    if key == "ARB":
        return _mean([float(a["bar_rate"]) for a in _corner_blocks(doc, axle, "arb")
                      if a.get("bar_rate") is not None])
    if key == "SPRING_RATE":
        return _mean([float(sp["rate"]) for sp in springs if sp.get("rate") is not None])
    if key == "ROD_LENGTH":
        return _mean([float(sp["rod_length"]) for sp in springs if sp.get("rod_length")])
    if key == "PROGRESSIVE_SPRING_RATE":
        return _mean([float(sp["progressive_factor"]) for sp in springs
                      if sp.get("progressive_factor")])
    if key == "STATIC_CAMBER":
        c = _mean([float(a["camber"]) for a in aligns if a.get("camber") is not None])
        return None if c is None else round(math.degrees(c), 4)
    if key == "TOE_OUT":
        return _mean([float(a["toe"]) for a in aligns if a.get("toe") is not None])
    if key == "CASTER":
        return _mean([float(a["caster"]) for a in aligns if a.get("caster") is not None])
    if key == "KINGPIN":
        return _mean([float(a["kpi"]) for a in aligns if a.get("kpi") is not None])
    if key == "BUMP_STOP_RATE":
        return _mean([float(b["rate"]) for b in stops if b.get("rate")])
    if key == "BUMPSTOP_UP":
        return _mean([float(b["up"]) for b in stops if b.get("up") is not None])
    if key == "BUMPSTOP_DN":
        return _mean([float(b["down"]) for b in stops if b.get("down") is not None])
    if key == "PACKER_RANGE":
        ss = _corner_blocks(doc, axle, "static_setup")
        return _mean([float(x["x_packer_range"]) for x in ss if x.get("x_packer_range")])
    if key.startswith("DAMP_"):
        which = "bump" if "BUMP" in key else "rebound"
        part = "thr" if key.endswith("THRESHOLD") else "fast" if "FAST" in key else "slow"
        vals = []
        for d in dampers:
            p = _damper_params(d.get(f"{which}_curve"))
            if p and p.get(part) is not None:
                vals.append(p[part])
        m = _mean(vals)
        return None if m is None else round(m, 5)
    return None


def _damper_params(curve) -> dict | None:
    """AC's damper (slow slope, fast slope, threshold speed) from a force-against-speed
    curve of the wheel: the first segment gives the slow slope, the first change of slope the
    threshold and the segment after it the fast slope; a straight curve has no fast part."""
    pts = sorted((float(p[0]), float(p[1])) for p in (curve or [])
                 if isinstance(p, (list, tuple)) and len(p) >= 2)
    if len(pts) > 1 and abs(pts[0][0]) < 1e-12:
        origin = pts[0]
    else:
        origin = (0.0, 0.0)
        pts = [origin] + pts
    segs = []
    for (v0, f0), (v1, f1) in zip(pts, pts[1:]):
        if v1 - v0 > 1e-9:
            segs.append((v0, v1, (f1 - f0) / (v1 - v0)))
    if not segs:
        return None
    slow = segs[0][2]
    out = {"slow": slow, "fast": None, "thr": None}
    for v0, v1, k in segs[1:]:
        if abs(k - slow) > 0.05 * max(abs(slow), 1e-9):
            out["thr"], out["fast"] = v0, k
            break
    return out


def _scalar_for_axle(doc: SvjDoc, axle: str, key: str) -> Any:
    ac_raw = (doc.assetto.raw.get("suspensions", {}) if doc.assetto else {}) or {}
    axle_ext = (ac_raw.get(axle) or {}) if isinstance(ac_raw, dict) else {}
    return axle_ext.get(key) if isinstance(axle_ext, dict) else None


# --------------------------------------------------------------------------- #
# Pickup generation per system type                                           #
# --------------------------------------------------------------------------- #


def _emit_pickups(doc: SvjDoc, axle: str, ctx: AxleContext, *, out_dir: Path | None) -> tuple[dict[str, str], list[str]]:
    pickups: dict[str, str] = {}
    trailing: list[str] = []
    corner_key = "FL" if axle == "FRONT" else "RL"
    corner = doc.suspension.corners.get(corner_key)
    if corner is None:
        return pickups, trailing
    top = corner.topology or {}
    links = top.get("links") or []
    upright = top.get("upright") or {}
    hardpoints_svj = (upright.get("hardpoints") or {}) if isinstance(upright, dict) else {}

    st = ctx.system_type
    if st == "double_wishbone":
        _emit_dwb(pickups, hardpoints_svj, links, ctx)
    elif st == "macpherson":
        _emit_strut(pickups, hardpoints_svj, links, ctx)
    elif st == "multi_link":
        _emit_multilink(pickups, hardpoints_svj, links, ctx)
    elif st == "solid_axle":
        _emit_solid_axle(pickups, trailing, hardpoints_svj, links, ctx, out_dir=out_dir)
    elif st == "trailing_arm":
        _emit_trailing_arm(pickups, hardpoints_svj, links, ctx)
    elif st == "semi_trailing_arm":
        _emit_semi_trailing_arm(pickups, hardpoints_svj, links, ctx)
    return pickups, trailing


def _pickup_value(svj_point: Sequence[float], ctx: AxleContext) -> str:
    """AC pickup: measured from the wheel centre, X inboard-positive, Y up, Z forward."""
    sx, sy, sz = float(svj_point[0]), float(svj_point[1]), float(svj_point[2])
    ax = ctx.lateral_to_ac(sy)
    if ctx.wheel_center is not None:
        wx, _, wz = ctx.wheel_center
        ay, az = -(sz - wz), sx - wx
    else:                                   # no wheel centre known: the old convention
        _, ay, az = svj_to_ac_point((sx, sy, sz))
        ay = ay + ctx.z_offset()
    return format_vec3([round(ax, 6), round(ay, 6), round(az, 6)])


def _link_by_name(links, *names):
    for link in links:
        n = str((link.get("name") or "")).lower()
        for needle in names:
            if needle in n:
                return link
    return None


def _emit_dwb(pickups, hardpoints, links, ctx):
    upper = _link_by_name(links, "upper_wishbone", "upper_arm", "upper_control")
    lower = _link_by_name(links, "lower_wishbone", "lower_arm", "lower_control")
    tie = _link_by_name(links, "tie_rod", "steering")
    if upper:
        ib = upper.get("inboard_points") or []
        if len(ib) >= 2:
            pickups["WBCAR_TOP_FRONT"] = _pickup_value(ib[0], ctx)
            pickups["WBCAR_TOP_REAR"] = _pickup_value(ib[1], ctx)
    if lower:
        ib = lower.get("inboard_points") or []
        if len(ib) >= 2:
            pickups["WBCAR_BOTTOM_FRONT"] = _pickup_value(ib[0], ctx)
            pickups["WBCAR_BOTTOM_REAR"] = _pickup_value(ib[1], ctx)
    if tie:
        ib = tie.get("inboard_points") or []
        if ib:
            pickups["WBCAR_STEER"] = _pickup_value(ib[0], ctx)
    if "upper_ball_joint" in hardpoints:
        pickups["WBTYRE_TOP"] = _pickup_value(hardpoints["upper_ball_joint"], ctx)
    if "lower_ball_joint" in hardpoints:
        pickups["WBTYRE_BOTTOM"] = _pickup_value(hardpoints["lower_ball_joint"], ctx)
    if "steering_tie_rod_end" in hardpoints:
        pickups["WBTYRE_STEER"] = _pickup_value(hardpoints["steering_tie_rod_end"], ctx)


def _emit_strut(pickups, hardpoints, links, ctx):
    lower = _link_by_name(links, "lower_wishbone", "lower_arm", "lower_control", "lower")
    if lower:
        ib = lower.get("inboard_points") or []
        if len(ib) >= 2:
            pickups["WBCAR_BOTTOM_FRONT"] = _pickup_value(ib[0], ctx)
            pickups["WBCAR_BOTTOM_REAR"] = _pickup_value(ib[1], ctx)
    if "lower_ball_joint" in hardpoints:
        pickups["WBTYRE_BOTTOM"] = _pickup_value(hardpoints["lower_ball_joint"], ctx)
    strut = _link_by_name(links, "strut")
    if strut:
        ib = strut.get("inboard_points") or []
        if ib:
            pickups["STRUT_CAR"] = _pickup_value(ib[0], ctx)
        ob = strut.get("outboard_point")
        if ob:
            pickups["STRUT_TYRE"] = _pickup_value(ob, ctx)
    tie = _link_by_name(links, "tie_rod", "steering")
    if tie:
        ib = tie.get("inboard_points") or []
        if ib:
            pickups["WBCAR_STEER"] = _pickup_value(ib[0], ctx)
        if "steering_tie_rod_end" in hardpoints:
            pickups["WBTYRE_STEER"] = _pickup_value(hardpoints["steering_tie_rod_end"], ctx)


def _emit_multilink(pickups, hardpoints, links, ctx):
    _emit_dwb(pickups, hardpoints, links, ctx)
    toe = _link_by_name(links, "toe")
    if toe:
        ib = toe.get("inboard_points") or []
        if ib:
            pickups["WBCAR_TIE"] = _pickup_value(ib[0], ctx)


def _emit_solid_axle(pickups, trailing, hardpoints, links, ctx, *, out_dir):
    link_idx = 0
    for link in links:
        name = str(link.get("name", "")).lower()
        if "panhard" in name or "watts" in name:
            continue
        ib = link.get("inboard_points") or []
        ob = link.get("outboard_point")
        if ib and ob and link_idx < 4:
            pickups[f"LINK_{link_idx}_CAR"] = _pickup_value(ib[0], ctx)
            pickups[f"LINK_{link_idx}_AXLE"] = _pickup_value(ob, ctx)
            link_idx += 1

    panhard = _link_by_name(links, "panhard")
    watts = _link_by_name(links, "watts")
    if panhard and watts:
        _log_warning(out_dir,
                     "solid_axle has both panhard_rod and watts_link; emitted PANHARD,"
                     " stashed WATTS geometry as comments.")
    if panhard:
        ib = panhard.get("inboard_points") or []
        ob = panhard.get("outboard_point")
        if ib:
            pickups["PANHARD_CAR"] = _pickup_value(ib[0], ctx)
        if ob:
            pickups["PANHARD_AXLE"] = _pickup_value(ob, ctx)
        if watts:
            ib_w = watts.get("inboard_points") or []
            ob_w = watts.get("outboard_point")
            if ib_w:
                trailing.append(f"WATTS_CAR={_pickup_value(ib_w[0], ctx)}")
            if ob_w:
                trailing.append(f"WATTS_AXLE={_pickup_value(ob_w, ctx)}")
    elif watts:
        ib_w = watts.get("inboard_points") or []
        ob_w = watts.get("outboard_point")
        if ib_w:
            pickups["WATTS_CAR"] = _pickup_value(ib_w[0], ctx)
        if ob_w:
            pickups["WATTS_AXLE"] = _pickup_value(ob_w, ctx)


def _emit_trailing_arm(pickups, hardpoints, links, ctx):
    arm = _link_by_name(links, "trailing")
    if arm:
        ib = arm.get("inboard_points") or []
        ob = arm.get("outboard_point")
        if ib:
            pickups["TRAIL_CAR"] = _pickup_value(ib[0], ctx)
        if ob:
            pickups["TRAIL_TYRE"] = _pickup_value(ob, ctx)


def _emit_semi_trailing_arm(pickups, hardpoints, links, ctx):
    arm = _link_by_name(links, "semi_trailing", "semi-trailing")
    if arm:
        ib = arm.get("inboard_points") or []
        ob = arm.get("outboard_point")
        if ib:
            pickups["SEMI_TRAIL_CAR"] = _pickup_value(ib[0], ctx)
        if ob:
            pickups["SEMI_TRAIL_TYRE"] = _pickup_value(ob, ctx)


def _log_warning(out_dir: Path | None, message: str) -> None:
    if out_dir is None:
        return
    log_path = Path(out_dir).parent / "conversion_log.txt"
    try:
        with log_path.open("a", encoding="utf-8") as f:
            f.write(f"WARNING [suspensions]: {message}\n")
    except OSError:
        pass


__all__ = ["FILENAME", "write"]
