"""
Find the tyres of the user's own Assetto Corsa library that behave like a given tyre.

AC has no tyre-size catalogue and no way to ship a tyre on its own: a car's grip lives in its
`tyres.ini`.  When a car comes from another simulator (or from an SVJ file) the Pacejka
coefficients can be turned back into AC's parameters only approximately, but the library of
an installed AC holds hundreds of real, tested tyre models.  This module

* scans the cars folder the user points at and describes every tyre section by what it does
  (peak grip, cornering stiffness, load and camber sensitivity, post-peak fall-off, ...)
  computed from AC's own force model, and by its size,
* describes a target tyre the same way, from an SVJ tyre set (its Pacejka block or the AC
  parameters it was converted from) or from another car in the library, and
* ranks the library by how alike they are.

Nothing here modifies a car: `tyres_ini_sections` returns the text of the chosen sections.
"""
from __future__ import annotations

import json
import math
import re
from dataclasses import asdict
from pathlib import Path
from typing import Callable, Iterable, Optional

import numpy as np

from ac_parsers import marked_size_for, parse_ini, parse_marked_tyre_sizes
from tire_lab import ACTyreParams, ac_fx, ac_fy, parse_tyre_section

CACHE_VERSION = 2
LOADS_N = (2000.0, 3500.0, 5000.0, 7000.0)        # vertical loads the tyres are compared at
_REF = 1                                           # index of the reference load (3500 N)
_ALPHA = np.radians(np.linspace(0.0, 20.0, 401))
_KAPPA = np.linspace(0.0, 0.4, 401)

# what is compared, with the weight of each term in the distance
FEATURE_WEIGHTS = {
    "mu_y": 3.0, "mu_y_lo": 1.0, "mu_y_hi": 1.0,
    "c_y": 2.0, "alpha_pk": 1.0, "falloff_y": 1.0,
    "mu_x": 2.0, "mu_x_lo": 0.5, "mu_x_hi": 0.5,
    "c_x": 1.0, "kappa_pk": 0.5, "falloff_x": 0.5,
    "camber": 0.3,
}

ForceFn = Callable[..., np.ndarray]


# ── describing a tyre by what it does ─────────────────────────────────────────────

def curve_features(fy: ForceFn, fx: ForceFn) -> dict:
    """Behaviour of a tyre from its force functions.

    `fy(alpha, Fz, gamma)` and `fx(kappa, Fz, gamma)` return newtons (any sign convention:
    magnitudes are used).  Returns the peak grip coefficients at three loads, the stiffness
    per newton, the slip of the peak, how much force remains well past the peak and the
    camber thrust.
    """
    out: dict = {}
    for tag, fz in (("", LOADS_N[_REF]), ("_lo", LOADS_N[0]), ("_hi", LOADS_N[3])):
        f = np.abs(np.asarray(fy(_ALPHA, fz, 0.0), float))
        out["mu_y" + tag] = float(f.max() / fz)
        g = np.abs(np.asarray(fx(_KAPPA, fz, 0.0), float))
        out["mu_x" + tag] = float(g.max() / fz)
    fz = LOADS_N[_REF]
    f = np.abs(np.asarray(fy(_ALPHA, fz, 0.0), float))
    k = int(np.argmax(f))
    out["alpha_pk"] = float(_ALPHA[k])
    a_small = np.radians(0.5)
    out["c_y"] = float(abs(fy(np.array([a_small]), fz, 0.0)[0]) / a_small / fz)
    out["falloff_y"] = float(f[-1] / max(f[k], 1e-9))
    g = np.abs(np.asarray(fx(_KAPPA, fz, 0.0), float))
    k = int(np.argmax(g))
    out["kappa_pk"] = float(_KAPPA[k])
    k_small = 0.01
    out["c_x"] = float(abs(fx(np.array([k_small]), fz, 0.0)[0]) / k_small / fz)
    out["falloff_x"] = float(g[-1] / max(g[k], 1e-9))
    a1, gam = np.radians(1.0), np.radians(-3.0)
    thrust = (np.asarray(fy(np.array([a1]), fz, gam), float)[0]
              - np.asarray(fy(np.array([a1]), fz, 0.0), float)[0])
    out["camber"] = float(abs(thrust) / fz / abs(gam))
    return out


def ac_features(p: ACTyreParams) -> dict:
    """Features of a tyre from its AC parameters (AC's own force model)."""
    return curve_features(lambda a, fz, g: ac_fy(a, fz, g, p), lambda k, fz, g: ac_fx(k, fz, g, p))


def _coef(d: dict, name: str, default: float = 0.0) -> float:
    v = d.get(name)
    return float(v) if isinstance(v, (int, float)) else default


def mf_pure_fy(alpha, fz, gamma, lat: dict, fz0: float):
    """Pacejka pure-slip lateral force (MF 6.2 equations; absent coefficients are zero, so an
    MF 5.2 block works too)."""
    alpha = np.asarray(alpha, float)
    dfz = (fz - fz0) / fz0
    sh = _coef(lat, "pHy1") + _coef(lat, "pHy2") * dfz + _coef(lat, "pHy3") * gamma
    sv = fz * ((_coef(lat, "pVy1") + _coef(lat, "pVy2") * dfz)
               + (_coef(lat, "pVy3") + _coef(lat, "pVy4") * dfz) * gamma)
    ay = alpha + sh
    c = _coef(lat, "pCy1", 1.3)
    mu = (_coef(lat, "pDy1") + _coef(lat, "pDy2") * dfz) * (1.0 - _coef(lat, "pDy3") * gamma ** 2)
    d = mu * fz
    e = (_coef(lat, "pEy1") + _coef(lat, "pEy2") * dfz) * (
        1.0 + _coef(lat, "pEy5") * gamma ** 2
        - (_coef(lat, "pEy3") + _coef(lat, "pEy4") * gamma) * np.sign(ay))
    pky2 = max(_coef(lat, "pKy2", 1.5), 1e-3)
    k = (_coef(lat, "pKy1") * fz0 * math.sin(2.0 * math.atan(fz / (pky2 * fz0)))
         * (1.0 - _coef(lat, "pKy3") * abs(gamma)))
    b = k / (c * d if abs(c * d) > 1e-6 else 1e-6)
    e = np.minimum(e, 1.0)
    bx = b * ay
    return d * np.sin(c * np.arctan(bx - e * (bx - np.arctan(bx)))) + sv


def mf_pure_fx(kappa, fz, gamma, lon: dict, fz0: float):
    """Pacejka pure-slip longitudinal force (MF 6.2 equations)."""
    kappa = np.asarray(kappa, float)
    dfz = (fz - fz0) / fz0
    kx = kappa + _coef(lon, "pHx1") + _coef(lon, "pHx2") * dfz
    c = _coef(lon, "pCx1", 1.6)
    mu = (_coef(lon, "pDx1") + _coef(lon, "pDx2") * dfz) * (1.0 - _coef(lon, "pDx3") * gamma ** 2)
    d = mu * fz
    e = (_coef(lon, "pEx1") + _coef(lon, "pEx2") * dfz + _coef(lon, "pEx3") * dfz ** 2) * (
        1.0 - _coef(lon, "pEx4") * np.sign(kx))
    kk = fz * (_coef(lon, "pKx1") + _coef(lon, "pKx2") * dfz) * math.exp(_coef(lon, "pKx3") * dfz)
    b = kk / (c * d if abs(c * d) > 1e-6 else 1e-6)
    e = np.minimum(e, 1.0)
    bx = b * kx
    sv = fz * (_coef(lon, "pVx1") + _coef(lon, "pVx2") * dfz)
    return d * np.sin(c * np.arctan(bx - e * (bx - np.arctan(bx)))) + sv


def svj_set_curves(tyre_set: dict) -> Optional[tuple]:
    """(fy, fx) force functions of a tyre set of an SVJ file, or None when it has no force
    model to read.

    A set converted from AC carries the AC parameters it came from (`_source_params`): those
    are used as they are.  Otherwise the Pacejka block (`pacejka`, else `pacejka_mf52`) is
    evaluated at the reference load of the set.
    """
    for key in ("pacejka_mf52", "pacejka"):
        blk = tyre_set.get(key)
        src = blk.get("_source_params") if isinstance(blk, dict) else None
        if isinstance(src, dict):
            fields = {k: v for k, v in src.items() if k in ACTyreParams.__dataclass_fields__}
            return _ac_curves(ACTyreParams(**fields))
    for key in ("pacejka", "pacejka_mf52"):
        blk = tyre_set.get(key)
        if not isinstance(blk, dict):
            continue
        lat, lon = blk.get("lateral"), blk.get("longitudinal")
        if not isinstance(lat, dict) or not isinstance(lon, dict):
            continue
        ref = tyre_set.get("reference") or {}
        fz0 = float(ref.get("load") or blk.get("fnomin") or lat.get("FNOMIN") or 4000.0)
        return ((lambda a, fz, g: mf_pure_fy(a, fz, g, lat, fz0)),
                (lambda k, fz, g: mf_pure_fx(k, fz, g, lon, fz0)))
    return None


def _ac_curves(p: ACTyreParams) -> tuple:
    return (lambda a, fz, g: ac_fy(a, fz, g, p)), (lambda k, fz, g: ac_fx(k, fz, g, p))


def entry_curves(entry: dict) -> tuple:
    """(fy, fx) of a library entry."""
    return _ac_curves(ACTyreParams(**entry["params"]))


def svj_set_features(tyre_set: dict) -> Optional[dict]:
    """Features of a tyre set of an SVJ file, or None when it has no force model to read."""
    curves = svj_set_curves(tyre_set)
    return None if curves is None else curve_features(*curves)


def comparison_png(a: tuple, b: tuple, a_label: str, b_label: str) -> bytes:
    """Lateral and longitudinal force of two tyres at three loads, as a PNG: the target
    (solid) against a candidate (dashed)."""
    import io

    import matplotlib
    matplotlib.use("Agg")
    from matplotlib.figure import Figure
    fig = Figure(figsize=(10, 4), dpi=100)
    ax1, ax2 = fig.subplots(1, 2)
    colours = {2000.0: "#1f77b4", 3500.0: "#2ca02c", 5000.0: "#d62728"}
    for fz, col in colours.items():
        for (fy, fx), ls, lab in ((a, "-", a_label), (b, "--", b_label)):
            ax1.plot(np.degrees(_ALPHA), np.abs(fy(_ALPHA, fz, 0.0)), ls, color=col,
                     label=f"{lab}, {int(fz)} N" if fz == 3500.0 else None)
            ax2.plot(_KAPPA * 100, np.abs(fx(_KAPPA, fz, 0.0)), ls, color=col)
    ax1.set_xlabel("slip angle (deg)")
    ax1.set_ylabel("lateral force (N)")
    ax2.set_xlabel("slip ratio (%)")
    ax2.set_ylabel("longitudinal force (N)")
    ax1.legend(fontsize=7)
    fig.suptitle("2000 / 3500 / 5000 N   (solid: target, dashed: candidate)", fontsize=9)
    for ax in (ax1, ax2):
        ax.grid(alpha=0.3)
    fig.tight_layout()
    buf = io.BytesIO()
    fig.savefig(buf, format="png")
    return buf.getvalue()


# ── size ──────────────────────────────────────────────────────────────────────────

def size_of_svj_set(tyre_set: dict) -> dict:
    dims = tyre_set.get("dimensions") or {}
    rim = tyre_set.get("rim") or {}
    width = dims.get("section_width")
    overall = dims.get("overall_diameter")
    rim_d = rim.get("diameter") or (dims.get("rim_diameter_code") or 0) * 0.0254 or None
    return {"width": width, "radius": overall / 2.0 if overall else None,
            "rim_radius": rim_d / 2.0 if rim_d else None,
            "size_code": tyre_set.get("size_code")}


def size_distance(a: dict, b: dict) -> Optional[float]:
    """0 for the same width, outer radius and rim; about 1 for 20 % apart.  None when the
    target gives no size."""
    parts = []
    for key, tol in (("width", 0.10), ("radius", 0.05), ("rim_radius", 0.04)):
        x, y = a.get(key), b.get(key)
        if x and y:
            parts.append(abs(x - y) / (0.5 * (x + y)) / tol)
    if not parts:
        return None
    return float(np.mean(parts)) / 2.0


# ── the library ───────────────────────────────────────────────────────────────────

def _entries_of_car(car: Path) -> list:
    from acd_reader import open_car
    loaded, data_dir, _ = open_car(car)
    text = next((v for k, v in loaded.items() if k.lower() == "tyres.ini"), None)
    if not text:
        return []
    ini = parse_ini(text)
    marked = parse_marked_tyre_sizes(text)
    out = []
    for section in ini:
        if not re.fullmatch(r"(FRONT|REAR)(_\d+)?", section):
            continue
        axle = "front" if section.startswith("FRONT") else "rear"
        sec = ini[section]
        try:
            p = parse_tyre_section(ini, section, axle, data_dir)
            feats = ac_features(p)
        except Exception:                                      # noqa: BLE001
            continue
        width = _num(sec.get("WIDTH"))
        radius = _num(sec.get("RADIUS"))
        rim = _num(sec.get("RIM_RADIUS"))
        mk = marked_size_for(marked, section)
        out.append({
            "car": car.name, "section": section, "axle": axle,
            "name": (sec.get("NAME") or "").strip(), "short_name": (sec.get("SHORT_NAME") or "").strip(),
            "width": width, "radius": radius, "rim_radius": rim,
            "size_code": f"{mk[0]}/{mk[1]}R{mk[2]}" if mk else None,
            "features": feats, "params": asdict(p), "source": p.source,
        })
    return out


def _num(v) -> Optional[float]:
    try:
        return float(str(v).split(";")[0])
    except (TypeError, ValueError):
        return None


def scan_library(cars_dir: Path, cache: Optional[Path] = None,
                 progress: Optional[Callable[[int, int, str], None]] = None,
                 should_stop: Optional[Callable[[], bool]] = None) -> list:
    """Describe every tyre section of every car under `cars_dir`.

    A car whose `tyres.ini` has not changed since the last scan is not read again (`cache`
    is a JSON file).  A car that cannot be read (encrypted `data.acd`) is skipped.
    """
    cars_dir = Path(cars_dir)
    db: dict = {}
    if cache and Path(cache).is_file():
        try:
            blob = json.loads(Path(cache).read_text(encoding="utf-8"))
            if blob.get("version") == CACHE_VERSION:
                db = blob.get("cars", {})
        except (OSError, ValueError):
            db = {}
    cars = sorted(p for p in cars_dir.iterdir() if p.is_dir())
    fresh: dict = {}
    for i, car in enumerate(cars):
        if should_stop and should_stop():
            break
        if progress:
            progress(i, len(cars), car.name)
        stamp = _stamp(car)
        old = db.get(car.name)
        if old and old.get("stamp") == stamp:
            fresh[car.name] = old
            continue
        try:
            entries = _entries_of_car(car)
        except Exception:                                      # noqa: BLE001
            entries = []
        fresh[car.name] = {"stamp": stamp, "entries": entries}
    if cache and not (should_stop and should_stop()):
        try:
            Path(cache).parent.mkdir(parents=True, exist_ok=True)
            Path(cache).write_text(json.dumps({"version": CACHE_VERSION, "cars": fresh}),
                                   encoding="utf-8")
        except OSError:
            pass
    return [e for c in fresh.values() for e in c["entries"]]


def _stamp(car: Path) -> str:
    parts = []
    for rel in ("data/tyres.ini", "data.acd"):
        p = car / rel
        try:
            st = p.stat()
            parts.append(f"{rel}:{st.st_size}:{int(st.st_mtime)}")
        except OSError:
            pass
    return "|".join(parts)


# ── ranking ───────────────────────────────────────────────────────────────────────

def behaviour_distance(a: dict, b: dict) -> float:
    """Weighted mean of the relative differences of the features (0 = identical)."""
    tot = w_sum = 0.0
    for k, w in FEATURE_WEIGHTS.items():
        x, y = a.get(k), b.get(k)
        if x is None or y is None:
            continue
        den = max(0.5 * (abs(x) + abs(y)), 1e-6)
        tot += w * min(abs(x - y) / den, 2.0)
        w_sum += w
    return tot / w_sum if w_sum else 1.0


def rank(target_features: dict, target_size: Optional[dict], library: Iterable[dict],
         axle: Optional[str] = None, size_weight: float = 0.25, top: int = 20) -> list:
    """The `top` library tyres most like the target, best first.

    `size_weight` (0..1) is the share of the size in the score; 0 ranks by behaviour only.
    Each result carries `score` (0..100), `behaviour` and `size` (0..100 each) and the entry.
    """
    out = []
    for e in library:
        if axle and e["axle"] != axle:
            continue
        beh = max(0.0, 1.0 - behaviour_distance(target_features, e["features"]) * 2.0)
        sd = size_distance(target_size, e) if target_size else None
        size = None if sd is None else max(0.0, 1.0 - sd)
        w = size_weight if size is not None else 0.0
        score = (1.0 - w) * beh + w * (size if size is not None else 0.0)
        out.append({"score": round(100 * score, 1), "behaviour": round(100 * beh, 1),
                    "size": None if size is None else round(100 * size, 1), "entry": e})
    out.sort(key=lambda r: -r["score"])
    return out[:top]


def library_target(library: Iterable[dict], car: str, section: str) -> Optional[tuple]:
    """(features, size) of one tyre of the library, to look for tyres like it."""
    for e in library:
        if e["car"] == car and e["section"] == section:
            return e["features"], {k: e.get(k) for k in ("width", "radius", "rim_radius", "size_code")}
    return None


# ── taking a tyre from a car ─────────────────────────────────────────────────────

def tyres_ini_sections(cars_dir: Path, car: str, section: str) -> str:
    """The text of one tyre section (and its thermal section) of a car's `tyres.ini`, as the
    car has it, ready to paste into another car's file."""
    from acd_reader import open_car
    loaded, _, _ = open_car(Path(cars_dir) / car)
    text = next((v for k, v in loaded.items() if k.lower() == "tyres.ini"), "")
    suffix = section.split("_", 1)[1] if "_" in section else ""
    axle = "FRONT" if section.startswith("FRONT") else "REAR"
    wanted = {section, f"THERMAL_{axle}" + (f"_{suffix}" if suffix else "")}
    chunks, take = [], False
    for line in text.splitlines():
        m = re.match(r"\s*\[(.+?)\]", line)
        if m:
            take = m.group(1).strip().upper() in wanted
        if take:
            chunks.append(line.rstrip())
    return "\n".join(chunks).strip() + "\n"
