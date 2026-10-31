"""MF 5.2 (SVJ) -> AC tyre parameter inverse.

AC's tyre model uses parametric forms (``LOAD_CURVE``, ``DX0/DX1``,
``DY0/DY1``, ``LS_EXPX/LS_EXPY``, ``FLEX``, ``CAMBER_GAIN``, etc.); the
AC_to_SVJ forward converter fits MF 5.2 coefficients from those by
sweeping the AC model over a virtual bench. Going the other way is
underdetermined.

Strategy (per ``02_PROJECT_SPEC.md §9``):

1. **Fast path -- ``_ac_native``.** AC_to_SVJ embeds the original AC
   parameters in the SVJ under
   ``tires.sets[<key>].models.pacejka_mf52._source_params`` (an
   ``AC_<axle>`` block carrying ``FZ0``, ``DY0``, ``DY1``, ``LS_EXPY``,
   ``DX0``, ``DX1``, ``LS_EXPX``, ``K_a``, ``K_k``, ``FLEX``,
   ``CAMBER_GAIN``, ``KINETIC_RATIO``, ``PRESSURE_*``). When present we
   return those verbatim -- the round-trip is exact.

2. **Slow path -- ``scipy.optimize.least_squares``.** If
   ``_source_params`` is absent we fit AC parameters that reproduce the
   MF 5.2 pure-slip Fx/Fy at the standard load points
   ``Fz in {1500, 3000, 4500, 6000} N`` for ``alpha`` /  ``kappa`` ramps
   and zero camber. The forward model is imported from
   ``AC_to_SVJ.tire_lab`` (single source of truth -- no duplication).
   The fit is logged to ``conversion_log.txt`` as a WARNING (caller
   passes a log path); the caller MUST tell the user the result is
   approximate.

The public API is a single :func:`fit_ac_params` that returns a
``dict`` with the canonical AC tyres.ini keys. The dict is consumed
verbatim by :mod:`ac_writers.tyres_ini`.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Mapping

# AC tyres.ini keys we emit per compound section. Keep this list narrow
# -- it's everything the round-trip path needs to be byte-faithful
# against `AC_to_SVJ/test_car/data/tyres.ini`. Additional fields
# (CAMBER_GAIN, KINETIC_RATIO, PRESSURE_*) are appended when present in
# the SVJ source.
CANONICAL_AC_KEYS: tuple[str, ...] = (
    "WIDTH",
    "RADIUS",
    "RIM_RADIUS",
    "FZ0",
    "DY0",
    "DY1",
    "LS_EXPY",
    "DX0",
    "DX1",
    "LS_EXPX",
    "FLEX",
    "CAMBER",
    "XMU",
    "PRESSURE_STATIC",
)

# Standard load points for the slow-path fit, in newtons.
_STANDARD_LOADS_N: tuple[float, ...] = (1500.0, 3000.0, 4500.0, 6000.0)


class PacejkaInverseError(Exception):
    """Raised by :func:`fit_ac_params` when neither path can produce a
    usable parameter set (e.g. the MF block is completely missing)."""


def fit_ac_params(
    compound: Mapping[str, Any],
    *,
    log_path: Path | None = None,
    side: str = "front",
    wheel_load: float | None = None,
) -> dict[str, Any]:
    """Return the AC parameter dict for one tyre compound.

    Parameters
    ----------
    compound
        One ``tires.sets[<key>]`` entry from an SVJ document.
    log_path
        Optional ``conversion_log.txt`` path; when fitter fallback fires
        a WARNING line is appended.
    side
        ``"front"`` or ``"rear"`` -- only used to label the warning.
    """
    fast = _try_source_params(compound)
    if fast is not None:
        return fast
    # An SVJ from another simulator: the standard's Pacejka block (MF 5.2 / 6.2).
    std = _fit_from_standard(compound, log_path=log_path, side=side, wheel_load=wheel_load)
    if std is not None:
        return std
    # Slow path. Only attempt if MF 5.2 coefficients are present.
    mf = _get_mf52(compound)
    if mf is None:
        # no force model to read (a tyre given only by its size, or by another model): a
        # generic road tyre of the right size, said so in the log
        _log_warning(log_path, f"tyre [{side}] has no Pacejka block: generic road tyre parameters "
                               "of its size (pick tyres with the Tyre Match tab to do better).")
        return _conservative_defaults(compound, {})
    fitted = _fit_least_squares(compound, mf)
    _log_warning(
        log_path,
        f"tyre [{side}] reconstructed via Pacejka inverse fitter -- "
        "expect grip drift +/- 5%.",
    )
    return fitted


def _geometry_and_pressure(compound: Mapping[str, Any], out: dict[str, Any]) -> None:
    """WIDTH, RADIUS, RIM_RADIUS, PRESSURE_STATIC, RATE and DAMP from the tyre set."""
    dims = compound.get("dimensions") or {}
    rim = compound.get("rim") or {}
    if isinstance(dims, Mapping):
        out["WIDTH"] = dims.get("section_width")
        if dims.get("overall_diameter"):
            out["RADIUS"] = float(dims["overall_diameter"]) / 2.0
        else:
            out["RADIUS"] = dims.get("carcass_radius") or dims.get("rolling_radius")
        if not (isinstance(rim, Mapping) and rim.get("diameter")) and dims.get("rim_radius"):
            out["RIM_RADIUS"] = dims["rim_radius"]
    if isinstance(rim, Mapping) and rim.get("diameter") is not None:
        out["RIM_RADIUS"] = float(rim["diameter"]) / 2.0
    pressure = compound.get("pressure")
    if pressure is not None:
        from units import pa_to_psi

        out["PRESSURE_STATIC"] = int(round(pa_to_psi(float(pressure))))
    stiff = compound.get("stiffness") or {}
    cons = compound.get("construction") or {}
    vert = (stiff.get("vertical") if isinstance(stiff, Mapping) else None)         or (cons.get("vertical_stiffness") if isinstance(cons, Mapping) else None)
    if vert:
        out["RATE"] = float(vert)
    damp = cons.get("vertical_damping") if isinstance(cons, Mapping) else None
    if damp:
        out["DAMP"] = float(damp)


def _fit_from_standard(compound: Mapping[str, Any], *, log_path: Path | None,
                       side: str, wheel_load: float | None = None) -> dict[str, Any] | None:
    """AC's tyre parameters from the standard's ``pacejka`` block (pure slip), by fitting AC's
    force model (as tire_lab implements it) to the Pacejka curves.

    Grip and load sensitivity come from DY0 / DY1 / LS_EXPY (lateral) and DX0 / DX1 / LS_EXPX
    (longitudinal), the stiffnesses from FRICTION_LIMIT_ANGLE (the slip angle of the lateral
    peak) and CX_MULT, the post-peak level from XMU.  Returns None when the set has no
    Pacejka block to read.
    """
    try:
        import numpy as np
        from scipy.optimize import least_squares
        from tire_lab import ACTyreParams, ac_fx, ac_fy
        from tyre_match import mf_pure_fx, mf_pure_fy
    except Exception:                                             # noqa: BLE001
        return None
    blk = compound.get("pacejka") if isinstance(compound.get("pacejka"), Mapping) else _mf52_block(compound)
    if not isinstance(blk, Mapping):
        return None
    lat, lon = blk.get("lateral"), blk.get("longitudinal")
    if not isinstance(lat, Mapping) or not isinstance(lon, Mapping):
        return None
    ref = compound.get("reference") or {}
    fz_ref = float(ref.get("load") or 4000.0)               # the load the Pacejka block is built at
    # AC's FZ0 is the load the car's tyre works at; with the weight of the car given, fit around
    # that, not around a reference load of the other simulator (17,100 N for a Skip Barber)
    fz0 = float(wheel_load) if wheel_load and wheel_load > 500 else fz_ref
    loads = [fz0 * f for f in (0.4, 0.7, 1.0, 1.5, 2.2)]
    alpha = np.radians(np.linspace(0.0, 18.0, 37))
    kappa = np.linspace(0.0, 0.35, 36)
    tgt_y = [np.abs(mf_pure_fy(alpha, fz, 0.0, lat, fz_ref)) for fz in loads]
    tgt_x = [np.abs(mf_pure_fx(kappa, fz, 0.0, lon, fz_ref)) for fz in loads]

    def params(x) -> ACTyreParams:
        return ACTyreParams(name="fit", axle=side, FZ0=fz0, DY0=x[0], DY1=x[1], LS_EXPY=x[2],
                            DX0=x[3], DX1=x[4], LS_EXPX=x[5], K_a=x[6], K_k=x[7], FLEX=0.0008,
                            CAMBER_GAIN=0.0, KINETIC_RATIO=x[8], PRESSURE_REF_PSI=29.0,
                            PRESSURE_NOW_PSI=29.0, PRESSURE_GAIN=0.0, source="fit")

    def residuals(x):
        p = params(x)
        r = []
        for fz, ty, tx in zip(loads, tgt_y, tgt_x):
            r.append((np.abs(ac_fy(alpha, fz, 0.0, p)) - ty) / fz)
            r.append((np.abs(ac_fx(kappa, fz, 0.0, p)) - tx) / fz)
        return np.concatenate(r)

    mu_y0 = float(tgt_y[2].max() / fz0)
    mu_x0 = float(tgt_x[2].max() / fz0)
    x0 = np.array([mu_y0, -0.05, 0.9, mu_x0, -0.05, 0.9, 16.0, 18.0, 0.9])
    lo = [0.3, -0.5, 0.4, 0.3, -0.5, 0.4, 4.0, 4.0, 0.5]
    hi = [3.5, 0.5, 1.4, 3.5, 0.5, 1.4, 60.0, 80.0, 1.0]
    fit = least_squares(residuals, x0, bounds=(lo, hi), max_nfev=300)
    x = fit.x
    rmse = float(np.sqrt(np.mean(fit.fun ** 2)))
    _log_warning(log_path, f"tyre [{side}] fitted to the Pacejka block (rms {rmse:.3f} of the load); "
                           "AC's tyre model is simpler than Pacejka, expect grip drift.")
    out: dict[str, Any] = {
        "FZ0": round(fz0), "DY0": round(float(x[0]), 4), "DY1": round(float(x[1]), 4),
        "LS_EXPY": round(float(x[2]), 4), "DX0": round(float(x[3]), 4),
        "DX1": round(float(x[4]), 4), "LS_EXPX": round(float(x[5]), 4), "FLEX": 0.0008,
        "XMU": round(float(x[8]), 3),
        # the slip angle of the lateral peak at FZ0, and the longitudinal stiffness scale
        "FRICTION_LIMIT_ANGLE": round(math.degrees(float(x[0]) * (1.0 + 0.0008 * fz0) / float(x[6])), 3),
    }
    _geometry_and_pressure(compound, out)
    return {k: v for k, v in out.items() if v is not None}


def _mf52_block(compound: Mapping[str, Any]) -> Mapping[str, Any] | None:
    """The MF 5.2 block of a tyre set: ``pacejka_mf52`` beside ``pacejka`` (AC_to_SVJ
    0.9.x), or under ``models`` (its first layout)."""
    mf = compound.get("pacejka_mf52")
    if isinstance(mf, Mapping):
        return mf
    models = compound.get("models")
    if isinstance(models, Mapping) and isinstance(models.get("pacejka_mf52"), Mapping):
        return models["pacejka_mf52"]
    return None


def _try_source_params(compound: Mapping[str, Any]) -> dict[str, Any] | None:
    """Fast path: read ``models.pacejka_mf52._source_params`` verbatim."""
    mf = _mf52_block(compound)
    if not isinstance(mf, Mapping):
        return None
    src = mf.get("_source_params")
    if not isinstance(src, Mapping):
        return None
    out: dict[str, Any] = {}
    # Geometry
    dims = compound.get("dimensions") or {}
    rim = compound.get("rim") or {}
    if isinstance(dims, Mapping):
        out["WIDTH"] = dims.get("section_width")
        radius = dims.get("rolling_radius") or dims.get("loaded_radius")
        if radius is None and "overall_diameter" in dims:
            radius = float(dims["overall_diameter"]) / 2.0
        out["RADIUS"] = radius
    if isinstance(rim, Mapping):
        rd = rim.get("diameter")
        if rd is not None:
            out["RIM_RADIUS"] = float(rd) / 2.0
    # AC params
    for ac_key, svj_key in (
        ("FZ0", "FZ0"),
        ("DY0", "DY0"),
        ("DY1", "DY1"),
        ("LS_EXPY", "LS_EXPY"),
        ("DX0", "DX0"),
        ("DX1", "DX1"),
        ("LS_EXPX", "LS_EXPX"),
        ("FLEX", "FLEX"),
    ):
        if svj_key in src:
            out[ac_key] = src[svj_key]
    # CAMBER / XMU are derived from CAMBER_GAIN / KINETIC_RATIO when present.
    if "CAMBER_GAIN" in src:
        out["CAMBER"] = src["CAMBER_GAIN"]
    if "KINETIC_RATIO" in src:
        out["XMU"] = src["KINETIC_RATIO"]
    # Static pressure: SVJ stores Pa; AC stores psi.
    pressure = compound.get("pressure")
    if pressure is not None:
        # Pa -> psi via units.py to keep the grep guard rail happy.
        from units import pa_to_psi

        out["PRESSURE_STATIC"] = int(round(pa_to_psi(float(pressure))))
    elif "PRESSURE_REF_PSI" in src:
        out["PRESSURE_STATIC"] = int(round(float(src["PRESSURE_REF_PSI"])))
    return {k: v for k, v in out.items() if v is not None}


def _get_mf52(compound: Mapping[str, Any]) -> Mapping[str, Any] | None:
    models = compound.get("models")
    if not isinstance(models, Mapping):
        return None
    mf = models.get("pacejka_mf52")
    if not isinstance(mf, Mapping):
        return None
    if "pure_slip_lateral" not in mf or "pure_slip_longitudinal" not in mf:
        return None
    return mf


def _fit_least_squares(
    compound: Mapping[str, Any], mf: Mapping[str, Any]
) -> dict[str, Any]:
    """Least-squares fit of AC parameters against the MF 5.2 forward model.

    We import the AC forward model (:mod:`tire_lab`) and the MF 5.2
    formulas (also in ``tire_lab``) so there is *one* source of truth.
    The fit reuses ``scipy.optimize.least_squares``.
    """
    try:
        import numpy as np  # type: ignore
        from scipy.optimize import least_squares  # type: ignore
        from tire_lab import (  # type: ignore  # noqa: F401  (loaded lazily)
            ACTyreParams,
            ac_fx,
            ac_fy,
            mf_lat,
            mf_long,
        )
    except Exception:  # pragma: no cover - scipy/tire_lab missing
        # If the scientific stack isn't available, fall back to a
        # passthrough using the MF reference load and conservative
        # AC defaults. This keeps the converter running even on a
        # minimal environment.
        return _conservative_defaults(compound, mf)

    Fz0 = float(mf.get("reference_load_N", 3000.0))
    lat = mf.get("pure_slip_lateral", {})
    lng = mf.get("pure_slip_longitudinal", {})

    # Build target curves from MF 5.2.
    alphas = np.linspace(-0.30, 0.30, 41)
    kappas = np.linspace(-0.20, 0.20, 41)
    targets_fy: list[tuple[float, float, float]] = []
    targets_fx: list[tuple[float, float, float]] = []
    for fz in _STANDARD_LOADS_N:
        for a in alphas:
            try:
                fy = float(mf_lat(float(a), float(fz), Fz0, lat))
                targets_fy.append((float(a), float(fz), fy))
            except Exception:
                pass
        for k in kappas:
            try:
                fx = float(mf_long(float(k), float(fz), Fz0, lng))
                targets_fx.append((float(k), float(fz), fx))
            except Exception:
                pass

    def make_params(x: np.ndarray) -> ACTyreParams:
        # Order: DY0, DY1, LS_EXPY, DX0, DX1, LS_EXPX, K_a, K_k, FLEX
        return ACTyreParams(
            name="fit",
            axle="fit",
            FZ0=Fz0,
            DY0=x[0],
            DY1=x[1],
            LS_EXPY=x[2],
            DX0=x[3],
            DX1=x[4],
            LS_EXPX=x[5],
            K_a=x[6],
            K_k=x[7],
            FLEX=x[8],
            CAMBER_GAIN=0.0,
            KINETIC_RATIO=0.95,
            PRESSURE_REF_PSI=29.0,
            PRESSURE_NOW_PSI=29.0,
            PRESSURE_GAIN=0.0,
            source="fit",
        )

    def residuals(x: np.ndarray) -> np.ndarray:
        p = make_params(x)
        out: list[float] = []
        for a, fz, target in targets_fy:
            out.append(ac_fy(a, fz, 0.0, p) - target)
        for k, fz, target in targets_fx:
            out.append(ac_fx(k, fz, 0.0, p) - target)
        return np.asarray(out, dtype=float)

    # Reasonable starting guess from typical road tyres.
    x0 = np.array([1.4, -0.1, 0.9, 1.4, -0.1, 0.9, 22.0, 18.0, 5e-4])
    bounds = (
        [0.5, -0.4, 0.5, 0.5, -0.4, 0.5, 5.0, 5.0, 1e-5],
        [3.0, 0.4, 1.5, 3.0, 0.4, 1.5, 100.0, 100.0, 1e-2],
    )
    result = least_squares(residuals, x0, bounds=bounds, max_nfev=400)
    x = result.x

    out: dict[str, Any] = {
        "FZ0": Fz0,
        "DY0": float(x[0]),
        "DY1": float(x[1]),
        "LS_EXPY": float(x[2]),
        "DX0": float(x[3]),
        "DX1": float(x[4]),
        "LS_EXPX": float(x[5]),
        "FLEX": float(x[8]),
        "CAMBER": 1.0,
        "XMU": 0.95,
    }
    # Geometry from compound dims.
    dims = compound.get("dimensions") or {}
    rim = compound.get("rim") or {}
    if isinstance(dims, Mapping):
        out["WIDTH"] = dims.get("section_width")
        radius = dims.get("rolling_radius") or dims.get("loaded_radius")
        if radius is None and "overall_diameter" in dims:
            radius = float(dims["overall_diameter"]) / 2.0
        out["RADIUS"] = radius
    if isinstance(rim, Mapping) and rim.get("diameter") is not None:
        out["RIM_RADIUS"] = float(rim["diameter"]) / 2.0
    pressure = compound.get("pressure")
    if pressure is not None:
        from units import pa_to_psi

        out["PRESSURE_STATIC"] = int(round(pa_to_psi(float(pressure))))

    return {k: v for k, v in out.items() if v is not None}


def _conservative_defaults(
    compound: Mapping[str, Any], mf: Mapping[str, Any]
) -> dict[str, Any]:
    """Minimal degenerate fit when scipy/tire_lab is unavailable."""
    out: dict[str, Any] = {
        "FZ0": float(mf.get("reference_load_N", 3000.0)),
        "DY0": 1.5,
        "DY1": -0.1,
        "LS_EXPY": 0.85,
        "DX0": 1.5,
        "DX1": -0.1,
        "LS_EXPX": 0.90,
        "FLEX": 5e-4,
        "CAMBER": 1.0,
        "XMU": 0.95,
    }
    dims = compound.get("dimensions") or {}
    if isinstance(dims, Mapping):
        out["WIDTH"] = dims.get("section_width", 0.225)
        out["RADIUS"] = (
            dims.get("rolling_radius")
            or dims.get("loaded_radius")
            or (float(dims["overall_diameter"]) / 2.0 if dims.get("overall_diameter") else 0.32)
        )
    rim = compound.get("rim") or {}
    if isinstance(rim, Mapping) and rim.get("diameter") is not None:
        out["RIM_RADIUS"] = float(rim["diameter"]) / 2.0
    pressure = compound.get("pressure")
    if pressure is not None:
        from units import pa_to_psi

        out["PRESSURE_STATIC"] = int(round(pa_to_psi(float(pressure))))
    return out


def _log_warning(log_path: Path | None, message: str) -> None:
    if log_path is None:
        return
    try:
        with Path(log_path).open("a", encoding="utf-8") as f:
            f.write(f"WARNING [pacejka]: {message}\n")
    except OSError:
        pass


__all__ = [
    "CANONICAL_AC_KEYS",
    "PacejkaInverseError",
    "fit_ac_params",
]
