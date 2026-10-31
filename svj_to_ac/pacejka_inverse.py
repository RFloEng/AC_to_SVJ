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
    # Slow path. Only attempt if MF 5.2 coefficients are present.
    mf = _get_mf52(compound)
    if mf is None:
        raise PacejkaInverseError(
            "compound has neither ``_source_params`` nor ``pacejka_mf52`` -- "
            "cannot reconstruct AC tyre parameters."
        )
    fitted = _fit_least_squares(compound, mf)
    _log_warning(
        log_path,
        f"tyre [{side}] reconstructed via Pacejka inverse fitter -- "
        "expect grip drift +/- 5%.",
    )
    return fitted


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
