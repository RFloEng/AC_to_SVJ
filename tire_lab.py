"""
tire_lab.py  —  AC tyre model  ⇆  Pacejka MF 5.2 virtual bench
================================================================

Used by:
  - The AC → SVJ converter (batch-fills Pacejka coefficients per axle)
  - The "Tire Lab" GUI tab (renders AC-vs-MF comparison plots live)

Public API
----------
    params   = parse_tyre_section(text)          # tyres.ini text → ACTyreParams
    result   = run_bench(params)                 # runs sweep + fit + plots
    payload  = build_svj_pacejka_block(result)   # → dict for SVJ tires.sets.*

The bench is intentionally self-contained; no dependency on Gradio.
"""

from __future__ import annotations

import io
import json
from dataclasses import dataclass, asdict, field
from pathlib import Path
from typing import Optional

import numpy as np
from scipy.optimize import least_squares
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


# ─────────────────────────────────────────────────────────────────────────────
# 1.  AC tyre parameters
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class ACTyreParams:
    """Subset of tyres.ini fields used by the planar (no transient) tyre model."""
    name: str = "AC_tyre"
    axle: str = "front"

    FZ0: float = 3500.0
    DY0: float = 1.55
    DY1: float = -0.10
    LS_EXPY: float = 0.85
    DX0: float = 1.60
    DX1: float = -0.08
    LS_EXPX: float = 0.90
    K_a: float = 22.0
    K_k: float = 18.0
    FLEX: float = 0.00018
    CAMBER_GAIN: float = 1.10
    KINETIC_RATIO: float = 0.92
    PRESSURE_REF_PSI: float = 27.0
    PRESSURE_NOW_PSI: float = 27.0
    PRESSURE_GAIN: float = 0.005

    source: str = "defaults"     # "defaults" | "tyres.ini" | "mixed"


def _f(raw, default: Optional[float]) -> Optional[float]:
    try:
        return float(raw)
    except (TypeError, ValueError):
        return default


def parse_tyre_section(parsed_ini: dict, section: str = "FRONT",
                       axle: str = "front") -> ACTyreParams:
    """
    Build an ACTyreParams from an already-parsed `tyres.ini` dict.
    Missing fields fall back to defaults. Section name is typically
    'FRONT' or 'REAR'; modern AC uses numbered sections like 'FRONT_1'.
    """
    sec = parsed_ini.get(section) or {}
    # AC also uses numbered suffixes for compound sets: FRONT_1, FRONT_2…
    if not sec:
        for k in parsed_ini.keys():
            if k.startswith(section + "_"):
                sec = parsed_ini[k]
                break

    d = ACTyreParams(axle=axle, name=f"AC_{axle}")
    matched = 0

    mapping = {
        "FZ0":               ("FZ0",               d.FZ0),
        "DY0":               ("DY0",               d.DY0),
        "DY1":               ("DY1",               d.DY1),
        "LS_EXPY":           ("LS_EXPY",           d.LS_EXPY),
        "DX0":               ("DX0",               d.DX0),
        "DX1":               ("DX1",               d.DX1),
        "LS_EXPX":           ("LS_EXPX",           d.LS_EXPX),
        "FLEX":              ("FLEX",              d.FLEX),
        "CAMBER":            ("CAMBER_GAIN",       d.CAMBER_GAIN),
        "CAMBER_GAIN":       ("CAMBER_GAIN",       d.CAMBER_GAIN),
        "PRESSURE_STATIC":   ("PRESSURE_REF_PSI",  d.PRESSURE_REF_PSI),
        "PRESSURE_IDEAL":    ("PRESSURE_REF_PSI",  d.PRESSURE_REF_PSI),
    }
    for key, (attr, dflt) in mapping.items():
        v = _f(sec.get(key), None)
        if v is not None:
            setattr(d, attr, v)
            matched += 1

    # Optional: derive kinetic ratio from XMU if present (XMU ≈ μ_kin / μ_peak)
    xmu = _f(sec.get("XMU"), None)
    if xmu is not None and 0.5 < xmu < 1.0:
        d.KINETIC_RATIO = xmu
        matched += 1

    if matched == 0:
        d.source = "defaults"
    elif matched < 4:
        d.source = "mixed"
    else:
        d.source = "tyres.ini"
    return d


# ─────────────────────────────────────────────────────────────────────────────
# 2.  AC forward model (brush with saturation)
# ─────────────────────────────────────────────────────────────────────────────

def _mu_lat(Fz, p: ACTyreParams):
    fz_n = Fz / p.FZ0
    pr = 1.0 - p.PRESSURE_GAIN * (p.PRESSURE_NOW_PSI - p.PRESSURE_REF_PSI)
    return (p.DY0 + p.DY1 * (fz_n - 1.0)) * np.power(fz_n, p.LS_EXPY - 1.0) * pr


def _mu_long(Fz, p: ACTyreParams):
    fz_n = Fz / p.FZ0
    pr = 1.0 - p.PRESSURE_GAIN * (p.PRESSURE_NOW_PSI - p.PRESSURE_REF_PSI)
    return (p.DX0 + p.DX1 * (fz_n - 1.0)) * np.power(fz_n, p.LS_EXPX - 1.0) * pr


def _K_lat(Fz, p):  return p.K_a * Fz / (1.0 + p.FLEX * Fz)
def _K_long(Fz, p): return p.K_k * Fz / (1.0 + p.FLEX * Fz)


def _shape(u, kin):
    sigma = 1.6
    bell = np.exp(-((np.maximum(u, 0.0) - 1.0) ** 2) / (sigma ** 2))
    pure = 2.0 * u / (1.0 + u * u)
    return pure * bell + kin * (1.0 - bell) * np.tanh(u * 1.5)


def ac_fy(alpha, Fz, gamma, p: ACTyreParams):
    alpha = np.asarray(alpha, float); Fz = np.asarray(Fz, float); gamma = np.asarray(gamma, float)
    mu = _mu_lat(Fz, p); Cα = _K_lat(Fz, p)
    peak = mu * Fz; α_p = peak / np.maximum(Cα, 1e-3)
    s = alpha / np.maximum(α_p, 1e-4)
    return np.sign(s) * peak * _shape(np.abs(s), p.KINETIC_RATIO) - p.CAMBER_GAIN * gamma * Fz


def ac_fx(kappa, Fz, gamma, p: ACTyreParams):
    kappa = np.asarray(kappa, float); Fz = np.asarray(Fz, float)
    mu = _mu_long(Fz, p); Cκ = _K_long(Fz, p)
    peak = mu * Fz; κ_p = peak / np.maximum(Cκ * Fz, 1e-3)
    s = kappa / np.maximum(κ_p, 1e-4)
    return np.sign(s) * peak * _shape(np.abs(s), p.KINETIC_RATIO)


# ─────────────────────────────────────────────────────────────────────────────
# 3.  Pacejka MF 5.2 — pure slip (implementation matches WP2 bench)
# ─────────────────────────────────────────────────────────────────────────────

def mf_lat(alpha, Fz, Fz0, theta):
    pCy1, pDy1, pDy2, pEy1, pEy2, pKy1, pKy2 = theta
    dfz = (Fz - Fz0) / Fz0
    C = pCy1
    D = (pDy1 + pDy2 * dfz) * Fz
    K = pKy1 * Fz0 * np.sin(2.0 * np.arctan(Fz / (np.maximum(pKy2, 1e-3) * Fz0)))
    B = K / np.maximum(C * D, 1e-3)
    E = np.clip(pEy1 + pEy2 * dfz, -10.0, 0.99)
    Bx = B * alpha
    return D * np.sin(C * np.arctan(Bx - E * (Bx - np.arctan(Bx))))


def mf_long(kappa, Fz, Fz0, theta):
    pCx1, pDx1, pDx2, pEx1, pEx2, pEx3, pEx4, pKx1, pKx2, pKx3 = theta
    dfz = (Fz - Fz0) / Fz0
    C = pCx1
    D = (pDx1 + pDx2 * dfz) * Fz
    K = (pKx1 + pKx2 * dfz) * Fz * np.exp(pKx3 * dfz)
    B = K / np.maximum(C * D, 1e-3)
    E = np.clip((pEx1 + pEx2 * dfz + pEx3 * dfz ** 2) * (1.0 - pEx4 * np.sign(kappa)),
                -10.0, 0.99)
    Bx = B * kappa
    return D * np.sin(C * np.arctan(Bx - E * (Bx - np.arctan(Bx))))


# ─────────────────────────────────────────────────────────────────────────────
# 4.  Sweep + fit
# ─────────────────────────────────────────────────────────────────────────────

FZ_LEVELS = np.array([1500.0, 2500.0, 3500.0, 5000.0, 7000.0])
GAMMA_LEVELS = np.array([0.0, np.deg2rad(-2.0)])


def sweep(p: ACTyreParams) -> dict:
    α = np.deg2rad(np.linspace(-12.0, 12.0, 121))
    κ = np.linspace(-0.25, 0.25, 121)
    lat, lon = [], []
    for Fz in FZ_LEVELS:
        for γ in GAMMA_LEVELS:
            Fy = ac_fy(α, np.full_like(α, Fz), np.full_like(α, γ), p)
            for a, fy in zip(α, Fy):
                lat.append((float(Fz), float(γ), float(a), float(fy)))
        Fx = ac_fx(κ, np.full_like(κ, Fz), np.zeros_like(κ), p)
        for k, fx in zip(κ, Fx):
            lon.append((float(Fz), 0.0, float(k), float(fx)))
    return {"lateral": lat, "longitudinal": lon}


def _fit_lat(data, Fz0):
    rows = [r for r in data["lateral"] if abs(r[1]) < 1e-6]
    Fz = np.array([r[0] for r in rows]); α = np.array([r[2] for r in rows])
    Fy = np.array([r[3] for r in rows])
    θ0 = np.array([1.30, 1.50, -0.10, -0.20, 0.10, 22.0, 1.5])
    lo = np.array([1.05, 0.50, -1.50, -10.0, -2.0, 1.0, 0.05])
    hi = np.array([2.00, 3.50,  1.50,   0.99,  2.0, 80.0, 20.0])
    res = least_squares(lambda t: mf_lat(α, Fz, Fz0, t) - Fy, θ0,
                        bounds=(lo, hi), method="trf", max_nfev=10000)
    yhat = mf_lat(α, Fz, Fz0, res.x)
    ss_res = float(np.sum((Fy - yhat) ** 2))
    ss_tot = float(np.sum((Fy - np.mean(Fy)) ** 2))
    # ss_tot can be 0 when every Fy is the same value (degenerate axle —
    # e.g. lm_chevy_cheyenne_v2's truck tyres trip this). Report r²=NaN
    # rather than crashing the entire conversion.
    r2 = float(1.0 - ss_res / ss_tot) if ss_tot > 0 else float("nan")
    keys = ["pCy1", "pDy1", "pDy2", "pEy1", "pEy2", "pKy1", "pKy2"]
    return {
        "params": dict(zip(keys, [float(v) for v in res.x])),
        "Fz0": float(Fz0),
        "rmse_N": float(np.sqrt(np.mean((Fy - yhat) ** 2))),
        "r2": r2,
        "n_points": int(len(Fy)),
    }


def _fit_long(data, Fz0):
    rows = data["longitudinal"]
    Fz = np.array([r[0] for r in rows]); κ = np.array([r[2] for r in rows])
    Fx = np.array([r[3] for r in rows])
    θ0 = np.array([1.65, 1.55, -0.10, -0.50, 0.10, 0.05, 0.0, 22.0, -0.20, 0.10])
    lo = np.array([1.20, 0.50, -1.50, -10.0, -5.0, -5.0, -1.0, 1.0, -5.0, -2.0])
    hi = np.array([2.20, 3.50,  1.50,   0.99,  5.0,  5.0,  1.0, 80.0, 5.0, 2.0])
    res = least_squares(lambda t: mf_long(κ, Fz, Fz0, t) - Fx, θ0,
                        bounds=(lo, hi), method="trf", max_nfev=10000)
    yhat = mf_long(κ, Fz, Fz0, res.x)
    ss_res = float(np.sum((Fx - yhat) ** 2))
    ss_tot = float(np.sum((Fx - np.mean(Fx)) ** 2))
    # ss_tot can be 0 when every Fx is the same value (degenerate axle).
    r2 = float(1.0 - ss_res / ss_tot) if ss_tot > 0 else float("nan")
    keys = ["pCx1", "pDx1", "pDx2", "pEx1", "pEx2", "pEx3", "pEx4",
            "pKx1", "pKx2", "pKx3"]
    return {
        "params": dict(zip(keys, [float(v) for v in res.x])),
        "Fz0": float(Fz0),
        "rmse_N": float(np.sqrt(np.mean((Fx - yhat) ** 2))),
        "r2": r2,
        "n_points": int(len(Fx)),
    }


# ─────────────────────────────────────────────────────────────────────────────
# 5.  Plots
# ─────────────────────────────────────────────────────────────────────────────

COLORS = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd"]


def plot_lateral_png(data, fit_y, p: ACTyreParams) -> bytes:
    α_plot = np.deg2rad(np.linspace(-12, 12, 240))
    fig, axes = plt.subplots(1, 2, figsize=(13, 5.2))
    ax = axes[0]
    theta = np.array(list(fit_y["params"].values()))
    for i, Fz in enumerate(FZ_LEVELS):
        Fy_ac = ac_fy(α_plot, np.full_like(α_plot, Fz), np.zeros_like(α_plot), p)
        ax.plot(np.rad2deg(α_plot), Fy_ac / 1000.0, color=COLORS[i % 5], lw=2.0,
                label=f"AC  Fz={Fz/1000:.1f} kN")
        Fy_mf = mf_lat(α_plot, np.full_like(α_plot, Fz), fit_y["Fz0"], theta)
        ax.plot(np.rad2deg(α_plot), Fy_mf / 1000.0, color=COLORS[i % 5], lw=1.0, ls="--",
                label=f"MF  Fz={Fz/1000:.1f} kN")
    ax.set_xlabel("Slip angle α [deg]"); ax.set_ylabel("Lateral force Fy [kN]")
    ax.set_title(f"Lateral — AC (solid) vs Pacejka MF 5.2 (dashed)\n"
                 f"R² = {fit_y['r2']:.4f}   RMSE = {fit_y['rmse_N']:.0f} N")
    ax.grid(alpha=0.3); ax.axhline(0, color="k", lw=0.5); ax.axvline(0, color="k", lw=0.5)
    ax.legend(fontsize=7, ncol=2, loc="lower right")

    ax = axes[1]
    rows = [r for r in data["lateral"] if abs(r[1]) < 1e-6]
    Fz_a = np.array([r[0] for r in rows]); α_a = np.array([r[2] for r in rows])
    Fy_a = np.array([r[3] for r in rows])
    err = Fy_a - mf_lat(α_a, Fz_a, fit_y["Fz0"], theta)
    sc = ax.scatter(np.rad2deg(α_a), err, c=Fz_a / 1000.0, cmap="viridis", s=10)
    ax.set_xlabel("Slip angle α [deg]"); ax.set_ylabel("AC − MF residual [N]")
    ax.set_title("Residual cloud (γ = 0)")
    ax.grid(alpha=0.3); ax.axhline(0, color="k", lw=0.5)
    plt.colorbar(sc, ax=ax, label="Fz [kN]")
    fig.tight_layout()
    buf = io.BytesIO(); fig.savefig(buf, format="png", dpi=140); plt.close(fig)
    return buf.getvalue()


def plot_longitudinal_png(data, fit_x, p: ACTyreParams) -> bytes:
    κ_plot = np.linspace(-0.25, 0.25, 240)
    fig, axes = plt.subplots(1, 2, figsize=(13, 5.2))
    ax = axes[0]
    theta = np.array(list(fit_x["params"].values()))
    for i, Fz in enumerate(FZ_LEVELS):
        Fx_ac = ac_fx(κ_plot, np.full_like(κ_plot, Fz), np.zeros_like(κ_plot), p)
        ax.plot(κ_plot, Fx_ac / 1000.0, color=COLORS[i % 5], lw=2.0,
                label=f"AC  Fz={Fz/1000:.1f} kN")
        Fx_mf = mf_long(κ_plot, np.full_like(κ_plot, Fz), fit_x["Fz0"], theta)
        ax.plot(κ_plot, Fx_mf / 1000.0, color=COLORS[i % 5], lw=1.0, ls="--",
                label=f"MF  Fz={Fz/1000:.1f} kN")
    ax.set_xlabel("Slip ratio κ [-]"); ax.set_ylabel("Longitudinal force Fx [kN]")
    ax.set_title(f"Longitudinal — AC (solid) vs Pacejka MF 5.2 (dashed)\n"
                 f"R² = {fit_x['r2']:.4f}   RMSE = {fit_x['rmse_N']:.0f} N")
    ax.grid(alpha=0.3); ax.axhline(0, color="k", lw=0.5); ax.axvline(0, color="k", lw=0.5)
    ax.legend(fontsize=7, ncol=2, loc="lower right")

    ax = axes[1]
    rows = data["longitudinal"]
    Fz_a = np.array([r[0] for r in rows]); κ_a = np.array([r[2] for r in rows])
    Fx_a = np.array([r[3] for r in rows])
    err = Fx_a - mf_long(κ_a, Fz_a, fit_x["Fz0"], theta)
    sc = ax.scatter(κ_a, err, c=Fz_a / 1000.0, cmap="viridis", s=10)
    ax.set_xlabel("Slip ratio κ [-]"); ax.set_ylabel("AC − MF residual [N]")
    ax.set_title("Residual cloud")
    ax.grid(alpha=0.3); ax.axhline(0, color="k", lw=0.5)
    plt.colorbar(sc, ax=ax, label="Fz [kN]")
    fig.tight_layout()
    buf = io.BytesIO(); fig.savefig(buf, format="png", dpi=140); plt.close(fig)
    return buf.getvalue()


def plot_mu_vs_fz_png(fit_y, fit_x, p: ACTyreParams) -> bytes:
    Fz = np.linspace(500.0, 8000.0, 200)
    mu_y_ac = _mu_lat(Fz, p); mu_x_ac = _mu_long(Fz, p)
    th_y = np.array(list(fit_y["params"].values()))
    th_x = np.array(list(fit_x["params"].values()))
    dfz = (Fz - fit_y["Fz0"]) / fit_y["Fz0"]
    mu_y_mf = th_y[1] + th_y[2] * dfz
    mu_x_mf = th_x[1] + th_x[2] * dfz

    fig, ax = plt.subplots(figsize=(8, 5))
    ax.plot(Fz / 1000, mu_y_ac, color=COLORS[0], lw=2.0, label="AC  μy")
    ax.plot(Fz / 1000, mu_y_mf, color=COLORS[0], lw=1.0, ls="--", label="MF  μy")
    ax.plot(Fz / 1000, mu_x_ac, color=COLORS[1], lw=2.0, label="AC  μx")
    ax.plot(Fz / 1000, mu_x_mf, color=COLORS[1], lw=1.0, ls="--", label="MF  μx")
    ax.axvline(p.FZ0 / 1000, color="grey", lw=0.7, ls=":")
    ax.text(p.FZ0 / 1000, ax.get_ylim()[1] * 0.95, " Fz0", color="grey",
            va="top", fontsize=9)
    ax.set_xlabel("Vertical load Fz [kN]"); ax.set_ylabel("Peak friction μ [-]")
    ax.set_title("Load sensitivity — peak μ vs Fz"); ax.grid(alpha=0.3); ax.legend()
    fig.tight_layout()
    buf = io.BytesIO(); fig.savefig(buf, format="png", dpi=140); plt.close(fig)
    return buf.getvalue()


# ─────────────────────────────────────────────────────────────────────────────
# 6.  Public bench entry point
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class BenchResult:
    params:        ACTyreParams
    fit_lateral:   dict
    fit_longitudinal: dict
    lateral_png:   bytes
    longitudinal_png: bytes
    mu_vs_fz_png:  bytes
    n_sweep_points: int


def run_bench(p: ACTyreParams) -> BenchResult:
    """Run the full AC → Pacejka pipeline. All outputs in-memory."""
    data = sweep(p)
    fit_y = _fit_lat(data, p.FZ0)
    fit_x = _fit_long(data, p.FZ0)
    return BenchResult(
        params=p,
        fit_lateral=fit_y,
        fit_longitudinal=fit_x,
        lateral_png=plot_lateral_png(data, fit_y, p),
        longitudinal_png=plot_longitudinal_png(data, fit_x, p),
        mu_vs_fz_png=plot_mu_vs_fz_png(fit_y, fit_x, p),
        n_sweep_points=len(data["lateral"]) + len(data["longitudinal"]),
    )


# ─────────────────────────────────────────────────────────────────────────────
# 7.  SVJ payload builder
# ─────────────────────────────────────────────────────────────────────────────

def build_svj_pacejka_block(result: BenchResult) -> dict:
    """Return a dict suitable for `svj.tires.sets.{name}.models.pacejka_mf52`."""
    p = result.params
    return {
        "version":   "MF 5.2",
        "reference_load_N": p.FZ0,
        "pure_slip_lateral": {
            **result.fit_lateral["params"],
            "_metrics": {
                "r2":       round(result.fit_lateral["r2"], 5),
                "rmse_N":   round(result.fit_lateral["rmse_N"], 1),
                "n_points": result.fit_lateral["n_points"],
            },
        },
        "pure_slip_longitudinal": {
            **result.fit_longitudinal["params"],
            "_metrics": {
                "r2":       round(result.fit_longitudinal["r2"], 5),
                "rmse_N":   round(result.fit_longitudinal["rmse_N"], 1),
                "n_points": result.fit_longitudinal["n_points"],
            },
        },
        "_source":       "virtual_bench (AC forward model → MF fit)",
        "_source_params": asdict(p),
        "_note": "Pure-slip only; combined-slip + thermal + transient terms deferred.",
    }


# ─────────────────────────────────────────────────────────────────────────────
# 8.  CLI for sanity tests
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    import sys
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("./tire_lab_out")
    out.mkdir(parents=True, exist_ok=True)
    res = run_bench(ACTyreParams())
    (out / "longitudinal.png").write_bytes(res.longitudinal_png)
    (out / "mu_vs_fz.png").write_bytes(res.mu_vs_fz_png)
    (out / "pacejka_fit.json").write_text(json.dumps(build_svj_pacejka_block(res), indent=2))
    print(f"lateral R² = {res.fit_lateral['r2']:.4f}  longitudinal R² = {res.fit_longitudinal['r2']:.4f}")
    print(f"outputs in {out}")
