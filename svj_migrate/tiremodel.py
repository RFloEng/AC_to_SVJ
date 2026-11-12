# Vendored from RFloEng/SVJ-standard-vehicle-json, tools/tire_model.py (commit 8564da9, Apache-2.0).
# Unchanged except for this header.
#!/usr/bin/env python3
"""
SVJ tyre force evaluator: the Magic Formula of a tyre set's `pacejka` block (spec §11.6, §11.13).

Evaluates the steady-state tyre forces and moments Fx, Fy, Mz (and My) for a vertical load, slip ratio, slip angle,
inclination (camber) angle and inflation pressure. The equations are those of the TNO MF-Tyre 6.1 Magic Formula
(I.J.M. Besselink, A.J.C. Schmeitz, H.B. Pacejka, "An improved Magic Formula/Swift tyre model that can handle
inflation pressure changes", Vehicle System Dynamics 48 (2010), Appendix, equations 31-90), written out in spec §11.13.
MF 5.2 coefficient sets run through the same equations (pressure and the 6.1 camber terms default to zero); the
`model` value picks the one place where they differ: the horizontal shift of Fy with camber (§11.13.3).

Sign convention (spec 11.13, v0.109): the equations are used as written and `pacejka.axis_system` says which signs the
coefficients are in. The results of `forces` are always in ISO 8855 tyre axes: a positive slip angle (velocity to the
right of the wheel heading) gives a positive Fy (to the left) and a negative Mz; the inclination angle gamma is a
rotation about +X (right-hand rule) and a positive gamma gives a negative Fy; kappa > 0 gives Fx > 0; My is
negative for a wheel rolling forward. A set declared `SAE_J670` (the Pacejka/TYDEX signs of earlier versions) is
converted at the interface: Fy, Mz and what follows their sign change. `to_iso_set` rewrites such a set to ISO
coefficients exactly; `mirror_camber` flips the camber coefficients of a set whose camber thrust points the wrong way.
Units are SI: N, m, rad, Pa, m/s.

Not evaluated: overturning moment Mx (qsx coefficients), turn slip, transient behaviour (relaxation), thermal and wear
effects, and the TMeasy and brush models.

Functions:
    params(tire_set, doc=None)      -> Params from a tyre set
    forces(p, Fz, kappa, alpha, gamma, pressure, Vx, combined=True) -> dict of forces, moments and the intermediates
    characteristics(p, Fz, ...)     -> cornering stiffness, slip stiffness, peak friction, peak slip, pneumatic trail
    sweep_alpha / sweep_kappa       -> curves

Usage:
    python tools/tire_model.py file.svj.json                         # every tyre set: characteristics at its reference load
    python tools/tire_model.py file.svj.json --set front_205_45r17 --fz 4000 --sweep alpha --range 0.3 --steps 13
    python tools/tire_model.py file.svj.json --set front_205_45r17 --sweep kappa --alpha 0.05
"""

import json
import math
import sys

_EPS = 1e-9
_PRESSURE_REF_DEFAULT = None

SCALING_KEYS = ("LFZO", "LCX", "LMUX", "LEX", "LKX", "LHX", "LVX", "LXAL", "LCY", "LMUY", "LEY", "LKY", "LHY", "LVY",
                "LKYC", "LYKA", "LVYKA", "LTR", "LRES", "LKZC", "LS", "LMY")


# ── parameters ─────────────────────────────────────────────────────────────────
class Params:
    """Coefficients and reference values of one tyre set."""

    def __init__(self, model, lat, lon, alg, comb, roll, scaling, Fz0, p0, R0, name="", V0=16.7, axis="ISO_8855"):
        self.model, self.name, self.axis = model, name, axis
        self.lat, self.lon, self.alg, self.comb, self.roll = lat, lon, alg, comb, roll
        self.lam = {k: 1.0 for k in SCALING_KEYS}
        self.lam.update({k: float(v) for k, v in (scaling or {}).items() if k in SCALING_KEYS and isinstance(v, (int, float))})
        self.Fz0, self.p0, self.R0, self.V0 = Fz0, p0, R0, V0

    def has_combined(self):
        return any(isinstance(self.comb.get(k), (int, float)) and self.comb[k] != 0 for k in ("rCx1", "rCy1"))


def _group(g):
    return {k: float(v) for k, v in (g or {}).items() if isinstance(v, (int, float)) and not isinstance(v, bool)}


def unloaded_radius(tire_set):
    """R0: half the unloaded overall diameter; else rim radius plus section height."""
    dim = tire_set.get("dimensions") or {}
    if isinstance(dim.get("overall_diameter"), (int, float)):
        return dim["overall_diameter"] / 2.0
    rim = (tire_set.get("rim") or {}).get("diameter")
    rim = rim if isinstance(rim, (int, float)) else (dim.get("rim_diameter_code") * 0.0254 if isinstance(dim.get("rim_diameter_code"), (int, float)) else None)
    h = dim.get("section_height")
    if h is None and isinstance(dim.get("section_width"), (int, float)) and isinstance(dim.get("aspect_ratio"), (int, float)):
        h = dim["section_width"] * dim["aspect_ratio"]
    if rim is not None and h is not None:
        return rim / 2.0 + h
    return None


def params(tire_set, name=""):
    pj = tire_set.get("pacejka")
    if not isinstance(pj, dict):
        raise ValueError("the tyre set has no `pacejka` block")
    ref = tire_set.get("reference") or {}
    Fz0 = pj.get("reference_load_N", ref.get("load"))
    if not isinstance(Fz0, (int, float)) or Fz0 <= 0:
        raise ValueError("no reference load (pacejka.reference_load_N or reference.load)")
    p0 = ref.get("pressure", tire_set.get("pressure"))
    R0 = unloaded_radius(tire_set)
    axis = pj.get("axis_system", "ISO_8855")
    if axis not in AXIS_SYSTEMS:
        raise ValueError("pacejka.axis_system must be one of %s" % (AXIS_SYSTEMS,))
    return Params(pj.get("model", "MF52"), _group(pj.get("lateral")), _group(pj.get("longitudinal")), _group(pj.get("aligning")),
                  _group(pj.get("combined")), _group(pj.get("rolling_resistance")), pj.get("scaling"), float(Fz0),
                  float(p0) if isinstance(p0, (int, float)) and p0 > 0 else None, R0, name,
                  float(ref["speed"]) if isinstance(ref.get("speed"), (int, float)) and ref["speed"] > 0 else 16.7, axis)


AXIS_SYSTEMS = ("ISO_8855", "SAE_J670")
# Quantities that change sign between the SAE/Pacejka tyre axes and ISO 8855 for the same physical situation.
_AXIS_FLIPPED = ("Fy", "Fy0", "Mz", "Mzr", "SVyk", "Kya", "s")
# Coefficients that change sign to rewrite a set from SAE signs to ISO signs (and back): Fy and Mz negate exactly,
# Fx and My are unchanged. The horizontal shifts (pHy1, pHy2, pHy3, qHz*) stay: they are in slip angle, not in force.
_SAE_ISO_FLIP = {
    "lateral": ("pKy1", "pVy1", "pVy2", "pVy3", "pVy4", "pKy6", "pKy7"),
    "aligning": ("qDz6", "qDz7", "qDz8", "qDz9", "qDz10", "qDz11"),
    "combined": ("rVy1", "rVy2", "rVy3"),
}
# Coefficients that multiply an odd power of the inclination angle: flipping them is the same as gamma -> -gamma.
_CAMBER_ODD = {
    "lateral": ("pKy6", "pKy7", "pVy3", "pVy4", "pHy3", "pEy4"),
    "aligning": ("qHz3", "qHz4", "qBz4", "qDz3", "qDz8", "qDz9", "qDz10", "qDz11", "qEz5", "ssz3", "ssz4"),
    "combined": ("rVy3",),
}


def _flipped(pj, table):
    out = json.loads(json.dumps(pj))
    for group, keys in table.items():
        g = out.get(group)
        if isinstance(g, dict):
            for k in keys:
                if isinstance(g.get(k), (int, float)) and not isinstance(g.get(k), bool) and g[k] != 0:
                    g[k] = -g[k]
    return out


def to_iso_set(pj):
    """Rewrite a `pacejka` block with SAE/Pacejka signs to ISO 8855 signs (and an ISO one to SAE: the map is its own
    inverse). The result drops `axis_system` when it is ISO and declares SAE_J670 otherwise."""
    was = pj.get("axis_system", "ISO_8855")
    out = _flipped(pj, _SAE_ISO_FLIP)
    now = "ISO_8855" if was == "SAE_J670" else "SAE_J670"
    if now == "ISO_8855":
        out.pop("axis_system", None)
    else:
        out["axis_system"] = now
    return out


def mirror_camber(pj):
    """Flip the camber coefficients of a `pacejka` block, i.e. evaluate it as if gamma changed sign. Used once, to
    correct sets whose camber thrust pointed the wrong way (spec 11.13: gamma is a rotation about +X, right-hand rule)."""
    return _flipped(pj, _CAMBER_ODD)


# ── helpers ────────────────────────────────────────────────────────────────────
def _sgn(x):
    return (x > 0) - (x < 0)


def _atan(x):
    return math.atan(x)


def _mf(B, C, D, E, x, shift=0.0):
    """D sin(C atan(B x - E (B x - atan(B x)))) + shift."""
    bx = B * x
    return D * math.sin(C * _atan(bx - E * (bx - _atan(bx)))) + shift


def _weight(B, C, E, s, sh):
    """Combined-slip weighting: cos(C atan(B s - E (B s - atan(B s)))) / cos(C atan(B sh - E (B sh - atan(B sh))))."""
    num = math.cos(C * _atan(B * s - E * (B * s - _atan(B * s))))
    den = math.cos(C * _atan(B * sh - E * (B * sh - _atan(B * sh))))
    return num / den if abs(den) > _EPS else 1.0


def _div(a, b):
    return a / b if abs(b) > _EPS else 0.0


# ── evaluation ─────────────────────────────────────────────────────────────────
def forces(p, Fz, kappa=0.0, alpha=0.0, gamma=0.0, pressure=None, Vx=None, combined=True):
    """Steady-state forces and moments of one tyre. Returns a dict:
    Fx, Fy (N), Mz, My (N m; My only with rolling-resistance coefficients and Vx), and the intermediates
    Fx0, Fy0 (pure slip), Gxa, Gyk, SVyk, Kya, Kxk, mu_x, mu_y, t (pneumatic trail), Mzr, s."""
    out = {"Fx": 0.0, "Fy": 0.0, "Mz": 0.0, "My": 0.0, "Fx0": 0.0, "Fy0": 0.0, "Gxa": 1.0, "Gyk": 1.0, "SVyk": 0.0,
           "Kya": 0.0, "Kxk": 0.0, "mu_x": 0.0, "mu_y": 0.0, "t": 0.0, "Mzr": 0.0, "s": 0.0}
    if not Fz or Fz <= 0:
        return out
    L, X, Y, Z, R = p.lam, p.lon, p.lat, p.alg, p.comb
    g = lambda d, k: d.get(k, 0.0)
    Fz0p = L["LFZO"] * p.Fz0
    dfz = (Fz - Fz0p) / Fz0p
    pi = pressure if pressure is not None else p.p0
    dpi = (pi - p.p0) / p.p0 if (p.p0 and pi is not None) else 0.0
    R0 = p.R0 or 0.0
    gm = gamma
    mf62 = p.model == "MF62"

    # longitudinal, pure slip (33-42)
    Cx = g(X, "pCx1") * L["LCX"]
    mu_x = (g(X, "pDx1") + g(X, "pDx2") * dfz) * (1 + g(X, "pPx3") * dpi + g(X, "pPx4") * dpi ** 2) * (1 - g(X, "pDx3") * gm ** 2) * L["LMUX"]
    Dx = mu_x * Fz
    SHx = (g(X, "pHx1") + g(X, "pHx2") * dfz) * L["LHX"]
    kx = kappa + SHx
    Ex = min(1.0, (g(X, "pEx1") + g(X, "pEx2") * dfz + g(X, "pEx3") * dfz ** 2) * (1 - g(X, "pEx4") * _sgn(kx)) * L["LEX"])
    Kxk = Fz * (g(X, "pKx1") + g(X, "pKx2") * dfz) * math.exp(g(X, "pKx3") * dfz) * (1 + g(X, "pPx1") * dpi + g(X, "pPx2") * dpi ** 2) * L["LKX"]
    Bx = _div(Kxk, Cx * Dx)
    SVx = Fz * (g(X, "pVx1") + g(X, "pVx2") * dfz) * L["LVX"] * L["LMUX"]
    Fx0 = _mf(Bx, Cx, Dx, Ex, kx, SVx) if Cx else 0.0

    # lateral, pure slip (52-66)
    def lateral(gm):
        Cy = g(Y, "pCy1") * L["LCY"]
        mu_y = (g(Y, "pDy1") + g(Y, "pDy2") * dfz) * (1 + g(Y, "pPy3") * dpi + g(Y, "pPy4") * dpi ** 2) * (1 - g(Y, "pDy3") * gm ** 2) * L["LMUY"]
        Dy = mu_y * Fz
        inner = _div(Fz / Fz0p, (g(Y, "pKy2") + g(Y, "pKy5") * gm ** 2) * (1 + g(Y, "pPy2") * dpi))
        Kya = g(Y, "pKy1") * Fz0p * (1 + g(Y, "pPy1") * dpi) * (1 - g(Y, "pKy3") * abs(gm)) * math.sin(g(Y, "pKy4") * _atan(inner)) * L["LKY"]
        Kyg = Fz * (g(Y, "pKy6") + g(Y, "pKy7") * dfz) * (1 + g(Y, "pPy5") * dpi) * L["LKYC"]
        SVyg = Fz * (g(Y, "pVy3") + g(Y, "pVy4") * dfz) * gm * L["LKYC"] * L["LMUY"]
        SVy = Fz * (g(Y, "pVy1") + g(Y, "pVy2") * dfz) * L["LVY"] * L["LMUY"] + SVyg
        SHy = (g(Y, "pHy1") + g(Y, "pHy2") * dfz) * L["LHY"]
        SHy += _div(Kyg * gm - SVyg, Kya) if mf62 else g(Y, "pHy3") * gm * L["LKYC"]
        ay = alpha + SHy
        Ey = min(1.0, (g(Y, "pEy1") + g(Y, "pEy2") * dfz) * (1 + g(Y, "pEy5") * gm ** 2 - (g(Y, "pEy3") + g(Y, "pEy4") * gm) * _sgn(ay)) * L["LEY"])
        By = _div(Kya, Cy * Dy)
        Fy0 = _mf(By, Cy, Dy, Ey, ay, SVy) if Cy else 0.0
        return dict(Cy=Cy, mu_y=mu_y, Dy=Dy, Kya=Kya, By=By, SVy=SVy, SHy=SHy, Fy0=Fy0)

    ly = lateral(gm)
    Fy0, Kya, mu_y, Dy, By, Cy = ly["Fy0"], ly["Kya"], ly["mu_y"], ly["Dy"], ly["By"], ly["Cy"]

    # combined slip (43-48, 67-74)
    def weights(gm, ly):
        if not combined:
            return 1.0, 1.0, 0.0
        if p.has_combined():
            SHxa = g(R, "rHx1")
            Bxa = (g(R, "rBx1") + g(R, "rBx3") * gm ** 2) * math.cos(_atan(g(R, "rBx2") * kappa)) * L["LXAL"]
            Exa = min(1.0, g(R, "rEx1") + g(R, "rEx2") * dfz)
            Gxa = _weight(Bxa, g(R, "rCx1"), Exa, alpha + SHxa, SHxa)
            SHyk = g(R, "rHy1") + g(R, "rHy2") * dfz
            Byk = (g(R, "rBy1") + g(R, "rBy4") * gm ** 2) * math.cos(_atan(g(R, "rBy2") * (alpha - g(R, "rBy3")))) * L["LYKA"]
            Eyk = min(1.0, g(R, "rEy1") + g(R, "rEy2") * dfz)
            Gyk = _weight(Byk, g(R, "rCy1"), Eyk, kappa + SHyk, SHyk)
            DVyk = ly["mu_y"] * Fz * (g(R, "rVy1") + g(R, "rVy2") * dfz + g(R, "rVy3") * gm) * math.cos(_atan(g(R, "rVy4") * alpha))
            SVyk = DVyk * math.sin(g(R, "rVy5") * _atan(g(R, "rVy6") * kappa)) * L["LVYKA"]
            return Gxa, Gyk, SVyk
        # SVJ fallback without combined coefficients (§11.13.5): friction-circle limiter on the pure forces. The demand
        # (Fx0/Dx, Fy0/Dy) is scaled back onto the unit circle when it lies outside; inside it the two forces do not interact.
        n = math.hypot(Fx0 / Dx if Dx else 0.0, Fy0 / ly["Dy"] if ly["Dy"] else 0.0)
        sc = 1.0 / n if n > 1.0 else 1.0
        return sc, sc, 0.0

    Gxa, Gyk, SVyk = weights(gm, ly)
    Fx = Fx0 * Gxa
    Fy = Fy0 * Gyk + SVyk

    # aligning moment (75-90)
    t = Mzr = s = Mz = 0.0
    if Z and R0:
        ly0 = lateral(0.0) if gm else ly
        Gxa0, Gyk0, _ = weights(0.0, ly0)
        Fy_p = ly0["Fy0"] * Gyk0                                            # side force at zero inclination, without SVyk
        SHt = g(Z, "qHz1") + g(Z, "qHz2") * dfz + (g(Z, "qHz3") + g(Z, "qHz4") * dfz) * gm
        at = alpha + SHt
        SHf = ly["SHy"] + _div(ly["SVy"], Kya)
        ar = alpha + SHf
        if combined and Kya:
            k2 = (Kxk / Kya) ** 2 * kappa ** 2
            at_eq = math.sqrt(at ** 2 + k2) * (_sgn(at) or 1)
            ar_eq = math.sqrt(ar ** 2 + k2) * (_sgn(ar) or 1)
        else:
            at_eq, ar_eq = at, ar
        Bt = (g(Z, "qBz1") + g(Z, "qBz2") * dfz + g(Z, "qBz3") * dfz ** 2) * (1 + g(Z, "qBz4") * gm + g(Z, "qBz5") * abs(gm)) * _div(L["LKY"], L["LMUY"])
        Ct = g(Z, "qCz1")
        Dt = Fz * (R0 / Fz0p) * (g(Z, "qDz1") + g(Z, "qDz2") * dfz) * (1 - g(Z, "pPz1") * dpi) * (1 + g(Z, "qDz3") * gm + g(Z, "qDz4") * gm ** 2) * L["LTR"]
        Et = min(1.0, (g(Z, "qEz1") + g(Z, "qEz2") * dfz + g(Z, "qEz3") * dfz ** 2) *
                 (1 + (g(Z, "qEz4") + g(Z, "qEz5") * gm) * (2 / math.pi) * _atan(Bt * Ct * at)))
        bt = Bt * at_eq
        t = Dt * math.cos(Ct * _atan(bt - Et * (bt - _atan(bt)))) * math.cos(alpha)
        Br = g(Z, "qBz9") * _div(L["LKY"], L["LMUY"]) + g(Z, "qBz10") * By * Cy
        Dr = Fz * R0 * ((g(Z, "qDz6") + g(Z, "qDz7") * dfz) * L["LRES"] +
                        ((g(Z, "qDz8") + g(Z, "qDz9") * dfz) * (1 + g(Z, "pPz2") * dpi) + (g(Z, "qDz10") + g(Z, "qDz11") * dfz) * abs(gm)) * gm * L["LKZC"]) * L["LMUY"]
        Mzr = Dr * math.cos(_atan(Br * ar_eq)) * math.cos(alpha)
        s = R0 * (g(Z, "ssz1") + g(Z, "ssz2") * Fy / Fz0p + (g(Z, "ssz3") + g(Z, "ssz4") * dfz) * gm) * L["LS"]
        Mz = -t * Fy_p + Mzr + s * Fx

    # rolling resistance moment (50)
    My = 0.0
    if p.roll and Vx is not None and R0:
        V0 = p.V0
        q = lambda k: p.roll.get(k, 0.0)
        pr = (pi / p.p0) if (p.p0 and pi) else 1.0
        My = -Fz * R0 * (q("qsy1") + q("qsy2") * Fx / p.Fz0 + q("qsy3") * abs(Vx / V0) + q("qsy4") * (Vx / V0) ** 4 +
                         (q("qsy5") + q("qsy6") * Fz / p.Fz0) * gm ** 2) * (Fz / p.Fz0) ** q("qsy7") * pr ** q("qsy8") * L["LMY"]

    out.update(Fx=Fx, Fy=Fy, Mz=Mz, My=My, Fx0=Fx0, Fy0=Fy0, Gxa=Gxa, Gyk=Gyk, SVyk=SVyk, Kya=Kya, Kxk=Kxk,
               mu_x=mu_x, mu_y=mu_y, t=t, Mzr=Mzr, s=s)
    if p.axis == "SAE_J670":                       # legacy signs: convert to the ISO result at the interface
        for k in _AXIS_FLIPPED:
            out[k] = -out[k] if out[k] else 0.0
    return out


# ── derived quantities ─────────────────────────────────────────────────────────
def sweep_alpha(p, Fz, alphas, kappa=0.0, gamma=0.0, pressure=None, combined=True):
    return [(a, forces(p, Fz, kappa, a, gamma, pressure, None, combined)) for a in alphas]


def sweep_kappa(p, Fz, kappas, alpha=0.0, gamma=0.0, pressure=None, combined=True):
    return [(k, forces(p, Fz, k, alpha, gamma, pressure, None, combined)) for k in kappas]


def characteristics(p, Fz=None, gamma=0.0, pressure=None):
    """Pure-slip characteristics at load Fz (default the reference load): cornering stiffness (slope of Fy at zero slip
    angle, including the shifts), slip stiffness, peak friction coefficients with the slip at the peak, and the pneumatic
    trail at small slip angle."""
    Fz = Fz or p.Fz0
    d = 1e-4
    f = lambda a, k: forces(p, Fz, k, a, gamma, pressure, None, False)
    fy0, fx0 = f(0.0, 0.0)["Fy"], f(0.0, 0.0)["Fx"]
    c_alpha = (f(d, 0.0)["Fy"] - f(-d, 0.0)["Fy"]) / (2 * d)
    c_kappa = (f(0.0, d)["Fx"] - f(0.0, -d)["Fx"]) / (2 * d)
    best_y = max(((abs(f(a / 1000.0, 0.0)["Fy"] - fy0), a / 1000.0) for a in range(1, 700)), default=(0.0, 0.0))
    best_x = max(((abs(f(0.0, k / 1000.0)["Fx"] - fx0), k / 1000.0) for k in range(1, 1000)), default=(0.0, 0.0))
    t = forces(p, Fz, 0.0, 0.02, gamma, pressure)["t"]
    return {"Fz": Fz, "cornering_stiffness": c_alpha, "slip_stiffness": c_kappa,
            "Fy_peak": best_y[0], "alpha_peak": best_y[1], "mu_y_peak": best_y[0] / Fz,
            "Fx_peak": best_x[0], "kappa_peak": best_x[1], "mu_x_peak": best_x[0] / Fz,
            "pneumatic_trail": t, "Fy_at_zero_slip": fy0, "Fx_at_zero_slip": fx0}


# ── CLI ────────────────────────────────────────────────────────────────────────
def main(argv):
    if not argv:
        print(__doc__)
        return 2
    with open(argv[0], "r", encoding="utf-8") as f:
        doc = json.load(f)
    sets = (doc.get("tires") or {}).get("sets") or {}
    opt = lambda k, d=None: (argv[argv.index(k) + 1] if k in argv else d)
    names = [opt("--set")] if opt("--set") else [n for n, s in sets.items() if isinstance(s, dict) and s.get("pacejka")]
    if not names:
        print("no tyre set with a pacejka block")
        return 1
    for n in names:
        p = params(sets[n], n)
        Fz = float(opt("--fz", p.Fz0))
        gamma = float(opt("--gamma", 0.0))
        pr = float(opt("--pressure")) if opt("--pressure") else None
        print(f"{n}: {p.model}, Fz0 {p.Fz0:.0f} N, p0 {p.p0 or 0:.0f} Pa, R0 {p.R0 or 0:.3f} m, combined: {'coefficients' if p.has_combined() else 'friction circle'}")
        sweep = opt("--sweep")
        if sweep:
            lim = float(opt("--range", 0.3 if sweep == "alpha" else 1.0))
            steps = int(opt("--steps", 13))
            xs = [-lim + 2 * lim * i / (steps - 1) for i in range(steps)]
            fixed_a, fixed_k = float(opt("--alpha", 0.0)), float(opt("--kappa", 0.0))
            print(f"  {'alpha' if sweep == 'alpha' else 'kappa':>8}  {'Fx [N]':>9} {'Fy [N]':>9} {'Mz [Nm]':>9}")
            for x in xs:
                r = forces(p, Fz, fixed_k if sweep == "alpha" else x, x if sweep == "alpha" else fixed_a, gamma, pr)
                print(f"  {x:8.4f}  {r['Fx']:9.1f} {r['Fy']:9.1f} {r['Mz']:9.2f}")
        else:
            c = characteristics(p, Fz, gamma, pr)
            print(f"  Fz {Fz:.0f} N: cornering stiffness {c['cornering_stiffness']:.0f} N/rad ({c['cornering_stiffness'] * math.pi / 180:.0f} N/deg), "
                  f"slip stiffness {c['slip_stiffness']:.0f} N")
            print(f"  peak Fy {c['Fy_peak']:.0f} N (mu {c['mu_y_peak']:.2f}) at {math.degrees(c['alpha_peak']):.1f} deg;  "
                  f"peak Fx {c['Fx_peak']:.0f} N (mu {c['mu_x_peak']:.2f}) at kappa {c['kappa_peak']:.3f};  pneumatic trail {c['pneumatic_trail'] * 1000:.1f} mm")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
